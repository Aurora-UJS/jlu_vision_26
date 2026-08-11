#include "sim_serial.hpp"

#include "basic/time_tools.hpp"
#include "math/angle_tools.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include <nlohmann/json.hpp>

#include "iceoryx_hoofs/cxx/string.hpp"
#include "iox/signal_watcher.hpp"
#include "quill/LogMacros.h"

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdlib>

namespace {
// The simulator steps at 32 ms.  Polling a little faster than that keeps the
// published gimbal pose within one step of the simulation without spinning.
constexpr auto kPollInterval = std::chrono::milliseconds(8);
constexpr auto kLinkTimeout = std::chrono::seconds(5);
}  // namespace

hardware::SimSerial::SimSerial(quill::Logger *logger,
                               const SerialConfigs &configs,
                               const std::string &shm_name)
    : logger_(logger), configs_(configs),
      task_mode_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.task_mode_topic}
                         .description),
      gimbal_info_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.gimbal_info_topic}
                           .description),
      enemy_color_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.enemy_color_topic}
                           .description),
      aim_cmd_sub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.aim_command_topic}
                       .description),
      tf_pub_(logger_) {
  if (!link_.open(shm_name.c_str(), /*create=*/false)) {
    LOG_CRITICAL(logger_,
                 "Cannot open the simulator aim link {}. Start Webots first; "
                 "in a container pass --ipc=host.",
                 shm_name);
    std::exit(EXIT_FAILURE);
  }
  LOG_INFO(logger_, "SimSerial attached to {}", shm_name);

  this->state_thread_ = std::jthread{[&]() {
    LOG_INFO(logger_, "sim state thread start!");
    stateThread();
    LOG_INFO(logger_, "sim state thread stop!");
  }};

  this->aim_cmd_listener_
      .attachEvent(this->aim_cmd_sub_,
                   iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onAimCommandReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach aim command listener");
        std::exit(EXIT_FAILURE);
      });
}

hardware::SimSerial::~SimSerial() = default;

void hardware::SimSerial::stateThread() {
  aim::GimbalState state{};
  uint64_t last_sequence = 0;
  auto last_seen = std::chrono::steady_clock::now();
  bool stall_logged = false;

  while (!iox::hasTerminationRequested()) {
    if (!link_.pollState(state) || state.sequence == last_sequence) {
      if (std::chrono::steady_clock::now() - last_seen > kLinkTimeout &&
          !stall_logged) {
        LOG_WARNING(logger_, "No simulator gimbal state for {} s",
                    kLinkTimeout.count());
        stall_logged = true;
      }
      std::this_thread::sleep_for(kPollInterval);
      continue;
    }
    last_sequence = state.sequence;
    last_seen = std::chrono::steady_clock::now();
    stall_logged = false;

    yaw_.store(state.yaw, std::memory_order_relaxed);
    pitch_.store(state.pitch, std::memory_order_relaxed);

    const auto now = tools::getTimeNowNanoSec();
    const iox::cxx::string<10> frame_id{iox::TruncateToCapacity,
                                        configs_.serial_frame_id.c_str()};

    // The simulator has no referee-system link, so mode and enemy colour come
    // from the config instead of a downlink packet.
    this->task_mode_pub_.loan().and_then(
        [&](iox::popo::Sample<msgs::TaskMode, msgs::Header> &sample) {
          sample.getUserHeader().stamp_ns = now;
          sample.getUserHeader().frame_id = frame_id;
          sample->mode = configs_.sim_task_mode;
          sample.publish();
        });
    this->enemy_color_pub_.loan().and_then(
        [&](iox::popo::Sample<msgs::EnemyColor, msgs::Header> &sample) {
          sample.getUserHeader().stamp_ns = now;
          sample.getUserHeader().frame_id = frame_id;
          sample->color = configs_.sim_enemy_color;
          sample.publish();
        });
    this->gimbal_info_pub_.loan().and_then(
        [&](iox::popo::Sample<msgs::GimbalInfo, msgs::Header> &sample) {
          sample.getUserHeader().stamp_ns = now;
          sample.getUserHeader().frame_id = frame_id;
          sample->bullet_speed = static_cast<float>(state.muzzle_speed);
          sample->pitch = static_cast<float>(state.pitch);
          sample->pitch_vel = static_cast<float>(state.pitch_velocity);
          sample->yaw = static_cast<float>(state.yaw);
          sample->yaw_vel = static_cast<float>(state.yaw_velocity);
          // The simulated gimbal has no roll degree of freedom.
          sample->roll = 0.0F;
          sample->bullet_id = static_cast<unsigned int>(state.frame_id);
          sample.publish();
        });

    // Everything the gimbal loop is doing, in one place: where it is, how fast
    // it is moving, where the aimer asked it to go, and the error between.
    // One datagram per simulator step, not one per field.  Sent separately,
    // each value arrives as its own message and PlotJuggler timestamps them
    // independently -- the commanded angle and the measured one then sit on
    // slightly different time axes, which is exactly the comparison this is
    // for.
    nlohmann::json frame;
    frame["sim_time"] = state.sim_time;
    frame["gimbal/yaw"] = state.yaw;
    frame["gimbal/pitch"] = state.pitch;
    frame["gimbal/yaw_vel"] = state.yaw_velocity;
    frame["gimbal/pitch_vel"] = state.pitch_velocity;
    frame["gimbal/muzzle_speed"] = state.muzzle_speed;
    if (have_target_) {
      frame["aim/target_yaw"] = target_yaw_;
      frame["aim/target_pitch"] = target_pitch_;
      frame["aim/yaw_error"] = target_yaw_ - state.yaw;
      frame["aim/pitch_error"] = target_pitch_ - state.pitch;
      frame["aim/control"] = control_ ? 1.0 : 0.0;
      frame["aim/fire"] = fire_ ? 1.0 : 0.0;
    }
    plotter_.plot(frame);

    // Same sign convention as the real node: roll and pitch are negated going
    // from the firmware's frame into odom.
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    transform.rotate(tools::rpyToQuaterniond(
        Eigen::Vector3d{0.0, -state.pitch, state.yaw}));
    this->tf_pub_.publishTransform(
        transform, configs_.odom_frame_id, configs_.gimbal_fram_id,
        std::chrono::system_clock::now() +
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                std::chrono::duration<double>(configs_.stamp_offset_sec)));
  }
}

void hardware::SimSerial::onAimCommandReceivedCallback(
    iox::popo::Subscriber<msgs::AimCommand, msgs::Header> *subscriber,
    SimSerial *self) {
  while (subscriber->take().and_then(
      [self](const iox::popo::Sample<const msgs::AimCommand,
                                     const msgs::Header> &sample) {
        // Reproduce the firmware's trigger rule rather than forwarding the
        // thresholds: shoot only once the gimbal is inside the window the
        // aiming program asked for.
        const double yaw_error =
            std::abs(sample->target_yaw - self->yaw_.load(std::memory_order_relaxed));
        const double pitch_error =
            std::abs(sample->target_pitch - self->pitch_.load(std::memory_order_relaxed));
        const bool on_target = yaw_error <= sample->fire_thres_yaw &&
                               pitch_error <= sample->fire_thres_pitch;

        aim::AimCommand command{};
        command.target_yaw = sample->target_yaw;
        command.target_pitch = sample->target_pitch;
        command.issued_at = sample.getUserHeader().stamp_ns * 1e-9;
        command.valid = sample->control ? 1U : 0U;
        command.fire = (sample->control && on_target) ? 1U : 0U;
        self->target_yaw_ = sample->target_yaw;
        self->target_pitch_ = sample->target_pitch;
        self->have_target_ = true;
        self->control_ = sample->control;
        self->fire_ = command.fire != 0;
        self->link_.publishCommand(command);
        LOG_DEBUG(self->logger_,
                  "aim -> sim: yaw {:.4f} pitch {:.4f} control {} fire {}",
                  sample->target_yaw, sample->target_pitch, sample->control,
                  command.fire);
      })) {
  }
}
