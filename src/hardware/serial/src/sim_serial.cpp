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
    latest_sim_time_.store(state.sim_time, std::memory_order_relaxed);

    // Everything downstream of the simulator runs on the simulator's clock:
    // SimCamera stamps frames with the sim_time published alongside the
    // pixels, and the tracker interpolates the odom->gimbal transform at the
    // frame stamp.  Stamping this state with wall time would put the only
    // dynamic transform on a different clock from every query.
    const auto sim_stamp = std::chrono::system_clock::time_point{
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(state.sim_time))};
    const auto now = tools::chronoPointToNanoSec(sim_stamp);
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
    frame["score/shots"] = static_cast<double>(state.shots_fired);
    frame["score/hits"] = static_cast<double>(state.target_hits);
    frame["score/hit_rate"] =
        state.shots_fired > 0
            ? static_cast<double>(state.target_hits) / state.shots_fired
            : 0.0;
    if (have_target_) {
      // The aimer's yaw lives on [-pi, pi) while the joint unwinds across
      // turns.  Publish the command unwrapped into the joint's neighbourhood
      // so the two curves overlay in PlotJuggler; the circle-folded error is
      // the ground truth either way.
      // raw = the planner's aim reference (tracks the selected plate),
      // mpc = the smoothed solution the gimbal follows.  On a fast spinner
      // the raw curve must oscillate with the plate ring; if the mpc curve
      // is flat while raw swings, the MPC output is averaging the plates
      // away and every shot threads the gap between them.
      const double raw_err = tools::limitRadian(target_yaw_ - state.yaw);
      const double mpc_err = tools::limitRadian(mpc_yaw_ - state.yaw);
      frame["aim/raw_target_yaw"] = state.yaw + raw_err;
      frame["aim/raw_target_pitch"] = target_pitch_;
      frame["aim/target_yaw"] = state.yaw + mpc_err;
      frame["aim/target_pitch"] = mpc_pitch_;
      frame["aim/yaw_error"] = mpc_err;
      frame["aim/pitch_error"] = mpc_pitch_ - state.pitch;
      frame["aim/control"] = control_ ? 1.0 : 0.0;
      frame["aim/fire"] = fire_ ? 1.0 : 0.0;
    }
    plotter_.plot(frame);

    // The real node negates pitch going from the firmware's frame into odom,
    // but the simulator's pitch joint (axis 0 -1 0, negative = camera down)
    // already matches odom's Ry sign -- negating it here mirrored the world
    // vertically: a plate physically 7 deg below the camera reconstructed
    // 0.7 m above it, and every trajectory came back unsolvable.  The odom
    // frame origin coincides with the gimbal centre; the map->odom static
    // transform carries its world position.
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    transform.rotate(tools::rpyToQuaterniond(
        Eigen::Vector3d{0.0, state.pitch, state.yaw}));
    this->tf_pub_.publishTransform(
        transform, configs_.odom_frame_id, configs_.gimbal_fram_id,
        sim_stamp +
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
        // aiming program asked for.  Compare yaw on the circle: the aimer's
        // angle lives on [-pi, pi) while the joint unwinds across turns, so
        // a plain difference is off by 2*pi*N and the gun would never fire.
        const double yaw_error = std::abs(tools::limitRadian(
            sample->target_yaw - self->yaw_.load(std::memory_order_relaxed)));
        const double pitch_error =
            std::abs(sample->target_pitch - self->pitch_.load(std::memory_order_relaxed));
        const bool on_target = yaw_error <= sample->fire_thres_yaw &&
                               pitch_error <= sample->fire_thres_pitch;

        aim::AimCommand command{};
        // Drive the gimbal with the MPC-smoothed state and its feedforward,
        // exactly what the vehicle firmware consumes.  The raw reference
        // (sample->target_yaw) steps at every armour switch -- forwarding it
        // as the position target whipped the simulated gimbal hard enough to
        // break tracking on a fast spinner.  It still decides on_target
        // above, because the fire window is defined around the aim point.
        command.target_yaw = sample->yaw;
        command.target_pitch = sample->pitch;
        command.target_yaw_velocity = sample->yaw_vel;
        command.target_pitch_velocity = sample->pitch_vel;
        command.target_yaw_acceleration = sample->yaw_acc;
        command.target_pitch_acceleration = sample->pitch_acc;
        command.fire_thres_yaw = sample->fire_thres_yaw;
        command.fire_thres_pitch = sample->fire_thres_pitch;
        // Express the command age in the simulator's clock: the aim link's
        // issued_at must be comparable to the simulator's sim_time, not the
        // wall-clock epoch, or every command looks perpetually fresh.
        command.issued_at =
            self->latest_sim_time_.load(std::memory_order_relaxed);
        command.valid = sample->control ? 1U : 0U;
        command.fire = (sample->control && on_target) ? 1U : 0U;
        self->target_yaw_ = sample->target_yaw;
        self->target_pitch_ = sample->target_pitch;
        self->mpc_yaw_ = sample->yaw;
        self->mpc_pitch_ = sample->pitch;
        self->have_target_ = true;
        self->control_ = sample->control;
        self->fire_ = command.fire != 0;
        self->link_.publishCommand(command);
        LOG_DEBUG(self->logger_,
                  "aim -> sim: yaw {:.4f} pitch {:.4f} control {} fire {}",
                  sample->yaw, sample->pitch, sample->control, command.fire);
      })) {
  }
}
