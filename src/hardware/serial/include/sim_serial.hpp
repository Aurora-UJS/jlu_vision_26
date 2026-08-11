#pragma once

// Stands in for the Serial node when the robot is the Webots simulator.
//
// It publishes exactly the topics the real serial node does -- gimbal_info,
// task_mode, enemy_color and the odom->gimbal transform -- and consumes
// AimCommand the same way, so nothing downstream can tell the difference.
// Instead of framing CRC packets over a UART it exchanges the same quantities
// through the simulator's shared-memory link (default /aurora_rm_aim).
//
// Fire control stays where it is on the vehicle: the real firmware compares the
// gimbal's tracking error against fire_thres_yaw / fire_thres_pitch and decides
// whether to shoot.  This node reproduces that check rather than forwarding the
// thresholds, so the aiming program's trigger logic is exercised unchanged.

#include "configs.hpp"
#include "sim_aim_link.hpp"

#include "msgs/AimCommand.hpp"
#include "msgs/EnemyColor.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"
#include "transform/dynamic_tf_publisher.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "quill/Logger.h"

#include <atomic>
#include <thread>

namespace hardware {

class SimSerial {
public:
  SimSerial(quill::Logger *logger, const SerialConfigs &configs,
            const std::string &shm_name);
  ~SimSerial();

private:
  static void onAimCommandReceivedCallback(
      iox::popo::Subscriber<msgs::AimCommand, msgs::Header> *subscriber,
      SimSerial *self);
  // Mirrors Serial::receiveThread: turns robot state into the same topics.
  void stateThread();

  quill::Logger *logger_;
  SerialConfigs configs_;
  aim::Link link_;
  std::jthread state_thread_;
  // Latest gimbal pose, kept so the aim callback can evaluate the fire
  // thresholds without racing the state thread.
  std::atomic<double> yaw_{0.0};
  std::atomic<double> pitch_{0.0};
  iox::popo::Publisher<msgs::TaskMode, msgs::Header> task_mode_pub_;
  iox::popo::Publisher<msgs::GimbalInfo, msgs::Header> gimbal_info_pub_;
  iox::popo::Publisher<msgs::EnemyColor, msgs::Header> enemy_color_pub_;
  iox::popo::Subscriber<msgs::AimCommand, msgs::Header> aim_cmd_sub_;
  iox::popo::Listener aim_cmd_listener_;
  tf::DynamicTransformPublisher tf_pub_;
};

} // namespace hardware
