// Copyright (c) 2026 aaa. All Rights Reserved.
#pragma once

#include "confs/IceoryxServiceDescription.hpp"

#include "quill/core/LogLevel.h"
#include "serial/serial.h"

#include <cstdint>

namespace hardware {

struct SerialConfig {
  std::string device_name;
  std::uint32_t baudrate;
  serial::stopbits_t stopbits;
  serial::flowcontrol_t flowcontrol;
  serial::parity_t parity;
};
struct IecoryxConfig {
  confs::IceoryxServiceDescription aim_command_topic;
  confs::IceoryxServiceDescription gimbal_info_topic;
  confs::IceoryxServiceDescription enemy_color_topic;
  confs::IceoryxServiceDescription task_mode_topic;
  confs::IceoryxServiceDescription bullet_id_topic;
};
struct SerialConfigs {
  SerialConfig serial_conf;
  IecoryxConfig iceoryx_conf;
  std::string serial_frame_id;
  std::string gimbal_fram_id;
  std::string odom_frame_id;
  double stamp_offset_sec;
  // bool publish_latency_ms;
  quill::LogLevel log_level;
  // Only used by the simulator backend.  A simulated robot has no referee
  // downlink, so these stand in for the fields a real packet would carry.
  bool use_simulator;
  std::string sim_shm_name;
  int sim_task_mode;
  int sim_enemy_color;
};

} // namespace hardware
