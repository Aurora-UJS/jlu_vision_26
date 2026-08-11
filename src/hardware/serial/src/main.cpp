// Copyright (c) 2026 aaa. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "serial.hpp"
#include "sim_serial.hpp"

#include <cxxopts.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <fstream>
#include <memory>
#include <string>

constexpr char APP_NAME[] = "serial";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME, "publish task_mode,enemy_color,gimbal_info from C board. "
                "subscribe aim command and send it to C board.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/hardware/serial.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/serial"))(
      "h,help", "Print usage.");
  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    std::cout << options.help() << std::endl;
    std::exit(EXIT_SUCCESS);
  }
  auto config_path = result["config"].as<std::string>();
  auto log_path = result["log"].as<std::string>();
  std::ifstream ifs(config_path);
  if (!ifs) {
    std::cerr << "Invalid config path!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto configs_opt = rfl::yaml::read<hardware::SerialConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  hardware::SerialConfigs configs = configs_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  // The simulator backend publishes the same topics over shared memory instead
  // of a UART, so everything downstream is unchanged.
  std::unique_ptr<hardware::Serial> serial;
  std::unique_ptr<hardware::SimSerial> sim_serial;
  if (configs.use_simulator) {
    sim_serial = std::make_unique<hardware::SimSerial>(
        logger, configs,
        configs.sim_shm_name.empty() ? "/aurora_rm_aim" : configs.sim_shm_name);
  } else {
    serial = std::make_unique<hardware::Serial>(logger, configs);
  }
  iox::waitForTerminationRequest();
  return 0;
}
