#pragma once

// Camera backend that reads frames rendered by the Webots simulator.
//
// The simulator publishes RGBA frames into a POSIX shared-memory segment
// (default /aurora_rm_vision).  Its camera is configured to the same 1440x1080
// geometry and the same fx as configs/hardware/camera.yaml, so no rescaling is
// needed here -- only the RGBA to BGR channel swap OpenCV expects.
//
// When running the framework in a container, the segment is only visible if the
// container shares the host IPC namespace (docker run --ipc=host).

#include "camera.hpp"
#include "sim_shm_layout.hpp"

#include "quill/Logger.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hardware {

class SimCamera : public CameraBase {
public:
  SimCamera(quill::Logger *logger, const std::string &shm_name,
            int width, int height);
  ~SimCamera() override;

  bool readImage(unsigned char *buffer, std::size_t buffer_size,
                 std::chrono::system_clock::time_point &stamp) override;
  // The simulator renders with a fixed exposure; accepted and ignored so the
  // parameter-change topic behaves the same as with a real camera.
  bool changeExposureGain(double exposure, double gain) override;

private:
  // Copies one stable frame out of the segment, or returns false if the writer
  // was mid-update through every attempt.
  bool copyStableFrame(std::vector<uint8_t> &out, uint64_t &sequence,
                       uint32_t &width, uint32_t &height);

  quill::Logger *logger_;
  std::string shm_name_;
  int expected_width_;
  int expected_height_;
  void *mapping_ = nullptr;
  std::size_t mapping_size_ = 0;
  uint64_t last_sequence_ = 0;
  std::vector<uint8_t> scratch_;
  bool geometry_warned_ = false;
};

}  // namespace hardware
