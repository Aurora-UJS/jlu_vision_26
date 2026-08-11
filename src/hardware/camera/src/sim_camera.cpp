#include "sim_camera.hpp"

#include "quill/LogMacros.h"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

namespace {
// The writer advances a sequence counter around each update.  Reading between
// those two increments is a torn frame, so retry a bounded number of times
// before giving up for this cycle.
constexpr int kSeqlockAttempts = 8;
// How long to wait for the simulator to produce a frame we have not seen yet.
// The simulator steps at 32 ms; this bounds a stall rather than paces reads.
constexpr auto kFrameTimeout = std::chrono::milliseconds(2000);
constexpr auto kPollInterval = std::chrono::microseconds(500);
}  // namespace

hardware::SimCamera::SimCamera(quill::Logger *logger,
                               const std::string &shm_name, int width,
                               int height)
    : logger_(logger), shm_name_(shm_name), expected_width_(width),
      expected_height_(height) {
  const int descriptor = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
  if (descriptor < 0) {
    LOG_CRITICAL(logger_,
                 "Cannot open simulator shared memory {}. Start the Webots "
                 "simulator first; in a container pass --ipc=host.",
                 shm_name_);
    std::exit(EXIT_FAILURE);
  }
  mapping_size_ = sim_shm::kDefaultSize;
  mapping_ = mmap(nullptr, mapping_size_, PROT_READ, MAP_SHARED, descriptor, 0);
  close(descriptor);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    LOG_CRITICAL(logger_, "Cannot map simulator shared memory {}", shm_name_);
    std::exit(EXIT_FAILURE);
  }

  const auto *header = static_cast<const sim_shm::Header *>(mapping_);
  if (header->magic_number != sim_shm::kMagicNumber ||
      header->version != sim_shm::kVersion) {
    LOG_CRITICAL(logger_,
                 "Simulator shared memory has magic {:#x} version {}, expected "
                 "{:#x} version {}",
                 header->magic_number, header->version, sim_shm::kMagicNumber,
                 sim_shm::kVersion);
    std::exit(EXIT_FAILURE);
  }
  LOG_INFO(logger_, "SimCamera attached to {} ({}x{} expected)", shm_name_,
           expected_width_, expected_height_);
}

hardware::SimCamera::~SimCamera() {
  if (mapping_)
    munmap(mapping_, mapping_size_);
}

bool hardware::SimCamera::copyStableFrame(std::vector<uint8_t> &out,
                                          uint64_t &sequence, uint32_t &width,
                                          uint32_t &height) {
  const auto *header = static_cast<const sim_shm::Header *>(mapping_);
  const auto *counter =
      reinterpret_cast<const std::atomic<uint64_t> *>(&header->sequence);
  for (int attempt = 0; attempt < kSeqlockAttempts; ++attempt) {
    const uint64_t before = counter->load(std::memory_order_acquire);
    if (before % 2 != 0)
      continue;
    const uint64_t offset = header->img_offset;
    const uint64_t size = header->img_size;
    const uint32_t frame_width = header->width;
    const uint32_t frame_height = header->height;
    if (size == 0 || offset + size > mapping_size_)
      return false;
    out.resize(size);
    std::memcpy(out.data(), static_cast<const uint8_t *>(mapping_) + offset,
                size);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (counter->load(std::memory_order_acquire) != before)
      continue;
    sequence = before;
    width = frame_width;
    height = frame_height;
    return true;
  }
  return false;
}

bool hardware::SimCamera::readImage(
    unsigned char *buffer, std::size_t buffer_size,
    std::chrono::system_clock::time_point &stamp) {
  uint64_t sequence = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  const auto deadline = std::chrono::steady_clock::now() + kFrameTimeout;
  // Block until the simulator has produced a frame newer than the last one we
  // handed out, so the pipeline sees each rendered frame once instead of
  // spinning on a stale copy.
  while (true) {
    if (copyStableFrame(scratch_, sequence, width, height) &&
        sequence != last_sequence_)
      break;
    if (std::chrono::steady_clock::now() > deadline) {
      // Pausing the simulator to look at something is routine, and the frame
      // in the segment is still perfectly valid -- so hand it over again
      // rather than reporting a failure.  The node counts failures and exits
      // after enough of them, which would tear the pipeline down every time
      // the simulation is paused.
      if (!stall_warned_) {
        LOG_WARNING(logger_,
                    "No new simulator frame for {} ms (paused?); repeating the "
                    "last one",
                    kFrameTimeout.count());
        stall_warned_ = true;
      }
      if (!copyStableFrame(scratch_, sequence, width, height))
        return false;
      break;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  if (sequence != last_sequence_)
    stall_warned_ = false;
  last_sequence_ = sequence;
  stamp = std::chrono::system_clock::now();

  if (static_cast<int>(width) != expected_width_ ||
      static_cast<int>(height) != expected_height_) {
    if (!geometry_warned_) {
      LOG_ERROR(logger_,
                "Simulator frame is {}x{} but the config expects {}x{}. Fix "
                "the Webots camera or camera.yaml -- rescaling here would "
                "silently invalidate the intrinsics.",
                width, height, expected_width_, expected_height_);
      geometry_warned_ = true;
    }
    return false;
  }
  const std::size_t required =
      static_cast<std::size_t>(expected_width_) * expected_height_ * 3;
  if (required > buffer_size) {
    LOG_CRITICAL(logger_, "Insufficient buffer size! require {}, actual {}",
                 required, buffer_size);
    std::exit(EXIT_FAILURE);
  }
  if (scratch_.size() < static_cast<std::size_t>(width) * height * 4) {
    LOG_ERROR(logger_, "Simulator frame payload is short: {} bytes",
              scratch_.size());
    return false;
  }

  const cv::Mat rgba(expected_height_, expected_width_, CV_8UC4,
                     scratch_.data());
  cv::Mat bgr(expected_height_, expected_width_, CV_8UC3, buffer);
  cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
  return true;
}

bool hardware::SimCamera::changeExposureGain(double exposure, double gain) {
  LOG_DEBUG(logger_,
            "Simulator renders at fixed exposure; ignoring exposure {} gain {}",
            exposure, gain);
  return true;
}
