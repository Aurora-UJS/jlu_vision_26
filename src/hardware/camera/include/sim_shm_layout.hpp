#pragma once

// Layout of the simulator's image channel.
//
// Copied from rm_vision_simulator (native/pulsescope_unity/vendor/
// shm_layout.hpp), which is the PulseScope ABI the Webots controller writes.
// Only the fields this reader needs are described; the trailing regions are
// located through the offsets in the header rather than assumed.  Keep
// kShmVersion in step with the writer -- SimCamera refuses to read a segment
// whose magic or version does not match, rather than misinterpreting bytes.

#include <cstddef>
#include <cstdint>

namespace sim_shm {

constexpr uint64_t kMagicNumber = 0x564953494F4E3031ULL;  // "VISION01"
constexpr uint64_t kVersion = 2;
constexpr size_t kEsdfWidth = 100;
constexpr size_t kEsdfHeight = 100;
constexpr size_t kEsdfCells = kEsdfWidth * kEsdfHeight;
constexpr size_t kDefaultSize = 10 * 1024 * 1024;

#pragma pack(push, 8)
struct Header {
  uint64_t magic_number;
  uint64_t version;
  uint64_t sequence;    // even = stable, odd = a write is in progress
  uint64_t timestamp_ms;

  uint64_t img_offset;
  uint64_t img_size;
  uint32_t width;
  uint32_t height;

  uint64_t json_offset;
  uint64_t json_size;

  float esdf_map[kEsdfCells];

  float pid_p;
  float pid_i;
  float pid_d;
  uint32_t exposure_time;
  uint8_t is_fire_enabled;
  uint8_t reserved[3];
};
#pragma pack(pop)

}  // namespace sim_shm
