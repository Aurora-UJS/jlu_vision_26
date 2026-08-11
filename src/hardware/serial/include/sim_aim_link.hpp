#pragma once

// Shared-memory link to the Webots simulator (mirror of rm_vision_simulator
// webots/controllers/rm_rune_demo/aim_link.hpp -- keep the two in step).
//
// This is deliberately separate from the PulseScope image channel
// (/aurora_rm_vision).  That layout is a vendored ABI shared with the
// PulseScope front end and has no room left in its control block, so extending
// it would break other readers.  An aiming program reads frames and intrinsics
// from PulseScope as before and exchanges gimbal angles here.
//
// Contract, matching how a real robot is driven: the aiming program publishes
// an ABSOLUTE gimbal target in the chassis yaw frame, the same frame the
// simulator reports its own joint angles in.  Ballistic lead and drop are the
// aiming program's job, exactly as they are on the vehicle -- the simulator
// only closes the position loop.
//
// Both sides use a sequence counter as a seqlock: odd means a write is in
// progress, even means the record is stable.  Readers retry while the counter
// is odd or changes across the read.

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aim {

constexpr uint64_t kMagic = 0x41494D4C494E4B31ULL;  // "AIMLINK1"
constexpr uint64_t kVersion = 1;

// Simulator -> aiming program.  Everything needed to turn a pixel target into
// a gimbal command without guessing the simulator's state.
//
// The ballistic block matters: the aiming program owns lead and drop, so it has
// to solve against the same model the simulator integrates, or the solution and
// the flight disagree.  The simulator applies gravity through the physics
// engine and quadratic drag explicitly, so a matching solver integrates
//   dv/dt = g - (rho*Cd*A / 2m) * |v| * v
// with the constants published here.
struct GimbalState {
  uint64_t sequence;
  double sim_time;
  double yaw;               // rad, chassis frame, left positive
  double pitch;             // rad, muzzle up positive
  double yaw_velocity;      // rad/s
  double pitch_velocity;    // rad/s
  double muzzle_speed;      // m/s, the speed a shot leaves at right now
  uint64_t frame_id;        // matches the PulseScope frame this pose belongs to
  // --- ballistic model the simulator is actually integrating ---
  double projectile_mass;         // kg
  double projectile_diameter;     // m
  double drag_coefficient;        // dimensionless, sphere
  double air_density;             // kg/m^3
  double gravity;                 // m/s^2, magnitude
  // Muzzle position and boresight in world coordinates, so the solver does not
  // have to reconstruct them from the joint angles and the robot pose.
  double muzzle_position[3];
  double muzzle_direction[3];
};

// Aiming program -> simulator.
struct AimCommand {
  uint64_t sequence;
  double target_yaw;        // rad, absolute, same frame as GimbalState::yaw
  double target_pitch;      // rad, absolute
  double issued_at;         // sim_time the solution was computed for
  uint32_t valid;           // zero means "no target", the gimbal holds
  uint32_t fire;            // non-zero requests one shot
};

struct Block {
  uint64_t magic;
  uint64_t version;
  GimbalState state;
  AimCommand command;
};

class Link {
public:
  // The simulator owns the segment and calls this with create = true.  An
  // aiming program attaches with create = false: re-opening someone else's
  // segment with O_CREAT is refused even when the file mode would allow the
  // write, so a client that asks to create it fails for no useful reason.
  bool open(const char *name = "/aurora_rm_aim", bool create = true) {
    const int descriptor =
        shm_open(name, create ? (O_CREAT | O_RDWR) : O_RDWR, 0666);
    if (descriptor < 0)
      return false;
    if (create) {
      // shm_open's mode is filtered by the process umask, which usually leaves
      // the segment world-read-only.  The aiming program has to write its
      // commands here and often runs as a different user -- in a container it
      // is typically root while the simulator is not.  Widen it explicitly.
      // This is a development link on a local machine, not a privilege
      // boundary.
      fchmod(descriptor, 0666);
      if (ftruncate(descriptor, sizeof(Block)) != 0) {
        close(descriptor);
        return false;
      }
    }
    void *mapping = mmap(nullptr, sizeof(Block), PROT_READ | PROT_WRITE,
                         MAP_SHARED, descriptor, 0);
    close(descriptor);
    if (mapping == MAP_FAILED)
      return false;
    block_ = static_cast<Block *>(mapping);
    if (block_->magic != kMagic || block_->version != kVersion) {
      if (!create) {
        // Never reinitialise a segment someone else owns -- that would wipe the
        // simulator's state.  A mismatch here means the two sides disagree on
        // the layout, which the caller has to resolve.
        munmap(block_, sizeof(Block));
        block_ = nullptr;
        return false;
      }
      std::memset(block_, 0, sizeof(Block));
      block_->magic = kMagic;
      block_->version = kVersion;
    }
    return true;
  }

  bool ready() const { return block_ != nullptr; }

  void publish(const GimbalState &state) {
    if (!block_)
      return;
    auto *sequence = reinterpret_cast<std::atomic<uint64_t> *>(
        &block_->state.sequence);
    const uint64_t next = sequence->load(std::memory_order_relaxed) + 1;
    sequence->store(next, std::memory_order_release);          // odd: writing
    GimbalState copy = state;
    copy.sequence = next;
    std::memcpy(&block_->state, &copy, sizeof(GimbalState));
    sequence->store(next + 1, std::memory_order_release);      // even: stable
  }

  // Returns false when no stable command is available this step.
  bool poll(AimCommand &out) const {
    if (!block_)
      return false;
    const auto *sequence = reinterpret_cast<const std::atomic<uint64_t> *>(
        &block_->command.sequence);
    for (int attempt = 0; attempt < 4; ++attempt) {
      const uint64_t before = sequence->load(std::memory_order_acquire);
      if (before % 2 != 0)
        continue;
      AimCommand copy;
      std::memcpy(&copy, &block_->command, sizeof(AimCommand));
      if (sequence->load(std::memory_order_acquire) != before)
        continue;
      if (before == 0)
        return false;
      out = copy;
      return true;
    }
    return false;
  }

  // --- the aiming program's side of the same link ---

  // Mirror of publish(), for the program driving the gimbal.
  void publishCommand(const AimCommand &command) {
    if (!block_)
      return;
    auto *sequence = reinterpret_cast<std::atomic<uint64_t> *>(
        &block_->command.sequence);
    const uint64_t next = sequence->load(std::memory_order_relaxed) + 1;
    sequence->store(next, std::memory_order_release);          // odd: writing
    AimCommand copy = command;
    copy.sequence = next;
    std::memcpy(&block_->command, &copy, sizeof(AimCommand));
    sequence->store(next + 1, std::memory_order_release);      // even: stable
  }

  // Mirror of poll(), returning the simulator's latest gimbal state.
  bool pollState(GimbalState &out) const {
    if (!block_)
      return false;
    const auto *sequence = reinterpret_cast<const std::atomic<uint64_t> *>(
        &block_->state.sequence);
    for (int attempt = 0; attempt < 4; ++attempt) {
      const uint64_t before = sequence->load(std::memory_order_acquire);
      if (before % 2 != 0)
        continue;
      GimbalState copy;
      std::memcpy(&copy, &block_->state, sizeof(GimbalState));
      if (sequence->load(std::memory_order_acquire) != before)
        continue;
      if (before == 0)
        return false;
      out = copy;
      return true;
    }
    return false;
  }

  ~Link() {
    if (block_)
      munmap(block_, sizeof(Block));
  }

private:
  Block *block_ = nullptr;
};

}  // namespace aim
