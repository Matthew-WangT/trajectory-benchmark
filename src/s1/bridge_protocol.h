#pragma once

// UDP frames between the replay and scripts/s1_bridge.py, the sidecar that drives the Astribot S1 through its SDK.
// Keep in sync with FEEDBACK / COMMAND of scripts/s1_bridge.py: little endian, packed, fixed size.
//
// The sidecar owns the clock: every SDK period (1 / bridge.freq_hz) it reads the robot, sends one Feedback and waits
// up to bridge.reply_timeout_s for the Command answering it (same seq). Without an answer in time it repeats the
// previous command and counts a miss. The command of tick k is sampled on the run clock at k / freq_hz, so that
// consecutive commands are exactly one SDK period apart, the step the SDK turns into its velocity feedforward.

#include <cstdint>

namespace s1 {

constexpr int kJoints = 18;                      // torso 4, left arm 7, right arm 7
constexpr int kPose = 7;                         // x, y, z, qx, qy, qz, qw
constexpr uint32_t kFeedbackMagic = 0x42463153;  // "S1FB"
constexpr uint32_t kCommandMagic = 0x4d433153;   // "S1CM"
constexpr uint16_t kVersion = 1;

enum FeedbackFlags : uint16_t {
  kFlagFake = 1 << 0,     // the sidecar runs without the SDK, a kinematic stand-in of the robot
  kFlagStopped = 1 << 1,  // the sidecar stopped commanding (stop received or too many misses) and exits
};

enum class CommandMode : uint16_t {
  kIdle = 0,       // command nothing, the robot holds its last desired position
  kJoint = 1,      // set_joints_position(torso, left, right; direct), q
  kCartesian = 2,  // set_cartesian_pose(left, right; direct, use_wbc), pose_left / pose_right
  kStop = 3,       // stop commanding and exit
};

#pragma pack(push, 1)
struct Feedback {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint32_t seq;  // tick of the sidecar, counted from its first answered feedback
  uint32_t consecutive_miss;
  uint32_t total_miss;
  uint32_t reserved;
  double t_mono_s;            // monotonic clock of the sidecar when the read started
  double read_s;              // time the SDK reads of this tick took
  double command_s;           // time the SDK command call of the previous tick took
  double q[kJoints];          // measured, rad, torso / left / right
  double dq[kJoints];         // measured, rad/s
  double tau[kJoints];        // measured, N·m
  double q_desired[kJoints];  // desired position of the SDK (its WBC output in the cartesian mode), rad
  double ee_left[kPose];      // SDK end-effector pose (astribot_arm_left_end_effector) in the chassis frame
  double ee_right[kPose];
};

struct Command {
  uint32_t magic;
  uint16_t version;
  uint16_t mode;  // CommandMode
  uint32_t seq;   // seq of the feedback answered
  uint32_t reserved;
  double q[kJoints];        // kJoint
  double pose_left[kPose];  // kCartesian, SDK end-effector pose in the chassis frame
  double pose_right[kPose];
};
#pragma pack(pop)

static_assert(sizeof(Feedback) == 4 * 6 + 8 * 3 + 8 * (4 * kJoints + 2 * kPose), "Feedback layout");
static_assert(sizeof(Command) == 4 * 4 + 8 * (kJoints + 2 * kPose), "Command layout");

}  // namespace s1
