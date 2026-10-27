#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>

#include "replay/config.h"

namespace replay {

// Joint vector of both arms, the left arm first.
using DualJoints = Eigen::Matrix<double, 2 * arm::kArmDoF, 1>;

// Per tick checks of the control loop. Any violation aborts the replay.
class SafetyMonitor {
 public:
  SafetyMonitor(const SafetyConfig& config, const DualJoints& lower_limit, const DualJoints& upper_limit);

  // Resets the per run state, `q_cmd` is the command the run starts from.
  void Reset(const DualJoints& q_cmd, double t);

  // Returns the reason of a violation at time `t`. `evaluated` is false on ticks without an IK evaluation,
  // `held` tells whether the evaluation held the previous command.
  std::optional<std::string> Check(double t, const DualJoints& q_cmd, const DualJoints& q_meas, bool feedback_is_new,
                                   bool evaluated, bool held);

 private:
  SafetyConfig config_;
  DualJoints lower_;
  DualJoints upper_;
  DualJoints prev_q_cmd_ = DualJoints::Zero();
  // Time of the last change of the command.
  double prev_t_ = 0.0;
  double last_feedback_t_ = 0.0;
  int consecutive_holds_ = 0;
};

}  // namespace replay
