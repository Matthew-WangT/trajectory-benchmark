#pragma once

#include <deque>
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

// Estimates how late the controller receives the commands: the received command (RT_IN m_Joint_CMD_Pos) is matched
// against the recently sent ones. Only measurable while the command moves, a still command reads as no delay.
class CommandDelayMonitor {
 public:
  // `window_s` bounds the delay that can be resolved, a longer one reads as `window_s`.
  explicit CommandDelayMonitor(double window_s = 0.2) : window_s_(window_s) {}

  void Reset() { sent_.clear(); }
  void AddSent(double t, const DualJoints& q);
  // Delay of the command received at time `t`, nullopt while the recent commands hardly move.
  std::optional<double> Estimate(double t, const DualJoints& received) const;

 private:
  struct Sent {
    double t;
    DualJoints q;
  };
  double window_s_;
  std::deque<Sent> sent_;
};

}  // namespace replay
