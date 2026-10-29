#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "replay/config.h"
#include "replay/joints.h"

namespace replay {

// Per tick checks of the control loop. Any violation aborts the replay.
class SafetyMonitor {
 public:
  // `joint_names` name the joints in the messages.
  SafetyMonitor(const SafetyConfig& config, const Joints& lower_limit, const Joints& upper_limit,
                std::vector<std::string> joint_names);

  // Resets the per run state, `q_cmd` is the command the run starts from.
  void Reset(const Joints& q_cmd, double t);

  // Returns the reason of a violation at time `t`. `evaluated` is false on ticks without an IK evaluation,
  // `held` tells whether the evaluation held the previous command.
  std::optional<std::string> Check(double t, const Joints& q_cmd, const Joints& q_meas, bool feedback_is_new,
                                   bool evaluated, bool held);

 private:
  SafetyConfig config_;
  std::vector<std::string> joint_names_;
  Joints lower_;
  Joints upper_;
  Joints prev_q_cmd_;
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
  void AddSent(double t, const Joints& q);
  // Delay of the command received at time `t`, nullopt while the recent commands hardly move.
  std::optional<double> Estimate(double t, const Joints& received) const;

 private:
  struct Sent {
    double t;
    Joints q;
  };
  double window_s_;
  std::deque<Sent> sent_;
};

}  // namespace replay
