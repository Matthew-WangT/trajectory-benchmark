#include "replay/safety.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace replay {
namespace {

std::string JointName(int index) {
  return std::string(index < arm::kArmDoF ? "left" : "right") + " joint " +
         std::to_string(index % arm::kArmDoF + 1);
}

}  // namespace

SafetyMonitor::SafetyMonitor(const SafetyConfig& config, const DualJoints& lower_limit, const DualJoints& upper_limit)
    : config_(config),
      lower_(lower_limit.array() + config.joint_limit_margin_rad),
      upper_(upper_limit.array() - config.joint_limit_margin_rad) {}

void SafetyMonitor::Reset(const DualJoints& q_cmd, double t) {
  prev_q_cmd_ = q_cmd;
  prev_t_ = t;
  last_feedback_t_ = t;
  consecutive_holds_ = 0;
}

std::optional<std::string> SafetyMonitor::Check(double t, const DualJoints& q_cmd, const DualJoints& q_meas,
                                                bool feedback_is_new, bool evaluated, bool held) {
  std::ostringstream reason;
  if (!q_cmd.allFinite()) {
    return std::string("joint command is not finite");
  }
  if (feedback_is_new) {
    last_feedback_t_ = t;
  } else if (t - last_feedback_t_ > config_.feedback_timeout_s) {
    reason << "no new feedback frame for " << (t - last_feedback_t_) * 1e3 << " ms";
    return reason.str();
  }

  for (int i = 0; i < q_cmd.size(); ++i) {
    if (q_cmd[i] < lower_[i] || q_cmd[i] > upper_[i]) {
      reason << JointName(i) << " command " << q_cmd[i] << " rad outside the limits [" << lower_[i] << ", "
             << upper_[i] << "]";
      return reason.str();
    }
  }

  DualJoints track_err = (q_cmd - q_meas).cwiseAbs();
  int joint = 0;
  if (track_err.maxCoeff(&joint) > config_.max_joint_track_err_rad) {
    reason << JointName(joint) << " tracking error " << track_err[joint] << " rad above "
           << config_.max_joint_track_err_rad;
    return reason.str();
  }

  // The command changes once per IK evaluation and is held in between, so the velocity is taken over the time since
  // the previous change.
  if (q_cmd != prev_q_cmd_) {
    const double dt = t - prev_t_;
    const DualJoints velocity = (q_cmd - prev_q_cmd_).cwiseAbs() / std::max(dt, 1e-6);
    if (velocity.maxCoeff(&joint) > config_.max_cmd_joint_vel_rad_s) {
      reason << JointName(joint) << " command step of " << velocity[joint] << " rad/s above "
             << config_.max_cmd_joint_vel_rad_s;
      return reason.str();
    }
    prev_q_cmd_ = q_cmd;
    prev_t_ = t;
  }

  if (evaluated) {
    consecutive_holds_ = held ? consecutive_holds_ + 1 : 0;
    if (consecutive_holds_ > config_.max_consecutive_holds) {
      reason << "IK controller held for " << consecutive_holds_ << " consecutive evaluations";
      return reason.str();
    }
  }
  return std::nullopt;
}

}  // namespace replay
