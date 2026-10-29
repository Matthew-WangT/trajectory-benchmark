#include "replay/safety.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace replay {

SafetyMonitor::SafetyMonitor(const SafetyConfig& config, const Joints& lower_limit, const Joints& upper_limit,
                             std::vector<std::string> joint_names)
    : config_(config),
      joint_names_(std::move(joint_names)),
      lower_(lower_limit.array() + config.joint_limit_margin_rad),
      upper_(upper_limit.array() - config.joint_limit_margin_rad) {}

void SafetyMonitor::Reset(const Joints& q_cmd, double t) {
  prev_q_cmd_ = q_cmd;
  prev_t_ = t;
  last_feedback_t_ = t;
  consecutive_holds_ = 0;
}

std::optional<std::string> SafetyMonitor::Check(double t, const Joints& q_cmd, const Joints& q_meas,
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
      reason << joint_names_[i] << " command " << q_cmd[i] << " rad outside the limits [" << lower_[i] << ", "
             << upper_[i] << "]";
      return reason.str();
    }
  }

  Joints track_err = (q_cmd - q_meas).cwiseAbs();
  int joint = 0;
  if (track_err.maxCoeff(&joint) > config_.max_joint_track_err_rad) {
    reason << joint_names_[joint] << " tracking error " << track_err[joint] << " rad above "
           << config_.max_joint_track_err_rad;
    return reason.str();
  }

  // The command changes once per IK evaluation and is held in between, so the velocity is taken over the time since
  // the previous change.
  if (q_cmd != prev_q_cmd_) {
    const double dt = t - prev_t_;
    const Joints velocity = (q_cmd - prev_q_cmd_).cwiseAbs() / std::max(dt, 1e-6);
    if (velocity.maxCoeff(&joint) > config_.max_cmd_joint_vel_rad_s) {
      reason << joint_names_[joint] << " command step of " << velocity[joint] << " rad/s above "
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

void CommandDelayMonitor::AddSent(double t, const Joints& q) {
  sent_.push_back({t, q});
  while (!sent_.empty() && sent_.front().t < t - window_s_) {
    sent_.pop_front();
  }
}

std::optional<double> CommandDelayMonitor::Estimate(double t, const Joints& received) const {
  // Below this spread of the sent commands the match is ambiguous.
  constexpr double kMinMotionRad = 0.002;
  if (sent_.size() < 2) {
    return std::nullopt;
  }
  Joints lower = sent_.front().q, upper = sent_.front().q;
  double best_err = std::numeric_limits<double>::infinity();
  double best_t = sent_.front().t;
  for (const Sent& sent : sent_) {
    lower = lower.cwiseMin(sent.q);
    upper = upper.cwiseMax(sent.q);
    const double err = (sent.q - received).cwiseAbs().maxCoeff();
    if (err < best_err) {
      best_err = err;
      best_t = sent.t;
    }
  }
  if ((upper - lower).maxCoeff() < kMinMotionRad) {
    return std::nullopt;
  }
  return t - best_t;
}

}  // namespace replay
