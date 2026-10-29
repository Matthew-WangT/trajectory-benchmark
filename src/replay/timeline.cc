#include "replay/timeline.h"

#include <algorithm>
#include <cmath>

namespace replay {
namespace {

double RotationAngle(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double s) {
  const Eigen::Quaterniond qa(a.linear());
  const Eigen::Quaterniond qb(b.linear());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = qa.slerp(s, qb).toRotationMatrix();
  pose.translation() = (1.0 - s) * a.translation() + s * b.translation();
  return pose;
}

}  // namespace

const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kS0JointMove:
      return "S0";
    case Phase::kS1EefMove:
      return "S1";
    case Phase::kS2Hold:
      return "S2";
    case Phase::kS3Track:
      return "S3";
    case Phase::kS4Hold:
      return "S4";
    case Phase::kDone:
      return "done";
  }
  return "unknown";
}

double MinJerk(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
}

Timeline::Timeline(const StartupConfig& config, const CaseTrajectory& traj, const Joints& q0,
                   const Eigen::Isometry3d& left_fk_pd, const Eigen::Isometry3d& right_fk_pd)
    : q0_(q0), q_pd_(config.QPd()) {
  const double max_dq = (q_pd_ - q0_).cwiseAbs().maxCoeff();
  s1_start_ = std::max(max_dq / config.joint_speed_rad_s, config.joint_min_duration_s);

  const double distance = std::max((traj.left_start.translation() - left_fk_pd.translation()).norm(),
                                   (traj.right_start.translation() - right_fk_pd.translation()).norm());
  const double angle =
      std::max(RotationAngle(left_fk_pd, traj.left_start), RotationAngle(right_fk_pd, traj.right_start));
  const double s1_duration = std::max({distance / config.eef_linear_speed_m_s, angle / config.eef_angular_speed_rad_s,
                                       config.eef_min_duration_s});
  s2_start_ = s1_start_ + s1_duration;
  s3_start_ = s2_start_ + config.hold_s;
  s4_start_ = s3_start_ + traj.t.back();
  end_ = s4_start_ + config.end_hold_s;

  const int s1_count = std::max(1, static_cast<int>(std::ceil(s1_duration * config.eef_waypoint_rate_hz)));
  waypoints_.reserve(s1_count + 1 + traj.t.size());
  for (int i = 0; i <= s1_count; ++i) {
    const double u = static_cast<double>(i) / s1_count;
    const double s = MinJerk(u);
    waypoints_.push_back({s1_start_ + u * s1_duration, Interpolate(left_fk_pd, traj.left_start, s),
                          Interpolate(right_fk_pd, traj.right_start, s), Phase::kS1EefMove});
  }
  for (size_t i = 0; i < traj.t.size(); ++i) {
    waypoints_.push_back({s3_start_ + traj.t[i], traj.left[i], traj.right[i], Phase::kS3Track});
  }
}

Phase Timeline::PhaseAt(double t) const {
  if (t < s1_start_) {
    return Phase::kS0JointMove;
  }
  if (t < s2_start_) {
    return Phase::kS1EefMove;
  }
  if (t < s3_start_) {
    return Phase::kS2Hold;
  }
  if (t < s4_start_) {
    return Phase::kS3Track;
  }
  if (t < end_) {
    return Phase::kS4Hold;
  }
  return Phase::kDone;
}

Joints Timeline::JointCommand(double t) const {
  // The quintic with zero boundary velocity and acceleration is the minimum jerk profile.
  // Without an S0 (the solve starts at q_pd) the command is q_pd.
  const double s = s1_start_ > 0.0 ? MinJerk(t / s1_start_) : 1.0;
  return q0_ + s * (q_pd_ - q0_);
}

int Timeline::LatestWaypoint(double t) const {
  const auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), t,
                                   [](double time, const Waypoint& waypoint) { return time < waypoint.t; });
  return static_cast<int>(it - waypoints_.begin()) - 1;
}

}  // namespace replay
