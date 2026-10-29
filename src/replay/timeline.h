#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "arm/arm_interface.h"
#include "replay/config.h"
#include "replay/trajectory.h"

namespace replay {

enum class Phase {
  kS0JointMove = 0,  // quintic joint interpolation to q_pd, sent without the IK controller
  kS1EefMove = 1,    // min-jerk TCP move from FK(q_pd) to the case start pose, through the IK controller
  kS2Hold = 2,       // still at the start pose
  kS3Track = 3,      // the case trajectory on its t_s axis
  kS4Hold = 4,       // still at the last pose
  kDone = 5,
};

const char* PhaseName(Phase phase);

// TCP waypoint released to the IK controller once the run clock reaches `t`.
struct Waypoint {
  double t = 0.0;
  Eigen::Isometry3d left = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right = Eigen::Isometry3d::Identity();
  Phase phase = Phase::kS1EefMove;
};

// Schedule of one run on the run clock, which starts at 0 when the run starts.
class Timeline {
 public:
  // `q0` is the joint vector the run starts from (torso, left arm, right arm, as StartupConfig::QPd), `left_fk_pd` /
  // `right_fk_pd` are the TCP poses at q_pd.
  Timeline(const StartupConfig& config, const CaseTrajectory& traj, const Joints& q0,
           const Eigen::Isometry3d& left_fk_pd, const Eigen::Isometry3d& right_fk_pd);

  Phase PhaseAt(double t) const;
  // S0 joint command at `t`, clamped to the phase.
  Joints JointCommand(double t) const;
  // Index of the newest waypoint with a time not after `t`, -1 before the first one.
  int LatestWaypoint(double t) const;

  const std::vector<Waypoint>& waypoints() const { return waypoints_; }
  double s1_start() const { return s1_start_; }
  double s2_start() const { return s2_start_; }
  double s3_start() const { return s3_start_; }
  double s4_start() const { return s4_start_; }
  double end() const { return end_; }

 private:
  Joints q0_;
  Joints q_pd_;
  double s1_start_ = 0.0;
  double s2_start_ = 0.0;
  double s3_start_ = 0.0;
  double s4_start_ = 0.0;
  double end_ = 0.0;
  std::vector<Waypoint> waypoints_;
};

// Minimum jerk time scaling s(u), u in [0, 1].
double MinJerk(double u);

}  // namespace replay
