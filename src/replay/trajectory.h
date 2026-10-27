#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace replay {

// Pose from a position and a quaternion given as x, y, z, w.
Eigen::Isometry3d MakePose(const Eigen::Vector3d& position, const Eigen::Vector4d& quat_xyzw);
// Quaternion of `pose` as x, y, z, w.
Eigen::Vector4d QuatXyzw(const Eigen::Isometry3d& pose);

// Dual-arm TCP trajectory of a benchmark case, eef_command.csv and meta.json of its folder.
struct CaseTrajectory {
  std::string name;
  // Command time axis of the rows, t_s of eef_command.csv.
  std::vector<double> t;
  std::vector<Eigen::Isometry3d> left;
  std::vector<Eigen::Isometry3d> right;
  Eigen::Isometry3d left_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_start = Eigen::Isometry3d::Identity();
  double duration_s = 0.0;

  // Smallest distance between the two TCPs over the rows.
  double MinTcpDistance() const;
};

CaseTrajectory LoadCase(const std::string& case_dir);

}  // namespace replay
