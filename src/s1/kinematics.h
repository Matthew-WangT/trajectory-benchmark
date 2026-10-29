#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "replay/joints.h"

namespace s1 {

// Forward kinematics of the robot URDF for the replay joint vector (the joints of robot.yaml in their order, the
// other URDF joints at zero), in the URDF root frame (benchmark_base).
class Kinematics {
 public:
  Kinematics(const std::string& urdf, const std::vector<std::string>& joint_names);
  ~Kinematics();

  // Frame id of a URDF frame, throws when it does not exist.
  int FrameId(const std::string& frame) const;
  // Updates the frame placements for `q`.
  void Update(const replay::Joints& q);
  // Pose of a frame at the last Update.
  Eigen::Isometry3d Pose(int frame_id) const;
  // Velocity and position limits of the URDF for the joints of the replay joint vector.
  replay::Joints VelocityLimits() const;
  void PositionLimits(replay::Joints& lower, replay::Joints& upper) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace s1
