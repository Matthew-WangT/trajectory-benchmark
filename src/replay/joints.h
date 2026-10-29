#pragma once

#include <Eigen/Core>

namespace replay {

// Upper bound of the joint count of a robot, so that a joint vector never allocates in the control loop.
constexpr int kMaxJoints = 24;
// Joint vector of the whole robot: the torso joints (none on a fixed-base dual arm), the left arm, the right arm.
using Joints = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, kMaxJoints, 1>;

}  // namespace replay
