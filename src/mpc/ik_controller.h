#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mpc/mpc_config.h"

namespace controller {

// Receding-horizon IK of a dual-arm robot as the replay loop drives it. The full joint vector holds the torso joints
// (none on a fixed-base dual arm) followed by the left and the right arm. All times are on one monotonic clock.
class IkController {
 public:
  virtual ~IkController() = default;

  // Starts / stops the background MPC solver threads.
  virtual void Start() = 0;
  virtual void Stop() = 0;

  virtual void Reset(const Eigen::VectorXd& full_q, double time) = 0;

  virtual void SetTargetTrajectory(const std::vector<double>& times,
                                   const std::vector<Eigen::Isometry3d>& left_poses_X_ParentEnd,
                                   const std::vector<Eigen::Isometry3d>& right_poses_X_ParentEnd) = 0;

  // False while no usable policy exists; full_q_cmd then holds the last command.
  virtual bool Evaluate(double time, Eigen::VectorXd& full_q_cmd) = 0;

  // Synchronous MPC iteration; test use, do not mix with Start().
  virtual bool AdvanceOnce() = 0;

  // Position (first 3) and orientation (last 3) error of both end effectors at q.
  virtual void Jnt2CartDualError(const Eigen::Isometry3d& left_target_X_ParentEnd,
                                 const Eigen::Isometry3d& right_target_X_ParentEnd, const Eigen::VectorXd& q,
                                 Eigen::VectorXd& left_error, Eigen::VectorXd& right_error) = 0;

  // Forward kinematics of both end effectors for a full joint vector, used to anchor the reference trajectory at the
  // pose of the last commanded joint position.
  virtual void GetEePoses(const Eigen::VectorXd& full_q, Eigen::Isometry3d& left_pose_X_ParentEnd,
                          Eigen::Isometry3d& right_pose_X_ParentEnd) const = 0;

  virtual double PolicyAge() const = 0;
  // Debug views in the full joint layout: commanded joint velocity after clamping, and the raw MPC policy velocity
  // before clamping. See Ocs2IkController::nominal_qd / mpc_qd.
  virtual Eigen::VectorXd FullNominalQd() const = 0;
  virtual Eigen::VectorXd FullMpcQd() const = 0;
  virtual int clamp_count() const = 0;
  // Size of the full joint vector.
  virtual int dof() const = 0;
};

// The controller named by config.type: one MPC per arm (dual_arm) or one MPC over all joints (whole_body).
std::unique_ptr<IkController> MakeIkController(const std::string& urdf_file_path, const Ocs2IkControllerConfig& config);

}  // namespace controller
