#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mpc/ik_controller.h"
#include "mpc/mpc_config.h"
#include "mpc/ocs2_ik_controller.h"

namespace controller {

// Dual-arm composition of Ocs2IkController sharing one monotonic clock, one MPC per arm as in the reference pipeline.
// The full joint vector holds the left arm in [0, kArmDoF) and the right arm in [kArmDoF, 2 * kArmDoF).
class DualArmOcs2IkController final : public IkController {
 public:
  static constexpr int kArmDoF = 7;

  DualArmOcs2IkController(const std::string& urdf_file_path, const Ocs2IkControllerConfig& config);

  void Start() override;
  void Stop() override;

  void Reset(const Eigen::VectorXd& full_q, double time) override;

  void SetTargetTrajectory(const std::vector<double>& times,
                           const std::vector<Eigen::Isometry3d>& left_poses_X_ParentEnd,
                           const std::vector<Eigen::Isometry3d>& right_poses_X_ParentEnd) override;

  // Updates each arm independently.
  bool Evaluate(double time, Eigen::VectorXd& full_q_cmd) override;

  // Synchronous MPC iteration for both arms; test use, do not mix with Start().
  bool AdvanceOnce() override;

  void Jnt2CartDualError(const Eigen::Isometry3d& left_target_X_ParentEnd,
                         const Eigen::Isometry3d& right_target_X_ParentEnd, const Eigen::VectorXd& q,
                         Eigen::VectorXd& left_error, Eigen::VectorXd& right_error) override;

  // Forward kinematics of both arms for a full joint vector, used to anchor the reference trajectory at the pose of
  // the last commanded joint position.
  void GetEePoses(const Eigen::VectorXd& full_q, Eigen::Isometry3d& left_pose_X_ParentEnd,
                  Eigen::Isometry3d& right_pose_X_ParentEnd) const override;

  double PolicyAge() const override;
  // Debug views in the full joint layout: commanded joint velocity after clamping, and the raw MPC policy velocity
  // before clamping. See Ocs2IkController::nominal_qd / mpc_qd.
  Eigen::VectorXd FullNominalQd() const override;
  Eigen::VectorXd FullMpcQd() const override;
  int clamp_count() const override { return left_arm_->clamp_count() + right_arm_->clamp_count(); }
  int dof() const override { return 2 * kArmDoF; }

 private:
  std::unique_ptr<Ocs2IkController> left_arm_;
  std::unique_ptr<Ocs2IkController> right_arm_;

  Eigen::VectorXd full_nominal_q_;
  int left_first_joint_id_ = 0;
  int right_first_joint_id_ = kArmDoF;
  int dof_ = kArmDoF;
};

}  // namespace controller
