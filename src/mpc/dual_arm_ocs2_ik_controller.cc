#include "mpc/dual_arm_ocs2_ik_controller.h"

#include <algorithm>
#include <utility>

#include <glog/logging.h>

namespace controller {

DualArmOcs2IkController::DualArmOcs2IkController(const std::string& urdf_file_path,
                                                 const Ocs2IkControllerConfig& config) {
  left_arm_ = std::make_unique<Ocs2IkController>(urdf_file_path, config, config.left_info, "left_arm");
  right_arm_ = std::make_unique<Ocs2IkController>(urdf_file_path, config, config.right_info, "right_arm");
  CHECK_EQ(left_arm_->state_dim(), static_cast<size_t>(dof_)) << "Unexpected left arm state dimension";
  CHECK_EQ(right_arm_->state_dim(), static_cast<size_t>(dof_)) << "Unexpected right arm state dimension";
}

void DualArmOcs2IkController::Start() {
  left_arm_->Start();
  right_arm_->Start();
}

void DualArmOcs2IkController::Stop() {
  left_arm_->Stop();
  right_arm_->Stop();
}

void DualArmOcs2IkController::Reset(const Eigen::VectorXd& full_q, double time) {
  CHECK_GE(full_q.size(), std::max(left_first_joint_id_, right_first_joint_id_) + dof_)
      << "full_q too short for the configured arm segments";
  full_nominal_q_ = full_q;
  left_arm_->Reset(full_q.segment(left_first_joint_id_, dof_), time);
  right_arm_->Reset(full_q.segment(right_first_joint_id_, dof_), time);
}

void DualArmOcs2IkController::SetTargetTrajectory(const std::vector<double>& times,
                                                  const std::vector<Eigen::Isometry3d>& left_poses_X_ParentEnd,
                                                  const std::vector<Eigen::Isometry3d>& right_poses_X_ParentEnd) {
  left_arm_->SetTargetTrajectory(times, left_poses_X_ParentEnd);
  right_arm_->SetTargetTrajectory(times, right_poses_X_ParentEnd);
}

bool DualArmOcs2IkController::Evaluate(double time, Eigen::VectorXd& full_q_cmd) {
  Eigen::VectorXd left_q;
  Eigen::VectorXd right_q;
  const bool left_ok = left_arm_->Evaluate(time, left_q);
  const bool right_ok = right_arm_->Evaluate(time, right_q);
  if (left_ok) {
    full_nominal_q_.segment(left_first_joint_id_, dof_) = left_q;
  }
  if (right_ok) {
    full_nominal_q_.segment(right_first_joint_id_, dof_) = right_q;
  }
  full_q_cmd = full_nominal_q_;
  return left_ok || right_ok;
}

Eigen::VectorXd DualArmOcs2IkController::FullNominalQd() const {
  Eigen::VectorXd full_qd = Eigen::VectorXd::Zero(full_nominal_q_.size());
  full_qd.segment(left_first_joint_id_, dof_) = left_arm_->nominal_qd();
  full_qd.segment(right_first_joint_id_, dof_) = right_arm_->nominal_qd();
  return full_qd;
}

Eigen::VectorXd DualArmOcs2IkController::FullMpcQd() const {
  Eigen::VectorXd full_qd = Eigen::VectorXd::Zero(full_nominal_q_.size());
  full_qd.segment(left_first_joint_id_, dof_) = left_arm_->mpc_qd();
  full_qd.segment(right_first_joint_id_, dof_) = right_arm_->mpc_qd();
  return full_qd;
}

bool DualArmOcs2IkController::AdvanceOnce() {
  const bool left_ok = left_arm_->AdvanceOnce();
  const bool right_ok = right_arm_->AdvanceOnce();
  return left_ok && right_ok;
}

void DualArmOcs2IkController::Jnt2CartDualError(const Eigen::Isometry3d& left_target_X_ParentEnd,
                                                const Eigen::Isometry3d& right_target_X_ParentEnd,
                                                const Eigen::VectorXd& q, Eigen::VectorXd& left_error,
                                                Eigen::VectorXd& right_error) {
  const Eigen::VectorXd left_q = q.segment(left_first_joint_id_, dof_);
  const Eigen::VectorXd right_q = q.segment(right_first_joint_id_, dof_);
  left_arm_->CalculatePoseError(left_target_X_ParentEnd, left_q, left_error);
  right_arm_->CalculatePoseError(right_target_X_ParentEnd, right_q, right_error);
}

void DualArmOcs2IkController::GetEePoses(const Eigen::VectorXd& full_q, Eigen::Isometry3d& left_pose_X_ParentEnd,
                                         Eigen::Isometry3d& right_pose_X_ParentEnd) const {
  left_pose_X_ParentEnd = left_arm_->EePose(full_q.segment(left_first_joint_id_, dof_));
  right_pose_X_ParentEnd = right_arm_->EePose(full_q.segment(right_first_joint_id_, dof_));
}

double DualArmOcs2IkController::PolicyAge() const { return std::max(left_arm_->PolicyAge(), right_arm_->PolicyAge()); }

}  // namespace controller
