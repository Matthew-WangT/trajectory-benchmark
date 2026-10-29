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

// One MPC over all joints of the task .info (a torso and both arms), tracking the left and the right end effector
// (model_information.eeFrames) together, so the shared torso joints serve both arms. The full joint vector is the
// MPC state in the joint order of the model.
class WholeBodyOcs2IkController final : public IkController {
 public:
  WholeBodyOcs2IkController(const std::string& urdf_file_path, const Ocs2IkControllerConfig& config);

  void Start() override { mpc_->Start(); }
  void Stop() override { mpc_->Stop(); }

  void Reset(const Eigen::VectorXd& full_q, double time) override { mpc_->Reset(full_q, time); }

  void SetTargetTrajectory(const std::vector<double>& times,
                           const std::vector<Eigen::Isometry3d>& left_poses_X_ParentEnd,
                           const std::vector<Eigen::Isometry3d>& right_poses_X_ParentEnd) override {
    mpc_->SetTargetTrajectory(times, {left_poses_X_ParentEnd, right_poses_X_ParentEnd});
  }

  bool Evaluate(double time, Eigen::VectorXd& full_q_cmd) override { return mpc_->Evaluate(time, full_q_cmd); }
  bool AdvanceOnce() override { return mpc_->AdvanceOnce(); }

  void Jnt2CartDualError(const Eigen::Isometry3d& left_target_X_ParentEnd,
                         const Eigen::Isometry3d& right_target_X_ParentEnd, const Eigen::VectorXd& q,
                         Eigen::VectorXd& left_error, Eigen::VectorXd& right_error) override {
    mpc_->CalculatePoseError(left_target_X_ParentEnd, q, left_error, 0);
    mpc_->CalculatePoseError(right_target_X_ParentEnd, q, right_error, 1);
  }

  void GetEePoses(const Eigen::VectorXd& full_q, Eigen::Isometry3d& left_pose_X_ParentEnd,
                  Eigen::Isometry3d& right_pose_X_ParentEnd) const override {
    left_pose_X_ParentEnd = mpc_->EePose(full_q, 0);
    right_pose_X_ParentEnd = mpc_->EePose(full_q, 1);
  }

  double PolicyAge() const override { return mpc_->PolicyAge(); }
  Eigen::VectorXd FullNominalQd() const override { return mpc_->nominal_qd(); }
  Eigen::VectorXd FullMpcQd() const override { return mpc_->mpc_qd(); }
  int clamp_count() const override { return mpc_->clamp_count(); }
  int dof() const override { return static_cast<int>(mpc_->state_dim()); }

 private:
  std::unique_ptr<Ocs2IkController> mpc_;
};

}  // namespace controller
