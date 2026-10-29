#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mpc/mpc_config.h"
#include "mpc/ocs2_interface_builder.h"

namespace controller {

// Receding-horizon IK controller for a single arm, ported from the reference pipeline (mindon_robotics
// src/core/controller/ocs2_ik_controller) without its parameter server and tracking-stats logging. It keeps the MPC
// warm across control ticks: a background thread runs advanceMpc() continuously while the control thread evaluates
// the latest policy once per tick. All state fed back to the MPC is nominal (previous command), never measured.
class Ocs2IkController {
 public:
  // Target state of one end effector: position and quaternion xyzw. With several end effectors (eeFrames of the task
  // .info) the target state holds them one after the other.
  static constexpr int kGoalStateDim = 7;
  static constexpr int kPoseErrorDim = 6;

  Ocs2IkController(const std::string& urdf_file_path, const Ocs2IkControllerConfig& config,
                   const std::string& info_path, const std::string& name);
  ~Ocs2IkController();

  // Starts / stops the background MPC solver thread.
  void Start();
  void Stop();

  // Clears MPC warm start and re-seeds the nominal state; drops the current target trajectory.
  // Must be called once before SetTargetTrajectory / Evaluate. `time` is the monotonic
  // controller clock shared with SetTargetTrajectory and Evaluate.
  void Reset(const Eigen::VectorXd& q, double time);

  // Sets the reference for the receding horizon. Times must be strictly increasing and lie
  // (at least partially) in the future of the controller clock.
  void SetTargetTrajectory(const std::vector<double>& times, const std::vector<Eigen::Isometry3d>& poses_X_ParentEnd);
  // One pose sequence per end effector, in the eeFrames order, each as long as `times`.
  void SetTargetTrajectory(const std::vector<double>& times,
                           const std::vector<std::vector<Eigen::Isometry3d>>& ee_poses_X_ParentEnd);

  // Called once per control tick. Returns false while no usable policy exists (not reset,
  // no target, no policy received yet, or policy older than policy_max_age_s); q_cmd then
  // holds the last nominal position so the caller can keep commanding it.
  bool Evaluate(double time, Eigen::VectorXd& q_cmd);

  // Runs one synchronous MPC iteration in the calling thread. Deterministic alternative to
  // Start() for tests; do not mix with a running solver thread.
  bool AdvanceOnce();

  void CalculatePoseError(const Eigen::Isometry3d& target_X_ParentEnd, const Eigen::VectorXd& q, Eigen::VectorXd& error,
                          int ee = 0);

  Eigen::Isometry3d EePose(const Eigen::VectorXd& q, int ee = 0) const;

  // Seconds since the solver last produced a policy; +inf before the first policy.
  double PolicyAge() const;
  int clamp_count() const { return clamp_count_; }
  size_t state_dim() const { return model_info_.stateDim; }
  int ee_count() const { return static_cast<int>(model_info_.eeFrames.size()); }
  const Eigen::VectorXd& nominal_q() const { return nominal_q_; }
  // Commanded joint velocity of the last Evaluate() (finite difference of nominal_q after the
  // velocity clamp); zero after Reset() and on any tick that did not produce a fresh command.
  // Not fed back to the solver. Debug only.
  const Eigen::VectorXd& nominal_qd() const { return nominal_qd_; }
  // Raw joint velocity input of the MPC policy at the last Evaluate(), before the clamp; zero
  // after Reset() and on any tick that did not produce a fresh command. Debug only.
  const Eigen::VectorXd& mpc_qd() const { return mpc_qd_; }
  const std::string& name() const { return name_; }

 private:
  void MpcLoop();
  // Must be called with mpc_mutex_ held. Once the control clock passes the
  // previous solver horizon, OCS2 refuses to solve again until it is reset.
  void RecoverIfHorizonExpired();
  static double NowSec();

  std::shared_ptr<const Ocs2IkControllerConfig> param_;
  std::string name_;

  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  ocs2::mobile_manipulator::ManipulatorModelInfo model_info_;
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> ee_kinematics_;
  std::shared_ptr<ocs2::vector_t> default_q_ref_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;

  // Guards advanceMpc/reset which mutate solver internals from different threads.
  std::mutex mpc_mutex_;
  std::thread mpc_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> has_target_{false};
  std::atomic<double> last_policy_sec_{-1.0};

  ocs2::SystemObservation observation_;
  Eigen::VectorXd nominal_q_;
  Eigen::VectorXd nominal_qd_;
  Eigen::VectorXd mpc_qd_;
  // Written by Evaluate(), read by the MPC thread.
  std::atomic<double> last_eval_time_{0.0};
  bool reset_done_ = false;
  int clamp_count_ = 0;
};

}  // namespace controller
