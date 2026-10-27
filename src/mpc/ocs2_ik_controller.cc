#include "mpc/ocs2_ik_controller.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include <glog/logging.h>
#include <pthread.h>

namespace controller {

Ocs2IkController::Ocs2IkController(const std::string& urdf_file_path, const Ocs2IkControllerConfig& config,
                                   const std::string& info_path, const std::string& name)
    : param_(std::make_shared<const Ocs2IkControllerConfig>(config)), name_(name) {
  if (param_->joint_reg_enable) {
    LOG(WARNING) << name_ << " joint_regularization pulls toward the task .info initial state in controller mode; "
                 << "it is designed for the one-shot solver and should normally be disabled here.";
  }

  auto bundle = kinematics::BuildOcs2Interface(urdf_file_path, info_path, *param_);
  interface_ = std::move(bundle.interface);
  model_info_ = bundle.model_info;
  ee_kinematics_ = std::move(bundle.ee_kinematics);
  default_q_ref_ = std::move(bundle.default_q_ref);

  observation_.time = 0.0;
  observation_.state = interface_->getInitialState();
  observation_.input.setZero(model_info_.inputDim);
  nominal_q_ = observation_.state;
  nominal_qd_ = Eigen::VectorXd::Zero(model_info_.stateDim);
  mpc_qd_ = Eigen::VectorXd::Zero(model_info_.stateDim);

  mpc_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(interface_->mpcSettings(), interface_->ddpSettings(),
                                                    interface_->getRollout(), interface_->getOptimalControlProblem(),
                                                    interface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(interface_->getReferenceManagerPtr());
  mpc_mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_);
}

Ocs2IkController::~Ocs2IkController() { Stop(); }

void Ocs2IkController::Start() {
  if (running_.exchange(true)) {
    return;
  }
  mpc_thread_ = std::thread([this]() { MpcLoop(); });

  const int cpu = param_->mpc_thread_cpu;
  if (cpu >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if (pthread_setaffinity_np(mpc_thread_.native_handle(), sizeof(cpu_set_t), &cpuset) != 0) {
      LOG(ERROR) << name_ << " failed to set MPC thread affinity to cpu " << cpu;
    }
  }
}

void Ocs2IkController::Stop() {
  if (!running_.exchange(false)) {
    return;
  }
  if (mpc_thread_.joinable()) {
    mpc_thread_.join();
  }
}

void Ocs2IkController::Reset(const Eigen::VectorXd& q, double time) {
  CHECK_EQ(static_cast<size_t>(q.size()), model_info_.stateDim) << name_ << " q size must match model stateDim";
  has_target_ = false;

  std::lock_guard<std::mutex> lock(mpc_mutex_);
  mpc_->reset();
  mpc_->getSolverPtr()->reset();
  last_policy_sec_ = -1.0;

  nominal_q_ = q;
  nominal_qd_.setZero();
  mpc_qd_.setZero();
  last_eval_time_.store(time, std::memory_order_relaxed);
  observation_.time = time;
  observation_.state = q;
  observation_.input.setZero(model_info_.inputDim);
  mpc_mrt_interface_->setCurrentObservation(observation_);
  reset_done_ = true;
}

void Ocs2IkController::SetTargetTrajectory(const std::vector<double>& times,
                                           const std::vector<Eigen::Isometry3d>& poses_X_ParentEnd) {
  CHECK(reset_done_) << name_ << " Reset() must be called before SetTargetTrajectory()";
  CHECK(!times.empty()) << name_ << " target trajectory is empty";
  CHECK_EQ(times.size(), poses_X_ParentEnd.size()) << name_ << " times and poses size mismatch";
  CHECK_GT(times.back(), last_eval_time_.load(std::memory_order_relaxed))
      << name_ << " target trajectory lies entirely in the past";

  ocs2::scalar_array_t time_trajectory;
  ocs2::vector_array_t state_trajectory;
  ocs2::vector_array_t input_trajectory;
  time_trajectory.reserve(times.size());
  state_trajectory.reserve(times.size());
  input_trajectory.reserve(times.size());
  for (size_t i = 0; i < times.size(); ++i) {
    if (i > 0) {
      CHECK_GT(times[i], times[i - 1]) << name_ << " target times must be strictly increasing";
    }
    ocs2::vector_t goal_state(kGoalStateDim);
    goal_state.segment<3>(0) = poses_X_ParentEnd[i].translation().cast<ocs2::scalar_t>();
    const auto rq = Eigen::Quaterniond(poses_X_ParentEnd[i].linear());
    goal_state.segment<4>(3) = Eigen::Quaternion<ocs2::scalar_t>(rq.w(), rq.x(), rq.y(), rq.z()).coeffs();
    time_trajectory.push_back(times[i]);
    state_trajectory.push_back(std::move(goal_state));
    input_trajectory.push_back(ocs2::vector_t::Zero(model_info_.inputDim));
  }

  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(
      ocs2::TargetTrajectories(std::move(time_trajectory), std::move(state_trajectory), std::move(input_trajectory)));
  has_target_ = true;
}

bool Ocs2IkController::Evaluate(double time, Eigen::VectorXd& q_cmd) {
  q_cmd = nominal_q_;
  if (!reset_done_) {
    return false;
  }
  // A held tick is an instantaneous stop as far as the debug velocities are concerned; they are
  // overwritten below only when this tick produces a fresh command.
  nominal_qd_.setZero();
  mpc_qd_.setZero();

  // Keep the MPC observation clock aligned with the control loop even before
  // the first target/policy arrives. Otherwise a delayed first target is solved
  // from the Reset() timestamp and is already expired when it is evaluated.
  const double prev_eval_time = last_eval_time_.load(std::memory_order_relaxed);
  observation_.time = time;
  observation_.state = nominal_q_;
  observation_.input.setZero(model_info_.inputDim);
  mpc_mrt_interface_->setCurrentObservation(observation_);
  last_eval_time_.store(time, std::memory_order_relaxed);

  if (!has_target_) {
    return false;
  }
  if (!mpc_mrt_interface_->initialPolicyReceived()) {
    return false;
  }

  const bool policy_updated = mpc_mrt_interface_->updatePolicy();
  if (policy_updated) {
    last_policy_sec_ = NowSec();
  }
  if (last_policy_sec_ >= 0.0 && PolicyAge() > param_->policy_max_age_s) {
    LOG_EVERY_N(WARNING, 1000) << name_ << " policy is stale (" << PolicyAge() << " s); holding position";
    return false;
  }

  // Never extrapolate an expired OCS2 policy. MPC_BASE::run() also refuses to
  // advance from an expired horizon, so MpcLoop() will reset and re-seed it.
  const auto& policy = mpc_mrt_interface_->getPolicy();
  if (policy.timeTrajectory_.empty() || time > policy.timeTrajectory_.back()) {
    LOG_EVERY_N(WARNING, 1000) << name_ << " active policy horizon expired (eval=" << time
                               << " final=" << (policy.timeTrajectory_.empty() ? -1.0 : policy.timeTrajectory_.back())
                               << "); holding until MPC recovers";
    return false;
  }

  size_t mode = 0;
  ocs2::vector_t optimal_q(model_info_.stateDim);
  ocs2::vector_t optimal_input;
  mpc_mrt_interface_->evaluatePolicy(time, observation_.state, optimal_q, optimal_input, mode);

  const double dt = std::max(time - prev_eval_time, 1e-4);
  mpc_qd_ = optimal_input;
  const double max_dq = param_->max_joint_velocity_rad_s * dt;
  Eigen::VectorXd dq = optimal_q - nominal_q_;
  if (dq.cwiseAbs().maxCoeff() > max_dq) {
    dq = dq.cwiseMax(-max_dq).cwiseMin(max_dq);
    ++clamp_count_;
  }
  nominal_q_ += dq;
  nominal_qd_ = dq / dt;

  observation_.state = nominal_q_;
  mpc_mrt_interface_->setCurrentObservation(observation_);

  q_cmd = nominal_q_;
  return true;
}

bool Ocs2IkController::AdvanceOnce() {
  CHECK(!running_) << name_ << " AdvanceOnce() must not be mixed with a running solver thread";
  if (!reset_done_ || !has_target_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mpc_mutex_);
  RecoverIfHorizonExpired();
  mpc_mrt_interface_->advanceMpc();
  return mpc_mrt_interface_->initialPolicyReceived();
}

void Ocs2IkController::RecoverIfHorizonExpired() {
  if (!mpc_mrt_interface_->initialPolicyReceived()) {
    return;
  }
  const double final_time = mpc_->getSolverPtr()->getFinalTime();
  const double eval_time = last_eval_time_.load(std::memory_order_relaxed);
  if (eval_time >= final_time) {
    LOG_EVERY_N(WARNING, 50) << name_ << " control time passed the MPC horizon (eval=" << eval_time
                             << " final=" << final_time << "); resetting solver";
    mpc_->reset();
  }
}

void Ocs2IkController::MpcLoop() {
  const auto min_period = std::chrono::duration<double>(1.0 / param_->mpc_max_frequency_hz);
  while (running_) {
    if (!has_target_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto start = std::chrono::steady_clock::now();
    try {
      std::lock_guard<std::mutex> lock(mpc_mutex_);
      RecoverIfHorizonExpired();
      mpc_mrt_interface_->advanceMpc();
    } catch (const std::exception& e) {
      LOG_EVERY_N(ERROR, 100) << name_ << " advanceMpc failed: " << e.what();
    }
    std::this_thread::sleep_until(start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(min_period));
  }
}

void Ocs2IkController::CalculatePoseError(const Eigen::Isometry3d& target_X_ParentEnd, const Eigen::VectorXd& q,
                                          Eigen::VectorXd& error) {
  const Eigen::Matrix<ocs2::scalar_t, 3, 1> goal_pos = target_X_ParentEnd.translation().cast<ocs2::scalar_t>();
  const auto rq = Eigen::Quaterniond(target_X_ParentEnd.linear());
  const Eigen::Quaternion<ocs2::scalar_t> goal_ori(rq.w(), rq.x(), rq.y(), rq.z());

  const Eigen::Matrix<ocs2::scalar_t, 3, 1> pos_err = ee_kinematics_->getPosition(q).front() - goal_pos;
  const Eigen::Matrix<ocs2::scalar_t, 3, 1> ori_err = ee_kinematics_->getOrientationError(q, {goal_ori}).front();

  error.resize(kPoseErrorDim);
  error.head<3>() = pos_err;
  error.tail<3>() = ori_err;
}

Eigen::Isometry3d Ocs2IkController::EePose(const Eigen::VectorXd& arm_q) const {
  const Eigen::Vector3d position = ee_kinematics_->getPosition(arm_q).front();
  // getOrientationError(q, {qRef}) == ocs2::quaternionDistance(q_fk, qRef). With qRef = identity it
  // reduces to -q_fk.vec() (see RotationTransforms.h), so the FK quaternion's vector part is exactly
  // the negated error.
  const Eigen::Vector3d vec = -ee_kinematics_->getOrientationError(arm_q, {Eigen::Quaterniond::Identity()}).front();
  const double w_mag = std::sqrt(std::max(0.0, 1.0 - vec.squaredNorm()));
  Eigen::Quaterniond orientation(w_mag, vec.x(), vec.y(), vec.z());
  const double residual = ee_kinematics_->getOrientationError(arm_q, {orientation}).front().norm();
  if (residual > 1e-6) {
    orientation.w() = -w_mag;
  }
  orientation.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = orientation.toRotationMatrix();
  pose.translation() = position;
  return pose;
}

double Ocs2IkController::PolicyAge() const {
  const double last = last_policy_sec_;
  if (last < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return NowSec() - last;
}

double Ocs2IkController::NowSec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace controller
