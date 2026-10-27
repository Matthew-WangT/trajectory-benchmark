#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>

namespace controller {

// Parameters of the receding-horizon IK controller, the fields and defaults of the Ocs2InverseKinematicsParam,
// Ocs2IkControllerParam and DualArmOcs2IkControllerParam of the reference pipeline that the controller reads.
struct Ocs2IkControllerConfig {
  // Folder of the CppAD generated libraries.
  std::string tmp_working_dir = "/tmp";
  // Quadratic joint regularization 0.5 * (q - q_ref)^T Q (q - q_ref) toward the task .info initialState.
  bool joint_reg_enable = true;
  double joint_reg_q_weight = 0.5;
  std::optional<Eigen::VectorXd> joint_reg_q_weight_scale;
  bool default_q_reg_enable = true;
  double default_q_reg_weight = 0.5;
  std::optional<Eigen::VectorXd> default_reg_q_weight_scale;

  // Per-tick command increment is clamped to max_joint_velocity_rad_s * tick period.
  double max_joint_velocity_rad_s = 1.5;
  // Evaluate() fails and holds position when the latest policy is older than this.
  double policy_max_age_s = 0.1;
  double mpc_max_frequency_hz = 200.0;
  // CPU core for the MPC solver thread; -1 leaves scheduling to the OS.
  int mpc_thread_cpu = -1;

  std::string left_info;
  std::string right_info;
};

// Loads the controller yaml. Relative paths in it are resolved against the folder of the yaml file.
Ocs2IkControllerConfig LoadOcs2IkControllerConfig(const std::string& yaml_path);

}  // namespace controller
