#pragma once

#include <memory>
#include <string>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "mpc/mpc_config.h"

namespace kinematics {

// Fully constructed OCS2 mobile-manipulator model of the IK controller.
struct Ocs2InterfaceBundle {
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface;
  ocs2::mobile_manipulator::ManipulatorModelInfo model_info;
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> ee_kinematics;
  // Mutable regularization references registered in the optimal control problem.
  // Null when the corresponding regularization is disabled in the param.
  std::shared_ptr<ocs2::vector_t> joint_reg_q_ref;
  std::shared_ptr<ocs2::vector_t> default_q_ref;
};

Ocs2InterfaceBundle BuildOcs2Interface(const std::string& urdf_file_path, const std::string& info_path,
                                       const controller::Ocs2IkControllerConfig& param);

}  // namespace kinematics
