#include "mpc/ocs2_interface_builder.h"

#include <optional>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>

#include <Eigen/Core>

namespace kinematics {
namespace {
class JointRegularizationCost final : public ocs2::StateCost {
 public:
  JointRegularizationCost(ocs2::matrix_t Q, std::shared_ptr<ocs2::vector_t> qRef)
      : Q_(std::move(Q)), qRef_(std::move(qRef)) {
    CHECK(qRef_ != nullptr);
    CHECK_EQ(Q_.rows(), Q_.cols()) << "Q must be square";
    CHECK_EQ(Q_.rows(), static_cast<int>(qRef_->size())) << "Q and qRef dimensions must match";
  }

  ~JointRegularizationCost() override = default;
  JointRegularizationCost* clone() const override { return new JointRegularizationCost(*this); }

  ocs2::scalar_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state,
                          const ocs2::TargetTrajectories& /*targetTrajectories*/,
                          const ocs2::PreComputation&) const override {
    const ocs2::vector_t qDeviation = state - *qRef_;
    return 0.5 * qDeviation.dot(Q_ * qDeviation);
  }

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::TargetTrajectories& /*targetTrajectories*/,
      const ocs2::PreComputation&) const override {
    const ocs2::vector_t qDeviation = state - *qRef_;

    ocs2::ScalarFunctionQuadraticApproximation Phi;
    Phi.dfdxx = Q_;
    Phi.dfdx.noalias() = Q_ * qDeviation;
    Phi.f = 0.5 * qDeviation.dot(Phi.dfdx);
    return Phi;
  }

 private:
  ocs2::matrix_t Q_;
  std::shared_ptr<ocs2::vector_t> qRef_;
};

void AddStateRegularization(ocs2::mobile_manipulator::MobileManipulatorInterface& iface, const std::string& cost_name,
                            const std::shared_ptr<ocs2::vector_t>& q_ref, size_t state_dim, double weight,
                            const std::optional<Eigen::VectorXd>& weight_scale = std::nullopt) {
  CHECK(q_ref != nullptr) << "state regularization reference is null for " << cost_name;
  const int n = static_cast<int>(state_dim);
  ocs2::matrix_t Q = ocs2::matrix_t::Identity(n, n) * static_cast<ocs2::scalar_t>(weight);
  if (weight_scale) {
    CHECK_EQ(weight_scale.value().size(), n) << "weight_scale size must match stateDim";
    Q = Q * weight_scale.value().asDiagonal();
  }
  iface.getOptimalControlProblem().stateCostPtr->add(cost_name,
                                                     std::make_unique<JointRegularizationCost>(std::move(Q), q_ref));
}
}  // namespace

Ocs2InterfaceBundle BuildOcs2Interface(const std::string& urdf_file_path, const std::string& info_path,
                                       const controller::Ocs2IkControllerConfig& param) {
  Ocs2InterfaceBundle bundle;
  bundle.interface = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorInterface>(
      info_path, param.tmp_working_dir, urdf_file_path);
  bundle.model_info = bundle.interface->getManipulatorModelInfo();

  ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingCppAd pinocchioMapping(bundle.model_info);
  const auto& pinocchio = bundle.interface->getPinocchioInterface();
  bundle.ee_kinematics = std::make_shared<ocs2::PinocchioEndEffectorKinematicsCppAd>(
      pinocchio, pinocchioMapping, bundle.model_info.eeFrames, bundle.model_info.stateDim, bundle.model_info.inputDim,
      "end_effector_kinematics");

  if (param.joint_reg_enable) {
    bundle.joint_reg_q_ref = std::make_shared<ocs2::vector_t>(bundle.interface->getInitialState());
    AddStateRegularization(*bundle.interface, "joint_regularization", bundle.joint_reg_q_ref,
                           bundle.model_info.stateDim, param.joint_reg_q_weight, param.joint_reg_q_weight_scale);
  }

  if (param.default_q_reg_enable) {
    bundle.default_q_ref = std::make_shared<ocs2::vector_t>(bundle.interface->getInitialState());
    AddStateRegularization(*bundle.interface, "default_q_regularization", bundle.default_q_ref,
                           bundle.model_info.stateDim, param.default_q_reg_weight, param.default_reg_q_weight_scale);
  }

  return bundle;
}

}  // namespace kinematics
