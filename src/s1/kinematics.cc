// Pinocchio raises the Boost MPL limits, it has to come before any Boost header.
#include <pinocchio/fwd.hpp>

#include "s1/kinematics.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace s1 {

struct Kinematics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<int> idx_q;
  std::vector<int> idx_v;
  Eigen::VectorXd q_full;
};

Kinematics::Kinematics(const std::string& urdf, const std::vector<std::string>& joint_names)
    : impl_(std::make_unique<Impl>()) {
  pinocchio::urdf::buildModel(urdf, impl_->model);
  impl_->data = pinocchio::Data(impl_->model);
  for (const std::string& name : joint_names) {
    if (!impl_->model.existJointName(name)) {
      throw std::runtime_error("joint " + name + " not in " + urdf);
    }
    const auto& joint = impl_->model.joints[impl_->model.getJointId(name)];
    impl_->idx_q.push_back(joint.idx_q());
    impl_->idx_v.push_back(joint.idx_v());
  }
  impl_->q_full = pinocchio::neutral(impl_->model);
}

Kinematics::~Kinematics() = default;

int Kinematics::FrameId(const std::string& frame) const {
  if (!impl_->model.existFrame(frame)) {
    throw std::runtime_error("frame " + frame + " not in the URDF");
  }
  return static_cast<int>(impl_->model.getFrameId(frame));
}

void Kinematics::Update(const replay::Joints& q) {
  for (size_t i = 0; i < impl_->idx_q.size(); ++i) {
    impl_->q_full[impl_->idx_q[i]] = q[static_cast<int>(i)];
  }
  pinocchio::framesForwardKinematics(impl_->model, impl_->data, impl_->q_full);
}

Eigen::Isometry3d Kinematics::Pose(int frame_id) const {
  const pinocchio::SE3& placement = impl_->data.oMf[frame_id];
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = placement.rotation();
  pose.translation() = placement.translation();
  return pose;
}

replay::Joints Kinematics::VelocityLimits() const {
  replay::Joints limits(static_cast<int>(impl_->idx_v.size()));
  for (size_t i = 0; i < impl_->idx_v.size(); ++i) {
    limits[static_cast<int>(i)] = impl_->model.velocityLimit[impl_->idx_v[i]];
  }
  return limits;
}

void Kinematics::PositionLimits(replay::Joints& lower, replay::Joints& upper) const {
  const int n = static_cast<int>(impl_->idx_q.size());
  lower.resize(n);
  upper.resize(n);
  for (int i = 0; i < n; ++i) {
    lower[i] = impl_->model.lowerPositionLimit[impl_->idx_q[i]];
    upper[i] = impl_->model.upperPositionLimit[impl_->idx_q[i]];
  }
}

}  // namespace s1
