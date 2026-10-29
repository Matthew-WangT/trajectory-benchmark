#pragma once

#include <chrono>

#include "arm/arm_interface.h"

namespace arm {

// Stand-in robot of the dry run: the measured joint position follows the command through a first order lag, a new
// feedback frame is published every millisecond of the monotonic clock. A zero time constant follows the command
// exactly.
class FakeArm : public ArmInterface {
 public:
  FakeArm(const JointVector& left_q0, const JointVector& right_q0, double time_constant_s);

  bool Connect() override { return true; }
  bool CheckRobotIni(const std::string& /*local_copy*/) override { return true; }
  bool EnterJointImpedance() override { return true; }
  bool Send(const JointVector& left_q, const JointVector& right_q) override;
  bool Read(Feedback& feedback) override;
  void Disable() override {}

 private:
  void Integrate(ArmFeedback& arm, const JointVector& cmd, double dt) const;

  double time_constant_s_;
  JointVector left_cmd_;
  JointVector right_cmd_;
  Feedback state_;
  std::chrono::steady_clock::time_point start_;
  int64_t last_frame_ = -1;
  int sent_frames_ = 0;
};

}  // namespace arm
