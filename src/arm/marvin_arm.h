#pragma once

#include <array>
#include <string>

#include "arm/arm_interface.h"

namespace arm {

// Marvin dual arms through the Tianji control SDK, ported from the MarvinDriver of the reference pipeline
// (mindon_robotics src/hardware/tianji/marvin_driver.cc) with only the joint impedance mode kept.
// Arm A is the left arm, arm B the right arm.
class MarvinArm : public ArmInterface {
 public:
  explicit MarvinArm(const RobotConfig& config);
  ~MarvinArm() override;

  bool Connect() override;
  bool CheckRobotIni(const std::string& local_copy) override;
  bool EnterJointImpedance() override;
  bool Send(const JointVector& left_q, const JointVector& right_q) override;
  bool Read(Feedback& feedback) override;
  void Disable() override;

 private:
  bool ConnectRobot(const unsigned char ip[4]);
  bool ApplyToolParams();
  // Sets the trajectory sending period of the PD velocity feedforward for both arms, |step_ms| < 1 disables it.
  bool SetVelFeedforward(int step_ms);
  bool Fail(const std::string& error);

  RobotConfig config_;
  bool connected_ = false;
  bool vel_feedforward_on_ = false;
  std::array<int, 2> last_frame_serial_{-1, -1};
};

}  // namespace arm
