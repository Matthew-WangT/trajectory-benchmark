#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace arm {

constexpr int kArmDoF = 7;
using JointVector = Eigen::Matrix<double, kArmDoF, 1>;

// Joint impedance settings of one arm.
struct ArmGains {
  JointVector stiffness = (JointVector() << 22, 22, 22, 15, 8, 8, 8).finished();
  JointVector damping = JointVector::Constant(0.3);
};

// Joint impedance mode, the only control mode of the benchmark.
struct ControlConfig {
  ArmGains left;
  ArmGains right;
  int vel_ratio = 100;
  int acc_ratio = 100;
  // PD velocity feedforward of the controller. Requires JointPIDCtlType=1 in robot.ini and joint velocities below
  // 180 deg/s in the commands.
  bool enable_vel_feedforward = true;
  // Period of the command stream in ms, within [1, 20].
  int vel_feedforward_step_ms = 2;
};

// Tool written to the controller by SetTool.
struct ToolConfig {
  // Offset from the flange: X, Y, Z in mm and the A, B, C rotations in degrees, XYZ order.
  std::array<double, 6> kine{};
  // Mass in kg, the center of mass x, y, z in mm, then Ixx, Ixy, Ixz, Iyy, Iyz, Izz.
  std::array<double, 10> dyn{};
};

struct RobotConfig {
  std::string ip = "192.168.1.190";
  // CPU of the 1 ms network thread the SDK creates on connect, -1 leaves it to the OS.
  int sdk_cpu = -1;
  int log_switch = 0;
  ControlConfig control;
  ToolConfig left_tool;
  ToolConfig right_tool;
};

// Feedback of one arm, joint values in rad, rad/s and N·m.
struct ArmFeedback {
  JointVector q = JointVector::Zero();
  JointVector dq = JointVector::Zero();
  // Joint position of the output side encoder.
  JointVector q_ext = JointVector::Zero();
  // Joint command echoed by the controller.
  JointVector q_cmd_echo = JointVector::Zero();
  JointVector current = JointVector::Zero();
  JointVector torque = JointVector::Zero();
  JointVector temperature = JointVector::Zero();
  JointVector est_force = JointVector::Zero();
  int state = 0;
  int error_code = 0;
  int frame_miss = 0;
};

struct Feedback {
  int64_t serial = -1;
  // False when the controller has not published a new frame since the previous read.
  bool is_new = false;
  ArmFeedback left;
  ArmFeedback right;
};

// Dual-arm robot in the joint impedance mode.
class ArmInterface {
 public:
  virtual ~ArmInterface() = default;

  virtual bool Connect() = 0;
  // Fetches robot.ini of the controller to `local_copy`, then checks the settings the benchmark relies on.
  virtual bool CheckRobotIni(const std::string& local_copy) = 0;
  // Writes the tools and gains and switches both arms to the joint impedance mode.
  virtual bool EnterJointImpedance() = 0;
  virtual bool Send(const JointVector& left_q, const JointVector& right_q) = 0;
  virtual bool Read(Feedback& feedback) = 0;
  // Disables the servos, the brakes hold the arms.
  virtual void Disable() = 0;

  const std::string& last_error() const { return last_error_; }

 protected:
  std::string last_error_;
};

}  // namespace arm
