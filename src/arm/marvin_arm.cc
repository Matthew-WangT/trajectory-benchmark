#include "arm/marvin_arm.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

#include <pthread.h>
#include <sched.h>

#include <glog/logging.h>

#include <FxRtCSDef.h>
#include <MarvinSDK.h>

#include "arm/robot_ini_check.h"

namespace arm {
namespace {

constexpr std::array<char, 2> kArmNames = {'A', 'B'};
// Trajectory sending period of the PD velocity feedforward is limited to 20ms by the controller.
constexpr int kMaxVelFeedforwardStepMs = 20;
// Response timeout and the settle time after each step of the vendor PD pipeline.
constexpr long kSetTimeoutMs = 100;
constexpr int kGainSettleMs = 500;
constexpr int kStateSettleMs = 1000;
// Number of state buffer reads while waiting for both arms to become stationary.
constexpr int kStationaryRetries = 5;
constexpr char kRemoteRobotIni[] = "/home/FUSION/Config/cfg/robot.ini";
constexpr double kDegreesPerRadian = 180.0 / M_PI;
constexpr double kRadiansPerDegree = M_PI / 180.0;

bool ParseIpv4(const std::string& ip, unsigned char octets[4]) {
  int a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4) {
    return false;
  }
  if (a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255 || d < 0 || d > 255) {
    return false;
  }
  octets[0] = static_cast<unsigned char>(a);
  octets[1] = static_cast<unsigned char>(b);
  octets[2] = static_cast<unsigned char>(c);
  octets[3] = static_cast<unsigned char>(d);
  return true;
}

bool ArmInError(const StateCtr& state) { return state.m_CurState == ARM_STATE_ERROR || state.m_ERRCode != 0; }

// Meanings from ArmError in FxRtCSDef.h.
std::string ArmErrorToString(FX_INT32 err_code) {
  switch (err_code) {
    case 0:
      return "ok";
    case ARM_ERR_BusPhysicAbnoraml:
      return "总线拓扑异常";
    case ARM_ERR_ServoError:
      return "伺服故障";
    case ARM_ERR_InvalidPVT:
      return "PVT异常";
    case ARM_ERR_RequestPositionMode:
      return "请求进位置失败";
    case ARM_ERR_PositionModeOK:
      return "进位置失败";
    case ARM_ERR_RequestSensorMode:
      return "请求进扭矩失败";
    case ARM_ERR_SensorModeOK:
      return "进扭矩失败";
    case ARM_ERR_RequestEnableServo:
      return "请求上伺服失败";
    case ARM_ERR_EnableServoOK:
      return "上伺服失败";
    case ARM_ERR_RequestDisableServo:
      return "请求下伺服失败";
    case ARM_ERR_DisableServoOK:
      return "下伺服失败";
    case ARM_ERR_InvalidSubState:
      return "内部错";
    case ARM_ERR_Emcy:
      return "急停";
    case ARM_DYNA_FLOAT_NO_GYRO:
      return "配置文件选择了浮动基座选项，但是UMI设置在配置文件未开";
    case ARM_ERR_PdoAbnormal:
      return "PDO数据异常";
    case ARM_ERR_AxisIsVirtual:
      return "手臂为虚拟轴";
    default:
      return "unknown";
  }
}

std::string FormatArmError(char arm, const StateCtr& state) {
  return std::string(1, arm) + " state=" + std::to_string(state.m_CurState) + " err=" +
         std::to_string(state.m_ERRCode) + "(" + ArmErrorToString(state.m_ERRCode) + ")";
}

// The controller rejects a switch to the torque state while an arm is still moving.
bool ArmsStationary() {
  DCSS dcss;
  for (int i = 0; i < kStationaryRetries; ++i) {
    if (!OnGetBuf(&dcss)) {
      return false;
    }
    if (dcss.m_Out[0].m_LowSpdFlag == 1 && dcss.m_Out[1].m_LowSpdFlag == 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

JointVector FromFloats(const FX_FLOAT* values, double scale) {
  return Eigen::Map<const Eigen::Matrix<float, kArmDoF, 1>>(values).cast<double>() * scale;
}

void ParseArm(const RT_OUT& out, const RT_IN& in, const StateCtr& state, ArmFeedback& arm) {
  arm.q = FromFloats(out.m_FB_Joint_Pos, kRadiansPerDegree);
  arm.dq = FromFloats(out.m_FB_Joint_Vel, kRadiansPerDegree);
  arm.q_ext = FromFloats(out.m_FB_Joint_PosE, kRadiansPerDegree);
  arm.q_cmd_echo = FromFloats(out.m_FB_Joint_Cmd, kRadiansPerDegree);
  arm.current = FromFloats(out.m_FB_Joint_CToq, 1.0);
  arm.torque = FromFloats(out.m_FB_Joint_SToq, 1.0);
  arm.temperature = FromFloats(out.m_FB_Joint_Them, 1.0);
  arm.est_force = FromFloats(out.m_EST_Joint_Force, 1.0);
  arm.state = state.m_CurState;
  arm.error_code = state.m_ERRCode;
  arm.frame_miss = in.m_FrameMissCnt;
}

}  // namespace

MarvinArm::MarvinArm(const RobotConfig& config) : config_(config) {}

MarvinArm::~MarvinArm() {
  if (connected_) {
    Disable();
  }
}

bool MarvinArm::Fail(const std::string& error) {
  last_error_ = error;
  LOG(ERROR) << error;
  return false;
}

bool MarvinArm::Connect() {
  unsigned char ip[4];
  if (!ParseIpv4(config_.ip, ip)) {
    return Fail("Invalid robot IP: " + config_.ip);
  }
  if (config_.sdk_cpu < 0) {
    return ConnectRobot(ip);
  }

  // The SDK creates its 1 ms network thread in OnLinkTo(). Make it inherit the requested CPU, then restore the
  // affinity of this caller.
  cpu_set_t original_affinity;
  CPU_ZERO(&original_affinity);
  if (pthread_getaffinity_np(pthread_self(), sizeof(original_affinity), &original_affinity) != 0 ||
      config_.sdk_cpu >= CPU_SETSIZE || !CPU_ISSET(config_.sdk_cpu, &original_affinity)) {
    return Fail("Invalid or unavailable SDK CPU: " + std::to_string(config_.sdk_cpu));
  }
  cpu_set_t sdk_affinity;
  CPU_ZERO(&sdk_affinity);
  CPU_SET(config_.sdk_cpu, &sdk_affinity);
  const int rc = pthread_setaffinity_np(pthread_self(), sizeof(sdk_affinity), &sdk_affinity);
  if (rc != 0) {
    return Fail("Failed to select SDK CPU " + std::to_string(config_.sdk_cpu) + ": " + std::strerror(rc));
  }
  const bool connected = ConnectRobot(ip);
  pthread_setaffinity_np(pthread_self(), sizeof(original_affinity), &original_affinity);
  return connected;
}

bool MarvinArm::ConnectRobot(const unsigned char ip[4]) {
  if (!OnLinkTo(ip[0], ip[1], ip[2], ip[3])) {
    return Fail("Failed to link to robot at " + config_.ip + " (port may be occupied)");
  }
  connected_ = true;

  if (!CheckArmError()) {
    DCSS dcss{};
    OnGetBuf(&dcss);
    return Fail("Arms in error state after link. " + FormatArmError('A', dcss.m_State[0]) + " " +
                FormatArmError('B', dcss.m_State[1]));
  }
  if (!CheckServoError()) {
    return Fail("Servos in error state after link to " + config_.ip);
  }

  // Same as MarvinSDK::Connect: wait briefly for a non-zero state frame serial.
  DCSS dcss{};
  int last_frame_serial = 0;
  bool frame_ok = false;
  for (int i = 0; i < 5; ++i) {
    if (!OnGetBuf(&dcss)) {
      return Fail("Failed to get state buffer while verifying connection");
    }
    const int frame_serial = dcss.m_Out[0].m_OutFrameSerial;
    if (frame_serial != 0 && frame_serial != last_frame_serial) {
      frame_ok = true;
      break;
    }
    last_frame_serial = frame_serial;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!frame_ok) {
    return Fail("Robot connection failed at " + config_.ip +
                ": no updating state frame (check ctrlsys version > 100341)");
  }
  if (ArmInError(dcss.m_State[0]) || ArmInError(dcss.m_State[1])) {
    return Fail("Arm error after connect. " + FormatArmError('A', dcss.m_State[0]) + " " +
                FormatArmError('B', dcss.m_State[1]));
  }

  LogSwitch(config_.log_switch == 1 ? 1 : 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  LOG(INFO) << "Connected to robot at " << config_.ip;
  return true;
}

bool MarvinArm::CheckRobotIni(const std::string& local_copy) {
  std::string local = local_copy;
  std::string remote = kRemoteRobotIni;
  if (!OnRecvFile(local.data(), remote.data())) {
    return Fail(std::string("Failed to fetch ") + kRemoteRobotIni + " from the controller");
  }
  std::ifstream file(local_copy);
  std::stringstream text;
  text << file.rdbuf();
  std::string error;
  if (!CheckRobotIniText(text.str(), error)) {
    return Fail("robot.ini of the controller (copy in " + local_copy + ") does not match the benchmark: " + error +
                "fix it with the vendor host software and restart the controller");
  }
  LOG(INFO) << "robot.ini checked, PosDTType=1 and JointPIDCtlType=1 on both arms";
  return true;
}

bool MarvinArm::ApplyToolParams() {
  // SetTool stages the parameters, sends them and reads them back from the controller to verify.
  const std::array<const ToolConfig*, 2> tools = {&config_.left_tool, &config_.right_tool};
  for (size_t i = 0; i < kArmNames.size(); ++i) {
    std::array<double, 6> kine = tools[i]->kine;
    std::array<double, 10> dyn = tools[i]->dyn;
    if (!SetTool(kArmNames[i], kine.data(), dyn.data())) {
      return Fail(std::string("Failed to set the tool parameters of arm ") + kArmNames[i]);
    }
  }
  DCSS dcss;
  if (!OnGetBuf(&dcss)) {
    return Fail("Failed to get state buffer while reading back the tool parameters");
  }
  for (size_t i = 0; i < kArmNames.size(); ++i) {
    LOG(INFO) << "Arm " << kArmNames[i] << " tool kine "
              << Eigen::Map<const Eigen::VectorXf>(dcss.m_In[i].m_ToolKine, 6).transpose() << ", dyn "
              << Eigen::Map<const Eigen::VectorXf>(dcss.m_In[i].m_ToolDyn, 10).transpose();
  }
  return true;
}

bool MarvinArm::EnterJointImpedance() {
  const ControlConfig& control = config_.control;
  // The controller drops the tool parameters on a restart, so they are rewritten before each mode switch.
  if (!ApplyToolParams()) {
    return false;
  }
  if (!CheckArmError() || !CheckServoError()) {
    return Fail("Arms or servos are in an error state, refusing to switch to the joint impedance mode");
  }
  if (!ArmsStationary()) {
    return Fail("Arms are not stationary, refusing to switch to the joint impedance mode");
  }

  // 1. Velocity and acceleration ratios plus the impedance gains.
  std::array<double, kArmDoF> left_k, left_d, right_k, right_d;
  Eigen::Map<JointVector>(left_k.data()) = control.left.stiffness;
  Eigen::Map<JointVector>(left_d.data()) = control.left.damping;
  Eigen::Map<JointVector>(right_k.data()) = control.right.stiffness;
  Eigen::Map<JointVector>(right_d.data()) = control.right.damping;
  if (!OnClearSet()) {
    return Fail("OnClearSet failed before setting the joint impedance gains");
  }
  if (!OnSetJointLmt_A(control.vel_ratio, control.acc_ratio) || !OnSetJointKD_A(left_k.data(), left_d.data()) ||
      !OnSetJointLmt_B(control.vel_ratio, control.acc_ratio) || !OnSetJointKD_B(right_k.data(), right_d.data())) {
    return Fail("Failed to stage the joint impedance gains");
  }
  if (OnSetSendWaitResponse(kSetTimeoutMs) < 0) {
    return Fail("Failed to send the joint impedance gains, timeout");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(kGainSettleMs));

  // 2. PD velocity feedforward, only meaningful while the gains are already in place.
  if (!SetVelFeedforward(control.enable_vel_feedforward ? control.vel_feedforward_step_ms : 0)) {
    return false;
  }

  // 3. Torque state with the joint impedance type.
  if (!OnClearSet()) {
    return Fail("OnClearSet failed before switching to the joint impedance state");
  }
  if (!OnSetTargetState_A(ARM_STATE_TORQ) || !OnSetImpType_A(ARM_IMP_JOINT) || !OnSetTargetState_B(ARM_STATE_TORQ) ||
      !OnSetImpType_B(ARM_IMP_JOINT)) {
    return Fail("Failed to stage the joint impedance state");
  }
  if (OnSetSendWaitResponse(kSetTimeoutMs) < 0) {
    return Fail("Failed to send the joint impedance state, timeout");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(kStateSettleMs));

  LOG(INFO) << "Entered the joint impedance mode. left K " << control.left.stiffness.transpose() << " D "
            << control.left.damping.transpose() << ", right K " << control.right.stiffness.transpose() << " D "
            << control.right.damping.transpose();
  return true;
}

bool MarvinArm::SetVelFeedforward(int step_ms) {
  if (step_ms >= 1 && (config_.control.vel_ratio < 100 || config_.control.acc_ratio < 100)) {
    LOG(WARNING) << "Velocity feedforward expects a vel_ratio and an acc_ratio of 100 to not limit the trajectory";
  }
  if (step_ms < 0 || step_ms > kMaxVelFeedforwardStepMs) {
    return Fail("Velocity feedforward step " + std::to_string(step_ms) + "ms is out of the range [0, 20]ms");
  }
  // FX_OnSetVelEstStep only stages bytes in the SDK send buffer, so it needs the OnClearSet / OnSetSend pair around it.
  // Without the send the next OnClearSet of the command loop drops the setting, silently leaving the feedforward off.
  if (!OnClearSet()) {
    return Fail("OnClearSet failed before setting the velocity feedforward");
  }
  for (char arm : kArmNames) {
    if (!FX_OnSetVelEstStep(arm, step_ms)) {
      return Fail("Failed to stage the velocity feedforward step for arm " + std::string(1, arm));
    }
  }
  if (OnSetSendWaitResponse(kSetTimeoutMs) < 0) {
    return Fail("Failed to send the velocity feedforward step, timeout");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(kGainSettleMs));
  vel_feedforward_on_ = step_ms >= 1;
  LOG(INFO) << (vel_feedforward_on_ ? "Enabled" : "Disabled") << " velocity feedforward with step " << step_ms << "ms";
  return true;
}

bool MarvinArm::Send(const JointVector& left_q, const JointVector& right_q) {
  if (!left_q.allFinite() || !right_q.allFinite()) {
    return Fail("Position command contains NaN or Inf");
  }
  std::array<double, kArmDoF> left_deg, right_deg;
  Eigen::Map<JointVector>(left_deg.data()) = left_q * kDegreesPerRadian;
  Eigen::Map<JointVector>(right_deg.data()) = right_q * kDegreesPerRadian;
  if (!OnClearSet()) {
    return Fail("OnClearSet failed");
  }
  if (!OnSetJointCmdPos_A(left_deg.data()) || !OnSetJointCmdPos_B(right_deg.data())) {
    return Fail("OnSetJointCmdPos failed");
  }
  if (!OnSetSend()) {
    return Fail("OnSetSend failed");
  }
  return true;
}

bool MarvinArm::Read(Feedback& feedback) {
  DCSS dcss;
  if (!OnGetBuf(&dcss)) {
    return Fail("Failed to get state buffer");
  }
  const int frame_a = dcss.m_Out[0].m_OutFrameSerial;
  const int frame_b = dcss.m_Out[1].m_OutFrameSerial;
  feedback.is_new = frame_a != last_frame_serial_[0] || frame_b != last_frame_serial_[1];
  last_frame_serial_ = {frame_a, frame_b};
  feedback.serial = frame_a;
  ParseArm(dcss.m_Out[0], dcss.m_In[0], dcss.m_State[0], feedback.left);
  ParseArm(dcss.m_Out[1], dcss.m_In[1], dcss.m_State[1], feedback.right);
  if (ArmInError(dcss.m_State[0]) || ArmInError(dcss.m_State[1])) {
    return Fail("Arm error. " + FormatArmError('A', dcss.m_State[0]) + " " + FormatArmError('B', dcss.m_State[1]));
  }
  return true;
}

void MarvinArm::Disable() {
  if (!connected_) {
    return;
  }
  // Servos first so the brakes engage without delay, the controller settings are restored afterwards.
  for (char arm : kArmNames) {
    if (!::Disable(arm)) {
      LOG(ERROR) << "Failed to disable arm " << arm;
    }
  }
  if (vel_feedforward_on_ && !SetVelFeedforward(0)) {
    LOG(ERROR) << "Failed to disable velocity feedforward";
  }
  OnRelease();
  connected_ = false;
  LOG(INFO) << "Arms disabled and the robot released";
}

}  // namespace arm
