#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "arm/arm_interface.h"
#include "replay/safety.h"
#include "replay/timeline.h"

namespace replay {

// One IK evaluation of the control loop.
struct CmdRecord {
  double t = 0.0;
  Phase phase = Phase::kS0JointMove;
  int waypoint = -1;
  // TCP target fed to the controller: x, y, z, qx, qy, qz, qw per arm.
  Eigen::Matrix<double, 7, 1> left_target = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> right_target = Eigen::Matrix<double, 7, 1>::Zero();
  DualJoints q_cmd = DualJoints::Zero();
  bool evaluated = false;
  bool held = false;
  double policy_age = 0.0;
  int clamp_count = 0;
  DualJoints nominal_qd = DualJoints::Zero();
};

// One command sent to the robot.
struct SendRecord {
  double t = 0.0;
  Phase phase = Phase::kS0JointMove;
  DualJoints q = DualJoints::Zero();
};

// One new feedback frame of the robot.
struct FeedbackRecord {
  double t = 0.0;
  arm::Feedback feedback;
};

// Buffers the records of one run in memory, preallocated so that the control loop never allocates, and writes them
// as CSV once the run is over.
class Recorder {
 public:
  explicit Recorder(double max_duration_s);

  void Add(const CmdRecord& record);
  void Add(const SendRecord& record);
  void Add(const FeedbackRecord& record);
  bool overflowed() const { return overflowed_; }

  const std::vector<CmdRecord>& cmd() const { return cmd_; }

  // Writes raw_cmd.csv, raw_send.csv and raw_fb.csv into `dir`.
  void Write(const std::string& dir) const;

 private:
  template <typename T>
  void Push(std::vector<T>& records, const T& record);

  std::vector<CmdRecord> cmd_;
  std::vector<SendRecord> send_;
  std::vector<FeedbackRecord> feedback_;
  bool overflowed_ = false;
};

}  // namespace replay
