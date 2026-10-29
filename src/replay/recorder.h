#pragma once

#include <cstdint>
#include <cstdio>
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
  Joints q_cmd;
  bool evaluated = false;
  bool held = false;
  double policy_age = 0.0;
  int clamp_count = 0;
  Joints nominal_qd;
};

// One command sent to the robot.
struct SendRecord {
  double t = 0.0;
  Phase phase = Phase::kS0JointMove;
  Joints q;
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
  // `torso_dof` torso joints lead the joint vectors, written as t_<field>_<i> columns before the arms.
  Recorder(double max_duration_s, int torso_dof);

  void Add(const CmdRecord& record);
  void Add(const SendRecord& record);
  void Add(const FeedbackRecord& record);
  bool overflowed() const { return overflowed_; }

  const std::vector<CmdRecord>& cmd() const { return cmd_; }

  // Writes raw_cmd.csv, raw_send.csv and raw_fb.csv into `dir`.
  void Write(const std::string& dir) const;

 private:
  // Without `torso` the columns of the two arms only, as in the arm feedback.
  void WriteJointHeader(std::FILE* file, const std::string& prefix, bool torso = true) const;
  // A joint vector left empty (no evaluation on the tick) is written as zeros.
  void WriteJoints(std::FILE* file, const Joints& q) const;

  template <typename T>
  void Push(std::vector<T>& records, const T& record);

  int torso_dof_;
  std::vector<CmdRecord> cmd_;
  std::vector<SendRecord> send_;
  std::vector<FeedbackRecord> feedback_;
  bool overflowed_ = false;
};

}  // namespace replay
