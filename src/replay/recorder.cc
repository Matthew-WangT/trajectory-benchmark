#include "replay/recorder.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

namespace replay {
namespace {

constexpr const char* kArms[] = {"l", "r"};

// Writes `n` values with the CSV separator in front of each.
template <typename Vector>
void WriteValues(std::FILE* file, const Vector& values) {
  for (int i = 0; i < values.size(); ++i) {
    std::fprintf(file, ",%.9g", values[i]);
  }
}

std::FILE* Open(const std::filesystem::path& path) {
  std::FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  return file;
}

}  // namespace

void Recorder::WriteJointHeader(std::FILE* file, const std::string& prefix, bool torso) const {
  for (int j = 1; torso && j <= torso_dof_; ++j) {
    std::fprintf(file, ",t_%s_%d", prefix.c_str(), j);
  }
  for (const char* arm : kArms) {
    for (int j = 1; j <= arm::kArmDoF; ++j) {
      std::fprintf(file, ",%s_%s_%d", arm, prefix.c_str(), j);
    }
  }
}

void Recorder::WriteJoints(std::FILE* file, const Joints& q) const {
  const int n = torso_dof_ + 2 * arm::kArmDoF;
  for (int i = 0; i < n; ++i) {
    std::fprintf(file, ",%.9g", i < q.size() ? q[i] : 0.0);
  }
}

Recorder::Recorder(double max_duration_s, int torso_dof) : torso_dof_(torso_dof) {
  // Margin for the scheduling jitter of the loop.
  const size_t ticks = static_cast<size_t>(std::ceil(max_duration_s * 1000.0 * 1.2)) + 1000;
  cmd_.reserve(ticks);
  send_.reserve(ticks);
  feedback_.reserve(ticks);
}

template <typename T>
void Recorder::Push(std::vector<T>& records, const T& record) {
  if (records.size() == records.capacity()) {
    overflowed_ = true;
    return;
  }
  records.push_back(record);
}

void Recorder::Add(const CmdRecord& record) { Push(cmd_, record); }
void Recorder::Add(const SendRecord& record) { Push(send_, record); }
void Recorder::Add(const FeedbackRecord& record) { Push(feedback_, record); }

void Recorder::Write(const std::string& dir) const {
  const std::filesystem::path out(dir);
  std::filesystem::create_directories(out);

  std::FILE* file = Open(out / "raw_cmd.csv");
  std::fprintf(file, "t_s,phase,waypoint,evaluated,held,policy_age_s,clamp_count");
  for (const char* arm : kArms) {
    for (const char* axis : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
      std::fprintf(file, ",%s_target_%s", arm, axis);
    }
  }
  WriteJointHeader(file, "q_cmd");
  WriteJointHeader(file, "nominal_qd");
  std::fprintf(file, "\n");
  for (const CmdRecord& r : cmd_) {
    // A policy age of +inf before the first policy is written as -1.
    std::fprintf(file, "%.6f,%s,%d,%d,%d,%.6f,%d", r.t, PhaseName(r.phase), r.waypoint, r.evaluated ? 1 : 0,
                 r.held ? 1 : 0, std::isfinite(r.policy_age) ? r.policy_age : -1.0, r.clamp_count);
    WriteValues(file, r.left_target);
    WriteValues(file, r.right_target);
    WriteJoints(file, r.q_cmd);
    WriteJoints(file, r.nominal_qd);
    std::fprintf(file, "\n");
  }
  std::fclose(file);

  file = Open(out / "raw_send.csv");
  std::fprintf(file, "t_s,phase");
  WriteJointHeader(file, "q_send");
  std::fprintf(file, "\n");
  for (const SendRecord& r : send_) {
    std::fprintf(file, "%.6f,%s", r.t, PhaseName(r.phase));
    WriteJoints(file, r.q);
    std::fprintf(file, "\n");
  }
  std::fclose(file);

  file = Open(out / "raw_fb.csv");
  std::fprintf(file, "t_s,serial");
  for (const char* field : {"q", "dq", "q_ext", "q_cmd_echo", "q_cmd_rx", "current", "tau", "temp", "est_force"}) {
    WriteJointHeader(file, field, /*torso=*/false);
  }
  std::fprintf(file,
               ",l_state,l_error,l_frame_miss,l_in_serial,l_syscyc_miss,r_state,r_error,r_frame_miss,r_in_serial,"
               "r_syscyc_miss\n");
  for (const FeedbackRecord& r : feedback_) {
    const arm::Feedback& f = r.feedback;
    std::fprintf(file, "%.6f,%lld", r.t, static_cast<long long>(f.serial));
    const arm::ArmFeedback* arms[] = {&f.left, &f.right};
    for (auto member : {&arm::ArmFeedback::q, &arm::ArmFeedback::dq, &arm::ArmFeedback::q_ext,
                        &arm::ArmFeedback::q_cmd_echo, &arm::ArmFeedback::q_cmd_received, &arm::ArmFeedback::current,
                        &arm::ArmFeedback::torque,
                        &arm::ArmFeedback::temperature, &arm::ArmFeedback::est_force}) {
      for (const arm::ArmFeedback* a : arms) {
        WriteValues(file, a->*member);
      }
    }
    for (const arm::ArmFeedback* a : arms) {
      std::fprintf(file, ",%d,%d,%d,%d,%d", a->state, a->error_code, a->frame_miss, a->in_frame_serial,
                   a->sys_cycle_miss);
    }
    std::fprintf(file, "\n");
  }
  std::fclose(file);
}

}  // namespace replay
