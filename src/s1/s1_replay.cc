#include "s1/s1_replay.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include "replay/joints.h"
#include "replay/timeline.h"
#include "replay/trajectory.h"
#include "s1/bridge_protocol.h"
#include "s1/kinematics.h"
#include "s1/s1_config.h"
#include "s1/udp_bridge.h"

namespace s1 {
namespace {

using replay::Joints;
using replay::Phase;
using Pose7 = Eigen::Matrix<double, kPose, 1>;

std::atomic<bool> g_interrupted{false};

void OnSignal(int /*signal*/) { g_interrupted = true; }

Pose7 PoseVector(const Eigen::Isometry3d& pose) {
  Pose7 v;
  v << pose.translation(), replay::QuatXyzw(pose);
  return v;
}

Eigen::Isometry3d PoseOf(const double* v) {
  return replay::MakePose(Eigen::Vector3d(v[0], v[1], v[2]), Eigen::Vector4d(v[3], v[4], v[5], v[6]));
}

void CopyPose(const Eigen::Isometry3d& pose, double* out) {
  const Pose7 v = PoseVector(pose);
  std::copy(v.data(), v.data() + kPose, out);
}

Joints JointsOf(const double* v) { return Eigen::Map<const Eigen::VectorXd>(v, kJoints); }

Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double s) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::Quaterniond(a.linear()).slerp(s, Eigen::Quaterniond(b.linear())).toRotationMatrix();
  pose.translation() = (1.0 - s) * a.translation() + s * b.translation();
  return pose;
}

// One SDK period of a run.
struct Record {
  double t = 0.0;
  Phase phase = Phase::kS0JointMove;
  int waypoint = -1;
  CommandMode mode = CommandMode::kIdle;
  // TCP targets in benchmark_base, and the SDK end-effector commands in the chassis frame.
  Pose7 left_target = Pose7::Zero();
  Pose7 right_target = Pose7::Zero();
  Pose7 left_sdk = Pose7::Zero();
  Pose7 right_sdk = Pose7::Zero();
  // Joint command of the joint mode (S0), zeros otherwise.
  Joints q_send = Joints::Zero(kJoints);
  Feedback feedback{};
};

struct RunResult {
  bool ok = true;
  std::string reason;
  std::vector<Record> records;
  Joints q_start;
  double s1_start = 0.0, s2_start = 0.0, s3_start = 0.0, s4_start = 0.0, end = 0.0;
  bool stop_after_s0 = false;
};

class S1Replay {
 public:
  S1Replay(const S1Config& config, const replay::CaseTrajectory& traj, const Options& options)
      : config_(config),
        traj_(traj),
        options_(options),
        kin_(config.model.urdf, config.model.JointNames()),
        left_tcp_(kin_.FrameId(config.model.left_ee_frame)),
        right_tcp_(kin_.FrameId(config.model.right_ee_frame)),
        left_sdk_(kin_.FrameId(config.sdk_left_ee)),
        right_sdk_(kin_.FrameId(config.sdk_right_ee)),
        dt_(1.0 / config.freq_hz) {
    kin_.PositionLimits(lower_, upper_);
    vel_limit_ = kin_.VelocityLimits();
    const int torso_base = kin_.FrameId("astribot_torso_base");
    kin_.Update(config.model.startup.QPd());
    // Fixed transforms: benchmark_base in the chassis frame, and the SDK end effector of each arm in its TCP frame.
    Eigen::Isometry3d chassis_torso_base = Eigen::Isometry3d::Identity();
    chassis_torso_base.translation() = config.torso_base_in_chassis;
    chassis_bb_ = chassis_torso_base * kin_.Pose(torso_base).inverse();
    left_tcp_sdk_ = kin_.Pose(left_tcp_).inverse() * kin_.Pose(left_sdk_);
    right_tcp_sdk_ = kin_.Pose(right_tcp_).inverse() * kin_.Pose(right_sdk_);
    left_fk_pd_ = kin_.Pose(left_tcp_);
    right_fk_pd_ = kin_.Pose(right_tcp_);
  }

  // Waits for the sidecar, answers idle, then checks that the SDK end-effector poses agree with FK of the measured
  // joints (frames and torso_base_in_chassis) and that the robot is near q_pd.
  bool Connect(UdpBridge& bridge, std::string& error) {
    LOG(INFO) << "Waiting up to " << config_.connect_timeout_s
              << " s for scripts/s1_bridge.py on 127.0.0.1:" << config_.port;
    if (!bridge.Receive(config_.connect_timeout_s, feedback_)) {
      error = "no feedback of the sidecar: " + bridge.last_error();
      return false;
    }
    fake_ = feedback_.flags & kFlagFake;
    Reply(bridge, Idle());
    const Joints q = JointsOf(feedback_.q);
    kin_.Update(q);
    const double left_mismatch =
        ((chassis_bb_ * kin_.Pose(left_sdk_)).translation() - PoseOf(feedback_.ee_left).translation()).norm();
    const double right_mismatch =
        ((chassis_bb_ * kin_.Pose(right_sdk_)).translation() - PoseOf(feedback_.ee_right).translation()).norm();
    LOG(INFO) << "Sidecar " << (fake_ ? "(fake robot)" : "(SDK)") << " connected, measured joints " << q.transpose()
              << "; SDK end effector vs FK: left " << left_mismatch * 1e3 << " mm, right " << right_mismatch * 1e3
              << " mm";
    if (std::max(left_mismatch, right_mismatch) > config_.safety.max_frame_mismatch_m) {
      error = "the SDK end-effector poses are " + std::to_string(std::max(left_mismatch, right_mismatch) * 1e3) +
              " mm off FK of the measured joints, check sdk_frame in the config";
      return false;
    }
    const double deviation = (q - config_.model.startup.QPd()).cwiseAbs().maxCoeff();
    if (deviation > config_.safety.max_initial_deviation_rad) {
      error = "joints are " + std::to_string(deviation) + " rad away from q_pd, move the robot closer first";
      return false;
    }
    return true;
  }

  RunResult Run(UdpBridge& bridge) {
    RunResult result;
    result.q_start = JointsOf(feedback_.q);
    replay::Timeline timeline(config_.model.startup, traj_, result.q_start, left_fk_pd_, right_fk_pd_);
    result.stop_after_s0 = options_.stop_after == "S0";
    const double end = result.stop_after_s0          ? timeline.s1_start() + config_.model.startup.hold_s
                       : options_.stop_after == "S2" ? timeline.s3_start()
                                                     : timeline.end();
    result.records.reserve(static_cast<size_t>(end / dt_) + 100);
    LOG(INFO) << "Run: S1 at " << timeline.s1_start() << " s, S3 at " << timeline.s3_start() << " s, end at " << end
              << " s";

    bool s1_anchored = false;
    Joints prev_q_desired = JointsOf(feedback_.q_desired);
    // The run clock counts SDK periods of the sidecar from the first feedback of the run, a frame lost on the way
    // does not shift it.
    std::optional<uint32_t> seq0;
    while (true) {
      if (!bridge.Receive(config_.safety.link.enable ? config_.safety.link.value : 1.0, feedback_)) {
        result = Fail(result, "link: " + bridge.last_error());
        break;
      }
      if (feedback_.flags & kFlagStopped) {
        result =
            Fail(result, "the sidecar stopped (" + std::to_string(feedback_.consecutive_miss) + " consecutive misses)");
        break;
      }
      if (!seq0) {
        seq0 = feedback_.seq;
      }
      const double t = static_cast<double>(feedback_.seq - *seq0) * dt_;
      if (g_interrupted) {
        Reply(bridge, Stop());
        result = Fail(result, "interrupted");
        break;
      }
      if (t >= end) {
        Reply(bridge, Idle());
        break;
      }

      Record record;
      record.t = t;
      record.phase = result.stop_after_s0 ? Phase::kS0JointMove : timeline.PhaseAt(t);
      record.feedback = feedback_;
      const Joints q = JointsOf(feedback_.q);
      const Joints q_desired = JointsOf(feedback_.q_desired);
      kin_.Update(q);
      const Eigen::Isometry3d left_meas = kin_.Pose(left_tcp_);
      const Eigen::Isometry3d right_meas = kin_.Pose(right_tcp_);

      Command command = Idle();
      std::optional<Eigen::Isometry3d> left_target, right_target;
      if (record.phase == Phase::kS0JointMove) {
        command.mode = static_cast<uint16_t>(CommandMode::kJoint);
        record.q_send = timeline.JointCommand(t);
        std::copy(record.q_send.data(), record.q_send.data() + kJoints, command.q);
      } else {
        if (!s1_anchored) {
          // The TCP move starts at the measured TCPs, not FK(q_pd), so that the first cartesian command does not step.
          timeline = replay::Timeline(config_.model.startup, traj_, result.q_start, left_meas, right_meas);
          s1_anchored = true;
        }
        record.waypoint = timeline.LatestWaypoint(t);
        Target(timeline, t, record.waypoint, left_target, right_target);
        command.mode = static_cast<uint16_t>(CommandMode::kCartesian);
        const Eigen::Isometry3d left_sdk = chassis_bb_ * *left_target * left_tcp_sdk_;
        const Eigen::Isometry3d right_sdk = chassis_bb_ * *right_target * right_tcp_sdk_;
        CopyPose(left_sdk, command.pose_left);
        CopyPose(right_sdk, command.pose_right);
        record.left_target = PoseVector(*left_target);
        record.right_target = PoseVector(*right_target);
        record.left_sdk = PoseVector(left_sdk);
        record.right_sdk = PoseVector(right_sdk);
      }
      record.mode = static_cast<CommandMode>(command.mode);

      const std::optional<std::string> violation =
          Check(record.phase, q, q_desired, prev_q_desired, JointsOf(feedback_.dq), left_meas, right_meas, left_target,
                right_target);
      prev_q_desired = q_desired;
      result.records.push_back(record);
      if (violation) {
        Reply(bridge, Stop());
        result = Fail(result, *violation);
        break;
      }
      if (!Reply(bridge, command)) {
        result = Fail(result, "send: " + bridge.last_error());
        break;
      }
    }
    result.s1_start = timeline.s1_start();
    result.s2_start = timeline.s2_start();
    result.s3_start = timeline.s3_start();
    result.s4_start = timeline.s4_start();
    result.end = timeline.end();
    return result;
  }

  // Answers the next feedback with a stop, the sidecar stops commanding and exits.
  void Finish(UdpBridge& bridge) {
    if (bridge.Receive(0.5, feedback_)) {
      Reply(bridge, Stop());
    }
  }

  bool fake() const { return fake_; }
  const replay::Joints& q_pd() const { return q_pd_; }

 private:
  Command Idle() const {
    Command command{};
    command.magic = kCommandMagic;
    command.version = kVersion;
    command.mode = static_cast<uint16_t>(CommandMode::kIdle);
    return command;
  }

  Command Stop() const {
    Command command = Idle();
    command.mode = static_cast<uint16_t>(CommandMode::kStop);
    return command;
  }

  bool Reply(UdpBridge& bridge, Command command) {
    command.seq = feedback_.seq;
    return bridge.Send(command);
  }

  RunResult Fail(RunResult result, const std::string& reason) {
    result.ok = false;
    result.reason = reason;
    LOG(ERROR) << "Abort: " << reason;
    return result;
  }

  // TCP targets at run time `t`: the waypoints around t interpolated (position linearly, orientation by slerp), the
  // next waypoint being at most one trajectory sample ahead; across a phase change and after the last waypoint the
  // newest one is held.
  static void Target(const replay::Timeline& timeline, double t, int k, std::optional<Eigen::Isometry3d>& left,
                     std::optional<Eigen::Isometry3d>& right) {
    const std::vector<replay::Waypoint>& w = timeline.waypoints();
    k = std::max(k, 0);
    if (k + 1 < static_cast<int>(w.size()) && w[k + 1].phase == w[k].phase && t >= w[k].t) {
      const double s = std::clamp((t - w[k].t) / (w[k + 1].t - w[k].t), 0.0, 1.0);
      left = Interpolate(w[k].left, w[k + 1].left, s);
      right = Interpolate(w[k].right, w[k + 1].right, s);
    } else {
      left = w[k].left;
      right = w[k].right;
    }
  }

  std::optional<std::string> Check(Phase phase, const Joints& q, const Joints& q_desired, const Joints& prev_q_desired,
                                   const Joints& dq, const Eigen::Isometry3d& left_meas,
                                   const Eigen::Isometry3d& right_meas,
                                   const std::optional<Eigen::Isometry3d>& left_target,
                                   const std::optional<Eigen::Isometry3d>& right_target) const {
    const SafetyConfig& s = config_.safety;
    const std::vector<std::string> names = config_.model.JointNames();
    std::ostringstream reason;
    if (feedback_.consecutive_miss > static_cast<uint32_t>(s.max_consecutive_miss) && s.link.enable) {
      reason << "the sidecar repeated the command for " << feedback_.consecutive_miss << " periods";
      return reason.str();
    }
    if (s.joint_limit.enable) {
      for (int i = 0; i < kJoints; ++i) {
        for (const auto& [label, value] :
             {std::pair<const char*, double>{"measured", q[i]}, {"desired", q_desired[i]}}) {
          if (value < lower_[i] + s.joint_limit.value || value > upper_[i] - s.joint_limit.value) {
            reason << names[i] << " " << label << " " << value << " rad beyond the limits [" << lower_[i] << ", "
                   << upper_[i] << "] less " << s.joint_limit.value;
            return reason.str();
          }
        }
      }
    }
    if (s.joint_velocity.enable) {
      for (int i = 0; i < kJoints; ++i) {
        const double limit = s.joint_velocity.value * vel_limit_[i];
        const double step = std::abs(q_desired[i] - prev_q_desired[i]) / dt_;
        if (std::abs(dq[i]) > limit || step > limit) {
          reason << names[i] << " velocity " << std::max(std::abs(dq[i]), step) << " rad/s (measured "
                 << std::abs(dq[i]) << ", desired step " << step << ") above " << limit;
          return reason.str();
        }
      }
    }
    if (s.tcp_error.enable && left_target && phase != Phase::kS4Hold) {
      const double left_err = (left_meas.translation() - left_target->translation()).norm();
      const double right_err = (right_meas.translation() - right_target->translation()).norm();
      if (std::max(left_err, right_err) > s.tcp_error.value) {
        reason << "TCP error left " << left_err * 1e3 << " mm, right " << right_err * 1e3 << " mm above "
               << s.tcp_error.value * 1e3 << " mm";
        return reason.str();
      }
    }
    if (s.arm_distance.enable) {
      const double distance = (left_meas.translation() - right_meas.translation()).norm();
      if (distance < s.arm_distance.value) {
        reason << "TCPs " << distance * 1e3 << " mm apart, below " << s.arm_distance.value * 1e3 << " mm";
        return reason.str();
      }
    }
    return std::nullopt;
  }

  const S1Config& config_;
  const replay::CaseTrajectory& traj_;
  const Options& options_;
  Kinematics kin_;
  int left_tcp_, right_tcp_, left_sdk_, right_sdk_;
  double dt_;
  Joints lower_, upper_, vel_limit_;
  Joints q_pd_ = config_.model.startup.QPd();
  Eigen::Isometry3d chassis_bb_, left_tcp_sdk_, right_tcp_sdk_, left_fk_pd_, right_fk_pd_;
  Feedback feedback_{};
  bool fake_ = false;
};

// Joint column names of a field, torso first: t_<field>_<i>, l_<field>_<i>, r_<field>_<i>.
void JointHeader(std::FILE* file, const char* field) {
  for (const auto& [prefix, n] : {std::pair<const char*, int>{"t", 4}, {"l", 7}, {"r", 7}}) {
    for (int j = 1; j <= n; ++j) {
      std::fprintf(file, ",%s_%s_%d", prefix, field, j);
    }
  }
}

void PoseHeader(std::FILE* file, const char* prefix, const char* field) {
  for (const char* axis : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
    std::fprintf(file, ",%s_%s_%s", prefix, field, axis);
  }
}

template <typename Values>
void WriteValues(std::FILE* file, const Values& values, int n) {
  for (int i = 0; i < n; ++i) {
    std::fprintf(file, ",%.9g", values[i]);
  }
}

const char* ModeName(CommandMode mode) {
  switch (mode) {
    case CommandMode::kJoint:
      return "joint";
    case CommandMode::kCartesian:
      return "cartesian";
    case CommandMode::kStop:
      return "stop";
    case CommandMode::kIdle:
      return "idle";
  }
  return "unknown";
}

// raw_cmd.csv: the target and the command of each period, q_cmd being the desired joints of the SDK (the joint
// command in S0, the WBC output from S1 on). raw_fb.csv: the feedback of each period.
void WriteRecords(const std::filesystem::path& dir, const std::vector<Record>& records) {
  std::filesystem::create_directories(dir);
  std::FILE* file = std::fopen((dir / "raw_cmd.csv").c_str(), "w");
  CHECK(file) << "Cannot write " << dir / "raw_cmd.csv";
  std::fprintf(file, "t_s,phase,waypoint,mode");
  PoseHeader(file, "l", "target");
  PoseHeader(file, "r", "target");
  PoseHeader(file, "l", "sdk_cmd");
  PoseHeader(file, "r", "sdk_cmd");
  JointHeader(file, "q_cmd");
  JointHeader(file, "q_send");
  std::fprintf(file, "\n");
  for (const Record& r : records) {
    std::fprintf(file, "%.6f,%s,%d,%s", r.t, replay::PhaseName(r.phase), r.waypoint, ModeName(r.mode));
    for (const Pose7* pose : {&r.left_target, &r.right_target, &r.left_sdk, &r.right_sdk}) {
      WriteValues(file, *pose, kPose);
    }
    WriteValues(file, r.feedback.q_desired, kJoints);
    WriteValues(file, r.q_send, kJoints);
    std::fprintf(file, "\n");
  }
  std::fclose(file);

  file = std::fopen((dir / "raw_fb.csv").c_str(), "w");
  CHECK(file) << "Cannot write " << dir / "raw_fb.csv";
  std::fprintf(file, "t_s,seq,t_mono_s,read_ms,command_ms,consecutive_miss,total_miss");
  for (const char* field : {"q", "dq", "tau", "q_desired"}) {
    JointHeader(file, field);
  }
  PoseHeader(file, "l", "sdk_ee");
  PoseHeader(file, "r", "sdk_ee");
  std::fprintf(file, "\n");
  for (const Record& r : records) {
    const Feedback& f = r.feedback;
    std::fprintf(file, "%.6f,%u,%.6f,%.3f,%.3f,%u,%u", r.t, f.seq, f.t_mono_s, f.read_s * 1e3, f.command_s * 1e3,
                 f.consecutive_miss, f.total_miss);
    WriteValues(file, f.q, kJoints);
    WriteValues(file, f.dq, kJoints);
    WriteValues(file, f.tau, kJoints);
    WriteValues(file, f.q_desired, kJoints);
    WriteValues(file, f.ee_left, kPose);
    WriteValues(file, f.ee_right, kPose);
    std::fprintf(file, "\n");
  }
  std::fclose(file);
}

void WriteVector(std::FILE* file, const char* name, const Joints& q) {
  std::fprintf(file, "  \"%s\": [", name);
  for (int i = 0; i < q.size(); ++i) {
    std::fprintf(file, "%s%.9g", i ? ", " : "", q[i]);
  }
  std::fprintf(file, "]");
}

void WriteRunInfo(const std::filesystem::path& dir, const S1Config& config, const replay::CaseTrajectory& traj,
                  const RunResult& result, bool fake) {
  std::vector<double> read_ms, command_ms;
  uint32_t total_miss = 0, max_consecutive_miss = 0;
  for (const Record& r : result.records) {
    read_ms.push_back(r.feedback.read_s * 1e3);
    command_ms.push_back(r.feedback.command_s * 1e3);
    max_consecutive_miss = std::max(max_consecutive_miss, r.feedback.consecutive_miss);
  }
  if (!result.records.empty()) {
    total_miss = result.records.back().feedback.total_miss - result.records.front().feedback.total_miss;
  }
  auto percentile = [](std::vector<double> v, double p) {
    if (v.empty()) {
      return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[static_cast<size_t>(p * (v.size() - 1))];
  };
  std::string disabled;
  for (const std::string& name : config.safety.Disabled()) {
    disabled += (disabled.empty() ? "\"" : ", \"") + name + "\"";
  }
  std::FILE* file = std::fopen((dir / "run_info.json").c_str(), "w");
  CHECK(file) << "Cannot write " << dir / "run_info.json";
  std::fprintf(file,
               "{\n  \"case\": \"%s\",\n  \"robot\": \"%s\",\n  \"backend\": \"s1_sdk\",\n  \"mode\": \"%s\",\n"
               "  \"dry_run\": %s,\n  \"stop_after_s0\": %s,\n  \"status\": \"%s\",\n  \"reason\": \"%s\",\n",
               traj.name.c_str(), config.model.name.c_str(), fake ? "fake" : "real", fake ? "true" : "false",
               result.stop_after_s0 ? "true" : "false", result.ok ? "ok" : "aborted", result.reason.c_str());
  std::fprintf(file,
               "  \"s1_start_s\": %.6f,\n  \"s2_start_s\": %.6f,\n  \"s3_start_s\": %.6f,\n  \"s4_start_s\": %.6f,\n"
               "  \"end_s\": %.6f,\n",
               result.s1_start, result.s2_start, result.s3_start, result.s4_start, result.end);
  std::fprintf(file,
               "  \"tick_s\": %.6f,\n  \"evaluate_every_ticks\": 1,\n  \"freq_hz\": %.3f,\n"
               "  \"command\": \"S0 set_joints_position direct, S1..S4 set_cartesian_pose direct use_wbc\",\n"
               "  \"torso_command\": \"%s\",\n  \"target_sampling\": \"linear / slerp between the waypoints around t, "
               "one trajectory sample ahead\",\n  \"safety_disabled\": [%s],\n",
               1.0 / config.freq_hz, config.freq_hz, config.torso_command.c_str(), disabled.c_str());
  std::fprintf(file,
               "  \"loop_overruns\": %u,\n  \"loop_max_lateness_s\": 0,\n"
               "  \"bridge\": {\"total_miss\": %u, \"max_consecutive_miss\": %u, \"read_ms_p50\": %.3f, "
               "\"read_ms_max\": %.3f, \"command_ms_p50\": %.3f, \"command_ms_max\": %.3f},\n",
               total_miss, total_miss, max_consecutive_miss, percentile(read_ms, 0.5), percentile(read_ms, 1.0),
               percentile(command_ms, 0.5), percentile(command_ms, 1.0));
  WriteVector(file, "q_start", result.q_start);
  std::fprintf(file, ",\n");
  WriteVector(file, "q_pd", config.model.startup.QPd());
  std::fprintf(file, "\n}\n");
  std::fclose(file);
}

}  // namespace

int RunReplay(const Options& options) {
  const S1Config config = LoadS1Config(options.config);
  const replay::CaseTrajectory traj = replay::LoadCase(options.case_dir);
  const double min_distance = traj.MinTcpDistance();
  if (min_distance < config.safety.min_tcp_distance_m) {
    LOG(ERROR) << "TCPs of " << traj.name << " come as close as " << min_distance << " m, below "
               << config.safety.min_tcp_distance_m << " m";
    return 1;
  }
  for (const std::string& name : config.safety.Disabled()) {
    LOG(WARNING) << "Safety check " << name << " is disabled";
  }

  UdpBridge bridge;
  if (!bridge.Open(config.port)) {
    LOG(ERROR) << bridge.last_error();
    return 1;
  }
  S1Replay replayer(config, traj, options);
  std::string error;
  if (!replayer.Connect(bridge, error)) {
    LOG(ERROR) << error;
    replayer.Finish(bridge);
    return 1;
  }
  if (options.stop_after == "connect") {
    LOG(INFO) << "Stopping after the connection checks";
    replayer.Finish(bridge);
    return 0;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const int runs = options.stop_after.empty() ? options.runs : 1;
  // The records stay in memory until the last run: writing them between runs would leave the sidecar unanswered.
  std::vector<RunResult> results;
  bool ok = true;
  for (int i = 1; i <= runs && ok; ++i) {
    results.push_back(replayer.Run(bridge));
    ok = results.back().ok;
    LOG(INFO) << "Run " << i << (ok ? " done" : " aborted: " + results.back().reason);
  }
  if (ok) {
    replayer.Finish(bridge);
  }
  const std::filesystem::path out(options.out);
  for (size_t i = 0; i < results.size(); ++i) {
    const std::filesystem::path dir = out / ("run_" + std::to_string(i + 1));
    WriteRecords(dir, results[i].records);
    WriteRunInfo(dir, config, traj, results[i], replayer.fake());
  }
  LOG(INFO) << "Records in " << out;
  return ok ? 0 : 2;
}

}  // namespace s1
