// Replays a benchmark case on the Marvin dual arms in the joint impedance mode, the TCP commands going through the
// receding-horizon OCS2 IK controller, and records the robot feedback.
//
//   replay --config config/marvin.yaml --case data/case02_fold_clothes_spd50 --dry-run
//   replay --config config/marvin.yaml --case data/case02_fold_clothes_spd50 --runs 3
//
// Each run: S0 joint move to q_pd, S1 TCP move to the start pose, S2 hold, S3 the trajectory, S4 hold. The loop runs
// every 1 ms: read the feedback, evaluate the IK controller every `evaluate_every_ticks` ticks, check, send.
// Pinocchio raises the Boost MPL limits, it has to come before any Boost header (here the OCS2 LoadData.h).
#include <pinocchio/fwd.hpp>

#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <ocs2_core/misc/LoadData.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

#include "arm/fake_arm.h"
#include "arm/marvin_arm.h"
#include "mpc/dual_arm_ocs2_ik_controller.h"
#include "replay/config.h"
#include "replay/recorder.h"
#include "replay/safety.h"
#include "replay/timeline.h"
#include "replay/trajectory.h"

namespace replay {
namespace {

constexpr int kDoF = arm::kArmDoF;
constexpr double kTickS = 1e-3;
constexpr double kRadToDeg = 180.0 / M_PI;

std::atomic<bool> g_interrupted{false};

void OnSignal(int /*signal*/) { g_interrupted = true; }

struct Args {
  std::string config;
  std::string case_dir;
  int runs = 3;
  bool dry_run = false;
  std::string out;
  // "connect" stops after the connection checks, "S0" after the joint move to q_pd and a hold, "S2" before the
  // trajectory (S3), so that only the slow moves run.
  std::string stop_after;
  bool skip_vel_check = false;
  // Prints the key settings of the config and exits.
  bool show_config = false;
};

void PrintUsage() {
  std::fprintf(stderr,
               "usage: replay --config CONFIG --case CASE_DIR [--runs N] [--dry-run] [--out DIR]\n"
               "              [--stop-after connect|S0|S2] [--skip-vel-check]\n"
               "       replay --config CONFIG --show-config\n");
}

std::optional<Args> ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " expects a value");
      }
      return argv[++i];
    };
    if (arg == "--config") {
      args.config = next();
    } else if (arg == "--case") {
      args.case_dir = next();
    } else if (arg == "--runs") {
      args.runs = std::stoi(next());
    } else if (arg == "--dry-run") {
      args.dry_run = true;
    } else if (arg == "--out") {
      args.out = next();
    } else if (arg == "--stop-after") {
      args.stop_after = next();
    } else if (arg == "--skip-vel-check") {
      args.skip_vel_check = true;
    } else if (arg == "--show-config") {
      args.show_config = true;
    } else {
      return std::nullopt;
    }
  }
  if (args.config.empty() || (args.case_dir.empty() && !args.show_config) || args.runs < 1 ||
      !(args.stop_after.empty() || args.stop_after == "connect" || args.stop_after == "S0" ||
        args.stop_after == "S2")) {
    return std::nullopt;
  }
  return args;
}

double MonotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string Timestamp() {
  const std::time_t now = std::time(nullptr);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now));
  return buffer;
}

DualJoints Join(const arm::JointVector& left, const arm::JointVector& right) {
  DualJoints q;
  q << left, right;
  return q;
}

Eigen::Matrix<double, 7, 1> PoseVector(const Eigen::Isometry3d& pose) {
  Eigen::Matrix<double, 7, 1> v;
  v << pose.translation(), QuatXyzw(pose);
  return v;
}

void LoadJointLimits(const std::string& urdf_path, DualJoints& lower, DualJoints& upper) {
  const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
  CHECK(model) << "Cannot parse " << urdf_path;
  for (int i = 0; i < 2 * kDoF; ++i) {
    const std::string name = "Joint" + std::to_string(i % kDoF + 1) + (i < kDoF ? "_L" : "_R");
    const urdf::JointConstSharedPtr joint = model->getJoint(name);
    CHECK(joint && joint->limits) << "Joint " << name << " without limits in " << urdf_path;
    lower[i] = joint->limits->lower;
    upper[i] = joint->limits->upper;
  }
}

// Runs the calling thread with SCHED_FIFO on `cpu`. Returns a description of what could not be set.
std::string SetupRealtime(int cpu, int priority) {
  std::string issues;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    issues += std::string("mlockall: ") + std::strerror(errno) + "; ";
  }
  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set); rc != 0) {
      issues += "affinity cpu " + std::to_string(cpu) + ": " + std::strerror(rc) + "; ";
    }
  }
  sched_param param{};
  param.sched_priority = priority;
  if (const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param); rc != 0) {
    issues += "SCHED_FIFO " + std::to_string(priority) + ": " + std::strerror(rc) + "; ";
  }
  return issues;
}

void WriteVector(std::FILE* file, const char* name, const DualJoints& q, bool last = false) {
  std::fprintf(file, "  \"%s\": [", name);
  for (int i = 0; i < q.size(); ++i) {
    std::fprintf(file, "%s%.9g", i ? ", " : "", q[i]);
  }
  std::fprintf(file, "]%s\n", last ? "" : ",");
}

struct RunResult {
  bool ok = true;
  std::string reason;
  int overruns = 0;
  double max_lateness_s = 0.0;
  // Largest delay of the received command behind the sent one, -1 when the command never moved enough to tell.
  double max_command_delay_s = -1.0;
};

class Replayer {
 public:
  Replayer(const ReplayConfig& config, const CaseTrajectory& traj, const Args& args,
           controller::DualArmOcs2IkController& controller, arm::ArmInterface& robot, double horizon_s,
           const DualJoints& lower, const DualJoints& upper)
      : config_(config),
        traj_(traj),
        args_(args),
        controller_(controller),
        robot_(robot),
        horizon_s_(horizon_s),
        reference_lead_s_(config.reference_lead_s > 0.0 ? config.reference_lead_s : horizon_s),
        reference_window_s_(config.reference_window_s > 0.0 ? config.reference_window_s : horizon_s),
        safety_(config.safety, lower, upper) {
    controller_.GetEePoses(Join(config.startup.left_q_pd, config.startup.right_q_pd), left_fk_pd_, right_fk_pd_);
  }

  RunResult Run(const std::filesystem::path& run_dir) {
    RunResult result;
    arm::Feedback feedback;
    if (!ReadNewFeedback(feedback)) {
      result.ok = false;
      result.reason = "no feedback before the run: " + robot_.last_error();
      return result;
    }
    const DualJoints q0 = Join(feedback.left.q, feedback.right.q);
    const Timeline timeline(config_.startup, traj_, feedback.left.q, feedback.right.q, left_fk_pd_, right_fk_pd_);
    const bool stop_after_s0 = args_.stop_after == "S0";
    const double end = stop_after_s0                ? timeline.s1_start() + config_.startup.hold_s
                       : args_.stop_after == "S2"   ? timeline.s3_start()
                                                    : timeline.end();
    Recorder recorder(end);
    LOG(INFO) << "Run " << run_dir.filename().string() << ": S1 at " << timeline.s1_start() << " s, S3 at "
              << timeline.s3_start() << " s, end at " << end << " s";

    DualJoints q_cmd = q0;
    safety_.Reset(q_cmd, 0.0);
    CommandDelayMonitor command_delay;
    int late_frames = 0;
    bool controller_reset = false;
    int last_waypoint = -1;
    int last_window_end = -1;
    Eigen::Isometry3d left_target = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_target = Eigen::Isometry3d::Identity();

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const double t0 = MonotonicSeconds();
    for (int64_t tick = 0;; ++tick) {
      next.tv_nsec += static_cast<long>(kTickS * 1e9);
      if (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        ++next.tv_sec;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
      const double now = MonotonicSeconds();
      const double t = now - t0;
      const double lateness = t - static_cast<double>(tick + 1) * kTickS;
      if (lateness > kTickS) {
        ++result.overruns;
      }
      result.max_lateness_s = std::max(result.max_lateness_s, lateness);

      if (g_interrupted) {
        result = Abort(result, "interrupted");
        break;
      }
      if (!robot_.Read(feedback)) {
        result = Abort(result, "feedback: " + robot_.last_error());
        break;
      }
      if (feedback.is_new) {
        recorder.Add(FeedbackRecord{t, feedback});
      }
      if (feedback.is_new) {
        const auto delay =
            command_delay.Estimate(t, Join(feedback.left.q_cmd_received, feedback.right.q_cmd_received));
        if (delay) {
          result.max_command_delay_s = std::max(result.max_command_delay_s, *delay);
          late_frames = *delay > config_.safety.max_command_delay_s ? late_frames + 1 : 0;
        }
        if (late_frames > config_.safety.max_command_delay_frames) {
          result = Abort(result, "the controller receives the commands " + std::to_string(*delay * 1e3) +
                                     " ms late, see control.send_every_ticks");
          break;
        }
      }
      if (t >= end) {
        break;
      }

      const Phase phase = timeline.PhaseAt(t);
      const bool evaluate_tick = tick % config_.evaluate_every_ticks == 0;
      bool evaluated = false;
      bool held = false;
      int waypoint = -1;
      if (phase == Phase::kS0JointMove || stop_after_s0) {
        arm::JointVector left_q, right_q;
        timeline.JointCommand(t, left_q, right_q);
        q_cmd = Join(left_q, right_q);
      } else if (evaluate_tick) {
        if (!controller_reset) {
          // Seeded with the last command rather than the measurement, so the command does not jump.
          controller_.Reset(q_cmd, now);
          controller_reset = true;
        }
        waypoint = timeline.LatestWaypoint(t);
        // The reference only changes when a waypoint arrives or, with a lookahead, enters the window.
        const int window_end =
            config_.reference_mode == ReferenceMode::kLookahead ? timeline.LatestWaypoint(t + reference_window_s_) : -1;
        if (waypoint >= 0 && (waypoint != last_waypoint || window_end != last_window_end)) {
          left_target = timeline.waypoints()[waypoint].left;
          right_target = timeline.waypoints()[waypoint].right;
          SetReference(timeline, t, now, q_cmd, waypoint, window_end);
          last_waypoint = waypoint;
          last_window_end = window_end;
        }
        Eigen::VectorXd full_q;
        held = !controller_.Evaluate(now, full_q);
        evaluated = true;
        q_cmd = full_q;
      }

      if (evaluate_tick) {
        CmdRecord record;
        record.t = t;
        record.phase = stop_after_s0 ? Phase::kS0JointMove : phase;
        record.waypoint = last_waypoint;
        if (controller_reset) {
          record.left_target = PoseVector(left_target);
          record.right_target = PoseVector(right_target);
        }
        record.q_cmd = q_cmd;
        record.evaluated = evaluated;
        record.held = held;
        record.policy_age = evaluated ? controller_.PolicyAge() : 0.0;
        record.clamp_count = controller_.clamp_count();
        if (evaluated) {
          record.nominal_qd = controller_.FullNominalQd();
        }
        recorder.Add(record);
      }

      const DualJoints q_meas = Join(feedback.left.q, feedback.right.q);
      if (const auto violation = safety_.Check(t, q_cmd, q_meas, feedback.is_new, evaluated, held)) {
        result = Abort(result, *violation);
        break;
      }
      if (tick % config_.send_every_ticks == 0) {
        if (!robot_.Send(q_cmd.head<kDoF>(), q_cmd.tail<kDoF>())) {
          result = Abort(result, "send: " + robot_.last_error());
          break;
        }
        command_delay.AddSent(t, q_cmd);
        recorder.Add(SendRecord{t, stop_after_s0 ? Phase::kS0JointMove : phase, q_cmd});
      }
    }

    if (recorder.overflowed()) {
      LOG(WARNING) << "Recorder buffers overflowed, the tail of the run is missing";
    }
    recorder.Write(run_dir.string());
    WriteRunInfo(run_dir, timeline, q0, result, stop_after_s0);
    if (!args_.dry_run || !result.ok) {
      return result;
    }
    UpdateVelocityStats(recorder);
    return result;
  }

  // Reference of the IK controller at run time `t` (controller clock `now`), from the pose of the last command `q_cmd`.
  // lookahead: the waypoints due in (t, t + window], the trajectory chunk the controller gets in the reference
  //   pipeline; without any (end of the trajectory) the newest received waypoint is held to the end of the horizon.
  // latest: only the newest received waypoint, at now + reference_lead and held beyond; nothing of the future.
  void SetReference(const Timeline& timeline, double t, double now, const DualJoints& q_cmd, int latest,
                    int window_end) {
    const std::vector<Waypoint>& waypoints = timeline.waypoints();
    std::vector<double> times{now};
    std::vector<Eigen::Isometry3d> left(1), right(1);
    controller_.GetEePoses(q_cmd, left[0], right[0]);
    for (int i = latest + 1; i <= window_end; ++i) {
      times.push_back(now + (waypoints[i].t - t));
      left.push_back(waypoints[i].left);
      right.push_back(waypoints[i].right);
    }
    if (times.size() == 1) {
      const double lead = config_.reference_mode == ReferenceMode::kLookahead ? horizon_s_ : reference_lead_s_;
      times.push_back(now + lead);
      left.push_back(waypoints[latest].left);
      right.push_back(waypoints[latest].right);
    }
    controller_.SetTargetTrajectory(times, left, right);
  }

  // Runs the started controller for `duration_s` toward the static TCP poses of `q`, nothing is sent to the robot.
  // The first solves after start-up are slow, which without this shows up as a fast transient at the start of the
  // first S1; the warm solver state survives the Reset of each run.
  void WarmUp(const DualJoints& q, double duration_s) {
    Eigen::Isometry3d left_pose, right_pose;
    controller_.GetEePoses(q, left_pose, right_pose);
    const double start = MonotonicSeconds();
    controller_.Reset(q, start);
    controller_.SetTargetTrajectory({start, start + horizon_s_}, {left_pose, left_pose}, {right_pose, right_pose});
    Eigen::VectorXd q_cmd;
    for (double now = start; now < start + duration_s; now = MonotonicSeconds()) {
      controller_.Evaluate(now, q_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  // Largest |dq_cmd| per joint in deg/s over the IK controlled phases of the dry runs.
  const DualJoints& max_cmd_vel_deg_s() const { return max_cmd_vel_deg_s_; }

 private:
  RunResult Abort(RunResult result, const std::string& reason) {
    // Stop updating the command and disable at once, the brakes hold the arms.
    robot_.Disable();
    result.ok = false;
    result.reason = reason;
    LOG(ERROR) << "Abort: " << reason;
    return result;
  }

  bool ReadNewFeedback(arm::Feedback& feedback) {
    for (int i = 0; i < 100; ++i) {
      if (!robot_.Read(feedback)) {
        return false;
      }
      if (feedback.is_new) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  void UpdateVelocityStats(const Recorder& recorder) {
    const std::vector<CmdRecord>& cmd = recorder.cmd();
    for (size_t i = 1; i < cmd.size(); ++i) {
      if (!cmd[i].evaluated || !cmd[i - 1].evaluated) {
        continue;
      }
      const double dt = cmd[i].t - cmd[i - 1].t;
      if (dt <= 0.0) {
        continue;
      }
      const DualJoints velocity = (cmd[i].q_cmd - cmd[i - 1].q_cmd).cwiseAbs() / dt * kRadToDeg;
      max_cmd_vel_deg_s_ = max_cmd_vel_deg_s_.cwiseMax(velocity);
    }
  }

  void WriteRunInfo(const std::filesystem::path& run_dir, const Timeline& timeline, const DualJoints& q0,
                    const RunResult& result, bool stop_after_s0) const {
    std::FILE* file = std::fopen((run_dir / "run_info.json").c_str(), "w");
    if (file == nullptr) {
      LOG(ERROR) << "Cannot write run_info.json in " << run_dir;
      return;
    }
    std::fprintf(file, "{\n  \"case\": \"%s\",\n  \"dry_run\": %s,\n  \"stop_after_s0\": %s,\n", traj_.name.c_str(),
                 args_.dry_run ? "true" : "false", stop_after_s0 ? "true" : "false");
    std::fprintf(file, "  \"status\": \"%s\",\n  \"reason\": \"%s\",\n", result.ok ? "ok" : "aborted",
                 result.reason.c_str());
    std::fprintf(file,
                 "  \"s1_start_s\": %.6f,\n  \"s2_start_s\": %.6f,\n  \"s3_start_s\": %.6f,\n  \"s4_start_s\": %.6f,\n"
                 "  \"end_s\": %.6f,\n",
                 timeline.s1_start(), timeline.s2_start(), timeline.s3_start(), timeline.s4_start(), timeline.end());
    std::fprintf(file,
                 "  \"tick_s\": %.6f,\n  \"evaluate_every_ticks\": %d,\n  \"horizon_s\": %.6f,\n"
                 "  \"reference_mode\": \"%s\",\n  \"reference_window_s\": %.6f,\n  \"reference_lead_s\": %.6f,\n",
                 kTickS, config_.evaluate_every_ticks, horizon_s_,
                 config_.reference_mode == ReferenceMode::kLookahead ? "lookahead" : "latest", reference_window_s_,
                 reference_lead_s_);
    std::fprintf(file,
                 "  \"loop_overruns\": %d,\n  \"loop_max_lateness_s\": %.6f,\n  \"send_every_ticks\": %d,\n"
                 "  \"max_command_delay_s\": %.4f,\n",
                 result.overruns, result.max_lateness_s, config_.send_every_ticks, result.max_command_delay_s);
    WriteVector(file, "q_start", q0);
    WriteVector(file, "q_pd", Join(config_.startup.left_q_pd, config_.startup.right_q_pd), true);
    std::fprintf(file, "}\n");
    std::fclose(file);
  }

  const ReplayConfig& config_;
  const CaseTrajectory& traj_;
  const Args& args_;
  controller::DualArmOcs2IkController& controller_;
  arm::ArmInterface& robot_;
  double horizon_s_;
  double reference_lead_s_;
  double reference_window_s_;
  SafetyMonitor safety_;
  Eigen::Isometry3d left_fk_pd_;
  Eigen::Isometry3d right_fk_pd_;
  DualJoints max_cmd_vel_deg_s_ = DualJoints::Zero();
};

void WriteVelocityCheck(const std::filesystem::path& path, const DualJoints& max_deg_s, double limit) {
  std::FILE* file = std::fopen(path.c_str(), "w");
  CHECK(file != nullptr) << "Cannot write " << path;
  std::fprintf(file, "{\n");
  WriteVector(file, "max_cmd_joint_vel_deg_s", max_deg_s);
  std::fprintf(file, "  \"limit_deg_s\": %.3f,\n  \"pass\": %s\n}\n", limit,
               max_deg_s.maxCoeff() < limit ? "true" : "false");
  std::fclose(file);
}

// Checks the dry run velocity of the case before driving the real robot.
bool CheckDryRunVelocity(const std::filesystem::path& path, double limit) {
  if (!std::filesystem::exists(path)) {
    LOG(ERROR) << "Missing " << path << ", run the case with --dry-run first (or pass --skip-vel-check)";
    return false;
  }
  const YAML::Node node = YAML::LoadFile(path.string());
  const std::vector<double> values = node["max_cmd_joint_vel_deg_s"].as<std::vector<double>>();
  const double max_value = *std::max_element(values.begin(), values.end());
  if (max_value >= limit) {
    LOG(ERROR) << "Dry run joint command velocity reaches " << max_value << " deg/s, the PD velocity feedforward "
               << "needs it below " << limit << " deg/s";
    return false;
  }
  LOG(INFO) << "Dry run joint command velocity peaks at " << max_value << " deg/s, below " << limit << " deg/s";
  return true;
}

std::string Format(const arm::JointVector& v) {
  std::ostringstream text;
  text << "[" << v.transpose().format(Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ")) << "]";
  return text.str();
}

void ShowConfig(const ReplayConfig& config) {
  const arm::ControlConfig& control = config.robot.control;
  auto loaded = [](const arm::ToolConfig& tool) {
    return std::any_of(tool.dyn.begin(), tool.dyn.end(), [](double v) { return v != 0.0; });
  };
  std::printf("robot ip             %s\n", config.robot.ip.c_str());
  std::printf("stiffness left       %s\n", Format(control.left.stiffness).c_str());
  std::printf("stiffness right      %s\n", Format(control.right.stiffness).c_str());
  std::printf("damping left         %s\n", Format(control.left.damping).c_str());
  std::printf("damping right        %s\n", Format(control.right.damping).c_str());
  std::printf("velocity feedforward %s, step %d ms; send every %d ms, IK every %d ms\n",
              control.enable_vel_feedforward ? "on" : "off", control.vel_feedforward_step_ms,
              config.send_every_ticks, config.evaluate_every_ticks);
  std::printf("tool dyn             left %s, right %s\n", loaded(config.robot.left_tool) ? "set" : "zeros (no load)",
              loaded(config.robot.right_tool) ? "set" : "zeros (no load)");
  std::printf("IK reference         %s\n",
              config.reference_mode == ReferenceMode::kLookahead ? "lookahead (trajectory chunk)" : "latest waypoint");
}

int Main(const Args& args) {
  const ReplayConfig config = LoadReplayConfig(args.config);
  if (args.show_config) {
    ShowConfig(config);
    return 0;
  }
  const CaseTrajectory traj = LoadCase(args.case_dir);
  const double min_distance = traj.MinTcpDistance();
  if (min_distance < config.safety.min_tcp_distance_m) {
    LOG(ERROR) << "TCPs of " << traj.name << " come as close as " << min_distance << " m, below "
               << config.safety.min_tcp_distance_m << " m";
    return 1;
  }
  LOG(INFO) << "Case " << traj.name << ": " << traj.t.size() << " rows over " << traj.t.back()
            << " s, TCP distance >= " << min_distance << " m";

  const std::filesystem::path runs_root = std::filesystem::path(args.config).parent_path() / ".." / "runs";
  const std::filesystem::path case_root = std::filesystem::weakly_canonical(runs_root / traj.name);
  const std::filesystem::path out =
      !args.out.empty() ? std::filesystem::path(args.out) : case_root / (args.dry_run ? "dry" : "real_" + Timestamp());
  std::filesystem::create_directories(out);
  const std::filesystem::path vel_check = case_root / "dry" / "vel_check.json";
  if (!args.dry_run && !args.skip_vel_check &&
      !CheckDryRunVelocity(vel_check, config.safety.max_feedforward_joint_vel_deg_s)) {
    return 1;
  }

  DualJoints lower, upper;
  LoadJointLimits(config.urdf, lower, upper);
  const controller::Ocs2IkControllerConfig controller_config =
      controller::LoadOcs2IkControllerConfig(config.controller_yaml);
  double horizon_s = 0.0;
  ocs2::loadData::loadCppDataType(controller_config.left_info, "mpc.timeHorizon", horizon_s);
  LOG(INFO) << "Building the IK controller, the first build generates the CppAD libraries and takes minutes";
  controller::DualArmOcs2IkController controller(config.urdf, controller_config);

  std::unique_ptr<arm::ArmInterface> robot;
  if (args.dry_run) {
    robot = std::make_unique<arm::FakeArm>(config.startup.left_q_pd, config.startup.right_q_pd,
                                           config.dry_run_time_constant_s);
  } else {
    robot = std::make_unique<arm::MarvinArm>(config.robot);
  }
  if (!robot->Connect() || !robot->CheckRobotIni((out / "robot.ini").string())) {
    LOG(ERROR) << robot->last_error();
    robot->Disable();
    return 1;
  }
  arm::Feedback feedback;
  if (!robot->Read(feedback)) {
    LOG(ERROR) << robot->last_error();
    robot->Disable();
    return 1;
  }
  const DualJoints q_meas = Join(feedback.left.q, feedback.right.q);
  const DualJoints q_pd = Join(config.startup.left_q_pd, config.startup.right_q_pd);
  const double deviation = (q_meas - q_pd).cwiseAbs().maxCoeff();
  LOG(INFO) << "Measured joints " << q_meas.transpose() << ", largest deviation from q_pd " << deviation << " rad";
  if (deviation > config.safety.max_initial_deviation_rad) {
    LOG(ERROR) << "Joints are " << deviation << " rad away from q_pd, above "
               << config.safety.max_initial_deviation_rad << " rad. Move the arms closer first";
    robot->Disable();
    return 1;
  }
  if (args.stop_after == "connect") {
    LOG(INFO) << "Stopping after the connection checks";
    robot->Disable();
    return 0;
  }

  const std::string rt_issues = SetupRealtime(config.rt_cpu, config.rt_priority);
  if (!rt_issues.empty()) {
    LOG(WARNING) << "Real-time setup incomplete, loop jitter is likely: " << rt_issues;
  }
  controller.Start();
  Replayer replayer(config, traj, args, controller, *robot, horizon_s, lower, upper);
  replayer.WarmUp(q_meas, config.mpc_warm_up_s);
  if (!robot->EnterJointImpedance()) {
    LOG(ERROR) << robot->last_error();
    controller.Stop();
    robot->Disable();
    return 1;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const int runs = args.stop_after.empty() ? args.runs : 1;
  bool ok = true;
  for (int i = 1; i <= runs && ok; ++i) {
    const RunResult result = replayer.Run(out / ("run_" + std::to_string(i)));
    ok = result.ok;
    LOG(INFO) << "Run " << i << (ok ? " done" : " aborted: " + result.reason) << ", loop overruns "
              << result.overruns << ", max lateness " << result.max_lateness_s * 1e3 << " ms, max command delay "
              << result.max_command_delay_s * 1e3 << " ms";
  }
  controller.Stop();
  robot->Disable();

  if (ok && args.dry_run && args.stop_after.empty()) {
    std::filesystem::create_directories(vel_check.parent_path());
    WriteVelocityCheck(vel_check, replayer.max_cmd_vel_deg_s(), config.safety.max_feedforward_joint_vel_deg_s);
    LOG(INFO) << "Joint command velocity peaks at " << replayer.max_cmd_vel_deg_s().maxCoeff() << " deg/s, written to "
              << vel_check;
  }
  LOG(INFO) << "Records in " << out;
  return ok ? 0 : 2;
}

}  // namespace
}  // namespace replay

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  std::optional<replay::Args> args;
  try {
    args = replay::ParseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
  }
  if (!args) {
    replay::PrintUsage();
    return 1;
  }
  try {
    return replay::Main(*args);
  } catch (const std::exception& e) {
    LOG(ERROR) << e.what();
    return 1;
  }
}
