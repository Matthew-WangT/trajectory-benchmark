#include "replay/config.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace replay {
namespace {

template <typename T>
void ReadIfPresent(const YAML::Node& node, const char* key, T& value) {
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}

template <size_t N>
void ReadArray(const YAML::Node& node, const char* key, std::array<double, N>& value) {
  if (!node || !node[key]) {
    return;
  }
  const std::vector<double> values = node[key].as<std::vector<double>>();
  if (values.size() != N) {
    throw std::runtime_error(std::string(key) + " expects " + std::to_string(N) + " values");
  }
  std::copy(values.begin(), values.end(), value.begin());
}

void ReadJoints(const YAML::Node& node, const char* key, arm::JointVector& value) {
  if (!node || !node[key]) {
    return;
  }
  const std::vector<double> values = node[key].as<std::vector<double>>();
  if (values.size() != arm::kArmDoF) {
    throw std::runtime_error(std::string(key) + " expects " + std::to_string(arm::kArmDoF) + " values");
  }
  value = Eigen::Map<const arm::JointVector>(values.data());
}

std::string ResolvePath(const std::filesystem::path& base_dir, const std::string& path) {
  if (path.empty() || std::filesystem::path(path).is_absolute()) {
    return path;
  }
  return std::filesystem::weakly_canonical(base_dir / path).string();
}

}  // namespace

ReplayConfig LoadReplayConfig(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const std::filesystem::path base_dir = std::filesystem::absolute(yaml_path).parent_path();
  ReplayConfig config;

  const YAML::Node robot = root["robot"];
  ReadIfPresent(robot, "ip", config.robot.ip);
  ReadIfPresent(robot, "sdk_cpu", config.robot.sdk_cpu);
  ReadIfPresent(robot, "log_switch", config.robot.log_switch);
  ReadIfPresent(robot, "rt_cpu", config.rt_cpu);
  ReadIfPresent(robot, "rt_priority", config.rt_priority);

  const YAML::Node control = root["control"];
  ReadJoints(control["joint_stiffness"], "left", config.robot.control.left.stiffness);
  ReadJoints(control["joint_stiffness"], "right", config.robot.control.right.stiffness);
  ReadJoints(control["joint_damping"], "left", config.robot.control.left.damping);
  ReadJoints(control["joint_damping"], "right", config.robot.control.right.damping);
  ReadIfPresent(control, "vel_ratio", config.robot.control.vel_ratio);
  ReadIfPresent(control, "acc_ratio", config.robot.control.acc_ratio);
  ReadIfPresent(control, "enable_vel_feedforward", config.robot.control.enable_vel_feedforward);
  ReadIfPresent(control, "vel_feedforward_step_ms", config.robot.control.vel_feedforward_step_ms);
  ReadIfPresent(control, "evaluate_every_ticks", config.evaluate_every_ticks);
  ReadIfPresent(control, "send_every_ticks", config.send_every_ticks);

  const YAML::Node tool = root["tool"];
  if (tool) {
    ReadArray(tool["left"], "kine", config.robot.left_tool.kine);
    ReadArray(tool["left"], "dyn", config.robot.left_tool.dyn);
    ReadArray(tool["right"], "kine", config.robot.right_tool.kine);
    ReadArray(tool["right"], "dyn", config.robot.right_tool.dyn);
  }

  const YAML::Node mpc = root["mpc"];
  config.controller_yaml = ResolvePath(base_dir, mpc["controller"].as<std::string>());
  config.urdf = ResolvePath(base_dir, mpc["urdf"].as<std::string>());
  if (mpc["reference_mode"]) {
    const std::string mode = mpc["reference_mode"].as<std::string>();
    if (mode == "lookahead") {
      config.reference_mode = ReferenceMode::kLookahead;
    } else if (mode == "latest") {
      config.reference_mode = ReferenceMode::kLatest;
    } else {
      throw std::runtime_error("mpc.reference_mode must be lookahead or latest, got " + mode);
    }
  }
  ReadIfPresent(mpc, "reference_window_s", config.reference_window_s);
  ReadIfPresent(mpc, "reference_lead_s", config.reference_lead_s);
  ReadIfPresent(mpc, "warm_up_s", config.mpc_warm_up_s);

  const YAML::Node startup = root["startup"];
  ReadJoints(startup["q_pd"], "left", config.startup.left_q_pd);
  ReadJoints(startup["q_pd"], "right", config.startup.right_q_pd);
  ReadIfPresent(startup, "joint_speed_rad_s", config.startup.joint_speed_rad_s);
  ReadIfPresent(startup, "joint_min_duration_s", config.startup.joint_min_duration_s);
  ReadIfPresent(startup, "eef_linear_speed_m_s", config.startup.eef_linear_speed_m_s);
  ReadIfPresent(startup, "eef_angular_speed_rad_s", config.startup.eef_angular_speed_rad_s);
  ReadIfPresent(startup, "eef_min_duration_s", config.startup.eef_min_duration_s);
  ReadIfPresent(startup, "eef_waypoint_rate_hz", config.startup.eef_waypoint_rate_hz);
  ReadIfPresent(startup, "hold_s", config.startup.hold_s);
  ReadIfPresent(startup, "end_hold_s", config.startup.end_hold_s);

  const YAML::Node safety = root["safety"];
  ReadIfPresent(safety, "max_joint_track_err_rad", config.safety.max_joint_track_err_rad);
  ReadIfPresent(safety, "max_cmd_joint_vel_rad_s", config.safety.max_cmd_joint_vel_rad_s);
  ReadIfPresent(safety, "joint_limit_margin_rad", config.safety.joint_limit_margin_rad);
  ReadIfPresent(safety, "feedback_timeout_s", config.safety.feedback_timeout_s);
  ReadIfPresent(safety, "max_consecutive_holds", config.safety.max_consecutive_holds);
  ReadIfPresent(safety, "max_command_delay_s", config.safety.max_command_delay_s);
  ReadIfPresent(safety, "max_command_delay_frames", config.safety.max_command_delay_frames);
  ReadIfPresent(safety, "max_initial_deviation_rad", config.safety.max_initial_deviation_rad);
  ReadIfPresent(safety, "min_tcp_distance_m", config.safety.min_tcp_distance_m);
  ReadIfPresent(safety, "max_feedforward_joint_vel_deg_s", config.safety.max_feedforward_joint_vel_deg_s);

  ReadIfPresent(root["dry_run"], "time_constant_s", config.dry_run_time_constant_s);
  return config;
}

}  // namespace replay
