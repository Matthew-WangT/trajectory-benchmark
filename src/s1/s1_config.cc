#include "s1/s1_config.h"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace s1 {
namespace {

template <typename T>
void ReadIfPresent(const YAML::Node& node, const char* key, T& value) {
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}

void ReadCheck(const YAML::Node& node, const char* key, const char* value_key, Check& check) {
  if (!node || !node[key]) {
    return;
  }
  ReadIfPresent(node[key], "enable", check.enable);
  ReadIfPresent(node[key], value_key, check.value);
}

}  // namespace

std::vector<std::string> SafetyConfig::Disabled() const {
  std::vector<std::string> names;
  const std::pair<const char*, const Check*> checks[] = {{"joint_limit", &joint_limit},
                                                         {"joint_velocity", &joint_velocity},
                                                         {"tcp_error", &tcp_error},
                                                         {"arm_distance", &arm_distance},
                                                         {"link", &link}};
  for (const auto& [name, check] : checks) {
    if (!check->enable) {
      names.push_back(name);
    }
  }
  return names;
}

bool IsS1Config(const std::string& yaml_path) { return static_cast<bool>(YAML::LoadFile(yaml_path)["bridge"]); }

S1Config LoadS1Config(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const std::filesystem::path base_dir = std::filesystem::absolute(yaml_path).parent_path();
  S1Config config;
  config.model = replay::LoadRobotModelConfig((base_dir / root["robot_dir"].as<std::string>()).string());
  if (config.model.dof() != 18) {
    throw std::runtime_error(yaml_path + ": the S1 bridge expects 18 joints, " + config.model.name + " has " +
                             std::to_string(config.model.dof()));
  }

  const YAML::Node bridge = root["bridge"];
  ReadIfPresent(bridge, "port", config.port);
  ReadIfPresent(bridge, "freq_hz", config.freq_hz);
  ReadIfPresent(bridge, "connect_timeout_s", config.connect_timeout_s);
  ReadIfPresent(bridge, "max_consecutive_miss", config.safety.max_consecutive_miss);
  ReadIfPresent(root, "torso_command", config.torso_command);
  if (config.torso_command != "free" && config.torso_command != "hold") {
    throw std::runtime_error("torso_command must be free or hold, got " + config.torso_command);
  }

  const YAML::Node frame = root["sdk_frame"];
  if (frame && frame["torso_base_in_chassis"]) {
    const auto v = frame["torso_base_in_chassis"].as<std::vector<double>>();
    if (v.size() != 3) {
      throw std::runtime_error("sdk_frame.torso_base_in_chassis expects x, y, z");
    }
    config.torso_base_in_chassis = Eigen::Vector3d(v[0], v[1], v[2]);
  }
  if (frame && frame["ee_frames"]) {
    ReadIfPresent(frame["ee_frames"], "left", config.sdk_left_ee);
    ReadIfPresent(frame["ee_frames"], "right", config.sdk_right_ee);
  }

  const YAML::Node safety = root["safety"];
  ReadCheck(safety, "joint_limit", "margin_rad", config.safety.joint_limit);
  ReadCheck(safety, "joint_velocity", "ratio", config.safety.joint_velocity);
  ReadCheck(safety, "tcp_error", "max_m", config.safety.tcp_error);
  ReadCheck(safety, "arm_distance", "min_m", config.safety.arm_distance);
  ReadCheck(safety, "link", "feedback_timeout_s", config.safety.link);
  ReadIfPresent(safety, "max_initial_deviation_rad", config.safety.max_initial_deviation_rad);
  ReadIfPresent(safety, "min_tcp_distance_m", config.safety.min_tcp_distance_m);
  ReadIfPresent(safety, "max_frame_mismatch_m", config.safety.max_frame_mismatch_m);
  return config;
}

}  // namespace s1
