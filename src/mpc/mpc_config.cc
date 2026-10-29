#include "mpc/mpc_config.h"

#include <filesystem>
#include <vector>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace controller {
namespace {

std::optional<Eigen::VectorXd> ReadOptionalVector(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    return std::nullopt;
  }
  const std::vector<double> values = node[key].as<std::vector<double>>();
  return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

template <typename T>
void ReadIfPresent(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

std::string ResolvePath(const std::filesystem::path& base_dir, const std::string& path) {
  if (path.empty() || std::filesystem::path(path).is_absolute()) {
    return path;
  }
  return std::filesystem::weakly_canonical(base_dir / path).string();
}

}  // namespace

Ocs2IkControllerConfig LoadOcs2IkControllerConfig(const std::string& yaml_path) {
  const YAML::Node node = YAML::LoadFile(yaml_path);
  const std::filesystem::path base_dir = std::filesystem::absolute(yaml_path).parent_path();

  Ocs2IkControllerConfig config;
  ReadIfPresent(node, "tmp_working_dir", config.tmp_working_dir);
  ReadIfPresent(node, "joint_reg_enable", config.joint_reg_enable);
  ReadIfPresent(node, "joint_reg_q_weight", config.joint_reg_q_weight);
  config.joint_reg_q_weight_scale = ReadOptionalVector(node, "joint_reg_q_weight_scale");
  ReadIfPresent(node, "default_q_reg_enable", config.default_q_reg_enable);
  ReadIfPresent(node, "default_q_reg_weight", config.default_q_reg_weight);
  config.default_reg_q_weight_scale = ReadOptionalVector(node, "default_reg_q_weight_scale");
  ReadIfPresent(node, "max_joint_velocity_rad_s", config.max_joint_velocity_rad_s);
  ReadIfPresent(node, "policy_max_age_s", config.policy_max_age_s);
  ReadIfPresent(node, "mpc_max_frequency_hz", config.mpc_max_frequency_hz);
  ReadIfPresent(node, "mpc_thread_cpu", config.mpc_thread_cpu);
  ReadIfPresent(node, "type", config.type);
  if (config.type == "dual_arm") {
    config.left_info = ResolvePath(base_dir, node["left_info"].as<std::string>());
    config.right_info = ResolvePath(base_dir, node["right_info"].as<std::string>());
  } else if (config.type == "whole_body") {
    config.info = ResolvePath(base_dir, node["info"].as<std::string>());
  } else {
    LOG(FATAL) << "Unknown IK controller type " << config.type << " in " << yaml_path << ", dual_arm or whole_body";
  }

  config.tmp_working_dir = ResolvePath(base_dir, config.tmp_working_dir);
  std::filesystem::create_directories(config.tmp_working_dir);
  LOG(INFO) << "Loaded the IK controller config " << yaml_path << ", CppAD libraries in " << config.tmp_working_dir;
  return config;
}

}  // namespace controller
