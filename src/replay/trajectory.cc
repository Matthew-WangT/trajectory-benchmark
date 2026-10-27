#include "replay/trajectory.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace replay {
namespace {

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

Eigen::Isometry3d PoseFromYaml(const YAML::Node& node) {
  const auto xyz = node["xyz_m"].as<std::vector<double>>();
  const auto quat = node["qxyzw"].as<std::vector<double>>();
  if (xyz.size() != 3 || quat.size() != 4) {
    throw std::runtime_error("meta.json start pose expects xyz_m[3] and qxyzw[4]");
  }
  return MakePose(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]), Eigen::Vector4d(quat[0], quat[1], quat[2], quat[3]));
}

}  // namespace

Eigen::Isometry3d MakePose(const Eigen::Vector3d& position, const Eigen::Vector4d& quat_xyzw) {
  const Eigen::Quaterniond q(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.normalized().toRotationMatrix();
  pose.translation() = position;
  return pose;
}

Eigen::Vector4d QuatXyzw(const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

double CaseTrajectory::MinTcpDistance() const {
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < t.size(); ++i) {
    min_distance = std::min(min_distance, (left[i].translation() - right[i].translation()).norm());
  }
  return min_distance;
}

CaseTrajectory LoadCase(const std::string& case_dir) {
  const std::filesystem::path dir(case_dir);
  CaseTrajectory traj;
  traj.name = std::filesystem::absolute(dir).lexically_normal().filename().string();
  if (traj.name.empty()) {
    traj.name = std::filesystem::absolute(dir).lexically_normal().parent_path().filename().string();
  }

  // meta.json is a JSON document, which is valid YAML.
  const YAML::Node meta = YAML::LoadFile((dir / "meta.json").string());
  traj.duration_s = meta["duration_s"].as<double>();
  traj.left_start = PoseFromYaml(meta["start_pose_in_base"]["l"]);
  traj.right_start = PoseFromYaml(meta["start_pose_in_base"]["r"]);

  std::ifstream file(dir / "eef_command.csv");
  if (!file) {
    throw std::runtime_error("Cannot open " + (dir / "eef_command.csv").string());
  }
  std::string line;
  std::getline(file, line);
  const std::vector<std::string> header = SplitCsv(line);
  std::map<std::string, size_t> column;
  for (size_t i = 0; i < header.size(); ++i) {
    column[header[i]] = i;
  }
  auto index = [&column](const std::string& name) {
    const auto it = column.find(name);
    if (it == column.end()) {
      throw std::runtime_error("eef_command.csv misses the column " + name);
    }
    return it->second;
  };
  const size_t t_col = index("t_s");
  std::map<char, std::array<size_t, 7>> pose_cols;
  for (char arm : {'l', 'r'}) {
    const std::string p(1, arm);
    pose_cols[arm] = {index(p + "_x_m"), index(p + "_y_m"), index(p + "_z_m"), index(p + "_qx"),
                      index(p + "_qy"),  index(p + "_qz"),  index(p + "_qw")};
  }

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = SplitCsv(line);
    auto value = [&fields](size_t i) { return std::stod(fields.at(i)); };
    traj.t.push_back(value(t_col));
    for (char arm : {'l', 'r'}) {
      const auto& c = pose_cols[arm];
      const Eigen::Isometry3d pose = MakePose(Eigen::Vector3d(value(c[0]), value(c[1]), value(c[2])),
                                              Eigen::Vector4d(value(c[3]), value(c[4]), value(c[5]), value(c[6])));
      (arm == 'l' ? traj.left : traj.right).push_back(pose);
    }
  }
  if (traj.t.size() < 2) {
    throw std::runtime_error("eef_command.csv holds fewer than two rows");
  }
  for (size_t i = 1; i < traj.t.size(); ++i) {
    if (traj.t[i] <= traj.t[i - 1]) {
      throw std::runtime_error("eef_command.csv t_s is not strictly increasing at row " + std::to_string(i));
    }
  }
  return traj;
}

}  // namespace replay
