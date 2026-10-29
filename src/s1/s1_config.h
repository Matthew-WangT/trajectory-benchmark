#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "replay/config.h"

namespace s1 {

// A safety check of the S1 replay with its enable switch; a disabled check is listed in run_info.json.
struct Check {
  bool enable = true;
  double value = 0.0;
};

struct SafetyConfig {
  // Measured and desired joints within the URDF limits shrunk by `value` rad.
  Check joint_limit{true, 0.02};
  // Measured joint velocity, and the step of the desired joints over one SDK period, below `value` times the URDF
  // velocity limit.
  Check joint_velocity{true, 0.8};
  // Measured TCP (FK of the measured joints) within `value` m of the TCP target, S1..S3.
  Check tcp_error{true, 0.08};
  // Measured TCPs at least `value` m apart.
  Check arm_distance{true, 0.2};
  // A feedback frame at least every `value` s, and the sidecar answered within max_consecutive_miss periods.
  Check link{true, 0.05};
  int max_consecutive_miss = 10;
  // Refuse to start when the measured joints are further than this from q_pd.
  double max_initial_deviation_rad = 2.0;
  // Refuse to start when the two TCPs of the reference trajectory come closer than this.
  double min_tcp_distance_m = 0.15;
  // The connect stage compares the SDK end-effector poses with FK of the measured joints, see S1Replay::Connect.
  double max_frame_mismatch_m = 0.005;

  // Names of the disabled checks.
  std::vector<std::string> Disabled() const;
};

// config/s1.yaml: the S1 replay through scripts/s1_bridge.py, see docs/replay.md.
struct S1Config {
  replay::RobotModelConfig model;
  // UDP port of the replay on 127.0.0.1, the sidecar sends its feedback there.
  int port = 47011;
  double freq_hz = 250.0;
  // How long the replay waits for the first feedback of the sidecar.
  double connect_timeout_s = 30.0;
  // free: the SDK whole-body control moves the torso as a redundancy (set_cartesian_pose of both arms with
  // add_default_torso off); hold: the torso is commanded at its pose at the start of S1.
  std::string torso_command = "free";
  // Pose of the URDF root astribot_torso_base in the SDK chassis frame, and the SDK end-effector frames of the URDF.
  Eigen::Vector3d torso_base_in_chassis = Eigen::Vector3d(0.0, 0.0, 0.097);
  std::string sdk_left_ee = "astribot_arm_left_end_effector";
  std::string sdk_right_ee = "astribot_arm_right_end_effector";
  SafetyConfig safety;
};

// True when the replay yaml is an S1 bridge config (it has a bridge section).
bool IsS1Config(const std::string& yaml_path);

S1Config LoadS1Config(const std::string& yaml_path);

}  // namespace s1
