#pragma once

#include <string>

#include "arm/arm_interface.h"

namespace replay {

struct StartupConfig {
  // Joint pose the S0 phase moves to, the PD stand pose of the reference pipeline.
  arm::JointVector left_q_pd = arm::JointVector::Zero();
  arm::JointVector right_q_pd = arm::JointVector::Zero();
  // S0 duration is max(max|dq| / joint_speed, min_duration).
  double joint_speed_rad_s = 0.3;
  double joint_min_duration_s = 3.0;
  // S1 duration is max(distance / eef_linear_speed, angle / eef_angular_speed, min_duration).
  double eef_linear_speed_m_s = 0.1;
  double eef_angular_speed_rad_s = 0.5;
  double eef_min_duration_s = 3.0;
  // Rate of the S1 waypoints, the same as the benchmark trajectories.
  double eef_waypoint_rate_hz = 200.0;
  // S2: still at the start pose before the trajectory.
  double hold_s = 2.0;
  // S4: still at the last pose after the trajectory.
  double end_hold_s = 2.0;
};

struct SafetyConfig {
  double max_joint_track_err_rad = 0.3;
  // Upper bound of |q_cmd(k) - q_cmd(k-1)| / dt.
  double max_cmd_joint_vel_rad_s = 10.5;
  // Margin kept from the URDF joint limits.
  double joint_limit_margin_rad = 0.02;
  // Abort when no new feedback frame arrives for this long.
  double feedback_timeout_s = 0.02;
  // Abort when the controller receives the commands this late for max_command_delay_frames feedback frames in a row,
  // see CommandDelayMonitor.
  double max_command_delay_s = 0.03;
  int max_command_delay_frames = 50;
  // Abort when the IK controller holds for this many consecutive evaluations.
  int max_consecutive_holds = 50;
  // Refuse to start when the measured joints are further than this from the S0 target.
  double max_initial_deviation_rad = 2.0;
  // Refuse to start when the two TCPs of the reference trajectory come closer than this.
  double min_tcp_distance_m = 0.15;
  // Joint velocity limit of the PD velocity feedforward, checked on the dry run commands.
  double max_feedforward_joint_vel_deg_s = 180.0;
};

// How the IK controller reference is built from the case trajectory, see Replayer::SetReference.
enum class ReferenceMode {
  kLookahead,  // the waypoints due within the next reference_window_s, as the reference pipeline gets them
  kLatest,     // only the newest received waypoint, no lookahead
};

struct ReplayConfig {
  arm::RobotConfig robot;
  // CPU and SCHED_FIFO priority of the 1 ms control loop, cpu -1 leaves it to the OS.
  int rt_cpu = -1;
  int rt_priority = 80;
  // The IK controller is evaluated every `evaluate_every_ticks` loop ticks of 1 ms.
  int evaluate_every_ticks = 2;
  // The command is sent every `send_every_ticks` loop ticks of 1 ms, see config/marvin.yaml.
  int send_every_ticks = 2;
  std::string controller_yaml;
  std::string urdf;
  ReferenceMode reference_mode = ReferenceMode::kLookahead;
  // kLookahead: length of the trajectory window given to the controller, 0 is the MPC horizon.
  double reference_window_s = 0.0;
  // kLatest: time after now at which the newest received waypoint is placed, held beyond it. 0 is the MPC horizon.
  double reference_lead_s = 0.0;
  // The IK controller runs this long before the first run without commanding the robot, see Replayer::WarmUp.
  double mpc_warm_up_s = 1.0;
  StartupConfig startup;
  SafetyConfig safety;
  // First order lag of the dry run robot.
  double dry_run_time_constant_s = 0.01;
};

// Loads the replay yaml. Relative paths in it are resolved against the folder of the yaml file.
ReplayConfig LoadReplayConfig(const std::string& yaml_path);

}  // namespace replay
