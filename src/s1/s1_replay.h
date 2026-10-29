#pragma once

#include <string>

namespace s1 {

struct Options {
  std::string config;
  std::string case_dir;
  // Session folder, created by the caller; the runs go to run_<i>/ in it.
  std::string out;
  int runs = 3;
  // "connect" stops after the connection checks, "S0" after the joint move to q_pd and a hold, "S2" before the
  // trajectory.
  std::string stop_after;
};

// Replays a case on the Astribot S1 through scripts/s1_bridge.py: S0 joint move to q_pd (set_joints_position, direct),
// then S1..S4 as TCP targets for the SDK whole-body control (set_cartesian_pose, direct, use_wbc), one command per SDK
// period. Records the commands and the feedback of each period. Returns the exit code of the replay.
int RunReplay(const Options& options);

}  // namespace s1
