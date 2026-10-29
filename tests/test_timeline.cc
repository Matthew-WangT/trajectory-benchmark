#include "replay/timeline.h"

#include <cmath>

#include <gtest/gtest.h>

#include "replay/trajectory.h"

namespace replay {
namespace {

Eigen::Isometry3d Pose(double x, double y, double z, double yaw) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

CaseTrajectory MakeCase() {
  CaseTrajectory traj;
  for (int i = 0; i < 11; ++i) {
    traj.t.push_back(0.005 * i);
    traj.left.push_back(Pose(0.5, 0.3, 0.7 + 0.001 * i, 0.0));
    traj.right.push_back(Pose(0.5, -0.3, 0.7, 0.0));
  }
  traj.left_start = traj.left.front();
  traj.right_start = traj.right.front();
  traj.duration_s = traj.t.back();
  return traj;
}

StartupConfig MakeConfig() {
  StartupConfig config;
  config.left_q_pd = arm::JointVector::Constant(0.5);
  config.right_q_pd = arm::JointVector::Constant(-0.5);
  return config;
}

TEST(TimelineTest, PhaseDurations) {
  const StartupConfig config = MakeConfig();
  const CaseTrajectory traj = MakeCase();
  // 1.5 rad at 0.3 rad/s takes 5 s, above the 3 s minimum.
  Joints q0(2 * arm::kArmDoF);
  q0 << arm::JointVector::Constant(-1.0), arm::JointVector::Constant(-0.5);
  // 0.2 m at 0.1 m/s takes 2 s and 1 rad at 0.5 rad/s takes 2 s, so the 3 s minimum applies.
  const Timeline timeline(config, traj, q0, Pose(0.5, 0.3, 0.5, 1.0), Pose(0.5, -0.3, 0.7, 0.0));

  EXPECT_NEAR(timeline.s1_start(), 5.0, 1e-9);
  EXPECT_NEAR(timeline.s2_start(), 8.0, 1e-9);
  EXPECT_NEAR(timeline.s3_start(), 10.0, 1e-9);
  EXPECT_NEAR(timeline.s4_start(), 10.05, 1e-9);
  EXPECT_NEAR(timeline.end(), 12.05, 1e-9);
  EXPECT_EQ(timeline.PhaseAt(0.0), Phase::kS0JointMove);
  EXPECT_EQ(timeline.PhaseAt(6.0), Phase::kS1EefMove);
  EXPECT_EQ(timeline.PhaseAt(9.0), Phase::kS2Hold);
  EXPECT_EQ(timeline.PhaseAt(10.01), Phase::kS3Track);
  EXPECT_EQ(timeline.PhaseAt(11.0), Phase::kS4Hold);
  EXPECT_EQ(timeline.PhaseAt(12.1), Phase::kDone);
}

TEST(TimelineTest, JointMoveIsContinuous) {
  const StartupConfig config = MakeConfig();
  const Joints q0 = Joints::Zero(2 * arm::kArmDoF);
  const Timeline timeline(config, MakeCase(), q0, Pose(0.5, 0.3, 0.7, 0.0), Pose(0.5, -0.3, 0.7, 0.0));
  EXPECT_TRUE(timeline.JointCommand(0.0).isApprox(q0));
  EXPECT_TRUE(timeline.JointCommand(timeline.s1_start()).isApprox(config.QPd()));
  // Holds beyond the phase.
  EXPECT_TRUE(timeline.JointCommand(timeline.s1_start() + 10.0).isApprox(config.QPd()));
}

TEST(TimelineTest, JointMoveIncludesTheTorso) {
  StartupConfig config = MakeConfig();
  config.torso_q_pd = Joints::Constant(4, 0.2);
  const Joints q0 = Joints::Zero(4 + 2 * arm::kArmDoF);
  const Timeline timeline(config, MakeCase(), q0, Pose(0.5, 0.3, 0.7, 0.0), Pose(0.5, -0.3, 0.7, 0.0));
  const Joints q_end = timeline.JointCommand(timeline.s1_start());
  ASSERT_EQ(q_end.size(), 18);
  EXPECT_TRUE(q_end.head(4).isApprox(Joints::Constant(4, 0.2)));
  EXPECT_TRUE(q_end.segment(4, arm::kArmDoF).isApprox(config.left_q_pd));
  EXPECT_TRUE(q_end.tail(arm::kArmDoF).isApprox(config.right_q_pd));
}

TEST(TimelineTest, StartsWithS1WithoutS0) {
  // The solve starts at q_pd with no S0 minimum duration, so the run opens with the TCP move.
  StartupConfig config = MakeConfig();
  config.joint_min_duration_s = 0.0;
  const Timeline timeline(config, MakeCase(), config.QPd(), Pose(0.5, 0.3, 0.7, 0.0), Pose(0.5, -0.3, 0.7, 0.0));
  EXPECT_DOUBLE_EQ(timeline.s1_start(), 0.0);
  EXPECT_EQ(timeline.PhaseAt(0.0), Phase::kS1EefMove);
  EXPECT_TRUE(timeline.JointCommand(0.0).isApprox(config.QPd()));
}

TEST(TimelineTest, WaypointsAreCausalAndConnected) {
  const StartupConfig config = MakeConfig();
  const CaseTrajectory traj = MakeCase();
  const Eigen::Isometry3d left_fk_pd = Pose(0.4, 0.3, 0.6, 0.5);
  const Timeline timeline(config, traj, config.QPd(), left_fk_pd, Pose(0.5, -0.3, 0.7, 0.0));
  const auto& waypoints = timeline.waypoints();
  ASSERT_GT(waypoints.size(), traj.t.size());

  for (size_t i = 1; i < waypoints.size(); ++i) {
    EXPECT_GT(waypoints[i].t, waypoints[i - 1].t);
  }
  // S1 starts at FK(q_pd) and ends at the start pose of the case.
  EXPECT_TRUE(waypoints.front().left.isApprox(left_fk_pd));
  const Waypoint& s1_last = waypoints[waypoints.size() - traj.t.size() - 1];
  EXPECT_NEAR(s1_last.t, timeline.s2_start(), 1e-9);
  EXPECT_TRUE(s1_last.left.isApprox(traj.left_start));

  EXPECT_EQ(timeline.LatestWaypoint(timeline.s1_start() - 1e-3), -1);
  EXPECT_EQ(timeline.LatestWaypoint(timeline.s1_start()), 0);
  // During S2 the newest waypoint stays the end of S1.
  EXPECT_EQ(waypoints[timeline.LatestWaypoint(timeline.s3_start() - 1e-3)].t, s1_last.t);
  // In S3 the newest waypoint never lies in the future.
  for (double t = timeline.s3_start(); t < timeline.s4_start(); t += 0.0013) {
    const int index = timeline.LatestWaypoint(t);
    ASSERT_GE(index, 0);
    EXPECT_LE(waypoints[index].t, t);
    if (index + 1 < static_cast<int>(waypoints.size())) {
      EXPECT_GT(waypoints[index + 1].t, t);
    }
  }
  EXPECT_EQ(timeline.LatestWaypoint(timeline.end()), static_cast<int>(waypoints.size()) - 1);
}

TEST(TimelineTest, MinJerkBoundaries) {
  EXPECT_DOUBLE_EQ(MinJerk(0.0), 0.0);
  EXPECT_DOUBLE_EQ(MinJerk(1.0), 1.0);
  EXPECT_NEAR(MinJerk(0.5), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(MinJerk(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(MinJerk(2.0), 1.0);
}

TEST(TrajectoryTest, LoadsTheBenchmarkCase) {
  const CaseTrajectory traj = LoadCase(std::string(BENCHMARK_SOURCE_DIR) + "/data/case02_fold_clothes_spd50");
  EXPECT_EQ(traj.name, "case02_fold_clothes_spd50");
  EXPECT_EQ(traj.t.size(), 2592u);
  EXPECT_DOUBLE_EQ(traj.t.front(), 0.0);
  EXPECT_NEAR(traj.t.back(), traj.duration_s, 0.01);
  EXPECT_TRUE(traj.left.front().isApprox(traj.left_start, 1e-5));
  EXPECT_GT(traj.MinTcpDistance(), 0.2);
  const Eigen::Vector4d q = QuatXyzw(traj.left_start);
  EXPECT_NEAR(std::abs(q[3]), 0.631875, 1e-5);
}

}  // namespace
}  // namespace replay
