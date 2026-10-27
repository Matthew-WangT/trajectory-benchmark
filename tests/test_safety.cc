#include "replay/safety.h"

#include <gtest/gtest.h>

namespace replay {
namespace {

class SafetyTest : public ::testing::Test {
 protected:
  SafetyTest() : monitor_(config_, DualJoints::Constant(-3.0), DualJoints::Constant(3.0)) {
    monitor_.Reset(DualJoints::Zero(), 0.0);
  }

  SafetyConfig config_;
  SafetyMonitor monitor_;
};

TEST_F(SafetyTest, PassesANominalTick) {
  DualJoints q = DualJoints::Constant(0.001);
  EXPECT_FALSE(monitor_.Check(0.001, q, q, true, true, false).has_value());
}

TEST_F(SafetyTest, AbortsOnTrackingError) {
  DualJoints q_cmd = DualJoints::Zero();
  DualJoints q_meas = DualJoints::Zero();
  q_meas[9] = 0.31;
  const auto reason = monitor_.Check(0.001, q_cmd, q_meas, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("right joint 3 tracking error"), std::string::npos);
}

TEST_F(SafetyTest, AbortsOnJointLimit) {
  DualJoints q = DualJoints::Zero();
  // 2.99 is inside the limit but within the 0.02 rad margin.
  q[0] = 2.99;
  monitor_.Reset(q, 0.0);
  const auto reason = monitor_.Check(0.001, q, q, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("left joint 1 command"), std::string::npos);
}

TEST_F(SafetyTest, AbortsOnCommandStep) {
  DualJoints q = DualJoints::Zero();
  // 0.02 rad in 1 ms is 20 rad/s, above the 10.5 rad/s bound.
  q[3] = 0.02;
  const auto reason = monitor_.Check(0.001, q, q, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("command step"), std::string::npos);
}

TEST_F(SafetyTest, MeasuresTheCommandVelocityOverItsHoldTime) {
  DualJoints q = DualJoints::Zero();
  // Held for one tick, then a step of 0.018 rad: 9 rad/s over the 2 ms since the last change.
  EXPECT_FALSE(monitor_.Check(0.001, q, q, true, false, false).has_value());
  q[3] = 0.018;
  EXPECT_FALSE(monitor_.Check(0.002, q, q, true, false, false).has_value());
}

TEST_F(SafetyTest, AbortsOnFeedbackTimeout) {
  const DualJoints q = DualJoints::Zero();
  for (int i = 1; i <= 19; ++i) {
    EXPECT_FALSE(monitor_.Check(0.001 * i, q, q, false, false, false).has_value());
  }
  EXPECT_TRUE(monitor_.Check(0.0215, q, q, false, false, false).has_value());
}

TEST_F(SafetyTest, AbortsOnConsecutiveHolds) {
  const DualJoints q = DualJoints::Zero();
  double t = 0.0;
  for (int i = 0; i < config_.max_consecutive_holds; ++i) {
    t += 0.002;
    EXPECT_FALSE(monitor_.Check(t, q, q, true, true, true).has_value());
  }
  // A fresh command resets the count.
  t += 0.002;
  EXPECT_FALSE(monitor_.Check(t, q, q, true, true, false).has_value());
  for (int i = 0; i <= config_.max_consecutive_holds; ++i) {
    t += 0.002;
    const auto reason = monitor_.Check(t, q, q, true, true, true);
    EXPECT_EQ(reason.has_value(), i == config_.max_consecutive_holds);
  }
}

TEST_F(SafetyTest, AbortsOnNonFiniteCommand) {
  DualJoints q = DualJoints::Zero();
  q[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(monitor_.Check(0.001, q, DualJoints::Zero(), true, false, false).has_value());
}

TEST(CommandDelayMonitorTest, EstimatesTheDelayOfAMovingCommand) {
  CommandDelayMonitor monitor;
  auto command = [](double t) { return DualJoints::Constant(0.5 * t); };
  for (int k = 0; k <= 100; ++k) {
    monitor.AddSent(0.002 * k, command(0.002 * k));
  }
  // Received what was sent 40 ms ago.
  const auto delay = monitor.Estimate(0.2, command(0.16));
  ASSERT_TRUE(delay.has_value());
  EXPECT_NEAR(*delay, 0.04, 1e-9);
}

TEST(CommandDelayMonitorTest, CannotTellOnAStillCommand) {
  CommandDelayMonitor monitor;
  for (int k = 0; k <= 100; ++k) {
    monitor.AddSent(0.002 * k, DualJoints::Constant(0.3));
  }
  EXPECT_FALSE(monitor.Estimate(0.2, DualJoints::Constant(0.3)).has_value());
}

TEST(CommandDelayMonitorTest, ADelayBeyondTheWindowReadsAsTheWindow) {
  CommandDelayMonitor monitor(0.1);
  auto command = [](double t) { return DualJoints::Constant(0.5 * t); };
  for (int k = 0; k <= 500; ++k) {
    monitor.AddSent(0.002 * k, command(0.002 * k));
  }
  const auto delay = monitor.Estimate(1.0, command(0.5));
  ASSERT_TRUE(delay.has_value());
  EXPECT_NEAR(*delay, 0.1, 0.003);
}

}  // namespace
}  // namespace replay
