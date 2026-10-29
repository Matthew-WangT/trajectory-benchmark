#include "replay/safety.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace replay {
namespace {

constexpr int kN = 2 * arm::kArmDoF;

std::vector<std::string> Names() {
  std::vector<std::string> names;
  for (const char* arm : {"left", "right"}) {
    for (int j = 1; j <= arm::kArmDoF; ++j) {
      names.push_back(std::string(arm) + " joint " + std::to_string(j));
    }
  }
  return names;
}

class SafetyTest : public ::testing::Test {
 protected:
  SafetyTest() : monitor_(config_, Joints::Constant(kN, -3.0), Joints::Constant(kN, 3.0), Names()) {
    monitor_.Reset(Joints::Zero(kN), 0.0);
  }

  SafetyConfig config_;
  SafetyMonitor monitor_;
};

TEST_F(SafetyTest, PassesANominalTick) {
  Joints q = Joints::Constant(kN, 0.001);
  EXPECT_FALSE(monitor_.Check(0.001, q, q, true, true, false).has_value());
}

TEST_F(SafetyTest, AbortsOnTrackingError) {
  Joints q_cmd = Joints::Zero(kN);
  Joints q_meas = Joints::Zero(kN);
  q_meas[9] = 0.31;
  const auto reason = monitor_.Check(0.001, q_cmd, q_meas, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("right joint 3 tracking error"), std::string::npos);
}

TEST_F(SafetyTest, AbortsOnJointLimit) {
  Joints q = Joints::Zero(kN);
  // 2.99 is inside the limit but within the 0.02 rad margin.
  q[0] = 2.99;
  monitor_.Reset(q, 0.0);
  const auto reason = monitor_.Check(0.001, q, q, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("left joint 1 command"), std::string::npos);
}

TEST_F(SafetyTest, AbortsOnCommandStep) {
  Joints q = Joints::Zero(kN);
  // 0.02 rad in 1 ms is 20 rad/s, above the 10.5 rad/s bound.
  q[3] = 0.02;
  const auto reason = monitor_.Check(0.001, q, q, true, false, false);
  ASSERT_TRUE(reason.has_value());
  EXPECT_NE(reason->find("command step"), std::string::npos);
}

TEST_F(SafetyTest, MeasuresTheCommandVelocityOverItsHoldTime) {
  Joints q = Joints::Zero(kN);
  // Held for one tick, then a step of 0.018 rad: 9 rad/s over the 2 ms since the last change.
  EXPECT_FALSE(monitor_.Check(0.001, q, q, true, false, false).has_value());
  q[3] = 0.018;
  EXPECT_FALSE(monitor_.Check(0.002, q, q, true, false, false).has_value());
}

TEST_F(SafetyTest, AbortsOnFeedbackTimeout) {
  const Joints q = Joints::Zero(kN);
  for (int i = 1; i <= 19; ++i) {
    EXPECT_FALSE(monitor_.Check(0.001 * i, q, q, false, false, false).has_value());
  }
  EXPECT_TRUE(monitor_.Check(0.0215, q, q, false, false, false).has_value());
}

TEST_F(SafetyTest, AbortsOnConsecutiveHolds) {
  const Joints q = Joints::Zero(kN);
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
  Joints q = Joints::Zero(kN);
  q[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(monitor_.Check(0.001, q, Joints::Zero(kN), true, false, false).has_value());
}

TEST(CommandDelayMonitorTest, EstimatesTheDelayOfAMovingCommand) {
  CommandDelayMonitor monitor;
  auto command = [](double t) { return Joints::Constant(kN, 0.5 * t); };
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
    monitor.AddSent(0.002 * k, Joints::Constant(kN, 0.3));
  }
  EXPECT_FALSE(monitor.Estimate(0.2, Joints::Constant(kN, 0.3)).has_value());
}

TEST(CommandDelayMonitorTest, ADelayBeyondTheWindowReadsAsTheWindow) {
  CommandDelayMonitor monitor(0.1);
  auto command = [](double t) { return Joints::Constant(kN, 0.5 * t); };
  for (int k = 0; k <= 500; ++k) {
    monitor.AddSent(0.002 * k, command(0.002 * k));
  }
  const auto delay = monitor.Estimate(1.0, command(0.5));
  ASSERT_TRUE(delay.has_value());
  EXPECT_NEAR(*delay, 0.1, 0.003);
}

}  // namespace
}  // namespace replay
