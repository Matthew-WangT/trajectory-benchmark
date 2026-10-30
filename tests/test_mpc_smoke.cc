#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mpc/dual_arm_ocs2_ik_controller.h"
#include "mpc/ik_controller.h"
#include "mpc/mpc_config.h"
#include "replay/config.h"

namespace controller {
namespace {

// Drives the ported controller synchronously toward a static target 5 cm away from the TCPs at q_pd, the reference
// being re-anchored the way the replay loop does it. The controller settles within about 1 s but keeps a static offset,
// the default_q regularization pulling the joints toward the task .info initialState: about 1 mm on the left arm and
// 10 mm / 1.5 deg on the right arm from q_pd. The bounds below catch a broken port, not that offset.
TEST(MpcSmokeTest, ConvergesToAStaticTarget) {
  const std::string source_dir = BENCHMARK_SOURCE_DIR;
  const replay::RobotModelConfig robot = replay::LoadRobotModelConfig(source_dir + "/config/robots/marvin");
  const Ocs2IkControllerConfig config = LoadOcs2IkControllerConfig(robot.controller_yaml);
  DualArmOcs2IkController controller(robot.urdf, config);

  Eigen::VectorXd q(2 * DualArmOcs2IkController::kArmDoF);
  q << robot.startup.left_q_pd, robot.startup.right_q_pd;
  Eigen::Isometry3d left_target, right_target;
  controller.GetEePoses(q, left_target, right_target);
  left_target.translation() += Eigen::Vector3d(0.05, 0.0, 0.0);
  right_target.translation() += Eigen::Vector3d(0.0, 0.0, 0.05);

  constexpr double kHorizon = 0.2;
  constexpr double kDt = 0.002;
  double t = 100.0;
  controller.Reset(q, t);
  Eigen::VectorXd q_settled;
  for (int tick = 0; tick < 1500; ++tick) {
    if (tick == 1000) {
      q_settled = q;
    }
    if (tick % 50 == 0) {
      Eigen::Isometry3d left_anchor, right_anchor;
      controller.GetEePoses(q, left_anchor, right_anchor);
      controller.SetTargetTrajectory({t, t + kHorizon}, {left_anchor, left_target}, {right_anchor, right_target});
    }
    controller.AdvanceOnce();
    t += kDt;
    Eigen::VectorXd q_cmd;
    if (controller.Evaluate(t, q_cmd)) {
      q = q_cmd;
    }
  }

  Eigen::VectorXd left_error, right_error;
  controller.Jnt2CartDualError(left_target, right_target, q, left_error, right_error);
  // Settled: the last second barely moves the joints, a slow null space drift of a few mrad remains.
  EXPECT_LT((q - q_settled).cwiseAbs().maxCoeff(), 0.01);
  EXPECT_LT(left_error.head<3>().norm(), 5e-3);
  EXPECT_LT(right_error.head<3>().norm(), 0.015);
  EXPECT_LT(left_error.tail<3>().norm(), 0.02);
  EXPECT_LT(right_error.tail<3>().norm(), 0.04);
}

// The whole-body controller of the S1 moves both TCPs 5 cm with one MPC over the torso and both arms. The bounds catch
// a broken multi end-effector problem (a swapped or ignored target), not the regularization offset.
TEST(MpcSmokeTest, WholeBodyConvergesToAStaticTarget) {
  const std::string source_dir = BENCHMARK_SOURCE_DIR;
  const replay::RobotModelConfig robot = replay::LoadRobotModelConfig(source_dir + "/config/robots/s1");
  const Ocs2IkControllerConfig config = LoadOcs2IkControllerConfig(robot.controller_yaml);
  ASSERT_EQ(config.type, "whole_body");
  const std::unique_ptr<IkController> controller = MakeIkController(robot.urdf, config);
  ASSERT_EQ(controller->dof(), 18);

  Eigen::VectorXd q = robot.startup.QPd();
  Eigen::Isometry3d left_target, right_target;
  controller->GetEePoses(q, left_target, right_target);
  // The TCPs at the S1 home pose, the shoulder midpoint of benchmark_base at 1.121 m.
  EXPECT_GT(left_target.translation().y(), 0.1);
  EXPECT_LT(right_target.translation().y(), -0.1);
  left_target.translation() += Eigen::Vector3d(0.05, 0.0, 0.0);
  right_target.translation() += Eigen::Vector3d(0.0, 0.0, 0.05);

  constexpr double kHorizon = 0.2;
  constexpr double kDt = 0.002;
  double t = 100.0;
  controller->Reset(q, t);
  for (int tick = 0; tick < 1500; ++tick) {
    if (tick % 50 == 0) {
      Eigen::Isometry3d left_anchor, right_anchor;
      controller->GetEePoses(q, left_anchor, right_anchor);
      controller->SetTargetTrajectory({t, t + kHorizon}, {left_anchor, left_target}, {right_anchor, right_target});
    }
    controller->AdvanceOnce();
    t += kDt;
    Eigen::VectorXd q_cmd;
    if (controller->Evaluate(t, q_cmd)) {
      q = q_cmd;
    }
  }

  Eigen::VectorXd left_error, right_error;
  controller->Jnt2CartDualError(left_target, right_target, q, left_error, right_error);
  EXPECT_LT(left_error.head<3>().norm(), 0.015);
  EXPECT_LT(right_error.head<3>().norm(), 0.015);
  EXPECT_LT(left_error.tail<3>().norm(), 0.04);
  EXPECT_LT(right_error.tail<3>().norm(), 0.04);
  // The torso is a redundancy, regularized toward q_pd: it helps but does not fold.
  EXPECT_LT((q.head<4>() - robot.startup.torso_q_pd).cwiseAbs().maxCoeff(), 0.3);
}

}  // namespace
}  // namespace controller
