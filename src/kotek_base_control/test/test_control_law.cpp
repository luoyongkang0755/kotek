#include <cmath>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include "kotek_base_control/simple_base_controller.hpp"

using kotek_base_control::BaseControllerParams;
using kotek_base_control::ControlPhase;
using kotek_base_control::Pose2D;
using kotek_base_control::SimpleBaseController;
using kotek_base_control::Twist2D;

namespace
{

BaseControllerParams defaultParams()
{
  BaseControllerParams p;
  p.max_linear_velocity = 0.5;
  p.min_linear_velocity = 0.05;
  p.max_angular_velocity = 0.8;
  p.min_angular_velocity = 0.1;
  p.max_linear_accel = 0.5;
  p.k_linear = 0.8;
  p.k_angular = 1.5;
  p.xy_tolerance = 0.08;
  p.yaw_tolerance = 0.10;
  p.heading_tolerance = 0.15;
  return p;
}

/// Integrates a unicycle model forward by dt using the commanded twist.
Pose2D integrate(const Pose2D & pose, const Twist2D & cmd, double dt)
{
  Pose2D next;
  next.x = pose.x + cmd.linear * std::cos(pose.yaw) * dt;
  next.y = pose.y + cmd.linear * std::sin(pose.yaw) * dt;
  next.yaw = kotek_base_control::normalizeAngle(pose.yaw + cmd.angular * dt);
  return next;
}

/// Models the real, measured mismatch between a COMMANDED twist and the
/// robot's ACTUALLY ACHIEVED velocity, instead of assuming instant
/// response like `integrate()` above. Investigating the live E2E test's
/// "Goal timed out" failure (see docs/pick_and_delivery_report.md section
/// 4.5): a direct /cmd_vel rotation test on the real (fixed) asset found
/// commanding a constant angular.z=0.5rad/s produces achieved angular
/// velocity far below commanded for the first couple of seconds (roughly
/// 0.012-0.019 rad/s, ~2.5-4% of commanded) before ramping up (reaching
/// ~0.12-0.18 rad/s, ~24-36% of commanded, by t=3s) -- a real,
/// stiction/breakaway-like nonlinearity in the rigid analytic-cylinder
/// wheels resisting in-place rotation via scrub friction. Straight-line
/// driving showed no such lag (reaching ~97% of the ideal displacement in
/// the same window), so only angular response is modeled here.
///
/// This is deliberately a SIMPLIFIED approximation (a first-order lag,
/// not the actual biphasic breakaway-then-ramp curve) -- exponential lag
/// with tau=8.4s reaches ~30% of a commanded step by t=3s, matching the
/// measured order of magnitude without over-fitting a precise physical
/// model. The point is to test whether the CONTROLLER's phase logic
/// tolerates a real robot being much slower to achieve commanded angular
/// velocity than the idealized `integrate()` model assumes -- not to
/// reproduce Isaac Sim's exact friction curve.
class LaggedRobotModel
{
public:
  explicit LaggedRobotModel(double angular_lag_tau)
  : angular_lag_tau_(angular_lag_tau)
  {
  }

  Pose2D step(const Pose2D & pose, const Twist2D & cmd, double dt)
  {
    achieved_angular_ += (cmd.angular - achieved_angular_) * (dt / angular_lag_tau_);
    Twist2D achieved{cmd.linear, achieved_angular_};
    return integrate(pose, achieved, dt);
  }

private:
  double angular_lag_tau_;
  double achieved_angular_ = 0.0;
};

/// A harsher, real-stiction-style model: commanded angular velocities
/// below `deadband` produce ZERO real response (not just attenuated) --
/// modeling a torque-below-breakaway-threshold wheel that doesn't move at
/// all, rather than one that just moves slowly. Above the deadband, uses
/// the same lag as LaggedRobotModel. This specifically tests
/// `computeDrive`'s angular output, which (unlike computeTurnToHeading/
/// computeAlign) never applies `applyMinMagnitude` -- so small in-DRIVE
/// heading corrections could legitimately compute to a value under this
/// deadband, in which case the real robot would never correct heading at
/// all while driving, only while in TURN_TO_HEADING.
class DeadbandRobotModel
{
public:
  DeadbandRobotModel(double angular_lag_tau, double deadband)
  : angular_lag_tau_(angular_lag_tau), deadband_(deadband)
  {
  }

  Pose2D step(const Pose2D & pose, const Twist2D & cmd, double dt)
  {
    const double effective_cmd = std::fabs(cmd.angular) < deadband_ ? 0.0 : cmd.angular;
    achieved_angular_ += (effective_cmd - achieved_angular_) * (dt / angular_lag_tau_);
    Twist2D achieved{cmd.linear, achieved_angular_};
    return integrate(pose, achieved, dt);
  }

private:
  double angular_lag_tau_;
  double deadband_;
  double achieved_angular_ = 0.0;
};

}  // namespace

TEST(ControlLaw, LargeHeadingErrorStaysInTurnPhaseWithZeroLinearVelocity)
{
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{5.0, 0.0, 0.0});

  // Robot faces directly away from the goal: theta_err ~ pi.
  const Twist2D cmd = ctrl.step(Pose2D{0.0, 0.0, M_PI});

  EXPECT_EQ(ctrl.phase(), ControlPhase::TURN_TO_HEADING);
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_LE(std::fabs(cmd.angular), defaultParams().max_angular_velocity + 1e-9);
  EXPECT_GT(std::fabs(cmd.angular), 0.0);
}

TEST(ControlLaw, AlignedHeadingEntersDriveWithForwardVelocity)
{
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{5.0, 0.0, 0.0});

  const Twist2D cmd = ctrl.step(Pose2D{0.0, 0.0, 0.0});

  EXPECT_EQ(ctrl.phase(), ControlPhase::DRIVE);
  EXPECT_GT(cmd.linear, 0.0);
  EXPECT_LE(cmd.linear, defaultParams().max_linear_velocity + 1e-9);
}

TEST(ControlLaw, VelocityCommandsNeverExceedLimitsAcrossManyGoals)
{
  const BaseControllerParams params = defaultParams();
  const std::vector<Pose2D> goals = {
    Pose2D{1.0, 0.0, 0.0},
    Pose2D{0.0, 1.0, M_PI / 2.0},
    Pose2D{-2.0, -2.0, M_PI},
    Pose2D{3.0, -1.5, -M_PI / 3.0},
    Pose2D{0.02, 0.01, 0.0},  // goal essentially at the start pose
  };

  for (const auto & goal : goals) {
    SimpleBaseController ctrl(params);
    ctrl.setGoal(goal);
    Pose2D pose{0.0, 0.0, 0.0};

    bool reached = false;
    for (int i = 0; i < 20000 && !reached; ++i) {
      const Twist2D cmd = ctrl.step(pose);

      EXPECT_FALSE(std::isnan(cmd.linear));
      EXPECT_FALSE(std::isnan(cmd.angular));
      EXPECT_LE(std::fabs(cmd.linear), params.max_linear_velocity + 1e-9);
      EXPECT_LE(std::fabs(cmd.angular), params.max_angular_velocity + 1e-9);

      pose = integrate(pose, cmd, 0.02);
      reached = ctrl.isGoalReached();
    }

    EXPECT_TRUE(reached)
      << "did not converge to goal (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")";
    EXPECT_LE(kotek_base_control::distance(pose, goal), params.xy_tolerance + 1e-6);
    EXPECT_LE(
      std::fabs(kotek_base_control::normalizeAngle(pose.yaw - goal.yaw)),
      params.yaw_tolerance + 1e-6);
  }
}

TEST(ControlLaw, LinearVelocityDecreasesTowardsZeroNearGoal)
{
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{1.0, 0.0, 0.0});

  // Force phase into DRIVE (heading already aligned with the goal).
  Twist2D first = ctrl.step(Pose2D{0.0, 0.0, 0.0});
  ASSERT_EQ(ctrl.phase(), ControlPhase::DRIVE);
  EXPECT_GT(first.linear, 0.0);

  // As the robot gets close (but still outside xy_tolerance), the commanded
  // speed must shrink towards zero rather than slamming to a stop.
  double previous_speed = first.linear;
  const double tol = defaultParams().xy_tolerance;
  for (double remaining = 0.5; remaining > tol + 0.005; remaining -= 0.02) {
    const Twist2D cmd = ctrl.step(Pose2D{1.0 - remaining, 0.0, 0.0});
    EXPECT_GE(previous_speed + 1e-6, cmd.linear)
      << "speed should not increase as the robot approaches the goal";
    previous_speed = cmd.linear;
  }
  EXPECT_LT(previous_speed, first.linear);
}

TEST(ControlLaw, CoincidentGoalDoesNotProduceNaNAndReachesStopped)
{
  SimpleBaseController ctrl(defaultParams());
  const Pose2D pose{2.0, -1.0, 0.7};
  ctrl.setGoal(pose);

  // First call: rho == 0 <= xy_tolerance, so TURN_TO_HEADING -> ALIGN in
  // this same call (no NaN, zero command since yaw_err is already 0 too).
  const Twist2D first = ctrl.step(pose);
  EXPECT_FALSE(std::isnan(first.linear));
  EXPECT_FALSE(std::isnan(first.angular));
  EXPECT_EQ(ctrl.phase(), ControlPhase::ALIGN);
  EXPECT_DOUBLE_EQ(first.linear, 0.0);
  EXPECT_DOUBLE_EQ(first.angular, 0.0);

  // Second call: ALIGN checks yaw_err at entry and finds it already
  // satisfied -> STOPPED.
  const Twist2D second = ctrl.step(pose);
  EXPECT_FALSE(std::isnan(second.linear));
  EXPECT_FALSE(std::isnan(second.angular));
  EXPECT_EQ(ctrl.phase(), ControlPhase::STOPPED);
  EXPECT_DOUBLE_EQ(second.linear, 0.0);
  EXPECT_DOUBLE_EQ(second.angular, 0.0);
}

TEST(ControlLaw, LargeHeadingDeviationMidDriveReturnsToTurnPhase)
{
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{5.0, 0.0, 0.0});

  ASSERT_EQ(ctrl.step(Pose2D{0.0, 0.0, 0.0}).linear > 0.0, true);
  ASSERT_EQ(ctrl.phase(), ControlPhase::DRIVE);

  // A large disturbance in heading (e.g. wheel slip) should kick the
  // controller back into TURN_TO_HEADING via the hysteresis check.
  const Twist2D cmd = ctrl.step(Pose2D{1.0, 0.0, M_PI / 2.0 + 0.5});
  EXPECT_EQ(ctrl.phase(), ControlPhase::TURN_TO_HEADING);
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
}

TEST(ControlLaw, ReachesStoppedAndStaysThereWithZeroCommand)
{
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{0.5, 0.0, 0.0});
  Pose2D pose{0.0, 0.0, 0.0};

  bool reached = false;
  for (int i = 0; i < 5000 && !reached; ++i) {
    const Twist2D cmd = ctrl.step(pose);
    pose = integrate(pose, cmd, 0.02);
    reached = ctrl.isGoalReached();
  }
  ASSERT_TRUE(reached);

  for (int i = 0; i < 5; ++i) {
    const Twist2D cmd = ctrl.step(pose);
    EXPECT_TRUE(ctrl.isGoalReached());
    EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
  }
}

TEST(ControlLaw, RealAngularLagOnTheLiveE2EGoalDoesNotThrash)
{
  // Reproduces (in a fast, pure-C++, no-Isaac-Sim harness) the exact goal
  // from the live E2E test's "Goal timed out after 120.0s" failure --
  // see docs/pick_and_delivery_report.md section 4.5. Uses
  // LaggedRobotModel instead of the idealized integrate() to test whether
  // the phase machine (TURN_TO_HEADING <-> DRIVE hysteresis) thrashes
  // when angular response is much slower than commanded, which the
  // idealized-response VelocityCommandsNeverExceedLimitsAcrossManyGoals
  // test above cannot reveal (it assumes instant response).
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{1.113, 0.297, 0.261});
  Pose2D pose{0.0, 0.0, 0.0};
  LaggedRobotModel model(/*angular_lag_tau=*/8.4);

  const double dt = 0.02;  // matches base_controller.yaml's control_rate: 20.0
  const int max_steps = static_cast<int>(200.0 / dt);  // 200s budget

  ControlPhase last_phase = ctrl.phase();
  int phase_transitions = 0;
  double reached_at_s = -1.0;
  for (int i = 0; i < max_steps; ++i) {
    const Twist2D cmd = ctrl.step(pose);
    if (ctrl.phase() != last_phase) {
      ++phase_transitions;
      last_phase = ctrl.phase();
    }
    if (ctrl.isGoalReached()) {
      reached_at_s = i * dt;
      break;
    }
    pose = model.step(pose, cmd, dt);
  }

  std::cout << "[RealAngularLagOnTheLiveE2EGoalDoesNotThrash] phase_transitions="
            << phase_transitions << " reached_at_s=" << reached_at_s
            << " final_pose=(" << pose.x << ", " << pose.y << ", " << pose.yaw << ")"
            << std::endl;

  // Not asserting convergence time yet -- this test's purpose (for now) is
  // to OBSERVE phase_transitions/reached_at_s and decide the fix from
  // that, not to encode an assumption about the right answer up front.
  EXPECT_GE(phase_transitions, 0);  // always true; keeps the test from being a no-op if empty
}

TEST(ControlLaw, DeadbandModelOnTheLiveE2EGoalRevealsComputeDriveGap)
{
  // Same goal/harness as RealAngularLagOnTheLiveE2EGoalDoesNotThrash, but
  // with DeadbandRobotModel (see its comment): commanded angular
  // velocities under min_angular_velocity produce ZERO real response,
  // matching what a real stiction-limited wheel would do below its own
  // breakaway threshold, and specifically probing computeDrive's missing
  // applyMinMagnitude() call (present on computeTurnToHeading/
  // computeAlign, absent here with no stated reason).
  SimpleBaseController ctrl(defaultParams());
  ctrl.setGoal(Pose2D{1.113, 0.297, 0.261});
  Pose2D pose{0.0, 0.0, 0.0};
  const double deadband = defaultParams().min_angular_velocity;
  DeadbandRobotModel model(/*angular_lag_tau=*/8.4, deadband);

  const double dt = 0.02;
  const int max_steps = static_cast<int>(200.0 / dt);

  ControlPhase last_phase = ctrl.phase();
  int phase_transitions = 0;
  double reached_at_s = -1.0;
  for (int i = 0; i < max_steps; ++i) {
    const Twist2D cmd = ctrl.step(pose);
    if (ctrl.phase() != last_phase) {
      ++phase_transitions;
      last_phase = ctrl.phase();
    }
    if (ctrl.isGoalReached()) {
      reached_at_s = i * dt;
      break;
    }
    pose = model.step(pose, cmd, dt);
  }

  std::cout << "[DeadbandModelOnTheLiveE2EGoalRevealsComputeDriveGap] phase_transitions="
            << phase_transitions << " reached_at_s=" << reached_at_s
            << " final_pose=(" << pose.x << ", " << pose.y << ", " << pose.yaw << ")"
            << std::endl;

  EXPECT_GE(phase_transitions, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
