#include "gtest/gtest.h"
#include "kotek_base_control/obstacle_avoidance.hpp"

using kotek_base_control::applyObstacleAvoidance;
using kotek_base_control::LaserScanSummary;
using kotek_base_control::ObstacleAvoidanceParams;
using kotek_base_control::Twist2D;

namespace
{
ObstacleAvoidanceParams defaultParams()
{
  ObstacleAvoidanceParams p;
  p.safety_distance = 0.6;
  p.stop_distance = 0.25;
  p.steer_gain = 1.0;
  p.max_steer_angular = 0.6;
  p.max_angular_velocity = 1.2;
  return p;
}
}  // namespace

TEST(ObstacleAvoidance, ClearPathPassesThroughUnchanged)
{
  const Twist2D desired{0.4, 0.1};
  LaserScanSummary scan;
  scan.min_range_forward = 5.0;
  scan.min_range_left = 5.0;
  scan.min_range_right = 5.0;
  const Twist2D out = applyObstacleAvoidance(desired, scan, defaultParams());
  EXPECT_DOUBLE_EQ(out.linear, desired.linear);
  EXPECT_DOUBLE_EQ(out.angular, desired.angular);
}

TEST(ObstacleAvoidance, NonPositiveLinearIsNeverGuarded)
{
  // Stopped or already reversing -- forward-facing lidar can't see behind,
  // and there is nothing to slow down further.
  LaserScanSummary scan;
  scan.min_range_forward = 0.05;  // right on top of an obstacle
  scan.min_range_left = 0.05;
  scan.min_range_right = 0.05;

  const Twist2D stopped{0.0, 0.3};
  EXPECT_DOUBLE_EQ(applyObstacleAvoidance(stopped, scan, defaultParams()).linear, 0.0);
  EXPECT_DOUBLE_EQ(applyObstacleAvoidance(stopped, scan, defaultParams()).angular, 0.3);

  const Twist2D reversing{-0.3, 0.0};
  const Twist2D out = applyObstacleAvoidance(reversing, scan, defaultParams());
  EXPECT_DOUBLE_EQ(out.linear, -0.3);
  EXPECT_DOUBLE_EQ(out.angular, 0.0);
}

TEST(ObstacleAvoidance, StopsForwardMotionAtOrBelowStopDistance)
{
  const Twist2D desired{0.5, 0.0};
  LaserScanSummary scan;
  scan.min_range_left = 5.0;
  scan.min_range_right = 5.0;

  scan.min_range_forward = 0.25;  // exactly at stop_distance
  EXPECT_DOUBLE_EQ(applyObstacleAvoidance(desired, scan, defaultParams()).linear, 0.0);

  scan.min_range_forward = 0.10;  // well inside stop_distance
  EXPECT_DOUBLE_EQ(applyObstacleAvoidance(desired, scan, defaultParams()).linear, 0.0);
}

TEST(ObstacleAvoidance, ScalesDownLinearlyBetweenStopAndSafetyDistance)
{
  const Twist2D desired{1.0, 0.0};
  LaserScanSummary scan;
  scan.min_range_left = 5.0;
  scan.min_range_right = 5.0;
  // Halfway between stop_distance (0.25) and safety_distance (0.6) -> scale 0.5.
  scan.min_range_forward = 0.425;
  const Twist2D out = applyObstacleAvoidance(desired, scan, defaultParams());
  EXPECT_NEAR(out.linear, 0.5, 1e-9);
}

TEST(ObstacleAvoidance, NoScalingAtOrBeyondSafetyDistance)
{
  const Twist2D desired{0.4, 0.0};
  LaserScanSummary scan;
  scan.min_range_left = 5.0;
  scan.min_range_right = 5.0;
  scan.min_range_forward = 0.6;  // exactly at safety_distance -- not yet guarded
  const Twist2D out = applyObstacleAvoidance(desired, scan, defaultParams());
  EXPECT_DOUBLE_EQ(out.linear, desired.linear);
}

TEST(ObstacleAvoidance, SteersTowardMoreClearanceWhenGuarding)
{
  const Twist2D desired{0.4, 0.0};
  LaserScanSummary scan;
  scan.min_range_forward = 0.4;  // inside safety_distance -> guarding active
  scan.min_range_left = 2.0;
  scan.min_range_right = 0.3;
  const Twist2D out = applyObstacleAvoidance(desired, scan, defaultParams());
  // More clearance on the left -> steer left (positive angular, REP-103).
  EXPECT_GT(out.angular, 0.0);
}

TEST(ObstacleAvoidance, SteerContributionIsClampedByMaxSteerAngular)
{
  const Twist2D desired{0.4, 0.0};
  LaserScanSummary scan;
  scan.min_range_forward = 0.4;
  scan.min_range_left = 100.0;  // extreme clearance imbalance
  scan.min_range_right = 0.0;
  ObstacleAvoidanceParams p = defaultParams();
  p.max_steer_angular = 0.3;
  p.max_angular_velocity = 10.0;  // large so it doesn't also clamp here
  const Twist2D out = applyObstacleAvoidance(desired, scan, p);
  EXPECT_NEAR(out.angular, 0.3, 1e-9);
}

TEST(ObstacleAvoidance, FinalAngularIsClampedByMaxAngularVelocity)
{
  const Twist2D desired{0.4, 1.0};  // already near max on its own
  LaserScanSummary scan;
  scan.min_range_forward = 0.4;
  scan.min_range_left = 100.0;
  scan.min_range_right = 0.0;
  ObstacleAvoidanceParams p = defaultParams();
  p.max_steer_angular = 1.0;
  p.max_angular_velocity = 1.2;
  const Twist2D out = applyObstacleAvoidance(desired, scan, p);
  EXPECT_LE(out.angular, p.max_angular_velocity + 1e-9);
}

TEST(ObstacleAvoidance, SymmetricClearanceProducesNoSteerBias)
{
  const Twist2D desired{0.4, 0.0};
  LaserScanSummary scan;
  scan.min_range_forward = 0.4;
  scan.min_range_left = 1.0;
  scan.min_range_right = 1.0;
  const Twist2D out = applyObstacleAvoidance(desired, scan, defaultParams());
  EXPECT_NEAR(out.angular, 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
