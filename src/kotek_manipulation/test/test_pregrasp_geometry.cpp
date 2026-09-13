#include <cmath>

#include "gtest/gtest.h"
#include "kotek_manipulation/pregrasp_geometry.hpp"

using kotek_base_control::normalizeAngle;
using kotek_base_control::Pose2D;
using kotek_manipulation::ApproachMode;
using kotek_manipulation::computeApproachBearing;
using kotek_manipulation::computeApproachPose;
using kotek_manipulation::computeBaseGoal;
using kotek_manipulation::computeGraspPose;
using kotek_manipulation::computeLiftPose;
using kotek_manipulation::isReachable;
using kotek_manipulation::Point3;
using kotek_manipulation::quaternionFromRPY;
using kotek_manipulation::rotateVectorByQuaternion;
using kotek_manipulation::ReachabilityLimits;
using kotek_manipulation::transformPointOdomToArmFrame;

TEST(ApproachBearing, FromCurrentMatchesBearingToSensor)
{
  Pose2D robot{0.0, 0.0, 0.0};
  Pose2D sensor{1.0, 1.0, 0.0};
  const double b = computeApproachBearing(sensor, robot, ApproachMode::FROM_CURRENT, 0.0, 0.0);
  EXPECT_NEAR(b, M_PI / 4.0, 1e-9);
}

TEST(ApproachBearing, FixedModeReturnsFixedYaw)
{
  Pose2D robot{0.0, 0.0, 0.0};
  Pose2D sensor{5.0, -3.0, 0.0};
  const double b = computeApproachBearing(sensor, robot, ApproachMode::FIXED, 1.234, 0.0);
  EXPECT_NEAR(b, 1.234, 1e-9);
}

TEST(ApproachBearing, ObjectFrameAddsOffsetToSensorYaw)
{
  Pose2D robot{0.0, 0.0, 0.0};
  Pose2D sensor{5.0, -3.0, 0.5};
  const double b = computeApproachBearing(sensor, robot, ApproachMode::OBJECT_FRAME, 0.0, 0.2);
  EXPECT_NEAR(b, normalizeAngle(0.7), 1e-9);
}

TEST(BaseGoal, StandsOffAtExactDistanceAndFacesSensor)
{
  Pose2D sensor{3.0, 4.0, 0.0};
  const double bearing = 0.9;
  const double standoff = 0.4;
  const Pose2D goal = computeBaseGoal(sensor, bearing, standoff);

  EXPECT_NEAR(kotek_base_control::distance(goal, sensor), standoff, 1e-9);
  EXPECT_NEAR(goal.yaw, bearing, 1e-9);
  EXPECT_NEAR(kotek_base_control::bearingTo(goal, sensor), bearing, 1e-9);

  // Sensor must never fall inside a small base footprint radius.
  EXPECT_GT(kotek_base_control::distance(goal, sensor), 0.1);
}

TEST(TransformOdomToArmFrame, IdentityWhenRobotAtOriginAndNoMountOffset)
{
  Point3 point{2.0, -1.0, 0.3};
  Pose2D robot{0.0, 0.0, 0.0};
  Point3 mount{0.0, 0.0, 0.0};

  const Point3 out = transformPointOdomToArmFrame(point, robot, 0.0, mount);
  EXPECT_NEAR(out.x, 2.0, 1e-9);
  EXPECT_NEAR(out.y, -1.0, 1e-9);
  EXPECT_NEAR(out.z, 0.3, 1e-9);
}

TEST(TransformOdomToArmFrame, SubtractsMountOffset)
{
  Point3 point{0.0, 0.0, 0.0};
  Pose2D robot{0.0, 0.0, 0.0};
  Point3 mount{0.0, 0.0, 0.29101};

  const Point3 out = transformPointOdomToArmFrame(point, robot, 0.0, mount);
  EXPECT_NEAR(out.z, -0.29101, 1e-9);
}

TEST(TransformOdomToArmFrame, RobotYawRotatesIntoBaseFrame)
{
  // A point directly "ahead" of the robot in world coordinates (robot
  // facing +y) should land on the base frame's +x axis.
  Point3 point{0.0, 1.0, 0.0};
  Pose2D robot{0.0, 0.0, M_PI / 2.0};
  Point3 mount{0.0, 0.0, 0.0};

  const Point3 out = transformPointOdomToArmFrame(point, robot, 0.0, mount);
  EXPECT_NEAR(out.x, 1.0, 1e-9);
  EXPECT_NEAR(out.y, 0.0, 1e-9);
}

TEST(TransformOdomToArmFrame, SubtractsRobotZBeforeMountOffset)
{
  // Locks in the section 4.16 fix: base_link's own height above odom's
  // Z=0 plane (robot_z_odom, ~0.081m on the real demo robot per a live
  // `tf2_echo odom base_link`) must be subtracted BEFORE the arm mount
  // offset -- omitting it was the root cause of a real live grasp miss
  // (the gripper closed ~8cm too high every time even with the
  // orientation and finger-offset fixes both already in place).
  Point3 point{0.0, 0.0, 0.5};
  Pose2D robot{0.0, 0.0, 0.0};
  Point3 mount{0.0, 0.0, 0.29101};

  const Point3 out = transformPointOdomToArmFrame(point, robot, 0.081, mount);
  EXPECT_NEAR(out.z, 0.5 - 0.081 - 0.29101, 1e-9);
}

TEST(Reachability, AcceptsWithinBoundsRejectsOutside)
{
  ReachabilityLimits limits;
  limits.min_reach = 0.15;
  limits.max_reach = 0.55;
  limits.z_min = -0.2;
  limits.z_max = 0.5;

  EXPECT_TRUE(isReachable(Point3{0.3, 0.0, 0.0}, limits));
  EXPECT_TRUE(isReachable(Point3{limits.max_reach, 0.0, 0.0}, limits));
  EXPECT_TRUE(isReachable(Point3{limits.min_reach, 0.0, 0.0}, limits));

  EXPECT_FALSE(isReachable(Point3{0.05, 0.0, 0.0}, limits));  // too close
  EXPECT_FALSE(isReachable(Point3{0.6, 0.0, 0.0}, limits));   // too far
  EXPECT_FALSE(isReachable(Point3{0.3, 0.0, 1.0}, limits));   // z out of range
}

TEST(GraspApproachLift, ApproachIsOffsetBehindAndAboveGrasp)
{
  Point3 object{0.4, 0.0, 0.1};
  const double approach_yaw = 0.0;
  const auto grasp = computeGraspPose(object, approach_yaw, 0.3);
  const auto approach = computeApproachPose(grasp, 0.12, 0.10, approach_yaw);

  EXPECT_NEAR(approach.position.x, grasp.position.x - 0.12, 1e-9);
  EXPECT_NEAR(approach.position.y, grasp.position.y, 1e-9);
  EXPECT_NEAR(approach.position.z, grasp.position.z + 0.10, 1e-9);
  // Orientation preserved between approach and grasp.
  EXPECT_NEAR(approach.orientation.x, grasp.orientation.x, 1e-9);
  EXPECT_NEAR(approach.orientation.y, grasp.orientation.y, 1e-9);
  EXPECT_NEAR(approach.orientation.z, grasp.orientation.z, 1e-9);
  EXPECT_NEAR(approach.orientation.w, grasp.orientation.w, 1e-9);
}

TEST(GraspApproachLift, LiftRaisesZOnlyKeepingOrientation)
{
  Point3 object{0.4, 0.0, 0.1};
  const auto grasp = computeGraspPose(object, 0.0, 0.3);
  const auto lift = computeLiftPose(grasp, 0.15);

  EXPECT_NEAR(lift.position.x, grasp.position.x, 1e-9);
  EXPECT_NEAR(lift.position.y, grasp.position.y, 1e-9);
  EXPECT_NEAR(lift.position.z, grasp.position.z + 0.15, 1e-9);
  EXPECT_NEAR(lift.orientation.w, grasp.orientation.w, 1e-9);
}

TEST(GraspApproachLift, OrientationPointsLocalZAlongApproachDirection)
{
  // Locks in the fix from docs/pick_and_delivery_report.md section 4.16:
  // computeGraspPose's resulting orientation must rotate link6's local +Z
  // (confirmed via the URDF: joint7/8 sit 0.135m along link6's local +Z)
  // onto the (yaw, pitch) approach direction.
  Point3 object{0.4, 0.1, 0.05};
  const double approach_yaw = std::atan2(object.y, object.x);
  const double gripper_pitch = 0.3;
  const auto grasp = computeGraspPose(object, approach_yaw, gripper_pitch);

  const Point3 expected_approach_dir{
    std::cos(approach_yaw) * std::cos(gripper_pitch),
    std::sin(approach_yaw) * std::cos(gripper_pitch), -std::sin(gripper_pitch)};
  const Point3 local_z = rotateVectorByQuaternion(Point3{0.0, 0.0, 1.0}, grasp.orientation);

  EXPECT_NEAR(local_z.x, expected_approach_dir.x, 1e-9);
  EXPECT_NEAR(local_z.y, expected_approach_dir.y, 1e-9);
  EXPECT_NEAR(local_z.z, expected_approach_dir.z, 1e-9);
}

TEST(GraspApproachLift, OrientationKeepsLocalXOpeningAxisHorizontal)
{
  // Locks in the other half of the section 4.16 fix: computeGraspPose must
  // rotate link6's local +X (confirmed via a live /piper/compute_fk sweep
  // of joint7/joint8 to be the finger-opening axis) to a HORIZONTAL
  // direction (world z == 0), regardless of approach_yaw -- otherwise the
  // fingers can't straddle an upright object symmetrically (the bug a live
  // screenshot, grasp.png, showed: the pre-fix formula only guaranteed
  // IK-reachability, not this).
  for (const double approach_yaw : {-1.2, -0.3, 0.0, 0.4, 1.0}) {
    Point3 object{0.4 * std::cos(approach_yaw), 0.4 * std::sin(approach_yaw), 0.05};
    const auto grasp = computeGraspPose(object, approach_yaw, 0.4);
    const Point3 local_x = rotateVectorByQuaternion(Point3{1.0, 0.0, 0.0}, grasp.orientation);
    EXPECT_NEAR(local_x.z, 0.0, 1e-9) << "approach_yaw=" << approach_yaw;
  }
}

TEST(GraspApproachLift, FingertipsNotLink6OriginLandOnObject)
{
  // Locks in the section 4.16 position fix: computeGraspPose's returned
  // position targets link6's OWN origin (it feeds moveArmToPose, whose
  // group tip_link is link6, per the SRDF), but the fingers sit 0.13503m
  // further along link6's local +Z (== the approach direction, once
  // oriented) -- NOT at link6's origin itself. A real live grasp attempt
  // (docs/pick_and_delivery_report.md section 4.16) missed the object on
  // every try because the pre-fix formula put link6's origin AT the
  // object, leaving the actual fingertips ~13.5cm past it. Walking
  // 0.13503m further along the returned pose's local +Z must land
  // exactly on the object, regardless of approach_yaw/gripper_pitch.
  for (const double approach_yaw : {-1.2, -0.3, 0.0, 0.4, 1.0}) {
    for (const double gripper_pitch : {0.0, 0.3, 0.5}) {
      Point3 object{0.4 * std::cos(approach_yaw), 0.4 * std::sin(approach_yaw), 0.05};
      const auto grasp = computeGraspPose(object, approach_yaw, gripper_pitch);
      const Point3 local_z = rotateVectorByQuaternion(Point3{0.0, 0.0, 1.0}, grasp.orientation);
      const Point3 fingertip{
        grasp.position.x + 0.13503 * local_z.x, grasp.position.y + 0.13503 * local_z.y,
        grasp.position.z + 0.13503 * local_z.z};
      EXPECT_NEAR(fingertip.x, object.x, 1e-9) << "yaw=" << approach_yaw << " pitch=" <<
        gripper_pitch;
      EXPECT_NEAR(fingertip.y, object.y, 1e-9) << "yaw=" << approach_yaw << " pitch=" <<
        gripper_pitch;
      EXPECT_NEAR(fingertip.z, object.z, 1e-9) << "yaw=" << approach_yaw << " pitch=" <<
        gripper_pitch;
    }
  }
}

TEST(QuaternionFromRPY, IdentityAtZero)
{
  const auto q = quaternionFromRPY(0.0, 0.0, 0.0);
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.z, 0.0, 1e-9);
  EXPECT_NEAR(q.w, 1.0, 1e-9);
}

TEST(QuaternionFromRPY, YawNinetyDegreesMatchesKnownQuaternion)
{
  const auto q = quaternionFromRPY(0.0, 0.0, M_PI / 2.0);
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.z, std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(q.w, std::cos(M_PI / 4.0), 1e-9);
  // Must be a unit quaternion.
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
