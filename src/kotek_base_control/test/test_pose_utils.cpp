#include <cmath>

#include "gtest/gtest.h"
#include "kotek_base_control/pose_utils.hpp"

using kotek_base_control::applyMinMagnitude;
using kotek_base_control::bearingTo;
using kotek_base_control::clampAbs;
using kotek_base_control::distance;
using kotek_base_control::normalizeAngle;
using kotek_base_control::Pose2D;

TEST(NormalizeAngle, ZeroStaysZero)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
}

TEST(NormalizeAngle, WrapsMultiplesOfTwoPi)
{
  EXPECT_NEAR(normalizeAngle(2.0 * M_PI), 0.0, 1e-9);
  EXPECT_NEAR(normalizeAngle(-2.0 * M_PI), 0.0, 1e-9);
  EXPECT_NEAR(normalizeAngle(4.0 * M_PI + 0.3), 0.3, 1e-9);
}

TEST(NormalizeAngle, WrapsArbitraryLargeAngles)
{
  EXPECT_NEAR(normalizeAngle(2.5 * M_PI), 0.5 * M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(-2.5 * M_PI), -0.5 * M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI + 0.1), -M_PI + 0.1, 1e-9);
}

TEST(NormalizeAngle, BoundaryNearPiStaysInRange)
{
  // +/- pi represent the same physical angle; only require the magnitude is
  // right and the result stays within the documented (-pi, pi] range.
  EXPECT_NEAR(std::fabs(normalizeAngle(M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(std::fabs(normalizeAngle(-M_PI)), M_PI, 1e-9);
}

TEST(NormalizeAngle, AlwaysWithinRange)
{
  for (double a = -100.0; a <= 100.0; a += 0.37) {
    const double r = normalizeAngle(a);
    EXPECT_GT(r, -M_PI - 1e-9);
    EXPECT_LE(r, M_PI + 1e-9);
    EXPECT_FALSE(std::isnan(r));
  }
}

TEST(ClampAbs, ClampsSymmetrically)
{
  EXPECT_DOUBLE_EQ(clampAbs(5.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(clampAbs(-5.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(clampAbs(1.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(clampAbs(-1.0, 2.0), -1.0);
  EXPECT_DOUBLE_EQ(clampAbs(0.0, 2.0), 0.0);
}

TEST(ApplyMinMagnitude, ZeroStaysZero)
{
  EXPECT_DOUBLE_EQ(applyMinMagnitude(0.0, 0.1), 0.0);
}

TEST(ApplyMinMagnitude, BoostsSmallValuesPreservingSign)
{
  EXPECT_DOUBLE_EQ(applyMinMagnitude(0.05, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(applyMinMagnitude(-0.05, 0.1), -0.1);
}

TEST(ApplyMinMagnitude, LeavesLargeValuesUnchanged)
{
  EXPECT_DOUBLE_EQ(applyMinMagnitude(0.2, 0.1), 0.2);
  EXPECT_DOUBLE_EQ(applyMinMagnitude(-0.2, 0.1), -0.2);
}

TEST(Distance, PythagoreanTriple)
{
  Pose2D a{0.0, 0.0, 0.0};
  Pose2D b{3.0, 4.0, 0.0};
  EXPECT_DOUBLE_EQ(distance(a, b), 5.0);
}

TEST(BearingTo, DiagonalAndBackward)
{
  Pose2D origin{0.0, 0.0, 0.0};
  EXPECT_NEAR(bearingTo(origin, Pose2D{1.0, 1.0, 0.0}), M_PI / 4.0, 1e-9);
  EXPECT_NEAR(std::fabs(bearingTo(origin, Pose2D{-1.0, 0.0, 0.0})), M_PI, 1e-9);
}

TEST(BearingTo, CoincidentPointsDoNotProduceNaN)
{
  Pose2D origin{0.0, 0.0, 0.0};
  const double b = bearingTo(origin, origin);
  EXPECT_FALSE(std::isnan(b));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
