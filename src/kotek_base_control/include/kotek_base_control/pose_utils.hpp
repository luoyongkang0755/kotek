#ifndef KOTEK_BASE_CONTROL__POSE_UTILS_HPP_
#define KOTEK_BASE_CONTROL__POSE_UTILS_HPP_

#include <algorithm>
#include <cmath>

namespace kotek_base_control
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Twist2D
{
  double linear = 0.0;
  double angular = 0.0;
};

/// Wraps an angle (radians) to (-pi, pi].
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Clamps `value` to [-max_abs, max_abs]. `max_abs` must be >= 0.
inline double clampAbs(double value, double max_abs)
{
  return std::clamp(value, -max_abs, max_abs);
}

/// If `value` is non-zero but smaller in magnitude than `min_abs`, boosts it
/// up to `min_abs` while preserving sign. Used to avoid commanding a
/// velocity too small to overcome static friction. `value == 0` stays 0.
inline double applyMinMagnitude(double value, double min_abs)
{
  if (value == 0.0) {
    return 0.0;
  }
  const double mag = std::fabs(value);
  if (mag >= min_abs) {
    return value;
  }
  return std::copysign(min_abs, value);
}

inline double distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

/// Bearing from `from` towards `to`, in the world frame (not relative to
/// `from`'s heading).
inline double bearingTo(const Pose2D & from, const Pose2D & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

}  // namespace kotek_base_control

#endif  // KOTEK_BASE_CONTROL__POSE_UTILS_HPP_
