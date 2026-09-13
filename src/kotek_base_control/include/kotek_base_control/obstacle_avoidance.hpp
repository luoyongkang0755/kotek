#ifndef KOTEK_BASE_CONTROL__OBSTACLE_AVOIDANCE_HPP_
#define KOTEK_BASE_CONTROL__OBSTACLE_AVOIDANCE_HPP_

#include "kotek_base_control/pose_utils.hpp"

namespace kotek_base_control
{

/// Minimum range (meters) seen in each of three cones of a forward-facing
/// lidar scan: straight ahead, and to each side. Holds no ROS types --
/// computed from a raw sensor_msgs/LaserScan in the node layer (which knows
/// angle_min/angle_increment) and fed into applyObstacleAvoidance() below.
/// A cone with no return within the scan's max range should be reported as
/// that max range (not infinity), so callers get a real, boundable number.
struct LaserScanSummary
{
  double min_range_forward = 1000.0;
  double min_range_left = 1000.0;
  double min_range_right = 1000.0;
};

struct ObstacleAvoidanceParams
{
  /// Forward speed is scaled down once min_range_forward drops below this.
  double safety_distance = 0.6;
  /// Forward speed is forced to zero once min_range_forward drops to this
  /// or below (must be < safety_distance).
  double stop_distance = 0.25;
  /// Angular correction (rad/s) applied per meter of clearance difference
  /// between the left and right cones, steering toward the clearer side.
  double steer_gain = 1.0;
  /// Caps the steering correction's own contribution to angular velocity.
  double max_steer_angular = 0.6;
  /// Caps the final (desired + steer) angular velocity magnitude.
  double max_angular_velocity = 1.2;
};

/// Reactive safety layer: given the base controller's desired command and a
/// summary of the current forward-facing scan, returns an adjusted command
/// that slows/stops for a close obstacle ahead and steers toward whichever
/// side has more clearance. Pure function, no ROS types, no wall-clock
/// state -- unit-testable by feeding it scans directly, same pattern as
/// SimpleBaseController's own pure control law.
///
/// Only forward motion (desired.linear > 0) is guarded: the lidar faces
/// forward only, so a non-positive desired linear velocity (stopped or
/// already reversing away from whatever is ahead) passes through
/// unchanged.
inline Twist2D applyObstacleAvoidance(
  const Twist2D & desired, const LaserScanSummary & scan, const ObstacleAvoidanceParams & params)
{
  Twist2D out = desired;

  if (desired.linear > 0.0 && scan.min_range_forward < params.safety_distance) {
    if (scan.min_range_forward <= params.stop_distance) {
      out.linear = 0.0;
    } else {
      const double span = params.safety_distance - params.stop_distance;
      const double scale = span > 0.0 ?
        (scan.min_range_forward - params.stop_distance) / span : 0.0;
      out.linear = desired.linear * std::clamp(scale, 0.0, 1.0);
    }

    const double clearance_diff = scan.min_range_left - scan.min_range_right;
    const double steer = clampAbs(clearance_diff * params.steer_gain, params.max_steer_angular);
    out.angular = clampAbs(desired.angular + steer, params.max_angular_velocity);
  }

  return out;
}

}  // namespace kotek_base_control

#endif  // KOTEK_BASE_CONTROL__OBSTACLE_AVOIDANCE_HPP_
