#ifndef KOTEK_MANIPULATION__PREGRASP_GEOMETRY_HPP_
#define KOTEK_MANIPULATION__PREGRASP_GEOMETRY_HPP_

#include <cmath>

#include "kotek_base_control/pose_utils.hpp"

// Pure, ROS-free geometry shared by two consumers (plan sections 6, 8, 9):
//   - kotek_task_coordinator uses the 2D base-standoff functions to compute
//     COMPUTE_BASE_PREGRASP_POSE, and the odom->arm-frame transform to
//     express the object pose in the Piper's own base_link frame before
//     sending a GraspObject goal (so piper_manipulator never needs a
//     cross-tree TF lookup of the scout's pose -- see plan section 6).
//   - kotek_manipulation's piper_manipulator uses the 3D approach/grasp/lift
//     pose functions to build the Cartesian waypoints for the arm, entirely
//     within the arm's own frame.
namespace kotek_manipulation
{

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose3
{
  Point3 position;
  Quaternion orientation;
};

enum class ApproachMode
{
  FROM_CURRENT,
  FIXED,
  OBJECT_FRAME,
};

/// ZYX Euler (roll, pitch, yaw) to quaternion.
inline Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

/// Hamilton product q1 * q2 (apply q2's rotation first, then q1's).
inline Quaternion quaternionMultiply(const Quaternion & q1, const Quaternion & q2)
{
  Quaternion q;
  q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return q;
}

inline Point3 vecCross(const Point3 & a, const Point3 & b)
{
  return Point3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Point3 vecNormalize(const Point3 & v)
{
  const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  return Point3{v.x / n, v.y / n, v.z / n};
}

/// Rotates `v` by unit quaternion `q` (i.e. q * v * q^-1, computed via the
/// standard expanded/Rodrigues form rather than full quaternion products).
/// Mainly useful for tests -- lets a test assert "this quaternion points
/// local axis A at world direction D" directly, instead of comparing raw
/// quaternion components.
inline Point3 rotateVectorByQuaternion(const Point3 & v, const Quaternion & q)
{
  const Point3 qv{q.x, q.y, q.z};
  const Point3 cross1 = vecCross(qv, v);
  const Point3 t{2.0 * cross1.x, 2.0 * cross1.y, 2.0 * cross1.z};
  const Point3 cross_qv_t = vecCross(qv, t);
  return Point3{
    v.x + q.w * t.x + cross_qv_t.x, v.y + q.w * t.y + cross_qv_t.y,
    v.z + q.w * t.z + cross_qv_t.z};
}

/// Builds the quaternion that rotates the identity frame's local
/// (+X, +Y, +Z) axes onto (`x_axis`, `y_axis`, `z_axis`) -- i.e. the three
/// arguments are the columns of the desired rotation matrix, each a unit
/// vector expressed in the parent frame, forming a right-handed basis
/// (`x_axis` x `y_axis` == `z_axis`). Standard trace/Shepperd's-method
/// matrix-to-quaternion conversion.
inline Quaternion quaternionFromAxes(
  const Point3 & x_axis, const Point3 & y_axis, const Point3 & z_axis)
{
  const double m00 = x_axis.x, m10 = x_axis.y, m20 = x_axis.z;
  const double m01 = y_axis.x, m11 = y_axis.y, m21 = y_axis.z;
  const double m02 = z_axis.x, m12 = z_axis.y, m22 = z_axis.z;
  const double trace = m00 + m11 + m22;

  Quaternion q;
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    q.w = 0.25 / s;
    q.x = (m21 - m12) * s;
    q.y = (m02 - m20) * s;
    q.z = (m10 - m01) * s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }
  return q;
}

// ---------------------------------------------------------------------
// Base standoff geometry (2D, plan section 8).
// ---------------------------------------------------------------------

/// Bearing the base should face while approaching the sensor, per mode.
inline double computeApproachBearing(
  const kotek_base_control::Pose2D & sensor, const kotek_base_control::Pose2D & robot,
  ApproachMode mode, double fixed_yaw, double object_yaw_offset)
{
  switch (mode) {
    case ApproachMode::FIXED:
      return kotek_base_control::normalizeAngle(fixed_yaw);
    case ApproachMode::OBJECT_FRAME:
      return kotek_base_control::normalizeAngle(sensor.yaw + object_yaw_offset);
    case ApproachMode::FROM_CURRENT:
    default:
      return kotek_base_control::bearingTo(robot, sensor);
  }
}

/// Base goal pose: standing off `standoff` meters behind the sensor along
/// `approach_bearing`, facing the sensor. Deliberately not the sensor's own
/// pose -- the object must never fall inside the base footprint.
inline kotek_base_control::Pose2D computeBaseGoal(
  const kotek_base_control::Pose2D & sensor, double approach_bearing, double standoff)
{
  kotek_base_control::Pose2D goal;
  goal.x = sensor.x - standoff * std::cos(approach_bearing);
  goal.y = sensor.y - standoff * std::sin(approach_bearing);
  goal.yaw = approach_bearing;
  return goal;
}

/// Expresses a point known in `odom` in the Piper arm's own base frame,
/// given the scout's current pose in odom, the scout base_link's own
/// height above odom's Z=0 plane (`robot_z_odom` -- see below), and the
/// constant arm mount offset (translation only -- root_joint's localRot0
/// is identity, see plan section 1.3/1.8).
///
/// `robot_z_odom` matters: `robot_pose_odom` is a `Pose2D` (x, y, yaw --
/// by design, 2D navigation never needs more), so without a separate Z
/// term this function has no way to know base_link sits above odom's Z=0
/// plane at all, and silently treated it as exactly 0. A live
/// `tf2_echo odom base_link` on the real demo robot showed base_link's
/// actual height is ~0.081m, not 0 -- confirmed as the root cause of a
/// real live grasp miss (docs/pick_and_delivery_report.md section 4.16):
/// even with the orientation and finger-offset fixes both in place and
/// verified, the gripper closed ~8cm too high every time, because the
/// object's z was being placed directly into base_link's frame
/// unmodified instead of first subtracting base_link's own height.
/// Callers should pass the SAME live TF-sourced Z used to look up
/// `robot_pose_odom` in the first place (see
/// TaskCoordinatorNode::lookupRobotPose()'s z_out parameter), not a
/// hardcoded constant -- keeps this correct even if the chassis's resting
/// height ever changes (e.g. a different robot model).
inline Point3 transformPointOdomToArmFrame(
  const Point3 & point_odom, const kotek_base_control::Pose2D & robot_pose_odom,
  double robot_z_odom, const Point3 & arm_mount_xyz)
{
  const double dx = point_odom.x - robot_pose_odom.x;
  const double dy = point_odom.y - robot_pose_odom.y;
  const double c = std::cos(robot_pose_odom.yaw);
  const double s = std::sin(robot_pose_odom.yaw);

  Point3 base_link_frame;
  base_link_frame.x = c * dx + s * dy;
  base_link_frame.y = -s * dx + c * dy;
  base_link_frame.z = point_odom.z - robot_z_odom;

  Point3 arm_frame;
  arm_frame.x = base_link_frame.x - arm_mount_xyz.x;
  arm_frame.y = base_link_frame.y - arm_mount_xyz.y;
  arm_frame.z = base_link_frame.z - arm_mount_xyz.z;
  return arm_frame;
}

struct ReachabilityLimits
{
  double min_reach = 0.15;
  double max_reach = 0.55;
  double z_min = -0.20;
  double z_max = 0.50;
};

/// Gate used before driving: is `point_arm_frame` within the arm's usable
/// envelope? See plan section 8, item 3.
inline bool isReachable(const Point3 & point_arm_frame, const ReachabilityLimits & limits)
{
  const double r = std::sqrt(
    point_arm_frame.x * point_arm_frame.x + point_arm_frame.y * point_arm_frame.y +
    point_arm_frame.z * point_arm_frame.z);
  return r >= limits.min_reach && r <= limits.max_reach &&
         point_arm_frame.z >= limits.z_min && point_arm_frame.z <= limits.z_max;
}

// ---------------------------------------------------------------------
// Arm-frame grasp/approach/lift poses (3D, plan section 9).
// ---------------------------------------------------------------------

/// The gripper's target pose at the object itself: positioned at the
/// object, oriented to approach along `approach_yaw` with a downward tilt
/// of `gripper_pitch` (radians, positive = nose-down).
///
/// This arm's actual tip link (link6, see the SRDF's
/// `<chain tip_link="link6"/>`) has two facts that fully determine the
/// required orientation, both confirmed directly against the live robot
/// (not assumed): (1) `piper_description.urdf` places joint7/8 (the
/// gripper fingers) at `xyz="0 0 0.13503"` relative to link6 -- the
/// fingers, and so the object once grasped, sit along link6's **local
/// +Z** -- so +Z is the approach/insertion axis, not +X (a common but
/// wrong-for-this-arm convention). (2) A live `/piper/compute_fk` sweep of
/// joint7/joint8 (holding all other joints fixed) showed link7/link8
/// moving to `local (-w, 0, 0.135)` / `(+w, 0, 0.135)` as the commanded
/// half-width `w` increases -- the fingers separate along link6's
/// **local +X**. That is the opening axis.
///
/// An earlier fix (see docs/pick_and_delivery_report.md section 4.15) only
/// used fact (1), composing a single fixed `Ry(+90deg)` correction onto
/// `quaternionFromRPY(0, pitch, yaw)`. That made the target IK-reachable
/// (confirmed via `/piper/compute_ik`) but NOT necessarily grasp-correct:
/// a live screenshot (`grasp.png`, reported by the user) showed the
/// gripper arriving twisted relative to the object -- the fixed correction
/// happened to solve fact (1) but said nothing about fact (2), so the
/// opening axis could end up pointing anywhere (e.g. vertically, unable to
/// straddle the object) depending on `approach_yaw`.
///
/// Fixed properly here using both facts together, built directly as a
/// look-at frame instead of composed Euler corrections:
///   - `approach_dir` (what should become link6's local +Z) is the
///     `(yaw, pitch)` pointing direction, same as before.
///   - `opening_axis` (what should become link6's local +X) is chosen
///     horizontal -- `normalize(cross(world_up, approach_dir))` -- so the
///     fingers always open in a level, side-to-side plane regardless of
///     `approach_yaw`, letting them straddle an upright object (like the
///     demo's sensor cube) instead of one finger going above/below it.
///   - the third axis (link6's local +Y) is whatever completes a
///     right-handed basis: `cross(approach_dir, opening_axis)`.
/// `quaternionFromAxes()` converts that basis directly to a quaternion --
/// no Euler composition, no guessing which fixed correction happens to
/// work for one tested `approach_yaw`.
inline Pose3 computeGraspPose(
  const Point3 & object_position_arm_frame, double approach_yaw, double gripper_pitch)
{
  const Point3 approach_dir = vecNormalize(Point3{
      std::cos(approach_yaw) * std::cos(gripper_pitch),
      std::sin(approach_yaw) * std::cos(gripper_pitch),
      -std::sin(gripper_pitch)});
  const Point3 world_up{0.0, 0.0, 1.0};
  const Point3 opening_axis = vecNormalize(vecCross(world_up, approach_dir));
  const Point3 third_axis = vecCross(approach_dir, opening_axis);

  Pose3 pose;
  // The returned pose targets link6's OWN origin (it feeds moveArmToPose,
  // which commands the arm group whose SRDF tip_link is link6) -- but the
  // fingers (joint7/8) sit 0.13503m further along link6's local +Z
  // (== approach_dir once the orientation below is applied; see this
  // function's doc comment), not at link6's origin itself. Confirmed
  // directly as the root cause of a real live-run grasp miss (docs/
  // pick_and_delivery_report.md section 4.16): setting pose.position to
  // the object's raw position put link6's origin AT the object, which put
  // the fingertips ~13.5cm PAST it, closing on empty air every time even
  // though the orientation fix (opening axis horizontal, approach axis
  // pointing at the object) was already correct. Back link6's target off
  // by that same offset along approach_dir so the FINGERTIPS -- not
  // link6's origin -- land on the object.
  constexpr double kFingertipOffsetFromLink6 = 0.13503;  // meters, piper_description.urdf
  pose.position.x = object_position_arm_frame.x - kFingertipOffsetFromLink6 * approach_dir.x;
  pose.position.y = object_position_arm_frame.y - kFingertipOffsetFromLink6 * approach_dir.y;
  pose.position.z = object_position_arm_frame.z - kFingertipOffsetFromLink6 * approach_dir.z;

  pose.orientation = quaternionFromAxes(opening_axis, third_axis, approach_dir);
  return pose;
}

/// Pre-grasp: back off `approach_distance` along the approach direction and
/// up by `approach_height`, same orientation as the grasp pose.
inline Pose3 computeApproachPose(
  const Pose3 & grasp_pose, double approach_distance, double approach_height,
  double approach_yaw)
{
  Pose3 pose = grasp_pose;
  pose.position.x -= approach_distance * std::cos(approach_yaw);
  pose.position.y -= approach_distance * std::sin(approach_yaw);
  pose.position.z += approach_height;
  return pose;
}

/// Lift: same xy and orientation as the grasp pose, raised by `lift_height`.
inline Pose3 computeLiftPose(const Pose3 & grasp_pose, double lift_height)
{
  Pose3 pose = grasp_pose;
  pose.position.z += lift_height;
  return pose;
}

}  // namespace kotek_manipulation

#endif  // KOTEK_MANIPULATION__PREGRASP_GEOMETRY_HPP_
