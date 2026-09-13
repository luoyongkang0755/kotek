#ifndef KOTEK_BASE_CONTROL__SIMPLE_BASE_CONTROLLER_HPP_
#define KOTEK_BASE_CONTROL__SIMPLE_BASE_CONTROLLER_HPP_

#include "kotek_base_control/pose_utils.hpp"

namespace kotek_base_control
{

enum class ControlPhase
{
  TURN_TO_HEADING,
  DRIVE,
  ALIGN,
  STOPPED,
};

const char * toString(ControlPhase phase);

struct BaseControllerParams
{
  double max_linear_velocity = 0.5;
  double min_linear_velocity = 0.05;
  double max_angular_velocity = 0.8;
  double min_angular_velocity = 0.1;
  double max_linear_accel = 0.5;
  double k_linear = 0.8;
  double k_angular = 1.5;
  double xy_tolerance = 0.08;
  double yaw_tolerance = 0.10;
  double heading_tolerance = 0.15;
};

/// Pure geometric control law for driving a differential/skid-steer base to
/// a 2D goal pose. Holds no ROS types and no wall-clock state, so it is
/// unit-testable by feeding it poses directly. See plan section 7.
///
/// Phase machine: TURN_TO_HEADING -> DRIVE -> ALIGN -> STOPPED, with
/// hysteresis re-entry from DRIVE back to TURN_TO_HEADING if the heading
/// error grows past 2x heading_tolerance mid-drive.
class SimpleBaseController
{
public:
  explicit SimpleBaseController(const BaseControllerParams & params);

  void setParams(const BaseControllerParams & params) {params_ = params;}
  const BaseControllerParams & params() const {return params_;}

  /// Sets a new goal and resets the phase to TURN_TO_HEADING.
  void setGoal(const Pose2D & goal);

  /// Computes the next command for `current` pose, advancing the internal
  /// phase. Returns a zero Twist2D once STOPPED is reached (call
  /// repeatedly; it stays zero and phase() stays STOPPED).
  Twist2D step(const Pose2D & current);

  ControlPhase phase() const {return phase_;}
  bool isGoalReached() const {return phase_ == ControlPhase::STOPPED;}

  double distanceRemaining(const Pose2D & current) const;

private:
  Twist2D computeTurnToHeading(const Pose2D & current, double theta_err) const;
  Twist2D computeDrive(const Pose2D & current, double rho, double theta_err) const;
  Twist2D computeAlign(double yaw_err) const;

  BaseControllerParams params_;
  Pose2D goal_;
  ControlPhase phase_ = ControlPhase::TURN_TO_HEADING;
};

}  // namespace kotek_base_control

#endif  // KOTEK_BASE_CONTROL__SIMPLE_BASE_CONTROLLER_HPP_
