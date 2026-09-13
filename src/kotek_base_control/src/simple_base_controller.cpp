#include "kotek_base_control/simple_base_controller.hpp"

#include <algorithm>
#include <cmath>

namespace kotek_base_control
{

const char * toString(ControlPhase phase)
{
  switch (phase) {
    case ControlPhase::TURN_TO_HEADING:
      return "TURN_TO_HEADING";
    case ControlPhase::DRIVE:
      return "DRIVE";
    case ControlPhase::ALIGN:
      return "ALIGN";
    case ControlPhase::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

SimpleBaseController::SimpleBaseController(const BaseControllerParams & params)
: params_(params)
{
}

void SimpleBaseController::setGoal(const Pose2D & goal)
{
  goal_ = goal;
  phase_ = ControlPhase::TURN_TO_HEADING;
}

double SimpleBaseController::distanceRemaining(const Pose2D & current) const
{
  return distance(current, goal_);
}

Twist2D SimpleBaseController::computeTurnToHeading(
  const Pose2D & /*current*/, double theta_err) const
{
  double w = clampAbs(params_.k_angular * theta_err, params_.max_angular_velocity);
  w = applyMinMagnitude(w, params_.min_angular_velocity);
  return Twist2D{0.0, w};
}

Twist2D SimpleBaseController::computeDrive(
  const Pose2D & /*current*/, double rho, double theta_err) const
{
  double v = std::clamp(
    params_.k_linear * rho, params_.min_linear_velocity, params_.max_linear_velocity);
  v *= std::max(0.0, std::cos(theta_err));

  const double decel_dist = std::max(0.0, rho - params_.xy_tolerance);
  const double v_decel = std::sqrt(2.0 * params_.max_linear_accel * decel_dist);
  v = std::min(v, v_decel);
  v = std::max(v, 0.0);

  const double w = clampAbs(params_.k_angular * theta_err, params_.max_angular_velocity);
  return Twist2D{v, w};
}

Twist2D SimpleBaseController::computeAlign(double yaw_err) const
{
  double w = clampAbs(params_.k_angular * yaw_err, params_.max_angular_velocity);
  w = applyMinMagnitude(w, params_.min_angular_velocity);
  return Twist2D{0.0, w};
}

Twist2D SimpleBaseController::step(const Pose2D & current)
{
  const double rho = distance(current, goal_);
  const double bearing = bearingTo(current, goal_);
  const double theta_err = normalizeAngle(bearing - current.yaw);
  const double yaw_err = normalizeAngle(goal_.yaw - current.yaw);

  switch (phase_) {
    case ControlPhase::TURN_TO_HEADING:
      if (rho <= params_.xy_tolerance) {
        phase_ = ControlPhase::ALIGN;
        return computeAlign(yaw_err);
      }
      if (std::fabs(theta_err) <= params_.heading_tolerance) {
        phase_ = ControlPhase::DRIVE;
        return computeDrive(current, rho, theta_err);
      }
      return computeTurnToHeading(current, theta_err);

    case ControlPhase::DRIVE:
      if (rho <= params_.xy_tolerance) {
        phase_ = ControlPhase::ALIGN;
        return computeAlign(yaw_err);
      }
      if (std::fabs(theta_err) > params_.heading_tolerance * 2.0) {
        phase_ = ControlPhase::TURN_TO_HEADING;
        return computeTurnToHeading(current, theta_err);
      }
      return computeDrive(current, rho, theta_err);

    case ControlPhase::ALIGN:
      if (std::fabs(yaw_err) <= params_.yaw_tolerance) {
        phase_ = ControlPhase::STOPPED;
        return Twist2D{};
      }
      return computeAlign(yaw_err);

    case ControlPhase::STOPPED:
    default:
      return Twist2D{};
  }
}

}  // namespace kotek_base_control
