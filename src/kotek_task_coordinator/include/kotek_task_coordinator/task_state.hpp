#ifndef KOTEK_TASK_COORDINATOR__TASK_STATE_HPP_
#define KOTEK_TASK_COORDINATOR__TASK_STATE_HPP_

#include <string>

namespace kotek_task_coordinator
{

// Exactly the flow requested in the project brief (plan section 10), plus
// the delivery leg (pick up -> drive to a delivery waypoint -> place ->
// release -> retreat) added on top of it. LIFT_OBJECT's success no longer
// leads straight to DONE -- it leads into DRIVE_TO_DELIVERY, mirroring the
// pickup leg's own DRIVE_TO_PREGRASP -> ALIGN_BASE -> STOP_BASE ->
// (arm states) shape exactly, just with a fixed delivery waypoint instead
// of one computed from a sensed object pose.
enum class TaskState
{
  IDLE,
  FIND_SENSOR,
  COMPUTE_BASE_PREGRASP_POSE,
  DRIVE_TO_PREGRASP,
  ALIGN_BASE,
  STOP_BASE,
  MOVE_ARM_TO_PREGRASP,
  MOVE_ARM_TO_GRASP,
  CLOSE_GRIPPER,
  LIFT_OBJECT,
  // Retracts the arm (still holding the object) to a lidar-clear pose
  // before driving -- see GraspObject.action's doc comment (section
  // 4.16). Without this, DRIVE_TO_DELIVERY's own reactive obstacle
  // avoidance (kotek_base_control) reads the raised arm/object as an
  // obstacle directly ahead and never lets the base move.
  STOW_OBJECT,
  DRIVE_TO_DELIVERY,
  ALIGN_AT_DELIVERY,
  STOP_BASE_AT_DELIVERY,
  PLACE_ARM,
  OPEN_GRIPPER,
  RETREAT_ARM,
  ABORTING,
  DONE,
  FAILED,
};

inline const char * toString(TaskState state)
{
  switch (state) {
    case TaskState::IDLE:
      return "IDLE";
    case TaskState::FIND_SENSOR:
      return "FIND_SENSOR";
    case TaskState::COMPUTE_BASE_PREGRASP_POSE:
      return "COMPUTE_BASE_PREGRASP_POSE";
    case TaskState::DRIVE_TO_PREGRASP:
      return "DRIVE_TO_PREGRASP";
    case TaskState::ALIGN_BASE:
      return "ALIGN_BASE";
    case TaskState::STOP_BASE:
      return "STOP_BASE";
    case TaskState::MOVE_ARM_TO_PREGRASP:
      return "MOVE_ARM_TO_PREGRASP";
    case TaskState::MOVE_ARM_TO_GRASP:
      return "MOVE_ARM_TO_GRASP";
    case TaskState::CLOSE_GRIPPER:
      return "CLOSE_GRIPPER";
    case TaskState::LIFT_OBJECT:
      return "LIFT_OBJECT";
    case TaskState::STOW_OBJECT:
      return "STOW_OBJECT";
    case TaskState::DRIVE_TO_DELIVERY:
      return "DRIVE_TO_DELIVERY";
    case TaskState::ALIGN_AT_DELIVERY:
      return "ALIGN_AT_DELIVERY";
    case TaskState::STOP_BASE_AT_DELIVERY:
      return "STOP_BASE_AT_DELIVERY";
    case TaskState::PLACE_ARM:
      return "PLACE_ARM";
    case TaskState::OPEN_GRIPPER:
      return "OPEN_GRIPPER";
    case TaskState::RETREAT_ARM:
      return "RETREAT_ARM";
    case TaskState::ABORTING:
      return "ABORTING";
    case TaskState::DONE:
      return "DONE";
    case TaskState::FAILED:
      return "FAILED";
  }
  return "UNKNOWN";
}

/// Maps a piper_manipulator GraspObject feedback stage string onto the
/// matching FSM state. Unrecognized/empty strings return `fallback`.
inline TaskState armStageToState(const std::string & stage, TaskState fallback)
{
  if (stage == "MOVE_ARM_TO_PREGRASP") {return TaskState::MOVE_ARM_TO_PREGRASP;}
  if (stage == "MOVE_ARM_TO_GRASP") {return TaskState::MOVE_ARM_TO_GRASP;}
  if (stage == "CLOSE_GRIPPER") {return TaskState::CLOSE_GRIPPER;}
  if (stage == "LIFT_OBJECT") {return TaskState::LIFT_OBJECT;}
  if (stage == "STOW_OBJECT") {return TaskState::STOW_OBJECT;}
  return fallback;
}

/// Maps a piper_manipulator PlaceObject feedback stage string onto the
/// matching FSM state. Unrecognized/empty strings return `fallback`.
/// PlaceObject reports 4 sub-stages (MOVE_ARM_TO_PREPLACE/MOVE_ARM_TO_PLACE/
/// OPEN_GRIPPER/RETREAT_ARM, mirroring GraspObject's own 4-stage
/// granularity for symmetry -- see PlaceObject.action) but the FSM tracks
/// only 3 states for the place leg (PLACE_ARM/OPEN_GRIPPER/RETREAT_ARM):
/// both of PlaceObject's "moving toward the place pose" sub-stages fold
/// into the single PLACE_ARM state.
inline TaskState placeStageToState(const std::string & stage, TaskState fallback)
{
  if (stage == "MOVE_ARM_TO_PREPLACE") {return TaskState::PLACE_ARM;}
  if (stage == "MOVE_ARM_TO_PLACE") {return TaskState::PLACE_ARM;}
  if (stage == "OPEN_GRIPPER") {return TaskState::OPEN_GRIPPER;}
  if (stage == "RETREAT_ARM") {return TaskState::RETREAT_ARM;}
  return fallback;
}

/// Only STOP_BASE may lead into the pickup arm states -- this is the hard
/// interlock referenced in plan section 10.
inline bool isArmState(TaskState state)
{
  return state == TaskState::MOVE_ARM_TO_PREGRASP || state == TaskState::MOVE_ARM_TO_GRASP ||
         state == TaskState::CLOSE_GRIPPER || state == TaskState::LIFT_OBJECT ||
         state == TaskState::STOW_OBJECT;
}

/// Only STOP_BASE_AT_DELIVERY may lead into the place arm states -- the
/// same interlock as isArmState()/STOP_BASE, mirrored for the delivery leg.
inline bool isPlaceArmState(TaskState state)
{
  return state == TaskState::PLACE_ARM || state == TaskState::OPEN_GRIPPER ||
         state == TaskState::RETREAT_ARM;
}

inline bool isTerminal(TaskState state)
{
  return state == TaskState::DONE || state == TaskState::FAILED;
}

}  // namespace kotek_task_coordinator

#endif  // KOTEK_TASK_COORDINATOR__TASK_STATE_HPP_
