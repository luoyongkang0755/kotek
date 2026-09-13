#ifndef KOTEK_TASK_COORDINATOR__TASK_FSM_HPP_
#define KOTEK_TASK_COORDINATOR__TASK_FSM_HPP_

#include <string>

#include "kotek_task_coordinator/task_state.hpp"

namespace kotek_task_coordinator
{

/// Pure transition logic for the flow in plan section 10, decoupled from
/// ROS I/O the same way kotek_base_control::SimpleBaseController decouples
/// the control law from the action server -- so the state machine itself
/// (including the STOP_BASE interlock) is unit-testable without a ROS
/// graph. task_coordinator_node owns one of these, feeds it Inputs
/// gathered each tick from TF/subscribers/action clients, and performs the
/// side effects (sending goals, publishing /task/state) associated with
/// whatever transition happened.
class TaskFsm
{
public:
  struct Inputs
  {
    bool start_requested = false;
    bool abort_requested = false;

    bool have_sensor_pose = false;
    bool sensor_wait_timed_out = false;

    bool pregrasp_computed_ok = false;
    bool pregrasp_reachable = false;

    bool nav_goal_done = false;
    bool nav_goal_success = false;

    bool aligned = false;
    bool realign_available = false;

    bool base_settled = false;

    bool grasp_goal_done = false;
    bool grasp_goal_success = false;
    // Set by the node layer only once (grasp_goal_done && !grasp_goal_success)
    // -- true if a retry hasn't been used yet for this task attempt.
    // Mirrors realign_available's single-retry pattern (plan section 10):
    // a real miss (verified by piper_manipulator reading back the gripper's
    // actual closed width, not just "the motion completed") gets ONE
    // re-attempt before aborting.
    bool grasp_retry_available = false;
    std::string arm_feedback_stage;

    // Delivery leg. nav_goal_done/nav_goal_success, aligned/
    // realign_available, and base_settled above are reused as-is for the
    // delivery drive/align/stop phases (DRIVE_TO_PREGRASP and
    // DRIVE_TO_DELIVERY are never active at the same time, so one set of
    // "is the currently-active nav/align/settle check done" fields is
    // enough for both legs -- the node layer just feeds whichever goal is
    // actually in flight). place_goal_done/place_goal_success are separate
    // from grasp_goal_* because PLACE_ARM/OPEN_GRIPPER/RETREAT_ARM run
    // concurrently-in-spirit with nothing else, but keeping them distinct
    // avoids any ambiguity about which action's result they refer to.
    bool place_goal_done = false;
    bool place_goal_success = false;

    bool aborting_settled = false;
  };

  explicit TaskFsm(TaskState initial = TaskState::IDLE)
  : state_(initial)
  {
  }

  TaskState state() const {return state_;}

  /// Advances the machine one tick given `in`. Returns true if the state
  /// changed (the caller uses this to know when to perform the side
  /// effects -- e.g. sending an action goal -- associated with entering
  /// the new state, and to publish/log the transition).
  bool step(const Inputs & in)
  {
    const TaskState prev = state_;

    // Global interlock: an abort request drops any non-terminal,
    // non-ABORTING state straight into ABORTING.
    if (in.abort_requested && !isTerminal(state_) && state_ != TaskState::ABORTING) {
      state_ = TaskState::ABORTING;
      return state_ != prev;
    }

    switch (state_) {
      case TaskState::IDLE:
        if (in.start_requested) {
          state_ = TaskState::FIND_SENSOR;
        }
        break;

      case TaskState::FIND_SENSOR:
        if (in.have_sensor_pose) {
          state_ = TaskState::COMPUTE_BASE_PREGRASP_POSE;
        } else if (in.sensor_wait_timed_out) {
          state_ = TaskState::FAILED;
        }
        break;

      case TaskState::COMPUTE_BASE_PREGRASP_POSE:
        if (!in.pregrasp_computed_ok || !in.pregrasp_reachable) {
          state_ = TaskState::FAILED;
        } else {
          state_ = TaskState::DRIVE_TO_PREGRASP;
        }
        break;

      case TaskState::DRIVE_TO_PREGRASP:
        if (in.nav_goal_done) {
          state_ = in.nav_goal_success ? TaskState::ALIGN_BASE : TaskState::FAILED;
        }
        break;

      case TaskState::ALIGN_BASE:
        if (in.aligned) {
          state_ = TaskState::STOP_BASE;
        } else if (in.realign_available) {
          // Re-issue the drive goal once (plan section 10); the base
          // controller re-runs against the same target.
          state_ = TaskState::DRIVE_TO_PREGRASP;
        } else {
          state_ = TaskState::FAILED;
        }
        break;

      case TaskState::STOP_BASE:
        // The ONLY path into the arm states -- see isArmState /
        // "STOP_BASE is the only path into the arm states" (plan section
        // 10). No other case in this switch sets MOVE_ARM_TO_PREGRASP.
        if (in.base_settled) {
          state_ = TaskState::MOVE_ARM_TO_PREGRASP;
        }
        break;

      case TaskState::MOVE_ARM_TO_PREGRASP:
      case TaskState::MOVE_ARM_TO_GRASP:
      case TaskState::CLOSE_GRIPPER:
      case TaskState::LIFT_OBJECT:
      case TaskState::STOW_OBJECT:
        if (in.grasp_goal_done) {
          // A successful lift now feeds into the delivery leg instead of
          // going straight to DONE -- see the enum's doc comment.
          if (in.grasp_goal_success) {
            state_ = TaskState::DRIVE_TO_DELIVERY;
          } else if (in.grasp_retry_available) {
            // Re-issue the grasp goal once (mirrors ALIGN_BASE's
            // realign_available pattern above): a confirmed miss (fingers
            // closed without contacting the object, per
            // piper_manipulator's own readback) gets one fresh attempt
            // via MOVE_ARM_TO_PREGRASP before giving up.
            state_ = TaskState::MOVE_ARM_TO_PREGRASP;
          } else {
            state_ = TaskState::ABORTING;
          }
        } else {
          // Mirror piper_manipulator's own feedback rather than
          // duplicating its internal sequencing logic.
          state_ = armStageToState(in.arm_feedback_stage, state_);
        }
        break;

      case TaskState::DRIVE_TO_DELIVERY:
        if (in.nav_goal_done) {
          state_ = in.nav_goal_success ? TaskState::ALIGN_AT_DELIVERY : TaskState::ABORTING;
        }
        break;

      case TaskState::ALIGN_AT_DELIVERY:
        if (in.aligned) {
          state_ = TaskState::STOP_BASE_AT_DELIVERY;
        } else if (in.realign_available) {
          state_ = TaskState::DRIVE_TO_DELIVERY;
        } else {
          state_ = TaskState::ABORTING;
        }
        break;

      case TaskState::STOP_BASE_AT_DELIVERY:
        // The ONLY path into the place arm states -- see isPlaceArmState(),
        // mirroring STOP_BASE's own interlock for the pickup leg.
        if (in.base_settled) {
          state_ = TaskState::PLACE_ARM;
        }
        break;

      case TaskState::PLACE_ARM:
      case TaskState::OPEN_GRIPPER:
      case TaskState::RETREAT_ARM:
        if (in.place_goal_done) {
          state_ = in.place_goal_success ? TaskState::DONE : TaskState::ABORTING;
        } else {
          state_ = placeStageToState(in.arm_feedback_stage, state_);
        }
        break;

      case TaskState::ABORTING:
        if (in.aborting_settled) {
          state_ = TaskState::FAILED;
        }
        break;

      case TaskState::DONE:
      case TaskState::FAILED:
        if (in.start_requested) {
          state_ = TaskState::IDLE;
        }
        break;
    }

    return state_ != prev;
  }

private:
  TaskState state_;
};

}  // namespace kotek_task_coordinator

#endif  // KOTEK_TASK_COORDINATOR__TASK_FSM_HPP_
