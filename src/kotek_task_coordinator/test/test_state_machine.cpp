#include <vector>

#include "gtest/gtest.h"
#include "kotek_task_coordinator/task_fsm.hpp"

using kotek_task_coordinator::TaskFsm;
using kotek_task_coordinator::TaskState;
using Inputs = TaskFsm::Inputs;

TEST(TaskFsm, IdleWaitsForStart)
{
  TaskFsm fsm;
  EXPECT_EQ(fsm.state(), TaskState::IDLE);
  EXPECT_FALSE(fsm.step(Inputs{}));
  EXPECT_EQ(fsm.state(), TaskState::IDLE);

  Inputs in;
  in.start_requested = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::FIND_SENSOR);
}

TEST(TaskFsm, FindSensorTimesOutToFailed)
{
  TaskFsm fsm(TaskState::FIND_SENSOR);
  Inputs in;
  in.sensor_wait_timed_out = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::FAILED);
}

TEST(TaskFsm, FindSensorSucceedsToComputePregrasp)
{
  TaskFsm fsm(TaskState::FIND_SENSOR);
  Inputs in;
  in.have_sensor_pose = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::COMPUTE_BASE_PREGRASP_POSE);
}

TEST(TaskFsm, UnreachablePregraspFails)
{
  TaskFsm fsm(TaskState::COMPUTE_BASE_PREGRASP_POSE);
  Inputs in;
  in.pregrasp_computed_ok = true;
  in.pregrasp_reachable = false;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::FAILED);
}

TEST(TaskFsm, ReachablePregraspDrivesToTarget)
{
  TaskFsm fsm(TaskState::COMPUTE_BASE_PREGRASP_POSE);
  Inputs in;
  in.pregrasp_computed_ok = true;
  in.pregrasp_reachable = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_PREGRASP);
}

TEST(TaskFsm, DriveStaysUntilNavGoalDone)
{
  TaskFsm fsm(TaskState::DRIVE_TO_PREGRASP);
  EXPECT_FALSE(fsm.step(Inputs{}));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_PREGRASP);
}

TEST(TaskFsm, DriveFailureGoesToFailed)
{
  TaskFsm fsm(TaskState::DRIVE_TO_PREGRASP);
  Inputs in;
  in.nav_goal_done = true;
  in.nav_goal_success = false;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::FAILED);
}

TEST(TaskFsm, DriveSuccessGoesToAlign)
{
  TaskFsm fsm(TaskState::DRIVE_TO_PREGRASP);
  Inputs in;
  in.nav_goal_done = true;
  in.nav_goal_success = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::ALIGN_BASE);
}

TEST(TaskFsm, AlignSuccessGoesToStopBase)
{
  TaskFsm fsm(TaskState::ALIGN_BASE);
  Inputs in;
  in.aligned = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::STOP_BASE);
}

TEST(TaskFsm, AlignFailureReissuesDriveOnceThenFails)
{
  TaskFsm fsm(TaskState::ALIGN_BASE);
  Inputs in;
  in.aligned = false;
  in.realign_available = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_PREGRASP);

  // Simulate the re-driven goal completing and landing back in ALIGN_BASE,
  // still misaligned, with the one-shot re-issue now used up.
  Inputs drive_done;
  drive_done.nav_goal_done = true;
  drive_done.nav_goal_success = true;
  EXPECT_TRUE(fsm.step(drive_done));
  EXPECT_EQ(fsm.state(), TaskState::ALIGN_BASE);

  Inputs still_misaligned;
  still_misaligned.aligned = false;
  still_misaligned.realign_available = false;
  EXPECT_TRUE(fsm.step(still_misaligned));
  EXPECT_EQ(fsm.state(), TaskState::FAILED);
}

TEST(TaskFsm, StopBaseWaitsForSettleBeforeArmStates)
{
  TaskFsm fsm(TaskState::STOP_BASE);
  EXPECT_FALSE(fsm.step(Inputs{}));
  EXPECT_EQ(fsm.state(), TaskState::STOP_BASE);

  Inputs in;
  in.base_settled = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_PREGRASP);
}

TEST(TaskFsm, ArmStatesOnlyReachableThroughStopBase)
{
  // Every state that is NOT STOP_BASE (and is not already one of the arm
  // states, where "staying put" is expected, not a fresh entry) must never
  // transition into MOVE_ARM_TO_PREGRASP, regardless of how favorable the
  // inputs look.
  const std::vector<TaskState> non_stop_states = {
    TaskState::IDLE, TaskState::FIND_SENSOR, TaskState::COMPUTE_BASE_PREGRASP_POSE,
    TaskState::DRIVE_TO_PREGRASP, TaskState::ALIGN_BASE,
    TaskState::ABORTING, TaskState::DONE, TaskState::FAILED,
  };

  for (const auto & s : non_stop_states) {
    TaskFsm fsm(s);
    Inputs in;
    in.start_requested = true;
    in.have_sensor_pose = true;
    in.pregrasp_computed_ok = true;
    in.pregrasp_reachable = true;
    in.nav_goal_done = true;
    in.nav_goal_success = true;
    in.aligned = true;
    in.base_settled = true;  // only STOP_BASE should act on this
    fsm.step(in);
    EXPECT_NE(fsm.state(), TaskState::MOVE_ARM_TO_PREGRASP)
      << "state " << kotek_task_coordinator::toString(s)
      << " must not transition directly into MOVE_ARM_TO_PREGRASP";
  }
}

TEST(TaskFsm, ArmFeedbackStageMirrorsManipulator)
{
  TaskFsm fsm(TaskState::MOVE_ARM_TO_PREGRASP);

  Inputs to_grasp;
  to_grasp.arm_feedback_stage = "MOVE_ARM_TO_GRASP";
  EXPECT_TRUE(fsm.step(to_grasp));
  EXPECT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_GRASP);

  Inputs to_close;
  to_close.arm_feedback_stage = "CLOSE_GRIPPER";
  EXPECT_TRUE(fsm.step(to_close));
  EXPECT_EQ(fsm.state(), TaskState::CLOSE_GRIPPER);

  Inputs to_lift;
  to_lift.arm_feedback_stage = "LIFT_OBJECT";
  EXPECT_TRUE(fsm.step(to_lift));
  EXPECT_EQ(fsm.state(), TaskState::LIFT_OBJECT);

  // STOW_OBJECT (section 4.16): retracts the arm/object clear of the
  // lidar before DRIVE_TO_DELIVERY -- see task_state.hpp's enum comment.
  Inputs to_stow;
  to_stow.arm_feedback_stage = "STOW_OBJECT";
  EXPECT_TRUE(fsm.step(to_stow));
  EXPECT_EQ(fsm.state(), TaskState::STOW_OBJECT);

  Inputs lift_done;
  lift_done.grasp_goal_done = true;
  lift_done.grasp_goal_success = true;
  EXPECT_TRUE(fsm.step(lift_done));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_DELIVERY);
}

TEST(TaskFsm, ArmFailureGoesToAbortingNotDirectlyFailed)
{
  TaskFsm fsm(TaskState::MOVE_ARM_TO_GRASP);
  Inputs in;
  in.grasp_goal_done = true;
  in.grasp_goal_success = false;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::ABORTING);
}

TEST(TaskFsm, GraspRetryReissuesPregraspOnceThenSucceeds)
{
  // Mirrors AlignFailureReissuesDriveOnceThenFails's structure for the
  // grasp_retry_available path added alongside moveGripperToWidth()'s
  // contact-detection readback (docs/pick_and_delivery_report.md section
  // 4.16): a confirmed miss with a retry still available goes back to
  // MOVE_ARM_TO_PREGRASP (which re-sends a fresh grasp_object goal)
  // instead of straight to ABORTING, and a subsequent success completes
  // normally.
  TaskFsm fsm(TaskState::CLOSE_GRIPPER);
  Inputs miss;
  miss.grasp_goal_done = true;
  miss.grasp_goal_success = false;
  miss.grasp_retry_available = true;
  EXPECT_TRUE(fsm.step(miss));
  EXPECT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_PREGRASP);

  // Re-attempt runs through the arm feedback stages again...
  Inputs to_grasp;
  to_grasp.arm_feedback_stage = "MOVE_ARM_TO_GRASP";
  EXPECT_TRUE(fsm.step(to_grasp));
  EXPECT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_GRASP);

  // ...and this time succeeds.
  Inputs success;
  success.grasp_goal_done = true;
  success.grasp_goal_success = true;
  EXPECT_TRUE(fsm.step(success));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_DELIVERY);
}

TEST(TaskFsm, GraspRetryExhaustedGoesToAborting)
{
  // Same setup as above, but the retry itself also misses -- the node
  // layer would no longer offer grasp_retry_available (it's a one-shot,
  // same as realign_available), and TaskFsm must fall through to
  // ABORTING rather than looping forever.
  TaskFsm fsm(TaskState::CLOSE_GRIPPER);
  Inputs first_miss;
  first_miss.grasp_goal_done = true;
  first_miss.grasp_goal_success = false;
  first_miss.grasp_retry_available = true;
  EXPECT_TRUE(fsm.step(first_miss));
  EXPECT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_PREGRASP);

  Inputs second_miss;
  second_miss.grasp_goal_done = true;
  second_miss.grasp_goal_success = false;
  second_miss.grasp_retry_available = false;
  EXPECT_TRUE(fsm.step(second_miss));
  EXPECT_EQ(fsm.state(), TaskState::ABORTING);
}

TEST(TaskFsm, AbortingSettlesToFailed)
{
  TaskFsm fsm(TaskState::ABORTING);
  EXPECT_FALSE(fsm.step(Inputs{}));
  Inputs in;
  in.aborting_settled = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::FAILED);
}

TEST(TaskFsm, AbortRequestInterruptsAnyActiveState)
{
  const std::vector<TaskState> active_states = {
    TaskState::FIND_SENSOR, TaskState::COMPUTE_BASE_PREGRASP_POSE, TaskState::DRIVE_TO_PREGRASP,
    TaskState::ALIGN_BASE, TaskState::STOP_BASE, TaskState::MOVE_ARM_TO_PREGRASP,
    TaskState::MOVE_ARM_TO_GRASP, TaskState::CLOSE_GRIPPER, TaskState::LIFT_OBJECT,
    TaskState::STOW_OBJECT, TaskState::DRIVE_TO_DELIVERY, TaskState::ALIGN_AT_DELIVERY,
    TaskState::STOP_BASE_AT_DELIVERY, TaskState::PLACE_ARM, TaskState::OPEN_GRIPPER,
    TaskState::RETREAT_ARM,
  };
  for (const auto & s : active_states) {
    TaskFsm fsm(s);
    Inputs in;
    in.abort_requested = true;
    EXPECT_TRUE(fsm.step(in));
    EXPECT_EQ(fsm.state(), TaskState::ABORTING)
      << "abort from " << kotek_task_coordinator::toString(s);
  }
}

TEST(TaskFsm, AbortRequestDoesNotReenterAbortingOrTerminalStates)
{
  TaskFsm aborting(TaskState::ABORTING);
  Inputs in;
  in.abort_requested = true;
  EXPECT_FALSE(aborting.step(in));
  EXPECT_EQ(aborting.state(), TaskState::ABORTING);

  TaskFsm done(TaskState::DONE);
  EXPECT_FALSE(done.step(in));
  EXPECT_EQ(done.state(), TaskState::DONE);

  TaskFsm failed(TaskState::FAILED);
  EXPECT_FALSE(failed.step(in));
  EXPECT_EQ(failed.state(), TaskState::FAILED);
}

TEST(TaskFsm, TerminalStatesReArmToIdleOnStart)
{
  TaskFsm done(TaskState::DONE);
  Inputs in;
  in.start_requested = true;
  EXPECT_TRUE(done.step(in));
  EXPECT_EQ(done.state(), TaskState::IDLE);

  TaskFsm failed(TaskState::FAILED);
  EXPECT_TRUE(failed.step(in));
  EXPECT_EQ(failed.state(), TaskState::IDLE);
}

TEST(TaskFsm, FullHappyPathReachesDone)
{
  TaskFsm fsm;

  Inputs start;
  start.start_requested = true;
  ASSERT_TRUE(fsm.step(start));
  ASSERT_EQ(fsm.state(), TaskState::FIND_SENSOR);

  Inputs sensor;
  sensor.have_sensor_pose = true;
  ASSERT_TRUE(fsm.step(sensor));
  ASSERT_EQ(fsm.state(), TaskState::COMPUTE_BASE_PREGRASP_POSE);

  Inputs pregrasp;
  pregrasp.pregrasp_computed_ok = true;
  pregrasp.pregrasp_reachable = true;
  ASSERT_TRUE(fsm.step(pregrasp));
  ASSERT_EQ(fsm.state(), TaskState::DRIVE_TO_PREGRASP);

  Inputs drive;
  drive.nav_goal_done = true;
  drive.nav_goal_success = true;
  ASSERT_TRUE(fsm.step(drive));
  ASSERT_EQ(fsm.state(), TaskState::ALIGN_BASE);

  Inputs align;
  align.aligned = true;
  ASSERT_TRUE(fsm.step(align));
  ASSERT_EQ(fsm.state(), TaskState::STOP_BASE);

  Inputs stop;
  stop.base_settled = true;
  ASSERT_TRUE(fsm.step(stop));
  ASSERT_EQ(fsm.state(), TaskState::MOVE_ARM_TO_PREGRASP);

  Inputs grasp_done;
  grasp_done.grasp_goal_done = true;
  grasp_done.grasp_goal_success = true;
  ASSERT_TRUE(fsm.step(grasp_done));
  ASSERT_EQ(fsm.state(), TaskState::DRIVE_TO_DELIVERY);

  Inputs deliver_drive;
  deliver_drive.nav_goal_done = true;
  deliver_drive.nav_goal_success = true;
  ASSERT_TRUE(fsm.step(deliver_drive));
  ASSERT_EQ(fsm.state(), TaskState::ALIGN_AT_DELIVERY);

  Inputs deliver_align;
  deliver_align.aligned = true;
  ASSERT_TRUE(fsm.step(deliver_align));
  ASSERT_EQ(fsm.state(), TaskState::STOP_BASE_AT_DELIVERY);

  Inputs deliver_stop;
  deliver_stop.base_settled = true;
  ASSERT_TRUE(fsm.step(deliver_stop));
  ASSERT_EQ(fsm.state(), TaskState::PLACE_ARM);

  Inputs place_done;
  place_done.place_goal_done = true;
  place_done.place_goal_success = true;
  ASSERT_TRUE(fsm.step(place_done));
  ASSERT_EQ(fsm.state(), TaskState::DONE);
}

TEST(TaskFsm, DriveToDeliveryFailureGoesToAborting)
{
  TaskFsm fsm(TaskState::DRIVE_TO_DELIVERY);
  Inputs in;
  in.nav_goal_done = true;
  in.nav_goal_success = false;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::ABORTING);
}

TEST(TaskFsm, DriveToDeliverySuccessGoesToAlignAtDelivery)
{
  TaskFsm fsm(TaskState::DRIVE_TO_DELIVERY);
  Inputs in;
  in.nav_goal_done = true;
  in.nav_goal_success = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::ALIGN_AT_DELIVERY);
}

TEST(TaskFsm, AlignAtDeliveryFailureReissuesDriveOnceThenAborts)
{
  TaskFsm fsm(TaskState::ALIGN_AT_DELIVERY);
  Inputs in;
  in.aligned = false;
  in.realign_available = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::DRIVE_TO_DELIVERY);

  Inputs drive_done;
  drive_done.nav_goal_done = true;
  drive_done.nav_goal_success = true;
  EXPECT_TRUE(fsm.step(drive_done));
  EXPECT_EQ(fsm.state(), TaskState::ALIGN_AT_DELIVERY);

  Inputs still_misaligned;
  still_misaligned.aligned = false;
  still_misaligned.realign_available = false;
  EXPECT_TRUE(fsm.step(still_misaligned));
  EXPECT_EQ(fsm.state(), TaskState::ABORTING);
}

TEST(TaskFsm, StopBaseAtDeliveryWaitsForSettleBeforePlaceArm)
{
  TaskFsm fsm(TaskState::STOP_BASE_AT_DELIVERY);
  EXPECT_FALSE(fsm.step(Inputs{}));
  EXPECT_EQ(fsm.state(), TaskState::STOP_BASE_AT_DELIVERY);

  Inputs in;
  in.base_settled = true;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::PLACE_ARM);
}

TEST(TaskFsm, PlaceArmStatesOnlyReachableThroughStopBaseAtDelivery)
{
  const std::vector<TaskState> non_stop_states = {
    TaskState::IDLE, TaskState::FIND_SENSOR, TaskState::COMPUTE_BASE_PREGRASP_POSE,
    TaskState::DRIVE_TO_PREGRASP, TaskState::ALIGN_BASE, TaskState::STOP_BASE,
    TaskState::MOVE_ARM_TO_PREGRASP, TaskState::MOVE_ARM_TO_GRASP, TaskState::CLOSE_GRIPPER,
    TaskState::LIFT_OBJECT, TaskState::DRIVE_TO_DELIVERY, TaskState::ALIGN_AT_DELIVERY,
    TaskState::ABORTING, TaskState::DONE, TaskState::FAILED,
  };

  for (const auto & s : non_stop_states) {
    TaskFsm fsm(s);
    Inputs in;
    in.start_requested = true;
    in.have_sensor_pose = true;
    in.pregrasp_computed_ok = true;
    in.pregrasp_reachable = true;
    in.nav_goal_done = true;
    in.nav_goal_success = true;
    in.aligned = true;
    in.grasp_goal_done = true;
    in.grasp_goal_success = true;
    in.base_settled = true;  // only STOP_BASE_AT_DELIVERY should act on this
    fsm.step(in);
    EXPECT_NE(fsm.state(), TaskState::PLACE_ARM)
      << "state " << kotek_task_coordinator::toString(s)
      << " must not transition directly into PLACE_ARM";
  }
}

TEST(TaskFsm, PlaceFeedbackStageMirrorsManipulator)
{
  TaskFsm fsm(TaskState::PLACE_ARM);

  // Both of PlaceObject's "moving toward place" sub-stages fold into the
  // single PLACE_ARM state (see placeStageToState's doc comment) -- this
  // step does not cross an FSM state boundary, so it correctly reports no
  // change, same as DriveStaysUntilNavGoalDone staying put.
  Inputs to_place;
  to_place.arm_feedback_stage = "MOVE_ARM_TO_PLACE";
  EXPECT_FALSE(fsm.step(to_place));
  EXPECT_EQ(fsm.state(), TaskState::PLACE_ARM);

  Inputs to_open;
  to_open.arm_feedback_stage = "OPEN_GRIPPER";
  EXPECT_TRUE(fsm.step(to_open));
  EXPECT_EQ(fsm.state(), TaskState::OPEN_GRIPPER);

  Inputs to_retreat;
  to_retreat.arm_feedback_stage = "RETREAT_ARM";
  EXPECT_TRUE(fsm.step(to_retreat));
  EXPECT_EQ(fsm.state(), TaskState::RETREAT_ARM);

  Inputs done;
  done.place_goal_done = true;
  done.place_goal_success = true;
  EXPECT_TRUE(fsm.step(done));
  EXPECT_EQ(fsm.state(), TaskState::DONE);
}

TEST(TaskFsm, PlaceFailureGoesToAbortingNotDirectlyFailed)
{
  TaskFsm fsm(TaskState::OPEN_GRIPPER);
  Inputs in;
  in.place_goal_done = true;
  in.place_goal_success = false;
  EXPECT_TRUE(fsm.step(in));
  EXPECT_EQ(fsm.state(), TaskState::ABORTING);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
