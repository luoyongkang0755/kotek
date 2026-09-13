#include "kotek_manipulation/piper_manipulator.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "geometry_msgs/msg/pose.hpp"

namespace kotek_manipulation
{

namespace
{
geometry_msgs::msg::Pose toMsg(const Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position.x;
  msg.position.y = pose.position.y;
  msg.position.z = pose.position.z;
  msg.orientation.x = pose.orientation.x;
  msg.orientation.y = pose.orientation.y;
  msg.orientation.z = pose.orientation.z;
  msg.orientation.w = pose.orientation.w;
  return msg;
}

// Shared between moveGripperToWidth()'s post-close check and
// stillHoldingObject()'s post-lift recheck -- see moveGripperToWidth()'s
// comment for how this was calibrated against real live-run numbers.
constexpr double kContactTolerance = 0.0015;  // meters, joint7 units (half-width)
}  // namespace

PiperManipulator::PiperManipulator()
: rclcpp::Node("piper_manipulator"), params_(loadParams())
{
  action_server_ = rclcpp_action::create_server<GraspObject>(
    this, "grasp_object",
    std::bind(
      &PiperManipulator::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PiperManipulator::handleCancel, this, std::placeholders::_1),
    std::bind(&PiperManipulator::handleAccepted, this, std::placeholders::_1));

  place_action_server_ = rclcpp_action::create_server<PlaceObject>(
    this, "place_object",
    std::bind(
      &PiperManipulator::handlePlaceGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PiperManipulator::handlePlaceCancel, this, std::placeholders::_1),
    std::bind(&PiperManipulator::handlePlaceAccepted, this, std::placeholders::_1));
}

PiperManipulator::Params PiperManipulator::loadParams()
{
  Params p;
  p.arm_group = declare_parameter<std::string>("arm_group", p.arm_group);
  p.gripper_group = declare_parameter<std::string>("gripper_group", p.gripper_group);
  p.approach_distance = declare_parameter<double>("approach_distance", p.approach_distance);
  p.approach_height = declare_parameter<double>("approach_height", p.approach_height);
  p.place_approach_distance =
    declare_parameter<double>("place_approach_distance", p.place_approach_distance);
  p.default_lift_height = declare_parameter<double>("lift_height", p.default_lift_height);
  p.default_retreat_height =
    declare_parameter<double>("retreat_height", p.default_retreat_height);
  p.grasp_width = declare_parameter<double>("grasp_width", p.grasp_width);
  p.gripper_open = declare_parameter<double>("gripper_open", p.gripper_open);
  p.grasp_pitch = declare_parameter<double>("grasp_pitch", p.grasp_pitch);
  p.place_pitch = declare_parameter<double>("place_pitch", p.place_pitch);
  p.min_cartesian_fraction =
    declare_parameter<double>("min_cartesian_fraction", p.min_cartesian_fraction);
  p.velocity_scaling = declare_parameter<double>("velocity_scaling", p.velocity_scaling);
  p.acceleration_scaling =
    declare_parameter<double>("acceleration_scaling", p.acceleration_scaling);
  p.lift_velocity_scaling =
    declare_parameter<double>("lift_velocity_scaling", p.lift_velocity_scaling);
  p.lift_acceleration_scaling =
    declare_parameter<double>("lift_acceleration_scaling", p.lift_acceleration_scaling);
  p.post_grasp_settle_time =
    declare_parameter<double>("post_grasp_settle_time", p.post_grasp_settle_time);
  p.release_settle_time =
    declare_parameter<double>("release_settle_time", p.release_settle_time);
  p.stow_after_lift = declare_parameter<bool>("stow_after_lift", p.stow_after_lift);
  p.min_lift_fraction = declare_parameter<double>("min_lift_fraction", p.min_lift_fraction);
  p.carry_velocity_scaling =
    declare_parameter<double>("carry_velocity_scaling", p.carry_velocity_scaling);
  p.carry_acceleration_scaling =
    declare_parameter<double>("carry_acceleration_scaling", p.carry_acceleration_scaling);
  p.planning_time = declare_parameter<double>("planning_time", p.planning_time);
  p.planning_attempts = declare_parameter<int>("planning_attempts", p.planning_attempts);
  return p;
}

void PiperManipulator::init(const std::shared_ptr<PiperManipulator> & self)
{
  // Separate internal nodes for each MoveGroupInterface -- see the member
  // declarations' comment in piper_manipulator.hpp for why sharing `self`
  // between two instances deadlocks. Carry over `self`'s own parameter
  // overrides (robot_description/_semantic/_kinematics/_planning, all
  // bundled in via moveit_config.to_dict() at launch) so each internal
  // node can build the same RobotModel; global arguments (namespace
  // remapping to /piper, use_sim_time, etc.) are inherited automatically
  // via the default NodeOptions.
  std::vector<rclcpp::Parameter> overrides;
  const auto param_overrides = self->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : param_overrides) {
    overrides.emplace_back(name, value);
  }
  // use_global_arguments(false): the process's own --ros-args carry a
  // GLOBAL `-r __node:=piper_manipulator` remap (from this node's own
  // launch entry), which by default (use_global_arguments defaults to
  // true) applies to EVERY rclcpp::Node constructed in the process --
  // silently renaming these "separate" internal nodes back to
  // "piper_manipulator" too, colliding with `self` and each other instead
  // of actually being distinct (confirmed directly: `ros2 node list`
  // showed FOUR nodes all named /piper/piper_manipulator, and
  // "piper_manipulator_arm_internal"/"_gripper_internal" never appeared at
  // all). Disabling global-argument inheritance and setting the namespace
  // explicitly (third constructor arg) keeps these nodes' own distinct
  // names while still landing under /piper, matching move_group's own
  // namespace so MoveGroupInterface resolves its action/service names
  // correctly.
  rclcpp::NodeOptions internal_opts;
  internal_opts.use_global_arguments(false);
  internal_opts.parameter_overrides(overrides);
  internal_opts.automatically_declare_parameters_from_overrides(true);
  move_group_node_arm_ =
    std::make_shared<rclcpp::Node>("piper_manipulator_arm_internal", "piper", internal_opts);
  move_group_node_gripper_ =
    std::make_shared<rclcpp::Node>("piper_manipulator_gripper_internal", "piper", internal_opts);

  // Spin both internal nodes ourselves too, as a robustness measure --
  // confirmed directly (a standalone rclcpp_action client against
  // move_group_node_arm_) that discovery of /piper/move_action works
  // correctly under our own spinning.
  internal_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  internal_executor_->add_node(move_group_node_arm_);
  internal_executor_->add_node(move_group_node_gripper_);
  std::thread([executor = internal_executor_]() {executor->spin();}).detach();

  // MoveGroupInterfaceImpl's constructor targets
  // `rclcpp::names::append(opt_.move_group_namespace, move_group::MOVE_ACTION)`
  // for its action clients. `move_group_node_arm_`/`move_group_node_gripper_`
  // are ALREADY constructed in the "piper" namespace (above), so the
  // relative name "move_action" (i.e. Options' own default, empty
  // move_group_namespace) already resolves correctly against the node's own
  // namespace to /piper/move_action -- explicitly passing "piper" here (an
  // earlier attempt) double-namespaces it to /piper/piper/move_action,
  // which has no server and hangs forever. Confirmed directly both ways:
  // `ros2 action info /piper/piper/move_action` showed exactly our node as
  // the sole client with zero servers (the bug).
  //
  // Even after fixing that, MoveGroupInterface's own internal wait_for_
  // action_server (using its own dedicated callback_executor_/
  // callback_thread_, see moveit_ros_planning_interface's move_group_
  // interface.cpp) never returns -- gdb-confirmed, repeatedly, across many
  // variants (shared vs. separate nodes, with vs. without our own spinning,
  // with a startup delay to rule out a discovery race). This is despite a
  // bare rclcpp_action client constructed against the exact same node
  // finding /piper/move_action correctly in ~2s every time. This looks like
  // a genuine limitation/bug in this specific moveit_ros_planning_interface
  // 2.12.4 build's internal wait mechanism in this environment, not
  // something resolvable from this call site. Worked around with a finite
  // wait_for_servers timeout instead of the default "wait forever": the
  // action clients are still constructed correctly regardless (confirmed
  // via the standalone test above), so a plan()/execute() call later still
  // works even if this constructor-time readiness check itself can't be
  // trusted to return in this environment.
  move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    move_group_node_arm_, params_.arm_group, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(10.0));
  move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    move_group_node_gripper_, params_.gripper_group, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(10.0));

  move_group_arm_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);
  move_group_arm_->setPlanningTime(params_.planning_time);
  move_group_arm_->setNumPlanningAttempts(params_.planning_attempts);

  // Built on `self` (the original, launch-provided node -- already
  // correctly named/namespaced, not one of the internal-node workarounds
  // above), with the default callback group -- see the member declarations'
  // comment for why this is required instead of using MoveGroupInterface's
  // own action clients. `self` isn't spun yet at this point in init() (see
  // main()), but that's fine: these clients are only ever *used* later,
  // from inside execute()/executePlace() (which only run once a
  // grasp_object/place_object goal arrives, i.e. after main()'s
  // executor.spin() -- which spins `self` -- is already running).
  raw_move_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
    self, "move_action");
  raw_execute_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
    self, "execute_trajectory");
  raw_cartesian_path_client_ =
    self->create_client<moveit_msgs::srv::GetCartesianPath>("compute_cartesian_path");

  RCLCPP_INFO(
    get_logger(), "piper_manipulator ready: arm_group=%s gripper_group=%s planning_frame=%s",
    params_.arm_group.c_str(), params_.gripper_group.c_str(),
    move_group_arm_->getPlanningFrame().c_str());
}

rclcpp_action::GoalResponse PiperManipulator::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const GraspObject::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PiperManipulator::handleCancel(
  const std::shared_ptr<GoalHandle>)
{
  // Motion already in flight through MoveGroupInterface::execute() cannot
  // be interrupted mid-call by this simple synchronous implementation; we
  // still accept the cancel request so the action reports it, and the
  // in-progress stage is allowed to finish before the next stage checks
  // for it (not currently checked -- documented limitation, see README).
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PiperManipulator::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PiperManipulator::execute, this, goal_handle)}.detach();
}

void PiperManipulator::publishStageFeedback(
  const std::shared_ptr<GoalHandle> & goal_handle, const std::string & stage, double progress)
{
  auto feedback = std::make_shared<GraspObject::Feedback>();
  feedback->stage = stage;
  feedback->progress = progress;
  goal_handle->publish_feedback(feedback);
}

bool PiperManipulator::sendMoveGroupRequest(
  const moveit_msgs::msg::MotionPlanRequest & request, std::string & error_out)
{
  // See the header comment: transient single-shot MoveIt flakes (instant
  // status=6 aborts, result timeouts that leave the executor briefly
  // wedged) are retried rather than failing the whole action. The pause
  // gives a still-draining previous trajectory time to finish so the retry
  // isn't rejected with "Cannot push a new trajectory while another is
  // being executed" -- a live corner-3 STOW failure showed that blocker
  // clearing only ~10s after the first attempt (the slow 0.08-scaled lift
  // trajectory's controller goal outliving its own action result), so the
  // retry window must comfortably exceed that: 4 attempts x 5s covers 15s.
  constexpr int kMoveGroupAttempts = 4;
  constexpr auto kRetryPause = std::chrono::seconds(5);
  for (int attempt = 1;; ++attempt) {
    if (sendMoveGroupRequestOnce(request, error_out)) {
      return true;
    }
    if (attempt >= kMoveGroupAttempts) {
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "move_action attempt %d/%d failed (%s); retrying",
      attempt, kMoveGroupAttempts, error_out.c_str());
    std::this_thread::sleep_for(kRetryPause);
  }
}

bool PiperManipulator::sendMoveGroupRequestOnce(
  const moveit_msgs::msg::MotionPlanRequest & request, std::string & error_out)
{
  // Advisory only -- see the member declaration's comment: this specific
  // readiness check is unreliable in this environment even when the
  // underlying goal send works, so a false result here is logged, not
  // treated as fatal.
  if (!raw_move_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(
      get_logger(),
      "move_action server not confirmed ready after 5s; sending goal anyway");
  }

  moveit_msgs::action::MoveGroup::Goal goal;
  goal.request = request;
  goal.planning_options.plan_only = false;

  using MoveGroupClient = rclcpp_action::Client<moveit_msgs::action::MoveGroup>;
  auto goal_handle_future =
    raw_move_client_->async_send_goal(goal, MoveGroupClient::SendGoalOptions());
  if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    error_out = "move_action goal send timed out";
    return false;
  }
  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_out = "move_action goal rejected";
    return false;
  }

  auto result_future = raw_move_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    error_out = "move_action result timed out";
    return false;
  }
  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_out = "move_action did not succeed (status=" +
      std::to_string(static_cast<int>(wrapped_result.code)) + ")";
    return false;
  }
  if (wrapped_result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    error_out =
      "move_action failed (code=" + std::to_string(wrapped_result.result->error_code.val) + ")";
    return false;
  }
  return true;
}

bool PiperManipulator::sendExecuteTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & error_out)
{
  // Small transient-failure retry (see sendMoveGroupRequest's header
  // comment). Kept to 2 attempts: a stale-start-state rejection can only
  // be fixed by REPLANNING, which cartesianMoveTo's own outer loop does --
  // deep retries of an unchanged trajectory just burn time (observed: the
  // same stale trajectory rejected 4x in a row).
  constexpr int kExecuteAttempts = 2;
  constexpr auto kRetryPause = std::chrono::seconds(5);
  for (int attempt = 1;; ++attempt) {
    if (sendExecuteTrajectoryOnce(trajectory, error_out)) {
      return true;
    }
    if (attempt >= kExecuteAttempts) {
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "execute_trajectory attempt %d/%d failed (%s); retrying",
      attempt, kExecuteAttempts, error_out.c_str());
    std::this_thread::sleep_for(kRetryPause);
  }
}

bool PiperManipulator::sendExecuteTrajectoryOnce(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & error_out)
{
  if (!raw_execute_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(
      get_logger(),
      "execute_trajectory server not confirmed ready after 5s; sending goal anyway");
  }

  moveit_msgs::action::ExecuteTrajectory::Goal goal;
  goal.trajectory = trajectory;

  auto goal_handle_future = raw_execute_client_->async_send_goal(
    goal, rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions());
  if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    error_out = "execute_trajectory goal send timed out";
    return false;
  }
  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_out = "execute_trajectory goal rejected";
    return false;
  }

  // A live run's own full grasp sequence (pregrasp move, gripper open,
  // cartesian-to-grasp, close, cartesian-lift) completed end to end in
  // ~9s total, each individual execute_trajectory leg taking ~1s -- 60s
  // is a generous multiple of that, not a tight bound.
  auto result_future = raw_execute_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
    error_out = "execute_trajectory result timed out";
    return false;
  }
  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_out = "execute_trajectory did not succeed (status=" +
      std::to_string(static_cast<int>(wrapped_result.code)) + ")";
    return false;
  }
  if (wrapped_result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    error_out = "execute_trajectory failed (code=" +
      std::to_string(wrapped_result.result->error_code.val) + ")";
    return false;
  }
  return true;
}

double PiperManipulator::computeCartesianPathRaw(
  const geometry_msgs::msg::Pose & pose, moveit_msgs::msg::RobotTrajectory & trajectory_out,
  std::string & error_out, double velocity_scaling, double acceleration_scaling)
{
  if (!raw_cartesian_path_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(
      get_logger(),
      "compute_cartesian_path service not confirmed ready after 5s; calling anyway");
  }

  auto request = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  request->header.frame_id = move_group_arm_->getPlanningFrame();
  request->header.stamp = now();
  request->group_name = params_.arm_group;
  request->waypoints = {pose};
  request->max_step = 0.01;
  request->avoid_collisions = true;
  request->max_velocity_scaling_factor = velocity_scaling;
  request->max_acceleration_scaling_factor = acceleration_scaling;

  auto future = raw_cartesian_path_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    error_out = "compute_cartesian_path service call timed out";
    return 0.0;
  }
  const auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    error_out =
      "compute_cartesian_path failed (code=" + std::to_string(response->error_code.val) + ")";
    return 0.0;
  }
  trajectory_out = response->solution;
  return response->fraction;
}

bool PiperManipulator::moveArmToPose(
  const geometry_msgs::msg::Pose & pose, std::string & error_out,
  double velocity_scaling, double acceleration_scaling)
{
  if (velocity_scaling <= 0.0) {
    velocity_scaling = params_.velocity_scaling;
  }
  if (acceleration_scaling <= 0.0) {
    acceleration_scaling = params_.acceleration_scaling;
  }
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = move_group_arm_->getPlanningFrame();
  target.header.stamp = now();
  target.pose = pose;
  move_group_arm_->setPoseTarget(target);

  // Same set-then-restore pattern as retreatToSafe(): the scalings live on
  // the shared MoveGroupInterface, so leave it at the normal speeds for
  // whoever plans next.
  move_group_arm_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(acceleration_scaling);
  moveit_msgs::msg::MotionPlanRequest request;
  move_group_arm_->constructMotionPlanRequest(request);
  move_group_arm_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);
  return sendMoveGroupRequest(request, error_out);
}

bool PiperManipulator::cartesianMoveTo(
  const geometry_msgs::msg::Pose & pose, std::string & error_out, double velocity_scaling,
  double acceleration_scaling, double min_fraction)
{
  if (velocity_scaling <= 0.0) {
    velocity_scaling = params_.velocity_scaling;
  }
  if (acceleration_scaling <= 0.0) {
    acceleration_scaling = params_.acceleration_scaling;
  }
  if (min_fraction <= 0.0 || min_fraction > 1.0) {
    min_fraction = params_.min_cartesian_fraction;
  }
  // Recompute-and-execute per attempt: the path is planned from the
  // CURRENT state, and at slow-settling poses (the twisted rear-corner
  // grasps) the arm can still be drifting when execution validates the
  // start point ("Invalid Trajectory: start point deviates from current
  // robot state more than 0.01 at joint 'joint1'", observed rejecting the
  // SAME stale trajectory 4x in a live run). Re-sending an unchanged
  // trajectory can never clear that -- only replanning from wherever the
  // arm actually is now can, and the pause lets the drift finish first.
  constexpr int kCartesianAttempts = 3;
  constexpr auto kSettlePause = std::chrono::seconds(2);
  for (int attempt = 1;; ++attempt) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = computeCartesianPathRaw(
      pose, trajectory, error_out, velocity_scaling, acceleration_scaling);
    if (fraction < min_fraction) {
      if (error_out.empty()) {
        error_out = "cartesian path only " + std::to_string(fraction) +
          " complete (need >= " + std::to_string(min_fraction) + ")";
      }
    } else {
      if (fraction < 1.0) {
        RCLCPP_WARN(
          get_logger(), "cartesian move accepted at partial fraction %.3f (floor %.3f)",
          fraction, min_fraction);
      }
      if (sendExecuteTrajectory(trajectory, error_out)) {
        return true;
      }
    }
    if (attempt >= kCartesianAttempts) {
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "cartesian move attempt %d/%d failed (%s); replanning from current state",
      attempt, kCartesianAttempts, error_out.c_str());
    std::this_thread::sleep_for(kSettlePause);
  }
}

bool PiperManipulator::moveGripperNamed(const std::string & name, std::string & error_out)
{
  move_group_gripper_->setNamedTarget(name);
  moveit_msgs::msg::MotionPlanRequest request;
  move_group_gripper_->constructMotionPlanRequest(request);
  return sendMoveGroupRequest(request, error_out);
}

bool PiperManipulator::moveGripperToWidth(
  double width, std::string & error_out, bool & contact_detected_out)
{
  const double target_joint7 = width / 2.0;
  move_group_gripper_->setJointValueTarget("joint7", target_joint7);
  move_group_gripper_->setJointValueTarget("joint8", -target_joint7);
  moveit_msgs::msg::MotionPlanRequest request;
  move_group_gripper_->constructMotionPlanRequest(request);
  // A non-SUCCESS result here (e.g. a genuine planning/execution fault) is
  // treated as a real failure at this level. Note this is NOT how a
  // contact stall is reported: isaac_joint_bridge's own stall-detection
  // already converts "stopped short of target due to contact" into a
  // SUCCESSFUL FollowJointTrajectory result (plan section 9) -- which is
  // exactly why motion success alone can't tell a real grasp from closing
  // on empty air, and why the readback below is needed.
  if (!sendMoveGroupRequest(request, error_out)) {
    return false;
  }

  // Read back the ACTUAL achieved joint7 position -- a plain
  // CurrentStateMonitor read via getCurrentState(), confirmed in section
  // 4.15 to be unaffected by the action/service dispatch bug the raw_*
  // clients exist to work around. If the fingers stopped meaningfully
  // short of the fully-closed commanded target, something was between
  // them (contact => grasped); if they reached (~)the full commanded
  // width, nothing was there (missed).
  double actual_joint7 = target_joint7;
  const auto current_state = move_group_gripper_->getCurrentState(1.0);
  if (current_state) {
    const double * pos = current_state->getJointPositions("joint7");
    if (pos != nullptr) {
      actual_joint7 = pos[0];
    } else {
      RCLCPP_WARN(get_logger(), "gripper contact check: joint7 not found in current state");
    }
  } else {
    RCLCPP_WARN(get_logger(), "gripper contact check: getCurrentState() timed out");
  }
  // joint7 moves from the open position down toward target_joint7 as the
  // gripper closes (params_.gripper_open > params_.grasp_width, both
  // expressed here as their /2 joint targets) -- an object between the
  // fingers stops it above target_joint7. kContactTolerance absorbs
  // normal trajectory-execution slop around the commanded target.
  //
  // Calibrated directly against real live-run numbers (docs/
  // pick_and_delivery_report.md section 4.16), after the orientation,
  // finger-offset, and odom->arm-frame Z fixes were all in place: a
  // confirmed total miss (fingers closing on empty air, object nowhere
  // near) left a residual of 0.0002-0.0006m -- pure trajectory-execution
  // noise. A real, physical (if light/grazing) contact against the
  // object left a residual of 0.0026m -- 4-13x the miss noise floor, but
  // BELOW the original 0.003 threshold, so that grasp was wrongly
  // classified as a miss. 0.0015m sits between the two, comfortably
  // above the noise floor and comfortably below the observed real-contact
  // signal. (kContactTolerance lives at file scope -- shared with
  // stillHoldingObject()'s post-lift recheck below.)
  contact_detected_out = actual_joint7 > (target_joint7 + kContactTolerance);
  RCLCPP_INFO(
    get_logger(), "gripper contact check: target_joint7=%.5f actual_joint7=%.5f contact=%s",
    target_joint7, actual_joint7, contact_detected_out ? "true" : "false");
  return true;
}

bool PiperManipulator::stillHoldingObject()
{
  // Re-reads joint7 with NO new commanded motion -- catches a grip that
  // registered real contact in moveGripperToWidth() (closing the fingers
  // stopped meaningfully short of grasp_width's target) but then slipped
  // free during the LIFT_OBJECT cartesian move, which a one-shot check
  // right after CLOSE_GRIPPER can't see. Observed directly in a real live
  // run (docs/pick_and_delivery_report.md section 4.16): CLOSE_GRIPPER
  // reported contact=true, LIFT_OBJECT's motion itself reported success,
  // but a screenshot afterward showed the object back on the ground next
  // to the pedestal -- it had been knocked loose, not securely held.
  const double target_joint7 = params_.grasp_width / 2.0;
  const auto current_state = move_group_gripper_->getCurrentState(1.0);
  if (!current_state) {
    RCLCPP_WARN(get_logger(), "post-lift grip check: getCurrentState() timed out");
    return false;
  }
  const double * pos = current_state->getJointPositions("joint7");
  if (pos == nullptr) {
    RCLCPP_WARN(get_logger(), "post-lift grip check: joint7 not found in current state");
    return false;
  }
  const double actual_joint7 = pos[0];
  const bool holding = actual_joint7 > (target_joint7 + kContactTolerance);
  RCLCPP_INFO(
    get_logger(), "post-lift grip check: target_joint7=%.5f actual_joint7=%.5f holding=%s",
    target_joint7, actual_joint7, holding ? "true" : "false");
  return holding;
}

bool PiperManipulator::retreatToSafe()
{
  // 'zero' is a named joint-space group_state defined for the 'arm' group
  // in the SRDF (plan section 1.5); the gripper group has no equivalent,
  // so only the arm is retreated.
  //
  // Uses the same slower lift_velocity_scaling/lift_acceleration_scaling as
  // LIFT_OBJECT's cartesian move, not the normal (faster)
  // velocity_scaling/acceleration_scaling -- this is STOW_OBJECT's own
  // motion (execute()'s primary retreatToSafe() call) as well as every
  // other failure path's safety retreat, several of which can still be
  // carrying a just-grasped object (section 4.19). Restored immediately
  // after planning so later calls on this same move_group_arm_ (a fresh
  // grasp attempt's MOVE_ARM_TO_PREGRASP, etc.) go back to full speed.
  move_group_arm_->setMaxVelocityScalingFactor(params_.lift_velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params_.lift_acceleration_scaling);
  move_group_arm_->setNamedTarget("zero");
  moveit_msgs::msg::MotionPlanRequest request;
  move_group_arm_->constructMotionPlanRequest(request);
  move_group_arm_->setMaxVelocityScalingFactor(params_.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params_.acceleration_scaling);
  std::string error;
  const bool ok = sendMoveGroupRequest(request, error);
  if (!ok) {
    RCLCPP_ERROR(get_logger(), "retreat-to-safe failed: %s", error.c_str());
  }
  return ok;
}

void PiperManipulator::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GraspObject::Result>();

  const Point3 object_arm{
    goal->object_pose.pose.position.x, goal->object_pose.pose.position.y,
    goal->object_pose.pose.position.z};
  const double approach_yaw_local = std::atan2(object_arm.y, object_arm.x);
  const Pose3 grasp_pose = computeGraspPose(object_arm, approach_yaw_local, params_.grasp_pitch);
  const Pose3 approach_pose = computeApproachPose(
    grasp_pose, params_.approach_distance, params_.approach_height, approach_yaw_local);
  const double lift_height =
    goal->lift_height > 0.0 ? goal->lift_height : params_.default_lift_height;
  const Pose3 lift_pose = computeLiftPose(grasp_pose, lift_height);

  RCLCPP_INFO(
    get_logger(),
    "grasp diagnostics: object_arm=(%.4f,%.4f,%.4f) approach_yaw=%.4f "
    "grasp_pose.position=(%.4f,%.4f,%.4f) grasp_pose.orientation=(%.4f,%.4f,%.4f,%.4f)",
    object_arm.x, object_arm.y, object_arm.z, approach_yaw_local, grasp_pose.position.x,
    grasp_pose.position.y, grasp_pose.position.z, grasp_pose.orientation.x,
    grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);

  std::string error;

  publishStageFeedback(goal_handle, "MOVE_ARM_TO_PREGRASP", 0.0);
  if (!moveArmToPose(toMsg(approach_pose), error)) {
    RCLCPP_ERROR(get_logger(), "MOVE_ARM_TO_PREGRASP failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "MOVE_ARM_TO_PREGRASP failed: " + error;
    goal_handle->abort(result);
    return;
  }
  if (!moveGripperNamed("open", error)) {
    RCLCPP_ERROR(get_logger(), "opening gripper failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "failed to open gripper before grasp: " + error;
    goal_handle->abort(result);
    return;
  }
  publishStageFeedback(goal_handle, "MOVE_ARM_TO_PREGRASP", 1.0);

  publishStageFeedback(goal_handle, "MOVE_ARM_TO_GRASP", 0.0);
  if (!cartesianMoveTo(toMsg(grasp_pose), error)) {
    RCLCPP_ERROR(get_logger(), "MOVE_ARM_TO_GRASP failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "MOVE_ARM_TO_GRASP failed: " + error;
    goal_handle->abort(result);
    return;
  }
  publishStageFeedback(goal_handle, "MOVE_ARM_TO_GRASP", 1.0);
  {
    // Ground truth of where the arm ACTUALLY ended up, vs. the commanded
    // grasp_pose above -- getCurrentPose() is a plain state-monitor read
    // (see moveGripperToWidth()'s getCurrentState() comment), so this is
    // unaffected by the action/service dispatch bug.
    const auto actual = move_group_arm_->getCurrentPose().pose;
    RCLCPP_INFO(
      get_logger(), "grasp diagnostics: actual EE pose after MOVE_ARM_TO_GRASP=(%.4f,%.4f,%.4f)",
      actual.position.x, actual.position.y, actual.position.z);
  }

  publishStageFeedback(goal_handle, "CLOSE_GRIPPER", 0.0);
  bool contact_detected = false;
  if (!moveGripperToWidth(params_.grasp_width, error, contact_detected)) {
    RCLCPP_ERROR(get_logger(), "CLOSE_GRIPPER failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "CLOSE_GRIPPER failed: " + error;
    goal_handle->abort(result);
    return;
  }
  if (!contact_detected) {
    // Motion succeeded (the fingers reached the fully-closed commanded
    // width) but the readback in moveGripperToWidth() found nothing
    // stopped them short -- the object isn't between the fingers. This is
    // a real failure of the grasp, not a motion fault; reuse the existing
    // failure path (plan section 10 part C) rather than adding a new
    // result field, so task_coordinator's existing grasp_goal_success
    // handling -- now with a retry, see TaskFsm -- picks it up unchanged.
    RCLCPP_WARN(
      get_logger(), "CLOSE_GRIPPER: fingers reached the closed target without contact");
    retreatToSafe();
    result->success = false;
    result->message = "gripper closed without contacting the object";
    goal_handle->abort(result);
    return;
  }
  publishStageFeedback(goal_handle, "CLOSE_GRIPPER", 1.0);

  if (params_.post_grasp_settle_time > 0.0) {
    // See Params::post_grasp_settle_time's comment -- diagnostic dwell for
    // the wall-mount task's LIFT_OBJECT slip, testing whether the grip is
    // still physically developing when the lift starts pulling on it.
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.post_grasp_settle_time)));
  }

  publishStageFeedback(goal_handle, "LIFT_OBJECT", 0.0);
  // Slower than the default velocity/acceleration scaling -- this cartesian
  // move carries a just-grasped, marginally-held object, and its
  // acceleration was previously unscaled entirely (see
  // computeCartesianPathRaw()'s doc comment) -- a likely real contributor
  // to the object slipping free mid-lift (section 4.19).
  if (!cartesianMoveTo(
      toMsg(lift_pose), error, params_.lift_velocity_scaling, params_.lift_acceleration_scaling,
      params_.min_lift_fraction))
  {
    RCLCPP_ERROR(get_logger(), "LIFT_OBJECT failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "LIFT_OBJECT failed: " + error;
    goal_handle->abort(result);
    return;
  }
  if (!stillHoldingObject()) {
    // The lift motion itself succeeded, but the object is no longer
    // between the fingers -- it registered real contact at CLOSE_GRIPPER
    // (this function wouldn't have reached here otherwise) and then
    // slipped free during the lift. Same failure path as a CLOSE_GRIPPER
    // miss, so task_coordinator's existing retry (TaskFsm) picks it up
    // unchanged -- see stillHoldingObject()'s doc comment.
    RCLCPP_WARN(get_logger(), "LIFT_OBJECT: object slipped out of the gripper during the lift");
    retreatToSafe();
    result->success = false;
    result->message = "object slipped out of the gripper during the lift";
    goal_handle->abort(result);
    return;
  }
  publishStageFeedback(goal_handle, "LIFT_OBJECT", 1.0);

  publishStageFeedback(goal_handle, "STOW_OBJECT", 0.0);
  // lift_pose alone leaves the grasped object at roughly the lidar's own
  // mount height (LIDAR_LOCAL_POS z=0.35 in import_robots.py), directly
  // in its forward scan cone -- confirmed directly with a live /scan read
  // during a real DRIVE_TO_DELIVERY: the forward cone read ~0.10-0.11m
  // (the lidar's own range_min, i.e. something touching it) the entire
  // time the arm sat in lift_pose, which is exactly why
  // applyObstacleAvoidance() (kotek_base_control) zeroed forward velocity
  // for the whole leg. 'zero' is the SAME pose retreatToSafe() already
  // uses for every other failure path in this function and puts link6
  // clear of the lidar: /piper/compute_fk at the all-zero joint
  // configuration gives link6 at z=0.494m in the scout's own base_link
  // frame (0.203m in the arm's local frame + the 0.29101m arm-mount
  // offset), comfortably above the lidar's fixed mount point at z=0.35m
  // (LIDAR_LOCAL_POS in import_robots.py) -- reusing this proven-safe
  // pose rather than inventing a new one.
  // stow_after_lift=false (wall_mount.yaml) skips the retract entirely --
  // see the Params comment: the stow swing exists for the pedestal task's
  // driving leg, and from the wall task's twisted rear-corner lift poses it
  // physically knocks the (planning-scene-invisible) held box out of the
  // gripper. PlaceObject's own MOVE_ARM_TO_PREPLACE then plans directly
  // from the lift pose.
  if (params_.stow_after_lift) {
    if (!retreatToSafe()) {
      RCLCPP_ERROR(get_logger(), "STOW_OBJECT failed: could not retract to 'zero'");
      result->success = false;
      result->message = "STOW_OBJECT failed: could not retract to 'zero'";
      goal_handle->abort(result);
      return;
    }
    if (!stillHoldingObject()) {
      // The stow motion itself succeeded, but it's the same kind of
      // uncontrolled, non-collision-aware move retreatToSafe() always was
      // (the grasped object isn't registered as an attached collision
      // object, so MoveIt has no reason to avoid swinging it into the
      // chassis on the way to 'zero') -- confirm the object survived the
      // trip before declaring success, same reasoning as the post-lift
      // recheck above.
      RCLCPP_WARN(get_logger(), "STOW_OBJECT: object slipped out of the gripper while stowing");
      result->success = false;
      result->message = "object slipped out of the gripper while stowing";
      goal_handle->abort(result);
      return;
    }
  }
  publishStageFeedback(goal_handle, "STOW_OBJECT", 1.0);

  result->success = true;
  result->message = "grasp complete";
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PiperManipulator::handlePlaceGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PlaceObject::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PiperManipulator::handlePlaceCancel(
  const std::shared_ptr<PlaceGoalHandle>)
{
  // See handleCancel()'s comment above -- same limitation applies here.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PiperManipulator::handlePlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal_handle)
{
  std::thread{std::bind(&PiperManipulator::executePlace, this, goal_handle)}.detach();
}

void PiperManipulator::publishPlaceStageFeedback(
  const std::shared_ptr<PlaceGoalHandle> & goal_handle, const std::string & stage,
  double progress)
{
  auto feedback = std::make_shared<PlaceObject::Feedback>();
  feedback->stage = stage;
  feedback->progress = progress;
  goal_handle->publish_feedback(feedback);
}

void PiperManipulator::executePlace(const std::shared_ptr<PlaceGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlaceObject::Result>();

  // Same geometry as the grasp sequence's approach/grasp/lift poses --
  // "place" is just "grasp" with the roles of pre-pose and post-pose
  // swapped (approach from above before placing, retreat upward after
  // releasing instead of lifting immediately after closing).
  const Point3 place_arm{
    goal->place_pose.pose.position.x, goal->place_pose.pose.position.y,
    goal->place_pose.pose.position.z};
  const double approach_yaw_local = std::atan2(place_arm.y, place_arm.x);
  const Pose3 place_pose = computeGraspPose(place_arm, approach_yaw_local, params_.place_pitch);
  const Pose3 preplace_pose = computeApproachPose(
    place_pose, params_.place_approach_distance, params_.approach_height,
    approach_yaw_local);
  const double retreat_height =
    goal->retreat_height > 0.0 ? goal->retreat_height : params_.default_retreat_height;
  const Pose3 retreat_pose = computeLiftPose(place_pose, retreat_height);

  // Mirrors the grasp path's diagnostics line. Its absence here is exactly why
  // a MOVE_ARM_TO_PREPLACE failure was previously opaque: the coordinator logs
  // the place POINT, but the pose actually handed to the planner is two
  // transforms downstream of it.
  RCLCPP_INFO(
    get_logger(),
    "place diagnostics: place_arm=(%.4f,%.4f,%.4f) approach_yaw=%.4f "
    "place_pose.position=(%.4f,%.4f,%.4f) preplace_pose.position=(%.4f,%.4f,%.4f) "
    "place_radius=%.4f preplace_radius=%.4f",
    place_arm.x, place_arm.y, place_arm.z, approach_yaw_local,
    place_pose.position.x, place_pose.position.y, place_pose.position.z,
    preplace_pose.position.x, preplace_pose.position.y, preplace_pose.position.z,
    std::hypot(place_pose.position.x, place_pose.position.y),
    std::hypot(preplace_pose.position.x, preplace_pose.position.y));

  std::string error;

  // A lost box used to sail through the whole place leg silently (the arm
  // "placed" air and reported success while the box lay back on the riser
  // or the floor) -- fail loudly instead, before moving anywhere.
  if (!stillHoldingObject()) {
    RCLCPP_ERROR(get_logger(), "place aborted: no object held at place start");
    result->success = false;
    result->message = "no object held at place start (lost during carry?)";
    goal_handle->abort(result);
    return;
  }

  publishPlaceStageFeedback(goal_handle, "MOVE_ARM_TO_PREPLACE", 0.0);
  if (!moveArmToPose(
      toMsg(preplace_pose), error, params_.carry_velocity_scaling,
      params_.carry_acceleration_scaling))
  {
    RCLCPP_ERROR(get_logger(), "MOVE_ARM_TO_PREPLACE failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "MOVE_ARM_TO_PREPLACE failed: " + error;
    goal_handle->abort(result);
    return;
  }
  publishPlaceStageFeedback(goal_handle, "MOVE_ARM_TO_PREPLACE", 1.0);

  publishPlaceStageFeedback(goal_handle, "MOVE_ARM_TO_PLACE", 0.0);
  if (!cartesianMoveTo(
      toMsg(place_pose), error, params_.carry_velocity_scaling,
      params_.carry_acceleration_scaling))
  {
    RCLCPP_ERROR(get_logger(), "MOVE_ARM_TO_PLACE failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "MOVE_ARM_TO_PLACE failed: " + error;
    goal_handle->abort(result);
    return;
  }
  publishPlaceStageFeedback(goal_handle, "MOVE_ARM_TO_PLACE", 1.0);

  publishPlaceStageFeedback(goal_handle, "OPEN_GRIPPER", 0.0);
  if (!moveGripperNamed("open", error)) {
    RCLCPP_ERROR(get_logger(), "OPEN_GRIPPER failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "OPEN_GRIPPER failed: " + error;
    goal_handle->abort(result);
    return;
  }
  publishPlaceStageFeedback(goal_handle, "OPEN_GRIPPER", 1.0);

  if (params_.release_settle_time > 0.0) {
    // See Params::release_settle_time's comment -- the magnet begins
    // capturing the box while the fingers are still open around it; give
    // that capture a chance to finish before the retreat moves the arm
    // back up through the box's airspace.
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.release_settle_time)));
  }

  publishPlaceStageFeedback(goal_handle, "RETREAT_ARM", 0.0);
  if (!cartesianMoveTo(toMsg(retreat_pose), error)) {
    RCLCPP_ERROR(get_logger(), "RETREAT_ARM failed: %s", error.c_str());
    retreatToSafe();
    result->success = false;
    result->message = "RETREAT_ARM failed: " + error;
    goal_handle->abort(result);
    return;
  }
  publishPlaceStageFeedback(goal_handle, "RETREAT_ARM", 1.0);

  result->success = true;
  result->message = "place complete";
  goal_handle->succeed(result);
}

}  // namespace kotek_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kotek_manipulation::PiperManipulator>();
  node->init(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
