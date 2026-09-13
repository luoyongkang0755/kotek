#ifndef KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_
#define KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_

#include <memory>
#include <string>

#include "kotek_manipulation/pregrasp_geometry.hpp"
#include "kotek_msgs/action/grasp_object.hpp"
#include "kotek_msgs/action/place_object.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace kotek_manipulation
{

/// Hosts the GraspObject and PlaceObject action servers. Runs inside the
/// /piper namespace (see plan section 6/9) so its MoveGroupInterface talks
/// to the namespaced move_group without any special options -- and so that
/// the actions are exposed at /piper/grasp_object and /piper/place_object.
/// Both servers share the same MoveGroupInterface handles (one arm, one
/// gripper -- there is only ever one physical arm to plan for, so a second
/// action executing concurrently would just fail to plan/execute against a
/// group already mid-motion, which is an acceptable failure mode here).
///
/// The incoming goal's object_pose/place_pose is expected already expressed
/// in the arm's own base frame (kotek_task_coordinator does the odom->arm-
/// frame transform via pregrasp_geometry::transformPointOdomToArmFrame
/// before sending the goal -- see plan section 6), so this node needs no TF
/// lookup of the scout's pose.
class PiperManipulator : public rclcpp::Node
{
public:
  using GraspObject = kotek_msgs::action::GraspObject;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GraspObject>;
  using PlaceObject = kotek_msgs::action::PlaceObject;
  using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<PlaceObject>;

  PiperManipulator();

  /// Two-phase init: MoveGroupInterface needs a shared_ptr<rclcpp::Node>,
  /// which isn't available from inside our own constructor. Call this once
  /// immediately after construction (see main()).
  void init(const std::shared_ptr<PiperManipulator> & self);

private:
  struct Params
  {
    std::string arm_group = "arm";
    std::string gripper_group = "gripper";
    double approach_distance = 0.12;
    double approach_height = 0.10;
    // Horizontal back-off for the PLACE approach, separate from the grasp's.
    // Defaults to 0, i.e. approach the place point straight down. A grasp has
    // to come in from behind-and-above so the fingers clear the object they
    // are reaching around; a place does not -- the object is already in the
    // hand and there is nothing to avoid. Backing off horizontally only pulls
    // the arm IN toward its own base, which is what made MOVE_ARM_TO_PREPLACE
    // unplannable: with approach_distance=0.12 the pre-place pose landed at
    // arm-frame radius 0.281 against a pre-grasp at 0.417 that plans fine, and
    // OMPL returned FAILURE. It also removes this stage's sensitivity to base
    // parking error, which is +/-0.07 m in practice.
    double place_approach_distance = 0.0;
    double default_lift_height = 0.15;
    double default_retreat_height = 0.15;
    double grasp_width = 0.024;
    double gripper_open = 0.04;
    // Split from a single shared gripper_pitch (docs/pick_and_delivery_report.md's
    // tuning history) once the wall-mount task's live /piper/compute_ik sweep
    // (with the riser cube and wall modeled as real MoveIt CollisionObjects,
    // not just self-collision) found the pedestal-tuned 0.4 rad works for
    // neither of that task's two legs: picking a sensor off the riser cube
    // (close to the arm's own shoulder) needs a much steeper ~1.2 rad, while
    // placing against the wall needs ~0.7 rad. Both default to the pedestal
    // demo's original tuned value so that demo is unaffected; the wall-mount
    // launch overrides grasp_pitch (and, if needed, place_pitch) via its own
    // wall_mount.yaml -- see that file's cross-reference comment.
    double grasp_pitch = 0.4;
    double place_pitch = 0.4;
    // Wall-mount grasp anti-V-grip fix (2026-09-13, measured): the riser
    // corners sit at approach_yaw = atan2(+-0.109, +-0.109) = +-45 deg, so
    // the gripper's opening axis (computeGraspPose's
    // cross(world_up, approach_dir), always horizontal and perpendicular to
    // the approach) also lands at 45 deg to the box's edges -- the jaws close
    // diagonally across the box corner in a V, and nothing geometrically
    // prevents the box yawing/rolling as the fingers stall (the stab-ON E2E
    // delivered the 4 boxes at 39.3/90/51.2/90 deg misalignment; see
    // physics_tuning.py's MAGNET_ATTRACT_RANGE history). Snap the approach
    // yaw to a multiple of this step (0 = off, pedestal demo unchanged) so
    // the jaws close PARALLEL to an edge pair instead of diagonally across
    // the object's corner. wall_mount.yaml uses pi, NOT pi/2: this box is
    // 8cm along X but only 3.5cm along Y and the gripper opens 4cm max, so
    // the jaws can straddle the Y width only -- which needs the approach
    // along +-X (opening axis along Y). pi/2 (approach along +-Y) would
    // demand an impossible 8cm straddle.
    double grasp_yaw_snap_step = 0.0;
    // Wall-mount place reorientation (2026-09-13, measured): the wall
    // target accepts the box only with its magnet face (local -Z) toward
    // the wall and its 8cm edge vertical, but the box is carried FLAT from
    // the riser. When true, executePlace() overrides the place orientation
    // with a FIXED absolute TCP frame (opening axis -Y, fingertip axis
    // (sin grasp_pitch, 0, -cos grasp_pitch) -- at the wall, tilted DOWN)
    // and recomputes link6's position so the FINGERTIPS still land on the
    // commanded place point. The relative rotation from either snapped
    // grasp orientation (yaw 0/pi) to this frame is exactly the box's
    // flat->vertical rotation (verified column-by-column for both yaw
    // signs); the downward fingertip tilt keeps link6 above the arm's
    // measured lower workspace boundary (z ~ -0.05 m, live OMPL sweep),
    // which the plain Ry(-90) composite violated by 12 cm. Requires
    // grasp_yaw_snap_step to be active so the box is picked up flat and
    // axis-aligned; false = pedestal demo unchanged.
    bool place_reorient = false;
    double min_cartesian_fraction = 0.9;
    double velocity_scaling = 0.2;
    double acceleration_scaling = 0.2;
    // Slower than velocity_scaling/acceleration_scaling above -- used only
    // for LIFT_OBJECT's cartesian move and STOW_OBJECT's retreat, the two
    // motions that carry a just-grasped, marginally-held object (see
    // cartesianMoveTo()/retreatToSafe() call sites in execute()). A fast
    // lift/retreat is exactly the kind of acceleration that shakes a
    // marginal grip loose -- docs/pick_and_delivery_report.md section 4.19.
    double lift_velocity_scaling = 0.08;
    double lift_acceleration_scaling = 0.08;
    // Diagnostic/tuning knob for the wall-mount task's LIFT_OBJECT slip: a
    // pause between CLOSE_GRIPPER's contact confirmation and LIFT_OBJECT's
    // cartesian move starting, in case PhysX's contact resolution needs a
    // few ticks after first detecting contact to build full squeeze force --
    // moveGripperToWidth()'s contact check and the lift's first planning
    // request were observed back-to-back (4ms apart) in a live log, so a
    // marginal grip has had essentially no time to develop before the lift
    // starts pulling on it. Defaults to 0 (no change to the pedestal demo's
    // timing); wall_mount.yaml sets this nonzero to test the hypothesis.
    double post_grasp_settle_time = 0.0;
    // Pause between OPEN_GRIPPER and RETREAT_ARM in executePlace(), in
    // seconds. The wall-mount release point sits INSIDE the magnet's attract
    // range (4.5cm short of the wall face vs MAGNET_ATTRACT_RANGE=5cm), so
    // the magnet starts capturing the box the moment the fingers release it
    // -- a live wall-mount run's joint telemetry caught the released box
    // being yanked against the still-opening fingers (joint7 pried 0.019 ->
    // 0.131 in ~1s) and the following full-speed (0.2-scaling) retreat then
    // dragged the not-yet-captured box back into the arm, where it jammed in
    // the elbow links and never reached the wall at all. A settle pause lets
    // the magnet finish pulling the box clear of the fingers BEFORE the
    // arm moves back up through the box's airspace. Defaults to 0 (pedestal
    // demo unchanged -- its release is onto a pedestal surface, with no
    // magnet racing the retreat); wall_mount.yaml sets this nonzero.
    double release_settle_time = 0.0;
    // STOW_OBJECT (retract to 'zero' after the lift) exists to clear the
    // lidar's forward scan cone for the pedestal task's DRIVE_TO_DELIVERY
    // leg. The wall-mount task never drives, and its rear-corner grasps
    // (approach_yaw ~225deg) leave the arm so twisted at the lift pose that
    // the stow swing drags the HELD box through the robot -- MoveIt doesn't
    // model the grasped object (no AttachedCollisionObject), so it happily
    // plans the arm clear while the box in the fingers is knocked out
    // (observed live: joint7 pried 11cm past its 4cm limit, then "object
    // slipped out of the gripper while stowing" on 3 consecutive corner-3
    // runs). wall_mount.yaml sets this false; the pedestal demo keeps the
    // default. The deeper fix is registering the held object as an
    // AttachedCollisionObject -- tracked as follow-up work.
    bool stow_after_lift = true;
    // Minimum acceptable cartesian fraction for LIFT_OBJECT specifically
    // (every other cartesian move keeps min_cartesian_fraction). Defaults
    // to 1.1 = sentinel for "use min_cartesian_fraction", preserving the
    // pedestal demo's behavior exactly. The wall-mount task sets
    // lift_height deliberately higher than the arm's cartesian ceiling at
    // some corners (the carry must clear the riser) with this at 0.5:
    // "lift as high as the arm can, at least half the requested height".
    double min_lift_fraction = 1.1;
    // Speed scaling for LOADED carries: PlaceObject's preplace swing and
    // place descent, i.e. every big arm motion made while an object is in
    // the gripper. Defaults to -1 = sentinel for "use velocity_scaling/
    // acceleration_scaling" (pedestal behavior unchanged). The wall-mount
    // task sets these to the slow lift scalings: at the normal 0.2 the
    // preplace swing's inertial torque pivoted the box out of the fingers
    // mid-carry (PhysX point contacts carry no torsional friction, so a
    // fast yaw swing spins the box about the finger-contact line -- boxes
    // were found fallen directly under the swing path, landed upright).
    double carry_velocity_scaling = -1.0;
    double carry_acceleration_scaling = -1.0;
    double planning_time = 5.0;
    int planning_attempts = 10;
    // Max seconds to wait for a /piper/move_action result once the goal is
    // accepted. On this Isaac stage a full plan+execute legitimately takes
    // 30s+ (preplace measured ~35s: 5s OMPL plan plus a ~25-30s real-time
    // trajectory at the sim's low joint-state rate). A hardcoded 30s timed
    // out on the first preplace of every E2E run, the retry then sent a
    // second concurrent goal while the first trajectory was still executing
    // -- which segfaulted move_group ("Cannot push a new trajectory while
    // another is being executed", exit -11, 2026-09-13). Default 120s.
    double move_action_result_timeout = 120.0;
  };

  Params loadParams();

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GraspObject::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  void publishStageFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle, const std::string & stage,
    double progress);

  rclcpp_action::GoalResponse handlePlaceGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlaceObject::Goal> goal);
  rclcpp_action::CancelResponse handlePlaceCancel(
    const std::shared_ptr<PlaceGoalHandle> goal_handle);
  void handlePlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal_handle);

  void executePlace(const std::shared_ptr<PlaceGoalHandle> goal_handle);

  void publishPlaceStageFeedback(
    const std::shared_ptr<PlaceGoalHandle> & goal_handle, const std::string & stage,
    double progress);

  /// Plans and executes the arm to `pose` (in the arm's planning frame).
  /// Returns true on success.
  /// Joint-space plan+move to `pose`. `velocity_scaling`/
  /// `acceleration_scaling` default to the normal planning scalings; the
  /// loaded preplace swing passes the carry scalings (see Params).
  bool moveArmToPose(
    const geometry_msgs::msg::Pose & pose, std::string & error_out,
    double velocity_scaling = -1.0, double acceleration_scaling = -1.0);

  /// Cartesian move from wherever the arm currently is to `pose`. Returns
  /// true only if computeCartesianPath's fraction reaches `min_fraction`
  /// (default: params_.min_cartesian_fraction) and execution succeeds.
  /// `velocity_scaling`/`acceleration_scaling` default to
  /// params_.velocity_scaling/acceleration_scaling; LIFT_OBJECT's call
  /// passes the slower lift scalings explicitly, plus
  /// params_.min_lift_fraction so a lift toward a deliberately-ambitious
  /// lift_height can succeed at "as high as the arm reaches" (the partial
  /// trajectory is still executed to wherever it ends).
  bool cartesianMoveTo(
    const geometry_msgs::msg::Pose & pose, std::string & error_out,
    double velocity_scaling = -1.0, double acceleration_scaling = -1.0,
    double min_fraction = -1.0);

  bool moveGripperNamed(const std::string & name, std::string & error_out);

  /// Plans/executes the gripper to `width` (full opening, meters -- joint7's
  /// commanded target is width/2 per the URDF's symmetric two-finger
  /// mimic). On success, also reads back the ACTUAL achieved joint7
  /// position (a plain CurrentStateMonitor read via
  /// move_group_gripper_->getCurrentState(), confirmed in section 4.15 to
  /// be unaffected by the action/service dispatch bug that raw_* clients
  /// exist to work around) and compares it to the commanded target: if the
  /// fingers stopped meaningfully short of the target (contact_tolerance),
  /// something was between them (grasped); if they reached ~the full
  /// commanded width, nothing was there (missed). This is necessary
  /// because isaac_joint_bridge.py's own stall-detection already turns
  /// "stopped short due to contact" into a MoveIt-level SUCCESS (plan
  /// section 9) -- motion success alone can't tell a real grasp from
  /// closing on empty air. Sets `contact_detected_out` only when the
  /// motion itself succeeded; leave its input value untouched otherwise.
  bool moveGripperToWidth(double width, std::string & error_out, bool & contact_detected_out);

  /// Re-reads joint7 against the SAME commanded-width/contact-tolerance
  /// logic as moveGripperToWidth()'s post-motion check, WITHOUT commanding
  /// any new motion -- used after LIFT_OBJECT to catch a grip that closed
  /// on the object (a real contact, per moveGripperToWidth()) but then
  /// slipped free during the lift, which moveGripperToWidth() alone can't
  /// see since it only checks once, right after CLOSE_GRIPPER. Returns
  /// false (not holding) if the state read itself fails, treating "can't
  /// confirm" the same as "lost it" -- the safer default for a retry
  /// decision.
  bool stillHoldingObject();

  bool retreatToSafe();

  /// Sends `request` to /piper/move_action directly via `raw_move_client_`
  /// (plan_only=false, so move_group both plans AND executes server-side),
  /// bypassing MoveGroupInterface's own plan()/move() -- see the member
  /// declaration's comment for why those are unusable here. Blocks the
  /// calling thread until the goal completes.
  ///
  /// Retries transient failures up to kMoveGroupAttempts times with a short
  /// pause between attempts: live full-demo runs showed single-shot MoveIt
  /// flakes (an OMPL plan that aborts instantly with status=6, a result
  /// timeout that leaves move_group's executor briefly refusing new
  /// trajectories with "Cannot push a new trajectory while another is being
  /// executed") aborting a whole otherwise-healthy pick+mount cycle. These
  /// requests are pure target-seeking (joint/pose goals), so re-sending the
  /// identical request is always safe -- worst case the arm is already at
  /// the target and the retry is a no-op plan.
  bool sendMoveGroupRequest(
    const moveit_msgs::msg::MotionPlanRequest & request, std::string & error_out);

  /// Single attempt of sendMoveGroupRequest() -- the original send/await
  /// logic, factored out so the retry wrapper stays readable.
  bool sendMoveGroupRequestOnce(
    const moveit_msgs::msg::MotionPlanRequest & request, std::string & error_out);

  /// Sends `trajectory` to /piper/execute_trajectory directly via
  /// `raw_execute_client_`, bypassing MoveGroupInterface::execute(). Blocks
  /// the calling thread until the goal completes. Retries transient
  /// failures the same way sendMoveGroupRequest() does (see its comment).
  bool sendExecuteTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & error_out);

  /// Single attempt of sendExecuteTrajectory().
  bool sendExecuteTrajectoryOnce(
    const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & error_out);

  /// Calls /piper/compute_cartesian_path directly via
  /// `raw_cartesian_path_client_`, bypassing MoveGroupInterface::
  /// computeCartesianPath() -- see the member declaration's comment;
  /// affected by the exact same broken-callback_group_ issue as the action
  /// clients. Blocks the calling thread until the service responds.
  /// Returns the path fraction achieved (0.0 if the call itself failed).
  /// `velocity_scaling`/`acceleration_scaling` are forwarded to the
  /// service's own max_velocity_scaling_factor/max_acceleration_scaling_
  /// factor fields -- previously left unset here (silently defaulting to
  /// full, unscaled speed per GetCartesianPath.srv's own doc comment,
  /// regardless of params_.velocity_scaling/acceleration_scaling) until
  /// section 4.19 wired them through.
  double computeCartesianPathRaw(
    const geometry_msgs::msg::Pose & pose, moveit_msgs::msg::RobotTrajectory & trajectory_out,
    std::string & error_out, double velocity_scaling, double acceleration_scaling);

  Params params_;

  // MoveGroupInterface needs its given node to already be actively spinning
  // to complete construction at all (it blocks waiting for the first
  // /joint_states message to build its CurrentStateMonitor) and to keep
  // working afterward -- but main() doesn't start spinning `self` until
  // AFTER init() returns, so passing `self` directly deadlocks init()
  // itself before it ever reaches executor.spin(). Confirmed directly:
  // with `self` shared between both instances, the node logged the arm
  // group's robot model/kinematics loading and then simply never printed
  // its own "piper_manipulator ready" line, and never responded to ANY
  // request afterward -- not just grasp_object, even a plain
  // get_parameters service call timed out, because the process never
  // reached executor.spin() at all. Fixed by giving each MoveGroupInterface
  // its own dedicated internal node, spun on our own background executor
  // for their whole lifetime (see init()) -- independent of main()'s own
  // executor/spin timing for `self`.
  rclcpp::Node::SharedPtr move_group_node_arm_;
  rclcpp::Node::SharedPtr move_group_node_gripper_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> internal_executor_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;

  // MoveGroupInterface's own internal move_action_client_/execute_action_
  // client_ are built with an explicit, non-auto-executor-added
  // callback_group_ (moveit_ros_planning_interface's own implementation
  // detail, not something the public constructor lets a caller override).
  // Their action_server_is_ready() (called fresh on every plan()/move()
  // call, not cached) never returns true in this environment -- confirmed
  // directly via gdb across many variants, and by reading the exact
  // installed moveit_ros_planning_interface 2.12.4 source. A bare,
  // plain-callback-group rclcpp_action::Client built directly on `self`,
  // targeting the exact same /piper/move_action and
  // /piper/execute_trajectory actions, reliably works (~2s to ready). These
  // two clients bypass MoveGroupInterface's broken dispatch while still
  // using MoveGroupInterface (constructMotionPlanRequest()) to build the
  // request content -- see sendMoveGroupRequest()/sendExecuteTrajectory().
  //
  // `computeCartesianPath()` is a SERVICE call (`cartesian_path_service_`),
  // built the same broken way -- confirmed directly in a live run: it hung
  // forever (no error, no timeout, node otherwise still responsive) on the
  // very first real MOVE_ARM_TO_GRASP call once the action-dispatch and
  // orientation bugs were both already fixed. `raw_cartesian_path_client_`
  // bypasses it the same way as the two action clients above.
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr raw_move_client_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr raw_execute_client_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr raw_cartesian_path_client_;

  rclcpp_action::Server<GraspObject>::SharedPtr action_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_action_server_;
};

}  // namespace kotek_manipulation

#endif  // KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_
