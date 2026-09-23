#ifndef KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_
#define KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "kotek_manipulation/pregrasp_geometry.hpp"
#include "kotek_msgs/action/grasp_object.hpp"
#include "kotek_msgs/action/place_object.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
    // --- Wall-mount Task 2a (2026-09-18, measured). The 50-run batch with
    // the 23 deg gate showed carry slips are the SOLE remaining failure
    // class (32/32 unwelded boxes), and per-tick magnet-graph traces of
    // slipping runs (e.g. baseline batch run_05 sensor4) show the box slowly
    // pivoting in the fingers through the ENTIRE carry at 4.5-7.7 rad/s --
    // the stow->preplace swing is the longest loaded segment and its
    // inertial torque is what works the grip loose (PhysX point contacts,
    // no torsional friction). carry_*_scaling is already down to 0.08;
    // these swing_* values apply ONLY when the current-TCP-to-preplace
    // distance exceeds swing_distance_threshold, so short carries keep the
    // 0.08 pace and the ~379 s per-run budget stays intact (smoke-measured
    // swing distances 0.235-0.362 m across the 4 targets; predicted +~15 s
    // on each long leg at 0.06).
    double swing_distance_threshold = 0.25;      // m
    double swing_velocity_scaling = 0.06;
    double swing_acceleration_scaling = 0.06;
    // --- Wall-mount Task 2b (2026-09-18, measured). Box-in-hand
    // verification from the stage-published sensor_cam_N TF frames
    // (kotek_wall_sensor_graph; its DDS-domain bug -- publishing on domain
    // 0 regardless of ROS_DOMAIN_ID -- was found and fixed 2026-09-18, see
    // build_wall_stage.py's add_sensor_tf_graph). The joint7-width check
    // above CANNOT see a box knocked out of stationary fingers: the fingers
    // keep their stall width, and every one of the 50-run batch's 32 slips
    // passed stillHoldingObject() at place start while the box lay on the
    // floor. The goal names the carried frame (object_frame); its distance
    // to the TCP is recorded at place start and re-checked right BEFORE
    // OPEN_GRIPPER -- a growing distance means the box is slipping out,
    // which the joint7 check cannot see.
    // TF positions come back in the planning frame directly (measured:
    // base_link->sensor_cam_1 == riser corner (0.109,0.109,0.01929) to 3
    // decimals). Failures abort the place loudly -- a lost box must not
    // "place air" and report success.
    //
    // Identity: the PlaceObject goal carries the object frame explicitly
    // (object_frame, e.g. sensor_cam_2). Nearest-frame guessing was tried
    // and REJECTED: at the stow pose all four riser boxes plus the carried
    // one sit in a tight cone from the TCP (tcp distances 0.185-0.287 m)
    // and riser boxes repeatedly won the "nearest" race by millimetres
    // (smoke2/5 2026-09-18).
    //
    // Metric: RIGID-LINK distance growth, no fingertip geometry at all --
    // a held box keeps a CONSTANT box-center-to-TCP distance through every
    // wrist orientation, so the check is |d_now - d_latch| and pose-based
    // fingertip metrics are unnecessary (smoke4/5 showed the 0.13503 offset
    // direction is pose-dependent and ambiguous). Measured: a stable carry
    // drifts ~0.02 m (fingers re-seat during the descent); a lost box
    // lands >=0.15 m farther.
    double held_box_max_distance = 0.30;   // m, absolute sanity at latch
    double held_box_slip_margin = 0.07;    // m, allowed d growth pre-release
    // Grasp-orientation sanity, pre-release only (2026-09-20, measured):
    // the e2e10_task2 batch's 6 unwelded boxes all reached the release point
    // RIGIDLY held (distance growth ~0.00-0.03) but rotated 62-90 deg in the
    // gripper -- the grasp-close contact kick spins the box about the
    // finger-contact line, the magnet face ends up pointing away from the
    // wall, capture fails and the box falls. A held-but-unpresentable box
    // must fail loudly too. With place_reorient, a healthy carried box has
    // its -Z (magnet face) aligned with the reorient approach_dir
    // (cos(tilt), 0, -sin(tilt)); fail above this angle. Inactive when
    // place_reorient is false (grasp orientation is task-specific there).
    double held_box_max_tilt_deg = 30.0;
    // Grasp-leg companion to held_box_max_tilt_deg (2026-09-20, measured):
    // checked right after CLOSE_GRIPPER while the box still rests on the
    // riser -- the flat-lying box's -Z must point straight down; a larger
    // angle means the close-kick rotated it, and the (reopened, retreated)
    // grasp aborts so the caller can re-grasp. Slightly looser than the
    // pre-release gate: this one fires before any lift, where a marginal
    // box can still be re-grasped, and healthy grasps were measured at
    // 19-29 deg pre-release (settling) vs kicks at 50-97 deg. Same value
    // as the pre-release gate on purpose (2026-09-20): a 35 deg grasp gate
    // created a 30-35 deg deadband where boxes passed here but failed at
    // release (e2e10_task2_retry runs 7/10) -- reject early, re-grasp.
    double grasp_kick_max_tilt_deg = 30.0;
    // Anti-kick slow close (2026-09-23, e2e10_holddown A/B): seconds for the
    // CLOSE_GRIPPER finger ramp. PhysX point contacts have no torsional
    // friction, so the close-kick rotates the box in the fingers 30-50% of
    // attempts; the hypothesis is that a quasi-static close lets sliding
    // friction self-center the box instead of kicking it. Velocity-scaling
    // the move_group request was tried FIRST and measured DEAD: the planned
    // trajectory stayed 0.6s regardless of max_velocity_scaling_factor
    // (e2e10_holddown_slowclose, param confirmed loaded), so this sends the
    // close trajectory DIRECTLY to isaac_joint_bridge's FollowJointTrajectory
    // action with hand-set timestamps (moveGripperToWidthTimed). 0 = legacy
    // move_group close. See docs/wall_mount_contact_placement.md.
    double grasp_close_duration = 0.0;
    // Split the ramp into this many ramp-and-hold cycles (creep close):
    // between segments the box's residual motion dies before the next push.
    // 1 = one smooth ramp.
    int grasp_close_steps = 1;
    // TF frame correction (2026-09-20, measured -- CRITICAL): the stage's
    // sensor_cam TF tree publishes box transforms relative to the SCOUT
    // base_link, whose name COLLIDES with the arm planning frame's
    // base_link in the merged TF tree -- so a raw base_link->sensor_cam_N
    // lookup returns positions in the scout/world frame (riser box at
    // z=0.310, see the task's own SENSOR_CAM_LOCAL_Z) while getCurrentPose
    // and the planner work in the ARM frame (riser box at z=0.019). The two
    // frames share x/y origin and differ only by the arm mount height.
    // Verified live: planning with the raw TF z (0.310) made pregrasp
    // unplannable (grasp_pose z=0.436, status=6, smoke9); subtracting this
    // constant restores it. The held-box distance check also mixes frames
    // without this correction (constant z offset) -- it separated well
    // enough empirically, but corrected distances are exact.
    double arm_mount_height = 0.29101;   // m, coordinator.yaml arm_mount_xyz.z
    // --- Realism pass (2026-09-21): model the carried box as an
    // ATTACHED collision object during the carry, so MoveIt plans
    // (preplace swing, place descent, retreat) account for the box and
    // can never drive its corners into the wall. This is what finally
    // allows the release point to sit ~2.5cm from the wall (face gap),
    // which the shortened MAGNET_ATTRACT_RANGE=0.03 demands -- before,
    // the release had to stay 4.5-9cm short precisely because the
    // planner was blind to the box (the corner-pry bug,
    // wall_mount_task.py's _WALL_X comment). Attached after CLOSE_GRIPPER
    // passes the kick check (box confirmed held and healthy), detached
    // after OPEN_GRIPPER. Published as PlanningScene diffs on
    // /piper/planning_scene (direct publisher -- see the raw_* clients'
    // comment for why nothing MoveGroupInterface-internal is used).
    bool attach_carried_box = false;
    double carried_box_size_x = 0.08;    // m, sensor box 8 x 3.5 x 4
    double carried_box_size_y = 0.035;
    double carried_box_size_z = 0.04;
    // Contact-placement press dwell (2026-09-22, batch-5 measured): seconds
    // to hold the arm still AT the press pose AFTER MOVE_ARM_TO_PLACE and
    // BEFORE the pre-release checks / OPEN_GRIPPER. The box crosses into
    // the 12mm magnet range while the arm is still descending (~0.15 m/s
    // measured) and gets yanked through the window (run_03: one in-range
    // tick at d=0.0049, speed 0.154, ang_speed 3.2, then lost). The dwell
    // lets damping + the magnet bring box AND arm to rest against the wall
    // (now safe: the contact-triggered force law keeps the unpressed force
    // at 15%, below the grip's friction budget); the fingers then open
    // onto a still, magnet-pressed box and the weld gates pass immediately.
    double press_dwell_time = 0.0;   // s; 0 = legacy behavior
    // Contact-holddown "press to stick" (2026-09-23): after the position
    // is confirmed, push this far along the wall normal (+X) to engage
    // the contact-triggered magnet (the magnet only exists once the box
    // face touches the wall -- this press IS the engage moment). 0 =
    // no press (legacy / non-wall demos).
    double press_distance = 0.0;     // m
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

  /// Hand-timed variant of moveGripperToWidth (see Params::grasp_close_duration):
  /// sends a `steps`-segment linear joint7/joint8 ramp directly to
  /// isaac_joint_bridge's FollowJointTrajectory action, total wall duration
  /// `duration` seconds, bypassing move_group entirely -- the bridge
  /// interpolates the trajectory points 1:1 and its stall detection reports
  /// contact-stall as SUCCESS, so the semantics match the move_group path.
  /// Same post-motion joint7 readback contact check as moveGripperToWidth().
  bool moveGripperToWidthTimed(
    double width, double duration, int steps, std::string & error_out, bool & contact_detected_out);

  /// Post-close joint7 readback contact test, shared by moveGripperToWidth()
  /// and moveGripperToWidthTimed() -- see kContactTolerance's calibrated
  /// rationale at moveGripperToWidth().
  bool gripperContactReadback(double target_joint7, bool & contact_detected_out);

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

  /// Task 2b box-in-hand verification from the stage-published sensor_cam_N
  /// TF frames -- see Params::held_box_max_distance for the full measured
  /// rationale. `object_frame` (from the PlaceObject goal) names the carried
  /// box; at place start (`latch`=true) its current box-to-TCP distance d0 is
  /// recorded, and before release (`latch`=false) the check fails if the
  /// distance has grown past d0 + Params::held_box_slip_margin (rigid-link
  /// test: a held box keeps a CONSTANT distance to the TCP in any wrist
  /// orientation, so no fingertip-frame geometry is needed -- smoke4/5
  /// 2026-09-18 proved pose-based fingertip metrics ambiguous at the stow
  /// pose, where all four riser boxes sit in a tight cone from the TCP).
  /// Returns true (check passed / nothing to check) and fills `why_not` on
  /// failure. Stays passive (warns once) when `object_frame` is empty or its
  /// TF is unavailable.
  bool heldBoxNearTcp(const std::string & object_frame, bool latch, std::string & why_not);

  /// Realism pass: attach the carried box (object_frame's TF pose) to the
  /// arm end-effector as an AttachedCollisionObject via a PlanningScene
  /// diff on /piper/planning_scene, so subsequent plans model the box.
  /// Detach (removeCarriedBox) after OPEN_GRIPPER. Both no-op unless
  /// Params::attach_carried_box is true; TF failure logs and continues
  /// unattached (planning then has the old blind-to-the-box behavior).
  void attachCarriedBox(const std::string & object_frame);
  void removeCarriedBox();

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
  // Direct FollowJointTrajectory client to isaac_joint_bridge's gripper
  // server (same bare-client rationale as raw_move_client_ above) --
  // moveGripperToWidthTimed() sends the hand-timed anti-kick close ramp
  // through this, bypassing move_group's time parameterization entirely
  // (the scaling-factor path never reached the executed trajectory,
  // measured 2026-09-23, see Params::grasp_close_duration).
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    raw_gripper_traj_client_;

  rclcpp_action::Server<GraspObject>::SharedPtr action_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_action_server_;

  // Realism pass: /piper/planning_scene diff publisher for attaching the
  // carried box to the end-effector during the carry (see Params::
  // attach_carried_box).
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  std::string carried_box_name_;

  // Task 2b TF state -- sees the stage-published sensor_cam_N frames (the
  // wall stage's kotek_wall_sensor_graph; DDS-domain bug fixed 2026-09-18).
  // Lives on `self` (the node's own executor spins it), lookups are
  // non-blocking with TimePointZero (latest available transform).
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Carried-box tracking across executePlace(): the object frame from the
  // PlaceObject goal and its box-to-TCP distance at place start (see
  // heldBoxNearTcp()). Frame empty = check passive.
  std::string carried_box_frame_;
  double carried_box_d0_ = 0.0;
  bool box_check_warned_ = false;
};

}  // namespace kotek_manipulation

#endif  // KOTEK_MANIPULATION__PIPER_MANIPULATOR_HPP_
