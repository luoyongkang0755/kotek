// Task coordinator node: gathers inputs each tick (TF, subscribers, action
// client state), feeds them to the pure kotek_task_coordinator::TaskFsm,
// and performs the side effects (sending goals, publishing /task/state)
// associated with whatever transition happens. See plan section 10.

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kotek_manipulation/pregrasp_geometry.hpp"
#include "kotek_msgs/action/grasp_object.hpp"
#include "kotek_msgs/action/place_object.hpp"
#include "kotek_task_coordinator/task_fsm.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace kotek_task_coordinator
{

class TaskCoordinatorNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GraspObject = kotek_msgs::action::GraspObject;
  using PlaceObject = kotek_msgs::action::PlaceObject;

  TaskCoordinatorNode()
  : rclcpp::Node("task_coordinator")
  {
    loadParams();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS latched(1);
    latched.transient_local();

    state_pub_ = create_publisher<std_msgs::msg::String>("/task/state", latched);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    sensor_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      sensor_topic_, latched,
      std::bind(&TaskCoordinatorNode::onSensorPose, this, std::placeholders::_1));

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/task/start", std::bind(
        &TaskCoordinatorNode::handleStart, this, std::placeholders::_1,
        std::placeholders::_2));
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "/task/abort", std::bind(
        &TaskCoordinatorNode::handleAbort, this, std::placeholders::_1,
        std::placeholders::_2));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    grasp_client_ = rclcpp_action::create_client<GraspObject>(this, grasp_action_name_);
    place_client_ = rclcpp_action::create_client<PlaceObject>(this, place_action_name_);

    state_entry_time_ = now();
    if (auto_start_) {
      start_requested_ = true;
    }

    // Publish the initial state so late subscribers see IDLE immediately.
    std_msgs::msg::String initial;
    initial.data = toString(fsm_.state());
    state_pub_->publish(initial);

    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / fsm_rate_),
      std::bind(&TaskCoordinatorNode::tick, this));

    RCLCPP_INFO(
      get_logger(), "task_coordinator ready: fsm_rate=%.1fHz sensor_topic=%s auto_start=%s",
      fsm_rate_, sensor_topic_.c_str(), auto_start_ ? "true" : "false");
  }

private:
  void loadParams()
  {
    // See coordinator.yaml's own comment: 0.40 left only 8mm of clearance
    // between the chassis front edge and the sensor pedestal.
    pregrasp_standoff_ = declare_parameter<double>("pregrasp_standoff", 0.48);
    const std::string mode_str =
      declare_parameter<std::string>("approach_mode", "from_current");
    if (mode_str == "fixed") {
      approach_mode_ = kotek_manipulation::ApproachMode::FIXED;
    } else if (mode_str == "object_frame") {
      approach_mode_ = kotek_manipulation::ApproachMode::OBJECT_FRAME;
    } else {
      approach_mode_ = kotek_manipulation::ApproachMode::FROM_CURRENT;
    }
    approach_yaw_ = declare_parameter<double>("approach_yaw", 0.0);
    object_yaw_offset_ = declare_parameter<double>("object_yaw_offset", 0.0);
    min_reach_ = declare_parameter<double>("min_reach", 0.15);
    max_reach_ = declare_parameter<double>("max_reach", 0.55);
    z_min_ = declare_parameter<double>("z_min", -0.20);
    z_max_ = declare_parameter<double>("z_max", 0.50);
    sensor_topic_ = declare_parameter<std::string>("sensor_topic", "/sensor/pose");
    sensor_wait_timeout_ = declare_parameter<double>("sensor_wait_timeout", 10.0);
    fsm_rate_ = declare_parameter<double>("fsm_rate", 10.0);
    auto_start_ = declare_parameter<bool>("auto_start", false);

    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.10);
    stop_settle_time_ = declare_parameter<double>("stop_settle_time", 0.5);
    abort_settle_time_ = declare_parameter<double>("abort_settle_time", 0.5);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    grasp_action_name_ =
      declare_parameter<std::string>("grasp_action_name", "/piper/grasp_object");
    place_action_name_ =
      declare_parameter<std::string>("place_action_name", "/piper/place_object");

    const auto mount = declare_parameter<std::vector<double>>(
      "arm_mount_xyz", std::vector<double>{0.0, 0.0, 0.29101});
    arm_mount_xyz_.x = mount.size() > 0 ? mount[0] : 0.0;
    arm_mount_xyz_.y = mount.size() > 1 ? mount[1] : 0.0;
    arm_mount_xyz_.z = mount.size() > 2 ? mount[2] : 0.29101;

    // Delivery leg: a second, fixed waypoint (no sensing involved, unlike
    // the pickup target) -- "stand off from a target point and face it" is
    // exactly pregrasp_geometry's own base-standoff problem, so it's reused
    // directly with a fixed approach bearing (delivery_yaw).
    delivery_x_ = declare_parameter<double>("delivery_x", 0.3);
    delivery_y_ = declare_parameter<double>("delivery_y", -1.2);
    delivery_z_ = declare_parameter<double>("delivery_z", 0.05);
    delivery_yaw_ = declare_parameter<double>("delivery_yaw", 0.0);
    delivery_standoff_ = declare_parameter<double>("delivery_standoff", 0.35);
    retreat_height_ = declare_parameter<double>("retreat_height", 0.0);

    kotek_base_control::Pose2D delivery_target;
    delivery_target.x = delivery_x_;
    delivery_target.y = delivery_y_;
    delivery_target.yaw = delivery_yaw_;
    delivery_goal_ =
      kotek_manipulation::computeBaseGoal(delivery_target, delivery_yaw_, delivery_standoff_);
  }

  // ---- subscriptions / services -----------------------------------

  void onSensorPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sensor_pose_ = *msg;
    have_sensor_pose_ = true;
  }

  void handleStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    start_requested_ = true;
    response->success = true;
    response->message = "start requested";
  }

  void handleAbort(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    abort_requested_ = true;
    response->success = true;
    response->message = "abort requested";
  }

  // ---- helpers ------------------------------------------------------

  // `z_out`, when non-null, receives base_link's own height above
  // global_frame_'s Z=0 plane -- Pose2D (out) deliberately carries no Z
  // (2D navigation never needs it), but transformPointOdomToArmFrame()
  // does (see its doc comment, section 4.16): omitting this made every
  // real grasp attempt aim ~8cm too high. Reads it from the SAME TF
  // lookup already being done here, rather than a hardcoded constant.
  bool lookupRobotPose(kotek_base_control::Pose2D & out, double * z_out = nullptr)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "TF lookup %s -> %s failed: %s", global_frame_.c_str(),
        base_frame_.c_str(), ex.what());
      return false;
    }
    out.x = t.transform.translation.x;
    out.y = t.transform.translation.y;
    out.yaw = tf2::getYaw(t.transform.rotation);
    if (z_out != nullptr) {
      *z_out = t.transform.translation.z;
    }
    return true;
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::Twist zero;
    cmd_vel_pub_->publish(zero);
  }

  // ---- per-state work invoked from tick() ---------------------------

  void computePregrasp()
  {
    kotek_base_control::Pose2D robot_pose;
    double robot_z = 0.0;
    if (!lookupRobotPose(robot_pose, &robot_z)) {
      pregrasp_computed_ok_ = false;
      failure_reason_ = "could not look up robot pose during COMPUTE_BASE_PREGRASP_POSE";
      return;
    }

    kotek_base_control::Pose2D sensor2d;
    sensor2d.x = sensor_pose_.pose.position.x;
    sensor2d.y = sensor_pose_.pose.position.y;
    sensor2d.yaw = tf2::getYaw(sensor_pose_.pose.orientation);

    const double bearing = kotek_manipulation::computeApproachBearing(
      sensor2d, robot_pose, approach_mode_, approach_yaw_, object_yaw_offset_);
    base_goal_ = kotek_manipulation::computeBaseGoal(sensor2d, bearing, pregrasp_standoff_);
    pregrasp_computed_ok_ = true;

    const kotek_manipulation::Point3 sensor_point{
      sensor_pose_.pose.position.x, sensor_pose_.pose.position.y,
      sensor_pose_.pose.position.z};
    const auto sensor_in_arm_frame_at_goal =
      kotek_manipulation::transformPointOdomToArmFrame(
      sensor_point, base_goal_, robot_z, arm_mount_xyz_);

    kotek_manipulation::ReachabilityLimits limits;
    limits.min_reach = min_reach_;
    limits.max_reach = max_reach_;
    limits.z_min = z_min_;
    limits.z_max = z_max_;
    pregrasp_reachable_ = kotek_manipulation::isReachable(sensor_in_arm_frame_at_goal, limits);
    if (!pregrasp_reachable_) {
      failure_reason_ = "sensor pose not reachable from computed pre-grasp base pose";
    }

    RCLCPP_INFO(
      get_logger(), "Pre-grasp base goal: x=%.3f y=%.3f yaw=%.3f reachable=%s", base_goal_.x,
      base_goal_.y, base_goal_.yaw, pregrasp_reachable_ ? "true" : "false");
  }

  void sendNavGoal(const kotek_base_control::Pose2D & target)
  {
    nav_goal_sent_ = true;
    nav_goal_done_ = false;
    nav_goal_success_ = false;

    if (!nav_client_->wait_for_action_server(0s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      nav_goal_done_ = true;
      failure_reason_ = "navigate_to_pose action server unavailable";
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, target.yaw);
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "navigate_to_pose goal rejected");
          nav_goal_done_ = true;
          failure_reason_ = "navigate_to_pose goal rejected";
        } else {
          nav_goal_handle_ = handle;
        }
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
        nav_goal_done_ = true;
        nav_goal_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (!nav_goal_success_) {
          failure_reason_ = "navigate_to_pose did not succeed";
        }
      };
    nav_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Sent navigate_to_pose goal: x=%.3f y=%.3f yaw=%.3f", target.x, target.y,
      target.yaw);
  }

  void checkAlignment(
    TaskFsm::Inputs & in, const kotek_base_control::Pose2D & target, bool & realign_used_flag,
    const char * state_name)
  {
    kotek_base_control::Pose2D robot_pose;
    if (!lookupRobotPose(robot_pose)) {
      in.aligned = false;
      in.realign_available = false;
      failure_reason_ = std::string("could not look up robot pose during ") + state_name;
      return;
    }
    const double yaw_err = kotek_base_control::normalizeAngle(target.yaw - robot_pose.yaw);
    in.aligned = std::fabs(yaw_err) <= yaw_tolerance_;
    in.realign_available = !in.aligned && !realign_used_flag;
    if (in.realign_available) {
      realign_used_flag = true;
      failure_reason_ = "base misaligned after drive; re-issuing goal";
      RCLCPP_WARN(
        get_logger(), "%s: yaw_err=%.3f, re-issuing drive goal once", state_name, yaw_err);
    } else if (!in.aligned) {
      failure_reason_ = "base misaligned after re-issued drive goal";
    }
  }

  void sendGraspGoal()
  {
    grasp_goal_sent_ = true;
    grasp_goal_done_ = false;
    grasp_goal_success_ = false;
    arm_feedback_stage_.clear();

    kotek_base_control::Pose2D robot_pose;
    double robot_z = 0.0;
    if (!lookupRobotPose(robot_pose, &robot_z)) {
      grasp_goal_done_ = true;
      failure_reason_ = "could not look up robot pose before sending grasp goal";
      return;
    }

    const kotek_manipulation::Point3 sensor_point{
      sensor_pose_.pose.position.x, sensor_pose_.pose.position.y,
      sensor_pose_.pose.position.z};
    const auto object_in_arm_frame =
      kotek_manipulation::transformPointOdomToArmFrame(
      sensor_point, robot_pose, robot_z, arm_mount_xyz_);

    if (!grasp_client_->wait_for_action_server(0s)) {
      RCLCPP_ERROR(get_logger(), "%s action server not available", grasp_action_name_.c_str());
      grasp_goal_done_ = true;
      failure_reason_ = "grasp_object action server unavailable";
      return;
    }

    GraspObject::Goal goal;
    // object_pose is expressed in the Piper's own base frame -- computed
    // analytically here from the scout's current odom pose, so
    // piper_manipulator never needs a cross-tree TF lookup (plan
    // section 6).
    goal.object_pose.header.frame_id = "piper_base_link";
    goal.object_pose.header.stamp = now();
    goal.object_pose.pose.position.x = object_in_arm_frame.x;
    goal.object_pose.pose.position.y = object_in_arm_frame.y;
    goal.object_pose.pose.position.z = object_in_arm_frame.z;
    goal.object_pose.pose.orientation.w = 1.0;
    goal.lift_height = 0.0;

    auto options = rclcpp_action::Client<GraspObject>::SendGoalOptions();
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<GraspObject>::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "grasp_object goal rejected");
          grasp_goal_done_ = true;
          failure_reason_ = "grasp_object goal rejected";
        } else {
          grasp_goal_handle_ = handle;
        }
      };
    options.feedback_callback =
      [this](
      rclcpp_action::ClientGoalHandle<GraspObject>::SharedPtr,
      const std::shared_ptr<const GraspObject::Feedback> feedback) {
        arm_feedback_stage_ = feedback->stage;
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<GraspObject>::WrappedResult & result) {
        grasp_goal_done_ = true;
        grasp_goal_success_ =
          result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
        if (!grasp_goal_success_) {
          failure_reason_ = "grasp_object failed: " +
            (result.result ? result.result->message : std::string("no result"));
        }
      };
    grasp_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Sent grasp_object goal: object in arm frame = (%.3f, %.3f, %.3f)",
      object_in_arm_frame.x, object_in_arm_frame.y, object_in_arm_frame.z);
  }

  void sendPlaceGoal()
  {
    place_goal_sent_ = true;
    place_goal_done_ = false;
    place_goal_success_ = false;
    arm_feedback_stage_.clear();

    kotek_base_control::Pose2D robot_pose;
    double robot_z = 0.0;
    if (!lookupRobotPose(robot_pose, &robot_z)) {
      place_goal_done_ = true;
      failure_reason_ = "could not look up robot pose before sending place goal";
      return;
    }

    // Fixed delivery point (not a subscribed sensor pose, unlike the
    // pickup leg) -- same odom->arm-frame transform either way.
    const kotek_manipulation::Point3 delivery_point{delivery_x_, delivery_y_, delivery_z_};
    const auto place_in_arm_frame =
      kotek_manipulation::transformPointOdomToArmFrame(
      delivery_point, robot_pose, robot_z, arm_mount_xyz_);

    if (!place_client_->wait_for_action_server(0s)) {
      RCLCPP_ERROR(get_logger(), "%s action server not available", place_action_name_.c_str());
      place_goal_done_ = true;
      failure_reason_ = "place_object action server unavailable";
      return;
    }

    PlaceObject::Goal goal;
    goal.place_pose.header.frame_id = "piper_base_link";
    goal.place_pose.header.stamp = now();
    goal.place_pose.pose.position.x = place_in_arm_frame.x;
    goal.place_pose.pose.position.y = place_in_arm_frame.y;
    goal.place_pose.pose.position.z = place_in_arm_frame.z;
    goal.place_pose.pose.orientation.w = 1.0;
    goal.retreat_height = retreat_height_;

    auto options = rclcpp_action::Client<PlaceObject>::SendGoalOptions();
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<PlaceObject>::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "place_object goal rejected");
          place_goal_done_ = true;
          failure_reason_ = "place_object goal rejected";
        } else {
          place_goal_handle_ = handle;
        }
      };
    options.feedback_callback =
      [this](
      rclcpp_action::ClientGoalHandle<PlaceObject>::SharedPtr,
      const std::shared_ptr<const PlaceObject::Feedback> feedback) {
        arm_feedback_stage_ = feedback->stage;
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<PlaceObject>::WrappedResult & result) {
        place_goal_done_ = true;
        place_goal_success_ =
          result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
        if (!place_goal_success_) {
          failure_reason_ = "place_object failed: " +
            (result.result ? result.result->message : std::string("no result"));
        }
      };
    place_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Sent place_object goal: place point in arm frame = (%.3f, %.3f, %.3f)",
      place_in_arm_frame.x, place_in_arm_frame.y, place_in_arm_frame.z);
  }

  void issueAbortActions()
  {
    if (nav_goal_handle_ && !nav_goal_done_) {
      nav_client_->async_cancel_goal(nav_goal_handle_);
    }
    if (grasp_goal_handle_ && !grasp_goal_done_) {
      grasp_client_->async_cancel_goal(grasp_goal_handle_);
    }
    if (place_goal_handle_ && !place_goal_done_) {
      place_client_->async_cancel_goal(place_goal_handle_);
    }
    publishZeroTwist();
    RCLCPP_WARN(get_logger(), "ABORTING: %s", failure_reason_.c_str());
  }

  // ---- main loop ------------------------------------------------------

  void tick()
  {
    const TaskState prev_state = fsm_.state();
    TaskFsm::Inputs in;
    in.start_requested = start_requested_;
    in.abort_requested = abort_requested_;

    switch (prev_state) {
      case TaskState::FIND_SENSOR:
        in.have_sensor_pose = have_sensor_pose_;
        if (!have_sensor_pose_ &&
          (now() - state_entry_time_).seconds() > sensor_wait_timeout_)
        {
          in.sensor_wait_timed_out = true;
          failure_reason_ = "no /sensor/pose received within sensor_wait_timeout";
        }
        break;

      case TaskState::COMPUTE_BASE_PREGRASP_POSE:
        computePregrasp();
        in.pregrasp_computed_ok = pregrasp_computed_ok_;
        in.pregrasp_reachable = pregrasp_reachable_;
        break;

      case TaskState::DRIVE_TO_PREGRASP:
        if (!nav_goal_sent_) {
          sendNavGoal(base_goal_);
        }
        in.nav_goal_done = nav_goal_done_;
        in.nav_goal_success = nav_goal_success_;
        break;

      case TaskState::ALIGN_BASE:
        checkAlignment(in, base_goal_, realign_used_, "ALIGN_BASE");
        break;

      case TaskState::STOP_BASE:
        if (!stop_base_zero_sent_) {
          publishZeroTwist();
          stop_base_zero_sent_ = true;
        }
        in.base_settled = (now() - state_entry_time_).seconds() >= stop_settle_time_;
        break;

      case TaskState::MOVE_ARM_TO_PREGRASP:
      case TaskState::MOVE_ARM_TO_GRASP:
      case TaskState::CLOSE_GRIPPER:
      case TaskState::LIFT_OBJECT:
      case TaskState::STOW_OBJECT:
        if (!grasp_goal_sent_) {
          sendGraspGoal();
        }
        in.grasp_goal_done = grasp_goal_done_;
        in.grasp_goal_success = grasp_goal_success_;
        in.arm_feedback_stage = arm_feedback_stage_;
        if (grasp_goal_done_ && !grasp_goal_success_) {
          in.grasp_retry_available = !grasp_retry_used_;
          if (in.grasp_retry_available) {
            grasp_retry_used_ = true;
            RCLCPP_WARN(
              get_logger(), "grasp_object failed (%s); retrying once",
              failure_reason_.c_str());
            // Reset directly here rather than relying on onTransition()'s
            // MOVE_ARM_TO_PREGRASP case: a retry triggered from a failure
            // that happened WHILE ALREADY IN MOVE_ARM_TO_PREGRASP (e.g. the
            // very first move_action call failing to plan) has TaskFsm
            // setting state_ back to the SAME state it's already in, so
            // step()'s own prev/next comparison reports no change and
            // onTransition() never runs -- confirmed directly in a live
            // run: "retrying once" logged, but grasp_goal_sent_ stayed
            // true from the failed attempt, no new goal was ever sent, and
            // the FSM aborted one tick later on the same stale done/failed
            // result. Resetting here covers both that case and the normal
            // CLOSE_GRIPPER/LIFT_OBJECT/STOW_OBJECT-originated retry (where
            // onTransition() would ALSO reset these -- harmless overlap).
            grasp_goal_sent_ = false;
            grasp_goal_done_ = false;
            grasp_goal_success_ = false;
            arm_feedback_stage_.clear();
            grasp_goal_handle_.reset();
          }
        }
        break;

      case TaskState::DRIVE_TO_DELIVERY:
        if (!nav_goal_sent_) {
          sendNavGoal(delivery_goal_);
        }
        in.nav_goal_done = nav_goal_done_;
        in.nav_goal_success = nav_goal_success_;
        break;

      case TaskState::ALIGN_AT_DELIVERY:
        checkAlignment(in, delivery_goal_, delivery_realign_used_, "ALIGN_AT_DELIVERY");
        break;

      case TaskState::STOP_BASE_AT_DELIVERY:
        if (!stop_base_zero_sent_) {
          publishZeroTwist();
          stop_base_zero_sent_ = true;
        }
        in.base_settled = (now() - state_entry_time_).seconds() >= stop_settle_time_;
        break;

      case TaskState::PLACE_ARM:
      case TaskState::OPEN_GRIPPER:
      case TaskState::RETREAT_ARM:
        if (!place_goal_sent_) {
          sendPlaceGoal();
        }
        in.place_goal_done = place_goal_done_;
        in.place_goal_success = place_goal_success_;
        in.arm_feedback_stage = arm_feedback_stage_;
        break;

      case TaskState::ABORTING:
        if (!abort_actions_issued_) {
          issueAbortActions();
          abort_actions_issued_ = true;
        }
        in.aborting_settled = (now() - state_entry_time_).seconds() >= abort_settle_time_;
        break;

      default:
        break;
    }

    const bool changed = fsm_.step(in);

    if (prev_state == TaskState::IDLE && fsm_.state() == TaskState::FIND_SENSOR) {
      start_requested_ = false;
    }
    if (in.abort_requested && fsm_.state() == TaskState::ABORTING &&
      prev_state != TaskState::ABORTING)
    {
      abort_requested_ = false;
    }

    if (changed) {
      onTransition(prev_state, fsm_.state());
    }
  }

  void onTransition(TaskState prev, TaskState next)
  {
    state_entry_time_ = now();

    switch (next) {
      case TaskState::IDLE:
        realign_used_ = false;
        delivery_realign_used_ = false;
        grasp_retry_used_ = false;
        have_sensor_pose_ = false;
        break;
      case TaskState::COMPUTE_BASE_PREGRASP_POSE:
        pregrasp_computed_ok_ = false;
        pregrasp_reachable_ = false;
        break;
      case TaskState::DRIVE_TO_PREGRASP:
      case TaskState::DRIVE_TO_DELIVERY:
        nav_goal_sent_ = false;
        nav_goal_done_ = false;
        nav_goal_success_ = false;
        nav_goal_handle_.reset();
        break;
      case TaskState::STOP_BASE:
      case TaskState::STOP_BASE_AT_DELIVERY:
        stop_base_zero_sent_ = false;
        break;
      case TaskState::MOVE_ARM_TO_PREGRASP:
        // STOP_BASE is the normal, first-attempt entry into the arm
        // states; isArmState(prev) covers the retry path, where TaskFsm
        // sends us back here from CLOSE_GRIPPER/LIFT_OBJECT/etc after a
        // confirmed grasp miss (grasp_retry_available, see step()'s
        // arm-states case above) -- without this branch the retry would
        // transition states but grasp_goal_sent_ would stay true from the
        // failed attempt, so step() would never call sendGraspGoal()
        // again and the retry would silently do nothing.
        if (prev == TaskState::STOP_BASE || isArmState(prev)) {
          grasp_goal_sent_ = false;
          grasp_goal_done_ = false;
          grasp_goal_success_ = false;
          arm_feedback_stage_.clear();
          grasp_goal_handle_.reset();
        }
        break;
      case TaskState::PLACE_ARM:
        if (prev == TaskState::STOP_BASE_AT_DELIVERY) {
          place_goal_sent_ = false;
          place_goal_done_ = false;
          place_goal_success_ = false;
          arm_feedback_stage_.clear();
          place_goal_handle_.reset();
        }
        break;
      case TaskState::ABORTING:
        abort_actions_issued_ = false;
        break;
      case TaskState::FAILED:
        RCLCPP_ERROR(get_logger(), "Task failed: %s", failure_reason_.c_str());
        break;
      case TaskState::DONE:
        RCLCPP_INFO(get_logger(), "Task complete: sensor grasped, delivered, and placed.");
        break;
      default:
        break;
    }

    RCLCPP_INFO(get_logger(), "[FSM] %s -> %s", toString(prev), toString(next));

    std_msgs::msg::String msg;
    msg.data = toString(next);
    state_pub_->publish(msg);
  }

  // ---- parameters ----
  double pregrasp_standoff_ = 0.48;
  kotek_manipulation::ApproachMode approach_mode_ = kotek_manipulation::ApproachMode::FROM_CURRENT;
  double approach_yaw_ = 0.0;
  double object_yaw_offset_ = 0.0;
  double min_reach_ = 0.15;
  double max_reach_ = 0.55;
  double z_min_ = -0.20;
  double z_max_ = 0.50;
  std::string sensor_topic_;
  double sensor_wait_timeout_ = 10.0;
  double fsm_rate_ = 10.0;
  bool auto_start_ = false;
  std::string global_frame_;
  std::string base_frame_;
  double yaw_tolerance_ = 0.10;
  double stop_settle_time_ = 0.5;
  double abort_settle_time_ = 0.5;
  std::string cmd_vel_topic_;
  std::string grasp_action_name_;
  std::string place_action_name_;
  kotek_manipulation::Point3 arm_mount_xyz_;

  double delivery_x_ = 0.3;
  double delivery_y_ = -1.2;
  double delivery_z_ = 0.05;
  double delivery_yaw_ = 0.0;
  double delivery_standoff_ = 0.35;
  double retreat_height_ = 0.0;

  // ---- FSM + bookkeeping ----
  TaskFsm fsm_;
  rclcpp::Time state_entry_time_;
  std::string failure_reason_;

  bool start_requested_ = false;
  bool abort_requested_ = false;

  geometry_msgs::msg::PoseStamped sensor_pose_;
  bool have_sensor_pose_ = false;

  kotek_base_control::Pose2D base_goal_;
  bool pregrasp_computed_ok_ = false;
  bool pregrasp_reachable_ = false;

  bool nav_goal_sent_ = false;
  bool nav_goal_done_ = false;
  bool nav_goal_success_ = false;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_handle_;

  bool realign_used_ = false;
  bool stop_base_zero_sent_ = false;

  bool grasp_goal_sent_ = false;
  bool grasp_goal_done_ = false;
  bool grasp_goal_success_ = false;
  // Mirrors realign_used_ above: a confirmed grasp miss (piper_manipulator
  // reads back the gripper's actual closed width -- see
  // moveGripperToWidth()'s contact_detected_out) gets exactly one retry
  // before ABORTING, see TaskFsm's MOVE_ARM_TO_PREGRASP/.../LIFT_OBJECT
  // case. Reset to false only on IDLE entry (onTransition()), same as
  // realign_used_, so it covers the whole grasp attempt for one task run.
  bool grasp_retry_used_ = false;
  std::string arm_feedback_stage_;
  rclcpp_action::ClientGoalHandle<GraspObject>::SharedPtr grasp_goal_handle_;

  kotek_base_control::Pose2D delivery_goal_;
  bool delivery_realign_used_ = false;

  bool place_goal_sent_ = false;
  bool place_goal_done_ = false;
  bool place_goal_success_ = false;
  rclcpp_action::ClientGoalHandle<PlaceObject>::SharedPtr place_goal_handle_;

  bool abort_actions_issued_ = false;

  // ---- ROS interfaces ----
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<GraspObject>::SharedPtr grasp_client_;
  rclcpp_action::Client<PlaceObject>::SharedPtr place_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace kotek_task_coordinator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kotek_task_coordinator::TaskCoordinatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
