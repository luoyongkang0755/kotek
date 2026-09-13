// ROS 2 wrapper around kotek_base_control::SimpleBaseController.
//
// Hosts a nav2_msgs/action/NavigateToPose action server on the action name
// "navigate_to_pose" -- byte-identical to Nav2's own server -- so that
// swapping this node for Nav2 later is a launch-file change only (see plan
// section 17). Publishes geometry_msgs/Twist on cmd_vel_topic and reads the
// robot pose from TF (global_frame -> base_frame) or /odom, selected by the
// pose_source parameter. Robot velocity (for the STOPPED settle check) is
// always read from odom_topic.

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kotek_base_control/obstacle_avoidance.hpp"
#include "kotek_base_control/simple_base_controller.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace kotek_base_control
{

class SimpleBaseControllerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  SimpleBaseControllerNode()
  : rclcpp::Node("simple_base_controller"), controller_(loadParams())
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SimpleBaseControllerNode::odomCallback, this, std::placeholders::_1));

    if (obstacle_avoidance_enabled_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SimpleBaseControllerNode::scanCallback, this, std::placeholders::_1));
    }

    if (pose_source_ == "tf") {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      std::bind(
        &SimpleBaseControllerNode::handleGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&SimpleBaseControllerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&SimpleBaseControllerNode::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "simple_base_controller ready: pose_source=%s global_frame=%s base_frame=%s "
      "cmd_vel=%s",
      pose_source_.c_str(), global_frame_.c_str(), base_frame_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  BaseControllerParams loadParams()
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pose_source_ = declare_parameter<std::string>("pose_source", "tf");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    stop_settle_time_ = declare_parameter<double>("stop_settle_time", 0.5);
    linear_stopped_threshold_ = declare_parameter<double>("linear_stopped_threshold", 0.02);
    angular_stopped_threshold_ = declare_parameter<double>("angular_stopped_threshold", 0.05);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 60.0);
    min_progress_ = declare_parameter<double>("min_progress", 0.05);
    progress_window_ = declare_parameter<double>("progress_window", 10.0);

    obstacle_avoidance_enabled_ = declare_parameter<bool>("obstacle_avoidance_enabled", true);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
    avoidance_params_.safety_distance =
      declare_parameter<double>("obstacle_safety_distance", 0.6);
    avoidance_params_.stop_distance = declare_parameter<double>("obstacle_stop_distance", 0.25);
    avoidance_params_.steer_gain = declare_parameter<double>("obstacle_steer_gain", 1.0);
    avoidance_params_.max_steer_angular =
      declare_parameter<double>("obstacle_max_steer_angular", 0.6);
    forward_half_angle_ = declare_parameter<double>("obstacle_forward_half_angle", 0.26);
    side_min_angle_ = declare_parameter<double>("obstacle_side_min_angle", 0.35);
    side_max_angle_ = declare_parameter<double>("obstacle_side_max_angle", 1.57);

    BaseControllerParams p;
    p.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 0.5);
    p.min_linear_velocity = declare_parameter<double>("min_linear_velocity", 0.05);
    p.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 0.8);
    p.min_angular_velocity = declare_parameter<double>("min_angular_velocity", 0.1);
    p.max_linear_accel = declare_parameter<double>("max_linear_accel", 0.5);
    p.k_linear = declare_parameter<double>("k_linear", 0.8);
    p.k_angular = declare_parameter<double>("k_angular", 1.5);
    p.xy_tolerance = declare_parameter<double>("xy_tolerance", 0.08);
    p.yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.10);
    p.heading_tolerance = declare_parameter<double>("heading_tolerance", 0.15);
    // Reuse the same overall angular-speed ceiling the base controller
    // itself is bounded by, rather than a separate, easy-to-desync param.
    avoidance_params_.max_angular_velocity = p.max_angular_velocity;
    return p;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    have_odom_ = true;
    if (pose_source_ == "odom") {
      last_pose_.x = msg->pose.pose.position.x;
      last_pose_.y = msg->pose.pose.position.y;
      last_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
      last_pose_stamp_ = now();
      have_pose_ = true;
    }
  }

  // Returns true and fills `out` if a fresh-enough pose is available.
  bool getCurrentPose(Pose2D & out)
  {
    if (pose_source_ == "tf") {
      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        tf_msg = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "TF lookup %s -> %s failed: %s",
          global_frame_.c_str(), base_frame_.c_str(), ex.what());
        std::lock_guard<std::mutex> lock(odom_mutex_);
        out = last_pose_;
        return have_pose_ && (now() - last_pose_stamp_).seconds() <= pose_timeout_;
      }
      std::lock_guard<std::mutex> lock(odom_mutex_);
      last_pose_.x = tf_msg.transform.translation.x;
      last_pose_.y = tf_msg.transform.translation.y;
      last_pose_.yaw = tf2::getYaw(tf_msg.transform.rotation);
      last_pose_stamp_ = now();
      have_pose_ = true;
      out = last_pose_;
      return true;
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    out = last_pose_;
    return have_pose_ && (now() - last_pose_stamp_).seconds() <= pose_timeout_;
  }

  bool getCurrentSpeed(double & linear_speed, double & angular_speed)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return false;
    }
    linear_speed = std::hypot(
      latest_odom_.twist.twist.linear.x, latest_odom_.twist.twist.linear.y);
    angular_speed = std::fabs(latest_odom_.twist.twist.angular.z);
    return true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    LaserScanSummary summary;
    summary.min_range_forward = msg->range_max;
    summary.min_range_left = msg->range_max;
    summary.min_range_right = msg->range_max;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double abs_angle = std::fabs(angle);
      if (abs_angle <= forward_half_angle_) {
        summary.min_range_forward = std::min(summary.min_range_forward, static_cast<double>(r));
      } else if (angle >= side_min_angle_ && angle <= side_max_angle_) {
        // REP-103: positive angle = left of forward.
        summary.min_range_left = std::min(summary.min_range_left, static_cast<double>(r));
      } else if (angle <= -side_min_angle_ && angle >= -side_max_angle_) {
        summary.min_range_right = std::min(summary.min_range_right, static_cast<double>(r));
      }
    }

    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_summary_ = summary;
    last_scan_stamp_ = now();
    have_scan_ = true;
  }

  // Returns a summary with every field at "no obstacle within range" if
  // avoidance is disabled or the scan is missing/stale -- obstacle
  // avoidance degrades to a no-op rather than blocking navigation, since a
  // stale scan is uninformative, not itself unsafe (the existing
  // pose/progress timeouts already guard against a genuinely stuck robot).
  LaserScanSummary currentScanSummary()
  {
    if (!obstacle_avoidance_enabled_) {
      return LaserScanSummary{};
    }
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!have_scan_ || (now() - last_scan_stamp_).seconds() > scan_timeout_) {
      return LaserScanSummary{};
    }
    return latest_scan_summary_;
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::Twist zero;
    cmd_vel_pub_->publish(zero);
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&SimpleBaseControllerNode::execute, this, goal_handle)}.detach();
  }

  static Pose2D goalToPose2D(const geometry_msgs::msg::PoseStamped & pose)
  {
    Pose2D p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.yaw = tf2::getYaw(pose.pose.orientation);
    return p;
  }

  geometry_msgs::msg::PoseStamped pose2DToMsg(const Pose2D & pose) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    return msg;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const Pose2D goal_pose = goalToPose2D(goal->pose);
    controller_.setGoal(goal_pose);

    RCLCPP_INFO(
      get_logger(), "New goal: x=%.3f y=%.3f yaw=%.3f (frame=%s)", goal_pose.x, goal_pose.y,
      goal_pose.yaw, goal->pose.header.frame_id.c_str());

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    const auto start_time = now();
    std::deque<std::pair<rclcpp::Time, Pose2D>> progress_history;
    rclcpp::Time settle_start;
    bool settling = false;
    ControlPhase last_logged_phase = controller_.phase();

    rclcpp::Rate rate(control_rate_);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        publishZeroTwist();
        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Goal canceled; base stopped.");
        return;
      }

      if ((now() - start_time).seconds() > goal_timeout_) {
        publishZeroTwist();
        // nav2_msgs/action/NavigateToPose only defines NONE=0 as a named
        // error code; any non-zero value signals a generic failure here.
        result->error_code = 1;
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "Goal timed out after %.1fs; base stopped.", goal_timeout_);
        return;
      }

      Pose2D current;
      const bool pose_ok = getCurrentPose(current);
      if (!pose_ok) {
        publishZeroTwist();
        // nav2_msgs/action/NavigateToPose only defines NONE=0 as a named
        // error code; any non-zero value signals a generic failure here.
        result->error_code = 1;
        goal_handle->abort(result);
        RCLCPP_ERROR(
          get_logger(), "Lost pose for > %.2fs; aborting goal, base stopped.", pose_timeout_);
        return;
      }

      // The min_progress/progress_window watchdog measures XY distance
      // covered, which is only a meaningful "is the robot stuck" signal
      // during ControlPhase::DRIVE -- TURN_TO_HEADING and ALIGN both
      // command v=0 by design (see simple_base_controller.cpp), so a
      // robot correctly executing a slow-but-real in-place turn (real
      // angular response is much slower than commanded for these rigid
      // wheels -- see docs/pick_and_delivery_report.md section 4.5) would
      // otherwise get killed by this check even though it isn't stuck at
      // all, just legitimately not translating yet. Confirmed directly:
      // a live diagnostic run hit exactly this -- "Insufficient progress
      // (0.003m in 10.0s)" while still cleanly converging in
      // TURN_TO_HEADING the entire window (yaw climbing steadily,
      // 0.001->0.082rad, never having left the phase). Clearing the
      // history on leaving DRIVE means the 10s window always starts fresh
      // from the first DRIVE tick, rather than being poisoned by
      // preceding turn-in-place time.
      if (controller_.phase() == ControlPhase::DRIVE) {
        progress_history.emplace_back(now(), current);
        while (!progress_history.empty() &&
          (now() - progress_history.front().first).seconds() > progress_window_)
        {
          progress_history.pop_front();
        }
        if ((now() - progress_history.front().first).seconds() >= progress_window_) {
          const double moved = distance(progress_history.front().second, current);
          if (moved < min_progress_) {
            publishZeroTwist();
            // nav2_msgs/action/NavigateToPose only defines NONE=0 as a named
            // error code; any non-zero value signals a generic failure here.
            result->error_code = 1;
            goal_handle->abort(result);
            RCLCPP_ERROR(
              get_logger(), "Insufficient progress (%.3fm in %.1fs); aborting, base stopped.",
              moved, progress_window_);
            return;
          }
        }
      } else {
        progress_history.clear();
      }

      if (!controller_.isGoalReached()) {
        settling = false;
        const Twist2D desired = controller_.step(current);
        const Twist2D cmd = applyObstacleAvoidance(
          desired, currentScanSummary(), avoidance_params_);
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = cmd.linear;
        twist_msg.angular.z = cmd.angular;
        cmd_vel_pub_->publish(twist_msg);

        // Diagnostics -- added to investigate the "Goal timed out" failure
        // documented in docs/pick_and_delivery_report.md section 4.5.
        // Kept permanently: the node previously logged nothing between
        // "New goal" and the terminal success/failure line, a real
        // visibility gap for any future navigate_to_pose debugging.
        if (controller_.phase() != last_logged_phase) {
          RCLCPP_INFO(
            get_logger(), "[phase] %s -> %s (pose=%.3f,%.3f,%.3f rho=%.3f)",
            toString(last_logged_phase), toString(controller_.phase()), current.x, current.y,
            current.yaw, controller_.distanceRemaining(current));
          last_logged_phase = controller_.phase();
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[tick] phase=%s pose=(%.3f,%.3f,%.3f) rho=%.3f desired=(v=%.3f,w=%.3f) "
          "applied=(v=%.3f,w=%.3f)",
          toString(controller_.phase()), current.x, current.y, current.yaw,
          controller_.distanceRemaining(current), desired.linear, desired.angular, cmd.linear,
          cmd.angular);
      } else {
        // Reached the pose geometrically -- confirm the base has actually
        // come to rest before declaring success. This is the interlock the
        // task coordinator's STOP_BASE state relies on.
        publishZeroTwist();
        double lin_speed = 0.0, ang_speed = 0.0;
        const bool have_speed = getCurrentSpeed(lin_speed, ang_speed);
        const bool at_rest = have_speed && lin_speed < linear_stopped_threshold_ &&
          ang_speed < angular_stopped_threshold_;

        if (!settling || !at_rest) {
          settling = true;
          settle_start = now();
        }
        if (at_rest && (now() - settle_start).seconds() >= stop_settle_time_) {
          result->error_code = NavigateToPose::Result::NONE;
          goal_handle->succeed(result);
          RCLCPP_INFO(
            get_logger(), "Goal reached and base settled (lin=%.3f ang=%.3f).", lin_speed,
            ang_speed);
          return;
        }
      }

      feedback->current_pose = pose2DToMsg(current);
      feedback->distance_remaining = controller_.distanceRemaining(current);
      feedback->navigation_time =
        static_cast<builtin_interfaces::msg::Duration>(now() - start_time);
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    publishZeroTwist();
  }

  // Parameters
  std::string global_frame_;
  std::string base_frame_;
  std::string pose_source_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double control_rate_ = 20.0;
  double stop_settle_time_ = 0.5;
  double linear_stopped_threshold_ = 0.02;
  double angular_stopped_threshold_ = 0.05;
  double pose_timeout_ = 0.5;
  double goal_timeout_ = 60.0;
  double min_progress_ = 0.05;
  double progress_window_ = 10.0;

  bool obstacle_avoidance_enabled_ = true;
  std::string scan_topic_;
  double scan_timeout_ = 0.5;
  double forward_half_angle_ = 0.26;
  double side_min_angle_ = 0.35;
  double side_max_angle_ = 1.57;
  ObstacleAvoidanceParams avoidance_params_;

  SimpleBaseController controller_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_ = false;
  Pose2D last_pose_;
  rclcpp::Time last_pose_stamp_;
  bool have_pose_ = false;

  std::mutex scan_mutex_;
  LaserScanSummary latest_scan_summary_;
  rclcpp::Time last_scan_stamp_;
  bool have_scan_ = false;
};

}  // namespace kotek_base_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kotek_base_control::SimpleBaseControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
