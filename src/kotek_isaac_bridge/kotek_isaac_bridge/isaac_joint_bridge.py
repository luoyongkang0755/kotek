#!/usr/bin/env python3
"""Bridges MoveIt 2's FollowJointTrajectory controllers to Isaac Sim's raw
sensor_msgs/JointState interface for the Piper arm.

Isaac's piper ActionGraph (see plan section 1.4) exposes only:
  - /isaac_joint_states   (sensor_msgs/JointState, published by Isaac)
  - /isaac_joint_commands (sensor_msgs/JointState, subscribed by Isaac)

There is no ros2_control in the stage, but
`piper_camera_moveit_config/config/moveit_controllers.yaml` already expects
exactly two FollowJointTrajectory action servers named `arm_controller` and
`gripper_controller`. This node hosts those two servers and republishes
Isaac's joint states on the topic MoveIt's robot_state_publisher listens to,
so MoveIt works against the existing config unmodified.
"""
import time
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def _duration_to_seconds(duration_msg) -> float:
    return duration_msg.sec + duration_msg.nanosec * 1e-9


def parse_joint_limits(urdf_xml: str) -> dict:
    """Extract {joint_name: (lower, upper)} for every bounded joint in a URDF.

    Only 'revolute' and 'prismatic' joints carry meaningful position bounds
    -- 'continuous' joints wrap and are never bounds-checked by MoveIt, and
    'fixed'/other joint types have no <limit> at all. A joint missing either
    lower or upper is skipped rather than guessed at.
    """
    limits = {}
    if not urdf_xml:
        return limits
    root = ET.fromstring(urdf_xml)
    for joint in root.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = joint.find('limit')
        if limit is None or limit.get('lower') is None or limit.get('upper') is None:
            continue
        name = joint.get('name')
        limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits


class TrajectoryGroup:
    """Tracks one FollowJointTrajectory action server's joint set."""

    def __init__(self, name, joint_names):
        self.name = name
        self.joint_names = list(joint_names)


class IsaacJointBridge(Node):

    def __init__(self, **kwargs):
        super().__init__('isaac_joint_bridge', **kwargs)

        self.declare_parameter('isaac_state_topic', '/isaac_joint_states')
        self.declare_parameter('isaac_command_topic', '/isaac_joint_commands')
        self.declare_parameter(
            'arm_joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joints', ['joint7', 'joint8'])
        self.declare_parameter('command_rate', 100.0)
        self.declare_parameter('goal_time_tolerance', 2.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('stall_position_delta', 0.001)
        self.declare_parameter('stall_ticks_required', 10)
        # robot_description: same URDF move_group/piper_manipulator are given
        # (piper_moveit.launch.py) -- used only to clamp Isaac's raw joint
        # states to their declared bounds before they reach MoveIt. See
        # _isaac_state_cb for why this clamp has to live here.
        self.declare_parameter('robot_description', '')
        self.declare_parameter('bounds_clamp_tolerance', 0.01)

        self._isaac_state_topic = self.get_parameter('isaac_state_topic').value
        self._isaac_command_topic = self.get_parameter('isaac_command_topic').value
        self._command_rate = float(self.get_parameter('command_rate').value)
        self._goal_time_tolerance = float(self.get_parameter('goal_time_tolerance').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._stall_position_delta = float(self.get_parameter('stall_position_delta').value)
        self._stall_ticks_required = int(self.get_parameter('stall_ticks_required').value)
        self._bounds_clamp_tolerance = float(self.get_parameter('bounds_clamp_tolerance').value)
        robot_description = self.get_parameter('robot_description').value
        self._joint_limits = parse_joint_limits(robot_description)
        if not robot_description:
            self.get_logger().warn(
                'robot_description parameter is empty; joint-state bounds clamping is '
                'disabled (see MOVE_ARM_TO_PREPLACE/START_STATE_INVALID failure mode)')

        arm_group = TrajectoryGroup('arm', self.get_parameter('arm_joints').value)
        gripper_group = TrajectoryGroup('gripper', self.get_parameter('gripper_joints').value)

        self._joint_positions = {}

        # `_execute()` below blocks its calling thread for the whole
        # trajectory (interpolation loop + settle-wait loop, both using
        # time.sleep()). With the default single-threaded executor
        # (`rclpy.spin(node)`) and everything on the node's default
        # MutuallyExclusiveCallbackGroup, starting a goal permanently starves
        # every other callback on this node -- including `_isaac_state_cb`
        # (so `self._joint_positions` freezes at whatever it held the instant
        # the goal started) and, with use_sim_time, the internal `/clock`
        # subscription that `self.get_clock().now()` depends on (so
        # `elapsed` in `_execute()` never advances either). The net effect:
        # every goal hangs forever, even though Isaac Sim is genuinely
        # moving the joint underneath it -- confirmed directly with a live
        # run (a raw FollowJointTrajectory goal reached the real target in
        # `/isaac_joint_states`, while this node's own feedback reported
        # `actual` frozen at its pre-goal value indefinitely). This is what
        # left the live pick-and-delivery task stuck in MOVE_ARM_TO_PREGRASP
        # forever, since MoveGroupInterface::execute() (piper_manipulator)
        # blocks synchronously on this action completing. Fixed by giving
        # the state subscription and both action servers a
        # ReentrantCallbackGroup and spinning with a MultiThreadedExecutor
        # (see main()) -- the same fix piper_manipulator_node.cpp's C++
        # side already applies via its own detached execute() thread +
        # MultiThreadedExecutor.
        callback_group = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._state_sub = self.create_subscription(
            JointState, self._isaac_state_topic, self._isaac_state_cb, sensor_qos,
            callback_group=callback_group)
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._command_pub = self.create_publisher(
            JointState, self._isaac_command_topic, 10)

        self._arm_server = ActionServer(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
            execute_callback=lambda gh: self._execute(gh, arm_group),
            goal_callback=lambda goal: self._handle_goal(goal, arm_group),
            cancel_callback=self._handle_cancel,
            callback_group=callback_group)
        self._gripper_server = ActionServer(
            self, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory',
            execute_callback=lambda gh: self._execute(gh, gripper_group),
            goal_callback=lambda goal: self._handle_goal(goal, gripper_group),
            cancel_callback=self._handle_cancel,
            callback_group=callback_group)

        self.get_logger().info(
            f'isaac_joint_bridge ready: arm={arm_group.joint_names} '
            f'gripper={gripper_group.joint_names} isaac_state={self._isaac_state_topic} '
            f'isaac_command={self._isaac_command_topic} '
            f'bounds_clamped_joints={len(self._joint_limits)}')

    def _isaac_state_cb(self, msg: JointState):
        # Clamp to URDF bounds before anything downstream sees this state.
        # Real trigger: joint2's URDF lower limit is exactly 0 (unlike every
        # other joint on this arm, whose home value sits strictly inside its
        # bounds), and STOW_OBJECT's retreat-to-'zero' commands joint2 = 0.
        # PhysX's position drive settles a hair under that (observed -0.0001
        # rad), and move_group's CheckStartStateBounds request adapter
        # hard-rejects any out-of-bounds start state with START_STATE_INVALID
        # -- there is no ROS parameter to relax this in MoveIt Jazzy
        # (CheckStartStateBounds only exposes fix_start_state, which by its
        # own doc only fixes continuous/planar/floating joints; and
        # CurrentStateMonitor's own bounds-error margin defaults to machine
        # epsilon with no parameter to raise it). So this clamp is the only
        # correct fix point: small (<= bounds_clamp_tolerance) violations are
        # PhysX settling noise and are clamped silently; larger ones are a
        # real asset/modelling bug and are left visible (raw value published,
        # warned) rather than papered over.
        if len(msg.position) == len(msg.name):
            for i, name in enumerate(msg.name):
                limit = self._joint_limits.get(name)
                if limit is None:
                    continue
                lower, upper = limit
                position = msg.position[i]
                if position < lower:
                    violation = lower - position
                    bound = lower
                elif position > upper:
                    violation = position - upper
                    bound = upper
                else:
                    continue
                if violation <= self._bounds_clamp_tolerance:
                    msg.position[i] = bound
                else:
                    self.get_logger().warn(
                        f"joint '{name}' outside URDF bounds by {violation:.5f} "
                        f'(should be in [{lower}, {upper}]); publishing raw value',
                        throttle_duration_sec=5.0)

        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position
        self._state_pub.publish(msg)

    def _handle_goal(self, goal_request, group: TrajectoryGroup):
        names = set(goal_request.trajectory.joint_names)
        if not names.issubset(set(group.joint_names)):
            self.get_logger().error(
                f'{group.name}_controller: goal names {names} not a subset of '
                f'{group.joint_names}; rejecting')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle: ServerGoalHandle):
        return CancelResponse.ACCEPT

    @staticmethod
    def _interpolate(points, elapsed):
        """Linear interpolation of trajectory positions at `elapsed` seconds."""
        if not points:
            return []
        stamps = [_duration_to_seconds(p.time_from_start) for p in points]
        if elapsed <= stamps[0]:
            return list(points[0].positions)
        if elapsed >= stamps[-1]:
            return list(points[-1].positions)
        for i in range(len(points) - 1):
            t0, t1 = stamps[i], stamps[i + 1]
            if t0 <= elapsed <= t1:
                span = max(t1 - t0, 1e-6)
                alpha = (elapsed - t0) / span
                p0, p1 = points[i].positions, points[i + 1].positions
                return [a + alpha * (b - a) for a, b in zip(p0, p1)]
        return list(points[-1].positions)

    def _publish_command(self, joint_names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = list(positions)
        self._command_pub.publish(msg)

    def _execute(self, goal_handle: ServerGoalHandle, group: TrajectoryGroup):
        traj = goal_handle.request.trajectory
        points = traj.points
        result = FollowJointTrajectory.Result()

        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        joint_names = list(traj.joint_names)
        start_time = self.get_clock().now()
        goal_duration = _duration_to_seconds(points[-1].time_from_start)
        dt = 1.0 / self._command_rate

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'canceled; holding last commanded position'
                return result

            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            interp = self._interpolate(points, elapsed)
            self._publish_command(joint_names, interp)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = joint_names
            feedback.desired.positions = interp
            feedback.actual.positions = [
                self._joint_positions.get(n, 0.0) for n in joint_names]
            goal_handle.publish_feedback(feedback)

            if elapsed >= goal_duration:
                break
            time.sleep(dt)

        # Physical settle: give the PhysX position drive up to
        # goal_time_tolerance to catch up to the final target before judging
        # success/failure. We also watch for a *stalled* position (moved
        # less than stall_position_delta for stall_ticks_required
        # consecutive ticks) and treat that as success too: this is what
        # lets CLOSE_GRIPPER succeed when the fingers are stopped short of
        # their commanded target by contact with the object (plan section
        # 9) -- we have no joint effort feedback to detect contact more
        # directly, so a stalled position is the best available signal.
        final_target = list(points[-1].positions)
        settle_deadline = self.get_clock().now().nanoseconds * 1e-9 + self._goal_time_tolerance
        previous_actual = None
        stalled_ticks = 0
        while rclpy.ok():
            actual = [self._joint_positions.get(n, 0.0) for n in joint_names]
            errors = [abs(a - t) for a, t in zip(actual, final_target)]
            if all(e <= self._position_tolerance for e in errors):
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            if previous_actual is not None:
                movement = max(abs(a - p) for a, p in zip(actual, previous_actual))
                stalled_ticks = stalled_ticks + 1 if movement < self._stall_position_delta else 0
            previous_actual = actual

            if stalled_ticks >= self._stall_ticks_required:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = f'stalled before reaching target (errors={errors})'
                return result

            if self.get_clock().now().nanoseconds * 1e-9 >= settle_deadline:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = (
                    f'joints did not settle within {self._goal_time_tolerance}s '
                    f'(errors={errors})')
                return result
            time.sleep(dt)

        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
        result.error_string = 'node shutting down mid-execution'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = IsaacJointBridge()
    # MultiThreadedExecutor, not plain rclpy.spin() -- see the callback_group
    # comment in __init__ for why a single-threaded executor deadlocks every
    # goal against this node's own state subscription.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
