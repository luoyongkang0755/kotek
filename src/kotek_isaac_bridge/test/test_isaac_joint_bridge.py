"""Tier 2 test: isaac_joint_bridge against a fake Isaac (plan section 13).

No Isaac Sim is involved. FakeIsaac stands in for the sim: it echoes
whatever it receives on /isaac_joint_commands back out on
/isaac_joint_states, as an ideal (instantaneous) physics stand-in. This
verifies the FollowJointTrajectory <-> JointState round trip and that
commands are interpolated and reach the goal, without asserting anything
about real PhysX dynamics.
"""
import threading
import time
import unittest

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from kotek_isaac_bridge.isaac_joint_bridge import IsaacJointBridge, parse_joint_limits


class FakeIsaac(Node):

    def __init__(self):
        super().__init__('fake_isaac')
        self.commands = []
        self._sub = self.create_subscription(
            JointState, '/isaac_joint_commands', self._on_command, 10)
        self._pub = self.create_publisher(JointState, '/isaac_joint_states', 10)

    def _on_command(self, msg: JointState):
        self.commands.append((self.get_clock().now().nanoseconds, list(msg.name),
                               list(msg.position)))
        echo = JointState()
        echo.name = list(msg.name)
        echo.position = list(msg.position)
        self._pub.publish(echo)


class TestIsaacJointBridge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.bridge = IsaacJointBridge()
        cls.fake = FakeIsaac()
        cls.client_node = Node('test_client')

        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.bridge)
        cls.executor.add_node(cls.fake)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.bridge.destroy_node()
        cls.fake.destroy_node()
        cls.client_node.destroy_node()
        rclpy.shutdown()

    def test_arm_trajectory_interpolates_and_reaches_goal(self):
        client = ActionClient(
            self.client_node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
        point.time_from_start = DurationMsg(sec=1)
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.client_node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.client_node, result_future, timeout_sec=10.0)
        result = result_future.result().result
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        self.assertGreater(len(self.fake.commands), 1, 'expected multiple interpolated ticks')

        # Commanded time is monotonically non-decreasing.
        stamps = [c[0] for c in self.fake.commands]
        self.assertEqual(stamps, sorted(stamps))

        last_names, last_positions = self.fake.commands[-1][1], self.fake.commands[-1][2]
        for name, target in zip(goal.trajectory.joint_names, point.positions):
            idx = last_names.index(name)
            self.assertAlmostEqual(last_positions[idx], target, delta=0.02)

    def test_gripper_and_arm_goals_do_not_cross_command_joints(self):
        arm_client = ActionClient(
            self.client_node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        gripper_client = ActionClient(
            self.client_node, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory')
        self.assertTrue(arm_client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(gripper_client.wait_for_server(timeout_sec=5.0))

        self.fake.commands.clear()

        gripper_goal = FollowJointTrajectory.Goal()
        gripper_goal.trajectory.joint_names = ['joint7', 'joint8']
        pt = JointTrajectoryPoint()
        pt.positions = [0.04, -0.04]
        pt.time_from_start = DurationMsg(sec=0, nanosec=200_000_000)
        gripper_goal.trajectory.points = [pt]

        send_future = gripper_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(self.client_node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.client_node, result_future, timeout_sec=10.0)
        self.assertEqual(
            result_future.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        # Every command published while only the gripper goal was active
        # must name gripper joints only -- the arm group must never be
        # touched by a gripper-only goal.
        for _, names, _ in self.fake.commands:
            self.assertTrue(set(names).issubset({'joint7', 'joint8'}))

    def test_goal_with_joints_outside_group_is_rejected(self):
        client = ActionClient(
            self.client_node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint7']  # gripper joint sent to arm server
        pt = JointTrajectoryPoint()
        pt.positions = [0.0]
        pt.time_from_start = DurationMsg(sec=1)
        goal.trajectory.points = [pt]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.client_node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertFalse(goal_handle.accepted)


class TestParseJointLimits(unittest.TestCase):
    """No ROS involved -- pure URDF-string parsing."""

    def test_extracts_revolute_and_prismatic_limits(self):
        urdf = '''<robot name="test">
          <joint name="joint1" type="revolute">
            <limit lower="0" upper="3.14" effort="10" velocity="1"/>
          </joint>
          <joint name="joint2" type="prismatic">
            <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
          </joint>
          <joint name="joint3" type="continuous">
            <limit effort="10" velocity="1"/>
          </joint>
          <joint name="joint4" type="fixed"/>
          <joint name="joint5" type="revolute"/>
        </robot>'''
        # joint3 (continuous) has no position bounds to enforce, joint4 is
        # fixed, and joint5 (revolute but no <limit> element at all) is
        # skipped rather than guessed at -- only joint1/joint2 come out.
        self.assertEqual(
            parse_joint_limits(urdf), {'joint1': (0.0, 3.14), 'joint2': (-0.5, 0.5)})

    def test_empty_urdf_returns_empty_dict(self):
        self.assertEqual(parse_joint_limits(''), {})


_JOINT2_TEST_URDF = '''<robot name="test">
  <joint name="joint2" type="revolute">
    <limit lower="0" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>'''


class TestJointBoundsClamp(unittest.TestCase):
    """Reproduces the STOW_OBJECT/PLACE_ARM START_STATE_INVALID failure:
    joint2's URDF lower bound is exactly 0, and PhysX settles a hair under
    it (observed -0.0001) after the SRDF 'zero' retreat pose."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.bridge = IsaacJointBridge(parameter_overrides=[
            Parameter('robot_description', Parameter.Type.STRING, _JOINT2_TEST_URDF),
            Parameter('bounds_clamp_tolerance', Parameter.Type.DOUBLE, 0.01),
        ])

    @classmethod
    def tearDownClass(cls):
        cls.bridge.destroy_node()
        rclpy.shutdown()

    def test_small_violation_is_clamped_to_bound(self):
        msg = JointState()
        msg.name = ['joint2']
        msg.position = [-0.0001]
        self.bridge._isaac_state_cb(msg)
        self.assertEqual(msg.position[0], 0.0)
        self.assertEqual(self.bridge._joint_positions['joint2'], 0.0)

    def test_gross_violation_is_published_unmodified(self):
        # Well past bounds_clamp_tolerance -- a real modelling/asset bug,
        # not settling noise, so it must stay visible rather than be
        # silently clamped away.
        msg = JointState()
        msg.name = ['joint2']
        msg.position = [-0.5]
        self.bridge._isaac_state_cb(msg)
        self.assertEqual(msg.position[0], -0.5)


if __name__ == '__main__':
    unittest.main()
