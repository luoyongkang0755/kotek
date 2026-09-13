#!/usr/bin/env python3
"""Real-node obstacle-avoidance integration test: replays one real,
physics-captured lidar scan (see capture_real_scan.py) against the actual
compiled `simple_base_controller_node` C++ binary and asserts its
published /cmd_vel is correctly suppressed near a real obstacle.

Why replay instead of a live Isaac Sim <-> docker stream: a genuine
cross-process DDS interop limitation between Isaac Sim's bundled ROS 2
stack and the system-installed ROS 2 Jazzy stack in the kotek docker
container blocks live streaming. Confirmed directly with a minimal
publisher/subscriber reproduction across that exact boundary: DDS
discovery/matching succeeds (subscription_count reaches 1) but zero
messages are ever actually delivered, even after 100 publishes over 10
real seconds and after trying `ipc: host` on the container (no change).
This is a deep, pre-existing environment limitation, not a kotek code bug
-- see docs/pick_and_delivery_report.md for the full diagnosis. Replaying
a captured-real scan sidesteps it while still exercising real, physics-
derived sensor data (not synthetic/hand-constructed) through the real,
compiled avoidance code path -- not a reimplementation or a mock.

Run entirely inside the kotek docker container (needs the real compiled
node + a real rclpy that matches it -- Isaac Sim's bundled Python is not
involved here at all). This script starts simple_base_controller_node
itself (as a subprocess, with the real base_controller.yaml params) and
tears it down when done -- a single command:

    source /opt/ros/jazzy/setup.bash
    cd /workspace && colcon build --symlink-install --packages-select kotek_base_control
    source install/setup.bash
    python3 src/kotek/kotek_isaac_stage/kotek_isaac_stage/tier3_obstacle_test.py

The captured scan file (real_scan.json, produced by capture_real_scan.py
via env_isaaclab -- see the README) must already exist alongside this
script.
"""
import json
import os
import subprocess
import sys
import time

SCAN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'real_scan.json')
PARAMS_FILE = (
    '/workspace/src/kotek/kotek_bringup/config/base_controller.yaml'
    if os.path.exists('/workspace/src/kotek/kotek_bringup/config/base_controller.yaml')
    else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'kotek_bringup', 'config',
        'base_controller.yaml')
)

GOAL_X, GOAL_Y = 2.0, 0.15
OBSTACLE_SAFETY_DISTANCE = 0.6
OBSTACLE_STOP_DISTANCE = 0.25
MAX_LINEAR_VELOCITY = 0.5


class Checker:
    def __init__(self):
        self.failures = []

    def check(self, condition, message):
        if condition:
            print(f'  [PASS] {message}', flush=True)
        else:
            print(f'  [FAIL] {message}', flush=True)
            self.failures.append(message)
        return bool(condition)


def start_controller():
    if not os.path.exists(PARAMS_FILE):
        print(f'### params file not found: {PARAMS_FILE}', flush=True)
        sys.exit(2)
    proc = subprocess.Popen(
        ['ros2', 'run', 'kotek_base_control', 'simple_base_controller_node',
         '--ros-args', '--params-file', PARAMS_FILE],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(f'### started simple_base_controller_node (pid={proc.pid})', flush=True)
    return proc


def main():
    c = Checker()

    if not os.path.exists(SCAN_FILE):
        print(
            f'### {SCAN_FILE} not found -- run capture_real_scan.py via env_isaaclab first '
            '(see module docstring / README).', flush=True)
        sys.exit(2)
    with open(SCAN_FILE) as f:
        scan_data = json.load(f)
    print(
        f'### loaded captured real scan: {len(scan_data["ranges"])} points, '
        f'min={min(scan_data["ranges"]):.3f}', flush=True)

    controller_proc = start_controller()
    time.sleep(3.0)  # let the node fully come up before rclpy discovery starts

    import rclpy

    rclpy.init()
    node = rclpy.create_node('tier3_obstacle_test')
    try:
        _run_test(node, c, scan_data)
    finally:
        rclpy.shutdown()
        controller_proc.terminate()
        try:
            out, _ = controller_proc.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            controller_proc.kill()
            out = ''
        print('### simple_base_controller_node output (tail):', flush=True)
        print('\n'.join((out or '').splitlines()[-20:]), flush=True)

    print()
    if c.failures:
        print(f'{len(c.failures)} check(s) FAILED:', flush=True)
        for f in c.failures:
            print(f'  - {f}', flush=True)
        sys.exit(1)
    print('All obstacle-avoidance integration checks PASSED.', flush=True)
    sys.exit(0)


def _run_test(node, c, scan_data):
    import rclpy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster

    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    tf_broadcaster = TransformBroadcaster(node)

    latest_cmd = {'linear': None, 'angular': None}
    node.create_subscription(
        Twist, '/cmd_vel', lambda m: latest_cmd.update(linear=m.linear.x, angular=m.angular.z), 10)

    def publish_scan_and_tf():
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = scan_data['angle_min']
        msg.angle_max = scan_data['angle_max']
        msg.angle_increment = scan_data['angle_increment']
        msg.time_increment = scan_data['time_increment']
        msg.scan_time = scan_data['scan_time']
        msg.range_min = scan_data['range_min']
        msg.range_max = scan_data['range_max']
        msg.ranges = [float(r) for r in scan_data['ranges']]
        msg.intensities = [float(i) for i in scan_data['intensities']]
        scan_pub.publish(msg)

        # simple_base_controller_node's pose_source=tf needs a real
        # odom->base_link transform to compute anything -- the robot is
        # stationary at the origin for this replay (matching the pose the
        # scan was actually captured at).
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

    # Publish for real wall-clock time so DDS discovery completes and the
    # controller has fresh pose/scan data before we send a goal.
    warmup_deadline = time.time() + 3.0
    while time.time() < warmup_deadline:
        publish_scan_and_tf()
        rclpy.spin_once(node, timeout_sec=0.05)

    nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    server_ready = nav_client.wait_for_server(timeout_sec=10.0)
    c.check(server_ready, 'navigate_to_pose action server (simple_base_controller_node) found')
    if not server_ready:
        sys.exit(2)

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'odom'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = GOAL_X
    goal_msg.pose.pose.position.y = GOAL_Y
    goal_msg.pose.pose.orientation.w = 1.0
    send_future = nav_client.send_goal_async(goal_msg)
    deadline = time.time() + 10.0
    while not send_future.done() and time.time() < deadline:
        publish_scan_and_tf()
        rclpy.spin_once(node, timeout_sec=0.05)
    goal_handle = send_future.result()
    c.check(
        goal_handle is not None and goal_handle.accepted,
        'navigate_to_pose goal (2.0m past the obstacle) accepted')
    if goal_handle is None:
        sys.exit(2)
    print(f'### sent navigate_to_pose goal ({GOAL_X}, {GOAL_Y})', flush=True)

    # Keep feeding the SAME real captured scan (obstacle stays put, robot
    # is not really moving in this replay) while monitoring what the real
    # controller decides to publish. The goal is ~2m away
    # (max_linear_velocity=0.5 means >4s of unobstructed travel), so any
    # suppression seen is attributable to the obstacle, not "arrived".
    max_observed_linear = 0.0
    samples = 0
    suppressed_samples = 0
    deadline = time.time() + 4.0
    while time.time() < deadline:
        publish_scan_and_tf()
        rclpy.spin_once(node, timeout_sec=0.05)
        lin = latest_cmd['linear']
        if lin is not None:
            samples += 1
            max_observed_linear = max(max_observed_linear, lin)
            if lin < MAX_LINEAR_VELOCITY * 0.5:
                suppressed_samples += 1

    print(
        f'### after 4s: samples={samples} max_observed_cmd_vel_linear={max_observed_linear:.3f} '
        f'suppressed_samples={suppressed_samples}', flush=True)

    c.check(samples > 0, 'received at least one real published /cmd_vel sample from the node')
    c.check(
        samples == 0 or suppressed_samples / max(samples, 1) > 0.8,
        f'published /cmd_vel linear speed was suppressed to well below max '
        f'({MAX_LINEAR_VELOCITY}) for the large majority of samples, given the real captured '
        f'scan shows an obstacle at {min(scan_data["ranges"]):.3f}m '
        f'({suppressed_samples}/{samples})')

    cancel_future = goal_handle.cancel_goal_async()
    deadline = time.time() + 5.0
    while not cancel_future.done() and time.time() < deadline:
        publish_scan_and_tf()
        rclpy.spin_once(node, timeout_sec=0.05)


if __name__ == '__main__':
    main()
