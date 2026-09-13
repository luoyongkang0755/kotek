#!/usr/bin/env python3
"""Publishes the sensor object's pose in the base controller's global frame.

Isaac publishes a `world -> sensor` transform (see the kotek_sensor_graph
added in kotek_isaac_stage, plan section 11). The base controller and task
coordinator work in `odom` (ground-truth chassis odometry, also from Isaac).
This node ties the two trees together with a single static `odom -> world`
transform (the scout spawns at the world origin, so this is the identity by
default -- see plan section 6) and republishes the resulting pose as a
latched PoseStamped on /sensor/pose, which is what
task_coordinator's FIND_SENSOR state waits on.
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SensorPosePublisher(Node):

    def __init__(self):
        super().__init__('sensor_pose_publisher')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('sensor_frame', 'sensor')
        self.declare_parameter('output_frame', 'odom')
        # x, y, z, yaw of the world origin expressed in output_frame.
        # Identity by default: the scout spawns at the world origin.
        self.declare_parameter('world_to_odom', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', 10.0)

        self._world_frame = self.get_parameter('world_frame').value
        self._sensor_frame = self.get_parameter('sensor_frame').value
        self._output_frame = self.get_parameter('output_frame').value
        offset = self.get_parameter('world_to_odom').value
        rate = float(self.get_parameter('publish_rate').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transform(offset)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseStamped, '/sensor/pose', latched_qos)

        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'sensor_pose_publisher ready: {self._output_frame} -> {self._world_frame} '
            f'-> {self._sensor_frame}, offset={offset}')

    def _publish_static_transform(self, offset):
        x, y, z, yaw = offset
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._output_frame
        t.child_frame_id = self._world_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._static_broadcaster.sendTransform(t)

    def _on_timer(self):
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._output_frame, self._sensor_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup {self._output_frame} -> {self._sensor_frame} failed: {ex}',
                throttle_duration_sec=2.0)
            return

        pose = PoseStamped()
        pose.header.stamp = tf_msg.header.stamp
        pose.header.frame_id = self._output_frame
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation = tf_msg.transform.rotation
        self._pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
