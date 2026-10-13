#!/usr/bin/env python3
"""Compact gripper-width recorder for the wall-mount E2E driver.

Subscribes to /piper/joint_states and writes one CSV line per sample at
~20 Hz: 'epoch joint7 joint8 width'. Tiny output volume (unlike
`ros2 topic echo`, which stalled the driver's recording after ~55s in
smoke13). Joint7/8 are the two gripper fingers; width = j7 + j8 is the
finger separation tracked by the grasp contact check.
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self):
        super().__init__('gripper_width_recorder')
        self.create_subscription(JointState, '/piper/joint_states', self.cb, 10)
        self._n = 0

    def cb(self, msg):
        self._n += 1
        if self._n % 6 != 0:  # ~20 Hz from ~120 Hz source
            return
        try:
            i7 = msg.name.index('joint7')
            i8 = msg.name.index('joint8')
        except ValueError:
            return
        j7, j8 = msg.position[i7], msg.position[i8]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        print('%.3f %.5f %.5f %.5f' % (t, j7, j8, j7 + j8), flush=True)


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
