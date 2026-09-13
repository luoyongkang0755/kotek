#!/usr/bin/env python3
"""Drives /cmd_vel from a 3Dconnexion SpaceMouse, via spacenavd (see
spnav_client.py for why: no root, no udev rule, reuses the already-running
host daemon).

Runs standalone -- this does NOT launch alongside base_control.launch.py
(simple_base_controller also publishes /cmd_vel; running both would fight
over the topic). kotek_teleop.launch.py enforces this by never including
base_control.launch.py, plus the startup publisher-count guard below.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from kotek_teleop.spnav_client import SpnavClient


def _deadzone_rescale(value: float, deadzone: float) -> float:
    """Apply a deadzone to a value already normalized to [-1, 1], rescaling
    so output is continuous at the deadzone edge (no jump on breakaway)."""
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _linear_raw(motion_z: float, max_raw: float) -> float:
    """Forward/back signal, from raw z alone.

    Superseded a y-based design after a structured 8-gesture recording
    session (scripts/spacemouse/record_gesture.py, see gesture_log.csv)
    showed raw y is NOT a usable directional signal on this device/grip: it
    sat strongly negative (-70 to -135) in literally every gesture tested,
    including back_only (-135) and pure rotation (-102/-109) -- it tracks
    "a hand is resting on the puck at all", not push direction. A forward
    mapping built on max(0, -y) therefore added constant unwanted
    forward-creep that partially cancelled genuine backward commands (roughly
    halving net speed during back_only), which is what presented as
    "gets very slow" once other gestures were combined in.

    raw z, by contrast, was clean and correctly signed across all 8
    recorded gestures: strongly negative for every forward-intent gesture
    (front_only -236, front_right -222, front_left -66), strongly positive
    for every backward-intent one (back_only +242, back_left +130,
    back_right +67), and near-zero for pure rotation (+-2) -- i.e. no
    linear creep while turning in place. Sign flip below matches this
    stage's established convention (positive linear.x == forward, verified
    against the autonomous demo and re-confirmed live this session): z
    negative (forward push) -> -z positive -> drives forward.
    """
    return -motion_z / max_raw


def _angular_raw(motion_rz: float, motion_ry: float, max_raw: float) -> float:
    """Turn signal, combining two gestures that both produce it.

    The same recording session found two physically distinct ways an
    operator indicates "turn", each dominant on a different raw axis, both
    sharing the SAME sign convention (positive = left, negative = right):
      - A diagonal push-and-tilt (drive forward/back while turning) drives
        rz strongly: front_left rz=+342, front_right rz=-252,
        back_left rz=+262, back_right rz=-350. ry stays near zero in all
        four (<=18 in magnitude) -- negligible cross-talk.
      - An in-place twist drives ry strongly instead: rotation_left
        ry=+350, rotation_right ry=-350. rz stays small in both (+4, -49)
        -- again negligible relative to the dominant axis.
    Since both axes carry valid, same-signed turn intent in DIFFERENT
    gesture contexts with only minor cross-talk, a plain sum robustly
    covers both gesture styles without needing to detect which one the
    operator is using.
    """
    return (motion_rz + motion_ry) / max_raw


class SpaceMouseTeleopNode(Node):

    def __init__(self):
        super().__init__('spacemouse_teleop_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # Raw spnav_event_motion range is roughly +-350 (spacenavd default
        # sensitivity); NOT re-derived here, confirmed empirically against a
        # live capture this session.
        self.declare_parameter('max_raw', 350.0)
        self.declare_parameter('deadzone', 0.08)
        # Matches base_controller.yaml's max_linear_velocity/max_angular_velocity
        # so a full-deflection SpaceMouse push can't out-drive what the
        # (non-teleop) base controller itself is limited to.
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 0.8)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('event_timeout_s', 0.2)
        # Linear axis has no invert flag -- see _linear_raw's docstring; its
        # sign is fixed by the stage's established forward convention, not
        # a single invertible raw axis. invert_angular flips the combined
        # rz+ry turn signal (_angular_raw) if ever needed for a different
        # device/grip -- not needed for the live-calibrated data this
        # session, which confirmed the sign as-is.
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('require_no_other_cmd_vel_publisher', True)
        # button 0 held = haptics/leader-follow armed (piper_leader_node's
        # engage gate and haptic_law's dead-man both key off this). button 1
        # press = latching e-stop; cleared via the /teleop/estop_reset
        # service. Buttons confirmed present: this device reports 2 buttons.
        self.declare_parameter('deadman_button', 0)
        self.declare_parameter('estop_button', 1)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._max_raw = float(self.get_parameter('max_raw').value)
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._scale_linear = float(self.get_parameter('scale_linear').value)
        self._scale_angular = float(self.get_parameter('scale_angular').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._event_timeout_s = float(self.get_parameter('event_timeout_s').value)
        self._invert_angular = -1.0 if self.get_parameter('invert_angular').value else 1.0
        self._deadman_button = int(self.get_parameter('deadman_button').value)
        self._estop_button = int(self.get_parameter('estop_button').value)

        self._pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._deadman_pub = self.create_publisher(Bool, '/teleop/deadman', 10)
        self._estop_pub = self.create_publisher(Bool, '/teleop/estop', estop_qos)
        self._estop_reset_srv = self.create_service(
            Trigger, '/teleop/estop_reset', self._on_estop_reset)
        self._deadman_held = False
        self._estop_latched = False

        if self.get_parameter('require_no_other_cmd_vel_publisher').value:
            self._check_no_competing_publisher()

        self._client = SpnavClient()
        self._client.open('kotek_teleop_spacemouse')
        self.get_logger().info(
            f'connected to spacenavd: device="{self._client.device_name()}" '
            f'buttons={self._client.device_buttons()} axes={self._client.device_axes()}')

        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_event_time = time.monotonic()

        dt = 1.0 / self._publish_rate
        self._timer = self.create_timer(dt, self._tick)

    def _check_no_competing_publisher(self):
        # count_publishers() counts every publisher on this topic INCLUDING
        # our own (self._pub, created just above) -- confirmed empirically
        # this session: a fresh node with nothing else running still reports
        # count=1. So the threshold here is > 1 (self + at least one other),
        # not > 0.
        deadline = time.monotonic() + 2.0
        count = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            count = self.count_publishers(self._cmd_vel_topic)
            if count > 1:
                break
        other_count = max(0, count - 1)
        if other_count > 0:
            raise RuntimeError(
                f'{other_count} other publisher(s) already on {self._cmd_vel_topic} -- '
                'this usually means base_control.launch.py (the autonomous demo) '
                'is running alongside teleop. They must not run together: the '
                'autonomous base controller and this node would fight over '
                '/cmd_vel. Stop the other launch first, or pass '
                '-p require_no_other_cmd_vel_publisher:=false to override '
                '(not recommended).')

    def _on_estop_reset(self, request, response):
        self._estop_latched = False
        self.get_logger().warn('e-stop RESET via /teleop/estop_reset')
        response.success = True
        response.message = 'estop cleared'
        return response

    def _tick(self):
        motion, buttons = self._client.drain_latest_motion()
        now = time.monotonic()

        for b in buttons:
            if b.bnum == self._deadman_button:
                self._deadman_held = b.press
            elif b.bnum == self._estop_button and b.press:
                self._estop_latched = True
                self.get_logger().error('E-STOP pressed (SpaceMouse button)')

        self._deadman_pub.publish(Bool(data=self._deadman_held))
        self._estop_pub.publish(Bool(data=self._estop_latched))

        if motion is not None:
            self._last_event_time = now
            # See _linear_raw / _angular_raw docstrings for the calibration
            # this mapping is based on.
            raw_lin = _linear_raw(motion.z, self._max_raw)
            raw_ang = _angular_raw(motion.rz, motion.ry, self._max_raw)
            lin = _deadzone_rescale(raw_lin, self._deadzone)
            ang = _deadzone_rescale(raw_ang, self._deadzone)
            self._last_linear = max(-1.0, min(1.0, lin)) * self._scale_linear
            self._last_angular = max(-1.0, min(1.0, ang)) * self._scale_angular * self._invert_angular

        stale = (now - self._last_event_time) > self._event_timeout_s

        msg = Twist()
        if stale or self._estop_latched:
            # Watchdog / e-stop: never publish a latched nonzero command.
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = self._last_linear
            msg.angular.z = self._last_angular
        self._pub.publish(msg)

    def destroy_node(self):
        # Publish one final zero Twist so the robot doesn't coast on shutdown.
        try:
            self._pub.publish(Twist())
        except Exception:
            pass
        self._client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
