#!/usr/bin/env python3
"""Sole owner of can0 / the real Piper leader arm. Two jobs, gated separately:

1. ALWAYS (once armed): read the real arm's joint + gripper feedback, map it
   onto the simulated Piper's joint space (leader_mapping.py), and publish it
   on /isaac_joint_commands -- the same topic the bypassed isaac_joint_bridge
   would otherwise own, so MoveIt/the task coordinator must not be running
   at the same time (kotek_teleop.launch.py never includes
   piper_moveit.launch.py; see the publisher-count guard below).

2. ONLY IF haptics_enabled (default False): drive the haptic contact-wall
   state machine (haptic_law.py) off /isaac/arm_contact, commanding
   JointMitCtrl torque on the real arm during a contact event and returning
   it to drag-teach otherwise. This is real hardware under torque control --
   see the plan's Phase 0 bench-test gate (scripts/piper/bench_mit_switch.py)
   and Phase 5 rollout. Do not flip haptics_enabled:=true outside a
   supervised bench session with a hand on the physical e-stop.

Exactly one process may touch can0 (C_PiperInterface_V2 owns its own reader
thread and a SocketCAN socket) -- that is why the haptic law, the CAN
commands, and the follow-mapping all live in this one node rather than being
split across nodes with a DDS hop between the watchdog and the thing it's
watching.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from kotek_teleop.haptic_law import HapticInputs, HapticLaw, HapticParams
from kotek_teleop.leader_mapping import (
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
    OnePoleLowPass,
    clamp_joint,
    gripper_width_to_finger_targets,
    leader_gripper_milli_mm_to_width_m,
    leader_joint_raw_to_sim_rad,
    rate_limit,
)

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:  # pragma: no cover -- only present in the container image
    C_PiperInterface_V2 = None


# GetArmStatus() enum values (piper_sdk piper_msgs.msg_v2.feedback.arm_feedback_status
# .ArmMsgFeedbackStatusEnum), transcribed here to avoid a hard import chain into a
# specific piper_sdk internal module path.
CTRL_MODE_TEACHING = 0x02
ARM_STATUS_NORMAL = 0x00
ARM_FAULT_CODES = {
    0x01: 'EMERGENCY_STOP', 0x05: 'JOINT_COMMUNICATION_ERR',
    0x06: 'JOINT_BRAKE_NOT_RELEASED', 0x07: 'COLLISION_OCCURRED',
    0x08: 'OVERSPEED_DURING_TEACHING_DRAG', 0x09: 'JOINT_STATUS_ERR',
    0x0A: 'OTHER_ERR',
}


class PiperLeaderNode(Node):

    def __init__(self):
        super().__init__('piper_leader_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('isaac_command_topic', '/isaac_joint_commands')
        self.declare_parameter('command_rate', 100.0)
        self.declare_parameter('lowpass_fc_hz', 10.0)
        self.declare_parameter('max_joint_rate_rad_s', 3.0)  # matches URDF velocity="3"
        self.declare_parameter('require_deadman_for_follow', True)
        self.declare_parameter('isaac_state_topic', '/isaac_joint_states')

        # Haptics -- OFF by default. See module docstring.
        self.declare_parameter('haptics_enabled', False)
        self.declare_parameter('contact_topic', '/isaac/arm_contact')
        self.declare_parameter('control_loop_hz', 100.0)
        self.declare_parameter('motion_ctrl2_rate_hz', 20.0)
        self.declare_parameter('t_max_nm', [2.0, 2.0, 2.0, 1.0, 1.0, 1.0])
        self.declare_parameter('kp_wall', 8.0)
        self.declare_parameter('kp_hard_max', 30.0)
        self.declare_parameter('kd_wall', 0.8)
        self.declare_parameter('torque_slew_nm_s', 20.0)
        self.declare_parameter('wall_max_offset_rad', 0.15)
        self.declare_parameter('force_to_torque_k', 0.35)
        self.declare_parameter('contact_enter_debounce_s', 0.05)
        self.declare_parameter('contact_exit_debounce_s', 0.15)
        self.declare_parameter('max_contact_duration_s', 5.0)
        self.declare_parameter('contact_stale_s', 0.25)
        self.declare_parameter('deadman_stale_s', 0.20)

        self._can_channel = self.get_parameter('can_channel').value
        self._isaac_command_topic = self.get_parameter('isaac_command_topic').value
        self._command_rate = float(self.get_parameter('command_rate').value)
        self._max_joint_rate = float(self.get_parameter('max_joint_rate_rad_s').value)
        self._require_deadman_for_follow = bool(
            self.get_parameter('require_deadman_for_follow').value)
        self._isaac_state_topic = self.get_parameter('isaac_state_topic').value
        self._haptics_enabled_param = bool(self.get_parameter('haptics_enabled').value)
        self._motion_ctrl2_rate = float(self.get_parameter('motion_ctrl2_rate_hz').value)

        fc = float(self.get_parameter('lowpass_fc_hz').value)
        self._arm_filters = {name: OnePoleLowPass(fc) for name in ARM_JOINT_NAMES}
        self._gripper_filter = OnePoleLowPass(fc)
        self._last_targets = {name: 0.0 for name in ARM_JOINT_NAMES}
        self._last_gripper_targets = {name: 0.0 for name in GRIPPER_JOINT_NAMES}
        # Tracks the sim's own actual joint state (from Isaac, not the
        # leader) so a fresh engage can anchor its ramp there instead of
        # snapping to the leader's raw position. See _on_isaac_state /
        # _tick_follow's engage-edge handling below -- this is the fix for
        # the live-verification finding that the sim arm jumped on the first
        # deadman press instead of easing in from wherever it actually was.
        self._sim_arm_state = {}
        self._was_deadman_held = False
        self._first_follow_tick = True

        if C_PiperInterface_V2 is None:
            raise RuntimeError(
                'piper_sdk is not importable -- this node must run inside the '
                'kotek_ws:jazzy container (installed via docker/Dockerfile), not '
                'on the bare host.')

        # judge_flag=False: C_PiperInterface_V2's default bitrate check reads
        # sysfs paths that do not exist inside the container (verified this
        # session -- ValueError: "CAN port can0 bitrate is (CAN_BITRATE_ERR...
        # FileNotFoundError...)"). The raw AF_CAN socket itself works fine
        # from the container (also verified); only that validation is broken
        # in this environment, so we skip it and rely on can_activate.sh
        # having already configured can0 at 1Mbit on the host.
        self._piper = C_PiperInterface_V2(self._can_channel, judge_flag=False)
        self._piper.ConnectPort()
        self.get_logger().info(f'connected to Piper leader arm on {self._can_channel}')

        self._cmd_pub = self.create_publisher(JointState, self._isaac_command_topic, 10)
        self._check_no_competing_publisher()
        sim_state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, self._isaac_state_topic, self._on_isaac_state, sim_state_qos)

        self._deadman_held = False
        self._deadman_last_msg_time = None
        self._estop_active = False
        self.create_subscription(Bool, '/teleop/deadman', self._on_deadman, 10)
        estop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/teleop/estop', self._on_estop, estop_qos)

        self._haptic_law = None
        self._contact_torque = tuple([0.0] * 6)
        self._sim_positions = tuple([0.0] * 6)
        self._contact_last_msg_time = None
        self._mit_active = False
        self._last_motion_ctrl2_time = 0.0
        if self._haptics_enabled_param:
            self._init_haptics()

        dt = 1.0 / self._command_rate
        self._timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'piper_leader_node ready: follow-publish requires deadman='
            f'{self._require_deadman_for_follow}, haptics_enabled='
            f'{self._haptics_enabled_param}')

    # -- setup helpers --------------------------------------------------

    def _check_no_competing_publisher(self) -> bool:
        # count_publishers() counts every publisher on this topic INCLUDING
        # our own (self._cmd_pub, created just above) -- confirmed
        # empirically this session: a fresh node with nothing else running
        # still reports count=1. So the threshold here is > 1, not > 0.
        deadline = time.monotonic() + 2.0
        count = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            count = self.count_publishers(self._isaac_command_topic)
            if count > 1:
                break
        other_count = max(0, count - 1)
        if other_count > 0:
            self.get_logger().error(
                f'{other_count} other publisher(s) already on {self._isaac_command_topic} -- '
                'this usually means isaac_joint_bridge (MoveIt path) is running '
                'alongside teleop. They will fight over this topic. Stop the other '
                'launch. Continuing to run, but expect erratic sim arm motion.')
        return other_count == 0

    def _init_haptics(self):
        t_max = tuple(float(x) for x in self.get_parameter('t_max_nm').value)
        params = HapticParams(
            t_max_nm=t_max,
            kp_wall=float(self.get_parameter('kp_wall').value),
            kp_hard_max=float(self.get_parameter('kp_hard_max').value),
            kd_wall=float(self.get_parameter('kd_wall').value),
            torque_slew_nm_s=float(self.get_parameter('torque_slew_nm_s').value),
            wall_max_offset_rad=float(self.get_parameter('wall_max_offset_rad').value),
            force_to_torque_k=float(self.get_parameter('force_to_torque_k').value),
            contact_enter_debounce_s=float(self.get_parameter('contact_enter_debounce_s').value),
            contact_exit_debounce_s=float(self.get_parameter('contact_exit_debounce_s').value),
            max_contact_duration_s=float(self.get_parameter('max_contact_duration_s').value),
            contact_stale_s=float(self.get_parameter('contact_stale_s').value),
            deadman_stale_s=float(self.get_parameter('deadman_stale_s').value),
        )
        try:
            self._haptic_law = HapticLaw(params)
        except ValueError as exc:
            self.get_logger().error(
                f'refusing to enable haptics: invalid parameters ({exc}). '
                'Running in follow-only mode instead.')
            self._haptics_enabled_param = False
            return

        if not self._haptics_startup_gate():
            self.get_logger().error(
                'refusing to enable haptics: startup gate failed (see above). '
                'Running in follow-only mode instead.')
            self._haptics_enabled_param = False
            self._haptic_law = None
            return

        contact_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, self.get_parameter('contact_topic').value,
            self._on_contact, contact_qos)
        self.get_logger().warn(
            'HAPTICS ARMED: this node may command MIT torque on the real leader '
            'arm during contact events. Keep clear and keep the e-stop reachable.')

    def _haptics_startup_gate(self) -> bool:
        status = self._piper.GetArmStatus()
        ctrl_mode = int(status.arm_status.ctrl_mode)
        arm_status = int(status.arm_status.arm_status)
        if ctrl_mode != CTRL_MODE_TEACHING:
            self.get_logger().error(
                f'arm is not in TEACHING_MODE (ctrl_mode=0x{ctrl_mode:02X}) at startup')
            return False
        if arm_status != ARM_STATUS_NORMAL:
            self.get_logger().error(
                f'arm reports a non-NORMAL status at startup (0x{arm_status:02X})')
            return False
        # MIT commands are silently ineffective on a disabled driver -- verified
        # this session that TEACHING_MODE's gravity-comp hold already implies
        # all 6 drivers report enabled (GetArmEnableStatus() == [True]*6 live),
        # so this node deliberately never calls EnablePiper() itself (that
        # would be an unexplained, unrequested state change to real hardware).
        # Refuse instead of silently proceeding into a no-op haptic session if
        # that assumption ever doesn't hold (e.g. an operator disabled a motor
        # via the teach pendant).
        enable_status = self._piper.GetArmEnableStatus()
        if not all(enable_status):
            self.get_logger().error(
                f'not all motor drivers report enabled at startup ({enable_status}) -- '
                'MIT commands would be silently ineffective. This node will not call '
                'EnablePiper() itself; check the arm/teach pendant.')
            return False
        return True

    # -- subscriptions ----------------------------------------------------

    def _on_isaac_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self._sim_arm_state[name] = position

    def _on_deadman(self, msg: Bool):
        self._deadman_held = bool(msg.data)
        self._deadman_last_msg_time = time.monotonic()

    def _on_estop(self, msg: Bool):
        was_active = self._estop_active
        self._estop_active = bool(msg.data)
        if self._estop_active and not was_active:
            self.get_logger().error('E-STOP received on /teleop/estop')

    def _on_contact(self, msg: JointState):
        # Isaac zeros effort per joint when that joint is not in gated
        # contact (see teleop_runner.py), so a nonzero reading here already
        # means "in contact" -- no separate boolean field needed. See
        # haptic_law.py's contact_force_min_n docstring for why the Newton
        # threshold check lives on the Isaac side, not here.
        by_name = dict(zip(msg.name, msg.effort)) if msg.effort else {}
        pos_by_name = dict(zip(msg.name, msg.position)) if msg.position else {}
        self._contact_torque = tuple(by_name.get(n, 0.0) for n in ARM_JOINT_NAMES)
        self._sim_positions = tuple(pos_by_name.get(n, 0.0) for n in ARM_JOINT_NAMES)
        self._contact_last_msg_time = time.monotonic()

    # -- main loop ----------------------------------------------------------

    def _tick(self):
        self._tick_follow()
        if self._haptics_enabled_param and self._haptic_law is not None:
            self._tick_haptics()

    def _reanchor_to_sim_state(self):
        """Called on every engage edge (deadman rising, or the very first
        follow tick regardless of gating mode). Without this, the low-pass
        filter's first sample passes straight through (see
        OnePoleLowPass.step) and the rate limiter starts counting from
        _last_targets' stale/zero value -- neither of which is where the sim
        arm actually is -- so the first published target could be far from
        the sim's current pose and the arm would visibly snap there instead
        of easing in. Live-verified this session: without this, engaging
        produced a jump; this anchors the ramp at the sim's own reported
        state instead, so the very first target is exactly where the sim
        already is and every joint eases toward the leader from there,
        bounded by max_joint_rate_rad_s same as any other tick.

        If /isaac_joint_states hasn't delivered anything yet (e.g. Isaac
        isn't running), falls back to the old behavior (anchor at 0.0) --
        follow-only mode would be pointless to block on this.
        """
        if not self._sim_arm_state:
            self.get_logger().warn(
                'engaging follow with no /isaac_joint_states data yet -- cannot '
                'anchor to the sim\'s actual pose, first target may jump')
            return
        for name in ARM_JOINT_NAMES:
            pos = self._sim_arm_state.get(name)
            if pos is None:
                continue
            self._arm_filters[name].reset(pos)
            self._last_targets[name] = pos
        j7 = self._sim_arm_state.get('joint7')
        j8 = self._sim_arm_state.get('joint8')
        if j7 is not None and j8 is not None:
            sim_width = j7 - j8  # inverse of gripper_width_to_finger_targets
            self._gripper_filter.reset(sim_width)
            self._last_gripper_targets['joint7'] = j7
            self._last_gripper_targets['joint8'] = j8
        self.get_logger().info('follow engaged: anchored ramp to current sim arm pose')

    def _tick_follow(self):
        if self._require_deadman_for_follow and not self._deadman_held:
            self._was_deadman_held = False
            return  # gate: don't publish until armed (see module docstring)

        just_engaged = (
            (self._deadman_held and not self._was_deadman_held) or self._first_follow_tick)
        self._was_deadman_held = self._deadman_held
        self._first_follow_tick = False
        if just_engaged:
            self._reanchor_to_sim_state()

        dt = 1.0 / self._command_rate
        joint_msg = self._piper.GetArmJointMsgs()
        gripper_msg = self._piper.GetArmGripperMsgs()

        names = []
        positions = []
        for name, raw in zip(
                ARM_JOINT_NAMES,
                (joint_msg.joint_state.joint_1, joint_msg.joint_state.joint_2,
                 joint_msg.joint_state.joint_3, joint_msg.joint_state.joint_4,
                 joint_msg.joint_state.joint_5, joint_msg.joint_state.joint_6)):
            raw_rad = leader_joint_raw_to_sim_rad(name, raw)
            filtered = self._arm_filters[name].step(raw_rad, dt)
            clamped = clamp_joint(name, filtered)
            if clamped.clamped:
                self.get_logger().warn(
                    f"leader joint '{name}' target clamped to sim limit "
                    f'({filtered:.3f} -> {clamped.value:.3f} rad) -- the real arm can '
                    'go further than the simulated one here',
                    throttle_duration_sec=5.0)
            target = rate_limit(
                self._last_targets[name], clamped.value, self._max_joint_rate * dt)
            self._last_targets[name] = target
            names.append(name)
            positions.append(target)

        width = leader_gripper_milli_mm_to_width_m(gripper_msg.gripper_state.grippers_angle)
        width_filtered = self._gripper_filter.step(width, dt)
        j7, j8 = gripper_width_to_finger_targets(width_filtered)
        for name, value in zip(GRIPPER_JOINT_NAMES, (j7, j8)):
            target = rate_limit(
                self._last_gripper_targets[name], value, self._max_joint_rate * dt)
            self._last_gripper_targets[name] = target
            names.append(name)
            positions.append(target)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._cmd_pub.publish(msg)

    def _tick_haptics(self):
        now = time.monotonic()
        contact_age = (
            now - self._contact_last_msg_time if self._contact_last_msg_time else math.inf)
        deadman_age = (
            now - self._deadman_last_msg_time if self._deadman_last_msg_time else math.inf)

        status = self._piper.GetArmStatus()
        arm_status = int(status.arm_status.arm_status)
        arm_fault = arm_status != ARM_STATUS_NORMAL

        joint_msg = self._piper.GetArmJointMsgs()
        leader_positions = tuple(
            leader_joint_raw_to_sim_rad(name, v) for name, v in zip(
                ARM_JOINT_NAMES,
                (joint_msg.joint_state.joint_1, joint_msg.joint_state.joint_2,
                 joint_msg.joint_state.joint_3, joint_msg.joint_state.joint_4,
                 joint_msg.joint_state.joint_5, joint_msg.joint_state.joint_6)))

        inputs = HapticInputs(
            now_s=now,
            contact_active=any(abs(t) > 1e-6 for t in self._contact_torque),
            contact_torque_nm=self._contact_torque,
            sim_joint_positions_rad=self._sim_positions,
            leader_joint_positions_rad=leader_positions,
            contact_data_age_s=contact_age,
            deadman_held=self._deadman_held,
            deadman_data_age_s=deadman_age,
            estop_active=self._estop_active,
            arm_fault=arm_fault,
        )
        out = self._haptic_law.step(inputs)

        if arm_fault:
            code = ARM_FAULT_CODES.get(arm_status, f'0x{arm_status:02X}')
            self.get_logger().error(f'arm fault during haptics: {code}', throttle_duration_sec=1.0)

        if out.enter_mit:
            self._piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
            self._mit_active = True
            self._last_motion_ctrl2_time = now
            self.get_logger().info(f'haptics: entering MIT (state={out.state.value})')

        if out.in_mit:
            # MIT is not sticky across a CAN gap on this firmware -- keep
            # re-asserting it at motion_ctrl2_rate_hz while engaged, not just
            # on the entry edge.
            if now - self._last_motion_ctrl2_time >= 1.0 / self._motion_ctrl2_rate:
                self._piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
                self._last_motion_ctrl2_time = now
            for i in range(6):
                self._piper.JointMitCtrl(
                    i + 1, out.pos_ref_rad[i], 0.0, out.kp, out.kd, out.t_ref_nm[i])

        if out.exit_to_teach:
            self._piper.MotionCtrl_1(0, 0, 0x01)  # re-enter drag-teach recording
            self._mit_active = False
            self.get_logger().info('haptics: released, back to drag-teach')

    def destroy_node(self):
        # Never leave the arm torque-driven on any exit path.
        try:
            if self._mit_active:
                self._piper.MotionCtrl_1(0, 0, 0x01)
                self.get_logger().warn('shutdown: forced arm back to drag-teach')
        except Exception as exc:  # noqa: BLE001 -- best-effort on the way out
            self.get_logger().error(f'failed to return arm to teach mode on shutdown: {exc}')
        try:
            self._piper.DisconnectPort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
