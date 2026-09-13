"""Pure event-driven "contact wall" state machine for rendering simulated
contact force onto the real Piper leader arm. No ROS, no CAN, no piper_sdk
import -- this is the unit-test surface; piper_leader_node.py is a thin
ROS/CAN shell that calls into it once per control tick and translates its
output into JointMitCtrl / MotionCtrl_1 / MotionCtrl_2 calls.

Design rationale (see the approved plan's Phase 3.5 revision for the full
writeup): the leader arm's drag-teach mode does firmware gravity
compensation for us, which is why it holds position when released -- that is
the single most reliable thing about this arm's control surface, verified
live this session. Continuous MIT torque control with a *simulated* gravity
model was rejected: this project has already found the source URDF's own
inertia values wrong once (the Scout wheel inertia bug), and an arm that
depends on a possibly-wrong model to hold itself up for an entire session,
unattended, is a real hazard.

Instead: stay in drag-teach essentially all the time, and enter MIT only for
the duration of an actual contact event, anchored with kp > 0 at the
position the sim arm is physically stuck at (never a zero-stiffness
free-float). Every path -- including every watchdog trip -- ramps torque to
zero and returns to teach mode. Nothing in this module can leave the arm
torque-driven with no bound.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field


class HapticState(enum.Enum):
    FREE = 'FREE'                # drag-teach, no torque commanded by us
    ENGAGING = 'ENGAGING'        # in MIT, kp ramping up toward kp_wall
    HELD = 'HELD'                # in MIT, kp settled at kp_wall
    DISENGAGING = 'DISENGAGING'  # in MIT, kp/torque ramping down to 0


@dataclass(frozen=True)
class HapticParams:
    t_max_nm: tuple = (2.0, 2.0, 2.0, 1.0, 1.0, 1.0)  # per joint1..6, SDK ceiling is +-18
    kp_wall: float = 8.0          # SDK reference value ~10
    kp_hard_max: float = 30.0     # refuse to start if configured above this
    kd_wall: float = 0.8          # SDK reference value
    kd_valid_range: tuple = (0.0, 2.0)
    torque_slew_nm_s: float = 20.0
    kp_ramp_up_s: float = 0.15
    kp_ramp_down_s: float = 0.30
    wall_max_offset_rad: float = 0.15   # |pos_ref - leader position| clamp
    force_to_torque_k: float = 0.35     # tuned empirically in Phase 5
    contact_force_min_n: float = 2.0    # matches haply_teleoperation.py's limit_force
    contact_enter_debounce_s: float = 0.05
    contact_exit_debounce_s: float = 0.15
    max_contact_duration_s: float = 5.0  # hard timeout -- a stuck sim can't lock the arm
    contact_stale_s: float = 0.25
    deadman_stale_s: float = 0.20


@dataclass
class HapticInputs:
    """One tick's worth of external observations. All in leader-joint order
    (joint1..joint6). now_s should be a monotonic clock, not wall time."""
    now_s: float
    contact_active: bool          # gated (>= contact_force_min_n) contact flag
    contact_torque_nm: tuple      # per joint, from Isaac's tau_ext, N*m
    sim_joint_positions_rad: tuple  # per joint, the sim arm's actual position
    leader_joint_positions_rad: tuple  # per joint, real arm's current position
    contact_data_age_s: float
    deadman_held: bool
    deadman_data_age_s: float
    estop_active: bool
    arm_fault: bool               # any GetArmStatus() error flag set


@dataclass
class HapticOutput:
    state: HapticState
    enter_mit: bool         # edge-triggered: True only on the FREE->MIT transition tick
    exit_to_teach: bool     # edge-triggered: True only on the MIT->FREE transition tick
    kp: float
    kd: float
    pos_ref_rad: tuple      # per joint
    t_ref_nm: tuple         # per joint, already clamped+slewed
    fault_reason: str = ''
    # level: True for every tick state != FREE -- piper_leader_node re-sends
    # MotionCtrl_2 at motion_ctrl2_rate_hz while this holds, since the
    # firmware's MIT mode is not sticky across a CAN gap.
    in_mit: bool = False


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class HapticLaw:
    """Stateful driver. Call step(inputs) once per control tick (piper_leader_node
    runs this at control_loop_hz, e.g. 100-200 Hz)."""

    def __init__(self, params: HapticParams = HapticParams()):
        if not (0.0 < params.kp_wall <= params.kp_hard_max):
            raise ValueError(
                f'kp_wall={params.kp_wall} must be in (0, kp_hard_max={params.kp_hard_max}]')
        lo, hi = params.kd_valid_range
        if not (lo <= params.kd_wall <= hi):
            raise ValueError(f'kd_wall={params.kd_wall} outside kd_valid_range={params.kd_valid_range}')
        self.params = params
        self.state = HapticState.FREE
        self._q_wall = None            # tuple, latched anchor position
        self._kp_current = 0.0
        self._t_ref_current = [0.0] * 6
        self._contact_since = None     # for enter-debounce
        self._no_contact_since = None  # for exit-debounce
        self._engaged_since = None     # for max_contact_duration_s
        self._last_tick_s = None

    def _n_joints(self) -> int:
        return len(self.params.t_max_nm)

    def _fault(self, inputs: HapticInputs) -> str:
        if inputs.estop_active:
            return 'estop_active'
        if inputs.arm_fault:
            return 'arm_fault'
        if inputs.contact_data_age_s > self.params.contact_stale_s:
            return 'contact_data_stale'
        if inputs.deadman_data_age_s > self.params.deadman_stale_s:
            return 'deadman_data_stale'
        if not inputs.deadman_held:
            return 'deadman_released'
        return ''

    def _ramp_kp(self, dt: float, target: float, ramp_s: float) -> float:
        if ramp_s <= 0.0:
            return target
        max_step = (self.params.kp_wall / ramp_s) * dt if ramp_s > 0 else target
        if target > self._kp_current:
            return min(target, self._kp_current + max_step)
        return max(target, self._kp_current - max_step)

    def _slew_torque(self, dt: float, targets) -> list:
        max_step = self.params.torque_slew_nm_s * dt
        out = []
        for i, t in enumerate(targets):
            prev = self._t_ref_current[i]
            if t > prev:
                out.append(min(t, prev + max_step))
            else:
                out.append(max(t, prev - max_step))
        return out

    def step(self, inputs: HapticInputs) -> HapticOutput:
        dt = 0.0 if self._last_tick_s is None else max(0.0, inputs.now_s - self._last_tick_s)
        self._last_tick_s = inputs.now_s

        prev_state = self.state
        fault = self._fault(inputs)

        if fault and self.state == HapticState.FREE:
            # Already free; nothing to unwind. Deadman-released is the
            # expected steady state, not worth logging as a fault upstream.
            return HapticOutput(
                state=self.state, enter_mit=False, exit_to_teach=False,
                kp=0.0, kd=self.params.kd_wall,
                pos_ref_rad=tuple([0.0] * self._n_joints()),
                t_ref_nm=tuple([0.0] * self._n_joints()),
                fault_reason=fault)

        if fault:
            # Any fault while engaged: force straight into DISENGAGING,
            # skipping debounce -- safety wins over a clean release curve.
            self.state = HapticState.DISENGAGING
            if self._no_contact_since is None:
                self._no_contact_since = inputs.now_s

        # -- state transitions --------------------------------------------
        # NOTE: use `is None` checks below, not `x = x or default` -- now_s
        # legitimately equals 0.0 at the first tick of any fresh clock (e.g.
        # every unit test, and any monotonic clock zeroed at process start),
        # and 0.0 is falsy in Python, so `or` would silently re-latch the
        # timestamp on every tick instead of once, breaking debounce/timeout
        # timing at exactly the least convenient moment. Caught by
        # test_contact_requires_debounce_before_engaging and
        # test_max_contact_duration_forces_disengage.
        if self.state == HapticState.FREE:
            if inputs.contact_active and not fault:
                if self._contact_since is None:
                    self._contact_since = inputs.now_s
                if inputs.now_s - self._contact_since >= self.params.contact_enter_debounce_s:
                    self.state = HapticState.ENGAGING
                    self._q_wall = tuple(
                        _clip(
                            s, l - self.params.wall_max_offset_rad,
                            l + self.params.wall_max_offset_rad)
                        for s, l in zip(
                            inputs.sim_joint_positions_rad, inputs.leader_joint_positions_rad))
                    self._kp_current = 0.0
                    self._engaged_since = inputs.now_s
                    self._no_contact_since = None
            else:
                self._contact_since = None

        elif self.state in (HapticState.ENGAGING, HapticState.HELD):
            timed_out = (
                self._engaged_since is not None
                and (inputs.now_s - self._engaged_since) >= self.params.max_contact_duration_s)
            if not inputs.contact_active or timed_out:
                if self._no_contact_since is None:
                    self._no_contact_since = inputs.now_s
                if (inputs.now_s - self._no_contact_since) >= self.params.contact_exit_debounce_s \
                        or timed_out:
                    self.state = HapticState.DISENGAGING
            else:
                self._no_contact_since = None
                if self.state == HapticState.ENGAGING and self._kp_current >= self.params.kp_wall:
                    self.state = HapticState.HELD

        elif self.state == HapticState.DISENGAGING:
            if self._kp_current <= 1e-6 and all(abs(t) <= 1e-6 for t in self._t_ref_current):
                self.state = HapticState.FREE
                self._q_wall = None
                self._engaged_since = None
                self._contact_since = None

        # Edge-triggered, not level: True only on the tick a transition
        # happens, so piper_leader_node sends MotionCtrl_2/MotionCtrl_1
        # exactly once per mode change rather than every tick (it re-sends
        # MotionCtrl_2 itself at motion_ctrl2_rate_hz while in MIT -- see
        # that module -- this flag is only the initial trigger).
        entered_mit = prev_state == HapticState.FREE and self.state != HapticState.FREE
        exited_to_teach = prev_state != HapticState.FREE and self.state == HapticState.FREE

        # -- outputs per state ----------------------------------------------
        n = self._n_joints()
        if self.state == HapticState.FREE:
            self._kp_current = 0.0
            self._t_ref_current = [0.0] * n
            return HapticOutput(
                state=self.state, enter_mit=False, exit_to_teach=exited_to_teach,
                kp=0.0, kd=self.params.kd_wall,
                pos_ref_rad=tuple([0.0] * n), t_ref_nm=tuple([0.0] * n),
                fault_reason=fault, in_mit=False)

        # ENGAGING / HELD / DISENGAGING: all live in MIT.
        if self.state in (HapticState.ENGAGING, HapticState.HELD):
            self._kp_current = self._ramp_kp(dt, self.params.kp_wall, self.params.kp_ramp_up_s)
            raw_targets = [
                _clip(-self.params.force_to_torque_k * tau, -tmax, tmax)
                for tau, tmax in zip(inputs.contact_torque_nm, self.params.t_max_nm)
            ]
        else:  # DISENGAGING
            self._kp_current = self._ramp_kp(dt, 0.0, self.params.kp_ramp_down_s)
            raw_targets = [0.0] * n

        self._t_ref_current = self._slew_torque(dt, raw_targets)

        q_wall = self._q_wall if self._q_wall is not None else tuple([0.0] * n)
        return HapticOutput(
            state=self.state,
            enter_mit=entered_mit,
            exit_to_teach=False,
            kp=self._kp_current,
            kd=self.params.kd_wall,
            pos_ref_rad=q_wall,
            t_ref_nm=tuple(self._t_ref_current),
            fault_reason=fault,
            in_mit=True,
        )
