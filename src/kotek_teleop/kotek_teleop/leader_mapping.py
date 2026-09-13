"""Pure functions mapping the real Piper leader arm's feedback onto the
simulated Piper's joint-space targets. No ROS, no CAN, no piper_sdk import --
this is the unit-test surface; piper_leader_node.py is a thin ROS/CAN shell
around it.

Unit conversions and joint limits below were confirmed live against this
project's actual hardware and stage this session:
  - GetArmJointMsgs() reports joint angle in 0.001 degree units (piper_sdk
    docstring, and confirmed against a live capture).
  - GetArmGripperMsgs().grippers_angle reports gripper opening in 0.001 mm.
  - The sim's Piper URDF (parsed directly from
    isaac_simulation/piper_isaac_sim/piper_description/urdf/piper_description.urdf)
    gives joint2's lower bound as exactly 0.0 rad and joint3's upper bound as
    exactly 0.0 rad -- both real limits the physical arm does not share, and
    both previously caused a live MoveIt START_STATE_INVALID bug in
    kotek_isaac_bridge.isaac_joint_bridge (see that module's _isaac_state_cb
    docstring). We pull every bound inward by LIMIT_MARGIN_RAD so a teleop
    target is never published exactly on a limit, which is what triggered
    that bug in the first place (PhysX's position drive settles a hair past
    an exact-zero bound).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

DEG_MILLI_TO_RAD = 0.001 * math.pi / 180.0  # piper_sdk joint units -> radians
MM_MILLI_TO_M = 1e-6  # piper_sdk gripper units (0.001 mm) -> meters

ARM_JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
GRIPPER_JOINT_NAMES = ('joint7', 'joint8')

# Raw URDF bounds (radians), pulled inward by LIMIT_MARGIN_RAD below.
_URDF_LIMITS_RAD = {
    'joint1': (-2.618, 2.618),
    'joint2': (0.0, 3.14),
    'joint3': (-2.967, 0.0),
    'joint4': (-1.832, 1.832),
    'joint5': (-1.22, 1.22),
    'joint6': (-3.14, 3.14),
}
LIMIT_MARGIN_RAD = 0.01
SIM_JOINT_LIMITS_RAD = {
    name: (lo + LIMIT_MARGIN_RAD, hi - LIMIT_MARGIN_RAD)
    for name, (lo, hi) in _URDF_LIMITS_RAD.items()
}

GRIPPER_MAX_WIDTH_M = 0.08  # link7/link8 prismatic range is [0, 0.04] each side
GRIPPER_JOINT_LIMIT_M = 0.04

# joint1-5 all rotate about the URDF's +Z (confirmed by parsing every
# <axis xyz="..."> in piper_description.urdf); joint6 is the lone exception,
# rotating about -Z. The real arm's raw feedback (GetArmJointMsgs) uses one
# consistent physical/motor convention across all six joints -- it has no
# notion of the sim's per-link URDF axis orientation -- so copying joint6's
# value straight through, the same way the other five are handled, spins
# the sim's gripper the WRONG way relative to the real wrist. Confirmed
# live this session: turning the real wrist one way turned the sim gripper
# the other way, exactly the signature of this axis flip.
JOINT_AXIS_SIGN = {
    'joint1': 1.0, 'joint2': 1.0, 'joint3': 1.0,
    'joint4': 1.0, 'joint5': 1.0, 'joint6': -1.0,
}


@dataclass
class ClampResult:
    value: float
    clamped: bool


def leader_joint_deg_milli_to_rad(raw_deg_milli: int) -> float:
    """0.001-degree integer feedback -> radians. Unit conversion ONLY --
    does not apply JOINT_AXIS_SIGN. Use leader_joint_raw_to_sim_rad for the
    full leader->sim conversion; this stays a plain unit converter so it's
    trivially testable and reusable (e.g. for diagnostics that want the
    leader's own raw angle, not the sim-axis-corrected one)."""
    return raw_deg_milli * DEG_MILLI_TO_RAD


def leader_joint_raw_to_sim_rad(name: str, raw_deg_milli: int) -> float:
    """Full leader->sim conversion for one arm joint: unit conversion plus
    the joint6 axis-sign correction (see JOINT_AXIS_SIGN). This is what
    piper_leader_node should call for every arm joint -- calling
    leader_joint_deg_milli_to_rad directly would reproduce the joint6
    inversion bug."""
    return JOINT_AXIS_SIGN.get(name, 1.0) * leader_joint_deg_milli_to_rad(raw_deg_milli)


def leader_gripper_milli_mm_to_width_m(raw_milli_mm: int) -> float:
    """0.001mm integer feedback -> gripper opening width in meters, clamped
    to the physically valid range so a garbled CAN read can't produce a
    finger target far outside [0, GRIPPER_MAX_WIDTH_M]."""
    width = raw_milli_mm * MM_MILLI_TO_M
    return max(0.0, min(GRIPPER_MAX_WIDTH_M, width))


def gripper_width_to_finger_targets(width_m: float) -> tuple[float, float]:
    """width -> (joint7, joint8): joint7 = +width/2, joint8 = -width/2,
    confirmed against import_robots.py's authored joint7/joint8 prismatic
    axes (link6 -> link7 / link6 -> link8, opposing signs)."""
    half = max(0.0, min(GRIPPER_JOINT_LIMIT_M, width_m / 2.0))
    return half, -half


def clamp_joint(name: str, value_rad: float) -> ClampResult:
    """Clamp one arm joint to SIM_JOINT_LIMITS_RAD. Unknown joint names pass
    through unclamped (gripper joints are handled separately, by width)."""
    limits = SIM_JOINT_LIMITS_RAD.get(name)
    if limits is None:
        return ClampResult(value_rad, False)
    lo, hi = limits
    if value_rad < lo:
        return ClampResult(lo, True)
    if value_rad > hi:
        return ClampResult(hi, True)
    return ClampResult(value_rad, False)


class OnePoleLowPass:
    """First-order low-pass, cutoff fc (Hz), sampled at a caller-supplied dt.
    Rejects the CAN-quantization jitter visible in raw leader joint reads
    without adding perceptible teleop lag at fc=10Hz / typical loop dt~=0.01s."""

    def __init__(self, fc_hz: float):
        self._fc = fc_hz
        self._y = None

    def reset(self, value: float = 0.0) -> None:
        self._y = value

    def step(self, x: float, dt: float) -> float:
        if self._y is None:
            self._y = x
            return x
        # alpha = dt / (RC + dt), RC = 1 / (2*pi*fc)
        rc = 1.0 / (2.0 * math.pi * self._fc)
        alpha = dt / (rc + dt)
        self._y = self._y + alpha * (x - self._y)
        return self._y


def rate_limit(previous: float, target: float, max_step: float) -> float:
    """Clamp |target - previous| to max_step (e.g. max_rad_per_s * dt)."""
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target
