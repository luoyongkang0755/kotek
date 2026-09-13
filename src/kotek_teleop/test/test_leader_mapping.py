import math

import pytest

from kotek_teleop.leader_mapping import (
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_LIMIT_M,
    GRIPPER_MAX_WIDTH_M,
    JOINT_AXIS_SIGN,
    LIMIT_MARGIN_RAD,
    OnePoleLowPass,
    clamp_joint,
    gripper_width_to_finger_targets,
    leader_gripper_milli_mm_to_width_m,
    leader_joint_deg_milli_to_rad,
    leader_joint_raw_to_sim_rad,
    rate_limit,
)


def test_deg_milli_to_rad_known_values():
    # 90000 milli-degrees == 90 degrees == pi/2 rad
    assert leader_joint_deg_milli_to_rad(90000) == pytest.approx(math.pi / 2, abs=1e-9)
    assert leader_joint_deg_milli_to_rad(0) == 0.0
    assert leader_joint_deg_milli_to_rad(-180000) == pytest.approx(-math.pi, abs=1e-9)


# joint6 is the lone joint whose URDF axis is -Z (all others are +Z,
# confirmed by parsing piper_description.urdf) -- live-confirmed this
# session: without this correction, turning the real wrist one way spun
# the sim gripper the other way.

@pytest.mark.parametrize('name', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
def test_joint_axis_sign_positive_for_all_but_joint6(name):
    assert JOINT_AXIS_SIGN[name] == 1.0


def test_joint_axis_sign_negative_for_joint6_only():
    assert JOINT_AXIS_SIGN['joint6'] == -1.0


def test_raw_to_sim_rad_flips_sign_for_joint6_only():
    for name in ARM_JOINT_NAMES:
        raw = 90000  # 90 degrees
        expected_sign = -1.0 if name == 'joint6' else 1.0
        result = leader_joint_raw_to_sim_rad(name, raw)
        assert result == pytest.approx(expected_sign * math.pi / 2, abs=1e-9)


def test_raw_to_sim_rad_matches_plain_conversion_for_non_joint6():
    raw = -45000
    for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5'):
        assert leader_joint_raw_to_sim_rad(name, raw) == pytest.approx(
            leader_joint_deg_milli_to_rad(raw))


def test_raw_to_sim_rad_inverts_for_joint6():
    raw = -45000
    assert leader_joint_raw_to_sim_rad('joint6', raw) == pytest.approx(
        -leader_joint_deg_milli_to_rad(raw))


def test_raw_to_sim_rad_unknown_joint_defaults_to_no_flip():
    assert leader_joint_raw_to_sim_rad('not_a_real_joint', 90000) == pytest.approx(
        leader_joint_deg_milli_to_rad(90000))


def test_gripper_milli_mm_to_width_clamps_to_physical_range():
    assert leader_gripper_milli_mm_to_width_m(50000) == pytest.approx(0.05, abs=1e-9)
    assert leader_gripper_milli_mm_to_width_m(-1000) == 0.0
    # a garbled CAN read can't exceed the physically valid range
    assert leader_gripper_milli_mm_to_width_m(999_000_000) == GRIPPER_MAX_WIDTH_M


def test_gripper_width_to_finger_targets_opposing_signs():
    j7, j8 = gripper_width_to_finger_targets(0.04)
    assert j7 == pytest.approx(0.02, abs=1e-9)
    assert j8 == pytest.approx(-0.02, abs=1e-9)
    assert j7 == -j8


def test_gripper_width_to_finger_targets_clamped_to_joint_limit():
    j7, j8 = gripper_width_to_finger_targets(10.0)  # absurd width
    assert j7 == GRIPPER_JOINT_LIMIT_M
    assert j8 == -GRIPPER_JOINT_LIMIT_M


def test_gripper_width_to_finger_targets_never_negative():
    j7, j8 = gripper_width_to_finger_targets(-5.0)
    assert j7 == 0.0
    assert j8 == 0.0


@pytest.mark.parametrize('name,lower,upper', [
    ('joint1', -2.618, 2.618),
    ('joint2', 0.0, 3.14),
    ('joint3', -2.967, 0.0),
    ('joint4', -1.832, 1.832),
    ('joint5', -1.22, 1.22),
    ('joint6', -3.14, 3.14),
])
def test_clamp_joint_matches_urdf_bounds_minus_margin(name, lower, upper):
    # Confirmed directly against
    # isaac_simulation/piper_isaac_sim/piper_description/urdf/piper_description.urdf
    result_lo = clamp_joint(name, lower - 1.0)
    result_hi = clamp_joint(name, upper + 1.0)
    assert result_lo.clamped and result_lo.value == pytest.approx(lower + LIMIT_MARGIN_RAD)
    assert result_hi.clamped and result_hi.value == pytest.approx(upper - LIMIT_MARGIN_RAD)


def test_clamp_joint_never_returns_exactly_on_the_raw_urdf_bound():
    # This is the specific bug class isaac_joint_bridge's _isaac_state_cb
    # documents (joint2 lower == 0 exactly triggered START_STATE_INVALID).
    result = clamp_joint('joint2', -100.0)
    assert result.value > 0.0
    result = clamp_joint('joint3', 100.0)
    assert result.value < 0.0


def test_clamp_joint_passthrough_within_bounds():
    result = clamp_joint('joint1', 0.5)
    assert not result.clamped
    assert result.value == 0.5


def test_clamp_joint_unknown_name_passthrough():
    result = clamp_joint('not_a_real_joint', 999.0)
    assert not result.clamped
    assert result.value == 999.0


def test_all_arm_joint_names_have_limits():
    from kotek_teleop.leader_mapping import SIM_JOINT_LIMITS_RAD
    for name in ARM_JOINT_NAMES:
        assert name in SIM_JOINT_LIMITS_RAD


def test_one_pole_low_pass_converges_toward_step_input():
    lp = OnePoleLowPass(fc_hz=10.0)
    y = lp.step(0.0, dt=0.01)
    assert y == 0.0
    # step to 1.0, should approach but not jump immediately
    y1 = lp.step(1.0, dt=0.01)
    assert 0.0 < y1 < 1.0
    y_last = y1
    for _ in range(200):
        y_last = lp.step(1.0, dt=0.01)
    assert y_last == pytest.approx(1.0, abs=1e-3)


def test_one_pole_low_pass_first_sample_passes_through():
    lp = OnePoleLowPass(fc_hz=10.0)
    assert lp.step(3.5, dt=0.01) == 3.5


def test_one_pole_low_pass_reset_reanchors_without_a_jump():
    # Load-bearing for piper_leader_node's engage-edge re-anchoring: after
    # tracking one value, reset() to a different value, and the VERY NEXT
    # step toward a third value must ease from the reset point, not snap.
    lp = OnePoleLowPass(fc_hz=10.0)
    for _ in range(50):
        lp.step(1.0, dt=0.01)  # settle near 1.0
    lp.reset(5.0)
    y = lp.step(9.0, dt=0.01)  # step toward 9.0 from the reset anchor (5.0)
    assert 5.0 < y < 9.0
    assert abs(y - 5.0) < abs(9.0 - 5.0)  # eases in, doesn't jump to 9.0


def test_rate_limit_clamps_large_positive_step():
    assert rate_limit(previous=0.0, target=10.0, max_step=0.1) == pytest.approx(0.1)


def test_rate_limit_clamps_large_negative_step():
    assert rate_limit(previous=0.0, target=-10.0, max_step=0.1) == pytest.approx(-0.1)


def test_rate_limit_passes_through_small_step():
    assert rate_limit(previous=1.0, target=1.05, max_step=0.1) == pytest.approx(1.05)
