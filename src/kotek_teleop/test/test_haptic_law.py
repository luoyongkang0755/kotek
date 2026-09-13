import pytest

from kotek_teleop.haptic_law import HapticInputs, HapticLaw, HapticParams, HapticState

N = 6
ZERO6 = tuple([0.0] * N)


def make_inputs(
    t, contact=False, torque=ZERO6, sim_pos=ZERO6, leader_pos=ZERO6,
    contact_age=0.0, deadman=True, deadman_age=0.0, estop=False, fault=False,
):
    return HapticInputs(
        now_s=t, contact_active=contact, contact_torque_nm=torque,
        sim_joint_positions_rad=sim_pos, leader_joint_positions_rad=leader_pos,
        contact_data_age_s=contact_age, deadman_held=deadman,
        deadman_data_age_s=deadman_age, estop_active=estop, arm_fault=fault,
    )


def fast_params(**overrides):
    base = dict(
        contact_enter_debounce_s=0.02, contact_exit_debounce_s=0.02,
        kp_ramp_up_s=0.02, kp_ramp_down_s=0.02, torque_slew_nm_s=1000.0,
        max_contact_duration_s=1.0,
    )
    base.update(overrides)
    return HapticParams(**base)


def run_ticks(law, t0, dt, n, **kwargs):
    out = None
    t = t0
    for _ in range(n):
        out = law.step(make_inputs(t, **kwargs))
        t += dt
    return out, t


def test_starts_free():
    law = HapticLaw(fast_params())
    assert law.state == HapticState.FREE


def test_no_contact_stays_free():
    law = HapticLaw(fast_params())
    out, _ = run_ticks(law, 0.0, 0.01, 20, contact=False)
    assert out.state == HapticState.FREE
    assert out.kp == 0.0
    assert all(t == 0.0 for t in out.t_ref_nm)
    assert not out.in_mit


def test_contact_requires_debounce_before_engaging():
    law = HapticLaw(fast_params(contact_enter_debounce_s=0.1))
    out = law.step(make_inputs(0.0, contact=True))
    assert out.state == HapticState.FREE  # not yet -- debounce not elapsed
    out = law.step(make_inputs(0.05, contact=True))
    assert out.state == HapticState.FREE
    out = law.step(make_inputs(0.11, contact=True))
    assert out.state != HapticState.FREE  # debounce elapsed


def test_full_happy_path_free_to_held_to_free():
    law = HapticLaw(fast_params())
    t = 0.0
    dt = 0.01
    # engage
    out, t = run_ticks(law, t, dt, 5, contact=True, torque=(1.0,) * N)
    assert out.state in (HapticState.ENGAGING, HapticState.HELD)
    # settle into HELD
    out, t = run_ticks(law, t, dt, 20, contact=True, torque=(1.0,) * N)
    assert out.state == HapticState.HELD
    assert out.kp == pytest.approx(law.params.kp_wall)
    assert out.in_mit
    # release contact -> DISENGAGING -> FREE
    out, t = run_ticks(law, t, dt, 30, contact=False)
    assert out.state == HapticState.FREE
    assert out.kp == 0.0
    assert all(abs(x) < 1e-6 for x in out.t_ref_nm)
    assert not out.in_mit


def test_enter_mit_is_edge_triggered_once():
    law = HapticLaw(fast_params())
    t = 0.0
    edges = 0
    for _ in range(30):
        out = law.step(make_inputs(t, contact=True, torque=(1.0,) * N))
        if out.enter_mit:
            edges += 1
        t += 0.01
    assert edges == 1


def test_exit_to_teach_is_edge_triggered_once():
    law = HapticLaw(fast_params())
    t = 0.0
    for _ in range(10):
        law.step(make_inputs(t, contact=True, torque=(1.0,) * N))
        t += 0.01
    edges = 0
    for _ in range(50):
        out = law.step(make_inputs(t, contact=False))
        if out.exit_to_teach:
            edges += 1
        t += 0.01
    assert edges == 1


def test_torque_clamped_to_t_max():
    params = fast_params(t_max_nm=(2.0,) * N, force_to_torque_k=1.0)
    law = HapticLaw(params)
    t = 0.0
    out, t = run_ticks(law, t, 0.01, 30, contact=True, torque=(100.0,) * N)
    assert all(abs(x) <= 2.0 + 1e-9 for x in out.t_ref_nm)


def test_torque_sign_opposes_external_torque():
    params = fast_params(force_to_torque_k=1.0, t_max_nm=(50.0,) * N)
    law = HapticLaw(params)
    t = 0.0
    out, t = run_ticks(law, t, 0.01, 30, contact=True, torque=(5.0,) * N)
    assert all(x < 0 for x in out.t_ref_nm)  # opposes positive external torque


def test_wall_max_offset_clamps_bogus_sim_position():
    params = fast_params(wall_max_offset_rad=0.15)
    law = HapticLaw(params)
    # sim position is wildly far from leader position (garbage/stale read)
    out = law.step(make_inputs(
        0.0, contact=True, sim_pos=(999.0,) * N, leader_pos=ZERO6))
    out = law.step(make_inputs(
        0.03, contact=True, sim_pos=(999.0,) * N, leader_pos=ZERO6))
    assert all(abs(p) <= 0.15 + 1e-9 for p in out.pos_ref_rad)


def test_max_contact_duration_forces_disengage():
    # A stuck contact signal must not be able to lock the arm in MIT
    # indefinitely: even under continuous contact_active=True, the hard
    # timeout forces a release. With exit_debounce huge (10s), the release
    # can only be explained by the timeout, not the normal exit path. Since
    # contact_active stays True, the law legitimately re-engages afterwards
    # (retry-after-cooldown, not a permanent lockout) -- so this test checks
    # that DISENGAGING/FREE was visited within the timeout window, not that
    # it's the final state after a longer run.
    params = fast_params(max_contact_duration_s=0.2, contact_exit_debounce_s=10.0)
    law = HapticLaw(params)
    t = 0.0
    seen_release = False
    for _ in range(40):
        out = law.step(make_inputs(t, contact=True, torque=(1.0,) * N))
        if out.state in (HapticState.DISENGAGING, HapticState.FREE):
            seen_release = True
        t += 0.01
        if seen_release:
            break
    assert seen_release, 'contact never released despite max_contact_duration_s timeout'


@pytest.mark.parametrize('break_kwargs', [
    dict(estop=True),
    dict(fault=True),
    dict(contact_age=999.0),
    dict(deadman_age=999.0),
    dict(deadman=False),
])
def test_every_watchdog_forces_disengage_from_held(break_kwargs):
    law = HapticLaw(fast_params())
    t = 0.0
    out, t = run_ticks(law, t, 0.01, 20, contact=True, torque=(1.0,) * N)
    assert out.state == HapticState.HELD
    out = law.step(make_inputs(t, contact=True, torque=(1.0,) * N, **break_kwargs))
    assert out.state == HapticState.DISENGAGING
    assert out.fault_reason != ''


def test_deadman_release_mid_hold_ramps_to_free():
    law = HapticLaw(fast_params())
    t = 0.0
    out, t = run_ticks(law, t, 0.01, 20, contact=True, torque=(1.0,) * N)
    assert out.state == HapticState.HELD
    out, t = run_ticks(law, t, 0.01, 30, contact=True, torque=(1.0,) * N, deadman=False)
    assert out.state == HapticState.FREE
    assert out.kp == 0.0


def test_estop_from_free_state_stays_free_no_crash():
    law = HapticLaw(fast_params())
    out = law.step(make_inputs(0.0, estop=True, deadman=False))
    assert out.state == HapticState.FREE


def test_estop_from_engaging_forces_disengage():
    law = HapticLaw(fast_params())
    t = 0.0
    out, t = run_ticks(law, t, 0.01, 3, contact=True, torque=(1.0,) * N)
    assert out.state == HapticState.ENGAGING
    out = law.step(make_inputs(t, contact=True, torque=(1.0,) * N, estop=True))
    assert out.state == HapticState.DISENGAGING


def test_never_produces_nonzero_torque_while_free():
    law = HapticLaw(fast_params())
    t = 0.0
    for _ in range(50):
        out = law.step(make_inputs(t, contact=False))
        assert out.state != HapticState.FREE or all(x == 0.0 for x in out.t_ref_nm)
        t += 0.01


def test_kp_wall_must_be_positive_and_under_hard_max():
    with pytest.raises(ValueError):
        HapticLaw(HapticParams(kp_wall=0.0))
    with pytest.raises(ValueError):
        HapticLaw(HapticParams(kp_wall=100.0, kp_hard_max=30.0))


def test_kd_wall_must_be_in_valid_range():
    with pytest.raises(ValueError):
        HapticLaw(HapticParams(kd_wall=5.0, kd_valid_range=(0.0, 2.0)))


def test_torque_slew_limits_step_change():
    params = fast_params(torque_slew_nm_s=10.0, force_to_torque_k=1.0, t_max_nm=(50.0,) * N)
    law = HapticLaw(params)
    t = 0.0
    # get into HELD first with zero external torque
    out, t = run_ticks(law, t, 0.01, 20, contact=True, torque=ZERO6)
    assert out.state == HapticState.HELD
    # now a sudden large external torque appears in one tick
    out = law.step(make_inputs(t, contact=True, torque=(20.0,) * N))
    # with slew=10 Nm/s and dt=0.01s, max change this tick is 0.1 Nm
    assert all(abs(x) <= 0.1 + 1e-6 for x in out.t_ref_nm)
