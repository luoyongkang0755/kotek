import pytest

from kotek_teleop.spacemouse_teleop_node import _angular_raw, _deadzone_rescale, _linear_raw


def test_deadzone_zeroes_small_values():
    assert _deadzone_rescale(0.05, deadzone=0.08) == 0.0
    assert _deadzone_rescale(-0.05, deadzone=0.08) == 0.0


def test_deadzone_continuous_at_edge():
    # Just past the deadzone edge should be very close to 0, not a jump.
    just_past = _deadzone_rescale(0.081, deadzone=0.08)
    assert 0.0 < just_past < 0.01


def test_deadzone_full_scale_maps_to_one():
    assert _deadzone_rescale(1.0, deadzone=0.08) == pytest.approx(1.0)
    assert _deadzone_rescale(-1.0, deadzone=0.08) == pytest.approx(-1.0)


def test_deadzone_midpoint():
    # value=0.5, deadzone=0.1 -> (0.5-0.1)/(1-0.1) = 0.4/0.9
    assert _deadzone_rescale(0.5, deadzone=0.1) == pytest.approx(0.4 / 0.9)


MAX_RAW = 350.0

# Mean values from the structured 8-gesture recording session
# (scripts/spacemouse/record_gesture.py -> gesture_log.csv), used here as
# regression fixtures so a future refactor can't silently reintroduce the
# y-based creep bug or break the rz/ry turn combination. Columns:
# (x, y, z, rx, ry, rz) -- only z, ry, rz are used by the mapping.
GESTURES = {
    'front_only':     dict(y=-128.08, z=-236.06, rx=317.36, ry=-0.36, rz=49.93),
    'back_only':      dict(y=-135.04, z=242.43, rx=-323.10, ry=-3.01, rz=4.11),
    'front_right':    dict(y=-103.78, z=-221.93, rx=206.62, ry=18.19, rz=-251.74),
    'front_left':     dict(y=-70.86, z=-66.17, rx=146.10, ry=1.83, rz=341.83),
    'back_left':      dict(y=-113.36, z=130.16, rx=-243.37, ry=2.82, rz=261.79),
    'back_right':     dict(y=-91.86, z=66.72, rx=-92.34, ry=2.00, rz=-350.0),
    'rotation_right': dict(y=-101.85, z=-2.0, rx=-2.48, ry=-349.96, rz=4.0),
    'rotation_left':  dict(y=-109.42, z=1.79, rx=3.31, ry=350.0, rz=-48.82),
}


def _linear(gesture):
    return _linear_raw(GESTURES[gesture]['z'], MAX_RAW)


def _angular(gesture):
    g = GESTURES[gesture]
    return _angular_raw(g['rz'], g['ry'], MAX_RAW)


@pytest.mark.parametrize('gesture', ['front_only', 'front_right', 'front_left'])
def test_linear_positive_forward_for_every_forward_gesture(gesture):
    assert _linear(gesture) > 0.0


@pytest.mark.parametrize('gesture', ['back_only', 'back_left', 'back_right'])
def test_linear_negative_backward_for_every_backward_gesture(gesture):
    assert _linear(gesture) < 0.0


@pytest.mark.parametrize('gesture', ['rotation_left', 'rotation_right'])
def test_linear_near_zero_during_pure_rotation_no_creep(gesture):
    # This is the specific bug the old y-based mapping had: y sat at -102
    # to -109 during pure rotation, producing ~0.3 of unwanted forward
    # creep. z-based linear must stay negligible here instead.
    assert abs(_linear(gesture)) < 0.02


def test_linear_back_only_is_not_attenuated_by_stray_y():
    # The old mapping (max(0,-y) - max(0,z)) reduced back_only's net speed
    # to ~0.31 (0.386 forward-creep cancelling 0.691 backward) even though
    # the operator was doing a clean, deliberate backward gesture. The new
    # z-only mapping must deliver close to the full raw magnitude.
    result = _linear('back_only')
    assert result == pytest.approx(-242.43 / MAX_RAW, abs=1e-3)
    assert result < -0.6  # not attenuated to roughly half by y-noise


@pytest.mark.parametrize('gesture', ['front_left', 'back_left', 'rotation_left'])
def test_angular_positive_left_for_every_left_gesture(gesture):
    assert _angular(gesture) > 0.0


@pytest.mark.parametrize('gesture', ['front_right', 'back_right', 'rotation_right'])
def test_angular_negative_right_for_every_right_gesture(gesture):
    assert _angular(gesture) < 0.0


def test_angular_combines_rz_and_ry_additively():
    assert _angular_raw(motion_rz=100.0, motion_ry=50.0, max_raw=350.0) == \
        pytest.approx(150.0 / 350.0)


def test_angular_dominant_axis_alone_still_works():
    # rz~0 during pure rotation, ry~0 during diagonal pushes -- either axis
    # alone must still produce a clear signal (this is the whole point of
    # summing rather than picking one axis).
    assert _angular_raw(motion_rz=0.0, motion_ry=350.0, max_raw=350.0) == pytest.approx(1.0)
    assert _angular_raw(motion_rz=350.0, motion_ry=0.0, max_raw=350.0) == pytest.approx(1.0)


def test_linear_raw_sign_matches_stage_convention():
    # Established and re-confirmed live this session: positive linear.x ==
    # forward. z negative (forward push) must produce positive linear.
    assert _linear_raw(motion_z=-100.0, max_raw=350.0) > 0.0
    assert _linear_raw(motion_z=100.0, max_raw=350.0) < 0.0
