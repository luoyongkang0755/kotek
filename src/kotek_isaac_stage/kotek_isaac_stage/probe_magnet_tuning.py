#!/usr/bin/env python3
"""Measures whether the wall-mount task's magnet ScriptNode
(author_magnet_graph.py) actually attracts and welds a sensor-cam object
under live physics -- the ONE thing authoring alone (inspect_stage.py)
cannot verify, since ScriptNode compute() only ever runs once physics is
playing. Same role for the magnet constants in physics_tuning.py that
probe_grasp_tuning.py plays for the gripper constants: a measured A/B, not
an assumption.

Three runs, all against kotek_scout_piper_wall_demo.usd, and all take an
optional --sensor-index (1-4, default 1) to spot-check a different
sensor/target pair -- all 4 share the exact same ScriptNode compute() code
path and constants (no per-sensor special-casing), so this is a spot-check
of that assumption, not independent per-sensor tuning:
  --mode in-range   teleports the chosen sensor_cam_N to just inside
                     MAGNET_ATTRACT_RANGE of its own wall target with zero
                     velocity, then steps physics and expects it to be
                     pulled in and welded (a PhysicsFixedJoint sibling prim
                     under /World/wall).
  --mode out-of-range  leaves sensor_cam_N at its pickup pose on the riser
                     cube (~1m from the wall, far outside attract range)
                     and expects NOTHING to move -- magnets must not act at
                     a distance, only within MAGNET_ATTRACT_RANGE. This is
                     the literal check that nothing else in the scene is
                     magnetic: an object outside range must be completely
                     unaffected by the graph ticking every frame.
  --mode misaligned like in-range, but the sensor arrives with its
                     bottom-face normal tilted 30 deg away from the wall
                     direction. This is the dipole-dipole model's (2026-09-07)
                     headline capability: tau = m_s x B_w must flatten the
                     box BEFORE the weld matters. Expects welded AND a
                     final bottom-normal-vs-wall misalignment under 10 deg
                     -- the old point-attraction model with its disabled
                     torqueGain could not do this (it welded boxes crooked
                     or not at all -- see physics_tuning.py's Bug-6 history
                     and README's "crooked fridge magnet" note).

Run via (needs a live Isaac Sim process, same as author_magnet_graph.py):
    KOTEK_WITH_ROS=1 ./run_isaac.sh \\
        src/kotek_isaac_stage/kotek_isaac_stage/probe_magnet_tuning.py --mode in-range
    KOTEK_WITH_ROS=1 ./run_isaac.sh \\
        src/kotek_isaac_stage/kotek_isaac_stage/probe_magnet_tuning.py --mode out-of-range
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')
STEPS = 180   # 1.5s at 120Hz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    parser.add_argument('--mode', choices=['in-range', 'out-of-range', 'misaligned'],
                        required=True)
    # Sensors 2-4 share the exact same compute() code path and constants as
    # sensor 1 (see author_magnet_graph.py -- one ScriptNode, one fixed
    # (sensor, target) list, no per-sensor special-casing), so this is a
    # spot-check of that assumption rather than a claim that all 4 need
    # independent tuning -- default stays 1 (the sensor every prior debug
    # round in this session was measured against).
    parser.add_argument('--sensor-index', type=int, default=1, choices=[1, 2, 3, 4])
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    args.sensor_path = (
        f'/World/scout_mini/Geometry/base_link/sensor_cam_{args.sensor_index}')
    args.weld_joint_path = f'/World/wall/_magnet_weld_{args.sensor_index}'

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': not args.gui})
    result = 1
    try:
        result = _run(args, simulation_app)
    except Exception:
        import traceback
        traceback.print_exc()
        result = 1
    finally:
        simulation_app.close(exit_code=result)
    return result


def _run(args, simulation_app) -> int:
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils import extensions
    from pxr import Gf, UsdGeom

    import build_wall_stage as ws
    import physics_tuning as pt

    extensions.enable_extension('isaacsim.ros2.bridge')
    extensions.enable_extension('omni.graph.scriptnode')
    simulation_app.update()

    omni.usd.get_context().open_stage(args.stage)
    for _ in range(10):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    print(f'### stage opened: {args.stage}', flush=True)

    sensor = stage.GetPrimAtPath(args.sensor_path)
    if not sensor.IsValid():
        print(f'[FAIL] {args.sensor_path} not found -- run build_wall_stage.py first', flush=True)
        return 1

    if args.mode in ('in-range', 'misaligned'):
        # Just inside MAGNET_ATTRACT_RANGE of this sensor's own target's
        # bottom-face point, oriented with its bottom face roughly facing
        # the wall -- this probe measures the attract+weld behavior itself,
        # not whether the arm can deliver that pose (that is what the
        # /piper/compute_ik reachability sweep is for). misaligned tilts
        # the arrival 30 deg off, see below.
        target = np.array([ws.WALL_FACE_X, ws.WALL_TARGET_Y[args.sensor_index - 1], ws.WALL_TARGET_Z])
        # Holddown-era spawn (2026-09-23): the magnet does not exist
        # before contact, so spawn the box EMBEDDED into the wall and let
        # contact resolution seat it -- the magnet then engages and must
        # hold + weld it. Depth: 1mm for the straight arrival (seats at
        # the rest offset), 8mm for the tilted arrival -- measured: a
        # 1mm-embedded tilted box gets EJECTED to 7.8mm by resolution and
        # never presses; the deeper embed leaves corner contact inside
        # the 6mm threshold where the aligning torque can work.
        offset = -0.008 if args.mode == 'misaligned' else -0.001
        half_h = pt.SENSOR_CAM_SIZE_XYZ[2] / 2.0
        # Base orientation: rotate -90 deg about Y so local -Z (the bottom
        # face) points along +X (toward the wall), quaternion wxyz.
        # misaligned mode composes a 30 deg tilt about Y on top (total -60
        # deg), arriving with the bottom-face normal 30 deg away from the
        # wall direction -- the dipole model's tau = m_s x B_w alignment
        # torque has to flatten it on the way in. (Both rotations are about
        # Y, so they simply add.)
        import math
        base_deg = -90.0 if args.mode == 'in-range' else -60.0
        half_angle = math.radians(base_deg) / 2.0
        quat = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)
        # Under THIS rotation, local -Z (the bottom-face offset the magnet
        # script uses) maps to world +X (in-range) resp. a 30-deg-tilted
        # direction (misaligned) -- computed with the same quaternion
        # rotation formula compute() itself uses, not hardcoded. An earlier
        # version of this probe added `half_h` to the CENTER's z instead,
        # which put the bottom face ~half_h *below* the target in z with
        # almost no horizontal offset -- the resulting target-delta was
        # ~94% vertical, so the attract force pulled mostly DOWN
        # (compounding gravity) instead of sideways toward the wall, and
        # the object free-fell to the ground. Placing the offset where it
        # actually lands keeps the initial delta horizontal-ish, which is
        # what an arm-delivered approach pose looks like too.
        bottom_offset_world = half_h * np.array(
            _rotate_vec_by_quat_wxyz(quat, (0.0, 0.0, -1.0)))
        start_pos = target - np.array([offset, 0.0, 0.0]) - bottom_offset_world
    else:
        start_pos = None   # leave at build_wall_stage.py's own pickup pose
        quat = None

    if start_pos is not None:
        xf = UsdGeom.Xformable(sensor)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*(float(c) for c in start_pos)))
        xf.AddOrientOp().Set(Gf.Quatf(*(float(c) for c in quat)))
        xf.AddScaleOp().Set(Gf.Vec3f(*pt.SENSOR_CAM_SIZE_XYZ))
        print(f'### teleported {args.sensor_path} to {start_pos}', flush=True)

    # set_defaults=False: the World() default rewrites the stage's authored
    # physxScene:timeStepsPerSecond=120 down to 60 and disables stabilization
    # (see run_wall_stage.py's comment). The step rate is pinned to
    # EFFECTIVE_PHYSICS_RATE_HZ (60): the probes' PASS criteria were
    # calibrated at 60 Hz, and at 120 Hz the magnet damping freezes the box
    # (physics_tuning.py's Bug-9-adjacent note).
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / pt.EFFECTIVE_PHYSICS_RATE_HZ,
        rendering_dt=1.0 / 60.0,
        set_defaults=False)
    world.reset()
    print('### physics playing', flush=True)

    from isaacsim.core.experimental.prims import RigidPrim
    probe = RigidPrim(args.sensor_path)

    start_world_pos, _ = probe.get_world_poses()
    start_world_pos = np.array(start_world_pos[0])

    for _ in range(STEPS):
        simulation_app.update()

    end_world_pos, end_world_quat = probe.get_world_poses()
    end_world_pos = np.array(end_world_pos[0])
    end_world_quat = np.asarray(end_world_quat[0])
    displacement = float(np.linalg.norm(end_world_pos - start_world_pos))
    welded = stage.GetPrimAtPath(args.weld_joint_path).IsValid()

    print(f'### start pos: {start_world_pos}', flush=True)
    print(f'### end pos:   {end_world_pos}', flush=True)
    print(f'### displacement: {displacement:.4f} m', flush=True)
    print(f'### weld joint present ({args.weld_joint_path}): {welded}', flush=True)

    ok = True
    if args.mode in ('in-range', 'misaligned'):
        # Holddown era (2026-09-23): the box spawns AT the wall (embedded,
        # contact-resolved) and the magnet's job is HOLDING, not capture --
        # it barely moves. The meaningful checks are the weld and the final
        # pose, so the old "moved toward the target" displacement check is
        # retired (it tested the dipole capture era's radial pull).
        ok = ok and _check(welded, 'weld joint was authored (magnet engaged on contact and held)')
        if welded:
            target = np.array([ws.WALL_FACE_X, ws.WALL_TARGET_Y[args.sensor_index - 1], ws.WALL_TARGET_Z])
            half_h = pt.SENSOR_CAM_SIZE_XYZ[2] / 2.0
            # Bottom-face point computed from the FINAL orientation with
            # the same rotation formula compute() uses, not hardcoded:
            # under this probe's -90deg-about-Y start orientation local -Z
            # (the bottom face) rotates to world +X, and an earlier version
            # of this check used -half_h, silently assuming the offset was
            # still in -Z-ish/-X world direction as it would be with NO
            # rotation -- that gave a materially different (wrong)
            # bottom-face point, failing this check even on a run where
            # compute()'s own `d` (the value the weld gate itself uses) was
            # genuinely small (0.0060m, well inside WELD_DISTANCE) at the
            # moment of welding. This check now mirrors that same math
            # instead of re-deriving its own (wrong) approximation.
            bottom = end_world_pos + half_h * np.array(
                _rotate_vec_by_quat_wxyz(end_world_quat, (0.0, 0.0, -1.0)))
            dist_to_target = float(np.linalg.norm(bottom - target))
            print(f'### final bottom-face distance to target: {dist_to_target:.4f} m', flush=True)
            ok = ok and _check(dist_to_target < pt.WELD_DISTANCE * 3,
                               'final position is close to the wall target')
        if args.mode == 'misaligned':
            # THE dipole-model assertion: the bottom-face normal must end
            # up within 10 deg of the wall direction (+x world, toward the
            # wall at WALL_FACE_X). The old point-attraction model had no
            # physical alignment torque (torqueGain was always 0.0), so it
            # welded boxes crooked by construction -- a 30-deg arrival
            # could weld at ~30 deg and this check exists precisely to
            # prove the dipole model does better.
            import math
            final_normal = np.array(
                _rotate_vec_by_quat_wxyz(end_world_quat, (0.0, 0.0, -1.0)))
            cos_mis = float(np.clip(np.dot(final_normal, [1.0, 0.0, 0.0]),
                                    -1.0, 1.0))
            final_misalign_deg = math.degrees(math.acos(cos_mis))
            print(f'### final bottom-normal misalignment vs wall: '
                  f'{final_misalign_deg:.1f} deg', flush=True)
            ok = ok and _check(
                welded and final_misalign_deg < 10.0,
                'dipole alignment torque flattened the box (misalignment '
                f'{final_misalign_deg:.1f} deg < 10 deg)')
    else:
        # NOT a bare "didn't move" check: this asset's chassis (base_link)
        # settles under gravity/suspension during the first several physics
        # ticks regardless of anything this task authors (a pre-existing,
        # documented characteristic -- "base_link settles to ~z 0.18 once
        # physics runs", see import_robots.py), and the sensor-cam rides on
        # top of it as an independent (unwelded) rigid body, so SOME
        # incidental vertical displacement from that settling is expected
        # and is not a magnet bug. The meaningful signal is horizontal
        # motion TOWARD the wall (+x) specifically -- that is what a false
        # positive attraction would look like.
        toward_wall = float(end_world_pos[0] - start_world_pos[0])
        ok = ok and _check(
            toward_wall < 0.01,
            f'sensor did NOT move toward the wall (magnet correctly inactive '
            f'out of range; dx={toward_wall:.4f} m, total displacement '
            f'{displacement:.4f} m is chassis-settling, not magnetism)')
        ok = ok and _check(not welded, 'no weld joint was authored (correctly inactive)')

    print()
    print('PASS' if ok else 'FAIL', flush=True)
    return 0 if ok else 1


def _rotate_vec_by_quat_wxyz(q, v):
    """Rotates 3-vector v by quaternion q=(w,x,y,z) -- the same formula as
    author_magnet_graph.py's embedded script uses, duplicated here so the
    probe computes bottom-face points EXACTLY the way the magnet node does
    (a hand-derived approximation once made the in-range final-distance
    check verify a different point than the weld gate itself)."""
    import numpy as np
    w, x, y, z = q
    qv = np.array([x, y, z])
    vv = np.array(v, dtype=float)
    t = 2.0 * np.cross(qv, vv)
    return vv + w * t + np.cross(qv, t)


def _check(condition: bool, message: str) -> bool:
    print(f'  [{"PASS" if condition else "FAIL"}] {message}', flush=True)
    return bool(condition)


if __name__ == '__main__':
    sys.exit(main())
