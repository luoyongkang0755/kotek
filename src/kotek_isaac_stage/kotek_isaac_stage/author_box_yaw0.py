#!/usr/bin/env python3
"""Axis-align the authored yaw of the 4 sensor-cam boxes (F group, 2026-09-29).

History (docs/wall_mount_contact_placement.md section 0): the boxes were
originally yaw-rotated by each corner's radial atan2(y, x) so that, in the
radial-approach era, the box's short (0.035 m) face met the gripper's
opening axis at that corner. The grasp later switched to the axis-aligned
{0, +-pi} snap (grasp_yaw_snap_step), but the box rotation was never
revisited -- so every grasp today starts 45 deg misaligned against the
jaws and relies on the slow close to self-center the box DURING the close
(D-group post-grasp skew median 3.0 deg is the evidence that re-alignment
happens mid-close). The E group (2026-09-28) then proved that making the
ARM follow the box's authored radial yaw is a regression (2/10 COMPLETE:
those arm configurations were never IK-verified at the riser corners).

This script is the complementary single factor: keep the verified
axis-aligned snap, and REMOVE the 45 deg mismatch at the source by
authoring the boxes axis-aligned (yaw = 0, identity orient op). The jaws
then close parallel to the box's large faces from FIRST contact -- the
mid-close re-orientation window, and its first-contact asymmetry (the
residual 10 percent kick source per the D-group mechanism picture), is
eliminated instead of damped.

Stage edit only (4 orient ops -> identity); the op ORDER
(translate -> orient -> scale) is preserved exactly because
probe_magnet_tuning.py teleports these prims assuming that ordering.
Idempotent; live Isaac, save-in-place convention of
author_riser_friction.py. physics_tuning/build_wall_stage.py kept in
lockstep for future rebuilds.

Run:
    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_box_yaw0.py
"""
import argparse
import os
import shutil
import sys

from isaacsim import SimulationApp

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')
BACKUP = '/home/trs/e2e_runs/kotek_scout_piper_wall_demo.before_boxyaw0.usd'

BOX_PATHS = [
    f'/World/scout_mini/Geometry/base_link/sensor_cam_{i + 1}'
    for i in range(4)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': True})
    ok = False
    try:
        import math

        import omni.usd
        from pxr import Gf, Usd, UsdGeom

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        print('### stage opened', flush=True)

        if not os.path.exists(BACKUP):
            shutil.copy2(args.stage, BACKUP)
            print(f'### backed up stage to {BACKUP}', flush=True)

        identity = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
        for path in BOX_PATHS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f'{path} missing')
            xformable = UsdGeom.Xformable(prim)
            op = xformable.GetOrientOp()
            if op is None:
                raise RuntimeError(f'{path} has no orient op (op ordering changed?)')
            before = op.Get()
            op.Set(identity)
            # World-yaw verification: decompose the local-to-world matrix.
            # The parent chain (scout base_link) contributes no yaw on this
            # stage (measured 2026-09-28 via the grasp logs: authored-radial
            # boxes reported world yaw == atan2 of their corner), so after
            # the edit the world yaw must be ~0, not ~atan2(y, x).
            m = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            rot = m.ExtractRotation()
            axis = rot.GetAxis()
            angle = rot.GetAngle()
            # Project the rotation onto world Z for a yaw reading.
            yaw_world = math.copysign(
                angle * abs(axis[2]), axis[2] * 1.0) if axis[2] != 0.0 else 0.0
            yaw_world = (yaw_world + math.pi) % (2.0 * math.pi) - math.pi
            local = prim.GetAttribute('xformOp:translate').Get()
            radial = math.atan2(local[1], local[0])
            print(f'### {path}: orient {before} -> identity; '
                  f'world yaw now {yaw_world:+.4f} rad '
                  f'(was authored radial {radial:+.4f})', flush=True)
            if abs(yaw_world) > 0.05:
                raise RuntimeError(
                    f'{path} world yaw {yaw_world:+.4f} not axis-aligned '
                    '-- parent chain carries an unexpected rotation; '
                    'investigate before saving')

        omni.usd.get_context().save_as_stage(args.stage)
        print(f'### saved to {args.stage}', flush=True)
        print('### DONE', flush=True)
        ok = True
        return 0
    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        return 1
    finally:
        simulation_app.close()
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
