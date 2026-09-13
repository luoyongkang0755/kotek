#!/usr/bin/env python3
"""Real Isaac Sim regression test for the MINIMAL scout-only sandbox
(kotek_scout_only.usd -- see import_scout_only.py). Mirrors
tier3_regression_test.py's Test 1 (zero-/cmd_vel drift) and Test 2
(commanded drive) exactly, including its fixed step-loop bug
(simulation_app.update(), not sim_ctx.step(render=False) -- see that
script's docstring for why). No Test 3: there is no arm in this sandbox.

Purpose: isolate whether Test 2's failure (wheels reach and hold the exact
commanded angular velocity but the chassis barely translates) is present
even in the smallest possible asset -- no Piper arm, no lidar, no
demo-stage sublayer, nothing but the scout and a ground plane -- as a much
cheaper, faster-iterating sandbox than the full composed stage.

Run via env_isaaclab's Python:
    python test_scout_only_drive.py           # headless (default)
    python test_scout_only_drive.py --gui      # visible Kit window
"""
import argparse
import sys
import time

from isaacsim import SimulationApp

SCOUT_ONLY_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_only.usd')
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'

MAX_UPDATES_PER_SECOND_WAIT = 600  # safety cap so a stalled sim can't hang forever


class Checker:
    def __init__(self):
        self.failures = []

    def check(self, condition, message):
        if condition:
            print(f'  [PASS] {message}', flush=True)
        else:
            print(f'  [FAIL] {message}', flush=True)
            self.failures.append(message)
        return bool(condition)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=SCOUT_ONLY_STAGE)
    parser.add_argument(
        '--gui', action='store_true', help='run with the Kit window visible instead of headless')
    args = parser.parse_args()

    simulation_app = SimulationApp(
        {'renderer': 'RayTracedLighting', 'headless': not args.gui})
    c = Checker()

    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions, stage as stage_utils
        import omni.usd
        from pxr import Gf, UsdGeom

        extensions.enable_extension('isaacsim.ros2.bridge')
        simulation_app.update()

        stage_utils.open_stage(args.stage)
        for _ in range(10):
            simulation_app.update()
        usd_stage = omni.usd.get_context().get_stage()
        print(f'### stage opened: {args.stage}', flush=True)

        world = World(stage_units_in_meters=1.0)
        world.reset()
        sim_ctx = world
        print('### physics playing', flush=True)

        import rclpy
        from geometry_msgs.msg import Twist

        rclpy.init()
        node = rclpy.create_node('test_scout_only_drive')
        cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        print('### rclpy node + pub ready', flush=True)

        def tick(n=1):
            for _ in range(n):
                simulation_app.update()
                rclpy.spin_once(node, timeout_sec=0.0)

        def tick_for_seconds(duration_s):
            target = sim_ctx.current_time + duration_s
            count = 0
            while sim_ctx.current_time < target and count < MAX_UPDATES_PER_SECOND_WAIT:
                tick(1)
                count += 1
            return count

        for _ in range(40):
            tick(1)
            time.sleep(0.05)
        print(
            f'### discovery warm-up done: cmd_vel subscribers='
            f'{cmd_vel_pub.get_subscription_count()}', flush=True)

        def base_pose():
            prim = usd_stage.GetPrimAtPath(SCOUT_BASE_LINK)
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            t = xf.ExtractTranslation()
            up = xf.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
            return Gf.Vec2d(t[0], t[1]), up

        def base_z():
            prim = usd_stage.GetPrimAtPath(SCOUT_BASE_LINK)
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return xf.ExtractTranslation()[2]

        z_before_settle = base_z()
        tick_for_seconds(2.0)
        z_after_settle = base_z()
        print(
            f'### settle check: base z {z_before_settle:.4f} -> {z_after_settle:.4f} '
            f'(sim_time={sim_ctx.current_time:.3f}s)', flush=True)
        c.check(sim_ctx.current_time > 0.01, 'physics is actually advancing (sim_time > 0)')

        # Direct tensor-API check too, since that's the authoritative source
        # used throughout the earlier investigation (not just USD readback).
        from isaacsim.core.prims import Articulation
        import numpy as np
        art = Articulation(SCOUT_BASE_LINK)
        art.initialize()
        print(f'### articulation dof names: {art.dof_names}', flush=True)

        # ---- Test 1: zero-cmd_vel regression ----
        print('== Test 1: no /cmd_vel publisher -> base must not move ==', flush=True)
        start_xy, start_up = base_pose()
        tick_for_seconds(3.0)
        end_xy, end_up = base_pose()
        drift = (end_xy - start_xy).GetLength()
        upright = start_up * end_up
        c.check(
            drift < 0.02,
            f'base XY drift with no /cmd_vel < 2cm over 3s (got {drift*100:.2f}cm)')
        c.check(
            upright > 0.995,
            f'base stayed upright (up-vector dot product) (got {upright:.4f})')

        # ---- Test 2: commanded drive ----
        print('== Test 2: /cmd_vel forward -> base moves forward, then stops ==', flush=True)
        pre_drive_xy, _ = base_pose()
        twist = Twist()
        twist.linear.x = 0.3
        drive_start = sim_ctx.current_time
        while sim_ctx.current_time - drive_start < 2.0:
            cmd_vel_pub.publish(twist)
            tick(1)
        driving_xy, _ = base_pose()
        delta_xy = driving_xy - pre_drive_xy
        moved = delta_xy.GetLength()
        wheel_vels = art.get_joint_velocities()
        print(f'### wheel joint velocities at end of drive window: {wheel_vels}', flush=True)
        c.check(moved > 0.15, f'base moved under command (got {moved*100:.2f}cm)')
        # Direction, not just magnitude -- see
        # docs/pick_and_delivery_report.md section 4.6.
        c.check(
            delta_xy[0] > 0.15,
            f'base moved in +x (forward, matching commanded +linear.x) '
            f'(got dx={delta_xy[0]*100:.2f}cm dy={delta_xy[1]*100:.2f}cm)')

        zero_twist = Twist()
        stop_start = sim_ctx.current_time
        while sim_ctx.current_time - stop_start < 1.5:
            cmd_vel_pub.publish(zero_twist)
            tick(1)
        tick_for_seconds(1.0)
        post_stop_xy, _ = base_pose()
        after_stop_drift = (post_stop_xy - driving_xy).GetLength()
        c.check(
            after_stop_drift < 0.05,
            f'base stopped after zero /cmd_vel (got {after_stop_drift*100:.2f}cm further drift)')

        sim_ctx.stop()
        node.destroy_node()
        rclpy.shutdown()

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        c.failures.append('unhandled exception -- see traceback above')
    finally:
        simulation_app.close()

    print()
    if c.failures:
        print(f'{len(c.failures)} check(s) FAILED:', flush=True)
        for f in c.failures:
            print(f'  - {f}', flush=True)
        sys.exit(1)
    print('All scout-only checks PASSED.', flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
