#!/usr/bin/env python3
"""Real Isaac Sim regression tests for the rebuilt robot (plan section
"Testing"). Promotes last session's scratch smoke-test script into a real,
repeatable test with pass/fail assertions.

1. Zero-cmd_vel regression test -- the direct test for the originally
   reported bug (robot drives backward and leans the instant Play is
   pressed, with no /cmd_vel publisher running at all): step physics with
   nothing publishing to /cmd_vel and assert the base barely moves and
   stays upright.
2. Commanded drive test: publish a real /cmd_vel Twist, assert the base
   actually displaces by roughly the right amount, then zero the command
   and assert it stops.
3. Arm command test: publish a JointState to /isaac_joint_commands and
   assert the arm joints converge to and hold the target, read back via
   /isaac_joint_states (the piper_arm_graph's own feedback topic).

ROOT CAUSE OF THE EARLIER FALSE "environment limitation" REPORT (fixed
here, not a real limitation): the step loop used to call
`sim_ctx.step(render=False)`. Reading SimulationContext.step()'s actual
source (isaacsim.core.api.simulation_context):

    if render:
        ...
        else:
            self._app.update()          # pumps OmniGraph + ROS 2 + everything
    else:
        if self.is_playing():
            self._physics_context._step(...)   # raw PhysX ONLY -- no app.update()

`render=False` calls PhysX directly and skips `self._app.update()`
entirely -- the thing that actually ticks OmniGraph's OnPlaybackTick chain
(DifferentialController, IsaacArticulationController,
IsaacReadSimulationTime, every ROS2Publish*/ROS2Subscribe* node). That is
why gravity/settling worked (pure PhysX, no graph involved) while
/clock, /isaac_joint_states, and commanded /cmd_vel motion never
appeared -- all downstream of OmniGraph nodes that were simply never
evaluated. Confirmed directly against the Kit log: the
"Failed to create simulation view: no active physics scene found" errors
only ever appeared before world.reset() finished (an expected early-
startup race) and after sim_ctx.stop() (the line immediately before that
final burst is literally "onStop: Cleared time samples" -- shutdown
cleanup). The entire real test window in between showed zero tensor
errors and one success line: "Created simulation views (engine: physx,
backend: numpy)". The view was valid and working throughout; the graph
just never ticked to use it.

Fix: drive every step with `simulation_app.update()` instead of
`sim_ctx.step(render=False)` -- the same pattern Isaac Sim's own
standalone-script documentation uses, and exactly what happens on every
rendered frame when a user presses Play in the GUI.

Run via env_isaaclab's Python (needs ROS_DISTRO/RMW_IMPLEMENTATION/
LD_LIBRARY_PATH set -- see README):
    python tier3_regression_test.py           # headless (default)
    python tier3_regression_test.py --gui      # visible Kit window
"""
import argparse
import sys
import time

from isaacsim import SimulationApp

DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd')
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Fallback only -- actual step counts are computed from measured
# sim_ctx.current_time, not assumed from this constant (World's default
# physics_dt is not guaranteed to be 1/60; asserting on assumed Hz was a
# real bug in the previous version of this script).
ASSUMED_PHYSICS_HZ = 60.0
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
    parser.add_argument('--stage', default=DEMO_STAGE)
    parser.add_argument(
        '--gui', action='store_true', help='run with the Kit window visible instead of headless')
    args = parser.parse_args()

    simulation_app = SimulationApp(
        {'renderer': 'RayTracedLighting', 'headless': not args.gui})
    c = Checker()
    core_bug_fixed = False

    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions, stage as stage_utils
        import omni.usd
        import omni.graph.core as og
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
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock

        rclpy.init()
        node = rclpy.create_node('tier3_regression_test')
        cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        joint_cmd_pub = node.create_publisher(JointState, '/isaac_joint_commands', 10)

        latest_joint_state = {}

        def on_joint_state(msg):
            for n, p in zip(msg.name, msg.position):
                latest_joint_state[n] = p

        node.create_subscription(JointState, '/isaac_joint_states', on_joint_state, 10)

        clock_count = {'n': 0}
        node.create_subscription(
            Clock, '/clock', lambda msg: clock_count.__setitem__('n', clock_count['n'] + 1), 10)
        print('### rclpy node + pub/sub ready', flush=True)

        # THE FIX: pump the real Kit app update loop (physics + OmniGraph +
        # ROS 2 bridge nodes + rclpy spin), not a physics-only step. This is
        # the direct fix for the false "environment limitation" reported
        # earlier -- see the module docstring for the evidence.
        def tick(n=1):
            for _ in range(n):
                simulation_app.update()
                rclpy.spin_once(node, timeout_sec=0.0)

        def tick_for_seconds(duration_s):
            """Ticks until sim_ctx.current_time has advanced by >= duration_s,
            measured for real rather than assumed from a fixed Hz."""
            target = sim_ctx.current_time + duration_s
            count = 0
            while sim_ctx.current_time < target and count < MAX_UPDATES_PER_SECOND_WAIT:
                tick(1)
                count += 1
            return count

        # DDS discovery needs real wall-clock time -- publishing immediately
        # after creating a publisher with zero delay can silently drop
        # messages before peers discover each other.
        for _ in range(40):
            tick(1)
            time.sleep(0.05)
        print(
            f'### discovery warm-up done: cmd_vel subscribers='
            f'{cmd_vel_pub.get_subscription_count()} '
            f'joint_cmd subscribers={joint_cmd_pub.get_subscription_count()} '
            f'/clock messages received so far={clock_count["n"]}', flush=True)

        # Diagnostic: is the graph node itself ticking/computing now?
        sim_time_attr = og.Controller.attribute(
            '/World/scout_mini/scout_drive_graph/sim_time.outputs:simulationTime')
        v1 = sim_time_attr.get()
        tick(5)
        v2 = sim_time_attr.get()
        print(f'### graph node diagnostic: sim_time {v1} -> {v2}', flush=True)
        c.check(v2 != v1, f'OmniGraph is actually ticking (sim_time changed: {v1} -> {v2})')

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

        # Let the robot settle onto the ground first (matches the plan's
        # "spawns above ground and drops" failure mode) before measuring
        # drift, so settling itself isn't mistaken for drive-bug drift.
        z_before_settle = base_z()
        tick_for_seconds(2.0)
        z_after_settle = base_z()
        print(
            f'### settle check: base z {z_before_settle:.4f} -> {z_after_settle:.4f} '
            f'(sim_time={sim_ctx.current_time:.3f}s)', flush=True)
        c.check(sim_ctx.current_time > 0.01, 'physics is actually advancing (sim_time > 0)')

        # ---- Test 1: zero-cmd_vel regression (the reported bug) ----
        print('== Test 1: no /cmd_vel publisher -> base must not move ==', flush=True)
        start_xy, start_up = base_pose()
        tick_for_seconds(3.0)
        end_xy, end_up = base_pose()
        drift = (end_xy - start_xy).GetLength()
        upright = start_up * end_up  # dot product; 1.0 == still perfectly upright
        core_bug_fixed = c.check(
            drift < 0.02,
            f'base XY drift with no /cmd_vel < 2cm over 3s (got {drift*100:.2f}cm)')
        core_bug_fixed &= c.check(
            upright > 0.995,
            f'base stayed upright (up-vector dot product) (got {upright:.4f})')

        # ---- Test 2: commanded drive ----
        print('== Test 2: /cmd_vel forward -> base moves forward, then stops ==', flush=True)
        pre_drive_xy, _ = base_pose()
        twist = Twist()
        twist.linear.x = 0.3
        drive_start = sim_ctx.current_time
        while sim_ctx.current_time - drive_start < 2.0:
            cmd_vel_pub.publish(twist)  # republish every tick -- no assumption of latching
            tick(1)
        driving_xy, _ = base_pose()
        delta_xy = driving_xy - pre_drive_xy
        moved = delta_xy.GetLength()
        c.check(moved > 0.15, f'base moved under command (got {moved*100:.2f}cm)')
        # Direction, not just magnitude -- a real bug this test previously
        # missed entirely (see docs/pick_and_delivery_report.md section
        # 4.6): the base was moving the right DISTANCE but in -x (backward)
        # under a commanded +linear.x, for the whole session, because only
        # magnitude was ever checked here.
        c.check(
            delta_xy[0] > 0.15,
            f'base moved in +x (forward, matching commanded +linear.x), not just some '
            f'direction (got dx={delta_xy[0]*100:.2f}cm dy={delta_xy[1]*100:.2f}cm)')

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

        # ---- Test 3: arm command ----
        print('== Test 3: arm joint command -> joints converge and hold ==', flush=True)
        target = [0.3, 0.4, -0.3, 0.2, 0.1, 0.0]
        msg = JointState()
        msg.name = ARM_JOINTS
        msg.position = target
        arm_start = sim_ctx.current_time
        while sim_ctx.current_time - arm_start < 3.0:
            joint_cmd_pub.publish(msg)  # republish every tick -- no assumption of latching
            tick(1)

        for name, tgt in zip(ARM_JOINTS, target):
            actual = latest_joint_state.get(name)
            c.check(
                actual is not None and abs(actual - tgt) < 0.05,
                f'{name} converged to {tgt:.3f} (got {actual})')

        # hold check: re-sample after more ticks without changing the command
        tick_for_seconds(1.0)
        for name, tgt in zip(ARM_JOINTS, target):
            actual = latest_joint_state.get(name)
            c.check(
                actual is not None and abs(actual - tgt) < 0.05,
                f'{name} still holding {tgt:.3f} (got {actual})')

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
    print(
        f'CORE BUG FIX (reported symptom: drives backward + leans with no /cmd_vel): '
        f'{"VERIFIED FIXED" if core_bug_fixed else "NOT VERIFIED"}', flush=True)
    if c.failures:
        print(f'{len(c.failures)} check(s) FAILED:', flush=True)
        for f in c.failures:
            print(f'  - {f}', flush=True)
        print(
            'NOTE: "base moved forward under command" is a known, deeply-investigated open '
            'issue (wheels reach and hold the exact commanded angular velocity but the chassis '
            'does not translate proportionally) -- see docs/pick_and_delivery_report.md for the '
            'full diagnostic trail of ruled-out causes.', flush=True)
        sys.exit(1)
    print('All Tier 3 regression checks PASSED.', flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
