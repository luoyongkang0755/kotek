#!/usr/bin/env python3
"""End-to-end pick-and-delivery test: real Isaac Sim physics on one side,
the real kotek ROS 2 stack (task_coordinator, simple_base_controller,
piper_manipulator/MoveIt) on the other, talking over real cross-process
DDS (RMW_IMPLEMENTATION=rmw_cyclonedds_cpp on BOTH sides -- see
docs/pick_and_delivery_report.md for why not FastRTPS: confirmed directly
that FastRTPS discovery succeeds across this Isaac-Sim-host <->
docker-container boundary but message delivery never arrives; CycloneDDS
delivers correctly with no other changes).

This script is the Isaac-Sim side: it opens+plays the demo stage and ticks
continuously (pumping physics + OmniGraph + the ROS 2 bridge) for the
whole task duration, while subscribing to /task/state to know when to
stop. It does NOT call /task/start itself -- see
docs/pick_and_delivery_report.md for the exact two-process reproduction
command (this script + `ros2 service call /task/start ...` from the
docker container, started separately, matching the README's documented
GUI + docker-stack pairing pattern).

At the end it reads the sensor object's final world pose directly from
the live USD stage (ground truth, not a self-reported topic) and reports
where it ended up.

Run via env_isaaclab's Python (needs RMW_IMPLEMENTATION=rmw_cyclonedds_cpp,
NOT rmw_fastrtps_cpp -- see module docstring):
    python pick_and_delivery_e2e_test.py [--timeout 90]
"""
import argparse
from pathlib import Path
import sys
import time

from isaacsim import SimulationApp

# kotek_scout_piper_demo.usd, NOT demo2.usd. This test was the only script in
# the workspace pointing at demo2 -- a GUI-saved variant that build_demo_stage.py
# never regenerates and inspect_stage.py never validates. So every fix made to
# the demo stage silently missed the one test that exercises the full pipeline:
# a run against demo2 still logs "ScaleOrientation is not supported for rigid
# bodies, prim path: /World/sensor", i.e. it was still loading the old
# non-uniformly-scaled sensor cube long after that was fixed.
DEMO_STAGE = str(
    Path(__file__).resolve().parents[1] / 'usd' / 'kotek_scout_piper_demo.usd')
SENSOR_PATH = '/World/sensor'

# Must match coordinator.yaml's delivery_x/y/z exactly.
DELIVERY_X, DELIVERY_Y, DELIVERY_Z = 0.3, -1.2, 0.05
DELIVERY_XY_TOLERANCE = 0.5  # generous: this is about verifying release near the target, not precision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=DEMO_STAGE)
    parser.add_argument('--timeout', type=float, default=90.0, help='max wall-clock seconds to wait')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': not args.gui})
    failures = []

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
        print('### physics playing', flush=True)

        import rclpy
        from std_msgs.msg import String

        rclpy.init()
        node = rclpy.create_node('pick_and_delivery_e2e_test')
        state_history = []
        latest_state = {'value': None}

        def on_state(msg):
            latest_state['value'] = msg.data
            if not state_history or state_history[-1] != msg.data:
                state_history.append(msg.data)
                print(f'### /task/state -> {msg.data}', flush=True)

        node.create_subscription(String, '/task/state', on_state, 10)

        def tick(n=1):
            for _ in range(n):
                simulation_app.update()
                rclpy.spin_once(node, timeout_sec=0.0)

        print('### ticking and waiting for /task/state updates ...', flush=True)
        wall_deadline = time.time() + args.timeout
        while time.time() < wall_deadline:
            tick(1)
            if latest_state['value'] in ('DONE', 'FAILED'):
                break
            time.sleep(0.01)

        final_state = latest_state['value']
        print(f'### final /task/state: {final_state}', flush=True)
        print(f'### full state history: {state_history}', flush=True)

        reached_done = final_state == 'DONE'
        if not reached_done:
            failures.append(
                f'task did not reach DONE within {args.timeout}s (final state: {final_state}, '
                f'history: {state_history})')

        sensor_prim = usd_stage.GetPrimAtPath(SENSOR_PATH)
        if sensor_prim.IsValid():
            xf = UsdGeom.Xformable(sensor_prim).ComputeLocalToWorldTransform(0)
            pos = xf.ExtractTranslation()
            print(f'### sensor final world position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})',
                  flush=True)
            dist_to_delivery = ((pos[0] - DELIVERY_X) ** 2 + (pos[1] - DELIVERY_Y) ** 2) ** 0.5
            print(f'### distance from sensor to delivery target (xy): {dist_to_delivery:.3f}m',
                  flush=True)
            if reached_done and dist_to_delivery > DELIVERY_XY_TOLERANCE:
                failures.append(
                    f'task reported DONE but sensor is {dist_to_delivery:.3f}m from the delivery '
                    f'target ({DELIVERY_X}, {DELIVERY_Y}), tolerance {DELIVERY_XY_TOLERANCE}m')
        else:
            failures.append(f'{SENSOR_PATH} not found in the stage at test end')

        rclpy.shutdown()

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        failures.append('unhandled exception -- see traceback above')
    finally:
        simulation_app.close()

    print()
    if failures:
        print(f'{len(failures)} check(s) FAILED:', flush=True)
        for f in failures:
            print(f'  - {f}', flush=True)
        sys.exit(1)
    print('Pick-and-delivery end-to-end test PASSED.', flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
