#!/usr/bin/env python3
"""Captures one real /scan message from a live Isaac Sim physics run (real
lidar raycasts against the real obstacle geometry in the demo stage) to a
JSON file, for use by tier3_obstacle_test.py.

Why this exists: a genuine cross-process DDS interop limitation between
Isaac Sim's bundled ROS 2 stack and the system-installed ROS 2 Jazzy stack
in the kotek docker container (see docs/pick_and_delivery_report.md for
the full diagnosis -- confirmed directly with a minimal publisher/
subscriber reproduction: DDS discovery/matching succeeds across the
boundary, subscription_count reaches 1, but zero messages are ever
actually delivered) makes a live, streamed Isaac-Sim-to-docker-container
test currently infeasible. Capturing one real scan and replaying it
entirely within the docker container (where simple_base_controller_node
also runs) sidesteps that specific limitation while still exercising real,
physics-derived sensor data end to end through the real compiled node.

Run via env_isaaclab's Python (see README for env vars):
    python capture_real_scan.py [--output real_scan.json]
"""
import argparse
import json
import time

from isaacsim import SimulationApp

DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=DEMO_STAGE)
    parser.add_argument(
        '--output',
        default='/home/trs/kotek_ws/src/kotek_isaac_stage/kotek_isaac_stage/'
        'real_scan.json')
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': True})
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions, stage as stage_utils

        extensions.enable_extension('isaacsim.ros2.bridge')
        simulation_app.update()
        stage_utils.open_stage(args.stage)
        for _ in range(10):
            simulation_app.update()

        world = World(stage_units_in_meters=1.0)
        world.reset()

        import rclpy
        from sensor_msgs.msg import LaserScan
        rclpy.init()
        node = rclpy.create_node('capture_real_scan')
        captured = {}

        def on_scan(msg):
            captured['msg'] = msg

        node.create_subscription(LaserScan, '/scan', on_scan, 10)

        def tick(n=1):
            for _ in range(n):
                simulation_app.update()
                rclpy.spin_once(node, timeout_sec=0.0)

        for _ in range(60):
            tick(1)
            time.sleep(0.02)

        # Let the robot settle onto the ground and take one more real scan
        # after settling, matching the pose tier3_obstacle_test.py assumes.
        t0 = world.current_time
        while world.current_time - t0 < 2.0:
            tick(1)

        if 'msg' not in captured:
            raise RuntimeError('no /scan message received -- pipeline not producing lidar data')

        m = captured['msg']
        data = {
            'angle_min': m.angle_min,
            'angle_max': m.angle_max,
            'angle_increment': m.angle_increment,
            'time_increment': m.time_increment,
            'scan_time': m.scan_time,
            'range_min': m.range_min,
            'range_max': m.range_max,
            'ranges': list(m.ranges),
            'intensities': list(m.intensities),
        }
        with open(args.output, 'w') as f:
            json.dump(data, f)

        finite = [r for r in data['ranges'] if r == r and data['range_min'] <= r <= data['range_max']]
        print(f'### captured real /scan: {len(data["ranges"])} points, '
              f'{len(finite)} within range, min={min(finite) if finite else None}', flush=True)
        print(f'### wrote {args.output}', flush=True)

        rclpy.shutdown()
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()


if __name__ == '__main__':
    main()
