#!/usr/bin/env python3
"""Plays the wall-mount demo stage headlessly: open, press-Play, idle.

This is the Isaac-side "Process 2" of the wall-mount demo (README section 7)
-- the exact runner every verified E2E used. It exists because the demo's
ROS side is dead in the water unless ONE Isaac Sim process is actively
PLAYING this stage: Isaac publishes /clock (the whole stack runs on sim
time) and /isaac_joint_states, and without them MoveIt's trajectories are
accepted and then hang forever ("move_action result timed out", followed by
"Cannot push a new trajectory while another is being executed" -- at which
point the ROS stack itself needs a restart, not just a retry).

Usage (host):
    ROS_DOMAIN_ID=<same as the container> KOTEK_WITH_ROS=1 ./run_isaac.sh \
        src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage.py --duration 3000

Prints "### physics playing" once the stack's clock/joint topics are live --
that is the moment to start the container-side 45s warm-up countdown.
The GUI app (README 7.2) is the interactive alternative; this script is the
reliable, no-clicks-needed one.

Recording (--record-dir, --gui): captures the app viewport to PNG frames
for assembling into a demo video. This exists because the host migrated to a
Wayland GNOME session (2026-09-25): x11grab reads the X root window, which
under Xwayland-on-Wayland never carries composited pixels (recordings from
09-28 are all black), and ANY new replicator render product (isaacsim
Camera() / rep.create.render_product) shuts the app down on first render,
headless EGL AND windowed alike (measured, no traceback). The GUI app's own
viewport renders through the working path, and
omni.kit.viewport.utility.capture_viewport_to_file renders on demand --
so recording must run with --gui (a window still presents black to x11grab,
but the captured frames carry real pixels).
"""
import argparse
import sys

from isaacsim import SimulationApp

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    parser.add_argument(
        '--duration', type=float, default=3000.0,
        help='wall-clock seconds to keep the sim playing (default 3000)')
    parser.add_argument(
        '--record-dir', default=None,
        help='if set, capture the app viewport to this dir as PNG frames '
             '(--record-fps) for assembling into a demo video. Requires '
             '--gui on this host (see module docstring).')
    parser.add_argument(
        '--record-fps', type=float, default=15.0,
        help='capture rate in frames per rendering second (default 15)')
    parser.add_argument(
        '--gui', action='store_true',
        help='run windowed (needs DISPLAY) instead of headless')
    args = parser.parse_args()
    print(f'### args: duration={args.duration} record_dir={args.record_dir} '
          f'record_fps={args.record_fps} gui={args.gui}', flush=True)

    simulation_app = SimulationApp(
        {'renderer': 'RayTracedLighting', 'headless': not args.gui})
    try:
        import time

        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions

        extensions.enable_extension('isaacsim.ros2.bridge')
        extensions.enable_extension('omni.graph.scriptnode')
        simulation_app.update()

        omni.usd.get_context().open_stage(args.stage)
        for _ in range(10):
            simulation_app.update()
        print(f'### stage opened: {args.stage}', flush=True)

        # World() with the default set_defaults=True stomps the stage-authored
        # physics scene back to Isaac defaults: PhysicsContext.__init__ runs
        # set_physics_dt(1/60) AND enable_stabilization(False) on the open
        # stage, overriding the physxScene:timeStepsPerSecond=120 and
        # enableStabilization=True baked into this USD by import_robots.py's
        # apply_physx_scene_tuning (observed live: /clock at ~55-60 Hz with the
        # stage on disk carrying 120). set_defaults=False keeps the stage as
        # the single source of truth for everything EXCEPT the step rate,
        # which the runners deliberately pin to EFFECTIVE_PHYSICS_RATE_HZ
        # (60 on this host: 120 Hz is unreachable in real time AND freezes
        # the magnet-damped sensor boxes -- physics_tuning.py documents both
        # measurements); it also keeps SimulationManager's dt in sync for
        # anything that queries it.
        import physics_tuning as pt
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / pt.EFFECTIVE_PHYSICS_RATE_HZ,
            rendering_dt=1.0 / 60.0,
            set_defaults=False)
        world.reset()
        print('### physics playing -- /clock and /isaac_joint_states are live; '
              'start the 45s warm-up now', flush=True)

        capture_vp = None
        capture_every = 1
        if args.record_dir:
            import os

            import omni.kit.viewport.utility as vp_util

            os.makedirs(args.record_dir, exist_ok=True)
            capture_vp = vp_util.get_active_viewport()
            if capture_vp is None:
                raise RuntimeError(
                    'no active viewport to capture from (did you forget --gui?)')
            capture_every = max(1, int(round(60.0 / args.record_fps)))
            print(f'### recording viewport to {args.record_dir} every '
                  f'{capture_every} ticks ({args.record_fps} fps)', flush=True)

        deadline = time.time() + args.duration
        last_print = 0.0
        tick = 0
        while time.time() < deadline:
            simulation_app.update()
            tick += 1
            if capture_vp is not None and tick % capture_every == 0:
                vp_util.capture_viewport_to_file(
                    capture_vp, os.path.join(
                        args.record_dir, f'frame_{tick:06d}.png'))
            now = time.time()
            if now - last_print > 30.0:
                print(f'### still playing, {deadline - now:.0f}s left', flush=True)
                last_print = now
        if capture_vp is not None:
            # capture requests flush one rendered frame later
            for _ in range(5):
                simulation_app.update()
        print('### duration elapsed, exiting', flush=True)
    finally:
        simulation_app.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
