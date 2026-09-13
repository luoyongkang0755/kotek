#!/usr/bin/env python3
"""Plays the wall-mount demo stage in a visible Isaac Sim window.

GUI twin of run_wall_stage.py (README section 7.2/7.3): same stage, same
extension set, same play loop -- the only difference is ``headless: False``,
so a viewport opens on the machine's desktop (display :1 on this host) and
the run can be watched live while the ROS side drives the arm.

This is for VISUALIZATION. The verified E2E path remains the headless
run_wall_stage.py -- this script differs from it exactly where rendering is
concerned and nowhere else, but every automated verification so far used the
headless one.

Usage (host, from a session that can reach the desktop display):
    DISPLAY=:1 ROS_DOMAIN_ID=<same as the container> KOTEK_WITH_ROS=1 \
        ./run_isaac.sh \
        src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage_gui.py

Prints "### physics playing" once /clock and /isaac_joint_states are live,
same as the headless runner. Exactly ONE Isaac process may run at a time
(README 7.4); starting this while the headless runner is up will corrupt
sim time for the whole stack.
"""
import sys

from isaacsim import SimulationApp

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')


def main():
    # NOTE: SimulationApp defaults headless=True (isaacsim.simulation_app,
    # 6.0.1) -- the key MUST be set explicitly False or no window is created
    # even though the GUI extensions load fine.
    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': False})
    try:
        import time

        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions

        extensions.enable_extension('isaacsim.ros2.bridge')
        extensions.enable_extension('omni.graph.scriptnode')
        simulation_app.update()

        omni.usd.get_context().open_stage(WALL_DEMO_STAGE)
        for _ in range(10):
            simulation_app.update()
        print(f'### stage opened: {WALL_DEMO_STAGE}', flush=True)

        # See run_wall_stage.py for why set_defaults=False + explicit
        # physics_dt: the World() default stomps the stage's authored
        # timeStepsPerSecond=120 back to 60 and disables stabilization, and
        # 120 Hz itself is both unreachable in real time on this host and a
        # freeze trigger for the magnet-damped boxes (physics_tuning.py's
        # EFFECTIVE_PHYSICS_RATE_HZ documents the measurements).
        import physics_tuning as pt
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / pt.EFFECTIVE_PHYSICS_RATE_HZ,
            rendering_dt=1.0 / 60.0,
            set_defaults=False)
        world.reset()
        print('### physics playing -- /clock and /isaac_joint_states are live; '
              'start the 45s warm-up now', flush=True)

        last_print = 0.0
        while True:
            simulation_app.update()
            now = time.time()
            if now - last_print > 60.0:
                print('### still playing (GUI)', flush=True)
                last_print = now
    finally:
        simulation_app.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
