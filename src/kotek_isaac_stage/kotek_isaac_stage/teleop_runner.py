#!/usr/bin/env python3
"""Isaac-side process for the kotek_teleop demo (SpaceMouse base + real Piper
leader arm + felt contact resistance). Runs standalone via run_isaac.sh, the
same way tier3_regression_test.py / pick_and_delivery_e2e_test.py do:

    KOTEK_WITH_ROS=1 ./run_isaac.sh \
      src/kotek_isaac_stage/kotek_isaac_stage/teleop_runner.py --gui

Three jobs, none of which touch the USD stage on disk:

1. Open kotek_scout_piper_demo.usd unmodified and start physics -- the
   /cmd_vel and /isaac_joint_commands OmniGraph subscriptions already in the
   stage (author_robot_graphs.py) are what actually move the robot; this
   script does not re-author them.

2. Override the Piper's per-joint max effort at RUNTIME (never into the USD)
   to TELEOP_ARM_MAX_EFFORTS / TELEOP_GRIPPER_MAX_EFFORT
   (physics_tuning.py). The stage's own ARM_MAX_FORCE (1e3 N*m, ~10x a real
   joint) is sized for the autonomous pick demo's gravity-holding needs and
   is wrong for teleop: at that authority the arm doesn't stall against an
   obstacle, it overpowers it, which makes any contact-force -> leader-torque
   mapping meaningless. This override does not touch inspect_stage.py's
   --assert-demo-ready checks (they inspect the file on disk).

3. Publish /isaac/arm_contact (sensor_msgs/JointState) at the physics rate:
   position/velocity = the sim arm's actual state, effort = an estimate of
   EXTERNAL joint torque (tau_ext = projected_joint_forces -
   gravity_compensation_forces), zeroed per-joint below CONTACT_TORQUE_MIN_NM
   so a subscriber never has to separately gate on a contact flag -- a
   nonzero reading already means "in contact". This is the "prefer
   get_dof_projected_joint_forces()" path from the plan; a per-link
   isaacsim.sensors.experimental.physics.ContactSensor is the documented
   fallback if this proves too noisy in practice (not implemented here).

Step loop discipline matches tier3_regression_test.py's hard-won finding:
call simulation_app.update() every tick, never sim_ctx.step(render=False) --
the latter skips OmniGraph entirely, silently killing /cmd_vel,
/isaac_joint_commands and /isaac_joint_states.
"""
import argparse
import time

from isaacsim import SimulationApp

DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd')
PIPER_ROOT = '/World/piper/Geometry/base_link'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINTS = ['joint7', 'joint8']

# Below this per-joint torque estimate, treat the reading as tracking-error
# noise rather than genuine contact. Matches haply_teleoperation.py's
# limit_force=2.0 reference value (that's a Cartesian N, this is N*m per
# joint -- not directly comparable, but the same order-of-magnitude
# "ignore small readings" intent). Tune during Phase 2 verification.
CONTACT_TORQUE_MIN_NM = 0.5

CONTACT_TOPIC = '/isaac/arm_contact'
PUBLISH_EVERY_N_TICKS = 2  # ~60Hz at a 120Hz physics stage -- see stage's physxScene:timeStepsPerSecond


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=DEMO_STAGE)
    parser.add_argument(
        '--gui', action='store_true', help='run with the Kit window visible instead of headless')
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': not args.gui})

    try:
        from isaacsim.core.api import World
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.utils import extensions, stage as stage_utils
        import omni.usd

        import physics_tuning

        extensions.enable_extension('isaacsim.ros2.bridge')
        simulation_app.update()

        stage_utils.open_stage(args.stage)
        for _ in range(10):
            simulation_app.update()
        omni.usd.get_context().get_stage()
        print(f'### stage opened: {args.stage}', flush=True)

        world = World(stage_units_in_meters=1.0)
        world.reset()
        print('### physics playing', flush=True)

        arm = Articulation(PIPER_ROOT)
        dof_index = {n: i for i, n in enumerate(arm.dof_names)}
        missing = set(ARM_JOINTS + GRIPPER_JOINTS) - set(arm.dof_names)
        if missing:
            raise RuntimeError(
                f'stage is missing expected Piper joints: {missing}; '
                f'dof_names={arm.dof_names}')
        arm_idx = [dof_index[n] for n in ARM_JOINTS]
        gripper_idx = [dof_index[n] for n in GRIPPER_JOINTS]

        # -- override 2: runtime-only effort caps (see module docstring) --
        n_dof = len(arm.dof_names)
        max_efforts = [0.0] * n_dof
        for i, name in zip(arm_idx, ARM_JOINTS):
            max_efforts[i] = physics_tuning.TELEOP_ARM_MAX_EFFORTS[ARM_JOINTS.index(name)]
        for i in gripper_idx:
            max_efforts[i] = physics_tuning.TELEOP_GRIPPER_MAX_EFFORT
        arm.set_dof_max_efforts([max_efforts])
        print(f'### teleop max efforts applied: {dict(zip(arm.dof_names, max_efforts))}',
              flush=True)

        import rclpy
        from sensor_msgs.msg import JointState

        rclpy.init()
        node = rclpy.create_node('kotek_teleop_runner')
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        contact_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        contact_pub = node.create_publisher(JointState, CONTACT_TOPIC, contact_qos)
        print('### rclpy node + /isaac/arm_contact publisher ready', flush=True)

        def tick(n=1):
            for _ in range(n):
                simulation_app.update()
                rclpy.spin_once(node, timeout_sec=0.0)

        # DDS discovery warm-up -- see tier3_regression_test.py's identical
        # comment for why this needs real wall-clock time, not just ticks.
        for _ in range(40):
            tick(1)
            time.sleep(0.05)
        print(
            f'### discovery warm-up done: contact subscribers='
            f'{contact_pub.get_subscription_count()}', flush=True)

        tick_count = 0
        try:
            while simulation_app.is_running():
                tick(1)
                tick_count += 1
                if tick_count % PUBLISH_EVERY_N_TICKS != 0:
                    continue

                positions = arm.get_dof_positions().numpy()[0]
                velocities = arm.get_dof_velocities().numpy()[0]
                projected = arm.get_dof_projected_joint_forces().numpy()[0]
                gravity_comp = arm.get_dof_gravity_compensation_forces().numpy()[0]

                msg = JointState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = list(ARM_JOINTS)
                msg.position = [float(positions[i]) for i in arm_idx]
                msg.velocity = [float(velocities[i]) for i in arm_idx]
                effort = []
                for i in arm_idx:
                    tau_ext = float(projected[i] - gravity_comp[i])
                    effort.append(tau_ext if abs(tau_ext) >= CONTACT_TORQUE_MIN_NM else 0.0)
                msg.effort = effort
                contact_pub.publish(msg)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    finally:
        simulation_app.close()


if __name__ == '__main__':
    main()
