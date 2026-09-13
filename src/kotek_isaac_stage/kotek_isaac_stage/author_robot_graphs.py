#!/usr/bin/env python3
"""Authors clean OmniGraphs for the freshly-imported scout + piper (see
import_robots.py, which must be run first).

Unlike kotek_sensor_graph in build_demo_stage.py (authored via raw USD
prims under plain usd-core, which caused a uint-vs-bool attribute-type bug
caught in a real Isaac Sim run last session), this script runs INSIDE
Isaac Sim and uses the real omni.graph.core.Controller API, which assigns
correct attribute types itself -- eliminating that whole class of bug.

Two graphs, each modeled on a *verified-clean* reference pattern read
directly from the source stage this session (not guessed):
  - scout drive graph: the odometry/TF/clock chain reproduces the source
    stage's own scout_action_graph exactly (that half had zero dangling
    connections). The drive-command chain
    (ROS2SubscribeTwist -> DifferentialController -> IsaacArticulationController)
    is newly built with single connections only -- this is the chain that
    was corrupted in the source stage (8 pins with a stale second
    connection to a nonexistent "/World/limo_env/..." prim tree, causing
    DifferentialController to fall back to stale authored defaults
    linearVelocity=-1.6/angularVelocity=10.2 -- the reported backward-and-
    leaning bug). See the plan for the full diagnosis.
  - arm graph: reproduces the source stage's /World/piper/ActionGraph
    pattern exactly (that graph had zero dangling connections already).

Run via env_isaaclab's Python:
    python author_robot_graphs.py
"""
import argparse

from isaacsim import SimulationApp

ROBOT_USD = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_robot.usd')

SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
PIPER_BASE_LINK = '/World/piper/Geometry/base_link'
SCOUT_WHEEL_LINKS = [
    '/World/scout_mini/Geometry/base_link/front_left_wheel_link',
    '/World/scout_mini/Geometry/base_link/front_right_wheel_link',
    '/World/scout_mini/Geometry/base_link/rear_left_wheel_link',
    '/World/scout_mini/Geometry/base_link/rear_right_wheel_link',
]
# Joint names as authored by the URDF (confirmed by reading back the
# imported stage in the previous step, not assumed).
WHEEL_JOINTS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINTS = ['joint7', 'joint8']

# Authored by import_robots.py's add-lidar step (RangeSensorCreateLidar
# under SCOUT_BASE_LINK, child name 'lidar_link').
LIDAR_PRIM_PATH = SCOUT_BASE_LINK + '/lidar_link'

# Matches the real, working geometry read from the source stage.
WHEEL_RADIUS = 0.08
WHEEL_DISTANCE = 0.5
MAX_LINEAR_SPEED = 2.7
MAX_ANGULAR_SPEED = 1.0


def build_scout_drive_graph(og, graph_path, simulation_app, include_lidar=True):
    """include_lidar=False skips the lidar->/scan nodes entirely -- required
    for the minimal scout-only sandbox (kotek_scout_only.usd), which has no
    lidar prim at all. Confirmed directly: authoring IsaacReadLidarBeams
    wired to a lidarPrim path that does not resolve to a real prim does not
    just no-op or warn -- it crashes the whole Kit process with a native
    'terminate called without an active exception' inside
    World.reset()/play(), no Python traceback at all. Isolated by
    reproducing the same crash with the graph authored vs. surviving
    world.reset() cleanly with no graph authored at all on the identical
    stage.
    """
    keys = og.Controller.Keys
    # Split into two edit() calls: ConstructArray's per-element input pins
    # (input0..N) are dynamically typed from inputs:arrayType, and that
    # retyping doesn't take effect synchronously within the same edit()
    # call as the CONNECT that uses them (observed directly: connecting to
    # them before this update leaves them at the default 'path' type,
    # OmniGraphError). An app update() between the two calls gives the
    # node a chance to regenerate its typed ports first.
    lidar_nodes = [
        ('read_lidar', 'isaacsim.sensors.physx.IsaacReadLidarBeams'),
        ('pub_scan', 'isaacsim.ros2.bridge.ROS2PublishLaserScan'),
    ] if include_lidar else []
    lidar_values = [
        ('read_lidar.inputs:lidarPrim', [LIDAR_PRIM_PATH]),
        ('pub_scan.inputs:topicName', 'scan'),
        ('pub_scan.inputs:frameId', 'lidar_link'),
    ] if include_lidar else []

    (graph, nodes, _, _) = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('tick', 'omni.graph.action.OnPlaybackTick'),
                ('ros2_context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('sub_twist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                ('break_linear', 'omni.graph.nodes.BreakVector3'),
                ('break_angular', 'omni.graph.nodes.BreakVector3'),
                ('diff_ctrl', 'isaacsim.robot.wheeled_robots.DifferentialController'),
                ('idx_left', 'omni.graph.nodes.ArrayIndex'),
                ('idx_right', 'omni.graph.nodes.ArrayIndex'),
                # wheel_link's own world orientation is mirrored between
                # left/right sides (front_left orient=(0.707,-0.707,0,0)
                # vs front_right=(0.707,0.707,0,0), same pattern rear,
                # confirmed directly) -- a positive commanded joint
                # velocity on one side rolls it opposite to the other
                # side's convention. Negating the right-side wheel command
                # here (relative to the left, unchanged) is necessary so
                # both sides drive the SAME physical direction together --
                # see docs/pick_and_delivery_report.md section 1.7 for the
                # original diagnosis. What section 1.7 got wrong (found
                # and fixed later, section 4.6): the MAGNITUDE fix was
                # right (58-60cm/2s, matching the pure-rolling prediction
                # almost exactly) but nobody had checked the SIGN --
                # tier3_regression_test.py's "base moved forward" check
                # only asserted `moved > 0.15` (a magnitude), never that
                # displacement was in the commanded +x direction. Direct
                # position logging proved the base was moving in -x the
                # entire time under a commanded +linear.x -- i.e. driving
                # backward by exactly the "forward" amount. See
                # negate_linear below for the actual direction fix; this
                # node (negate_right) only ever needed to make left/right
                # agree with each other, not to pick which way was
                # "forward" -- that's a property of the OVERALL sign
                # applied to the linear component, orthogonal to this.
                ('negate_right', 'omni.graph.nodes.Multiply'),
                ('neg_one', 'omni.graph.nodes.ConstantDouble'),
                # Real fix for the direction bug above: negate the LINEAR
                # command before DifferentialController, mirroring
                # negate_angular below exactly. Confirmed by direct
                # derivation and empirical /cmd_vel testing: negating only
                # negate_right's output (this file's original section 1.7
                # fix) cannot independently correct the translation
                # (common-mode) component's sign without ALSO flipping the
                # rotation (differential-mode) component's sign, since
                # both share the same per-side JOINT output -- the two
                # components must be sign-corrected separately, before
                # DifferentialController combines them, exactly like
                # negate_angular already does for rotation.
                ('negate_linear', 'omni.graph.nodes.Multiply'),
                # The right-side negation above correctly fixes straight-
                # line driving (confirmed: 58.84cm/2s against target
                # ~60cm) but was found to invert the ROTATION direction --
                # commanding /cmd_vel angular.z=+0.5 (intended CCW)
                # produced real, measured CW rotation instead (confirmed
                # directly: -0.038rad over 3s where +1.5rad was expected).
                # Root cause: the same left/right mirroring that requires
                # negating the translational ("common") component also
                # inverts the sign of the rotational ("differential")
                # component, but in the OPPOSITE way -- verified via a
                # direct joint-velocity sweep outside the graph entirely
                # (commanding all 4 wheels the SAME sign produces
                # rotation, matching the intended direction only when that
                # sign matches -angular, not +angular). Negating the
                # angular command itself before it reaches
                # DifferentialController fixes this independently of the
                # right-side fix above (diff=0 for pure straight driving,
                # so negating it there is a no-op and doesn't disturb the
                # already-confirmed-correct translation behavior).
                ('negate_angular', 'omni.graph.nodes.Multiply'),
                ('name_fl', 'omni.graph.nodes.ConstantToken'),
                ('name_fr', 'omni.graph.nodes.ConstantToken'),
                ('name_rl', 'omni.graph.nodes.ConstantToken'),
                ('name_rr', 'omni.graph.nodes.ConstantToken'),
                ('names_array', 'omni.graph.nodes.ConstructArray'),
                ('vel_array', 'omni.graph.nodes.ConstructArray'),
                ('art_ctrl', 'isaacsim.core.nodes.IsaacArticulationController'),
                ('sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('pub_odom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('pub_raw_tf', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                ('pub_tf', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
                ('pub_clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ] + lidar_nodes,
            # input0 exists by default; input1-3 must be created explicitly
            # with an explicit type (arrayType alone does not retype them
            # in time for a same-call CONNECT -- confirmed by two failed
            # attempts: OmniGraphError "path attribute" persisted even
            # after arrayType + an app update() between two edit() calls).
            keys.CREATE_ATTRIBUTES: [
                ('names_array.inputs:input1', 'token'),
                ('names_array.inputs:input2', 'token'),
                ('names_array.inputs:input3', 'token'),
                ('vel_array.inputs:input1', 'double'),
                ('vel_array.inputs:input2', 'double'),
                ('vel_array.inputs:input3', 'double'),
            ],
            keys.SET_VALUES: [
                ('name_fl.inputs:value', WHEEL_JOINTS[0]),
                ('name_fr.inputs:value', WHEEL_JOINTS[1]),
                ('name_rl.inputs:value', WHEEL_JOINTS[2]),
                ('name_rr.inputs:value', WHEEL_JOINTS[3]),
                ('names_array.inputs:arraySize', 4),
                ('names_array.inputs:arrayType', 'token[]'),
                ('vel_array.inputs:arraySize', 4),
                ('vel_array.inputs:arrayType', 'double[]'),
                ('diff_ctrl.inputs:wheelRadius', WHEEL_RADIUS),
                ('diff_ctrl.inputs:wheelDistance', WHEEL_DISTANCE),
                ('diff_ctrl.inputs:maxLinearSpeed', MAX_LINEAR_SPEED),
                ('diff_ctrl.inputs:maxAngularSpeed', MAX_ANGULAR_SPEED),
                ('idx_left.inputs:index', 0),
                ('idx_right.inputs:index', 1),
                ('neg_one.inputs:value', -1.0),
                ('art_ctrl.inputs:targetPrim', [SCOUT_BASE_LINK]),
                ('odom.inputs:chassisPrim', [SCOUT_BASE_LINK]),
                ('pub_raw_tf.inputs:parentFrameId', 'odom'),
                ('pub_raw_tf.inputs:childFrameId', 'base_link'),
                ('pub_tf.inputs:parentPrim', [SCOUT_BASE_LINK]),
                ('pub_tf.inputs:targetPrims', SCOUT_WHEEL_LINKS),
            ] + lidar_values,
        },
    )
    simulation_app.update()

    # Short node names created in the first edit() call only resolve
    # within that same call's CONNECT list -- a second, separate edit()
    # call needs full prim paths (observed directly: short names here
    # raised "At least one end of the connection must be an attribute").
    def p(short_ref):
        node, attr = short_ref.split('.', 1)
        return f'{graph_path}/{node}.{attr}'

    connections = [
        # drive-command chain -- single connections throughout, unlike
        # the corrupted source (this is the fix).
        ('tick.outputs:tick', 'sub_twist.inputs:execIn'),
        ('ros2_context.outputs:context', 'sub_twist.inputs:context'),
        ('sub_twist.outputs:linearVelocity', 'break_linear.inputs:tuple'),
        ('sub_twist.outputs:angularVelocity', 'break_angular.inputs:tuple'),
        ('sub_twist.outputs:execOut', 'diff_ctrl.inputs:execIn'),
        ('break_linear.outputs:x', 'negate_linear.inputs:a'),
        ('neg_one.inputs:value', 'negate_linear.inputs:b'),
        ('negate_linear.outputs:product', 'diff_ctrl.inputs:linearVelocity'),
        ('break_angular.outputs:z', 'negate_angular.inputs:a'),
        ('neg_one.inputs:value', 'negate_angular.inputs:b'),
        ('negate_angular.outputs:product', 'diff_ctrl.inputs:angularVelocity'),
        ('diff_ctrl.outputs:velocityCommand', 'idx_left.inputs:array'),
        ('diff_ctrl.outputs:velocityCommand', 'idx_right.inputs:array'),
        ('name_fl.inputs:value', 'names_array.inputs:input0'),
        ('name_fr.inputs:value', 'names_array.inputs:input1'),
        ('name_rl.inputs:value', 'names_array.inputs:input2'),
        ('name_rr.inputs:value', 'names_array.inputs:input3'),
        # right-side wheel command (indices 1 and 3, front_right and
        # rear_right per WHEEL_JOINTS/names_array's ordering) is negated
        # before reaching the articulation controller -- see neg_one's
        # comment above for why.
        ('idx_right.outputs:value', 'negate_right.inputs:a'),
        ('neg_one.inputs:value', 'negate_right.inputs:b'),
        ('idx_left.outputs:value', 'vel_array.inputs:input0'),
        ('negate_right.outputs:product', 'vel_array.inputs:input1'),
        ('idx_left.outputs:value', 'vel_array.inputs:input2'),
        ('negate_right.outputs:product', 'vel_array.inputs:input3'),
        ('names_array.outputs:array', 'art_ctrl.inputs:jointNames'),
        ('vel_array.outputs:array', 'art_ctrl.inputs:velocityCommand'),
        # articulation controller ticks every frame directly off the
        # clock -- matches the verified-working pattern (it reads
        # whatever velocityCommand is currently set, rather than only
        # updating synchronously after the twist subscriber fires).
        ('tick.outputs:tick', 'art_ctrl.inputs:execIn'),

        # odometry / TF / clock chain -- reproduces the source stage's
        # own (already-clean) wiring exactly.
        ('tick.outputs:tick', 'odom.inputs:execIn'),
        ('tick.outputs:tick', 'pub_odom.inputs:execIn'),
        ('tick.outputs:tick', 'pub_raw_tf.inputs:execIn'),
        ('tick.outputs:tick', 'pub_tf.inputs:execIn'),
        ('tick.outputs:tick', 'pub_clock.inputs:execIn'),
        ('ros2_context.outputs:context', 'pub_odom.inputs:context'),
        ('ros2_context.outputs:context', 'pub_raw_tf.inputs:context'),
        ('ros2_context.outputs:context', 'pub_tf.inputs:context'),
        ('ros2_context.outputs:context', 'pub_clock.inputs:context'),
        ('sim_time.outputs:simulationTime', 'pub_odom.inputs:timeStamp'),
        ('sim_time.outputs:simulationTime', 'pub_raw_tf.inputs:timeStamp'),
        ('sim_time.outputs:simulationTime', 'pub_tf.inputs:timeStamp'),
        ('sim_time.outputs:simulationTime', 'pub_clock.inputs:timeStamp'),
        ('odom.outputs:position', 'pub_odom.inputs:position'),
        ('odom.outputs:orientation', 'pub_odom.inputs:orientation'),
        ('odom.outputs:linearVelocity', 'pub_odom.inputs:linearVelocity'),
        ('odom.outputs:angularVelocity', 'pub_odom.inputs:angularVelocity'),
        ('odom.outputs:position', 'pub_raw_tf.inputs:translation'),
        ('odom.outputs:orientation', 'pub_raw_tf.inputs:rotation'),
    ]

    if include_lidar:
        # lidar -> /scan, same tick-driven pattern as the rest of this graph.
        connections += [
            ('tick.outputs:tick', 'read_lidar.inputs:execIn'),
            ('read_lidar.outputs:execOut', 'pub_scan.inputs:execIn'),
            ('ros2_context.outputs:context', 'pub_scan.inputs:context'),
            ('sim_time.outputs:simulationTime', 'pub_scan.inputs:timeStamp'),
            ('read_lidar.outputs:azimuthRange', 'pub_scan.inputs:azimuthRange'),
            ('read_lidar.outputs:depthRange', 'pub_scan.inputs:depthRange'),
            ('read_lidar.outputs:horizontalFov', 'pub_scan.inputs:horizontalFov'),
            ('read_lidar.outputs:horizontalResolution', 'pub_scan.inputs:horizontalResolution'),
            ('read_lidar.outputs:intensitiesData', 'pub_scan.inputs:intensitiesData'),
            ('read_lidar.outputs:linearDepthData', 'pub_scan.inputs:linearDepthData'),
            ('read_lidar.outputs:numCols', 'pub_scan.inputs:numCols'),
            ('read_lidar.outputs:numRows', 'pub_scan.inputs:numRows'),
            ('read_lidar.outputs:rotationRate', 'pub_scan.inputs:rotationRate'),
        ]

    (graph, nodes, _, _) = og.Controller.edit(
        graph,
        {keys.CONNECT: [(p(src), p(dst)) for src, dst in connections]},
    )
    return graph, nodes


def build_piper_arm_graph(og, graph_path):
    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('tick', 'omni.graph.action.OnPlaybackTick'),
                ('ros2_context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('sub_joint_state', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                ('art_ctrl', 'isaacsim.core.nodes.IsaacArticulationController'),
                ('pub_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
            ],
            keys.SET_VALUES: [
                ('sub_joint_state.inputs:topicName', 'isaac_joint_commands'),
                ('pub_joint_state.inputs:topicName', 'isaac_joint_states'),
                ('art_ctrl.inputs:targetPrim', [PIPER_BASE_LINK]),
                ('pub_joint_state.inputs:targetPrim', [PIPER_BASE_LINK]),
            ],
            keys.CONNECT: [
                ('tick.outputs:tick', 'sub_joint_state.inputs:execIn'),
                ('ros2_context.outputs:context', 'sub_joint_state.inputs:context'),
                ('tick.outputs:tick', 'art_ctrl.inputs:execIn'),
                ('sub_joint_state.outputs:jointNames', 'art_ctrl.inputs:jointNames'),
                ('sub_joint_state.outputs:positionCommand', 'art_ctrl.inputs:positionCommand'),
                ('sub_joint_state.outputs:velocityCommand', 'art_ctrl.inputs:velocityCommand'),
                ('sub_joint_state.outputs:effortCommand', 'art_ctrl.inputs:effortCommand'),
                ('tick.outputs:tick', 'pub_joint_state.inputs:execIn'),
                ('ros2_context.outputs:context', 'pub_joint_state.inputs:context'),
                ('sim_time.outputs:simulationTime', 'pub_joint_state.inputs:timeStamp'),
            ],
        },
    )
    return graph, nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=ROBOT_USD)
    parser.add_argument(
        '--scout-only', action='store_true',
        help='Skip build_piper_arm_graph entirely -- for the minimal scout-only sandbox '
        '(kotek_scout_only.usd), which has no /World/piper prim at all. If not passed, '
        'this is still auto-detected from the opened stage.')
    parser.add_argument('--headless', action='store_true', default=True)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': args.headless})

    try:
        import omni.graph.core as og
        import omni.usd
        from isaacsim.core.utils import extensions
        from pxr import Sdf

        extensions.enable_extension('isaacsim.ros2.bridge')
        simulation_app.update()
        print('### ros2 bridge extension enabled', flush=True)

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        print('### stage opened', flush=True)

        stage = omni.usd.get_context().get_stage()
        has_piper = bool(stage.GetPrimAtPath(Sdf.Path('/World/piper')))
        scout_only = args.scout_only or not has_piper
        if scout_only:
            print(
                '### scout-only mode (no /World/piper in this stage) -- skipping '
                'build_piper_arm_graph', flush=True)

        build_scout_drive_graph(
            og, '/World/scout_mini/scout_drive_graph', simulation_app,
            include_lidar=not scout_only)
        print('### scout drive graph authored', flush=True)

        if not scout_only:
            build_piper_arm_graph(og, '/World/piper/piper_arm_graph')
            print('### piper arm graph authored', flush=True)

        simulation_app.update()
        omni.usd.get_context().save_as_stage(args.stage)
        print(f'### saved to {args.stage}', flush=True)
        print('### DONE', flush=True)

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close()


if __name__ == '__main__':
    main()
