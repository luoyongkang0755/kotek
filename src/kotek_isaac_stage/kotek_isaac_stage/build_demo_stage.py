#!/usr/bin/env python3
"""Authors the demo overlay stage on top of the freshly-rebuilt robot asset.

`kotek_scout_piper_robot.usd` (see import_robots.py + author_robot_graphs.py,
both of which must be run first, in that order) already contains both
robots -- imported fresh from URDF, physically mounted, and wired with
clean OmniGraphs -- plus a ground plane and physics scene. This script
pulls that in as a SUBLAYER, never edited, and adds:
  1. A graspable sensor object + static pedestal.
  2. A new OmniGraph publishing `world -> sensor` over ROS 2 TF, with
     parentPrim pointed at the scout's own (physics-tracked)
     base_link -- NOT a bare `/World` Xform. Using /World here previously
     produced a real, observed "Failed to create simulation view" error in
     a bounded Isaac Sim run (the working odometry/TF pattern always uses
     a physics-tracked parentPrim); this version avoids the issue by
     construction instead of by workaround.

This is the ONLY overlay layer needed now: the drive-command corruption
that motivated the whole rebuild (see the plan), the articulation-merge
fix, and the joint drive gains are all handled upstream, at the source, by
import_robots.py / author_robot_graphs.py -- not patched here after the
fact.

This script only needs `pxr` (usd-core) -- it does not need Isaac Sim
installed to run. See kotek_ws/README.md for the exact command.
"""
import argparse
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import physics_tuning  # noqa: E402  (needs the path insert above)

DEFAULT_SOURCE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_robot.usd')

# Must match import_robots.py / author_robot_graphs.py exactly -- these
# are read back from the real imported stage, not assumed.
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'


def add_sensor_object(stage: Usd.Stage, x: float, y: float, sensor_height: float):
    """Adds a graspable rigid-body sensor cube on a static pedestal.

    sensor_height is the CENTER height of the sensor cube; the pedestal
    fills the space from the ground to the bottom of the cube.

    Pedestal radius: was 0.08, an arbitrary cosmetic choice never checked
    against the robot's own approach geometry. Combined with the old
    `pregrasp_standoff=0.40` (coordinator.yaml), the parked robot's chassis
    front edge (0.312m ahead of base_link, measured directly from the
    composed stage) came within 8mm of the pedestal -- a live test
    confirmed the robot actually driving into it (see
    docs/pick_and_delivery_report.md section 4.12). Shrunk to 0.05 (still
    comfortably supports the 0.05x0.05m sensor cube footprint) as part of
    that fix, alongside raising `pregrasp_standoff` to 0.48.
    """
    size = physics_tuning.SENSOR_SIZE
    pedestal_height = max(sensor_height - size / 2.0, 0.02)

    material = physics_tuning.define_physics_material(
        stage, '/World/sensor_material',
        physics_tuning.SENSOR_STATIC_FRICTION,
        physics_tuning.SENSOR_DYNAMIC_FRICTION,
        physics_tuning.SENSOR_RESTITUTION)

    pedestal = UsdGeom.Cylinder.Define(stage, '/World/sensor_pedestal')
    pedestal.CreateRadiusAttr(0.05)
    pedestal.CreateHeightAttr(pedestal_height)
    pedestal.CreateAxisAttr('Z')
    pedestal.AddTranslateOp().Set(Gf.Vec3d(x, y, pedestal_height / 2.0))
    UsdPhysics.CollisionAPI.Apply(pedestal.GetPrim())
    # The pedestal is what the object rests on and is pulled off of, so it
    # gets the same friction as the object rather than PhysX's default 0.5.
    physics_tuning.bind_physics_material(pedestal.GetPrim(), material)

    # A TRUE cube, sized via CreateSizeAttr with NO scale op. This was
    # previously a size-1.0 cube with a non-uniform
    # AddScaleOp(0.05, 0.05, 0.08) -- a scaled analytic collider, gripped on
    # its narrow face with its long axis vertical, which maximised the
    # tipping moment about any off-centre finger contact. hsr_sim (the
    # working reference on this host) deliberately sizes its graspable
    # object with CreateSizeAttr and uses AddScaleOp only on static tables
    # (hsr_sim/scene.py:128-150).
    sensor = UsdGeom.Cube.Define(stage, '/World/sensor')
    sensor.CreateSizeAttr(size)
    sensor.AddTranslateOp().Set(Gf.Vec3d(x, y, sensor_height))
    UsdPhysics.RigidBodyAPI.Apply(sensor.GetPrim())
    UsdPhysics.CollisionAPI.Apply(sensor.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(sensor.GetPrim())
    mass_api.CreateMassAttr(physics_tuning.SENSOR_MASS)
    physics_tuning.bind_physics_material(sensor.GetPrim(), material)
    # Until now this prim carried UsdPhysics.RigidBodyAPI + CollisionAPI and
    # NOTHING else -- verified against the binary stage, `strings` yielded
    # zero physx* tokens on it. So maxDepenetrationVelocity, the solver
    # iteration counts and the sleep/stabilization thresholds were all at
    # PhysX defaults, and contactOffset defaulted to 0.02 m: 40% of the
    # object's own width. See physics_tuning's module docstring.
    physics_tuning.apply_graspable_rigid_body_tuning(sensor.GetPrim())

    return sensor.GetPrim()


def add_obstacles(stage: Usd.Stage):
    """Static collision boxes placed near, but not blocking, the corridors
    the robot drives: spawn (0,0) -> pregrasp near the sensor (1.5, 0.4,
    standoff 0.40 -- see coordinator.yaml), and pickup -> the delivery
    waypoint (0.3, -1.2, standoff 0.35 -- see task_coordinator's
    delivery_x/y params, added alongside the FSM's delivery leg).

    The original placements here (`obstacle_pickup_leg` at (0.70, 0.15),
    `obstacle_delivery_leg` at (0.55, -0.55)) were intended to sit "a bit
    to one side" of each straight-line corridor per this docstring's
    earlier claim, but were never actually checked against the real goal
    poses -- direct calculation (during the live E2E investigation, see
    docs/pick_and_delivery_report.md section 4.8) found BOTH sat almost
    exactly ON their corridor's straight line (perpendicular clearance
    -3.6cm and -6.0cm respectively -- i.e. inside the 30cm-wide box's own
    footprint), which a live run confirmed: the reactive (non-path-
    planning) obstacle-avoidance layer correctly refused to push through
    an obstacle directly ahead, suppressing forward /cmd_vel to near-zero
    and triggering the progress watchdog. Repositioned once already (see
    section 4.8/4.9) to sit clear of each corridor's straight line -- but
    that reposition only subtracted the OBSTACLE's own half-width from the
    centerline distance, never the ROBOT's, so it placed each box's edge a
    real but thin distance from the path a zero-width point would follow,
    with no margin at all for the robot's actual footprint. A live run
    confirmed the consequence directly: driving to the pickup position, a
    wheel physically struck `obstacle_pickup_leg`.

    Reworked below to compute clearance from real geometry, not
    approximation:
      - `ROBOT_HALF_WIDTH = 0.2883m` = the wheel joint's own lateral offset
        from base_link's centerline (0.2082515m, read directly from all 4
        wheel joints in scout_mini.urdf -- identical on every wheel) plus
        the wheel collision cylinder's own half-thickness (0.08m, half of
        `WHEEL_COLLISION_HEIGHT` in import_scout_only.py/import_robots.py)
        -- i.e. the true distance from the chassis centerline to a wheel's
        outer face, the widest point on the robot.
      - `PATH_TRACKING_MARGIN = 0.20m`: a buffer for how far the real
        driven path can bow off the idealized straight-line corridor.
        `SimpleBaseController::step()`'s own `DRIVE` phase tolerates
        heading error up to `heading_tolerance * 2 = 0.30rad` before
        reverting to `TURN_TO_HEADING` (see simple_base_controller.cpp),
        so the actual path is not guaranteed to hug the straight line
        between start and goal as tightly as a zero-tolerance point-mass
        assumption would suggest.
      - Required obstacle-center-to-centerline clearance = obstacle
        half-width (0.15m) + `ROBOT_HALF_WIDTH` + `PATH_TRACKING_MARGIN`.

    Recomputed directly (scratchpad/compute_obstacle_clearance.py, reusing
    the exact same `bearingTo`/`computeBaseGoal` formulas as
    pregrasp_geometry.hpp, against the real spawn/pregrasp/delivery goal
    poses): `obstacle_pickup_leg`'s old position had only a 12.7cm
    wheel-to-obstacle-edge margin (perpendicular clearance to centerline
    was 0.5649m against a required 0.6383m) -- consistent with, and now
    understood to be the direct cause of, the observed wheel strike.
    `obstacle_delivery_leg` already had a real 22.8cm margin (perpendicular
    clearance 0.6664m) and needed no change. Repositioned
    `obstacle_pickup_leg` to (0.521, 0.820) -- same ~0.71m progress along
    the corridor as before (still requires the reactive avoidance layer to
    react, still within lidar range), perpendicular clearance now 0.66m,
    giving a 22.0cm wheel-to-obstacle-edge margin, matching the delivery
    leg's own already-safe margin.

    Height raised from 0.40 to 0.70: `import_robots.py`'s `LIDAR_LOCAL_POS`
    was separately raised (world z ~0.53) to stop the lidar self-detecting
    the Piper arm's own mount structure (see
    docs/pick_and_delivery_report.md section 4.9) -- confirmed directly
    that this made the original 0.40-tall boxes invisible to the lidar
    entirely (a robot placed 0.8m from one, facing it, read `range_max`
    with nothing detected). 0.70 clears the new scan height with real
    margin (~0.17m), re-verified the same way (a robot placed 0.8m away
    now reads back a `min_range` matching that real distance).
    """
    boxes = [
        # (name, x, y, width (full, both x and y), height)
        ('obstacle_pickup_leg', 0.521, 0.820, 0.30, 0.70),
        ('obstacle_delivery_leg', 1.077, -0.836, 0.30, 0.70),
    ]
    for name, x, y, width, height in boxes:
        # UsdGeom.Cube's size=1.0 spans exactly [-0.5, 0.5] per axis, so a
        # scale op of (width, width, height) produces a box of exactly
        # those final dimensions -- same convention already used for the
        # sensor cube above.
        box = UsdGeom.Cube.Define(stage, f'/World/{name}')
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(x, y, height / 2.0))
        box.AddScaleOp().Set(Gf.Vec3f(width, width, height))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())


def _og_node(stage: Usd.Stage, path: str, node_type: str) -> Usd.Prim:
    prim = stage.DefinePrim(path, 'OmniGraphNode')
    prim.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(node_type)
    return prim


def add_sensor_tf_graph(stage: Usd.Stage):
    """New OmniGraph publishing `world -> sensor` over ROS 2 TF."""
    graph_path = '/World/kotek_sensor_graph'
    stage.DefinePrim(graph_path, 'OmniGraph')

    # uint, not bool -- verified against a real Isaac Sim 6.0.1 headless
    # load (using Bool loaded the stage but omni.graph logged
    # createInputsAndOutputs() type-mismatch errors and dropped the
    # connections).
    tick = _og_node(stage, f'{graph_path}/on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    tick.CreateAttribute('outputs:tick', Sdf.ValueTypeNames.UInt)

    context = _og_node(stage, f'{graph_path}/ros2_context', 'isaacsim.ros2.bridge.ROS2Context')
    context.CreateAttribute('outputs:context', Sdf.ValueTypeNames.UInt64)

    # Without this, inputs:timeStamp is never connected and every message
    # this graph publishes carries the attribute's uninitialized default
    # (0) forever -- confirmed directly in a real end-to-end run: a
    # consumer's TF lookup failed indefinitely with "the latest data is at
    # time 0.000000" even after tens of real seconds of the stage playing,
    # while the scout drive graph's own TF (which DOES wire
    # IsaacReadSimulationTime -> inputs:timeStamp, see
    # author_robot_graphs.py) advanced normally over the same window.
    sim_time = _og_node(
        stage, f'{graph_path}/isaac_read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    sim_time.CreateAttribute('outputs:simulationTime', Sdf.ValueTypeNames.Double)

    publish_tf = _og_node(
        stage, f'{graph_path}/ros2_publish_transform_tree',
        'isaacsim.ros2.bridge.ROS2PublishTransformTree')
    publish_tf.CreateAttribute('inputs:execIn', Sdf.ValueTypeNames.UInt).AddConnection(
        Sdf.Path(f'{graph_path}/on_playback_tick.outputs:tick'))
    publish_tf.CreateAttribute('inputs:context', Sdf.ValueTypeNames.UInt64).AddConnection(
        Sdf.Path(f'{graph_path}/ros2_context.outputs:context'))
    publish_tf.CreateAttribute('inputs:timeStamp', Sdf.ValueTypeNames.Double).AddConnection(
        Sdf.Path(f'{graph_path}/isaac_read_simulation_time.outputs:simulationTime'))
    # Physics-tracked parentPrim (not a bare Xform) -- see module docstring.
    publish_tf.CreateRelationship('inputs:parentPrim').SetTargets([Sdf.Path(SCOUT_BASE_LINK)])
    publish_tf.CreateRelationship('inputs:targetPrims').SetTargets([Sdf.Path('/World/sensor')])


def build(source: str, output: str, sensor_x: float, sensor_y: float, sensor_height: float):
    if not os.path.isfile(source):
        raise FileNotFoundError(
            f'source stage not found: {source} -- run import_robots.py and '
            'author_robot_graphs.py first')

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    stage = Usd.Stage.CreateNew(output)
    stage.SetMetadata('upAxis', 'Z')
    stage.SetMetadata('metersPerUnit', 1.0)

    rel_source = os.path.relpath(os.path.abspath(source), os.path.dirname(os.path.abspath(output)))
    stage.GetRootLayer().subLayerPaths.append(rel_source)
    stage.SetDefaultPrim(stage.DefinePrim('/World', 'Xform'))

    add_sensor_object(stage, sensor_x, sensor_y, sensor_height)
    add_sensor_tf_graph(stage)
    add_obstacles(stage)

    stage.GetRootLayer().Save()
    print(f'Wrote {output}')
    print(f'  sublayer: {rel_source}')
    print(f'  sensor at ({sensor_x}, {sensor_y}, {sensor_height})')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default=DEFAULT_SOURCE, help='source stage to overlay')
    # Absolute, matching import_robots.py/import_scout_only.py's own
    # defaults -- a relative default here previously caused a real bug: a
    # rebuild run from this script's own directory (kotek_isaac_stage/
    # kotek_isaac_stage/, as the project's own README instructs) silently
    # wrote to kotek_isaac_stage/kotek_isaac_stage/usd/... instead of the
    # canonical kotek_isaac_stage/usd/... every other script/test reads,
    # so several obstacle-repositioning edits appeared to have "no effect"
    # -- they were real, just landing in a file nothing else looked at.
    parser.add_argument(
        '--output',
        default='/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
        'kotek_scout_piper_demo.usd')
    parser.add_argument('--sensor-x', type=float, default=1.5)
    parser.add_argument('--sensor-y', type=float, default=0.4)
    parser.add_argument(
        '--sensor-height', type=float, default=0.30,
        help='center height (m) of the sensor cube')
    args = parser.parse_args()

    build(args.source, args.output, args.sensor_x, args.sensor_y, args.sensor_height)


if __name__ == '__main__':
    main()
