#!/usr/bin/env python3
"""Authors the wall-mount demo overlay stage on top of the freshly-rebuilt
robot asset (kotek_scout_piper_robot.usd -- see import_robots.py +
author_robot_graphs.py, both of which must be run first, in that order).

This is a SIBLING overlay to build_demo_stage.py, not a change to it: the
two demos (pick-and-place onto a pedestal, wall-mount of 4 camera sensors)
describe different worlds on the same robot asset and are kept independent
so neither can break the other. Like build_demo_stage.py, this script pulls
the robot asset in as a SUBLAYER, never edited, and adds:

  1. Four 8x3.5x4cm camera-sensor objects at the four top corners of
     `/World/scout_mini/Geometry/base_link/Cube` -- the riser the user has
     already hand-added directly to kotek_scout_piper_robot.usd. That prim
     has NO code behind it anywhere in this repo (re-running
     import_robots.py destroys it); this script only ever REFERENCES it by
     path, exactly the way ~/Projects/kotek_sim/probe_multi_spawn.py does
     for the same reason.
  2. A static wall with four non-colliding 'guide' markers showing the
     mount targets.
  3. A single new OmniGraph publishing `world -> sensor_cam_N` (N=1..4)
     over ROS 2 TF, one node with targetPrims set to all four sensor
     paths -- the same ROS2PublishTransformTree node type
     build_demo_stage.py's kotek_sensor_graph uses, just with a
     multi-target relationship instead of a single one.
  4. Dome + distant lighting -- this stage tree carries ZERO UsdLux prims
     anywhere (confirmed directly against the composed stage), so headless
     camera captures come out solid black without this.

This script only needs `pxr` (usd-core) -- it does not need Isaac Sim
installed to run, exactly like build_demo_stage.py. Camera-publishing
OmniGraph nodes (IsaacCreateRenderProduct, ROS2CameraHelper) are NOT
authored here: those need dynamically-typed ports that only resolve inside
a live Isaac Sim process, so they're authored by the separate
author_camera_graphs.py (and the magnet graph by author_magnet_graph.py),
both run against this script's OUTPUT with a live Isaac Sim process, in
that order. See the wall-mount task plan for the full build sequence.
"""
import argparse
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import physics_tuning  # noqa: E402  (needs the path insert above)

DEFAULT_SOURCE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_robot.usd')

# Must match import_robots.py exactly -- read back from the real imported
# stage, not assumed. The user's hand-added riser cube is a child of this
# prim: '/World/scout_mini/Geometry/base_link/Cube'.
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
RISER_CUBE_PATH = f'{SCOUT_BASE_LINK}/Cube'

# --- riser cube geometry (measured directly against the composed stage,
# NOT assumed): child of base_link, local translate (0, 0, 0.1803016),
# scale (0.3, 0.3, 0.22) on a unit cube (half-extents 0.15/0.15/0.11 m in
# its own local frame). Top face is therefore at base_link-local
# z = 0.1803016 + 0.11 = 0.2903, footprint x,y in [-0.15, 0.15] centered on
# base_link's own x,y origin -- which is also the Piper arm's own mount
# point (ARM_MOUNT_OFFSET in import_robots.py). Sensors sitting on this
# cube are close to the arm's own shoulder; see the wall-mount task plan's
# "biggest risks" section for why the reachability of these corners must be
# verified with a live /piper/compute_ik sweep before being treated as
# final.
RISER_TOP_LOCAL_Z = 0.2903
RISER_HALF_EXTENT_XY = 0.15

# --- the 4 sensor-cam pickup poses (base_link-local frame) ---------------
# Corners of the riser's top face, inset so an AXIS-ALIGNED (unrotated)
# sensor box never overhangs the cube's edge regardless of which corner:
# inset = half the box's LONGER footprint dimension (8cm -> 0.04) plus a
# 1mm clearance, applied symmetrically to both x and y (conservative on the
# y side, where the box's actual half-extent is only 0.0175m, but this
# keeps the placement independent of any future in-plane rotation choice).
_SENSOR_LONG_HALF = physics_tuning.SENSOR_CAM_SIZE_XYZ[0] / 2.0
_CORNER_INSET = _SENSOR_LONG_HALF + 0.001
CORNER_XY = RISER_HALF_EXTENT_XY - _CORNER_INSET   # ~0.109 m
SENSOR_CAM_LOCAL_Z = RISER_TOP_LOCAL_Z + physics_tuning.SENSOR_CAM_SIZE_XYZ[2] / 2.0
SENSOR_CAM_CORNERS_LOCAL = [
    (+CORNER_XY, +CORNER_XY, SENSOR_CAM_LOCAL_Z),
    (+CORNER_XY, -CORNER_XY, SENSOR_CAM_LOCAL_Z),
    (-CORNER_XY, -CORNER_XY, SENSOR_CAM_LOCAL_Z),
    (-CORNER_XY, +CORNER_XY, SENSOR_CAM_LOCAL_Z),
]
SENSOR_CAM_PATHS = [f'{SCOUT_BASE_LINK}/sensor_cam_{i + 1}' for i in range(4)]

# --- the wall and its 4 mount targets (world frame) -----------------------
# The robot spawns at the sublayer's own default pose (world origin, facing
# +x) -- confirmed directly against the composed stage (/World/scout_mini is
# a bare Xform with zero authored xform ops) and left untouched here. An
# earlier version of this script added a translate op on /World/scout_mini
# to "park" it WALL_PARK_STANDOFF in front of the wall instead of moving the
# wall -- that broke the articulation outright under live physics ("Invalid
# PhysX transform detected for .../base_link", "Illegal BroadPhaseUpdateData
# - non-finite bounds", sensor free-falling instead of settling): something
# in the scout's physics setup (root_joint anchoring, or PhysX's cooked
# collision/transform cache for this referenced asset) assumes this prim
# stays at its authored origin. Positioning the WALL at the desired standoff
# from the robot's actual (origin) spawn achieves the identical relative
# geometry without touching the robot asset at all.
#
# WALL_FACE_X is therefore exactly the robot-to-wall standoff (robot x=0), a
# live /piper/compute_ik sweep -- with the wall AND the riser cube modeled as
# real MoveIt CollisionObjects, not just self-collision -- confirmed all 4
# riser-corner grasps and all 4 wall-target places are reachable at this
# exact standoff (see the wall-mount task's chat history for the full
# pitch/clearance data): grasp_pitch~1.2 rad off the riser corners,
# place_pitch~0.7 rad into the wall, both far from the pedestal demo's tuned
# 0.4 rad -- see wall_mount.yaml and piper_manipulator.hpp's Params comment.
WALL_FACE_X = 0.40           # m, world x of the wall's front (near) face == robot standoff (robot spawns at x=0)
WALL_TARGET_Z = 0.35         # m, world z of every mount target
WALL_TARGET_Y = [-0.15, -0.05, 0.05, 0.15]   # m, world y of each target
WALL_THICKNESS = 0.05
WALL_WIDTH = 1.0             # m, along y
WALL_HEIGHT = 1.0            # m, along z (from the ground up)

# Parking-brake anchor pose for the chassis (see add_parking_brake). The
# chassis SETTLES upward to exactly this base_link world z within the first
# ~5s of physics (measured 0.1811 on three independent live runs -- the
# authored spawn z is ~0.152 and wheel/suspension contact pushes it up), so
# the brake pins it at the settled equilibrium rather than the authored
# height, which would leave the wheels fighting 3cm of ground penetration
# forever. x/y/yaw stay at their authored spawn values (measured drift
# during settle: <1mm).
SCOUT_PARK_BASE_Z = 0.1811   # m, measured settled base_link world z


def add_parking_brake(stage: Usd.Stage):
    """World-anchored FixedJoint pinning the scout chassis at its parked pose.

    This demo never drives the base (it spawns parked at the wall standoff,
    by design), but a live 4-cycle E2E showed the chassis CREEPING ~0.12mm/s
    in world x for as long as the sim runs (wheel/ground sliding -- neither
    has a friction material bound, and the huge wheel-drive damping can't
    stop body-level sliding). Grasps survive that (targets are chassis-
    relative) but places die: by cycles 2-4 the release point had crept
    outside MAGNET_ATTRACT_RANGE (0.03m) of the world-fixed wall targets and
    sensors 2-4 free-fell (measured: base x=-0.058 by cycle 4, exactly one
    weld engaged).

    A FixedJoint between the world and an ARTICULATION link is unbreakable
    and un-releasable at runtime (the same PhysX behavior that made the old
    riser-hold joints a bug -- see the riser-hold NOTE in
    add_sensor_cam_objects) -- which is exactly what a parking brake wants.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, '/World/_scout_parking_brake')
    joint.CreateBody0Rel().SetTargets([Sdf.Path(SCOUT_BASE_LINK)])
    # No Body1 target: anchored to the world frame.
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, SCOUT_PARK_BASE_Z))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def add_sensor_cam_objects(stage: Usd.Stage):
    """The 4 graspable camera-sensor boxes, riding on the riser cube until
    picked. True 8x3.5x4cm boxes: CreateSizeAttr(1.0) + a non-uniform
    AddScaleOp is CORRECT here (unlike the original demo cube bug this
    convention normally guards against -- these objects really are that
    shape, not a cube being stretched to fake one).

    Each box is yaw-rotated (about its own center, via AddOrientOp between
    the translate and scale ops -- same op ordering probe_magnet_tuning.py
    already uses to teleport a sensor-cam with position+orientation+size) by
    exactly that corner's own approach_yaw = atan2(y, x). A live E2E run
    without this rotation reached CLOSE_GRIPPER successfully (contact
    detected) but the object slipped free during LIFT_OBJECT every time:
    computeGraspPose()'s opening_axis is at a fixed +90deg offset from
    approach_yaw, and an UNROTATED box's edges sit at 0/90deg -- so at every
    one of these 4 corners (approach_yaw = +/-45deg, +/-135deg, from the
    riser's symmetric corner layout) the gripper's opening axis lands ~45deg
    off the box's own edges. A parallel-jaw gripper closing 45deg off a
    rectangular box's faces pinches near two adjacent corners/edges instead
    of clamping two parallel faces -- registers contact fine, has no stable
    equilibrium under lift acceleration. Rotating the box by approach_yaw
    aligns its short (0.035m) face flat against the opening axis at that
    specific corner, restoring a normal flat-face parallel-jaw grasp."""
    material = physics_tuning.define_physics_material(
        stage, '/World/sensor_cam_material',
        physics_tuning.SENSOR_CAM_STATIC_FRICTION,
        physics_tuning.SENSOR_CAM_DYNAMIC_FRICTION,
        physics_tuning.SENSOR_CAM_RESTITUTION)

    prims = []
    sx, sy, sz = physics_tuning.SENSOR_CAM_SIZE_XYZ
    for i, (path, (x, y, z)) in enumerate(zip(SENSOR_CAM_PATHS, SENSOR_CAM_CORNERS_LOCAL)):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        yaw = math.atan2(y, x)
        half = yaw / 2.0
        quat = Gf.Quatf(math.cos(half), Gf.Vec3f(0.0, 0.0, math.sin(half)))
        cube.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        cube.AddOrientOp().Set(quat)
        cube.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(physics_tuning.SENSOR_CAM_MASS)
        UsdGeom.Gprim(cube).CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.2, 0.85)])
        physics_tuning.bind_physics_material(prim, material)
        physics_tuning.apply_graspable_rigid_body_tuning(
            prim,
            contact_offset=physics_tuning.SENSOR_CAM_CONTACT_OFFSET,
            rest_offset=physics_tuning.SENSOR_CAM_REST_OFFSET,
            max_linear_velocity=physics_tuning.SENSOR_CAM_MAX_LINEAR_VELOCITY,
            linear_damping=physics_tuning.SENSOR_CAM_LINEAR_DAMPING,
            angular_damping=physics_tuning.SENSOR_CAM_ANGULAR_DAMPING)

        # NOTE (2026-08-25): a _riser_hold_{i+1} FixedJoint used to be
        # authored here (box <-> base_link) to pin each box through the
        # chassis settling transient, released later by a time-gated
        # ScriptNode. REMOVED -- root cause of the unfixable LIFT_OBJECT
        # "slip": PhysX joints anchored to an ARTICULATION link ignore
        # breakForce AND every runtime release path (jointEnabled=False,
        # RemovePrim -- all three verified live), so the "released" boxes
        # stayed bolted to the chassis forever and the gripper's fingers were
        # pried off an immovable object on every lift. Measured relative
        # box-vs-chassis drift WITHOUT the joints is 0.1-0.2mm over the whole
        # settle transient (the SENSOR_CAM_*_DAMPING + friction tuning above
        # is sufficient on its own), so no replacement hold is needed.
        # inspect_stage.py now asserts these joints do NOT exist.
        prims.append(prim)
    return prims


def add_wall(stage: Usd.Stage):
    """Static collider wall plus 4 non-colliding 'guide' markers at the
    mount targets (same purpose='guide' pattern build_demo_stage.py's
    add_place_target_marker uses -- out of both the physics scene and
    beauty renders).

    Parent Xform ('/World/wall', translate only, RigidBodyAPI + kinematic)
    with a child Cube ('/World/wall/geom', the actual anisotropically-scaled
    collision/visual box) -- NOT a single Cube carrying both the scale and
    RigidBodyAPI. A live probe found PhysX reporting the magnet weld's
    UsdPhysics.FixedJoint as having "disjointed body transforms" when
    RigidBodyAPI sat directly on the scaled Cube: non-uniform scale
    (WALL_THICKNESS != WALL_WIDTH != WALL_HEIGHT here) on a rigid-body prim
    is a well-known PhysX limitation -- joint/inertia composition assumes a
    rigid or uniform-scale transform, and a thin-wide-tall wall is neither.
    physics_tuning.py's own SENSOR_SIZE comment already documents avoiding
    scale ops on physics-bearing prims for a related reason (tipping
    moment); this is the joint-math instance of the same rule. Keeping
    RigidBodyAPI on a plain, scale-free parent and pushing the anisotropic
    scale down onto a CollisionAPI-only child (a standard "compound rigid
    body" shape) gives the joint a well-defined, unscaled frame to anchor
    into while the child still renders/collides at the correct wall size.
    """
    wall_xf = UsdGeom.Xform.Define(stage, '/World/wall')
    center_x = WALL_FACE_X + WALL_THICKNESS / 2.0
    wall_xf.AddTranslateOp().Set(Gf.Vec3d(center_x, 0.0, WALL_HEIGHT / 2.0))
    # RigidBodyAPI + kinematicEnabled=True (not just CollisionAPI, and not
    # on the scaled child below): the magnet weld authors a
    # UsdPhysics.FixedJoint against this prim (author_magnet_graph.py). A
    # kinematic rigid body is the standard PhysX pattern for an immovable
    # object that still participates correctly in joints: it never moves
    # under simulation (kinematicEnabled=True), but gives the joint math a
    # real, scale-free body frame to work against.
    UsdPhysics.RigidBodyAPI.Apply(wall_xf.GetPrim())
    wall_xf.GetPrim().CreateAttribute(
        'physics:kinematicEnabled', Sdf.ValueTypeNames.Bool).Set(True)

    wall_geom = UsdGeom.Cube.Define(stage, '/World/wall/geom')
    wall_geom.CreateSizeAttr(1.0)
    wall_geom.AddScaleOp().Set(Gf.Vec3f(WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT))
    UsdPhysics.CollisionAPI.Apply(wall_geom.GetPrim())
    physics_tuning.apply_collision_contact_tuning(
        wall_geom.GetPrim(),
        physics_tuning.WALL_CONTACT_OFFSET,
        physics_tuning.WALL_REST_OFFSET)
    UsdGeom.Gprim(wall_geom).CreateDisplayColorAttr([Gf.Vec3f(0.75, 0.75, 0.72)])
    wall = wall_xf

    markers = []
    for i, y in enumerate(WALL_TARGET_Y):
        sphere = UsdGeom.Sphere.Define(stage, f'/World/wall_target_{i + 1}')
        sphere.CreateRadiusAttr(0.015)
        sphere.AddTranslateOp().Set(Gf.Vec3d(WALL_FACE_X, y, WALL_TARGET_Z))
        sphere.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
        markers.append(sphere.GetPrim())
    return wall.GetPrim(), markers


def add_lighting(stage: Usd.Stage):
    """Dome + distant light. Same rationale as
    ~/Projects/kotek_sim/scene.py:add_lighting (reference only, not
    imported -- kotek_sim is a separate, out-of-scope project): the robot
    asset carries no lights at all, so a headless capture of this stage
    would otherwise be solid black."""
    from pxr import UsdLux
    dome = UsdLux.DomeLight.Define(stage, '/World/Lights/dome')
    dome.CreateIntensityAttr(600.0)
    sun = UsdLux.DistantLight.Define(stage, '/World/Lights/sun')
    sun.CreateIntensityAttr(2000.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))
    return dome.GetPrim(), sun.GetPrim()


def _og_node(stage: Usd.Stage, path: str, node_type: str) -> Usd.Prim:
    prim = stage.DefinePrim(path, 'OmniGraphNode')
    prim.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(node_type)
    return prim


def add_sensor_tf_graph(stage: Usd.Stage):
    """One OmniGraph publishing `world -> sensor_cam_N` for all 4 sensors
    over ROS 2 TF, authored as raw OmniGraphNode prims exactly like
    build_demo_stage.py's kotek_sensor_graph (this script runs under plain
    usd-core, so it cannot use omni.graph.core.Controller). The only
    difference from that graph: `inputs:targetPrims` is a multi-target
    relationship here (all 4 sensor paths) instead of one."""
    graph_path = '/World/kotek_wall_sensor_graph'
    stage.DefinePrim(graph_path, 'OmniGraph')

    # uint, not bool -- see build_demo_stage.py's add_sensor_tf_graph for
    # why (a real, confirmed Isaac Sim 6.0.1 headless-load failure mode).
    tick = _og_node(stage, f'{graph_path}/on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    tick.CreateAttribute('outputs:tick', Sdf.ValueTypeNames.UInt)

    context = _og_node(stage, f'{graph_path}/ros2_context', 'isaacsim.ros2.bridge.ROS2Context')
    context.CreateAttribute('outputs:context', Sdf.ValueTypeNames.UInt64)
    # WITHOUT this the context ignores ROS_DOMAIN_ID and publishes on the DDS
    # default domain 0 -- found 2026-09-18: the wall-mount stack runs on
    # domain 77 and its /tf never carried sensor_cam_N frames (15 s listen,
    # zero samples) while the pick-and-delivery demo's Controller-authored
    # graphs (same node type, no explicit flag) defaulted to env-var use.
    use_env = context.CreateAttribute('inputs:useDomainIDEnvVar', Sdf.ValueTypeNames.Bool)
    use_env.Set(True)

    # Without this wired, every message carries timeStamp=0 forever -- see
    # build_demo_stage.py's add_sensor_tf_graph for the exact failure this
    # avoids (a consumer's TF lookup that never resolves).
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
    # Physics-tracked parentPrim (not a bare Xform) -- see
    # build_demo_stage.py's module docstring for why.
    publish_tf.CreateRelationship('inputs:parentPrim').SetTargets([Sdf.Path(SCOUT_BASE_LINK)])
    publish_tf.CreateRelationship('inputs:targetPrims').SetTargets(
        [Sdf.Path(p) for p in SENSOR_CAM_PATHS])
    return graph_path


def build(source: str, output: str):
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

    add_sensor_cam_objects(stage)
    add_wall(stage)
    add_parking_brake(stage)
    add_lighting(stage)
    add_sensor_tf_graph(stage)

    stage.GetRootLayer().Save()
    print(f'Wrote {output}')
    print(f'  sublayer: {rel_source}')
    print(f'  4 sensor-cams at riser corners (local xy +/-{CORNER_XY:.4f}, z={SENSOR_CAM_LOCAL_Z:.4f})')
    print(f'  wall face at x={WALL_FACE_X}, targets at y={WALL_TARGET_Y}, z={WALL_TARGET_Z}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default=DEFAULT_SOURCE, help='source stage to overlay')
    # Absolute default, matching import_robots.py/build_demo_stage.py's own
    # defaults -- see build_demo_stage.py's --output comment for the real
    # bug a relative default caused there.
    parser.add_argument(
        '--output',
        default='/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
        'kotek_scout_piper_wall_demo.usd')
    args = parser.parse_args()

    build(args.source, args.output)


if __name__ == '__main__':
    main()
