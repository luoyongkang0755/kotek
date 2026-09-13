#!/usr/bin/env python3
"""Verifies a demo overlay stage produced by build_demo_stage.py OR
build_wall_stage.py (both sublayer kotek_scout_piper_robot.usd, produced by
import_robots.py + author_robot_graphs.py).

Runs under plain `usd-core` -- no Isaac Sim needed (plan section 13, Tier
1). This checks everything that is verifiable from the USD data itself:
prim structure, applied schemas, attribute values, OmniGraph connection
sanity, and that the imported robots still compose correctly. It cannot
verify PhysX runtime behavior or that OmniGraph nodes actually tick
correctly -- that needs Isaac Sim (Tier 3, see tier3_regression_test.py).

The two demos are independent siblings on the same robot asset (see the
wall-mount task plan): checks specific to one are skipped, not failed, when
that demo's own marker prim (/World/sensor for the pick-place demo,
/World/wall for the wall-mount demo) is absent -- this script auto-detects
which stage it was given rather than needing a flag.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sys

from pxr import Sdf, Usd

SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
PIPER_BASE_LINK = '/World/piper/Geometry/base_link'
SCOUT_ARM_JOINTS_PATH = '/World/scout_mini/Physics'
PIPER_JOINTS_PATH = '/World/piper/Physics'
WHEEL_JOINTS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']


class Checker:
    def __init__(self):
        self.failures = []

    def check(self, condition, message):
        if condition:
            print(f'  [PASS] {message}')
        else:
            print(f'  [FAIL] {message}')
            self.failures.append(message)
        return bool(condition)


def check_no_dangling_or_duplicate_connections(stage: Usd.Stage, c: Checker):
    """The exact bug class that motivated the whole rebuild (see the
    plan): the source stage's scout drive-command chain had 8 attribute
    pins each carrying a second, stale connection to a prim tree that did
    not exist anywhere in the stage ("/World/limo_env/.../scout_v2_09_29_usb/...").
    This check makes that regression impossible to reintroduce silently:
    every OmniGraphNode attribute connection must (a) be the only
    connection on that attribute, and (b) target a prim that actually
    exists in the composed stage.
    """
    all_paths = {str(p.GetPath()) for p in stage.TraverseAll()}
    duplicate_count = 0
    dangling = []
    for prim in stage.TraverseAll():
        if str(prim.GetTypeName()) != 'OmniGraphNode':
            continue
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not (name.startswith('inputs:') or name.startswith('outputs:')):
                continue
            conns = attr.GetConnections()
            if len(conns) > 1:
                duplicate_count += 1
                c.check(
                    False,
                    f'{prim.GetPath()}.{name} has {len(conns)} connections (expected <= 1): '
                    f'{[str(x) for x in conns]}')
            for conn in conns:
                target_prim_path = str(conn.GetPrimPath())
                if target_prim_path not in all_paths:
                    dangling.append(f'{prim.GetPath()}.{name} -> {conn} (prim does not exist)')
    if duplicate_count == 0:
        c.check(True, 'no OmniGraphNode attribute has more than one connection')
    if dangling:
        for d in dangling:
            c.check(False, d)
    else:
        c.check(True, 'no OmniGraphNode connection targets a nonexistent prim')


# Expected values, mirrored from physics_tuning.py. Imported lazily inside the
# function so inspect_stage.py keeps working as a standalone usd-core script
# even if that module is not importable for some reason -- the checks are then
# skipped loudly rather than crashing the whole inspection.
def check_physx_grasp_properties(stage: Usd.Stage, c: 'Checker') -> None:
    try:
        import physics_tuning as pt
    except Exception as exc:  # noqa: BLE001
        c.check(False, f'physics_tuning importable for the PhysX checks ({exc})')
        return

    def attr(prim, name):
        a = prim.GetAttribute(name) if prim else None
        return a.Get() if a and a.HasAuthoredValue() else None

    def has_api(prim, name):
        """Applied-schema test that works under PLAIN usd-core.

        Usd.Prim.GetAppliedSchemas() filters against the schema registry, and
        the PhysX schemas ship as a Kit extension (extscache/
        omni.usd.schema.physx-*) that plain usd-core does not have registered
        -- so it silently reports PhysxRigidBodyAPI etc. as absent even when
        they are authored right there in the file. Reading the raw apiSchemas
        list op instead is correct both here and inside Kit. (UsdPhysics
        schemas ARE registered, which is why the PhysicsRigidBodyAPI check
        above can use the normal call.)
        """
        if not prim:
            return False
        listop = prim.GetMetadata('apiSchemas')
        if listop is None:
            return False
        items = list(getattr(listop, 'explicitItems', []) or [])
        items += list(getattr(listop, 'prependedItems', []) or [])
        items += list(getattr(listop, 'appendedItems', []) or [])
        return name in [str(i) for i in items]

    sensor = stage.GetPrimAtPath(pt.SENSOR_PRIM_PATH)
    if sensor:
        c.check(has_api(sensor, 'PhysxRigidBodyAPI'),
                '/World/sensor has PhysxRigidBodyAPI')
        c.check(has_api(sensor, 'PhysxCollisionAPI'),
                '/World/sensor has PhysxCollisionAPI')
        v = attr(sensor, 'physxRigidBody:maxDepenetrationVelocity')
        c.check(v is not None and abs(v - pt.SENSOR_MAX_DEPENETRATION_VELOCITY) < 1e-6,
                f'/World/sensor maxDepenetrationVelocity == '
                f'{pt.SENSOR_MAX_DEPENETRATION_VELOCITY} (got {v})')
        v = attr(sensor, 'physxCollision:contactOffset')
        c.check(v is not None and 0.0 < v <= 0.2 * pt.SENSOR_SIZE,
                f'/World/sensor contactOffset is small relative to the object '
                f'(got {v}, object is {pt.SENSOR_SIZE} m; PhysX defaults to 0.02)')
        v = attr(sensor, 'physxRigidBody:solverPositionIterationCount')
        c.check(v is not None and v >= pt.SOLVER_POSITION_ITERATIONS,
                f'/World/sensor solver position iterations >= '
                f'{pt.SOLVER_POSITION_ITERATIONS} (got {v})')

    scene_prim = stage.GetPrimAtPath(pt.PHYSICS_SCENE_PATH)
    c.check(bool(scene_prim), f'{pt.PHYSICS_SCENE_PATH} exists')
    if scene_prim:
        c.check(has_api(scene_prim, 'PhysxSceneAPI'),
                'physics scene has PhysxSceneAPI')
        v = attr(scene_prim, 'physxScene:minPositionIterationCount')
        c.check(v is not None and v >= pt.SOLVER_POSITION_ITERATIONS,
                f'physics scene min position iterations >= '
                f'{pt.SOLVER_POSITION_ITERATIONS} (got {v}; PhysX defaults to 4)')
        v = attr(scene_prim, 'physxScene:timeStepsPerSecond')
        c.check(v is not None and v >= pt.PHYSICS_TIME_STEPS_PER_SECOND,
                f'physics scene steps at >= {pt.PHYSICS_TIME_STEPS_PER_SECOND} Hz '
                f'(got {v}; Isaac defaults to 60)')

    for root in ('/World/piper/Geometry/base_link',
                 '/World/scout_mini/Geometry/base_link'):
        prim = stage.GetPrimAtPath(root)
        if not prim:
            continue
        c.check(has_api(prim, 'PhysxArticulationAPI'),
                f'{root} has PhysxArticulationAPI')
        v = attr(prim, 'physxArticulation:solverPositionIterationCount')
        c.check(v is not None and v >= pt.SOLVER_POSITION_ITERATIONS,
                f'{root} solver position iterations >= '
                f'{pt.SOLVER_POSITION_ITERATIONS} (got {v}; PhysX defaults to 4)')


# Finger friction material: grasp_fix_report.md's fix binds a
# FINGER_*_FRICTION material to both finger collision meshes
# (import_robots.py), but until the wall-mount slip investigation nothing
# asserted the binding actually landed in the built asset -- a silent
# regression here would quietly drop finger friction to PhysX's 0.5
# default and resurrect the lift-slip failure mode. A property of the shared
# robot asset, not of either demo's objects, so it is checked for BOTH demo
# stages (its own function, unlike check_physx_grasp_properties which is
# pedestal-only).
def check_finger_friction_material(stage: Usd.Stage, c: 'Checker') -> None:
    try:
        import physics_tuning as pt
    except Exception as exc:  # noqa: BLE001
        c.check(False, f'physics_tuning importable for the finger checks ({exc})')
        return

    def attr(prim, name):
        a = prim.GetAttribute(name) if prim else None
        return a.Get() if a and a.HasAuthoredValue() else None

    for path in pt.FINGER_COLLISION_PRIMS:
        prim = stage.GetPrimAtPath(path)
        name = path.rsplit('/', 1)[-1]
        c.check(bool(prim), f'finger collision prim {path} exists')
        if not prim:
            continue
        v = attr(prim, 'physics:approximation')
        c.check(v == pt.FINGER_COLLISION_APPROXIMATION,
                f'finger {name} collision approximation == '
                f'{pt.FINGER_COLLISION_APPROXIMATION} (got {v})')
        rel = prim.GetRelationship('material:binding:physics')
        targets = [str(t) for t in rel.GetTargets()] if rel else []
        c.check(len(targets) == 1,
                f'finger {name} has exactly one bound physics material '
                f'(got {targets or "none"})')
        if len(targets) != 1:
            continue
        mat = stage.GetPrimAtPath(targets[0])
        sf = attr(mat, 'physics:staticFriction')
        df = attr(mat, 'physics:dynamicFriction')
        cm = attr(mat, 'physxMaterial:frictionCombineMode')
        c.check(sf is not None and abs(sf - pt.FINGER_STATIC_FRICTION) < 1e-6,
                f'finger {name} material staticFriction == '
                f'{pt.FINGER_STATIC_FRICTION} (got {sf})')
        c.check(df is not None and abs(df - pt.FINGER_DYNAMIC_FRICTION) < 1e-6,
                f'finger {name} material dynamicFriction == '
                f'{pt.FINGER_DYNAMIC_FRICTION} (got {df})')
        c.check(cm == pt.FRICTION_COMBINE_MODE,
                f'finger {name} material frictionCombineMode == '
                f'{pt.FRICTION_COMBINE_MODE} (got {cm})')

    # Finger drives: the single most consequential number in the whole asset.
    # 1e3 N of maxForce on a prismatic finger joint -- inherited from the arm's
    # rotational N*m constant -- is what made the gripper eject the object.
    for joint in ('joint7', 'joint8'):
        prim = stage.GetPrimAtPath(f'/World/piper/Physics/{joint}')
        if not prim:
            continue
        v = attr(prim, 'drive:linear:physics:maxForce')
        c.check(v is not None and abs(v - pt.GRIPPER_MAX_FORCE) < 1e-6,
                f'{joint} drive maxForce == {pt.GRIPPER_MAX_FORCE} N (got {v})')
        v = attr(prim, 'drive:linear:physics:stiffness')
        c.check(v is not None and abs(v - pt.GRIPPER_STIFFNESS) < 1e-6,
                f'{joint} drive stiffness == {pt.GRIPPER_STIFFNESS} N/m (got {v})')
        v = attr(prim, 'physxJoint:maxJointVelocity')
        c.check(v is not None and v <= 10.0,
                f'{joint} maxJointVelocity is a sane finger speed (got {v}; the '
                f'asset shipped 57.29578, a degrees->metres unit leak)')

    # Arm damping ratio: 0.001 leaves the arm in a sustained ~3 Hz limit cycle
    # that swings the gripper +/-6-7 cm sideways during a straight-line move.
    for joint in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'):
        prim = stage.GetPrimAtPath(f'/World/piper/Physics/{joint}')
        if not prim:
            continue
        k = attr(prim, 'drive:angular:physics:stiffness')
        d = attr(prim, 'drive:angular:physics:damping')
        ratio = (d / k) if (k and d) else None
        c.check(ratio is not None and ratio >= 0.5 * pt.ARM_DAMPING_RATIO,
                f'{joint} damping/stiffness ratio >= '
                f'{0.5 * pt.ARM_DAMPING_RATIO:.4f} (got {ratio})')
        # Authored values are per-DEGREE; the validated gains are radian-scale.
        # Asserting the authored number catches the 57x error of writing the
        # radian value straight into an angular DriveAPI.
        c.check(k is not None and abs(k - pt.ARM_STIFFNESS_AUTHORED) < 1.0,
                f'{joint} authored angular stiffness == '
                f'{pt.ARM_STIFFNESS_AUTHORED:.1f} per-degree '
                f'(= {pt.ARM_STIFFNESS:.4g} N*m/rad) (got {k})')


def check_wall_demo(stage: Usd.Stage, c: 'Checker') -> None:
    """Everything specific to the wall-mount task's stage
    (kotek_scout_piper_wall_demo.usd): the 4 sensor-cam objects, the wall +
    its 4 targets, the sensor TF graph, and -- once author_camera_graphs.py
    / author_magnet_graph.py have also been run against this stage with a
    live Isaac Sim process -- the camera and magnet OmniGraphs. Camera/
    magnet checks are REQUIRED for --assert-demo-ready to pass, matching
    this project's own convention of asserting rather than trusting; run
    those two scripts first if this section fails on a fresh
    build_wall_stage.py output.
    """
    try:
        import physics_tuning as pt
    except Exception as exc:  # noqa: BLE001
        c.check(False, f'physics_tuning importable for the wall-demo checks ({exc})')
        return

    import build_wall_stage as ws

    print('  -- 4 sensor-cam objects --')
    for path in ws.SENSOR_CAM_PATHS:
        prim = stage.GetPrimAtPath(path)
        c.check(bool(prim), f'{path} exists')
        if not prim:
            continue
        c.check(prim.GetTypeName() == 'Cube', f'{path} is a Cube')
        c.check('PhysicsRigidBodyAPI' in prim.GetAppliedSchemas(),
                f'{path} has PhysicsRigidBodyAPI (graspable, not static)')
        mass_attr = prim.GetAttribute('physics:mass')
        mass = mass_attr.Get() if mass_attr else None
        c.check(mass is not None and abs(mass - pt.SENSOR_CAM_MASS) < 1e-6,
                f'{path} mass == {pt.SENSOR_CAM_MASS} kg (got {mass})')
        scale_attr = prim.GetAttribute('xformOp:scale')
        scale = scale_attr.Get() if scale_attr else None
        expected = pt.SENSOR_CAM_SIZE_XYZ
        c.check(
            scale is not None and all(
                abs(scale[i] - expected[i]) < 1e-4 for i in range(3)),
            f'{path} scale == {expected} (true 8x3.5x4cm box, got {tuple(scale) if scale else None})')

    print('  -- NO riser-hold joints (removed subsystem, regression guard) --')
    # The riser-hold FixedJoints (settling-drift fix, removed 2026-08-25)
    # were anchored to the scout ARTICULATION link -- and PhysX joints
    # anchored to an articulation link ignore breakForce, runtime
    # physics:jointEnabled=False, AND runtime RemovePrim (all three verified
    # live), so their time-gated "release" never reached the running sim and
    # every sensor stayed bolted to the chassis through LIFT_OBJECT. Measured
    # relative (box-vs-chassis) drift without them is 0.1-0.2mm (the friction
    # + damping tuning is sufficient), so the whole subsystem is deleted.
    # Anything that re-adds such a joint recreates the un-liftable-object
    # bug, hence this inverted check.
    for i in range(4):
        joint_path = f'{ws.SCOUT_BASE_LINK}/_riser_hold_{i + 1}'
        c.check(not stage.GetPrimAtPath(joint_path),
                f'{joint_path} does NOT exist (articulation-anchored holds '
                f'can never release at runtime)')

    print('  -- chassis parking brake (creep fix, see add_parking_brake) --')
    brake = stage.GetPrimAtPath('/World/_scout_parking_brake')
    c.check(bool(brake), '/World/_scout_parking_brake exists')
    if brake:
        c.check(brake.GetTypeName() == 'PhysicsFixedJoint',
                'parking brake is a PhysicsFixedJoint')
        b0 = brake.GetRelationship('physics:body0').GetTargets()
        b1 = brake.GetRelationship('physics:body1').GetTargets()
        c.check(b0 == [Sdf.Path(ws.SCOUT_BASE_LINK)] and not b1,
                f'parking brake anchors {ws.SCOUT_BASE_LINK} to the WORLD '
                f'(got body0={b0}, body1={b1})')
        pos1 = brake.GetAttribute('physics:localPos1').Get()
        c.check(pos1 is not None and abs(pos1[2] - ws.SCOUT_PARK_BASE_Z) < 1e-6,
                f'parking brake pins at settled z={ws.SCOUT_PARK_BASE_Z} '
                f'(got {pos1})')

    print('  -- wall + 4 mount targets --')
    wall = stage.GetPrimAtPath('/World/wall')
    c.check(bool(wall), '/World/wall exists')
    if wall:
        # Parent Xform: a KINEMATIC RigidBodyAPI (not a plain static
        # collider) -- see build_wall_stage.add_wall's docstring for why
        # (a magnet weld FixedJoint needs a well-defined, scale-free rigid
        # body frame to anchor into; a bare CollisionAPI-only prim produced
        # PhysX's own "disjointed body transforms" warning in a live probe).
        # The actual (anisotropically-scaled) collision geometry lives on
        # the '/World/wall/geom' child instead, kept scale-free here.
        c.check('PhysicsRigidBodyAPI' in wall.GetAppliedSchemas(),
                '/World/wall has PhysicsRigidBodyAPI (kinematic anchor for the weld joint)')
        c.check(bool(wall.GetAttribute('physics:kinematicEnabled').Get()),
                '/World/wall is kinematic (never moves under simulation)')
        wall_geom = stage.GetPrimAtPath('/World/wall/geom')
        c.check(bool(wall_geom), '/World/wall/geom exists')
        if wall_geom:
            c.check('PhysicsCollisionAPI' in wall_geom.GetAppliedSchemas(),
                    '/World/wall/geom has PhysicsCollisionAPI')
    for i in range(4):
        path = f'/World/wall_target_{i + 1}'
        prim = stage.GetPrimAtPath(path)
        c.check(bool(prim), f'{path} exists')
        if prim:
            purpose = prim.GetAttribute('purpose').Get()
            c.check(purpose == 'guide', f'{path} purpose == guide (got {purpose})')

    print('  -- wall sensor TF graph --')
    publish_tf = stage.GetPrimAtPath(
        '/World/kotek_wall_sensor_graph/ros2_publish_transform_tree')
    c.check(bool(publish_tf), 'kotek_wall_sensor_graph/ros2_publish_transform_tree exists')
    if publish_tf:
        parent_targets = publish_tf.GetRelationship('inputs:parentPrim').GetTargets()
        c.check(
            [str(p) for p in parent_targets] == [ws.SCOUT_BASE_LINK],
            f'parentPrim -> physics-tracked scout base_link (got {parent_targets})')
        child_targets = [str(p) for p in
                         publish_tf.GetRelationship('inputs:targetPrims').GetTargets()]
        c.check(
            sorted(child_targets) == sorted(ws.SENSOR_CAM_PATHS),
            f'targetPrims -> all 4 sensor-cams (got {child_targets})')

    print('  -- cameras (need author_camera_graphs.py, live Isaac Sim) --')
    camera_paths = {
        'wrist': None,   # exact path is chosen inside author_camera_graphs.py
        'external': '/World/external_camera',
    }
    cameras = [p for p in stage.TraverseAll() if str(p.GetTypeName()) == 'Camera']
    camera_names = {p.GetName() for p in cameras}
    c.check(len(cameras) >= 6,
            f'>= 6 Camera prims exist (1 wrist + 1 external + 4 sensor-cam, got {len(cameras)}: '
            f'{sorted(camera_names)})')
    # IsaacCreateRenderProduct only creates its RenderProduct-typed prim at
    # RUNTIME when the node first executes (like every other OGN compute
    # node here, e.g. IsaacArticulationController) -- a static USD read can
    # only see the AUTHORING-time node, not that later artifact.
    render_product_nodes = [
        p for p in stage.TraverseAll()
        if str(p.GetTypeName()) == 'OmniGraphNode'
        and p.GetAttribute('node:type').Get() == 'isaacsim.core.nodes.IsaacCreateRenderProduct']
    c.check(len(render_product_nodes) >= 6,
            f'>= 6 IsaacCreateRenderProduct nodes exist, one per camera '
            f'(got {len(render_product_nodes)})')
    camera_helpers = [p for p in stage.TraverseAll()
                      if str(p.GetTypeName()) == 'OmniGraphNode'
                      and p.GetAttribute('node:type').Get() == 'isaacsim.ros2.bridge.ROS2CameraHelper']
    c.check(len(camera_helpers) >= 6,
            f'>= 6 ROS2CameraHelper nodes exist (got {len(camera_helpers)})')

    print('  -- magnet graph (needs author_magnet_graph.py, live Isaac Sim) --')
    script_nodes = [p for p in stage.TraverseAll()
                    if str(p.GetTypeName()) == 'OmniGraphNode'
                    and 'omni.graph.scriptnode' in str(p.GetAttribute('node:type').Get() or '')]
    # The stage carries TWO scriptnode graphs: the magnet graph (per-sensor
    # sensorPaths/wallTargets inputs) and the riser-release graph
    # (jointPaths/releaseTime inputs, checked separately below). Applying the
    # magnet-shape checks to every scriptnode falsely failed the release node.
    magnet_nodes = [p for p in script_nodes
                    if str(p.GetPath()).startswith('/World/kotek_magnet_graph/')]
    c.check(bool(magnet_nodes), 'a scriptnode magnet node exists')
    for node in magnet_nodes:
        sensor_attr = node.GetAttribute('inputs:sensorPaths')
        target_attr = node.GetAttribute('inputs:wallTargets')
        sensors = list(sensor_attr.Get() or []) if sensor_attr else []
        targets = list(target_attr.Get() or []) if target_attr else []
        c.check(
            sorted(str(s) for s in sensors) == sorted(ws.SENSOR_CAM_PATHS),
            f'{node.GetPath()} inputs:sensorPaths lists all 4 sensor-cams 1:1 '
            f'(got {sensors})')
        # Flattened xyz per sensor (author_magnet_graph.py's own
        # wallTargets encoding), not one entry per sensor.
        c.check(
            len(targets) == 3 * len(sensors),
            f'{node.GetPath()} inputs:wallTargets has 3 (x,y,z) entries per sensor '
            f'(got {len(targets)} doubles for {len(sensors)} sensors, expected '
            f'{3 * len(sensors)})')

    print('  -- NO riser-release graph (removed with the riser-hold joints) --')
    # The release ScriptNode only ever edited USD; PhysX never saw it (see
    # the riser-hold regression guard above). With the joints gone the graph
    # is gone too -- its presence would mean a stale stage build.
    release_nodes = [
        p for p in script_nodes
        if str(p.GetPath()).startswith('/World/kotek_riser_release_graph/')]
    c.check(not release_nodes,
            'no riser-release scriptnode remains (stale-stage guard)')


def inspect(path: str) -> int:
    stage = Usd.Stage.Open(path)
    if not stage:
        print(f'Could not open stage: {path}')
        return 1

    c = Checker()

    print('== robots compose correctly from the sublayer ==')
    scout = stage.GetPrimAtPath(SCOUT_BASE_LINK)
    c.check(bool(scout), f'{SCOUT_BASE_LINK} exists')
    if scout:
        c.check(
            'PhysicsArticulationRootAPI' in scout.GetAppliedSchemas(),
            'scout base_link is an articulation root')
    piper = stage.GetPrimAtPath(PIPER_BASE_LINK)
    c.check(bool(piper), f'{PIPER_BASE_LINK} exists')
    if piper:
        c.check(
            'PhysicsArticulationRootAPI' in piper.GetAppliedSchemas(),
            'piper base_link is an articulation root')

    print('== mount joint ==')
    mount = stage.GetPrimAtPath('/World/piper_mount_joint')
    c.check(bool(mount), '/World/piper_mount_joint exists')
    if mount:
        exclude = mount.GetAttribute('physics:excludeFromArticulation').Get()
        c.check(exclude is True, f'excludeFromArticulation == True (got {exclude})')
        body0 = mount.GetRelationship('physics:body0').GetTargets()
        body1 = mount.GetRelationship('physics:body1').GetTargets()
        c.check(
            [str(p) for p in body0] == [SCOUT_BASE_LINK],
            f'body0 -> scout base_link (got {body0})')
        c.check(
            [str(p) for p in body1] == [PIPER_BASE_LINK],
            f'body1 -> piper base_link (got {body1})')

    print('== wheel and arm joints present ==')
    for j in WHEEL_JOINTS:
        c.check(
            bool(stage.GetPrimAtPath(f'{SCOUT_ARM_JOINTS_PATH}/{j}')), f'scout joint {j} exists')
    for j in ARM_JOINTS:
        c.check(bool(stage.GetPrimAtPath(f'{PIPER_JOINTS_PATH}/{j}')), f'piper joint {j} exists')

    print('== scout drive graph: no stale defaults on the velocity chain ==')
    diff_ctrl = stage.GetPrimAtPath('/World/scout_mini/scout_drive_graph/diff_ctrl')
    c.check(bool(diff_ctrl), 'scout_drive_graph/diff_ctrl exists')
    if diff_ctrl:
        for attr_name in ('inputs:linearVelocity', 'inputs:angularVelocity'):
            attr = diff_ctrl.GetAttribute(attr_name)
            conns = attr.GetConnections()
            c.check(
                len(conns) == 1,
                f'diff_ctrl.{attr_name} has exactly one live connection (got {len(conns)}): '
                f'{[str(x) for x in conns]}')
            value = attr.Get()
            # This is exactly the bug that motivated the rebuild: a
            # connected attribute should have no meaningful authored
            # fallback value of its own left behind.
            c.check(
                value is None or value == 0.0,
                f'diff_ctrl.{attr_name} has no stale authored default (got {value!r})')

    # The two demos are independent siblings on the same robot asset (see the
    # wall-mount task plan) -- auto-detect which one this stage is rather
    # than hard-failing on the other demo's absent prims.
    is_pick_place_demo = bool(stage.GetPrimAtPath('/World/sensor'))
    is_wall_demo = bool(stage.GetPrimAtPath('/World/wall'))

    print('== gripper finger friction material (robot asset, both demos) ==')
    check_finger_friction_material(stage, c)

    if is_pick_place_demo:
        print('== sensor object ==')
        sensor = stage.GetPrimAtPath('/World/sensor')
        c.check(bool(sensor), '/World/sensor exists')
        if sensor:
            c.check(sensor.GetTypeName() == 'Cube', '/World/sensor is a Cube')
            c.check(
                'PhysicsRigidBodyAPI' in sensor.GetAppliedSchemas(),
                '/World/sensor has PhysicsRigidBodyAPI (graspable, not static)')
            mass_attr = sensor.GetAttribute('physics:mass')
            c.check(
                mass_attr and mass_attr.Get() and mass_attr.Get() > 0.0,
                f'/World/sensor mass > 0 (got {mass_attr.Get() if mass_attr else None})')
            # A true cube sized by CreateSizeAttr, with NO scale op. The prior
            # asset used a size-1.0 cube with a non-uniform AddScaleOp(0.05, 0.05,
            # 0.08) -- a scaled analytic collider gripped on its narrow face with
            # its long axis vertical, maximising the tipping moment.
            c.check(
                not any(op.startswith('xformOp:scale')
                        for op in (sensor.GetAttribute('xformOpOrder').Get() or [])),
                '/World/sensor has no scale op (collider is an unscaled analytic cube)')

        print('== sensor object: PhysX properties (grasp-fix regression guard) ==')
        # This whole block exists because the object previously carried
        # UsdPhysics.RigidBodyAPI + CollisionAPI and NOTHING else -- every PhysX
        # property silently at its default, including a 0.02 m contactOffset on a
        # 0.05 m object. That is a bug class that leaves no trace in the file, so
        # it is asserted here rather than trusted. See physics_tuning.py.
        check_physx_grasp_properties(stage, c)

        print('== sensor TF graph ==')
        publish_tf = stage.GetPrimAtPath('/World/kotek_sensor_graph/ros2_publish_transform_tree')
        c.check(bool(publish_tf), 'ros2_publish_transform_tree node exists')
        if publish_tf:
            parent_targets = publish_tf.GetRelationship('inputs:parentPrim').GetTargets()
            c.check(
                [str(p) for p in parent_targets] == [SCOUT_BASE_LINK],
                f'parentPrim -> physics-tracked scout base_link, not a bare Xform '
                f'(got {parent_targets})')
            child_targets = publish_tf.GetRelationship('inputs:targetPrims').GetTargets()
            c.check(
                [str(p) for p in child_targets] == ['/World/sensor'],
                f'targetPrims -> /World/sensor (got {child_targets})')
    else:
        print('== (pick-place demo checks skipped -- /World/sensor not present) ==')

    if is_wall_demo:
        print('== wall-mount demo ==')
        check_wall_demo(stage, c)
    else:
        print('== (wall-mount demo checks skipped -- /World/wall not present) ==')

    print('== dangling / duplicate OmniGraph connections (regression guard) ==')
    check_no_dangling_or_duplicate_connections(stage, c)

    print()
    if c.failures:
        print(f'{len(c.failures)} check(s) FAILED:')
        for f in c.failures:
            print(f'  - {f}')
        return 1
    print('All checks PASSED.')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', help='path to the demo stage to verify')
    parser.add_argument(
        '--assert-demo-ready', action='store_true',
        help='exit non-zero if any check fails (default behavior; flag kept for '
             'explicitness in CI invocations)')
    args = parser.parse_args()
    sys.exit(inspect(args.stage))


if __name__ == '__main__':
    main()
