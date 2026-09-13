#!/usr/bin/env python3
"""Rebuilds the Scout Mini + Piper robot asset from URDF, from scratch.

Runs INSIDE Isaac Sim (needs a live Kit process for the URDF->USD
converter) -- unlike build_demo_stage.py, this cannot run under plain
usd-core. Invoke via run_isaac.sh (no venv to source -- run_isaac.sh
picks the interpreter itself, see README):

    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/import_robots.py

Why this exists: the source stage's scout drive OmniGraph is corrupted --
8 attribute pins (all in the ROS2SubscribeTwist -> DifferentialController
-> IsaacArticulationController drive-command chain) carry a stale second
connection to a nonexistent prim tree from an unrelated template scene
("/World/limo_env/.../scout_v2_09_29_usb/..."), leaving
DifferentialController's velocity inputs to fall back to stale authored
defaults (linearVelocity=-1.6, angularVelocity=10.2) -- which is exactly
why the robot drives backward and veers the instant Play is pressed, with
no /cmd_vel publisher needed. See the plan for the full diagnosis.

Pipeline (two steps, matching how isaacsim.asset.importer.urdf's
URDFImporter actually works in this Isaac Sim version -- it converts a
URDF to a standalone USD FILE on disk, it does not spawn directly into an
open stage):
  1. Convert scout_mini.urdf and piper_description.urdf each to their own
     standalone robot USD (usd/robots/scout_mini.usd, usd/robots/piper.usd).
  2. Compose a new scene stage that references both, positions the Piper
     at the same fixed-joint offset already verified from the (otherwise-
     corrupted) source stage's root_joint (localPos0 = (0, 0, 0.29101)),
     and authors that mount as a fixed joint with
     excludeFromArticulation=True (the fix already proven necessary/stable
     in an earlier session's Tier 3 run).

OmniGraph authoring is left to author_robot_graphs.py so each stage can be
inspected/tested independently.

isaac_simulation (the URDF sources) is never modified: package:// mesh
URIs are rewritten to absolute file:// paths in temporary copies of the
URDF files, not the originals.
"""
import argparse
import os
import shutil
import sys
import tempfile

from isaacsim import SimulationApp

SCOUT_URDF = '/home/trs/isaac_simulation/scout_description/scout_mini.urdf'
SCOUT_PKG_ROOT = '/home/trs/isaac_simulation/scout_description'
PIPER_URDF = (
    '/home/trs/isaac_simulation/piper_isaac_sim/piper_description/'
    'urdf/piper_description.urdf'
)
PIPER_PKG_ROOT = '/home/trs/isaac_simulation/piper_isaac_sim/piper_description'

ROBOTS_OUT_DIR = '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/robots'

# From root_joint in the (corrupted-graph, but geometrically fine) source
# stage: physics:localPos0 = (0, 0, 0.29101), body0=scout base_link,
# body1=piper base_link, localRot0/1 = identity.
ARM_MOUNT_OFFSET = (0.0, 0.0, 0.29101)

# Confirmed by reading back the freshly-converted USD (not assumed): the
# new importer puts the articulation root at Geometry/base_link and all
# joints in a sibling Geometry/Physics scope (a different layout from the
# old source stage, which nested joints under their child link directly).
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
PIPER_BASE_LINK = '/World/piper/Geometry/base_link'
SCOUT_WHEEL_JOINTS = [
    'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
PIPER_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
PIPER_GRIPPER_JOINTS = ['joint7', 'joint8']

# Matches the drive gain correction already proven necessary/stable last
# session (source stage shipped damping=0, stiffness=1e7 -- unstable).
JOINT_STIFFNESS = 1.0e5
JOINT_DAMPING = 1.0e2

# Wheel-specific: found by directly comparing our rebuild against two
# independent working references for this exact geometry --
# /World/scout_working in the original (read-only) scout_and_piper_ros2_final.usd
# (same 60kg chassis / 3kg wheel masses as our own rebuild) and the Limo
# robot's own drive (limo_env.usd, a different but structurally similar
# AgileX platform -- and the confirmed source template the original,
# corrupted drive graph was cloned from: its differential_controller's
# stale angularVelocity=10.2 default is the exact same value baked into
# the original bug). Both use damping=1e6 (1e4x ours) and an explicitly
# unbounded physxJoint:maxJointVelocity (we never set this at all, so it
# was left at whatever the URDF importer's own default is). Our own
# damping sweep before this only went up to 1000 -- three orders of
# magnitude short of what a working same-geometry configuration actually
# uses.
WHEEL_DAMPING = 1.0e6
WHEEL_MAX_JOINT_VELOCITY = float('inf')

# --- the real fix for commanded straight-line drive (Test 2) -------------
# Found via exhaustive isolation testing in a from-scratch minimal
# scout-only sandbox (import_scout_only.py), outside this asset entirely
# -- see docs/pick_and_delivery_report.md sections 1.5-1.7 for the full
# diagnostic trail. Three compounding, independently-confirmed bugs, none
# of which is a stage-composition or drive-graph-wiring problem:
#
# 1. Wheel collision is 7 fragmented convex-hull sub-parts (per-mesh-part
#    from the URDF's multi-piece wheel.dae). Direct PhysX contact-report
#    instrumentation showed only ~5-7% of contact candidates had nonzero
#    impulse / were actually touching -- mostly non-touching speculative
#    contact, not continuous rolling contact. Fixed by replacing it with
#    a single clean analytic UsdGeom.Cylinder, sized/positioned/oriented
#    from ground-truth measurements (world-space bbox + the joint's own
#    physics:localPos1, NOT UsdGeom.BBoxCache.ComputeLocalBound(prim),
#    which returns the bound in the prim's PARENT frame, not its own
#    local space -- a real trap that produced a wildly wrong collider
#    position on the first attempt).
# 2. The URDF authors identical inertia (ixx=iyy=0.7171, izz=0.1361) for
#    ALL FOUR wheels -- a solid 3kg/0.08m-radius wheel should have I on
#    the order of 0.01-0.02 kg*m^2 (I=0.5*m*r^2=0.0096 axial), i.e. this
#    is ~15-75x too large, almost certainly a copy-paste error in the
#    upstream URDF. This alone was enough to suppress essentially all
#    momentum transfer even with perfect collision geometry. Fixed by
#    clearing physics:diagonalInertia/principalAxes to the "let PhysX
#    auto-compute from collision geometry + mass" sentinel.
# 3. wheel_link's own world orientation is MIRRORED between left/right
#    sides (front_left orient=(0.707,-0.707,0,0) vs
#    front_right=(0.707,0.707,0,0), same pattern rear) -- so a positive
#    commanded joint velocity rolls one side forward and the other side
#    BACKWARD. This is fixed at the control layer, in
#    author_robot_graphs.py's build_scout_drive_graph (negates the
#    right-side wheel command).
#
# With all three combined, a real ROS 2 /cmd_vel-driven test on the
# scout-only sandbox moved 58.33cm in 2s against a 0.3 m/s command --
# matching the ~60cm/2s pure-rolling prediction almost exactly, using
# PhysX's own native velocity drive (no custom torque controller needed).
WHEEL_RADIUS = 0.08
WHEEL_COLLISION_HEIGHT = 0.16

# The Piper URDF specifies effort=100 (N*m) uniformly for every arm joint,
# which is not enough torque authority for joint3 to hold some reachable
# target poses against the gravity load of the rest of the arm -- confirmed
# directly: commanding joint3 to -0.3 rad (with joint2=0.4, joint4=0.2 etc.)
# leaves it stuck at 0.0 with drive:angular:physics:maxForce=100 (the
# imported default), and it converges exactly to -0.3 with maxForce bumped
# to 400. This is real physical torque insufficiency, not a wiring bug (the
# command chain was independently verified correct: IsaacArticulationController
# receives targetPosition=-0.3 every tick regardless). Overridden generously
# for all arm+gripper joints for headroom (e.g. holding a grasped object).
JOINT_MAX_FORCE = 1.0e3

# Mounted at the front of the chassis, facing forward (+X). A 270 degree
# horizontal FOV / 1 degree resolution / 10Hz rotation rate matches common
# real 2D lidars (e.g. RPLidar/Hokuyo) used for reactive obstacle
# avoidance.
#
# Z was originally 0.12 ("just above deck height", base_link's own local
# bbox top is ~0.069m) -- confirmed by direct /scan inspection (live E2E
# investigation, see docs/pick_and_delivery_report.md section 4.9) that
# this self-detects part of the Piper arm's own mount structure: at world
# z~0.30 (matching the old local 0.12 + base_link's own ~0.18 settle
# height) the forward-facing beam hit something ~0.27m dead ahead with the
# robot still stationary at spawn, nowhere near either real demo-stage
# obstacle. The Piper's own base_link sits at local z=ARM_MOUNT_OFFSET[2]
# (0.29101, i.e. world ~0.47) -- the mount structure spans roughly
# base_link's own deck (~0.07) up to that, squarely through the lidar's
# original height. Raised here, then re-verified empirically with the
# same /scan inspection technique (not just a guessed value) --
# see the report for the exact before/after numbers.
LIDAR_LOCAL_POS = (0.30, 0.0, 0.35)
LIDAR_MIN_RANGE = 0.1
LIDAR_MAX_RANGE = 10.0
LIDAR_HORIZONTAL_FOV = 270.0
LIDAR_HORIZONTAL_RESOLUTION = 1.0
LIDAR_ROTATION_RATE = 10.0


def _make_resolved_urdf_copy(urdf_path: str, pkg_root: str, pkg_name: str, tmp_dir: str) -> str:
    """Copies `urdf_path` into `tmp_dir` with package://pkg_name/... rewritten
    to absolute file paths, so mesh resolution doesn't depend on
    ROS_PACKAGE_PATH/ament index at all. Never touches the original file.
    """
    with open(urdf_path, 'r') as f:
        text = f.read()
    prefix = f'package://{pkg_name}/'
    replacement = pkg_root.rstrip('/') + '/'
    text = text.replace(prefix, replacement)
    out_path = os.path.join(tmp_dir, os.path.basename(urdf_path))
    with open(out_path, 'w') as f:
        f.write(text)
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        default='/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
        'kotek_scout_piper_robot.usd')
    parser.add_argument('--headless', action='store_true', default=True)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': args.headless})

    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.core.utils import extensions
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    # Imported here, not at module scope: physics_tuning imports pxr,
    # which does not exist until SimulationApp has been constructed.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import physics_tuning

    tmp_dir = tempfile.mkdtemp(prefix='kotek_urdf_')
    os.makedirs(ROBOTS_OUT_DIR, exist_ok=True)
    try:
        scout_urdf = _make_resolved_urdf_copy(
            SCOUT_URDF, SCOUT_PKG_ROOT, 'scout_description', tmp_dir)
        piper_urdf = _make_resolved_urdf_copy(
            PIPER_URDF, PIPER_PKG_ROOT, 'piper_description', tmp_dir)

        def convert(
                urdf_path, out_subdir, merge_fixed, joint_target_type, joint_stiffness,
                merge_mesh=False):
            out_dir = os.path.join(ROBOTS_OUT_DIR, out_subdir)
            # Cleared each run so re-running this script is idempotent --
            # otherwise the importer auto-increments (scout_mini_1, _2, ...)
            # to avoid overwriting prior output.
            shutil.rmtree(out_dir, ignore_errors=True)
            os.makedirs(out_dir, exist_ok=True)
            config = URDFImporterConfig(
                urdf_path=urdf_path,
                usd_path=out_dir,
                merge_fixed_joints=merge_fixed,
                # Scout only (see call site): consolidates each link's
                # multiple per-part meshes (visual AND collision) into one
                # -- our wheel collision was fragmented into 7 disjoint
                # convex-hull sub-parts (hub/spokes/tread as separate
                # pieces) instead of one continuous rolling surface, unlike
                # two independent working references for this exact
                # geometry (see WHEEL_DAMPING's comment).
                merge_mesh=merge_mesh,
                collision_from_visuals=False,
                collision_type='Convex Hull',
                allow_self_collision=False,
                fix_base=False,  # floating base; we mount/position explicitly ourselves
                joint_target_type=joint_target_type,
                override_joint_stiffness=joint_stiffness,
                override_joint_damping=JOINT_DAMPING,
            )
            importer = URDFImporter(config)
            usd_file = importer.import_urdf()
            print(f'### converted {urdf_path} -> {usd_file}', flush=True)
            return usd_file

        # Scout wheels are continuous-rotation joints driven by velocity
        # (DifferentialController -> IsaacArticulationController's
        # velocityCommand in the drive graph) -- a 'position' drive with
        # high stiffness would fight that. Stiffness 0 here means "pure
        # velocity/damping drive", the standard PhysX pattern for wheels;
        # JOINT_DAMPING (100) still provides the actual velocity-tracking
        # gain. The Piper's arm+gripper joints are genuinely
        # position-controlled (JointState position commands), so they keep
        # 'position' with real stiffness.
        scout_usd = convert(
            scout_urdf, 'scout_mini', merge_fixed=True, joint_target_type='velocity',
            joint_stiffness=0.0, merge_mesh=False)
        print('### about to convert piper', flush=True)
        piper_usd = convert(
            piper_urdf, 'piper', merge_fixed=False, joint_target_type='position',
            joint_stiffness=JOINT_STIFFNESS)
        print('### both conversions done, starting scene composition', flush=True)

        # --- compose the scene referencing both freshly-converted robots ---
        omni.usd.get_context().new_stage()
        print('### new_stage() ok', flush=True)
        stage = omni.usd.get_context().get_stage()
        print('### get_stage() ok', flush=True)
        world = stage.DefinePrim('/World', 'Xform')
        stage.SetDefaultPrim(world)
        stage.SetMetadata('upAxis', 'Z')
        stage.SetMetadata('metersPerUnit', 1.0)
        print('### /World defined', flush=True)

        physics_scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics_scene.CreateGravityMagnitudeAttr(9.81)
        # A bare UsdPhysics.Scene is enough for basic PhysX stepping, but
        # isaacsim.core.nodes.* OGN nodes (IsaacReadSimulationTime,
        # IsaacArticulationController, IsaacComputeOdometry) use the
        # omni.physx.tensors batched API internally, which requires
        # PhysxSceneAPI to recognize the scene as active -- confirmed by a
        # real Tier 3 run: without this, those nodes' outputs never
        # advance (IsaacReadSimulationTime stuck at its first value) even
        # though sim_ctx.step() genuinely advances physics, and
        # omni.physx.tensors logs "Failed to create simulation view: no
        # active physics scene found" on every tick.
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        # ...but applying PhysxSceneAPI with ZERO attributes, which is
        # all this did until now, leaves solver type, iteration counts,
        # CCD, stabilization and the step rate at PhysX's own defaults:
        # 4 position / 1 velocity iterations at 60 Hz. That is not
        # enough to resolve finger-on-object contact, and is a direct
        # cause of the object being ejected from the gripper. See
        # physics_tuning's module docstring for the full diagnosis.
        physics_tuning.apply_physx_scene_tuning(physics_scene.GetPrim())
        print('### physics scene defined (+ PhysX solver tuning)', flush=True)

        ground = UsdGeom.Plane.Define(stage, '/World/GroundPlane')
        ground.CreateAxisAttr('Z')
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        print('### ground plane defined', flush=True)

        scout_prim = stage.DefinePrim('/World/scout_mini', 'Xform')
        scout_prim.GetReferences().AddReference(scout_usd)
        print('### scout reference added', flush=True)

        piper_prim = stage.DefinePrim('/World/piper', 'Xform')
        piper_prim.GetReferences().AddReference(piper_usd)
        print('### piper reference added', flush=True)
        UsdGeom.Xformable(piper_prim).AddTranslateOp().Set(Gf.Vec3d(*ARM_MOUNT_OFFSET))
        print('### piper positioned', flush=True)

        # The URDF importer's asset-structure convention puts the joint
        # definitions (PhysicsRevoluteJoint etc.) behind a PAYLOAD arc
        # (the "Physics" variant set), selected via a variant switch, not a
        # plain reference. Payloads are not guaranteed to auto-load just
        # because the prim that hosts them is referenced. Force-loading
        # here makes the physics payload unconditionally present regardless
        # of any default load-set policy a consumer might apply later.
        # (Collision geometry itself -- the instanceable Part__FeatureNNN_1
        # prims -- is pulled in via a plain `references` arc inside the
        # geometry payload and composes automatically once that payload is
        # loaded; it does NOT need a separate Load() call. An earlier
        # diagnostic that appeared to show zero collision prims anywhere on
        # the robot was itself wrong: Usd.Stage.TraverseAll() silently
        # skips into collapsed instance prototypes for `instanceable=true`
        # prims rather than visiting them at their instance paths, so it
        # undercounted schema-carrying prims for every instanceable mesh on
        # the robot, not just collision meshes. Traversing with
        # Usd.TraverseInstanceProxies() found 92 PhysicsCollisionAPI prims
        # in the composed stage -- collision geometry was present the whole
        # time.)
        stage.Load(scout_prim.GetPath())
        stage.Load(piper_prim.GetPath())
        print('### physics payloads force-loaded', flush=True)

        # Mount joint: two separate articulations (per the approved plan,
        # not merged into one) joined rigidly. excludeFromArticulation=True
        # is required -- both base_links carry PhysicsArticulationRootAPI,
        # and PhysX cannot merge two articulation roots through an
        # articulation joint (this is the fix already proven necessary in
        # an earlier session's Tier 3 run against the old stage).
        mount_joint = UsdPhysics.FixedJoint.Define(stage, '/World/piper_mount_joint')
        mount_joint.CreateBody0Rel().SetTargets([Sdf.Path(SCOUT_BASE_LINK)])
        mount_joint.CreateBody1Rel().SetTargets([Sdf.Path(PIPER_BASE_LINK)])
        mount_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*ARM_MOUNT_OFFSET))
        mount_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mount_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        mount_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        mount_joint.GetPrim().CreateAttribute(
            'physics:excludeFromArticulation', Sdf.ValueTypeNames.Bool).Set(True)
        print('### mount joint authored', flush=True)

        # --- direct drive-gain overrides -----------------------------------
        # URDFImporterConfig's override_joint_damping/override_joint_stiffness
        # do NOT produce the values passed in: a real Tier 3 run measured the
        # resulting angular damping on a wheel joint at 1.7453292608261108
        # for an intended value of 100.0 -- that is exactly 100 * (pi/180),
        # i.e. the importer silently treats the value as degrees and
        # converts it to radians, silently reducing it ~57x. Confirmed this
        # was a pure gain bug, not a wiring bug: the same run showed
        # DifferentialController correctly computing 3.75 rad/s (matching
        # 0.3 m/s / 0.08 m wheel radius exactly) and IsaacArticulationController
        # correctly receiving it on all 4 wheel joints -- the command chain
        # was always right, the wheels just didn't have enough torque
        # authority to track it quickly. Fixed by setting the drive
        # attributes directly on the composed stage, bypassing the
        # importer's conversion entirely.
        def set_drive_gains(
                joint_path, is_linear, stiffness, damping, max_force=None,
                max_joint_velocity=None):
            joint = stage.GetPrimAtPath(joint_path)
            axis = 'linear' if is_linear else 'angular'
            # The real, schema-correct, PhysX-read attributes are
            # namespaced "drive:<axis>:physics:<prop>" (UsdPhysics.DriveAPI
            # applied with instance name <axis>) -- confirmed by reading
            # the composed stage back: the importer's own drive lives at
            # drive:angular:physics:damping etc. A first attempt at this
            # fix wrote to "{axis}:physics:{prop}" (missing the "drive:"
            # prefix), which created harmless attributes PhysX never reads
            # -- Test 2's result was byte-identical before/after that
            # no-op fix, which is what caught the mistake.
            joint.CreateAttribute(
                f'drive:{axis}:physics:stiffness', Sdf.ValueTypeNames.Float).Set(float(stiffness))
            joint.CreateAttribute(
                f'drive:{axis}:physics:damping', Sdf.ValueTypeNames.Float).Set(float(damping))
            if max_force is not None:
                joint.CreateAttribute(
                    f'drive:{axis}:physics:maxForce', Sdf.ValueTypeNames.Float).Set(
                        float(max_force))
            if max_joint_velocity is not None:
                # Plain PhysX joint attribute (PhysxJointAPI), not part of
                # the UsdPhysics.DriveAPI namespace above -- both working
                # reference assets (scout_working, Limo) author this
                # explicitly as unbounded; our importer never authors it at
                # all, leaving it at PhysX's own default.
                joint.CreateAttribute(
                    'physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float).Set(
                        float(max_joint_velocity))

        for wheel_name in ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel',
                            'rear_right_wheel'):
            # Pure velocity drive: stiffness=0 (no position term), damping
            # is the velocity-tracking gain. See WHEEL_DAMPING's comment
            # for why this is 1e6, not JOINT_DAMPING's 100.
            #
            # NOT given the per-degree -> per-radian conversion applied to the
            # arm joints below, deliberately. WHEEL_DAMPING=1e6 is the value
            # the base-drive fix was validated against (58-60 cm of real
            # displacement in 2 s, matching the pure-rolling prediction --
            # report sections 1.7 and 4.5, eleven ruled-out hypotheses before
            # it). Rescaling it by 57x would be changing a hard-won,
            # separately-verified number on the strength of an argument rather
            # than a measurement. The unit quirk is noted, not "corrected"
            # here; if the wheels are ever retuned, retune them by measurement.
            set_drive_gains(
                f'/World/scout_mini/Physics/{wheel_name}', False, 0.0, WHEEL_DAMPING,
                max_joint_velocity=WHEEL_MAX_JOINT_VELOCITY)

        # JOINT_DAMPING (1e2) against JOINT_STIFFNESS (1e5) is a damping
        # ratio of 0.001, which leaves this arm in a sustained ~3 Hz limit
        # cycle under closed-loop Cartesian control -- measured at +/-6-7 cm
        # of LATERAL fingertip excursion during a commanded straight-line
        # descent, enough to sweep the gripper into the object it is
        # approaching. physics_tuning.ARM_DAMPING is the measured optimum;
        # see that module for the sweep.
        for arm_joint in PIPER_ARM_JOINTS:
            set_drive_gains(
                f'/World/piper/Physics/{arm_joint}', False,
                physics_tuning.ARM_STIFFNESS_AUTHORED,
                physics_tuning.ARM_DAMPING_AUTHORED,
                physics_tuning.ARM_MAX_FORCE)
        # joint7/joint8 are PRISMATIC. JOINT_MAX_FORCE (1e3) was chosen
        # for joint3's gravity-holding TORQUE in N*m (see its comment
        # above) and means NEWTONS here -- which made the gripper a
        # ~900 N vise on a 0.2 kg object and is why the object was
        # violently ejected on contact. Sized from first principles in
        # physics_tuning instead; see that module's docstring.
        for gripper_joint in PIPER_GRIPPER_JOINTS:
            set_drive_gains(
                f'/World/piper/Physics/{gripper_joint}', True,
                physics_tuning.GRIPPER_STIFFNESS, physics_tuning.GRIPPER_DAMPING,
                physics_tuning.GRIPPER_MAX_FORCE,
                max_joint_velocity=physics_tuning.GRIPPER_MAX_JOINT_VELOCITY)
        print('### drive gains overridden directly (bypassing importer unit bug)', flush=True)

        # --- wheel collision + inertia fix (see WHEEL_RADIUS's comment) -----
        for wheel_name in ('front_left_wheel_link', 'front_right_wheel_link',
                            'rear_left_wheel_link', 'rear_right_wheel_link'):
            wl_path = f'{SCOUT_BASE_LINK}/{wheel_name}'
            wl_prim = stage.GetPrimAtPath(wl_path)

            wl_prim.GetAttribute('physics:diagonalInertia').Set(Gf.Vec3f(0, 0, 0))
            wl_prim.GetAttribute('physics:principalAxes').Set(Gf.Quatf(0, 0, 0, 0))

            # 'wheel_1' is the imported collision-only copy ('wheel' is
            # visual-only); its Part__FeatureNNN children are individually
            # instanceable=True, invisible to a plain Usd.PrimRange and
            # un-editable without de-instancing first (a local override on
            # this stage's own composed prim -- does not touch the shared
            # prototype or any other wheel).
            collision_root = stage.GetPrimAtPath(f'{wl_path}/wheel_1')
            for child in collision_root.GetChildren():
                if child.IsInstanceable():
                    child.SetInstanceable(False)
            for p in Usd.PrimRange(collision_root):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    p.CreateAttribute(
                        'physics:collisionEnabled', Sdf.ValueTypeNames.Bool).Set(False)

            cyl = UsdGeom.Cylinder.Define(stage, f'{wl_path}/collision_cylinder')
            cyl.CreateRadiusAttr(WHEEL_RADIUS)
            cyl.CreateHeightAttr(WHEEL_COLLISION_HEIGHT)
            cyl.CreateAxisAttr('Z')
            UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        print('### wheel collision replaced with analytic cylinders, inertia corrected',
              flush=True)

        # --- articulation solver tuning ------------------------------------
        # Neither articulation carried PhysxArticulationAPI at all, so both
        # ran at PhysX's default 4 position / 1 velocity iterations.
        # hsr_sim (the working reference on this host) documents a
        # catastrophic articulation explosion caused by nothing but those
        # defaults, fixed by raising them alone -- see
        # hsr_sim/hsr_urdf_model.py:186-194.
        for art_root in (SCOUT_BASE_LINK, PIPER_BASE_LINK):
            art_prim = stage.GetPrimAtPath(art_root)
            if not art_prim.IsValid():
                raise RuntimeError(f'articulation root missing: {art_root}')
            physics_tuning.apply_physx_articulation_tuning(art_prim)
        print('### articulation solver iterations raised to 32/8', flush=True)

        # --- gripper finger contact surfaces -------------------------------
        # The fingers had NO physics material bound, so the sensor's
        # authored staticFriction=1.0 was combined against PhysX's default
        # 0.5 -- an effective ~0.7, not the 1.0 the stage file claims.
        def _deinstance_chain(path):
            """De-instances every ancestor on `path` the importer marked
            instanceable, so the leaf collider becomes editable on this
            stage. Same dance as the wheel colliders above: instanceable
            prims are invisible to a plain Usd.PrimRange and silently
            reject edits. This is a local override on this stage's own
            composed prims -- it does not touch the shared prototype.
            """
            parts = path.strip('/').split('/')
            for i in range(1, len(parts) + 1):
                p = stage.GetPrimAtPath('/' + '/'.join(parts[:i]))
                if p and p.IsInstanceable():
                    p.SetInstanceable(False)

        finger_material = physics_tuning.define_physics_material(
            stage, '/World/Looks/piper_finger_physics',
            physics_tuning.FINGER_STATIC_FRICTION,
            physics_tuning.FINGER_DYNAMIC_FRICTION,
            physics_tuning.FINGER_RESTITUTION)
        for finger_path in physics_tuning.FINGER_COLLISION_PRIMS:
            _deinstance_chain(finger_path)
            finger = stage.GetPrimAtPath(finger_path)
            if not finger.IsValid():
                raise RuntimeError(f'finger collider missing: {finger_path}')
            # A convex hull of the fork-shaped link7/link8 STL is a solid
            # wedge, so contact happened on a surface that does not exist
            # in the visual mesh.
            finger.CreateAttribute(
                'physics:approximation', Sdf.ValueTypeNames.Token).Set(
                    physics_tuning.FINGER_COLLISION_APPROXIMATION)
            physics_tuning.bind_physics_material(finger, finger_material)
        print('### finger colliders: convexDecomposition + high-friction material',
              flush=True)

        # --- lidar sensor (for reactive obstacle avoidance) -----------------
        # isaacsim.sensors.physx's PhysX-raycast lidar (not RTX) -- lighter
        # weight, no render-product/annotator machinery needed, and pairs
        # directly with the isaacsim.sensors.physx.IsaacReadLidarBeams OG
        # node (verified against a live Kit process: RangeSensorCreateLidar
        # returns a RangeSensorSchema.Lidar object, not a path -- the actual
        # prim path is sensor_handle.GetPath()).
        import omni.kit.commands
        extensions.enable_extension('isaacsim.sensors.physx')
        _, lidar_handle = omni.kit.commands.execute(
            'RangeSensorCreateLidar',
            path='/lidar_link',
            parent=SCOUT_BASE_LINK,
            translation=Gf.Vec3d(*LIDAR_LOCAL_POS),
            min_range=LIDAR_MIN_RANGE,
            max_range=LIDAR_MAX_RANGE,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=LIDAR_HORIZONTAL_FOV,
            vertical_fov=1.0,
            horizontal_resolution=LIDAR_HORIZONTAL_RESOLUTION,
            vertical_resolution=1.0,
            rotation_rate=LIDAR_ROTATION_RATE,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        lidar_prim_path = str(lidar_handle.GetPath())
        print(f'### lidar sensor created at {lidar_prim_path}', flush=True)

        # new_stage() creates an anonymous in-memory layer; it has no file
        # identifier yet, so GetRootLayer().Save() fails ("Cannot save
        # anonymous layer") -- save_as_stage() is what actually writes it.
        omni.usd.get_context().save_as_stage(args.output)
        print(f'### Saved composed scene to {args.output}', flush=True)
        print('### DONE', flush=True)
        # Prim-tree inspection is intentionally NOT done here: reading the
        # freshly-saved file back with a separate plain-pxr (usd-core)
        # process avoids a cross-binding quirk between this Kit process's
        # internal pxr and a top-level `from pxr import ...` (GetPrimAtPath
        # rejected both str and Sdf.Path with an ArgumentError here, even
        # though the save itself succeeded) -- see inspect_new_robot.py.

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        # NOTE: no `import sys` here. `sys` is imported at module scope, and a
        # local import inside this handler makes the name local to main() for
        # the WHOLE function -- so the sys.path.insert near the top raises
        # UnboundLocalError before any of this is reached.
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        simulation_app.close()


if __name__ == '__main__':
    main()
