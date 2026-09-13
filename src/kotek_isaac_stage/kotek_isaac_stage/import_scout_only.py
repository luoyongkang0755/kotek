#!/usr/bin/env python3
"""Builds a MINIMAL scout-only USD directly from the official URDF -- no
Piper arm, no lidar, no demo-stage sublayer, nothing but the scout and a
ground plane.

Why this exists: the full scout+piper pipeline (import_robots.py) produces
a scout that reaches and holds the exact commanded wheel angular velocity
(verified via the tensor-backed Articulation API) but whose chassis barely
translates (~1cm vs a ~60cm/2s pure-rolling prediction), and an extensive
attribute-by-attribute comparison against a proven-working reference robot
of the *same* geometry (scout_working, in the original source stage) ruled
out every robot-level attribute (drive damping, maxJointVelocity, collision
topology, ground primitive type, inertia tensor, mounted-arm presence)
without finding the cause -- see the plan's "Result: reference gains
applied..." section for the full trail. Rather than keep comparing against
an external reference inside the slow, multi-layer composed stage, this
script builds the robot up from nothing in the smallest possible sandbox,
so real commanded motion can be verified (or debugged) in ~20s per
iteration instead of a full pipeline rebuild.

Deliberately NOT sharing import_robots.py's piper-handling code paths, so
this script stays trivially auditable as "just the scout, nothing else."
Config values for the URDF conversion and wheel drive gains are copied
from import_robots.py's current, already-tuned values (WHEEL_DAMPING=1e6,
maxJointVelocity=inf -- matching scout_working and the Limo reference).

Run via run_isaac.sh (needs a live Kit process for the URDF->USD
converter, same as import_robots.py):

    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/import_scout_only.py

isaac_simulation (the URDF source) is never modified: package:// mesh URIs
are rewritten to absolute file:// paths in a temporary copy of the URDF
file, not the original.
"""
import argparse
import os
import shutil
import tempfile

from isaacsim import SimulationApp

SCOUT_URDF = '/home/trs/isaac_simulation/scout_description/scout_mini.urdf'
SCOUT_PKG_ROOT = '/home/trs/isaac_simulation/scout_description'

ROBOTS_OUT_DIR = '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/robots_scout_only'

# Same layout import_robots.py found and confirmed by reading back the
# freshly-converted USD: articulation root at Geometry/base_link, joints in
# a sibling Geometry/Physics scope.
SCOUT_BASE_LINK = '/World/scout_mini/Geometry/base_link'
SCOUT_WHEEL_JOINTS = [
    'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']

# Matches import_robots.py's JOINT_DAMPING (the importer's own
# override_joint_damping -- affected by the same degrees->radians unit bug
# documented there, irrelevant here since we override the wheel joints'
# actual drive gains directly afterward anyway).
JOINT_DAMPING = 1.0e2

# Best-known wheel drive gains so far (kept from import_robots.py's own
# investigation -- matches scout_working/Limo exactly; did not by itself
# fix Test 2 in the full stage, but is a real, harmless improvement and the
# natural starting point for this from-scratch rebuild).
WHEEL_DAMPING = 1.0e6
WHEEL_MAX_JOINT_VELOCITY = float('inf')

# --- the real fix for commanded straight-line drive (Test 2), found by ---
# --- exhaustive isolation testing outside this asset, see the plan and ---
# --- docs/pick_and_delivery_report.md section 1.5/1.6/1.7 for the full ---
# --- diagnostic trail. Three compounding, independently-confirmed bugs: --
#
# 1. Wheel collision is 7 fragmented convex-hull sub-parts (per-mesh-part
#    from the URDF's multi-piece wheel.dae). Direct PhysX contact-report
#    instrumentation showed only ~5-7% of contact candidates had nonzero
#    impulse / were actually touching (mean separation 1.5-1.6cm) --
#    mostly non-touching speculative contact, not continuous rolling
#    contact. Fixed by replacing it with a single clean analytic
#    UsdGeom.Cylinder, sized/positioned/oriented from ground-truth
#    measurements (world-space bbox + the joint's own physics:localPos1,
#    NOT UsdGeom.BBoxCache.ComputeLocalBound(prim), which returns the
#    bound in the prim's PARENT frame, not its own local space -- a real
#    trap that produced a wildly wrong collider position on the first
#    attempt).
# 2. The URDF authors identical inertia (ixx=iyy=0.7171, izz=0.1361) for
#    ALL FOUR wheels -- a solid 3kg/0.08m-radius wheel should have I on
#    the order of 0.01-0.02 kg*m^2 (I=0.5*m*r^2=0.0096 axial), i.e. this
#    is ~15-75x too large, almost certainly a copy-paste error in the
#    upstream URDF. This alone was enough to suppress essentially all
#    momentum transfer even with perfect collision geometry -- confirmed
#    directly (clearing it, alone, changed a torque-driven wheel from ~0
#    net displacement to real, if initially unstable, motion). Fixed by
#    clearing physics:diagonalInertia/principalAxes to the "let PhysX
#    auto-compute from collision geometry + mass" sentinel.
# 3. wheel_link's own world orientation is MIRRORED between left/right
#    sides (front_left orient=(0.707,-0.707,0,0) vs
#    front_right=(0.707,0.707,0,0), same pattern rear) -- so a positive
#    commanded joint velocity rolls one side forward and the other side
#    BACKWARD. author_robot_graphs.py's drive graph must command opposite
#    signs for left vs right wheels; this file only fixes the asset-level
#    bugs (1) and (2) -- the sign convention is a control-layer fix.
#
# With all three combined (collision + inertia fixed here, sign fixed in
# author_robot_graphs.py's build_scout_drive_graph), a real, direct
# tensor-API drive test moved 59.34cm in 2s against a 0.3 m/s command --
# matching the ~60cm/2s pure-rolling prediction almost exactly, using
# PhysX's own native velocity drive (no custom torque controller needed).
WHEEL_RADIUS = 0.08
WHEEL_COLLISION_HEIGHT = 0.16


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


def fix_wheel_collision_and_inertia(stage, Gf, Sdf, Usd, UsdGeom, UsdPhysics):
    """Replaces each wheel's fragmented 7-part convex-hull collision with a
    single analytic cylinder, and clears the URDF's corrupted inertia --
    see the module-level comment above WHEEL_RADIUS for the full diagnostic
    trail behind both fixes.
    """
    for name in ['front_left_wheel_link', 'front_right_wheel_link',
                 'rear_left_wheel_link', 'rear_right_wheel_link']:
        wl_path = f'{SCOUT_BASE_LINK}/{name}'
        wl_prim = stage.GetPrimAtPath(wl_path)

        wl_prim.GetAttribute('physics:diagonalInertia').Set(Gf.Vec3f(0, 0, 0))
        wl_prim.GetAttribute('physics:principalAxes').Set(Gf.Quatf(0, 0, 0, 0))

        # The imported collision copy lives under wheel_1 ('wheel' is the
        # visual-only copy); its Part__FeatureNNN children are
        # individually instanceable=True, so a plain Usd.PrimRange (no
        # TraverseInstanceProxies()) both fails to find them for editing
        # AND can't author collisionEnabled directly on an instance-proxy
        # path -- SetInstanceable(False) on this stage's own composed prim
        # first (a local override; does not touch the shared prototype or
        # any other wheel) makes them directly editable.
        collision_root = stage.GetPrimAtPath(f'{wl_path}/wheel_1')
        for child in collision_root.GetChildren():
            if child.IsInstanceable():
                child.SetInstanceable(False)
        for p in Usd.PrimRange(collision_root):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                p.CreateAttribute(
                    'physics:collisionEnabled', Sdf.ValueTypeNames.Bool).Set(False)

        # Ground truth for radius/center/axis, established directly
        # (world-space bbox measurement + the joint's own
        # physics:localPos1, cross-checked against the wheel's measured
        # world-frame angular velocity during a drive command -- NOT
        # UsdGeom.BBoxCache.ComputeLocalBound(prim), which returns the
        # bound in the prim's PARENT frame and silently produces a wrong
        # answer that happens to resemble the wheel's position in
        # base_link's frame): wheel_link's own local origin (0,0,0) is the
        # true rotation/hub center (matches the joint's own
        # physics:localPos1 exactly); radius 0.08m; rolling axis is local
        # Z (matches the URDF's own declared joint axis "0 0 -1").
        cyl = UsdGeom.Cylinder.Define(stage, f'{wl_path}/collision_cylinder')
        cyl.CreateRadiusAttr(WHEEL_RADIUS)
        cyl.CreateHeightAttr(WHEEL_COLLISION_HEIGHT)
        cyl.CreateAxisAttr('Z')
        UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        default='/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
        'kotek_scout_only.usd')
    parser.add_argument(
        '--collision-type', default='Convex Hull',
        help="URDFImporterConfig collision_type -- override for sandbox sweeps "
        "(e.g. 'Bounding Cube', 'Bounding Sphere', 'Sphere Fill').")
    parser.add_argument(
        '--fix-base', action='store_true',
        help='Pin the chassis in place (diagnostic only, not a real fix) -- isolates '
        'pure wheel-spin-under-load dynamics from free-base dynamics.')
    parser.add_argument(
        '--merge-mesh', action='store_true',
        help='Consolidate each link into one collision mesh (already tried once in the '
        'full pipeline and made things worse, kept as a sandbox sweep option).')
    parser.add_argument('--headless', action='store_true', default=True)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': args.headless})

    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    tmp_dir = tempfile.mkdtemp(prefix='kotek_scout_only_urdf_')
    try:
        scout_urdf = _make_resolved_urdf_copy(
            SCOUT_URDF, SCOUT_PKG_ROOT, 'scout_description', tmp_dir)

        out_dir = os.path.join(ROBOTS_OUT_DIR, 'scout_mini')
        shutil.rmtree(out_dir, ignore_errors=True)
        os.makedirs(out_dir, exist_ok=True)
        config = URDFImporterConfig(
            urdf_path=scout_urdf,
            usd_path=out_dir,
            merge_fixed_joints=True,
            merge_mesh=args.merge_mesh,
            collision_from_visuals=False,
            collision_type=args.collision_type,
            allow_self_collision=False,
            fix_base=args.fix_base,
            joint_target_type='velocity',
            override_joint_stiffness=0.0,
            override_joint_damping=JOINT_DAMPING,
        )
        importer = URDFImporter(config)
        scout_usd = importer.import_urdf()
        print(f'### converted {scout_urdf} -> {scout_usd}', flush=True)

        # --- compose the minimal scene -----------------------------------
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        world = stage.DefinePrim('/World', 'Xform')
        stage.SetDefaultPrim(world)
        stage.SetMetadata('upAxis', 'Z')
        stage.SetMetadata('metersPerUnit', 1.0)
        print('### /World defined', flush=True)

        physics_scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics_scene.CreateGravityMagnitudeAttr(9.81)
        # See import_robots.py's matching comment: required for
        # isaacsim.core.nodes.* OGN nodes / the tensor API to recognize the
        # scene as active.
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        print('### physics scene defined', flush=True)

        ground = UsdGeom.Plane.Define(stage, '/World/GroundPlane')
        ground.CreateAxisAttr('Z')
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        print('### ground plane defined', flush=True)

        scout_prim = stage.DefinePrim('/World/scout_mini', 'Xform')
        scout_prim.GetReferences().AddReference(scout_usd)
        print('### scout reference added', flush=True)

        # Force-load the physics payload -- see import_robots.py's matching
        # comment for why this is needed (joints live behind a real payload
        # arc, selected via a variant switch, not a plain reference).
        stage.Load(scout_prim.GetPath())
        print('### physics payload force-loaded', flush=True)

        # --- direct wheel drive-gain overrides ----------------------------
        # See import_robots.py's set_drive_gains for why these are set
        # directly on the composed stage rather than trusted to the
        # importer's override_joint_damping (a real, confirmed unit bug --
        # degrees silently treated as radians).
        def set_drive_gains(joint_path, damping, max_joint_velocity):
            joint = stage.GetPrimAtPath(joint_path)
            joint.CreateAttribute(
                'drive:angular:physics:stiffness', Sdf.ValueTypeNames.Float).Set(0.0)
            joint.CreateAttribute(
                'drive:angular:physics:damping', Sdf.ValueTypeNames.Float).Set(float(damping))
            joint.CreateAttribute(
                'physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float).Set(
                    float(max_joint_velocity))

        for wheel_name in SCOUT_WHEEL_JOINTS:
            set_drive_gains(
                f'/World/scout_mini/Physics/{wheel_name}', WHEEL_DAMPING,
                WHEEL_MAX_JOINT_VELOCITY)
        print('### wheel drive gains overridden directly', flush=True)

        fix_wheel_collision_and_inertia(stage, Gf, Sdf, Usd, UsdGeom, UsdPhysics)
        print('### wheel collision replaced with analytic cylinders, inertia corrected',
              flush=True)

        omni.usd.get_context().save_as_stage(args.output)
        print(f'### Saved minimal scout-only stage to {args.output}', flush=True)
        print('### DONE', flush=True)

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        simulation_app.close()


if __name__ == '__main__':
    main()
