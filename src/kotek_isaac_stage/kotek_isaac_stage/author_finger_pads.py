#!/usr/bin/env python3
"""Soft finger pads for the Piper gripper -- the sim equivalent of rubber
finger pads: a thin compliant-contact box on each finger's inner face.

Why (2026-09-25, measured kick forensics in docs/wall_mount_contact_placement.md
section 0): the close-kick rotates the box in the fingers because PhysX
point contacts carry no torsional friction -- the only yaw resistance is the
small contact patch of the fork-shaped finger STL. FRICTION is already at
rubber level (FINGER_STATIC_FRICTION 1.0, combine=max), so the remaining
real-world lever is CONTACT AREA: a real soft pad deforms and presses on a
face, giving distributed normal forces that resist rotation. This script
authors exactly that: a thin box collider embedded on each finger's inner
face, bound to the same finger material.

Geometry is MEASURED from the stage, not guessed: the world-space bounds of
the two FINGER_COLLISION_PRIMS mesh prims at the authored rest pose give
each finger plate's inner face (the bbox side facing the other finger) and
its face extent; the pad spans that face and protrudes PAD_THICKNESS beyond
it, half embedded into the finger so no gap can open. Pads are authored as
sibling collision prims under the same moving Xform as the finger mesh, so
they ride the prismatic joint for free, and they add no mass (the links
carry explicit URDF mass -- verified at runtime).

Run (live Isaac, same pattern as author_magnet_graph.py):
    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_finger_pads.py            # measure-only, prints numbers
    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_finger_pads.py --write  # author pads + save in place

Idempotent: existing pad prims are removed and re-authored. The stage USD
is backed up to /home/trs/e2e_runs/ before the first --write.
"""
import argparse
import os
import shutil
import sys

from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# NOTE: physics_tuning (and anything importing pxr) must be imported INSIDE
# main(), after SimulationApp() -- a module-level pxr import breaks kit's
# pybind bootstrap ("No to_python converter for GfVec3f" at startup).

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')
BACKUP = '/home/trs/e2e_runs/kotek_scout_piper_wall_demo.before_pads.usd'

PAD_THICKNESS = 0.003        # m, protrusion per finger (soft-pad stand-in)
PAD_EMBED = 0.5              # fraction of thickness embedded into the finger
PAD_CONTACT_OFFSET = 0.001   # m; PhysX default 0.02 would touch from 2 cm
PAD_MAX_HEIGHT = 0.04        # m; never taller than the sensor box
PAD_MAX_WIDTH = 0.08         # m; never wider than the sensor box long edge
PAD_PRIM_NAMES = ('link7_softpad', 'link8_softpad')


def _deinstance_chain(stage, path):
    """Same instanceable-override dance as import_robots.py's finger block."""
    parts = path.strip('/').split('/')
    for i in range(1, len(parts) + 1):
        p = stage.GetPrimAtPath('/' + '/'.join(parts[:i]))
        if p and p.IsInstanceable():
            p.SetInstanceable(False)


def _world_bounds(stage, path, cache):
    from pxr import UsdGeom
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f'missing prim: {path}')
    bound = cache.ComputeWorldBound(prim)
    box = bound.ComputeAlignedBox()
    return box.GetMin(), box.GetMax()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    parser.add_argument('--write', action='store_true',
                        help='author pads and save (default: measure only)')
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': True})
    ok = False
    try:
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

        import physics_tuning  # noqa: E402  (after SimulationApp, see header)

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        print('### stage opened', flush=True)

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  ['default', 'render', 'proxy', 'guide'])

        # --- measure the two finger plates --------------------------------
        paths = physics_tuning.FINGER_COLLISION_PRIMS
        mins, maxs = [], []
        for p in paths:
            mn, mx = _world_bounds(stage, p, cache)
            mins.append(mn)
            maxs.append(mx)
            print(f'### {p}\n    min=({mn[0]:.4f},{mn[1]:.4f},{mn[2]:.4f}) '
                  f'max=({mx[0]:.4f},{mx[1]:.4f},{mx[2]:.4f})', flush=True)

        c7 = (mins[0] + maxs[0]) * 0.5
        c8 = (mins[1] + maxs[1]) * 0.5
        opening = (c8 - c7)
        if opening.GetLength() < 1e-4:
            raise RuntimeError('finger plates coincide at rest pose -- '
                               'cannot identify inner faces')
        opening.Normalize()
        up = Gf.Vec3d(0, 0, 1)
        along_jaw = (opening ^ up)  # third axis of the face plane
        along_jaw.Normalize()
        print(f'### opening axis=({opening[0]:.3f},{opening[1]:.3f},{opening[2]:.3f}) '
              f'along-jaw=({along_jaw[0]:.3f},{along_jaw[1]:.3f},{along_jaw[2]:.3f})',
              flush=True)

        # --- verify links carry explicit mass (pads must not change it) ---
        for link_name in ('link7', 'link8'):
            link = None
            for p in paths:
                prim = stage.GetPrimAtPath(p)
                anc = prim.GetParent().GetParent()  # .../linkN/linkN_1/<mesh>
                if anc.GetName() == link_name:
                    link = anc
                    break
            mass_api = UsdPhysics.MassAPI(link) if link else None
            mass = mass_api.GetMassAttr().Get() if mass_api else None
            print(f'### {link_name} mass={mass} (explicit URDF mass expected)', flush=True)
            if mass is None:
                print('### WARN: no explicit mass on finger link -- pad collision '
                      'geometry could feed PhysX mass inference; aborting --write',
                      flush=True)
                if args.write:
                    return 1

        # --- derive one pad per finger -------------------------------------
        pads = []
        for i, path in enumerate(paths):
            mn, mx = mins[i], maxs[i]
            center_i = (mn + mx) * 0.5
            other = c8 if i == 0 else c7
            # inner face = the bbox extreme closest to the OTHER finger's
            # center along the opening axis
            d_min, d_max = mn * opening, mx * opening
            inner_val = d_max if (other * opening) > (center_i * opening) else d_min
            inner_pt = center_i + opening * (inner_val - center_i * opening)
            # face extents
            width = min(abs((mx - mn) * along_jaw), PAD_MAX_WIDTH)
            height = min(abs((mx - mn) * up), PAD_MAX_HEIGHT)
            if width < 0.01 or height < 0.01:
                raise RuntimeError(f'finger {i} face too small: {width}x{height}')
            # pad center: half embedded past the inner face
            pad_center = inner_pt + opening * (PAD_THICKNESS * (0.5 - PAD_EMBED))
            pads.append((path, pad_center, width, height))
            print(f'### finger{i}: inner face @ dot={inner_val:.4f} '
                  f'face {width:.3f}x{height:.3f} pad_center='
                  f'({pad_center[0]:.4f},{pad_center[1]:.4f},{pad_center[2]:.4f})',
                  flush=True)

        if not args.write:
            print('### measure-only done (no --write: stage untouched)', flush=True)
            return 0

        # --- author ---------------------------------------------------------
        if not os.path.exists(BACKUP):
            shutil.copy2(args.stage, BACKUP)
            print(f'### backed up stage to {BACKUP}', flush=True)

        from pxr import UsdShade

        existing_mat = stage.GetPrimAtPath('/World/Looks/piper_finger_physics')
        if not existing_mat.IsValid():
            raise RuntimeError('finger material missing -- import_robots.py '
                               'finger block has not run on this stage')
        material = UsdShade.Material(existing_mat)

        for i, (path, pad_center, width, height) in enumerate(pads):
            mesh_prim = stage.GetPrimAtPath(path)
            parent = mesh_prim.GetParent()          # .../linkN_1 Xform
            pad_path = parent.GetPath().AppendChild(PAD_PRIM_NAMES[i])
            _deinstance_chain(stage, str(parent.GetPath()))

            old = stage.GetPrimAtPath(pad_path)
            if old.IsValid():
                stage.RemovePrim(pad_path)
            # Cube is Xformable: define it directly and author the xform ops
            # on it (same size=1 + scale-op pattern as build_wall_stage.py's
            # sensor boxes).
            pad = UsdGeom.Cube.Define(stage, pad_path)
            pad.CreateSizeAttr(1.0)

            # world -> parent-local transform of the pad center
            parent_to_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            world_to_parent = parent_to_world.GetInverse()
            # pxr Gf uses ROW-vector convention (translation lives in ROW 3,
            # Transform(p) == p * M): for p' = p*S*R*T the local axis images
            # are the ROWS of R. Cube local axes: x=thickness/opening,
            # y=width/along-jaw, z=height/up.
            r4 = Gf.Matrix4d()
            r4.SetRow(0, Gf.Vec4d(opening[0], opening[1], opening[2], 0.0))
            r4.SetRow(1, Gf.Vec4d(along_jaw[0], along_jaw[1], along_jaw[2], 0.0))
            r4.SetRow(2, Gf.Vec4d(up[0], up[1], up[2], 0.0))
            r4.SetRow(3, Gf.Vec4d(0.0, 0.0, 0.0, 1.0))
            s4 = Gf.Matrix4d()
            s4.SetScale(Gf.Vec3d(PAD_THICKNESS, width, height))
            t4 = Gf.Matrix4d()
            t4.SetTranslate(pad_center)
            # pxr composition: world_row = c * S * R * T * w2p
            m_local = s4 * r4 * t4 * world_to_parent

            xformable = UsdGeom.Xformable(pad)
            op = xformable.AddTransformOp()
            op.Set(m_local)
            # DIAGNOSTIC: intent box from pure vector math (no Gf conventions)
            # vs the authored prim's composed transform
            exp = [pad_center + opening * (sx * PAD_THICKNESS * 0.5)
                   + along_jaw * (sy * width * 0.5) + up * (sz * height * 0.5)
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            exs = [p[0] for p in exp]
            eys = [p[1] for p in exp]
            ezs = [p[2] for p in exp]
            print(f'### pad{i} intent world box x[{min(exs):.4f},{max(exs):.4f}] '
                  f'y[{min(eys):.4f},{max(eys):.4f}] z[{min(ezs):.4f},{max(ezs):.4f}]',
                  flush=True)
            pad_to_world = UsdGeom.Xformable(pad).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            corners = [pad_to_world.Transform(Gf.Vec3d(sx * 0.5, sy * 0.5, sz * 0.5))
                       for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            zs = [c[2] for c in corners]
            print(f'### pad{i} authored world box x[{min(xs):.4f},{max(xs):.4f}] '
                  f'y[{min(ys):.4f},{max(ys):.4f}] z[{min(zs):.4f},{max(zs):.4f}]',
                  flush=True)

            UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
            tuning_prim = pad.GetPrim()
            physics_tuning._float(tuning_prim, 'physxCollision:contactOffset',
                                  PAD_CONTACT_OFFSET)
            physics_tuning._float(tuning_prim, 'physxCollision:restOffset', 0.0)
            physics_tuning.bind_physics_material(tuning_prim, material)

            # verify world bounds of what we just authored
            cache2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       ['default', 'render', 'proxy', 'guide'])
            mn, mx = _world_bounds(stage, str(pad_path), cache2)
            print(f'### authored {pad_path}\n    world min=({mn[0]:.4f},{mn[1]:.4f},{mn[2]:.4f}) '
                  f'max=({mx[0]:.4f},{mx[1]:.4f},{mx[2]:.4f})', flush=True)

        omni.usd.get_context().save_as_stage(args.stage)
        print(f'### saved to {args.stage}', flush=True)
        print('### DONE', flush=True)
        ok = True
        return 0
    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
        return 1
    finally:
        simulation_app.close()
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
