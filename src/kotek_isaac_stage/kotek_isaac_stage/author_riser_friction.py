#!/usr/bin/env python3
"""Rubber-contact upgrade for the anti-kick D-group experiment (2026-09-28).

Hypothesis (docs/wall_mount_contact_placement.md section 0, unified picture):
the close-kick is dominated by first-contact torque asymmetry, and the
slow-close win comes from a symmetric-contact-formation window whose
effectiveness is bounded by how strongly the box's EXISTING contacts resist
yaw/slip. The box already carries mu=1.0 with frictionCombineMode=max, so
every pair it participates in is already at 1.0 -- the only remaining mu
lever is RUBBER-RUBBER levels above 1.0:
  - box material 1.0/0.9 -> 1.6/1.4 (rubber-coated sensor box)
  - riser Cube gets an explicit physics-purpose rubber material (it is
    currently bound only to the scout import's VISUAL material, so its
    physics friction was PhysX's default 0.5 combined... irrelevantly, since
    the box's combine=max already dictated 1.0 -- binding 1.6 makes the
    pair symmetric and documents intent)
Side benefits: finger-box slip resistance during the self-centering window
also rises (same box material), and the box-riser patch's yaw resistance
roughly doubles (small but aligned with the hypothesis).

Stage edit only (material attrs + one physics-purpose binding); live Isaac,
same save-in-place convention as author_finger_pads.py. Idempotent.
Physics-tuning constants updated in lockstep for future rebuilds.

Run:
    ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_riser_friction.py
"""
import argparse
import os
import shutil
import sys

from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')
BACKUP = '/home/trs/e2e_runs/kotek_scout_piper_wall_demo.before_rubber.usd'

BOX_STATIC = 1.0
BOX_DYNAMIC = 0.9
RISER_STATIC = 1.6
RISER_DYNAMIC = 1.4
BOX_MATERIAL_PATH = '/World/sensor_cam_material'
RISER_RUBBER_PATH = '/World/Looks/riser_rubber_physics'
RISER_CUBE_PATH = '/World/scout_mini/Geometry/base_link/Cube'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': True})
    ok = False
    try:
        import omni.usd
        from pxr import UsdPhysics

        import physics_tuning  # noqa: E402  (after SimulationApp)

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        print('### stage opened', flush=True)

        if not os.path.exists(BACKUP):
            shutil.copy2(args.stage, BACKUP)
            print(f'### backed up stage to {BACKUP}', flush=True)

        # --- box material: raise to rubber-rubber level --------------------
        box_mat = stage.GetPrimAtPath(BOX_MATERIAL_PATH)
        if not box_mat.IsValid():
            raise RuntimeError(f'{BOX_MATERIAL_PATH} missing')
        api = UsdPhysics.MaterialAPI(box_mat)
        before = (api.GetStaticFrictionAttr().Get(), api.GetDynamicFrictionAttr().Get())
        api.CreateStaticFrictionAttr(BOX_STATIC)
        api.CreateDynamicFrictionAttr(BOX_DYNAMIC)
        print(f'### box material {before[0]:.2f}/{before[1]:.2f} -> '
              f'{BOX_STATIC}/{BOX_DYNAMIC}', flush=True)

        # --- riser Cube: explicit physics-purpose rubber binding ------------
        cube = stage.GetPrimAtPath(RISER_CUBE_PATH)
        if not cube.IsValid():
            raise RuntimeError(f'{RISER_CUBE_PATH} missing')
        rubber = physics_tuning.define_physics_material(
            stage, RISER_RUBBER_PATH, RISER_STATIC, RISER_DYNAMIC, 0.0)
        physics_tuning.bind_physics_material(cube, rubber)
        print(f'### riser Cube bound to {RISER_RUBBER_PATH} '
              f'({RISER_STATIC}/{RISER_DYNAMIC}, physics purpose)', flush=True)

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
