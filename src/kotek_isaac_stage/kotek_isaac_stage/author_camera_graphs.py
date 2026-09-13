#!/usr/bin/env python3
"""Authors the 3 camera sets for the wall-mount task (wrist, external, and
one per sensor-cam object) on the wall-demo stage produced by
build_wall_stage.py.

Runs INSIDE Isaac Sim (needs `isaacsim.core.nodes.IsaacCreateRenderProduct`
and `isaacsim.ros2.bridge.ROS2CameraHelper`, both dynamically-typed-port
node types that only resolve correctly inside a live Isaac Sim process --
confirmed directly against this install's OGN definitions, same class of
constraint author_robot_graphs.py already documents for ConstructArray).
Uses the real `omni.graph.core.Controller` API, the same
CREATE_NODES-then-CONNECT-in-a-second-edit()-call pattern as
author_robot_graphs.py's build_scout_drive_graph -- see that function's own
comments for why short node names only resolve within the SAME edit() call.

Run via (from kotek_ws, needs the ROS 2 bridge extension's bundled libs):
    KOTEK_WITH_ROS=1 ./run_isaac.sh \\
        src/kotek_isaac_stage/kotek_isaac_stage/author_camera_graphs.py

Must run AFTER build_wall_stage.py (needs the 4 sensor-cam prims to exist)
and BEFORE author_magnet_graph.py (no ordering dependency between the two,
but this is the order the wall-mount task plan documents).
"""
import argparse

from isaacsim import SimulationApp

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')

PIPER_ARM_CHAIN = (
    '/World/piper/Geometry/base_link/link1/link2/link3/link4/link5/link6')
WRIST_CAMERA_PATH = f'{PIPER_ARM_CHAIN}/wrist_camera'
EXTERNAL_CAMERA_PATH = '/World/external_camera'
SENSOR_CAM_PATHS = [
    f'/World/scout_mini/Geometry/base_link/sensor_cam_{i + 1}' for i in range(4)
]

RENDER_WIDTH = 640
RENDER_HEIGHT = 480


def _look_basis_matrix(translation_local, forward_local, up_hint=(0.0, 0.0, 1.0)):
    """Gf.Matrix4d placing a camera at `translation_local` (in its
    parent's local frame) looking along `forward_local` (also parent-local).
    USD cameras look down their own local -Z with +Y up -- same convention
    and construction as ~/Projects/kotek_sim/scene.py:add_camera (reference
    only, reimplemented here since kotek_sim is a separate, out-of-scope
    project), generalized to operate in a PARENT's local frame (link6 or a
    sensor-cam prim) rather than world space, since these cameras are
    parented to moving links.
    """
    import numpy as np
    from pxr import Gf

    fwd = np.array(forward_local, dtype=float)
    fwd = fwd / max(np.linalg.norm(fwd), 1e-9)
    up = np.array(up_hint, dtype=float)
    if abs(float(np.dot(fwd, up))) > 0.99:
        # forward is (near-)parallel to the hint -- fall back to a
        # guaranteed-non-parallel axis rather than producing a degenerate
        # (zero-length) right vector.
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(fwd, up)
    right = right / max(np.linalg.norm(right), 1e-9)
    up = np.cross(right, fwd)
    tx, ty, tz = translation_local
    return Gf.Matrix4d(
        float(right[0]), float(right[1]), float(right[2]), 0.0,
        float(up[0]), float(up[1]), float(up[2]), 0.0,
        float(-fwd[0]), float(-fwd[1]), float(-fwd[2]), 0.0,
        float(tx), float(ty), float(tz), 1.0)


def add_camera_prims(stage):
    """Defines the 6 UsdGeom.Camera prims (plain USD authoring -- Isaac
    Sim's pxr.UsdGeom is the same API build_wall_stage.py already uses
    under plain usd-core). Returns {name: path}."""
    from pxr import UsdGeom

    paths = {}

    # Wrist camera: parented under link6, offset toward the fingertip
    # region (FINGERTIP_OFFSET_LOCAL in robot_config.py is (0,0,0.13503)
    # from link6's own origin -- 0.08 sits comfortably before that, keeping
    # the gripper's own fingers in frame) and looking down link6's local
    # +Z, the confirmed approach/insertion axis (APPROACH_AXIS_LOCAL).
    wrist_cam = UsdGeom.Camera.Define(stage, WRIST_CAMERA_PATH)
    wrist_cam.CreateFocalLengthAttr(18.0)   # wide-ish, for a close-in eye-in-hand view
    UsdGeom.Xformable(wrist_cam.GetPrim()).AddTransformOp().Set(
        _look_basis_matrix((0.0, 0.0, 0.08), (0.0, 0.0, 1.0), up_hint=(0.0, 1.0, 0.0)))
    paths['wrist'] = WRIST_CAMERA_PATH

    # External camera: static, world-frame 3/4 view framing both the robot
    # (parked near the world origin) and the wall (near-face at x=1.0, see
    # build_wall_stage.WALL_FACE_X).
    ext_cam = UsdGeom.Camera.Define(stage, EXTERNAL_CAMERA_PATH)
    ext_cam.CreateFocalLengthAttr(24.0)
    UsdGeom.Xformable(ext_cam.GetPrim()).AddTransformOp().Set(
        _look_basis_matrix((0.6, -1.8, 1.6), (0.0, 1.8, -1.2), up_hint=(0.0, 0.0, 1.0)))
    paths['external'] = EXTERNAL_CAMERA_PATH

    # Sensor cameras: one per sensor-cam object, parented to it so it moves
    # with the object once grasped. Looks out along the sensor's local +Y
    # (the short 3.5cm face), offset just outside that face.
    sy = 0.035
    for i, sensor_path in enumerate(SENSOR_CAM_PATHS):
        cam_path = f'{sensor_path}/camera'
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.CreateFocalLengthAttr(24.0)
        UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(
            _look_basis_matrix((0.0, sy / 2.0 + 0.002, 0.0), (0.0, 1.0, 0.0),
                               up_hint=(0.0, 0.0, 1.0)))
        paths[f'sensor_{i + 1}'] = cam_path

    return paths


def build_camera_graph(og, graph_path, camera_paths):
    """One graph, one OnPlaybackTick + one ROS2Context shared by all 6
    cameras (same reuse pattern as kotek_sensor_graph's single tick/context
    for all TF publishing)."""
    keys = og.Controller.Keys

    specs = [
        ('wrist', camera_paths['wrist'], '/piper/camera/wrist/image_raw', 'wrist_camera'),
        ('external', camera_paths['external'], '/external_camera/image_raw', 'external_camera'),
        ('sensor_1', camera_paths['sensor_1'], '/sensor_camera_1/image_raw', 'sensor_camera_1'),
        ('sensor_2', camera_paths['sensor_2'], '/sensor_camera_2/image_raw', 'sensor_camera_2'),
        ('sensor_3', camera_paths['sensor_3'], '/sensor_camera_3/image_raw', 'sensor_camera_3'),
        ('sensor_4', camera_paths['sensor_4'], '/sensor_camera_4/image_raw', 'sensor_camera_4'),
    ]

    create_nodes = [
        ('tick', 'omni.graph.action.OnPlaybackTick'),
        ('ros2_context', 'isaacsim.ros2.bridge.ROS2Context'),
    ]
    set_values = []
    for key, cam_path, topic, frame_id in specs:
        create_nodes += [
            (f'render_product_{key}', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
            (f'camera_helper_{key}', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
        ]
        set_values += [
            (f'render_product_{key}.inputs:width', RENDER_WIDTH),
            (f'render_product_{key}.inputs:height', RENDER_HEIGHT),
            # "target"-typed input -- set as a list of prim path strings,
            # same pattern author_robot_graphs.py uses for
            # art_ctrl.inputs:targetPrim.
            (f'render_product_{key}.inputs:cameraPrim', [cam_path]),
            (f'camera_helper_{key}.inputs:topicName', topic),
            (f'camera_helper_{key}.inputs:frameId', frame_id),
            (f'camera_helper_{key}.inputs:type', 'rgb'),
        ]

    (graph, _nodes, _, _) = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: create_nodes,
            keys.SET_VALUES: set_values,
        },
    )

    def p(short_ref):
        node, attr = short_ref.split('.', 1)
        return f'{graph_path}/{node}.{attr}'

    connections = []
    for key, *_rest in specs:
        connections += [
            ('tick.outputs:tick', f'render_product_{key}.inputs:execIn'),
            (f'render_product_{key}.outputs:execOut', f'camera_helper_{key}.inputs:execIn'),
            (f'render_product_{key}.outputs:renderProductPath',
             f'camera_helper_{key}.inputs:renderProductPath'),
            ('ros2_context.outputs:context', f'camera_helper_{key}.inputs:context'),
        ]

    og.Controller.edit(
        graph,
        {keys.CONNECT: [(p(src), p(dst)) for src, dst in connections]},
    )
    return graph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    parser.add_argument('--headless', action='store_true', default=True)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': args.headless})

    try:
        import omni.graph.core as og
        import omni.usd
        from isaacsim.core.utils import extensions

        extensions.enable_extension('isaacsim.ros2.bridge')
        simulation_app.update()
        print('### ros2 bridge extension enabled', flush=True)

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        print('### stage opened', flush=True)

        stage = omni.usd.get_context().get_stage()
        camera_paths = add_camera_prims(stage)
        print(f'### camera prims defined: {camera_paths}', flush=True)

        build_camera_graph(og, '/World/kotek_camera_graph', camera_paths)
        simulation_app.update()
        print('### camera graph authored', flush=True)

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
