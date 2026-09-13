"""Top-level launch for the kotek Scout Mini + Piper pick demo (v1, no
Nav2). Assumes Isaac Sim is already running with
kotek_isaac_stage/usd/kotek_scout_piper_demo.usd loaded and playing (see
kotek_ws/README.md for exact steps).

sensor_pose_publisher runs in the GLOBAL namespace (not /piper): it relates
Isaac's global `world` frame to the scout's `odom` frame, both of which
live on the unnamespaced /tf that Isaac itself publishes to -- it has
nothing to do with the Piper's isolated /piper/tf tree.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('kotek_bringup')

    base_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'base_control.launch.py')))

    piper_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'piper_moveit.launch.py')))

    # Isaac Sim publishes /clock with simulation time -- see
    # base_control.launch.py's comment for the concrete TF-lookup failure
    # this fixes (a wall-clock-timestamped lookup against Isaac's
    # sim-time-stamped /tf never resolves).
    sim_time = {'use_sim_time': True}

    sensor_pose_publisher = Node(
        package='kotek_isaac_bridge',
        executable='sensor_pose_publisher',
        name='sensor_pose_publisher',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'isaac_bridge.yaml'), sim_time],
    )

    task_coordinator = Node(
        package='kotek_task_coordinator',
        executable='task_coordinator_node',
        name='task_coordinator',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'coordinator.yaml'), sim_time],
    )

    return LaunchDescription([
        base_control,
        piper_moveit,
        sensor_pose_publisher,
        task_coordinator,
    ])
