"""Wall-mount demo: Piper picks each of 4 camera-sensor objects off the
robot's own riser cube and magnetically mounts it on a wall. Assumes Isaac
Sim is already running with
kotek_isaac_stage/usd/kotek_scout_piper_wall_demo.usd loaded and playing.

Deliberately narrower than kotek_demo.launch.py: no base_control (the robot
never drives -- it spawns already parked at the wall's standoff, see
build_wall_stage.py), no sensor_pose_publisher or task_coordinator (this
task's poses are fixed/pre-measured, not TF-sensed, and its sequencing is
handled by kotek_wall_mount_task's own script rather than the pick-and-
deliver FSM -- see that package's module docstring for why). Brings up just
the Piper/MoveIt stack (with wall_mount.yaml's grasp_pitch/place_pitch/
approach_distance overrides -- see that file's own comment) and rviz.

After this is up and Isaac Sim is playing, run the task with:
    ros2 run kotek_wall_mount_task wall_mount_task
(kept as a separate, explicitly-run step rather than auto-included here, so
it can't race move_group's own startup.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('kotek_bringup')

    piper_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'piper_moveit.launch.py')),
        launch_arguments={'manipulation_config': 'wall_mount.yaml'}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'wall_mount.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([piper_moveit, rviz])
