"""Human-in-the-loop teleop: SpaceMouse -> base, real Piper leader arm -> sim
arm, with an optional felt-contact haptic loop back onto the leader
(disabled by default; see config/teleop.yaml).

Deliberately does NOT include base_control.launch.py or
piper_moveit.launch.py (which pulls in isaac_joint_bridge + move_group +
task_coordinator's arm path): those own /cmd_vel and /isaac_joint_commands
respectively for the AUTONOMOUS demo, and would fight the teleop nodes over
the same topics. spacemouse_teleop_node and piper_leader_node each refuse to
start cleanly (loud log, not a crash) if they detect a competing publisher
on their topic -- see their _check_no_competing_publisher() methods -- as a
second line of defense against running both launch files at once.

Needs, on the host side (not started by this launch file):
  - spacenavd running (systemd service) with /run/spnav.sock bind-mounted
    into the container (docker/compose.yaml)
  - can0 up at 1Mbit (scripts/piper/can_activate.sh in the kotek_teleop repo)
  - a live Isaac Sim process running teleop_runner.py
    (KOTEK_WITH_ROS=1 ./run_isaac.sh
     src/kotek_isaac_stage/kotek_isaac_stage/teleop_runner.py --gui)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('kotek_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'teleop.yaml')

    spacemouse_node = Node(
        package='kotek_teleop',
        executable='spacemouse_teleop_node',
        name='spacemouse_teleop_node',
        output='screen',
        parameters=[params_file],
    )

    piper_leader = Node(
        package='kotek_teleop',
        executable='piper_leader_node',
        name='piper_leader_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([spacemouse_node, piper_leader])
