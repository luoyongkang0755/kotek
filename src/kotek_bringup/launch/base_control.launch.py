"""Mobile base control -- the ONE file that changes for the Nav2 migration.

v1: SimpleBaseController hosts nav2_msgs/action/NavigateToPose on the action
name "navigate_to_pose" directly against Isaac's ground-truth odometry/TF.

v2 (future): delete the Node below and instead
IncludeLaunchDescription(.../scout_nav2_pkg/launch/bt.launch.py) (or a
plain Nav2 bringup). Nothing else in this workspace changes -- the
coordinator, manipulator, and bridge nodes only ever talk to the
"navigate_to_pose" action, and Nav2 exposes the exact same action name and
type (see plan section 17).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('kotek_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'base_controller.yaml')

    simple_base_controller = Node(
        package='kotek_base_control',
        executable='simple_base_controller_node',
        name='simple_base_controller',
        output='screen',
        # Isaac Sim publishes /clock with simulation time, not wall time --
        # without this, this node's now()-stamped goal/pose-timeout logic
        # and TF lookups desync from Isaac's sim-time-stamped transforms
        # (confirmed directly: a TF lookup failed with "Requested time
        # 30.4 but the latest data is at time 0.0" using the node's default
        # wall-clock time against Isaac's sim-time-stamped /tf).
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([simple_base_controller])
