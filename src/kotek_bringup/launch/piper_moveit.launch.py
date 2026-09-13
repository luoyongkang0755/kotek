"""Piper arm stack: robot_state_publisher + move_group (from the existing,
unmodified piper_camera_moveit_config) + the isaac_joint_bridge that makes
MoveIt's expected arm_controller/gripper_controller FollowJointTrajectory
servers exist against Isaac's raw isaac_joint_states/isaac_joint_commands
topics (plan section 9) + piper_manipulator (the GraspObject server).

Everything here is pushed into the /piper namespace. Two reasons:
  1. TF isolation: tf2_ros subscribes to the RELATIVE topics "tf"/
     "tf_static", so namespacing this whole group automatically isolates
     the arm's TF tree to /piper/tf, /piper/tf_static -- resolving the
     base_link name collision between the scout and the Piper's own URDF
     root link (plan section 1.8/6) without any explicit remap.
  2. move_group's action/topic names, and the two controller names in
     moveit_controllers.yaml (arm_controller, gripper_controller), land at
     /piper/... consistently, and piper_manipulator's grasp_object action
     is exposed at /piper/grasp_object.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_dir = get_package_share_directory('kotek_bringup')

    # Which piper_manipulator param file to load -- defaults to the tuned
    # pedestal pick-and-place config. The wall-mount task's own launch file
    # overrides this to wall_mount.yaml, which carries different
    # grasp_pitch/place_pitch/approach_distance values (see that file's own
    # comment): the pedestal-tuned 0.4 rad gripper pitch doesn't reach either
    # leg of that task (riser-corner pickups need ~1.2 rad, wall places need
    # ~0.7 rad -- confirmed via a live /piper/compute_ik sweep with the wall
    # and riser cube modeled as real MoveIt CollisionObjects).
    manipulation_config_arg = DeclareLaunchArgument(
        'manipulation_config', default_value='manipulation.yaml')
    manipulation_config = PathJoinSubstitution(
        [bringup_dir, 'config', LaunchConfiguration('manipulation_config')])

    moveit_config = (
        MoveItConfigsBuilder('piper_description', package_name='piper_camera_moveit_config')
        .to_moveit_configs()
    )

    # piper_camera_moveit_config's own joint_limits.yaml (read-only submodule,
    # never edited directly -- see kotek_ws project rules) sets
    # has_acceleration_limits: false and max_acceleration: 0 (a bare YAML
    # integer) for every joint (1-8). This isn't just a missing optimization:
    # MoveGroup's own AddTimeOptimalParameterization response adapter
    # *requires* acceleration limits to produce a time-parameterized
    # trajectory at all, and hard-fails without them -- confirmed directly in
    # a live run: every successful plan (including retreatToSafe's "zero"
    # pose) failed with "No acceleration limit was defined for joint jointN!
    # ... Response adapter 'AddTimeOptimalParameterization' failed to
    # generate a trajectory."
    #
    # Overridden below with generous but bounded placeholder values -- the
    # real, physical peak acceleration isn't documented anywhere for this
    # arm, but joint_limits.yaml's own default_acceleration_scaling_factor:
    # 0.1 already scales commanded acceleration down to 10% of whatever
    # ceiling is set here, so the exact value matters far less than simply
    # having one declared. 5.0 rad/s^2 for the 6 arm joints (max_velocity 3.0
    # rad/s -- roughly a 0.6s ramp to full speed) and 2.0 rad/s^2 for the 2
    # gripper joints (max_velocity 1.0 rad/s).
    #
    # Mutated directly on moveit_config.to_dict()'s own dict, in place --
    # NOT passed as a separate, later entry in a node's `parameters=[...]`
    # list. Tried that first and it crashed both move_group and
    # piper_manipulator on startup: "parameter
    # 'robot_description_planning.joint_limits.joint1.max_acceleration' has
    # invalid type: expected [double] got [integer]" -- the original file's
    # bare `0` gets auto-declared as an INTEGER parameter, and a later
    # `--params-file` supplying a `5.0` (double) for the SAME parameter name
    # does not change its already-declared type, it just fails the type
    # check. A single, pre-merged dict with the right values (and right
    # types) from the start avoids the whole multi-file precedence question.
    piper_config_dict = moveit_config.to_dict()
    joint_limits = piper_config_dict['robot_description_planning']['joint_limits']
    arm_and_gripper_accel = (
        [(f'joint{i}', 5.0) for i in range(1, 7)] + [(f'joint{i}', 2.0) for i in range(7, 9)])
    for joint_name, max_accel in arm_and_gripper_accel:
        joint_limits[joint_name]['has_acceleration_limits'] = True
        joint_limits[joint_name]['max_acceleration'] = max_accel

    # piper_camera_moveit_config's own kinematics.yaml (read-only, same rule
    # as above) sets kinematics_solver_timeout: 0.005 -- 5 MILLISECONDS -- for
    # the arm group's KDL numerical IK solver. KDL is an iterative
    # (Newton-Raphson-style) solver; 5ms is nowhere near enough for it to
    # converge from a cold seed. Confirmed directly via /piper/compute_ik:
    # even a trivial, straight-ahead, identity-orientation pose well inside
    # the arm's reach (0.25, 0, 0.3) returned error_code=-31
    # (NO_IK_SOLUTION) -- not a hard kinematic limit, a solver-timeout
    # artifact (a live `ros2 param set` bump to 0.5s couldn't be applied at
    # runtime -- "Setting parameter failed", the plugin reads this once at
    # load time -- confirming this needs a launch-time override, not a
    # runtime one). Raised to 0.5s here, using the same in-place dict
    # mutation as the joint-acceleration fix above (for the same reason: a
    # separate, later parameters-file entry doesn't reliably override an
    # already-typed/declared value).
    piper_config_dict['robot_description_kinematics']['arm']['kinematics_solver_timeout'] = 0.5

    # Isaac Sim publishes /clock with simulation time -- every node here
    # needs use_sim_time so its own now()-stamped state (TF, planning
    # timeouts, trajectory execution timing) stays consistent with Isaac's
    # sim-time-stamped data (see base_control.launch.py's comment for the
    # concrete failure this fixes).
    sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, sim_time],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[piper_config_dict, sim_time],
    )

    isaac_joint_bridge = Node(
        package='kotek_isaac_bridge',
        executable='isaac_joint_bridge',
        name='isaac_joint_bridge',
        output='screen',
        # moveit_config.robot_description (the same URDF move_group and
        # piper_manipulator get, above) lets the bridge clamp Isaac's raw
        # /isaac_joint_states to the URDF's own declared joint limits before
        # republishing them as MoveIt's current state -- fixes
        # START_STATE_INVALID at PLACE_ARM/STOW_OBJECT, where PhysX settles
        # joint2 a hair past its lower bound of exactly 0. Passing the same
        # dict move_group uses means the bounds enforced here can never
        # drift from the bounds move_group itself checks against.
        parameters=[
            os.path.join(bringup_dir, 'config', 'isaac_bridge.yaml'),
            moveit_config.robot_description,
            sim_time,
        ],
    )

    piper_manipulator = Node(
        package='kotek_manipulation',
        executable='piper_manipulator_node',
        name='piper_manipulator',
        output='screen',
        # MoveGroupInterface builds its own RobotModel from parameters on
        # THIS node -- it does not fetch robot_description/
        # robot_description_semantic from move_group remotely by default
        # (confirmed by a real launch: without this it times out after 10s
        # waiting on a robot_description_semantic topic that move_group
        # never publishes). piper_config_dict carries the same
        # robot_description/_semantic/_kinematics/_planning (plus the joint
        # acceleration-limit fix above) that move_group itself was given.
        parameters=[
            piper_config_dict,
            manipulation_config,
            sim_time,
        ],
    )

    piper_group = GroupAction([
        PushRosNamespace('piper'),
        robot_state_publisher,
        move_group,
        isaac_joint_bridge,
        piper_manipulator,
    ])

    return LaunchDescription([manipulation_config_arg, piper_group])
