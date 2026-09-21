#!/usr/bin/env python3
"""Orchestrates the wall-mount task: 4 pick+magnetic-mount cycles, calling
/piper/grasp_object and /piper/place_object directly.

Deliberately bypasses kotek_task_coordinator's FSM (task_fsm.hpp /
task_coordinator_node.cpp) entirely -- that machinery exists to sense an
external object's pose over TF, drive the base to a pregrasp standoff, then
drive again to a separate delivery point. This task needs none of that: the
4 sensor-cam objects are rigidly mounted on the robot's own riser cube (no
TF sensing needed -- their pose relative to the arm is fixed), and the robot
spawns already parked at the wall's standoff (build_wall_stage.py) with no
re-driving between any of the 4 mounts. A live /piper/compute_ik sweep (with
the wall and the riser cube modeled as real MoveIt CollisionObjects, not
just self-collision) confirmed all 8 target poses below are reachable from
that single spawn pose -- see the wall-mount task's design chat for the full
measurement.

Both GraspObject and PlaceObject take only a raw target POSITION in the
goal (piper_manipulator_node.cpp reads goal->object_pose.pose.position /
goal->place_pose.pose.position and ignores orientation entirely) --
computeGraspPose() derives approach_yaw from atan2(y, x) and applies
grasp_pitch/place_pitch (wall_mount.yaml) internally, so this script only
ever needs to supply the 8 fixed positions below, all in the arm's own
planning frame (piper's "base_link", matching how piper_manipulator_node
already treats every incoming goal -- no TF lookup happens on the arm side).
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from kotek_msgs.action import GraspObject, PlaceObject
from rclpy.action import ActionClient
from rclpy.node import Node

# Riser-corner pickup positions (arm frame) -- corners of
# /World/scout_mini/Geometry/base_link/Cube's top face, inset so the
# 8x3.5x4cm sensor box never overhangs (build_wall_stage.py's CORNER_XY),
# converted into the arm's own base_link frame by subtracting
# coordinator.yaml's arm_mount_xyz=[0,0,0.29101] (translation only, scout
# base_link and the arm's own base_link share x/y origin and orientation).
_CORNER_XY = 0.109
_GRASP_Z = 0.3103 - 0.29101  # SENSOR_CAM_LOCAL_Z - arm_mount_xyz.z
GRASP_POSITIONS = [
    (+_CORNER_XY, +_CORNER_XY, _GRASP_Z),
    (+_CORNER_XY, -_CORNER_XY, _GRASP_Z),
    (-_CORNER_XY, -_CORNER_XY, _GRASP_Z),
    (-_CORNER_XY, +_CORNER_XY, _GRASP_Z),
]

# Wall mount target positions (arm frame) -- build_wall_stage.py's
# WALL_FACE_X/WALL_TARGET_Y/WALL_TARGET_Z (world frame), converted the same
# way: the robot spawns at world origin (x=0,y=0,yaw=0, untouched sublayer
# default -- see build_wall_stage.py's own comment on why it stays that
# way), so world x/y equal arm-frame x/y directly, and world z is converted
# via the chassis's own settled height (~0.18m, physics_tuning.py) plus the
# arm mount offset.
# 0.40 - 0.045 -> 0.30 -> 0.386 (2026-09-21, realism pass, measured twice):
# the carried box is now an AttachedCollisionObject during the place leg
# (piper_manipulator Params::attach_carried_box), so the planner keeps the
# box's corners off the wall by construction. MAGNET_ATTRACT_RANGE is now a
# realistic 0.03 m, so the RELEASE must put the box's magnet face within
# ~2.5cm of its patch (d = sqrt(horiz^2 + vert^2) budget). Smoke-r1 measured
# the actual release geometry at _WALL_X=0.30: face-to-patch d=0.103 m --
# horizontal 10.6cm (box face lands at cmd_x - 0.006: fingertips AT cmd,
# box center +0.014 along the tilted axis, face -0.02) plus ~6cm VERTICAL
# (the old +0.06 _WALL_Z lift, a 10cm-range-era cushion). Working backwards
# with the reoriented carry (corner reach only 3.46cm at the residual
# ~24deg tilt): face gap 0.40 - (cmd - 0.006), corner clearance 0.40 -
# (cmd - 0.0206); cmd=0.386 gives face 2.0cm / corner 3.5cm clearance.
# Vertical: dropping the +0.06 lift puts the face AT patch height (cmd_z =
# 0.35 - 0.18 - 0.29101) and link6 at z = cmd_z + 0.0955 = -0.0255, still
# above the measured -0.05 workspace boundary. The magnet's pull-down
# during release_settle handles the last fraction.
_WALL_X = 0.386
_WALL_Z = 0.35 - 0.18 - 0.29101
PLACE_POSITIONS = [
    (_WALL_X, -0.15, _WALL_Z),
    (_WALL_X, -0.05, _WALL_Z),
    (_WALL_X, 0.05, _WALL_Z),
    (_WALL_X, 0.15, _WALL_Z),
]

ARM_FRAME = 'base_link'


def make_pose(xyz):
    ps = PoseStamped()
    ps.header.frame_id = ARM_FRAME
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
    ps.pose.orientation.w = 1.0
    return ps


class WallMountTask(Node):
    def __init__(self):
        super().__init__('wall_mount_task')
        self.grasp_client = ActionClient(self, GraspObject, '/piper/grasp_object')
        self.place_client = ActionClient(self, PlaceObject, '/piper/place_object')

    def _feedback_cb(self, prefix):
        def cb(feedback_msg):
            fb = feedback_msg.feedback
            self.get_logger().info(f'{prefix}: {fb.stage} ({fb.progress:.0%})')
        return cb

    def run_grasp(self, index, xyz):
        self.get_logger().info(f'--- grasp sensor_cam_{index} at {xyz} ---')
        goal = GraspObject.Goal()
        goal.object_pose = make_pose(xyz)
        goal.lift_height = 0.0  # use piper_manipulator's configured default
        # Task 2b (2026-09-18/20): name the box so piper_manipulator can verify
        # after CLOSE_GRIPPER that the close-kick did not rotate it in the
        # fingers (a 30-50% event on this stage, measured; a rotated box
        # fails wall capture later no matter how well the carry goes). A
        # kick aborts the grasp BEFORE the lift with the box still on the
        # riser -- the retry loop in main() then simply re-grasps.
        goal.object_frame = f'sensor_cam_{index}'
        return self._send_and_wait(self.grasp_client, goal, f'grasp[{index}]')

    def run_place(self, index, xyz):
        self.get_logger().info(f'--- place onto wall_target_{index} at {xyz} ---')
        goal = PlaceObject.Goal()
        goal.place_pose = make_pose(xyz)
        goal.retreat_height = 0.0  # use piper_manipulator's configured default
        # Task 2b (2026-09-18): tell the manipulator WHICH box it is carrying so
        # its pre-release held-box TF check tracks that frame instead of
        # guessing the nearest sensor_cam frame to the TCP (at the stow pose
        # all four riser boxes sit in a tight cone from the TCP -- nearest-
        # guessing mis-latched riser boxes twice in smoke testing).
        goal.object_frame = f'sensor_cam_{index}'
        return self._send_and_wait(self.place_client, goal, f'place[{index}]')

    def _send_and_wait(self, client, goal, label):
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f'{label}: action server not available')
            return False
        send_future = client.send_goal_async(goal, feedback_callback=self._feedback_cb(label))
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label}: goal rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'{label}: FAILED -- {result.message}')
        else:
            self.get_logger().info(f'{label}: succeeded -- {result.message}')
        return result.success


def main():
    rclpy.init()
    node = WallMountTask()
    ok = True
    for i in range(4):
        index = i + 1
        # Re-grasp retry (Task 2b, 2026-09-20): the grasp-close kick rotates
        # the box 30-50% of the time on this stage (chaotic contact
        # dynamics, measured across the e2e10 batches); piper now rejects a
        # kicked grasp before the lift (box still on the riser), so retrying
        # the SAME grasp is cheap and usually seats the box on attempt 2.
        grasp_ok = False
        for attempt in (1, 2, 3):
            if node.run_grasp(index, GRASP_POSITIONS[i]):
                grasp_ok = True
                break
            node.get_logger().warn(
                f'grasp[{index}] attempt {attempt}/3 failed (kick?) -- re-grasping')
        if not grasp_ok:
            ok = False
            break
        if not node.run_place(index, PLACE_POSITIONS[i]):
            ok = False
            break
    node.get_logger().info('wall-mount task ' + ('COMPLETE' if ok else 'ABORTED (see errors above)'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
