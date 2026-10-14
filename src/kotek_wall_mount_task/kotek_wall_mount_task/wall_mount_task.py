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

# Rear-pair pickup positions (arm frame) -- 2026-10-14 USER-DIRECTED
# revert to the proven-working configuration: the pre-platform-layout era
# (arm centered, boxes at arm-frame (-0.109, +-0.109)) ran the wall task
# to full success with the ORIGINAL motion logic (axis-snapped approach,
# 0.018 close, standard lift -- no near-column special casing). The only
# real-bench change kept is the arm's measured position (scout x=+0.06);
# the boxes move forward by the same amount so the arm-frame pickup
# geometry is EXACTLY the era's: scout x -0.049 - arm 0.06 = -0.109. Only
# the rear pair is kept (user: two boxes, rear only), so no box is near
# the arm column and every near-band special in wall_mount.yaml is
# disabled. y=0.1065 (platform-width 1mm-edge inset, 2.5mm inside the
# era's 0.109 -- negligible). z unchanged.
_GRASP_X = -0.049 - 0.06   # -0.109 arm frame (era value)
_GRASP_Y = 0.1065
_GRASP_Z = 0.3103 - 0.29101  # SENSOR_CAM_LOCAL_Z - arm_mount_xyz.z (z unchanged)
GRASP_POSITIONS = [
    (_GRASP_X, +_GRASP_Y, _GRASP_Z),
    (_GRASP_X, -_GRASP_Y, _GRASP_Z),
]

# Wall mount target positions (arm frame) -- build_wall_stage.py's
# WALL_FACE_X/WALL_TARGETS (world frame), converted the same way: the
# robot spawns at world origin (x=0,y=0,yaw=0, untouched sublayer
# default), so world x/y equal arm-frame x/y directly, and world z is
# converted via the chassis's own settled height (~0.18m) plus the arm
# mount offset. 2026-10-14 user-directed revert: the wall moved AWAY with
# the arm (world 0.46 = era face 0.40 + arm 0.06), keeping the era's
# arm-frame distances EXACTLY -- face 0.40, release _WALL_X 0.39 (1cm
# short of the face, the era's corner-reach clearance), targets the era's
# outer pair y=+-0.15, z=0.35. The arm replays the original trajectories.
# Recalibrate against the real wall position on the real robot (same
# measure-don't-assume rule as ever).
_WALL_X = 0.39     # = 0.46 face - 0.06 arm - 0.01 clearance (era value)
_WALL_Z = 0.3734 - 0.18 - 0.29101
PLACE_POSITIONS = [
    (_WALL_X, -0.15, _WALL_Z),
    (_WALL_X, +0.15, _WALL_Z),
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
    for i in range(len(GRASP_POSITIONS)):
        index = i + 1
        # Re-grasp retry (Task 2b, 2026-09-20): the grasp-close kick rotates
        # the box 30-50% of the time on this stage (chaotic contact
        # dynamics, measured across the e2e10 batches); piper now rejects a
        # kicked grasp before the lift (box still on the riser), so retrying
        # the SAME grasp is cheap and usually seats the box on attempt 2.
        #
        # Cycle retry (2026-10-14, measured): the near-column weak squeeze
        # (2-5mm, see wall_mount.yaml's grasp_width_near_column) can slip
        # during the preplace swing -- the box falls back onto the riser
        # near its corner (v6: held-box check fired at place with the box
        # found back at its pickup corner). A failed place no longer aborts
        # the whole task: the cycle (re-grasp via live TF + place) repeats
        # once, which turns the stochastic slip into a recoverable event
        # instead of a demo-killer.
        cycle_ok = False
        for cycle in (1, 2):
            grasp_ok = False
            for attempt in (1, 2, 3):
                if node.run_grasp(index, GRASP_POSITIONS[i]):
                    grasp_ok = True
                    break
                node.get_logger().warn(
                    f'grasp[{index}] attempt {attempt}/3 failed (kick?) -- re-grasping')
            if not grasp_ok:
                break
            if node.run_place(index, PLACE_POSITIONS[i]):
                cycle_ok = True
                break
            node.get_logger().warn(
                f'place[{index}] cycle {cycle}/2 failed (slip?) -- re-grasping')
        if not cycle_ok:
            ok = False
            break
    node.get_logger().info('wall-mount task ' + ('COMPLETE' if ok else 'ABORTED (see errors above)'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
