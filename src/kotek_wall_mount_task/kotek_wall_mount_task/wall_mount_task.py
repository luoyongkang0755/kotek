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

# Riser-corner pickup positions (arm frame) -- corners of the real-robot
# platform layout (build_wall_stage.py, 2026-10-13, orientation corrected
# 2026-10-14): platform 35cm(x, front-back) x 25cm(y, left-right) centered
# on scout base_link, boxes axis-aligned at (x=+-0.134, y=+-0.1065)
# scout-frame, arm mounted FORWARD at scout x=+0.06 (mounting plate rear
# 19.5cm from the platform rear, plate 8cm deep: -0.175 + 0.195 + 0.04).
# y=0.1065 uses the per-axis 1mm-edge inset (the 2026-10-14 pre-fix 0.084
# pushed the near pair deeper into the arm column's ungraspable crescent --
# see build_wall_stage.py's inset comment and ik_probe15-17).
# Arm frame = scout frame - arm mount: the near pair (cycles 1/2) lands
# IN FRONT of the arm column (arm-frame x=+0.074), the far pair 19.4cm
# BEHIND it (x=-0.194). z is unchanged: the platform top sits at the same
# height as the old riser top.
_GRASP_X_NEAR = 0.134 - 0.06   # +0.074
_GRASP_X_FAR = -0.134 - 0.06   # -0.194
_CORNER_Y = 0.1065
_GRASP_Z = 0.3103 - 0.29101  # SENSOR_CAM_LOCAL_Z - arm_mount_xyz.z (z unchanged)
GRASP_POSITIONS = [
    (_GRASP_X_NEAR, +_CORNER_Y, _GRASP_Z),
    (_GRASP_X_NEAR, -_CORNER_Y, _GRASP_Z),
    (_GRASP_X_FAR, -_CORNER_Y, _GRASP_Z),
    (_GRASP_X_FAR, +_CORNER_Y, _GRASP_Z),
]

# Wall mount target positions (arm frame) -- build_wall_stage.py's
# WALL_FACE_X/WALL_TARGET_Y/WALL_TARGET_Z (world frame), converted the same
# way: the robot spawns at world origin (x=0,y=0,yaw=0, untouched sublayer
# default -- see build_wall_stage.py's own comment on why it stays that
# way), so world x/y equal arm-frame x/y directly, and world z is converted
# via the chassis's own settled height (~0.18m, physics_tuning.py) plus the
# arm mount offset.
# NOT the wall face itself: commanding the object CENTER onto the face
# (x=0.40) drives the pitched box's leading corner (the 8cm axis tilted at
# place_pitch=0.7 reaches ~3.5cm beyond the center along x) INTO the wall
# during MOVE_ARM_TO_PREPLACE -- MoveIt does not model the carried object,
# so the arm keeps driving while the wall pries the fingers open (a live
# monitor trace caught joint7 forced from its 0.019 grip stall to 0.058 as
# the box corner met the wall, then free-fall). This only ever "worked"
# while pre-parking-brake chassis creep happened to pull the whole robot
# 2-3cm back before the first place. Released 4.5cm short instead (corner
# reach + 1cm clearance), the magnet's widened attract range
# (MAGNET_ATTRACT_RANGE=0.08, physics_tuning.py) covers the final hop and
# the weld still engages at its usual ~6mm equilibrium.
# _WALL_Z +0.06 vs the mount target's own height (2026-09-13, measured):
# with the reoriented vertical delivery (place_reorient), the fingertip
# axis tilts down by grasp_pitch and link6 must stay above the arm's
# measured lower workspace boundary (link6 z ~ -0.05 m -- a live OMPL sweep
# grid found everything below that unplannable at the wall standoff). The
# box's own magnet target stays at the authored 0.35m; the released box is
# pulled down onto it during release_settle_time, 5cm is well inside the
# magnet's capture range.
# 2026-10-13 real-robot platform layout (arm at scout x=+0.11), reworked
# 2026-10-14 for the corrected orientation (arm at scout x=+0.06, wall
# still at world x=0.40) -- the arm-frame wall face is 0.34 and the
# release point shifts by the same -0.06 (old 0.39 was face - 0.01; the
# 1cm offset and all the corner-reach reasoning above are unchanged).
# Recalibrate against the real wall position before the wall-mount phase
# on the real robot (same measure-don't-assume rule as _WALL_Z).
# TARGET PAIRING (2026-10-14, measured): the near-column pair (grasped
# first, weak 2-5mm squeeze) takes the NEAR targets (+-0.09 -- a ~2cm
# swing from their +-0.1065 pickup corners), and the firmly-held far pair
# takes the FAR targets (+-0.19). The old pairing sent the weak-carry
# near boxes on the longest possible swing (across the whole arm) and the
# slip rate showed it (v6: box back on the riser after the swing).
_WALL_X = 0.33     # = 0.39 - 0.06 arm-forward shift; face 10mm off at PLACE
_WALL_Z = 0.3734 - 0.18 - 0.29101
# Target y derives from build_wall_stage.py's WALL_TARGET_Y (2026-10-13:
# |y| 0.05 -> 0.09/0.19 after the arm-forward move put the inner release
# poses inside a hard IK dead zone -- see that file's comment).
PLACE_POSITIONS = [
    (_WALL_X, +0.09, _WALL_Z),   # grasp[1] near pair, y=+0.1065 -> near target
    (_WALL_X, -0.09, _WALL_Z),   # grasp[2] near pair, y=-0.1065 -> near target
    (_WALL_X, -0.19, _WALL_Z),   # grasp[3] far pair, y=-0.1065 -> far target
    (_WALL_X, +0.19, _WALL_Z),   # grasp[4] far pair, y=+0.1065 -> far target
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
