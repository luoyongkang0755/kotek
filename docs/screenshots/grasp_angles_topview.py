#!/usr/bin/env python3
"""Top-view diagram of the three grasp angles for the wall-mount demo.

Angles (all about the world +Z axis, arm base frame):
  psi_obj : object yaw in the world frame (box spawns at 0, kick rotates it)
  psi_app : approach-vector yaw = direction base->object, raw atan2(y, x),
            SNAPPED to the nearest multiple of grasp_yaw_snap_step (= pi)
  psi_tcp : gripper/TCP yaw. The TCP frame is BUILT from the approach vector
            (pregrasp_geometry.hpp computeGraspPose):
              TCP +Z = approach_dir  (tilted down by grasp_pitch, 1.2 rad)
              TCP +X = opening axis  = normalize(cross(world_up, approach))
            so psi_tcp == psi_app (mod pi) BY CONSTRUCTION; the opening axis
            sits at psi_app + 90 deg, straddling the box's 3.5 cm Y width.

Real numbers (live run, e2e10_holddown_slowclose2 run_01 stack log):
  sensor_cam_1 corner (0.109, 0.109):
    raw psi_app = atan2(0.109, 0.109) = +45 deg -> snapped 0
    grasp_pose.position = (0.0589, 0.1079)  == object - 0.13503 * approach
  rear corner (-0.128, 0.132) after a kick moved the box:
    raw psi_app ~ 134 deg -> snapped 180
    grasp_pose.position = (-0.0795, 0.1323)
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Arc, Circle
import numpy as np

DEG = 180.0 / np.pi
PITCH = 1.2                      # grasp_pitch (rad) -- noted, not shown top-view
FINGERTIP_OFFSET = 0.13503       # link6 origin -> fingertips, along approach
BOX_X, BOX_Y = 0.08, 0.035       # sensor box footprint (m)
OBJ1 = (0.109, 0.109)            # sensor_cam_1 authored corner
OBJ3 = (-0.128, 0.132)           # sensor_cam_3 after kick (reactive re-grasp)

C_OBJ = "#1f77b4"   # blue  : object yaw
C_APP = "#d62728"   # red   : approach yaw
C_TCP = "#2ca02c"   # green : gripper/TCP yaw


def rot2(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s], [s, c]])


def draw_box(ax, center, psi_obj):
    """Box footprint + an exterior 'box +X' yaw arrow from the near corner."""
    center = np.array(center)
    corners = np.array([[-BOX_X / 2, -BOX_Y / 2], [BOX_X / 2, -BOX_Y / 2],
                        [BOX_X / 2, BOX_Y / 2], [-BOX_X / 2, BOX_Y / 2]])
    rc = corners @ rot2(psi_obj).T + center
    ax.add_patch(mpatches.Polygon(rc, closed=True, facecolor="#dbe9f6",
                                  edgecolor=C_OBJ, lw=2.0, zorder=3))
    # yaw arrow: starts just outside the -Y-local corner, points along box +X
    c0 = center + rot2(psi_obj) @ np.array([-BOX_X / 2, -BOX_Y / 2 - 0.010])
    d = rot2(psi_obj) @ np.array([1.0, 0.0])
    ax.add_patch(FancyArrowPatch(c0, c0 + 0.055 * d, arrowstyle="-|>",
                                 mutation_scale=11, color=C_OBJ, lw=1.8,
                                 zorder=4))
    return c0, d


def draw_jaws(ax, center, psi_opening, opening=0.04, depth=0.06, gap=0.004):
    """Two parallel jaw plates straddling the box along the opening axis."""
    center = np.array(center)
    d = rot2(psi_opening) @ np.array([1.0, 0.0])
    t = np.array([-d[1], d[0]])
    for sgn in (+1, -1):
        off = sgn * (opening / 2 + gap + BOX_Y / 2) * d
        p0 = center + off - depth / 2 * t
        p1 = center + off + depth / 2 * t
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=C_TCP, lw=5,
                solid_capstyle="butt", zorder=5, alpha=0.85)


def draw_tcp_at_fingertip(ax, center, psi_app, x_lab, z_lab, L=0.038):
    """TCP triad drawn at the fingertip (box center): +X = opening (green,
    solid), +Z projected on XY = approach (red, dashed)."""
    center = np.array(center)
    d_open = rot2(psi_app + np.pi / 2) @ np.array([1.0, 0.0])
    d_appr = rot2(psi_app) @ np.array([1.0, 0.0])
    ax.add_patch(FancyArrowPatch(center, center + L * d_open, arrowstyle="-|>",
                                 mutation_scale=11, color=C_TCP, lw=2.0, zorder=6))
    ax.add_patch(FancyArrowPatch(center, center + L * d_appr, arrowstyle="-|>",
                                 mutation_scale=11, color=C_APP, lw=2.0,
                                 linestyle="--", zorder=6))
    ax.text(*x_lab, "TCP +X (jaw opening)", color=C_TCP, fontsize=8,
            ha="center", va="center", weight="bold")
    ax.text(*z_lab, "TCP +Z proj.\n(approach)", color=C_APP, fontsize=8,
            ha="center", va="center", weight="bold")


def draw_approach_arrow(ax, target_xy, color=C_APP):
    ax.add_patch(FancyArrowPatch((0, 0), target_xy, arrowstyle="-|>",
                                 mutation_scale=16, color=color, lw=2.4,
                                 zorder=4))


def base_frame(ax):
    ax.add_patch(Circle((0, 0), 0.018, facecolor="k", edgecolor="k", zorder=6))
    ax.text(-0.028, -0.030, "arm base\n(base_link)", fontsize=7.5,
            ha="left", va="top")
    ax.add_patch(FancyArrowPatch((0, 0), (0.05, 0), arrowstyle="-|>",
                                 mutation_scale=10, color="k", lw=1.2))
    ax.add_patch(FancyArrowPatch((0, 0), (0, 0.05), arrowstyle="-|>",
                                 mutation_scale=10, color="k", lw=1.2))
    ax.text(0.054, -0.005, "X", fontsize=9)
    ax.text(-0.007, 0.054, "Y", fontsize=9)


def setup_ax(ax, title):
    ax.set_aspect("equal")
    ax.set_xlim(-0.24, 0.24)
    ax.set_ylim(-0.20, 0.24)
    ax.grid(True, ls=":", lw=0.5, alpha=0.6)
    ax.set_xlabel("X [m]  (wall direction)", fontsize=9)
    ax.set_ylabel("Y [m]", fontsize=9)
    ax.set_title(title, fontsize=10.5)


fig, axes = plt.subplots(1, 3, figsize=(16.5, 6.4))
fig.subplots_adjust(bottom=0.15, top=0.85, wspace=0.18)

# ---------------- Panel A: UNSNAPPED (what the snap prevents) ----------------
ax = axes[0]
setup_ax(ax, "A.  WITHOUT snap -- V-grip on the corner (rejected)")
base_frame(ax)
psi_raw = np.arctan2(OBJ1[1], OBJ1[0])          # 45 deg
draw_box(ax, OBJ1, 0.0)
draw_jaws(ax, OBJ1, psi_raw + np.pi / 2)
draw_approach_arrow(ax, OBJ1)
ax.add_patch(Arc((0, 0), 2 * 0.075, 2 * 0.075, angle=0, theta1=0,
                 theta2=psi_raw * DEG, color=C_APP, lw=2.4))
ax.text(0.092 * np.cos(np.radians(22.5)), 0.092 * np.sin(np.radians(22.5)),
        r"$\psi_{app}$=45°", color=C_APP, fontsize=10, ha="center",
        va="center", weight="bold")
ax.add_patch(Arc((0, 0), 2 * 0.04, 2 * 0.04, angle=0, theta1=0, theta2=0.1,
                 color=C_OBJ, lw=2.2))
ax.text(0.052, 0.012, r"$\psi_{obj}$=0°", color=C_OBJ, fontsize=10,
        ha="left", va="center", weight="bold")
c = np.array(OBJ1) + np.array([BOX_X / 2, BOX_Y / 2])
ax.add_patch(Circle(c, 0.012, fill=False, edgecolor="#9467bd", lw=2.5, zorder=6))
ax.annotate("corner contact:\none finger off-center\n-> tumble torque",
            xy=c, xytext=(0.115, -0.150), fontsize=8.5, color="#9467bd",
            arrowprops=dict(arrowstyle="->", color="#9467bd"))
ax.text(0, 0.232, "raw approach yaw = atan2(y, x) = 45°\njaws land 45° to the box faces",
        fontsize=8.5, ha="center", va="top",
        bbox=dict(boxstyle="round", fc="#fdf2f2", ec=C_APP, lw=1))

# ---------------- Panel B: SNAPPED solution, corner 1 ----------------
ax = axes[1]
setup_ax(ax, "B.  WITH snap -- current solution (sensor_cam_1 corner)")
base_frame(ax)
psi_app = 0.0
link6 = np.array(OBJ1) - FINGERTIP_OFFSET * np.cos(PITCH) * rot2(psi_app)[:, 0]
draw_box(ax, OBJ1, 0.0)
draw_jaws(ax, OBJ1, psi_app + np.pi / 2)
draw_tcp_at_fingertip(ax, OBJ1, psi_app, x_lab=(OBJ1[0], 0.170),
                      z_lab=(0.175, 0.086))
draw_approach_arrow(ax, link6)
ax.add_patch(Arc((0, 0), 2 * 0.088, 2 * 0.088, angle=0, theta1=0,
                 theta2=psi_raw * DEG, color="#cccccc", lw=1.6))
ax.text(0.101 * np.cos(np.radians(22.5)), 0.101 * np.sin(np.radians(22.5)),
        "raw 45°", fontsize=8, color="#999999", ha="center", rotation=22.5)
ax.add_patch(Arc((0, 0), 2 * 0.06, 2 * 0.06, angle=0, theta1=0, theta2=0.1,
                 color=C_APP, lw=2.4))
ax.text(0.072, -0.014, r"$\psi_{app}$=0°", color=C_APP, fontsize=10,
        ha="left", va="center", weight="bold")
ax.add_patch(Arc((0, 0), 2 * 0.035, 2 * 0.035, angle=0, theta1=0, theta2=0.1,
                 color=C_OBJ, lw=2.2))
ax.text(0.02, 0.02, r"$\psi_{obj}$=0°", color=C_OBJ, fontsize=10,
        ha="left", va="center", weight="bold")
ax.plot([link6[0], OBJ1[0] - BOX_X / 2], [link6[1], OBJ1[1]], color="0.4",
        lw=1.0, ls=":", zorder=3)
ax.add_patch(Circle(link6, 0.004, facecolor="k", edgecolor="k", zorder=6))
ax.text(link6[0] - 0.024, link6[1] - 0.026,
        "link6 origin (0.135 m\nbehind fingertips)", fontsize=7.5,
        color="0.3", ha="center", va="top")
ax.text(0, 0.232, "snap: round(45°/180°)·180° = 0°  →  ψ_tcp = ψ_app (mod π)\n"
        "jaws ∥ box faces, straddle the 3.5 cm width",
        fontsize=8.5, ha="center", va="top",
        bbox=dict(boxstyle="round", fc="#f2fbf2", ec=C_TCP, lw=1))

# ---------------- Panel C: rear corner, snapped to 180 ----------------
ax = axes[2]
setup_ax(ax, "C.  Rear corner after kick -- reactive re-grasp (sensor_cam_3)")
base_frame(ax)
psi_raw3 = np.arctan2(OBJ3[1], OBJ3[0])          # ~134 deg
psi_app3 = round(psi_raw3 / np.pi) * np.pi       # snap -> pi
link63 = np.array(OBJ3) - FINGERTIP_OFFSET * np.cos(PITCH) * rot2(psi_app3)[:, 0]
draw_box(ax, OBJ3, 0.0)
draw_jaws(ax, OBJ3, psi_app3 + np.pi / 2)
draw_tcp_at_fingertip(ax, OBJ3, psi_app3, x_lab=(-0.128, 0.062),
                      z_lab=(-0.215, 0.155), L=0.034)
draw_approach_arrow(ax, link63)
ax.add_patch(Arc((0, 0), 2 * 0.075, 2 * 0.075, angle=0, theta1=0,
                 theta2=psi_raw3 * DEG, color="#cccccc", lw=1.6))
ax.text(0.088 * np.cos(np.radians(60)), 0.088 * np.sin(np.radians(60)),
        "raw 134°", fontsize=8, color="#999999", ha="center", rotation=60)
ax.add_patch(Arc((0, 0), 2 * 0.055, 2 * 0.055, angle=0, theta1=0,
                 theta2=180, color=C_APP, lw=2.4))
ax.text(-0.115, -0.038, r"$\psi_{app}$=180°", color=C_APP, fontsize=10,
        ha="center", va="center", weight="bold")
ax.add_patch(Arc((0, 0), 2 * 0.035, 2 * 0.035, angle=0, theta1=0, theta2=0.1,
                 color=C_OBJ, lw=2.2))
ax.text(0.02, 0.02, r"$\psi_{obj}$=0°", color=C_OBJ, fontsize=10,
        ha="left", va="center", weight="bold")
ax.plot([link63[0], OBJ3[0] + BOX_X / 2], [link63[1], OBJ3[1]], color="0.4",
        lw=1.0, ls=":", zorder=3)
ax.add_patch(Circle(link63, 0.004, facecolor="k", edgecolor="k", zorder=6))
ax.text(0, 0.232, "snap: round(134°/180°)·180° = 180°  →  ψ_tcp = 180°\n"
        "grasp target = box TF center after kick\n(live: 0.025 m off authored corner)",
        fontsize=8.5, ha="center", va="top",
        bbox=dict(boxstyle="round", fc="#f2fbf2", ec=C_TCP, lw=1))

# ---------------- legend + summary ----------------
handles = [
    mpatches.Patch(color=C_OBJ, label=r"object yaw $\psi_{obj}$ (world frame)"),
    mpatches.Patch(color=C_APP, label=r"approach yaw $\psi_{app}$ = snap(atan2(y,x), π)"),
    mpatches.Patch(color=C_TCP, label=r"gripper/TCP yaw $\psi_{tcp}$ = $\psi_{app}$ (mod π)"),
    mpatches.Patch(fc="#dbe9f6", ec=C_OBJ, label="box 8×3.5 cm (magnet face down)"),
    plt.Line2D([0], [0], color=C_TCP, lw=5, label="jaw plates (open 4 cm)"),
]
fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=9,
           frameon=True, bbox_to_anchor=(0.5, 0.005))
fig.suptitle("Grasp angles, top view (arm base_link frame) -- why the approach yaw is snapped to {0°, 180°}",
             fontsize=12.5, weight="bold")
fig.text(0.5, 0.905,
         r"TCP frame built from the approach vector: +Z = approach (pitched down 1.2 rad), +X = opening = up × approach."
         "  Box yaw is 0 at spawn; a close-kick rotates it -> |ψ_obj − ψ_tcp| is the kick-check angle.",
         fontsize=9, ha="center")

out = "/home/trs/kotek_ws/docs/screenshots/grasp_angles_topview.png"
fig.savefig(out, dpi=160)
print("saved", out)
