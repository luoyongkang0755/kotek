# AGENTS.md — Guidance for AI coding agents working in kotek_ws

## Project overview

`kotek_ws` is a ROS 2 Jazzy workspace for a mobile-manipulation demo: an AgileX
Scout Mini mobile base carrying a Piper robot arm (6 DoF + gripper), simulated in
NVIDIA Isaac Sim 6.0.1. Three scenarios are built on the same robot asset:

1. **Pick-and-delivery** (autonomous): the robot drives to a sensor object
   (`task_coordinator` state machine), aligns/stops, grasps and lifts it with the
   arm, drives to a delivery waypoint, places it, and retreats.
2. **Wall-mount** (`kotek_wall_mount_task`): the parked robot picks 4
   camera-sensor boxes off its own riser and magnetically mounts them on a wall
   (attract-then-weld magnet OmniGraph, 3 camera views).
3. **Teleop** (`kotek_teleop`): SpaceMouse → `/cmd_vel` base driving, real Piper
   leader arm (over SocketCAN `can0`) → simulated arm following, optional
   haptic feedback loop (disabled by default — real hardware under torque
   control).

Deliberately **no Nav2 in v1**: the base is driven by a simple closed-loop
controller (`SimpleBaseController`) against Isaac ground-truth odometry,
publishing `geometry_msgs/Twist` on `/cmd_vel`. It serves the *same*
`nav2_msgs/action/NavigateToPose` action name/fields as Nav2, so swapping in Nav2
later only touches `kotek_bringup/launch/base_control.launch.py` (see README
"Nav2 migration").

Canonical user-facing documentation is `README.md`. Every non-obvious fix and
limitation is backed by a diagnostic trail in
`docs/pick_and_delivery_report.md` and `docs/grasp_fix_report.md` — when
changing sim/asset behavior, check those reports for the root-cause history
before assuming something is a bug.

## Technology stack

- **ROS 2 Jazzy**, C++17 (`rclcpp`) for control/manipulation/coordinator nodes,
  Python (`rclpy`) for bridges, stage tooling, and task scripts.
- **MoveIt** (`ros-jazzy-moveit*`) for arm planning via `MoveGroupInterface`;
  the Piper MoveIt config comes from the external `isaac_simulation` repo
  (`piper_camera_moveit_config`) unmodified.
- **Isaac Sim 6.0.1 (pip install, aarch64 host)** for simulation + USD stage
  authoring. The ROS side runs in Docker; Isaac runs on the host — they talk
  over DDS with `network_mode: host`.
- **USD/usd-core** for stage authoring (`kotek_isaac_stage`).

## Repository layout (src/, all ROS packages)

| Package | Build type | Role |
|---|---|---|
| `kotek_msgs` | ament_cmake | Custom actions: `GraspObject.action`, `PlaceObject.action` |
| `kotek_base_control` | ament_cmake | `simple_base_controller_node`: NavigateToPose server → `/cmd_vel`, with reactive lidar obstacle-avoidance layer (`obstacle_avoidance.hpp`) |
| `kotek_manipulation` | ament_cmake | `piper_manipulator`: GraspObject/PlaceObject action servers via MoveGroupInterface |
| `kotek_task_coordinator` | ament_cmake | `task_coordinator_node`: IDLE→FIND_SENSOR→…→DONE FSM (`task_fsm.hpp`) orchestrating nav + grasp + place actions; services `/task/start`, `/task/abort`; latched `/task/state` |
| `kotek_isaac_bridge` | ament_python | `isaac_joint_bridge` (Isaac JointState ↔ MoveIt FollowJointTrajectory, gripper stall detection) and `sensor_pose_publisher` (Isaac TF → `/sensor/pose`) |
| `kotek_wall_mount_task` | ament_python | Script calling GraspObject/PlaceObject directly for 4 wall-mount cycles (bypasses the coordinator FSM by design) |
| `kotek_teleop` | ament_python | SpaceMouse teleop node, Piper leader-arm node, haptic law, spnav client |
| `kotek_bringup` | ament_cmake | Launch files + all parameter YAMLs (`config/*.yaml`) + rviz configs |
| `kotek_isaac_stage` | ament_python | Isaac-side USD stage authoring/scripts and Tier-3 sim tests (see below) |

External dependency: `isaac_simulation` (GitHub `TitusAwakes/isaac_simulation`,
branch `navigation`) provides `piper_description` and
`piper_camera_moveit_config`. It is **bind-mounted read-only** from the host
clone at `/home/trs/isaac_simulation` into the container (documented in
`kotek.repos` for fresh clones; requires git-lfs + a submodule). It is never
modified, and its `scout_nav2_pkg` is excluded from every build.

## Build, test, run (all ROS work happens inside Docker — no ROS on the host)

Build image once: `cd docker && docker compose -f compose.yaml build`.

**Critical convention: every `docker compose run --rm` starts a fresh
container** — `build/`, `install/`, `log/` do NOT persist between invocations;
only `src/` is bind-mounted. Always chain build + your command in one
`bash -lc '...'` call (see README §2–§5 for the exact patterns).

```bash
# Build workspace
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg'

# Tier 1+2 tests (no Isaac Sim needed): gtest (C++) + pytest (Python)
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg &&
  source /workspace/install/setup.bash &&
  colcon test --packages-select kotek_base_control kotek_manipulation
              kotek_task_coordinator kotek_isaac_bridge kotek_msgs &&
  colcon test-result --all'
```

Inside the container sources are at `/workspace/src/kotek` and
`/workspace/src/isaac_simulation`. Last verified: 121 tests, 0 failures.

**Isaac Sim runs on the host, never in Docker.** Use `./run_isaac.sh <script.py>`
(it selects the working Isaac python at `/home/trs/env_isaaclab/bin/python`,
preloads `libgomp` — required on this aarch64 host — and honors `KOTEK_WITH_ROS=1`
to wire Isaac's bundled ROS 2 libs). Isaac-only scripts run without docker;
anything crossing the Isaac↔container boundary needs
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both sides (FastRTPS matches DDS
endpoints but never delivers messages across this boundary — verified, see
report §4.1). README §6–§7 give the full Tier-3/E2E recipes and the run order
(ROS stack in container + Isaac stage playing on host, `/task/start` or
`wall_mount_task` afterwards).

## Architecture conventions (follow these when adding code)

- **Pure logic is ROS-free; nodes are thin wrappers.** `simple_base_controller`,
  `obstacle_avoidance`, `pregrasp_geometry`, `task_fsm`, `pose_utils` are pure
  C++ headers/libraries with no rclcpp in their interfaces; the `*_node.cpp`
  files own all ROS I/O. Unit tests (gtest) target the pure layer only. Keep it
  that way — new control/geometry logic goes into the pure layer with gtest
  coverage, not into node code.
- **Action-first orchestration.** Base motion is a `NavigateToPose` goal; arm
  work is `GraspObject`/`PlaceObject` goals (position-only in the Piper's own
  `base_link` frame — orientation is ignored; approach yaw is derived inside
  `piper_manipulator`). The coordinator never commands joints directly.
- **Parameters, not constants.** All tunables live in
  `src/kotek_bringup/config/*.yaml`; YAML values carry comments explaining the
  physical/diagnostic reason for each non-default value. New tunables must be
  ROS parameters loaded via `declare_parameter` with a documented default.
- **Comments explain *why*, and cite the diagnostic trail.** The codebase
  convention is rich comments referencing `docs/pick_and_delivery_report.md`
  sections, live-run measurements, and ruled-out hypotheses. Match this: if you
  fix a sim/asset issue, add the guard that would have caught it (see
  `inspect_stage.py --assert-demo-ready`, which permanently asserts the USD
  asset's physics/contact/connection properties).
- **USD assets are generated, not edited by hand.** Robot asset pipeline:
  `import_robots.py` → `author_robot_graphs.py` (needs live Isaac) →
  `build_demo_stage.py`/`build_wall_stage.py` (usd-core only) → verify with
  `inspect_stage.py --assert-demo-ready`. Camera/magnet graph scripts save the
  stage **in place** — never run them while another Isaac process has it open.
- **Lint**: C++ packages use `ament_lint_auto` (with cpplint/copyright
  explicitly skipped where configured); Python packages use
  `ament_flake8`/`ament_pep257`/`ament_copyright` via pytest. `colcon test`
  runs them — keep them green.
- Build style: C++17, `-Wall -Wextra -Wpedantic`, `--symlink-install`.

## Operational gotchas (they bite; check before debugging "bugs")

- **Exactly one Isaac Sim process, ever** — two means two `/clock` publishers
  and a sim-time storm that aborts every MoveIt goal. Check
  `ros2 topic info /clock` (Publisher count must be 1) before any E2E run.
- The whole stack runs on **Isaac sim time**; without Isaac playing, action
  goals hang and wedge move_group (then the container stack needs a restart).
- Warm-up: wait ~45 s after both sides are up before sending the first arm
  goal, or MoveIt's current state isn't synced yet (instant abort).
- `ROS_DOMAIN_ID` must match on both sides for any demo; Isaac's bridge reads
  the env var at launch, so export it *in front of* `./run_isaac.sh`. The
  host's real-robot teleop container lives on domain 0 — never run the demos
  there.
- Re-running a demo needs fresh state on **both** sides (restart Isaac *and*
  the container stack).
- `piper_manipulator` cancel handling is best-effort (synchronous
  MoveGroupInterface calls can't be interrupted mid-call).
- Teleop and autonomous stacks must never run together: `kotek_teleop.launch.py`
  deliberately omits `base_control`/`piper_moveit` and its nodes refuse to
  start if they see a competing publisher on `/cmd_vel` or
  `/isaac_joint_commands`.

## Testing strategy summary

- **Tier 1** — gtest/pytest unit tests of pure logic (control law, obstacle
  avoidance, pose utils, pregrasp geometry, FSM transitions, teleop mapping).
  Run via `colcon test` in the container; no sim needed.
- **Tier 2** — launch smoke tests (`ros2 launch kotek_bringup kotek_demo.launch.py`)
  verifying all nodes come up together without Isaac.
- **Tier 3** — real Isaac Sim tests in `kotek_isaac_stage`:
  `tier3_regression_test.py` (drift + commanded drive + arm convergence),
  `tier3_obstacle_test.py` (replays a captured scan from `real_scan.json`
  against the real compiled controller), `pick_and_delivery_e2e_test.py`
  (full live E2E). The wall-mount demo's E2E is run manually via
  `run_wall_stage.py` + `wall_mount_task` (README §7).
