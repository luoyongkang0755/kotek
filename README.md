# kotek_ws — Scout Mini + Piper autonomous pick-and-delivery demo (v1, no Nav2)

An AgileX Scout Mini carrying a Piper arm drives to a sensor object, aligns and stops, grasps
and lifts it with the arm, drives to a delivery waypoint, places the object, and retreats — all
coordinated by a ROS 2 state machine, with a reactive lidar-based obstacle-avoidance layer
running underneath the whole drive. Version 1 deliberately does not use Nav2: the base is driven
by a simple closed-loop controller (`kotek_base_control`) against Isaac Sim's ground-truth
odometry, publishing `geometry_msgs/Twist` on `/cmd_vel`. It exposes the same
`nav2_msgs/action/NavigateToPose` interface Nav2 itself would, so it is a drop-in replacement
later (see "Nav2 migration" below).

See `docs/pick_and_delivery_report.md` for the full diagnostic trail behind every fix and
limitation mentioned in this README — root causes, ruled-out hypotheses, and exact reproduction
commands for every claim. This README is the practical "how to run it" companion.

**Update**: the base-drive limitation previously documented here (the chassis not reliably
translating under a commanded `/cmd_vel`) is now fixed and verified — see
`docs/pick_and_delivery_report.md` section 1.7 for the full root-cause trail (three compounding
asset bugs: fragmented wheel collision geometry, a corrupted URDF wheel-inertia value, and a
mirrored left/right joint-orientation sign convention) plus section 4.5 for a follow-up sign fix
that was needed on the rotation/turning path specifically (the translation fix alone inverted
turning). `tier3_regression_test.py`'s commanded-drive check now passes with ~58-60cm of real
displacement against a ~60cm/2s prediction, and direct rotation commands turn in the correct
direction with real, physically-expected (if slower-than-straight-driving) magnitude.

**Update 2**: `navigate_to_pose` now succeeds end to end. The remaining blockers turned out not to
be a control-law issue at all: (1) the progress watchdog didn't account for turn-in-place phases
(fixed), and (2) the robot's own lidar was self-detecting the Piper arm's mount structure,
permanently triggering obstacle-avoidance suppression regardless of any real obstacle (fixed by
raising the lidar's mounting height, with the demo obstacles' heights raised to match so they stay
detectable at the new scan height). See `docs/pick_and_delivery_report.md` sections 4.6 and
4.9-4.10 for the full trail. A live E2E run now reaches `DRIVE_TO_PREGRASP` successfully and
proceeds automatically through the align/stop/arm stages, sending a real grasp goal — the first
time in this project's history. Remaining scope (arm-grasp-and-beyond, or just budgeting more time
for a full run) is narrow and unrelated to the base-drive/obstacle-avoidance work above.

## What's here

```
kotek_ws/
├── kotek.repos                  # documents the isaac_simulation dependency (see below)
├── docker/                      # ROS 2 Jazzy + MoveIt build/run environment
├── docs/pick_and_delivery_report.md   # full diagnostic trail, root causes, reproduction commands
├── run_isaac.sh                 # launches any script with the working Isaac Sim 6.0.1 (aarch64)
├── env_isaaclab/                # DEAD: leftover of the old x86_64 install, no interpreter
└── src/
    ├── kotek_msgs/               GraspObject.action, PlaceObject.action
    ├── kotek_base_control/       SimpleBaseController: NavigateToPose server -> /cmd_vel,
    │                             obstacle_avoidance.hpp: reactive lidar-based safety layer
    ├── kotek_isaac_bridge/       isaac_joint_bridge (MoveIt <-> Isaac JointState),
    │                             sensor_pose_publisher (Isaac TF -> /sensor/pose)
    ├── kotek_manipulation/       piper_manipulator: GraspObject + PlaceObject servers via MoveGroupInterface
    ├── kotek_task_coordinator/   task_coordinator: the IDLE..DONE..delivery state machine
    ├── kotek_wall_mount_task/    wall_mount_task: orchestrates the 4-cycle wall-mount demo
    │                             (section 7) by calling GraspObject/PlaceObject directly
    ├── kotek_bringup/            launch files + parameter configs
    └── kotek_isaac_stage/        USD stage authoring: import_robots.py + author_robot_graphs.py
                                  (need a live Isaac Sim process) rebuild the robot from URDF,
                                  add a lidar sensor; build_demo_stage.py + inspect_stage.py
                                  (usd-core only) layer the demo scene + obstacles on top;
                                  tier3_regression_test.py / tier3_obstacle_test.py /
                                  pick_and_delivery_e2e_test.py are the real Isaac Sim tests
```

`isaac_simulation` (the existing repo at `/home/trs/isaac_simulation`, containing
`piper_description` and `piper_camera_moveit_config`) is bind-mounted read-only into the
container by `docker/compose.yaml` rather than re-cloned — it requires git-lfs and a submodule,
and the clone is already present locally. `kotek.repos` documents the same dependency for anyone
building from a fresh clone.

`scout_nav2_pkg` from that repo is intentionally excluded from the build (v1 does not use it —
see `--packages-skip scout_nav2_pkg` below) and is never modified.

## Prerequisites

- Docker (for the ROS 2 Jazzy + MoveIt build/run environment — this host has no ROS 2 installed)
- `usd-core` (pip) for the stage-authoring step — install into any Python 3.10+ environment:
  `pip install usd-core`
- Isaac Sim 6.0.1.0 for Tier 3 (full sim) runs. **This host was migrated from x86_64 to
  aarch64 (NVIDIA GB10 / DGX Spark), and `kotek_ws/env_isaaclab` did not survive it** --
  only a stray `lib/` directory remains, with no interpreter. The working install is the
  one at `/home/trs/env_isaaclab` (the `DEFAULT_PYTHON` in `run_isaac.sh`). Use `./run_isaac.sh`
  (see below) rather than sourcing an environment by hand: on aarch64, `import isaacsim`
  aborts outright unless `libgomp` is preloaded, and Isaac's startup check string-matches
  the `LD_PRELOAD` path rather than resolving it.

## A note on `RMW_IMPLEMENTATION`

**Use `rmw_cyclonedds_cpp`, not `rmw_fastrtps_cpp`, for anything that crosses between a
host-side Isaac Sim process and the docker container.** This was confirmed directly: with
FastRTPS, DDS discovery/matching between the two succeeds (`get_subscription_count()` reaches 1)
but **no messages are ever actually delivered**, even after 100 publishes over 10 real seconds —
neither `ipc: host` nor any RMW/QoS tweak short of switching implementations fixed it.
CycloneDDS delivers correctly across the same boundary with no other changes. `docker/
compose.yaml` is already set to `rmw_cyclonedds_cpp`; every command below that talks to a live
Isaac Sim process sets it explicitly too. Isaac-Sim-only commands that never talk to the docker
container (e.g. `tier3_regression_test.py`) are unaffected either way. See
`docs/pick_and_delivery_report.md` section 4.1 for the full diagnosis.

## 1. Build the container

```bash
cd /home/trs/kotek_ws/docker
docker compose -f compose.yaml build
```

## 2. Build the workspace

```bash
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --symlink-install --packages-skip scout_nav2_pkg
'
```

Every `docker compose run --rm` call starts a **fresh container** — `build/`, `install/`, and
`log/` do not persist between invocations (only `src/` is bind-mounted). Chain build + your
actual command in one `bash -lc '...'` invocation, as in all examples below.

## 3. Run the tests (Tier 1 + Tier 2 — no Isaac Sim needed)

```bash
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --symlink-install --packages-skip scout_nav2_pkg
  source /workspace/install/setup.bash
  colcon test --packages-select kotek_base_control kotek_manipulation kotek_task_coordinator kotek_isaac_bridge kotek_msgs
  colcon test-result --all
'
```

Verified result at the time of writing: **121 tests, 0 failures** across the base controller's
control law (angle wrap, velocity clamps, convergence, hysteresis) and its new obstacle-avoidance
layer (9 cases: passthrough, hard stop, linear scaling, steering, clamping), the pre-grasp/place
geometry, the task coordinator's full state machine including the delivery leg (that
`MOVE_ARM_TO_PREGRASP`/`PLACE_ARM` are unreachable except through `STOP_BASE`/
`STOP_BASE_AT_DELIVERY`), and `isaac_joint_bridge` against a fake-Isaac stand-in node.

## 4. The robot asset: rebuilt from URDF, plus a lidar sensor

**Why rebuilt from URDF.** The original source stage (`scout_and_piper_ros2_final.usd`) made the
Scout Mini drive backward and veer hard the instant Play was pressed, with no `/cmd_vel`
publisher connected at all — traced to 8 dangling OmniGraph connections cloned from an unrelated
robot template. Fixed by rebuilding both robots from their URDFs (never modified) through Isaac
Sim's real URDF importer, then authoring fresh OmniGraphs with single connections only.
`inspect_stage.py` has a permanent regression check for this corruption class (every
`OmniGraphNode` connection must be single and must target a prim that actually exists).

**Also added**: a forward-facing PhysX lidar (270°, 1° resolution, 10Hz, `/scan`) for reactive
obstacle avoidance, and a torque-authority fix for the Piper's `joint3` (see the report, section
1.2). Three scripts, run in order, each needing a live Isaac Sim process:

```bash
cd /home/trs/kotek_ws
# KOTEK_WITH_ROS=1 adds Isaac's own bundled ROS 2 libraries to LD_LIBRARY_PATH,
# which the ROS 2 bridge extension needs before author_robot_graphs.py can
# create any isaacsim.ros2.bridge.* OmniGraph node.
./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/import_robots.py
KOTEK_WITH_ROS=1 ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_robot_graphs.py
```

This produces `usd/kotek_scout_piper_robot.usd`. Then `build_demo_stage.py` sublayers it (never
edits it) to add the sensor object + pedestal + two static obstacles + the `world -> sensor` TF
graph, verified with `inspect_stage.py` under plain `usd-core` (no Isaac Sim needed for this
step):

```bash
python3 -m venv /tmp/usdenv && /tmp/usdenv/bin/pip install usd-core
/tmp/usdenv/bin/python build_demo_stage.py \
  --output /home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd
/tmp/usdenv/bin/python inspect_stage.py \
  /home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd \
  --assert-demo-ready
```

Verified result: **all checks pass**, including the dangling/duplicate-connection regression
guard and the drive-command chain's stale-default check.

## 5. Launch smoke tests (verified against the real build, no Isaac Sim)

```bash
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /workspace && colcon build --symlink-install --packages-skip scout_nav2_pkg
  source /workspace/install/setup.bash
  ros2 launch kotek_bringup kotek_demo.launch.py
'
```

Confirmed: `simple_base_controller`, `robot_state_publisher`, `move_group` ("You can start
planning now!" — the existing `piper_camera_moveit_config` loads and plans with **zero
modifications**), `isaac_joint_bridge`, `piper_manipulator` (both `/piper/grasp_object` and
`/piper/place_object` servers), `sensor_pose_publisher`, and `task_coordinator` all start
cleanly together. Calling `/task/start` with no Isaac Sim connected correctly drives the FSM
`IDLE -> FIND_SENSOR -> FAILED` after `sensor_wait_timeout` (10s, since no `/sensor/pose` is
being published).

## 6. Tier 3 tests (need Isaac Sim)

### 6.1 Core regression test (arm + drive)

```bash
cd /home/trs/kotek_ws
KOTEK_WITH_ROS=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/tier3_regression_test.py
```

Verified result: **Test 1** (zero-`/cmd_vel` drift) and **Test 3** (arm joint convergence, all 12
checks) pass. **Test 2** (commanded straight-line drive) fails — this is the base-translation
limitation described up top; see `docs/pick_and_delivery_report.md` section 1.3 for the full,
exhaustive diagnostic trail (11 ruled-out hypotheses).

### 6.2 Obstacle-avoidance integration test (real compiled node, real captured scan)

```bash
# 1. Capture one real lidar scan from live physics (env_isaaclab, same env as above):
python capture_real_scan.py

# 2. Replay it against the real compiled simple_base_controller_node (docker container):
cd /home/trs/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-select kotek_base_control &&
  source install/setup.bash &&
  python3 src/kotek/kotek_isaac_stage/kotek_isaac_stage/tier3_obstacle_test.py'
```

Verified result: the real, compiled avoidance code suppresses forward `/cmd_vel` to near-zero
(161/161 samples below half of max speed) given a real captured obstacle scan at 0.251m. This
test replays a captured scan rather than streaming live from Isaac Sim because of the
cross-process DDS limitation above (RMW fix does not apply here since this test runs its
consumer entirely inside the container) — see the report section 2.4.

### 6.3 Full pick-and-delivery end-to-end test (real Isaac Sim + real ROS 2 stack, live)

Two processes, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on **both** sides:

```bash
# Process 1 (docker container):
cd /home/trs/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg &&
  source install/setup.bash && ros2 launch kotek_bringup kotek_demo.launch.py'

# Process 2 (host, env_isaaclab), once move_group logs "You can start planning now!":
cd /home/trs/kotek_ws
KOTEK_WITH_ROS=1 ./run_isaac.sh \
  src/kotek_isaac_stage/kotek_isaac_stage/pick_and_delivery_e2e_test.py --timeout 90

# Once "physics playing" appears and ~15s have passed (DDS discovery warm-up), from the
# container: ros2 service call /task/start std_srvs/srv/Trigger "{}"
```

Verified result: `/task/state` walks `IDLE -> FIND_SENSOR -> COMPUTE_BASE_PREGRASP_POSE ->
DRIVE_TO_PREGRASP -> FAILED`, aborting with `simple_base_controller`'s own safety check
(`Insufficient progress (0.044m in 10.0s)`) — the exact, already-documented base-translation
signature, not a new bug. Sensor discovery, pregrasp reachability, nav-goal dispatch/monitoring,
and cross-process DDS + sim-time synchronization are all confirmed working live. If/when the
base-translation limitation is resolved, this same test should be expected to progress through
the full `... -> STOP_BASE -> MOVE_ARM_TO_PREGRASP -> ... -> LIFT_OBJECT -> DRIVE_TO_DELIVERY ->
... -> PLACE_ARM -> OPEN_GRIPPER -> RETREAT_ARM -> DONE` sequence with no further code changes.

### GUI, for interactive/visual confirmation

```bash
cd /home/trs/kotek_ws
KOTEK_WITH_ROS=1 ./run_isaac.sh -m isaacsim isaacsim.exp.base.python.kit \
  /home/trs/kotek_ws/src/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd
# then press Play
```

`docker compose` uses `network_mode: host`, so ROS 2 (DDS) discovery reaches Isaac Sim's ROS 2
bridge directly — no explicit `ROS_DOMAIN_ID` coordination needed beyond both sides using the
default (0), and both sides using `rmw_cyclonedds_cpp` (see above). Standalone Isaac Sim scripts
additionally need `LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../isaacsim/exts/isaacsim.ros2.core/jazzy/
lib` for its internal `rclpy` to load (the GUI app wires this automatically; a standalone script
does not).

## 7. The wall-mount demo (4 camera-sensors, magnetic attach, 3 camera views)

The second demo on the same robot: 4 camera-sensor boxes (8×3.5×4 cm) ride a riser cube on the
Scout's chassis; the Piper picks each one and mounts it on a wall via simulated magnetism
(attract-then-weld, `author_magnet_graph.py`), with three live camera feeds (wrist,
sensor-in-hand, external). The robot spawns parked at the wall standoff and never drives — a
world-anchored "parking brake" joint pins the chassis (see `build_wall_stage.py`'s
`add_parking_brake` for why that matters).

### 7.1 Build the wall stage (once, and after changing any stage-side constant)

Prerequisite: the shared robot asset exists (section 4's `import_robots.py` +
`author_robot_graphs.py` — unchanged, shared with the pick-and-delivery demo).

```bash
cd /home/trs/kotek_ws

# 1. Compose the scene (boxes, wall, targets, parking brake, lighting, TF graph).
#    Plain USD authoring — any Python with pxr works; run_isaac.sh's interpreter has it.
./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/build_wall_stage.py

# 2. Author the camera graphs, then the magnet graph (each needs a live Isaac Sim
#    process; each opens the stage, edits it, and SAVES IT IN PLACE — so never run
#    these while another Isaac Sim has the stage open).
KOTEK_WITH_ROS=1 ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_camera_graphs.py
KOTEK_WITH_ROS=1 ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/author_magnet_graph.py

# 3. Verify everything landed (finger friction material, no riser-hold joints,
#    parking brake, 6 cameras, magnet graph wiring, ...):
./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/inspect_stage.py \
  src/kotek_isaac_stage/usd/kotek_scout_piper_wall_demo.usd --assert-demo-ready
```

### 7.2 Run the demo

Two processes, `rmw_cyclonedds_cpp` on both sides (see the RMW note above). Order matters
only in that both must be fully up before step 3.

**Both sides run on the same `ROS_DOMAIN_ID`, and it is set by environment variable in BOTH
places** — the container's comes from its environment (`compose.yaml` defaults to 0; override
with `-e`), and Isaac's ROS 2 bridge reads the variable at launch, so it must be exported in
front of `run_isaac.sh`. Every verified run used `77`: `network_mode: host` means DDS reaches
everything on this machine, and anything else ROS 2 that may be alive (on this host, a real-arm
teleop container that sits on domain 0!) must never share the demo's domain.

```bash
# 1. ROS 2 stack (docker container):
cd /home/trs/kotek_ws
docker compose -f docker/compose.yaml run --rm -e ROS_DOMAIN_ID=77 kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg &&
  source install/setup.bash && ros2 launch kotek_bringup kotek_wall_mount_demo.launch.py'

# 2. Isaac Sim on the wall stage (host). Headless runner — the reliable path,
#    and the one every verified E2E used (opens the stage, presses Play for you,
#    idles; prints "### physics playing" when the ROS side can see it):
cd /home/trs/kotek_ws
ROS_DOMAIN_ID=77 KOTEK_WITH_ROS=1 ./run_isaac.sh \
  src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage.py --duration 3000

#    GUI alternative, for the visual demo — run_wall_stage_gui.py is the headless
#    runner with headless: False: same stage/extensions/play loop, plus a viewport.
#    It presses Play itself, so a GUI run publishes exactly like the headless one.
#    (Do NOT use "-m isaacsim isaacsim.exp.base.python.kit ...": in the pip-style
#    Isaac Sim 6.0.1 install on this host, `python -m isaacsim` only implements
#    --generate-vscode-settings and silently exits 0 on any other args.)
#    The window renders on the host desktop (display :1, tty2) — not over SSH.
DISPLAY=:1 ROS_DOMAIN_ID=77 KOTEK_WITH_ROS=1 ./run_isaac.sh \
  src/kotek_isaac_stage/kotek_isaac_stage/run_wall_stage_gui.py

# 3. Readiness check (from the container) — do not skip this:
ros2 topic info /clock | grep "Publisher count"      # MUST be exactly 1
ros2 topic hz /piper/joint_states --window 20        # expect ~50-60 Hz (see 7.4:
#    physics runs at EFFECTIVE_PHYSICS_RATE_HZ=60, not the stage-authored 120)
#    0 clock publishers = Isaac not running/not playing; 2 = a second forgotten
#    Isaac (see 7.4) — fix either before continuing.

# 4. WAIT ~45 seconds after BOTH sides are up ("piper_manipulator ready" in the
#    container, "### physics playing" in Isaac). Starting earlier reliably
#    produces an instant MOVE_ARM_TO_PREGRASP abort (status=6) from MoveIt's
#    not-yet-synced current state. Then, from the container:
ros2 run kotek_wall_mount_task wall_mount_task
```

Expected output: `grasp[1..4]` / `place[1..4]` stage-by-stage feedback
(`MOVE_ARM_TO_PREGRASP → ... → LIFT_OBJECT → ... → OPEN_GRIPPER → RETREAT_ARM`), ending in
`wall-mount task COMPLETE`. As each sensor settles at the wall, the magnet graph prints
`### kotek_magnet[...]: welded sensor N ...` on Isaac's console and authors a live
`/World/wall/_magnet_weld_N` FixedJoint — that joint existing is the ground truth for "mounted".

### 7.3 The three camera views

All six cameras publish whenever the stage is playing (no extra ROS process):

- `/piper/camera/wrist/image_raw` — the arm's wrist camera
- `/sensor_camera_1..4/image_raw` — the camera inside each sensor box (the one currently
  being carried is the interesting feed; the others sit on the riser or the wall)
- `/external_camera/image_raw` — static third-person view

From the container: `rviz2 -d /workspace/src/kotek/kotek_bringup/rviz/wall_mount.rviz` shows
the three panels side by side (needs a display/X forwarding). In the Isaac GUI, extra
viewports on the same camera prims are a free alternative.

### 7.4 Rules of engagement / known gotchas

- **Exactly ONE Isaac Sim process, ever.** Two (e.g. a forgotten headless run behind a GUI)
  means two `/clock` publishers; the interleaved clocks throw every node into a continuous
  "Detected jump back in time" storm and MoveIt aborts every goal in ~0.7 s. If anything
  behaves strangely, check `ros2 topic info /clock` — Publisher count must be exactly 1.
- **Shared network?** `network_mode: host` means DDS reaches everything on this machine. If
  any other ROS 2 system is live (especially a real-robot teleop), export the same nonzero
  `ROS_DOMAIN_ID` in the container *and* in front of `run_isaac.sh` for every command above —
  Isaac Sim's bridge reads the env var at launch, so a bare `./run_isaac.sh ...` lands on
  domain 0 no matter what the container is set to. On this host specifically, the teleop
  container that drives the REAL Piper over CAN sits on domain 0: never run the demo there.
- **Re-running the demo** needs fresh state on both sides: restart Isaac (puts the boxes back
  on the riser) *and* restart the container stack, then re-apply the 45 s warm-up. A stack
  that outlives an Isaac restart carries a poisoned TF/time buffer.
- **`move_action result timed out` on the very first arm move, then
  `Cannot push a new trajectory while another is being executed`** = the trajectory was sent
  but nothing executed it: Isaac is not running, or the GUI stage was opened but **Play was
  never pressed**, or it is playing the *wrong* stage (the pick-and-delivery demo's
  `kotek_scout_piper_demo.usd` counts as an Isaac for the one-Isaac rule but does nothing for
  this demo). The whole stack runs on sim time, so without Isaac's `/clock` the bridge's
  execution loop freezes rather than failing fast — and the stuck goal wedges move_group's
  executor, so after fixing the Isaac side you must ALSO restart the container stack before
  retrying. The readiness check in 7.2 step 3 exists to catch all of this beforehand.
- **Mid-carry slips are fixed** (`carry_velocity_scaling`/`carry_acceleration_scaling`:
  loaded carries run at the slow lift scalings, because at the normal 0.2 the preplace
  swing's inertial torque pivoted the box out of the fingers — PhysX point contacts have no
  torsional friction). A box lost anyway now aborts the place loudly
  (`no object held at place start`) instead of silently placing air. Verified: a full run
  with all 4 sensors attached on the wall (3 welded, 1 magnet-held just outside the weld
  gate).
- Boxes self-align before welding: since the 2026-09-07 magnet-model rewrite the magnet is a
  true dipole-dipole interaction (`author_magnet_graph.py`, `physics_tuning.py`), so the
  alignment torque `tau = m x B` falls out of the physics instead of being a disabled tuning
  knob — a box arriving tilted gets flattened onto the wall before the weld fires, and a
  backwards-held box is repelled rather than welded crooked. Verified by
  `probe_magnet_tuning.py --mode misaligned` (30°-tilted arrival must weld at <10°
  misalignment). The old "crooked fridge magnet, by design" behavior is gone.
- **Never apply script-side ANGULAR velocity-feedback forces** (2026-09-08, Bug 9): the
  magnet ScriptNode's tiny angular-damping torque (`-c*omega` through
  `apply_forces_and_torques_at_pos`) froze the sensor boxes solid mid-air in the Newton
  integrator — pose bit-identical for hundreds of ticks, velocity readback pinned at a
  constant (cap) value, no weld. The LINEAR damping twin is freeze-free and stays
  (load-bearing per Bug 2/4). Live-bisected by toggling the node's `inputs:*` attrs;
  `MAGNET_ANGULAR_DAMPING_COEFF` is 0.0 permanently. Symptom to recognize: frozen pose +
  constant nonzero speed + zero integration while the graph keeps printing heartbeats.
- **Runners must construct `World(..., set_defaults=False, physics_dt=1/EFFECTIVE_PHYSICS_RATE_HZ)`**
  (2026-09-10): `World()`'s default `set_defaults=True` makes `PhysicsContext.__init__`
  rewrite the OPEN stage's physics scene to Isaac defaults — `set_physics_dt(1/60)`
  (stomping the stage-authored `physxScene:timeStepsPerSecond=120` down to 60) AND
  `enable_stabilization(False)` (stomping the tuned `True`). Symptom: `/clock` and
  `/isaac_joint_states` at ~50-60 Hz no matter what the USD says (check with pxr — the
  on-disk value stays 120; the stomp is in-memory only). `run_wall_stage.py`,
  `run_wall_stage_gui.py` and `probe_magnet_tuning.py` pass
  `physics_dt=1/pt.EFFECTIVE_PHYSICS_RATE_HZ, rendering_dt=1/60, set_defaults=False`;
  the other demos' runners (`teleop_runner.py`, `tier3_regression_test.py`,
  `pick_and_delivery_e2e_test.py`, `test_scout_only_drive.py`, `capture_real_scan.py`)
  still carry the bug — fix them before trusting their physics behavior.
- **The effective step rate is 60 Hz, not the stage-authored 120**
  (`physics_tuning.py`'s `EFFECTIVE_PHYSICS_RATE_HZ`, 2026-09-10, measured not assumed):
  at 120 Hz this host only achieves ~52 physics steps per wall-second (0.43x real-time
  — ~17 ms per rendered frame of render+graph overhead vs ~1 ms per physics step, so
  the sim-time-driven ROS stack would run at half wall speed), AND the magnet graph's
  script-applied linear damping (`MAGNET_DAMPING_COEFF` via
  `apply_forces_and_torques_at_pos`) freezes the sensor boxes solid at 120 Hz — Bug 9's
  signature (pose bit-identical, velocity readback pinned at F/c), live-bisected by
  killing the damping mid-run: frozen at 2.0, unfrozen at 0.0, but at 0.0 the
  undamped box bang-bangs off the wall and escapes the attract range. At 60 Hz +
  stabilization ON all three magnet probes PASS (weld at tick 32, d=0.0088 m;
  30-deg arrival welds at 6.6 deg), matching the historical calibration. Expect
  `/clock` and `/piper/joint_states` at ~50-60 Hz in the readiness check.

## Configuration

All tunable values are ROS 2 parameters in `src/kotek_bringup/config/*.yaml` — nothing
architecturally important is hardcoded. Notably:

- `base_controller.yaml`: velocity limits, gains, tolerances, watchdog timeouts, and the
  obstacle-avoidance layer's `obstacle_safety_distance`/`obstacle_stop_distance`/
  `obstacle_steer_gain`/cone angles
- `coordinator.yaml`: `pregrasp_standoff` (0.40m), reachability envelope, `arm_mount_xyz`
  (`[0, 0, 0.29101]`), and the delivery leg's `delivery_x/y/z`, `delivery_yaw`,
  `delivery_standoff`
- `manipulation.yaml`: approach/lift/place geometry, grasp width, planning scaling
- `wall_mount.yaml`: the wall-mount demo's own full copy of the manipulator block
  (loaded INSTEAD of `manipulation.yaml` by `kotek_wall_mount_demo.launch.py`):
  riser-corner approach, `grasp_pitch`/`place_pitch` split, sensor-box grasp width
- `isaac_bridge.yaml`: Isaac topic names, command rate, gripper stall-detection thresholds

## Nav2 migration (v2)

Only `src/kotek_bringup/launch/base_control.launch.py` changes: delete the
`simple_base_controller_node` `Node(...)` and include Nav2's bringup (e.g.
`isaac_simulation/scout_nav2_pkg/launch/bt.launch.py`) instead. `SimpleBaseController` and Nav2
both serve `nav2_msgs/action/NavigateToPose` on the action name `navigate_to_pose` with the same
feedback fields, so `task_coordinator`, `piper_manipulator`, `isaac_joint_bridge`, and every
config file except `base_controller.yaml` are unchanged. Note that Nav2 would need its own
obstacle-avoidance/costmap configuration — the reactive layer in `kotek_base_control` is
intentionally simple/local and is not meant to survive this swap (same as the rest of
`SimpleBaseController`).

## The grasp fix

The object being violently ejected from the gripper is **fixed**; see
`docs/grasp_fix_report.md` for the full root-cause trail and every measurement.
Short version: it was never a friction problem. `import_robots.py` applied the
arm's ROTATIONAL drive constants to the PRISMATIC finger joints, where
`JOINT_MAX_FORCE = 1e3` -- chosen for joint3's holding torque in N*m -- means
~900 N of squeeze; the arm's own drives ran at a damping ratio of 0.001 and sat
in a ~3 Hz limit cycle that swung the gripper +/-6-7 cm sideways and knocked the
object off its pedestal before the gripper even closed; and `/World/sensor`
carried no PhysX properties at all, leaving a 0.02 m contact offset on a 0.05 m
object. All three are fixed in the asset, so this pipeline inherits the fix by
re-running the stage-authoring steps in section 4.

Measured on identical motion (`~/Projects/kotek_sim/tests/test_grasp.py`):

| | before | after |
|---|---|---|
| peak object speed | 4.29 m/s | 0.18 m/s |
| finger stall width | 0.0400 m (closed on air) | 0.0520 m (= the object) |
| slip after lift + hold | 0.1125 m | 0.0011 m |
| object rise | -0.325 m (ejected to the floor) | +0.109 m (held) |

`inspect_stage.py --assert-demo-ready` now asserts every one of those asset
properties, so the "authored nothing, silently got a PhysX default" bug class
cannot recur silently.

A standalone, ROS-free implementation of the same task lives at
`~/Projects/kotek_sim` and shares this workspace's robot asset and
`physics_tuning.py` module, so the two cannot drift apart.

## Known limitations

- **Base translation under commanded `/cmd_vel`** -- RESOLVED (report sections 1.7 and 4.5).
  `tier3_regression_test.py` measures 58.72 cm of real displacement in the commanded +x
  direction against a ~60 cm prediction, and re-passes against the rebuilt asset.
- `piper_manipulator`'s cancel handling is best-effort: an in-flight `MoveGroupInterface::plan()`
  / `execute()` call cannot be interrupted mid-call by this synchronous implementation.
- Gripper grasp success is judged by position stall detection in `isaac_joint_bridge` (no joint
  effort feedback is available from the stage) — if this proves unreliable in practice, the
  documented fallback is a fixed-joint attach on contact.
- Visual/collision meshes from the URDF import have some duplicate sibling Xforms (e.g.
  `scout_mini_base_link` and `scout_mini_base_link_1`) — cosmetic, does not affect physics or
  joint names.
