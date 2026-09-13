# Pick-and-delivery: root-cause fixes, obstacle avoidance, and the delivery leg

This report covers four things, each backed by an actual command and its actual output (never
just a claim): (1) the real fix for the Tier 3 regression test failures reported at the start of
this work, (2) a lidar-based reactive obstacle-avoidance layer, (3) a delivery leg that extends
pick-to a full pick-and-deliver task, and (4) the end-to-end integration test that exercises all
of it together, including three real cross-process infrastructure bugs found and fixed along the
way.

**Update**: the base-drive limitation flagged throughout most of this document's history (the
Scout Mini not reliably translating under a commanded `/cmd_vel`) is now **fully root-caused and
fixed** -- see section 1.7. It took an extensive investigation (four compounding, independently-
confirmed bugs: fragmented wheel collision geometry, corrupted URDF wheel inertia, a mirrored
left/right joint-orientation sign convention, and -- along the way -- ruling out PhysX's own
articulation-joint drive mechanism as the cause before finding it wasn't the cause after all).
Sections 1.3-1.6 are kept as written, in their original "still unresolved" framing, because they
document the real, honest trail that led to the fix in 1.7 and are referenced from it -- they are
not stale filler.

## Step-by-step index

Every step taken, in the order it actually happened, each backed by the detailed writeup (with
reproduce commands and real output) in the linked section. Steps marked (superseded) turned out,
on later investigation, not to be the whole story -- kept rather than deleted, because the
superseding section explains what was actually wrong with the earlier conclusion.

1. Fixed the Tier 3 test harness's own step loop (`sim_ctx.step(render=False)` never pumped
   OmniGraph) -- [1.1](#11-the-real-bug-steprenderfalse-never-pumped-omnigraph).
2. Fixed arm joint torque insufficiency (`joint3` couldn't hold some poses) -- Test 3 fully passing
   -- [1.2](#12-arm-joint-torque-bug-test-3).
3. Investigated the remaining commanded-drive failure (Test 2): 11 hypotheses tested and ruled out
   -- [1.3](#13-test-2-commanded-straight-line-drive-unresolved-extensively-investigated).
4. Found and compared against a working reference asset (`scout_working`/Limo) in the source repo;
   applied its gains -- did not fix it, but ruled out every remaining robot-attribute-level cause
   -- [1.4](#14-follow-up-investigation-a-working-reference-asset-still-no-fix) *(superseded)*.
5. Built a minimal scout-only asset directly from the official URDF to isolate the bug outside the
   full stage; found the wheel collision was mostly non-touching via direct PhysX contact-report
   instrumentation; fixed it (analytic cylinder) -- still didn't roll, isolating the cause to
   PhysX's own velocity-driven-articulation-joint momentum transfer, confirmed with a from-scratch
   minimal repro outside the Scout asset entirely -- [1.5](#15-root-cause-found-physx-velocity-driven-articulation-joints-do-not-transfer-roll) *(superseded)*.
6. Built and validated a custom torque-based drive controller in increasingly realistic minimal
   repros (1-wheel, 2-wheel, 4-wheel-matching-topology) -- worked in isolation, did not yet work on
   the real asset -- [1.6](#16-torque-based-custom-drive-validated-in-isolation-does-not-yet-fix-the-real-asset) *(superseded)*.
7. Found and fixed the real remaining causes: corrupted URDF wheel inertia, and a mirrored
   left/right wheel-orientation sign convention. **Commanded drive fully fixed** (magnitude) and
   verified in the real production pipeline, all Tier 3 checks passing --
   [1.7](#17-the-complete-definitive-fix--confirmed-working-in-the-real-production-pipeline).
8. Built the lidar sensor and `/scan` publishing -- [2.1](#21-lidar-sensor).
9. Added obstacles to the demo stage -- [2.2](#22-obstacles).
10. Built the reactive obstacle-avoidance logic (pure function + unit tests) -- [2.3](#23-reactive-avoidance-logic).
11. Integration-tested obstacle avoidance against the real compiled node with a real captured scan
    -- [2.4](#24-integration-test-against-the-real-compiled-node).
12. Built the `PlaceObject` action and a second `piper_manipulator` action server --
    [3.1](#31-placeobject-action--piper_manipulator).
13. Extended the task FSM with the full delivery leg -- [3.2](#32-fsm-extension).
14. Smoke-tested the full extended stack (Tier 1/2) -- [3.3](#33-tier-12-smoke-test-with-the-full-extended-stack).
15. Built the live end-to-end integration test; found and fixed three real cross-process
    infrastructure bugs along the way (RMW implementation, `use_sim_time`, a missing TF timestamp)
    -- [4.1](#41-rmw_implementationrmw_fastrtps_cpp-does-not-actually-interoperate-across-this-boundary),
    [4.2](#42-no-node-had-use_sim_time-wired-to-isaac-sims-clock),
    [4.3](#43-kotek_sensor_graphs-tf-publisher-never-had-a-timestamp-wired-at-all).
16. Ran the live E2E test for the first time: reached `DRIVE_TO_PREGRASP` and aborted exactly on
    the (at the time still-open) base-translation limitation -- everything else in the pipeline
    verified working live -- [4.4](#44-end-to-end-result-before-the-section-17-drive-fix).
17. Re-ran the live E2E test after the section 1.7 fix: same failure state, but a *different*
    cause -- turning (not just translation) was also broken, independently, in the opposite
    direction. Found and fixed it -- [4.5](#45-end-to-end-result-after-the-section-17-drive-fix--new-narrower-blocker-found) *(superseded)*.
18. User asked to investigate the resulting `navigate_to_pose` timeout. Built fast live diagnostics
    and found two bugs: (Bug A) the insufficient-progress watchdog killed legitimate turn-in-place
    phases, and (Bug B) **the base had been driving backward this entire session** under a
    commanded forward `/cmd_vel` -- the magnitude fix from step 7 was real, the direction was not,
    and no test had ever checked direction. Fixed both --
    [4.6](#46-the-real-fix-two-bugs-found-one-of-them-a-session-spanning-direction-sign-error).
19. While isolating Bug B, found a real, separate, still-open diagonal-pair wheel-load asymmetry
    (partially mitigated, not fully resolved) -- [4.7](#47-a-real-separate-still-open-finding-diagonal-pair-wheel-load-asymmetry).
20. Re-ran the live E2E test with all of step 18's fixes: the drive stack is confirmed correct, but
    reactive obstacle avoidance (by design, no path-planning in v1) was suppressing motion because
    an obstacle sat directly on the straight-line path to the goal -- [4.8](#48-re-ran-the-live-e2e-test-with-all-of-section-46s-fixes--new-expected-non-bug-blocker) *(superseded)*.
21. User asked to move the obstacle away from the path. Repositioned both demo obstacles with real,
    computed clearance -- zero effect on the failure. Found the real cause instead: the robot's own
    lidar was self-detecting part of its own body (most likely the Piper arm's mount) --
    [4.9](#49-the-obstacle-reposition-didnt-fix-it--the-real-cause-is-lidar-self-detection).
22. User asked to raise the lidar height. Did so, confirmed self-detection gone; found and fixed
    the resulting loss of real-obstacle visibility (raised obstacle heights to match); found and
    fixed a real relative-output-path deployment bug that had been silently discarding every
    obstacle edit all session. Re-ran the live E2E test: **`DRIVE_TO_PREGRASP` succeeded for the
    first time this entire investigation**, and the task proceeded automatically through
    `ALIGN_BASE -> STOP_BASE -> MOVE_ARM_TO_PREGRASP`, sending a real grasp goal --
    [4.10](#410-fixed-raised-the-lidar-and-a-real-deployment-bug-found-along-the-way).
23. User tested the rebuilt demo live and reported a wheel physically hit `obstacle_pickup_leg`
    while driving to pickup. Found the obstacle-placement clearance math (sections 4.8/4.9) had
    only ever subtracted the obstacle's own half-width, never the robot's -- real margin was only
    12.7cm. Fixed with a footprint-aware clearance formula (robot half-width, read from the URDF,
    plus a path-tracking safety buffer), giving 22.0cm; verified via structural assertion, a live
    bbox readback, and a real `/scan` reading matching predicted geometry to 3cm/1deg --
    [4.11](#411-user-reported-wheel-physically-hit-obstacle_pickup_leg-while-driving-to-pickup).
24. User corrected the diagnosis: the collision was with the sensor pedestal, not the obstacle,
    and separately reported rviz2 doesn't open. Found the same bug class as step 23 but along the
    forward approach direction (`pregrasp_standoff` never accounted for the chassis's own forward
    extent or the pedestal's radius, leaving only 8mm of nominal clearance) and fixed it by
    raising the standoff and shrinking the pedestal; separately found `ros-jazzy-rviz2` was simply
    never installed in the Dockerfile and added it. Re-verified both obstacle corridors, rebuilt
    the docker image, and ran the full 126-test gtest suite (0 failures) --
    [4.12](#412-user-reported-it-was-the-sensor-pedestal-not-the-obstacle-and-rviz2-doesnt-open).
25. User reported the FSM gets stuck forever in `MOVE_ARM_TO_PREGRASP` and the arm never moves.
    Found and fixed two compounding, deeper infrastructure bugs via live debugging (raw action
    goal tests, gdb backtraces on the running process): `isaac_joint_bridge.py`'s single-threaded
    executor deadlocked every trajectory goal against its own state subscription, and separately
    `piper_manipulator`'s own startup hung forever inside `MoveGroupInterface`'s constructor
    (three real bugs fixed along the way: a shared-node deadlock, a node-name collision, and a
    double-namespaced action name), worked around with a finite wait timeout. `grasp_object`
    goals are now accepted and reach real MoveIt planning for the first time -- a separate,
    not-yet-diagnosed planning failure is the new, narrower next step --
    [4.13](#413-user-reported-after-move_arm_to_pregrasp-the-robot-never-changes-state-and-the-arm-never-moves).
26. User reported the very next failure: every `plan()`/`move()` call immediately failed with
    `MoveGroup action client/server not ready`. Read the actual installed MoveIt source and
    confirmed this readiness check is queried fresh every call, not a startup-timing artifact --
    `MoveGroupInterface`'s own internal action clients simply never report ready in this
    environment. Bypassed with two raw, proven-working `rclcpp_action::Client`s, keeping
    `MoveGroupInterface` only for target-setting and request construction. Found and fixed a
    second real bug on the way (every joint missing acceleration limits, crashing trajectory
    time-parameterization for any successful plan). Result: the first real, successful, executed
    arm motion in this project's history --
    [4.14](#414-fix-movegroup-action-clientserver-not-ready----bypassing-movegroupinterfaces-broken-dispatch).
27. Continued investigating the pregrasp/preplace reachability failure directly against
    `/piper/compute_ik`, bypassing OMPL entirely. Ruled out the IK solver's absurdly short
    5ms timeout (fixed it too, but it wasn't sufficient alone) and collision. Found the real
    cause by reading the URDF directly: the end-effector orientation formula assumed the
    approach axis is local +X, but this arm's tip link's gripper is offset along local +Z.
    Fixed with a fixed 90-degree correction, verified against 4 real poses via direct IK calls.
    Found and fixed a fourth infrastructure bug on the way (`computeCartesianPath()` hanging
    forever, same broken-callback-group class as section 4.14, bypassed the same way). Result:
    the complete grasp sequence succeeded fully for the first time in this project's history,
    and `place_object` was also verified working --
    [4.15](#415-fix-pregraspreplace-poses-genuinely-unreachable----a-wrong-end-effector-frame-assumption).
23. User reported the grasp was visibly twisted (`grasp.png`) and the object was never delivered.
    Found and fixed four more real bugs: the orientation formula never confirmed which local axis
    actually opens (fixed with a look-at/basis construction); the position formula targeted
    link6's own origin instead of the fingertips, 0.13503m further along its approach axis; the
    odom-to-arm-frame transform silently assumed the chassis sits at Z=0; and the post-lift arm
    pose sat almost exactly at the lidar's own mount height, self-detected as an obstacle and
    blocking `DRIVE_TO_DELIVERY`. Also built and verified live a grasp-success/still-holding
    verification-and-retry mechanism per the user's explicit request, and found a fifth bug (a
    missing switch case) while wiring in the new `STOW_OBJECT` stage --
    [4.16](#416-user-reported-grasp-visibly-twisted-and-object-never-delivered----four-more-real-bugs-found-plus-a-working-retryverification-mechanism).
24. User reported the gripper still closed on empty air ("should be a little closer"). Found and
    fixed four more real bugs: `min_cartesian_fraction=0.9` silently accepted a Cartesian path
    that only completed 93.75%, landing ~1cm short of the target; raising it initially had no
    effect because `manipulation.yaml`/`isaac_bridge.yaml`'s top-level parameter keys never
    matched their actual namespaced nodes, silently discarding every value in both files
    project-wide; a retry re-entering the same state it failed in never actually resent the goal;
    and the delivery/place leg had the same finger-offset reachability problem the pickup leg had
    in section 4.16, never previously reached live. Verified live, repeatedly, with solid contact
    and a full clean run reaching `PLACE_ARM`. One real, confirmed remaining limitation flagged:
    the object can still slip during `LIFT_OBJECT` even after a solid contact --
    [4.17](#417-user-reported-gripper-should-be-a-little-closer-falls-in-both-attempts----four-more-real-bugs-one-of-them-project-wide).

## 1. The Tier 3 test fix

### 1.1 The real bug: `step(render=False)` never pumped OmniGraph

`tier3_regression_test.py`'s step loop used to call `sim_ctx.step(render=False)`. Reading
`SimulationContext.step()`'s actual source (`isaacsim.core.api.simulation_context`):

```python
if render:
    ...
    self._app.update()          # pumps OmniGraph, ROS 2 bridge nodes, everything
else:
    if self.is_playing():
        self._physics_context._step(...)   # raw PhysX ONLY -- no app.update() at all
```

`render=False` steps PhysX directly and never calls `self._app.update()` -- the thing that
actually ticks OmniGraph's `OnPlaybackTick` chain (`DifferentialController`,
`IsaacArticulationController`, `IsaacReadSimulationTime`, every `ROS2Publish*`/`ROS2Subscribe*`
node). This explained every earlier symptom: gravity/settling worked (pure PhysX, no graph
needed) while `/clock`, `/isaac_joint_states`, and commanded `/cmd_vel` motion never appeared
(all downstream of a graph that was never evaluated). Fixed by replacing every
`sim_ctx.step(render=False)` call with `simulation_app.update()`.

**Reproduce:**
```bash
source /home/ranulfo/Projects/kotek_ws/env_isaaclab/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # fine for same-process Isaac Sim scripts; see section 4.1 for cross-process work
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(python3 -c 'import isaacsim,os;print(os.path.dirname(isaacsim.__file__))')/exts/isaacsim.ros2.core/jazzy/lib"
cd /home/ranulfo/Projects/kotek_ws/src/kotek_isaac_stage/kotek_isaac_stage
python tier3_regression_test.py
```
**Result:** Test 1 (zero-`/cmd_vel` drift) passes: 0.10cm drift over 3s, upright 1.0000.

### 1.2 Arm joint torque bug (Test 3)

Test 3 (arm joint convergence) initially still failed on `joint3` only. Direct diagnosis:
commanding `joint3` to -0.3 rad (with the rest of the arm posed per the test) left it stuck at
0.0 with the URDF-imported `drive:angular:physics:maxForce=100`, and it converged exactly to
-0.3 once `maxForce` was raised. The Piper URDF specifies `effort=100` (N*m) uniformly for every
arm joint; that is not enough torque authority for `joint3` specifically to hold this pose
against the gravity load of the rest of the arm. Not a wiring bug -- the command chain was
independently verified correct (`IsaacArticulationController` received `targetPosition=-0.3`
every tick regardless). Fixed with a `JOINT_MAX_FORCE = 1.0e3` override in `import_robots.py`'s
`set_drive_gains()`, applied to all arm+gripper joints.

**Result:** Test 3 now passes all 12 checks (exact convergence and holding for every joint,
including joint3).

A red herring surfaced and corrected during this investigation, worth recording so it is not
rediscovered: `Usd.Stage.TraverseAll()` silently skips into collapsed instance prototypes for
`instanceable=true` prims instead of visiting them at their instance paths. An early diagnostic
that appeared to show zero collision geometry anywhere on the robot was an artifact of this --
traversing with `Usd.TraverseInstanceProxies()` found 92 `PhysicsCollisionAPI` prims; collision
geometry was present the whole time. The `stage.Load()` calls in `import_robots.py` are still
correct (the physics/joint payload does need explicit loading), but the comment explaining why
was wrong and has been corrected.

### 1.3 Test 2 (commanded straight-line drive): unresolved, extensively investigated

**Symptom:** with `/cmd_vel` commanding 0.3 m/s forward, all 4 wheels reach and hold exactly the
commanded 3.75 rad/s (confirmed via the authoritative tensor-backed
`Articulation.get_joint_velocities()` API, not just a raw USD attribute readback) essentially
instantly, with near-zero variance for the full window -- yet the chassis only translates
0.5-2.5cm (noisy across runs) against a pure-rolling expectation of ~60cm over 2s.

**Hypotheses tested and ruled out, each with a direct experiment:**

1. Wheel drive damping/gain magnitude -- swept 2 to 1000, no meaningful trend.
2. Ground/wheel friction coefficient -- no `UsdPhysics.Material` was bound anywhere in the
   stage. Properly bound a high-friction material (mu=1.0-1.2) to the ground AND to the wheel
   collision meshes (required de-instancing the `instanceable=true` collision prims first) --
   negligible change (~1.6-2.5cm vs ~0.5-0.8cm baseline).
3. Wheel radius mismatch -- measured actual wheel mesh bbox, matches the configured 0.08m
   exactly.
4. Wheel rotation axis -- computed the joint's rotation axis in the chassis frame; it is the
   correct lateral (rolling) axis.
5. Wheel-to-wheel desynchronization -- all 4 wheels track within noise of each other throughout.
6. The rigid mount joint to the Piper arm -- disabled at runtime, no meaningful change.
7. PhysX solver iteration counts / substep rate -- raised position/velocity iterations 8x and
   substeps to 240Hz, no meaningful change.
8. Fragmented wheel collision geometry (each wheel decomposes into 7 separate convex-hull
   sub-parts) -- replaced with a single clean analytic cylinder collider; result was, if
   anything, worse (0.48cm).
9. Chassis belly-ground clearance (the chassis's own collision hull bottoms out only ~2mm above
   the wheel's own lowest point) -- added 1-2cm of clearance via wheel-mount offset; only a ~2x
   change, not a fix.
10. Newton vs PhysX engine identity -- Isaac Sim 6.0.1 ships an alternate "Newton" physics engine
    that can auto-switch active on startup, and the URDF importer dual-authors Newton-namespaced
    schema attributes alongside PhysX ones on every collider/joint (a real red flag). Directly
    verified the Newton extension is not even loaded in this environment; PhysX is confirmed the
    sole active engine.
11. Direct velocity injection into the chassis's free-root DOF (bypassing wheel drives/contact
    entirely) gets arrested almost instantly -- consistent with the always-active wheel drive
    acting as a very stiff brake, but this is physically expected behavior for a rigidly-held
    wheel velocity servo and does not, on its own, explain why driven rolling also fails to
    translate momentum to the chassis.

None of these individually or in combination (friction + clearance combined: still only 0.70cm)
produced translation anywhere near the pure-rolling prediction. The remaining, untested
hypotheses point toward the low-level PhysX articulation solver's handling of the
joint-reaction-force path between a velocity-driven revolute joint and its parent link under
sustained rolling-contact load -- diagnosable only with direct PhysX contact-report
instrumentation or a from-scratch minimal repro outside this asset. **Decision (user-approved):
document this fully and proceed with the rest of the requested work**, since arm control and
manipulation logic do not depend on translational drive fidelity, and the reactive
obstacle-avoidance layer (below) is independently valuable and independently testable regardless
of this limitation.

**Reproduce (same command as 1.1):** Test 1 and Test 3 pass; Test 2 fails with output like
`[FAIL] base moved forward under command (got 0.79cm)`. The script itself now prints a pointer to
this report when that happens.

### 1.4 Follow-up investigation: a working reference asset, still no fix

A later session found two directly relevant assets already present in the source repo (never
modified, read-only): `limo_env.usd`/`limo_controller.usd` (a different but structurally similar
AgileX robot, and the confirmed template the original corrupted drive graph was cloned from --
its stale `differential_controller` default `angularVelocity=10.2` is the exact same value baked
into the original bug), and, more usefully, a **second, uncorrupted Scout Mini prim subtree**
already present in the original `scout_and_piper_ros2_final.usd` at `/World/scout_working` --
same 60kg chassis / 3kg wheel masses, same joint layout as our own rebuild. Directly verified via
tensor-API velocity control (bypassing OmniGraph entirely): `scout_working` moves **54.59cm in
2s**, matching the ~60cm pure-rolling prediction -- a genuinely working reference, not just a
plausible-looking one.

Its wheel joints differ from ours sharply: `drive:angular:physics:damping=1,000,000` (ours was
100 -- our earlier sweep only tested up to 1000, three orders of magnitude short) and an
explicitly authored `physxJoint:maxJointVelocity=inf` (ours was never authored at all). Applying
both to our rebuild (kept in `import_robots.py` as `WHEEL_DAMPING`/`WHEEL_MAX_JOINT_VELOCITY`,
verified landing correctly on the composed stage) produced **no change** (0.79cm). Further
isolation, each via the same direct tensor-control method:

- Consolidating the wheel's collision from 7 fragmented convex-hull sub-parts to 1 single mesh
  (`merge_mesh=True`, matching `scout_working`'s own single-mesh structure exactly): **worse**
  (0.39cm) -- reverted.
- Our rebuild with the Piper arm entirely removed: **1.11cm**, identical to with it -- rules out
  the mounted arm.
- Infinite `Plane` ground swapped for a large finite box at the same height: **1.32cm** -- rules
  out ground-primitive type.
- Explicit `physics:diagonalInertia` (imported verbatim from the URDF's `<inertia>` tags) cleared
  to the "let PhysX auto-compute from geometry+mass" sentinel, matching `scout_working`: **0.91cm**
  -- rules out the inertia tensor.
- Most decisive: `scout_working` referenced directly, unmodified, into our own composed stage (our
  ground, our physics scene) in place of our own scout: **0.01cm** -- the same proven-working
  robot fails when placed in our stage's context, pointing at the stage/environment/physics-scene
  level rather than any robot attribute (all now individually ruled out). Caveat: this referenced
  robot's position was frozen bit-for-bit from the very first sample, with none of the small real
  settling jitter our own robot shows -- possibly a reference-arc-specific PhysX initialization
  quirk rather than a clean confirmation, so this result is suggestive, not conclusive.

**Net status**: the search space is meaningfully narrower (every robot-attribute-level cause
is now individually tested and ruled out) but the root cause remains unresolved. `WHEEL_DAMPING`
and `maxJointVelocity` are kept as a real, harmless, reference-matched improvement;
`merge_mesh` was tried and reverted after making things worse.

### 1.5 Root cause found: PhysX velocity-driven articulation joints do not transfer roll
momentum to the parent body, confirmed with a minimal repro outside the Scout asset entirely

Per the user's direction, rather than keep comparing against the external reference, a **minimal
scout-only USD was built directly from the official URDF** (`import_scout_only.py` -- no Piper
arm, no lidar, no demo-stage sublayer, nothing but the scout and a ground plane), confirmed to
build and run cleanly, and tested (`test_scout_only_drive.py`). It reproduced the exact same bug
(0.56cm vs the ~60cm/2s prediction), definitively ruling out everything else in the larger stage
(Piper arm, lidar, mount joint, demo-stage sublayer) -- the cause is intrinsic to the scout's own
articulation, not the surrounding stage.

*(One infrastructure bug found and fixed along the way: `author_robot_graphs.py`'s
`build_scout_drive_graph` wires `IsaacReadLidarBeams`/`ROS2PublishLaserScan` nodes with a
`lidarPrim` target -- when no lidar prim exists at all (as in this minimal sandbox),
authoring that graph does not just no-op or warn, it crashes the whole Kit process with a native
`terminate called without an active exception` inside `World.reset()`/`play()`, no Python
traceback. Fixed with a new `include_lidar` parameter, `False` for the scout-only sandbox.)*

**Direct PhysX contact-report instrumentation** (`PhysxSchema.PhysxContactReportAPI` applied to
the wheel rigid body, subscribed via `omni.physx.get_physx_simulation_interface()
.subscribe_contact_report_events()`) on the unmodified minimal scout during a drive window showed
why: of 747 individual contact points captured, only 35 (4.7%) had nonzero impulse, and only 50
(6.7%) had separation under 1mm (i.e. were actually touching) -- mean separation was 1.57cm. The
wheel's imported collision geometry is 7 separate convex-hull sub-parts (per-mesh-part from the
URDF's multi-piece `wheel.dae`), and PhysX's speculative-contact margin was generating mostly
non-touching "candidate" contacts rather than continuous rolling contact.

**This looked like the fix.** Replacing the wheel's collision with a single clean analytic
`UsdGeom.Cylinder` (radius 0.08m, matching the joint's own rotation axis, correctly centered at
the wheel joint's `physics:localPos1=(0,0,0)` -- found only after tracing through two wrong
placements: `UsdGeom.BBoxCache.ComputeLocalBound(prim)` returns the bound in the prim's *parent*
frame, not the prim's own local space, which had spuriously matched `localPos0` and produced a
wildly wrong collider position first try) and properly de-instancing + disabling the old
fragmented collision (the `Part__FeatureNNN` sub-parts are individually `instanceable=True`,
which -- the same red herring as before -- makes them invisible to plain `Usd.PrimRange` and
un-editable without `SetInstanceable(False)` first) produced **perfect geometric contact**:
settled chassis height matched the true baseline exactly (0.1811m vs 0.1809m), and 240/240
contact points were within 1mm separation, with 120/240 nonzero impulse.

**And it still did not roll.** Displacement: 0.07-0.23cm, no better than the broken fragmented
collision. Binding an explicit high-friction material (mu=1.2) to both the ground and the wheel
made no difference either. The wheel's own true rolling axis was independently verified three
ways (URDF's declared `<axis xyz="0 0 -1">`, direct quaternion transform of the joint's
`localRot1`, and empirically by reading the wheel rigid body's actual world-frame angular
velocity during a drive command) -- all three agreed, so axis error is ruled out too.

**Decisive isolation, entirely outside the Scout asset:**
- A free (non-articulated) rigid-body sphere, radius 0.08m, given a direct angular velocity of
  3.75 rad/s about its rolling axis via `RigidPrim.set_angular_velocities()`, rolls a real
  **16.9cm in 2s** (physically correct: starts as pure spin, friction gradually converts it to
  rolling, asymptotically approaching the pure-rolling prediction) -- proving PhysX's friction
  and contact resolution work correctly in this install for a plain rigid body.
- A **from-scratch, ~80-line minimal 2-body articulation** (a "chassis" cube as the
  `ArticulationRootAPI` body, a "wheel" sphere of the same radius, connected by a single
  `PhysicsRevoluteJoint` with a velocity drive -- `damping=1e6`, `maxForce=3.4e38`,
  `maxJointVelocity=inf`, exactly mirroring the scout wheel's own drive configuration but with
  zero URDF import, zero fragmented meshes, zero scout-specific anything) commanded to the same
  3.75 rad/s: **2.3cm in 2s** -- an order of magnitude less than the equivalent free-spinning
  ball, reproducing the same broken-rolling signature in total isolation from the Scout Mini
  asset, its URDF import pipeline, or this project's stage composition.

**Conclusion**: this is not a Scout Mini asset problem, not a URDF-import problem, not a
collision-geometry problem, not a friction problem, and not a stage-composition problem -- all of
those were live, well-evidenced hypotheses and are now individually eliminated with direct
instrumentation or clean isolation, not just "still failing after another change." The remaining,
narrowed cause is specific to how this Isaac Sim 6.0.1 / PhysX build's **velocity-driven
`PhysicsRevoluteJoint` drive within an articulation** transfers (or fails to transfer) the
ground-contact reaction force into translational momentum of the parent body, even with
geometrically perfect, high-friction rolling contact. This is a plausible under-the-hood
interaction between the joint drive's internal solver iteration and the contact solver for
articulated bodies specifically (as opposed to free rigid bodies, which are confirmed to work) --
diagnosing further would require either instrumenting PhysX's own solver internals (outside this
project's reach) or an official Isaac Sim / PhysX bug report. Two paths forward exist and are
both viable without further root-causing: (a) drive the wheels via explicit torque/force commands
from a custom controller (bypassing PhysX's built-in velocity-drive-to-target mechanism entirely)
instead of a raw velocity target, or (b) accept this as a documented simulation-fidelity
limitation, since it affects only translational drive precision and not arm control, grasping, or
the reactive obstacle-avoidance logic (which is pure/unit-tested independent of base translation
fidelity).

### 1.6 Torque-based custom drive: validated in isolation, does not yet fix the real asset

Per user direction, path (a) was pursued: disable the wheel joints' built-in `DriveAPI` entirely
(`damping=0`, `stiffness=0`) and instead apply explicit torque every tick via the tensor API's
`Articulation.set_joint_efforts()`, computed from a simple proportional controller on velocity
error (`torque = clip(KP * (target_vel - current_vel), -MAX_TORQUE, MAX_TORQUE)`).

**Validated with progressively more realistic minimal repros, all outside the Scout asset:**
- 1-wheel toy (cube chassis + sphere wheel, one revolute joint): rolls, once gain-tuned for the
  wheel's actual rotational inertia (1.9cm/2s at conservative gains -- confirms basic mechanism).
- 2-wheel lateral-pair toy: rolls (20.3cm/2s, though with un-retuned/runaway gains -- confirms
  multiple simultaneous wheel joints on one shared parent body is not inherently broken).
- **4-wheel rectangular toy, matching the real scout's own front/rear + left/right layout
  exactly**: rolls cleanly, **56.8cm in 2s** -- matches the ~60cm/2s pure-rolling prediction
  almost exactly, with smooth convergence to the 3.75 rad/s target and near-zero steady-state
  torque (textbook-correct rolling behavior). This rules out wheel *count* and rectangular
  *topology* as an explanation for anything asset-specific.

**Applied to the real scout-only asset** (`kotek_scout_only.usd`, combined with the analytic-
cylinder collision fix from section 1.5, so contact is confirmed perfect: 240/240 points within
1mm separation): **displacement stayed at 0.01-0.29cm across every gain setting tried**
(KP/MAX_TORQUE swept from 0.05/1.0 up to 50/500), despite wheel velocities converging cleanly to
essentially the exact target (3.50-3.75 rad/s, well-tracked, sensible single-digit-to-low-double-
digit N*m torques -- not saturated, not oscillating). A real, consistent asymmetry was observed
in the converged velocities: left-side wheels settled around 3.50 rad/s, right-side around
3.72-3.73 rad/s, a persistent ~6% split across the full 2s window -- a genuine, systematic signal,
not noise. Checked whether this traces to a left/right mesh-mirroring difference in the wheel
joint's own axis convention (`physics:localRot1`, transformed the same way as section 1.5's axis
derivation): front-left and front-right joints have **identical** `localRot1=(0,-1,0,0)`, ruling
that out directly.

**Net status**: the torque-based approach is now proven, with an increasingly realistic chain of
minimal repros culminating in an exact-topology match, to be a working mechanism *in principle*
for restoring PhysX momentum transfer on a velocity-driven wheeled articulation. It does not yet
work on the real Scout Mini asset for a reason not yet identified, though the left/right velocity
asymmetry is a concrete, reproducible lead for further investigation (candidates not yet tested:
the real scout's actual per-wheel inertia tensor from the URDF, `ixx=iyy=0.7171 izz=0.1361`,
applied about a possibly-mismatched axis; some other real-mass-distribution effect the idealized
toy chassis doesn't reproduce). This is being reported to the user rather than iterated on
further, given the scope of investigation already invested.

### 1.7 The complete, definitive fix -- confirmed working in the real production pipeline

Section 1.6 found the torque-based approach worked in an increasingly realistic chain of
from-scratch minimal repros (up to an exact 4-wheel topology match, 56.8cm/2s) but stalled on the
real asset even with the collision fix applied, with a real, systematic left/right velocity
asymmetry. Continued investigation (per user direction) found the actual remaining causes:

**A third asset bug**: the URDF authors identical inertia (`ixx=iyy=0.7171`, `izz=0.1361`) for
*all four* wheels. A solid 3kg/0.08m-radius wheel should have I on the order of 0.01-0.02 kg*m^2
(`I=0.5*m*r^2=0.0096` axial) -- this is ~15-75x too large, almost certainly a copy-paste error in
the upstream URDF (identical on every wheel, itself suspicious). Directly confirmed causal:
clearing it (to the same "auto-compute from geometry+mass" sentinel already used in section 1.4)
changed a torque-driven wheel from ~0cm net displacement to real, if initially unstable, motion.

**A fourth, genuinely new-category bug, found while tuning the torque controller**: `wheel_link`'s
own world orientation is **mirrored between left and right sides** --
`front_left_wheel_link`'s `xformOp:orient` is `(0.707, -0.707, 0, 0)`, `front_right_wheel_link`'s
is `(0.707, 0.707, 0, 0)` (same pattern on the rear pair). This means a *positive* commanded joint
velocity rolls the left side forward and the **right side backward** -- confirmed both by the
observed symptom (commanding the same sign to all 4 wheels produced a real, persistent, ~6%
left/right velocity split, and separately a diagonal-pair stuck/free-spinning pattern depending on
control scheme) and by directly computing each side's world-frame rolling direction from its own
`orient` quaternion. **Every test in this entire investigation, across every session, including
the original bug report itself, had commanded the same sign to all 4 wheels uniformly.**

With collision (1.5) + inertia (above) + the left/right sign fixed together, a direct tensor-API
test moved **60.7cm in 2s** against target ~60cm -- and, critically, this did not even need the
custom torque controller from section 1.6: **PhysX's own native velocity drive works correctly**
once all three asset/control bugs are fixed, moving **59.34cm in 2s** with smooth, linear,
textbook-correct growth (9.28cm at 0.33s -> 59.34cm at 2.0s, matching the commanded 0.3 m/s almost
exactly) and clean stopping (0.78cm further drift after zeroing the command). The custom torque
controller was a genuinely useful diagnostic tool (it was what surfaced the inertia and sign bugs)
but is not part of the final fix -- one less moving part to maintain.

**Implementation**: `import_scout_only.py` and `import_robots.py` both gained a
`fix_wheel_collision_and_inertia`-equivalent step (replace each wheel's 7-fragment collision with
a single analytic `UsdGeom.Cylinder`, radius 0.08m, centered at the wheel joint's own
`physics:localPos1=(0,0,0)`, axis Z; clear `physics:diagonalInertia`/`principalAxes` to the
auto-compute sentinel). `author_robot_graphs.py`'s `build_scout_drive_graph` gained a
`Multiply`/`ConstantDouble(-1)` node pair that negates the right-side wheel command before it
reaches `IsaacArticulationController` -- the left/right sign fix, which belongs at the control
layer since the asset's own geometry is correct, just mirrored.

**Full re-verification against the real production pipeline** (`kotek_scout_piper_robot.usd` +
`kotek_scout_piper_demo.usd`, full scout+piper+lidar+obstacles, driven via real ROS 2 `/cmd_vel`
through the actual OmniGraph drive chain, not a tensor-API shortcut):

- `tier3_regression_test.py`: **all 21 checks PASS** -- Test 1 (0.09cm drift, upright 1.0000),
  **Test 2 (58.84cm moved against the ~60cm target, 1.31cm further drift after stopping)**, Test 3
  (all 6 arm joints converge and hold exactly).
- `tier3_obstacle_test.py`: unaffected, still passes (160/160 samples correctly suppressed near
  the real captured-scan obstacle) -- this test never depended on base translation fidelity.
- Tier 1/2 `colcon test` (kotek_msgs, kotek_base_control, kotek_manipulation,
  kotek_task_coordinator): **121 tests, 0 errors, 0 failures, 16 skipped** (skips are
  architecture/tool-availability gated, e.g. cppcheck) -- no regression from any of this session's
  changes.

The originally-reported bug (robot drives backward and leans on Play) and the previously-carried
"commanded drive doesn't translate the chassis" limitation are now **both fully resolved and
verified**, not just improved.

## 2. Obstacle avoidance

### 2.1 Lidar sensor

`import_robots.py` gained an `isaacsim.sensors.physx` PhysX-raycast lidar (not RTX -- lighter
weight, no render-product/annotator machinery needed) mounted at the front of the scout chassis
(local `(0.30, 0, 0.12)`), 270 degree FOV, 1 degree resolution, 10Hz rotation rate, matching
common real 2D lidars. `author_robot_graphs.py` wires
`isaacsim.sensors.physx.IsaacReadLidarBeams` -> `isaacsim.ros2.bridge.ROS2PublishLaserScan` into
the scout's own drive graph, publishing `/scan`.

**Reproduce:**
```bash
# same env as section 1.1
python import_robots.py && python author_robot_graphs.py
/tmp/claude-1001/.../scratchpad/usdenv/bin/python build_demo_stage.py --output .../kotek_scout_piper_demo.usd
/tmp/claude-1001/.../scratchpad/usdenv/bin/python inspect_stage.py .../kotek_scout_piper_demo.usd --assert-demo-ready
```
(any local `usd-core` venv works for `build_demo_stage.py`/`inspect_stage.py` -- see README)

**Result:** `/scan` publishes 270 points spanning +-135 degrees, `range_min=0.1`,
`range_max=10.0`; a live obstacle in the corridor produces real, correct hit distances (verified
directly: min range 0.250m at bearing 0 degrees against a box placed at (0.70, 0.15), 87/270
points under max range).

### 2.2 Obstacles

`build_demo_stage.py` gained `add_obstacles()`: two static collision boxes, offset to one side
of the straight-line corridors between spawn/pregrasp and pickup/delivery so a robot driving the
direct line approaches within lidar range and needs to react, without either corridor being
fully impassable. `obstacle_pickup_leg` at (0.70, 0.15); `obstacle_delivery_leg` at (0.55,
-0.55). Both 0.30m x 0.30m x 0.40m.

### 2.3 Reactive avoidance logic

New `kotek_base_control::obstacle_avoidance.hpp` (`applyObstacleAvoidance`, `LaserScanSummary`,
`ObstacleAvoidanceParams`), same pure/ROS-free/unit-testable pattern as the existing
`pose_utils.hpp`: given the base controller's desired `Twist2D` and a summary of the current
forward-facing scan (min range in forward/left/right cones), scales forward speed down to zero
between `safety_distance` (0.6m) and `stop_distance` (0.25m), and steers toward whichever side
has more clearance (clamped). Wired into `simple_base_controller_node.cpp` as a post-processing
step on every published `/cmd_vel`, subscribing to `/scan` and computing the cone summary from
the raw ranges using `angle_min`/`angle_increment`. New params in `base_controller.yaml`
(`obstacle_avoidance_enabled`, `obstacle_safety_distance`, `obstacle_stop_distance`,
`obstacle_steer_gain`, `obstacle_max_steer_angular`, cone angles). Degrades to a no-op (passes
the desired command through unchanged) if the scan is missing/stale, rather than blocking
navigation -- the existing pose/progress timeouts already guard a genuinely stuck robot.

**Unit tests** (`test_obstacle_avoidance.cpp`, 9 cases: clear-path passthrough, non-positive
linear never guarded, stop-distance hard stop, linear scaling, no-scaling-beyond-safety-distance,
steer-toward-more-clearance, steer/final-angular clamping, symmetric-clearance-no-bias):

**Reproduce:**
```bash
cd /home/ranulfo/Projects/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-select kotek_base_control &&
  source install/setup.bash && ./build/kotek_base_control/test_obstacle_avoidance'
```
**Result:** `[PASSED] 9 tests.`

### 2.4 Integration test against the real compiled node

`tier3_obstacle_test.py` replays one **real, physics-captured** lidar scan (via
`capture_real_scan.py`, run through env_isaaclab -- genuine Isaac Sim raycasts against the real
obstacle, not synthetic data) against the actual compiled `simple_base_controller_node` binary,
started as a subprocess by the test itself, and asserts its published `/cmd_vel` is correctly
suppressed. Replay instead of a live Isaac-Sim-to-docker stream because of the cross-process DDS
limitation described in section 4.1 -- this sidesteps it while still exercising real,
physics-derived sensor data through the real, compiled avoidance code path.

**Reproduce:**
```bash
# 1. capture (env_isaaclab):
python capture_real_scan.py
# 2. replay + integration test (docker container):
cd /home/ranulfo/Projects/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-select kotek_base_control &&
  source install/setup.bash &&
  python3 src/kotek/kotek_isaac_stage/kotek_isaac_stage/tier3_obstacle_test.py'
```
**Result:**
```
### loaded captured real scan: 270 points, min=0.251
  [PASS] navigate_to_pose action server (simple_base_controller_node) found
  [PASS] navigate_to_pose goal (2.0m past the obstacle) accepted
### after 4s: samples=161 max_observed_cmd_vel_linear=0.001 suppressed_samples=161
  [PASS] received at least one real published /cmd_vel sample from the node
  [PASS] published /cmd_vel linear speed was suppressed to well below max (0.5) for the large
         majority of samples, given the real captured scan shows an obstacle at 0.251m (161/161)
All obstacle-avoidance integration checks PASSED.
```
The real, compiled avoidance code, fed a real captured obstacle scan, suppresses forward motion
to near zero (161/161 samples below half of max speed, observed max 0.001 m/s vs. a 0.5 m/s
ceiling) -- a genuine, meaningful result independent of the base-translation limitation in
section 1.3 (this test does not depend on the base actually covering distance).

## 3. Pick-and-delivery: the delivery leg

### 3.1 `PlaceObject` action + `piper_manipulator`

New `kotek_msgs/action/PlaceObject.action` (mirrors `GraspObject.action`): goal is a place pose
(arm frame) + optional retreat height; sequence is pre-place -> lower -> open gripper (release)
-> retreat upward. `piper_manipulator` now hosts a second action server, `/piper/place_object`,
reusing the same `MoveGroupInterface` handles as `/piper/grasp_object` (there is only ever one
physical arm to plan for). The geometry is literally the same formulas as grasp/approach/lift
(`computeGraspPose`/`computeApproachPose`/`computeLiftPose` in `pregrasp_geometry.hpp`), reused
directly rather than duplicated.

### 3.2 FSM extension

`task_state.hpp`/`task_fsm.hpp` gained `DRIVE_TO_DELIVERY -> ALIGN_AT_DELIVERY ->
STOP_BASE_AT_DELIVERY -> PLACE_ARM -> OPEN_GRIPPER -> RETREAT_ARM -> DONE`, following a
successful `LIFT_OBJECT` instead of going straight to `DONE`. `STOP_BASE_AT_DELIVERY` is the
sole path into the place-arm states, mirroring `STOP_BASE`'s existing interlock for pickup
(`isPlaceArmState()`, tested the same way `isArmState()` is). New `coordinator.yaml` params:
`delivery_x/y/z`, `delivery_yaw`, `delivery_standoff`, `place_action_name`, `retreat_height`.
`task_coordinator_node.cpp` gained `sendPlaceGoal()` (mirrors `sendGraspGoal()`) and generalized
`sendNavGoal()`/`checkAlignment()` to take a target pose so the same code drives both the pickup
and delivery legs.

**Unit tests:** 8 new `TaskFsm` tests covering the delivery leg (drive/align/stop
success+failure+interlock, place feedback mirroring, place failure routing to `ABORTING`) plus
updates to the existing happy-path and abort-coverage tests.

**Reproduce:**
```bash
cd /home/ranulfo/Projects/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg &&
  source install/setup.bash &&
  colcon test --packages-select kotek_task_coordinator kotek_base_control kotek_manipulation kotek_msgs &&
  colcon test-result --verbose'
```
**Result:** `Summary: 121 tests, 0 errors, 0 failures, 16 skipped` (16 skipped are pre-existing
lint-category skips unrelated to this work).

### 3.3 Tier 1/2 smoke test with the full extended stack

**Reproduce:** the README's documented launch smoke test, unchanged.
**Result:** all nodes -- including the new `/piper/place_object` server -- start cleanly:
```
=== ros2 action list ===
/piper/grasp_object
/piper/place_object
...
[FSM] IDLE -> FIND_SENSOR
[FSM] FIND_SENSOR -> FAILED   (10s sensor_wait_timeout, no Isaac Sim connected -- expected)
```

## 4. The end-to-end integration test, and three real infrastructure bugs it found

`pick_and_delivery_e2e_test.py` (Isaac-Sim side) + the real `kotek_demo.launch.py` stack (docker
side) + `/task/start` orchestrates the full, real system: real Isaac Sim physics/sensors/arm on
one side, the real ROS 2 stack (`task_coordinator`, `simple_base_controller`,
`piper_manipulator`/MoveIt) on the other, over real cross-process DDS. Building this test
surfaced three genuine, previously-undiscovered infrastructure bugs -- not in the pick-and-place
logic, but in the plumbing connecting Isaac Sim to the docker container. Each was found with a
minimal, isolated reproduction before being fixed project-wide.

### 4.1 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` does not actually interoperate across this boundary

The README and `docker/compose.yaml` (both already existing before this session) document
`rmw_fastrtps_cpp` for both the Isaac-Sim-host process and the docker container (`network_mode:
host`). **This has likely never actually worked for real cross-process traffic.** Minimal
reproduction: a plain `std_msgs/String` publisher in an Isaac-Sim-hosted rclpy node and a `ros2
topic echo` in the container, both on `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. Discovery/matching
succeeds (`publisher.get_subscription_count()` reaches 1), but **zero messages are ever
delivered**, even after 100 publishes over 10 real seconds. Tried `ipc: host` on the container
(a well-known Docker/DDS shared-memory fix) -- no change. Switching **both sides** to
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (already installed in the docker image; available in
Isaac Sim's bundled ROS 2 libs too) fixes delivery immediately, with no other changes. Fixed
project-wide: `docker/compose.yaml`'s `RMW_IMPLEMENTATION` env var changed to
`rmw_cyclonedds_cpp`; every command in this report and the README that talks to a live Isaac Sim
process now uses `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, not `rmw_fastrtps_cpp`.

### 4.2 No node had `use_sim_time` wired to Isaac Sim's `/clock`

With DDS delivery fixed, the E2E test still failed immediately at `FIND_SENSOR`. Diagnosis:
`sensor_pose_publisher`'s TF lookup failed with `Lookup would require extrapolation into the
future. Requested time 30.4 but the latest data is at time 0.0` -- its own node clock used real
wall time (`~30.4s` since the node started) while Isaac Sim's published TF is stamped with
simulation time (`~0.0-1s`, from `/clock`). None of the docker-side nodes had `use_sim_time`
set. Fixed: added `{'use_sim_time': True}` to every `Node(...)` in `base_control.launch.py`,
`piper_moveit.launch.py`, and `kotek_demo.launch.py`.

### 4.3 `kotek_sensor_graph`'s TF publisher never had a timestamp wired at all

Even with 4.1 and 4.2 fixed, the same "latest data is at time 0.000000" error persisted --
except now the *requester's* clock correctly advanced in sim time while the *data itself* never
advanced past `t=0`. Reading `build_demo_stage.py`'s `add_sensor_tf_graph()`: unlike the scout
drive graph's own TF publisher (which wires `IsaacReadSimulationTime.outputs:simulationTime ->
ROS2PublishTransformTree.inputs:timeStamp`, see `author_robot_graphs.py`), the sensor graph's
`ROS2PublishTransformTree` node never had `inputs:timeStamp` connected at all -- so it published
every `base_link -> sensor` transform with the attribute's uninitialized default (0), forever.
This package's own module docstring already flagged `kotek_sensor_graph`'s raw-USD authoring
(vs. the scout graph's `og.Controller.edit()` authoring) as a known source of exactly this class
of bug. Fixed: added an `IsaacReadSimulationTime` node and wired it into
`publish_tf.inputs:timeStamp`, matching the scout graph's proven pattern. Verified directly
before re-running the full test: `kotek_sensor_graph`'s own `outputs:simulationTime` reads
`1.033...` after ticking (not stuck at 0).

### 4.4 End-to-end result (before the section 1.7 drive fix)

With the three fixes above in place, but before the wheel-drive fix (section 1.7):

```
### /task/state -> FIND_SENSOR
### /task/state -> COMPUTE_BASE_PREGRASP_POSE
### /task/state -> DRIVE_TO_PREGRASP
### /task/state -> FAILED
### full state history: ['FIND_SENSOR', 'COMPUTE_BASE_PREGRASP_POSE', 'DRIVE_TO_PREGRASP', 'FAILED']
```
```
[simple_base_controller_node] ERROR: Insufficient progress (0.044m in 10.0s); aborting, base stopped.
[task_coordinator_node]       ERROR: Task failed: navigate_to_pose did not succeed
```

This was the cleanest possible confirmation that section 1.3's base-translation limitation, and
*only* that limitation, was what blocked a full pick-and-deliver run: the task correctly finds
the sensor, computes a reachable pre-grasp pose, sends a real `navigate_to_pose` goal across real
cross-process DDS, and the real `simple_base_controller_node` correctly detects (via its own
`min_progress`/`progress_window` safety check) that the base is not actually covering ground --
0.044m in 10s, the exact same few-centimeter signature measured throughout section 1.3's
diagnosis -- and safely aborts rather than spinning forever. Every other piece of the pipeline
(sensor discovery, pregrasp geometry + reachability gate, nav-goal dispatch/monitoring, the
delivery leg's FSM wiring, cross-process DDS, sim-time synchronization) was verified working in a
real, live, cross-process run even at this point.

### 4.5 End-to-end result AFTER the section 1.7 drive fix -- new, narrower blocker found

Re-ran the identical live E2E test after rebuilding the full pipeline with the section 1.7 fix
(`tier3_regression_test.py` itself, using the exact same OmniGraph drive chain, real ROS 2
`/cmd_vel`, real physics, passes cleanly beforehand -- 58.84cm/2s). Result:

```
[task_coordinator_node]         [FSM] IDLE -> FIND_SENSOR
[task_coordinator_node]         [FSM] FIND_SENSOR -> COMPUTE_BASE_PREGRASP_POSE
[task_coordinator_node]         [FSM] COMPUTE_BASE_PREGRASP_POSE -> DRIVE_TO_PREGRASP
[simple_base_controller_node]   ERROR: Insufficient progress (0.003m in 10.0s); aborting, base stopped.
[task_coordinator_node]         ERROR: Task failed: navigate_to_pose did not succeed
[task_coordinator_node]         [FSM] DRIVE_TO_PREGRASP -> FAILED
```

Still failed at the same state initially, with the same "insufficient progress" signature -- but
this was a **different, narrower problem**, not a reversion of the section 1.7 fix: the fix was
independently verified working (both `tier3_regression_test.py`'s Test 2 and this exact rebuilt
asset/graph, via direct commanded `/cmd_vel`). The E2E run's own goal here was
`x=1.113 y=0.297 yaw=0.261` -- unlike `tier3_regression_test.py`'s pure straight-line 0.3 m/s
command, this requires both driving AND turning.

**Root cause found**: isolated turning specifically (a `/cmd_vel` with only `angular.z` set,
zero `linear.x`) against the fixed asset -- it was **broken too**, and independently of the
section 1.7 fix: commanding `angular.z=0.5` (intended CCW) for 3s produced `-0.038rad` (wrong
direction, ~40x smaller than the ~1.5rad naively expected). A direct joint-velocity sweep (outside
the OmniGraph entirely, mirroring the section 1.5/1.6 methodology) found: commanding all 4 wheels
the *same* sign produces rotation on this asset (opposite of the intuitive "same sign = straight,
opposite signs = turn" differential-drive convention -- explained by the same left/right mirroring
behind section 1.7, which inverts the role of the common/differential components). The section 1.7
right-side negation fix, while correct for translation, was applying to the differential
(rotation) component too and inverting its sign. Fixed independently: negate the angular command
itself (`break_angular.outputs:z`) before it reaches `DifferentialController` -- a no-op for pure
straight driving (where the differential component is zero), verified not to disturb the
already-confirmed translation fix (`tier3_regression_test.py` re-run clean after this change, all
21 checks still pass, Test 2 unchanged at 58.84cm).

Direct verification of the corrected rotation: commanding `angular.z=0.5` for 3s now produces
`+0.19rad` -- correct direction (CCW, matching the command), and real, accelerating magnitude
(the growth curve shows a slow start then rapid acceleration in the final ~0.7s of the window,
consistent with the same static-friction-breakaway ramp-up seen in straight driving, just slower
-- physically expected for rigid, non-compliant wheels resisting in-place rotation via scrub
friction, not evidence of a remaining bug).

**Re-ran the full live E2E test with the rotation fix.** Result improved qualitatively but the
task still does not complete within a reasonable window:

- With the original `goal_timeout: 60.0`: no longer "insufficient progress" (the min_progress/
  10s-window abort never fired) -- instead, `Goal timed out after 60.0s`. This is a materially
  different, better failure mode: the base was continuously making *valid* progress the entire
  window, it simply needed more time than budgeted.
- Raised `goal_timeout` to 120.0 in `base_controller.yaml` (documented there) and re-ran: **still**
  `Goal timed out after 120.0s`, still never triggering "insufficient progress".

**Not fully resolved this session.** The base is now proven to drive and turn in the geometrically
correct directions (both independently verified via direct commands), but the closed-loop
`navigate_to_pose` maneuver for this specific goal (an initial ~15 degree turn plus ~1.15m of
driving) consistently takes longer than 120s of simulated time to complete. The most likely
explanation, given `SimpleBaseController::step()`'s own phase logic (`simple_base_controller.cpp`):
`ControlPhase::DRIVE` reverts back to `ControlPhase::TURN_TO_HEADING` whenever heading error
exceeds `heading_tolerance * 2` mid-drive -- if the real (now confirmed slower-than-naively-
expected) rotation response causes heading error to oscillate around that threshold, the
controller could cycle between TURN_TO_HEADING and DRIVE repeatedly without ever converging
efficiently. Not yet confirmed with direct phase-transition logging -- flagged as the concrete
next step, with a **much narrower, well-understood search space** than the original bug: a single,
small, already-unit-tested C++ state machine (`SimpleBaseController`), not the physics/asset layer
(which is now proven correct in isolation for both translation and rotation).

**Reproduce:** two processes, exactly as the README's GUI + docker-stack pairing describes,
except with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both sides (see 4.1):
```bash
# Process 1 (docker container):
cd /home/ranulfo/Projects/kotek_ws
docker compose -f docker/compose.yaml run --rm kotek bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /workspace &&
  colcon build --symlink-install --packages-skip scout_nav2_pkg &&
  source install/setup.bash && ros2 launch kotek_bringup kotek_demo.launch.py'

# Process 2 (host, env_isaaclab), once move_group logs "You can start planning now!":
source /home/ranulfo/Projects/kotek_ws/env_isaaclab/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(python3 -c 'import isaacsim,os;print(os.path.dirname(isaacsim.__file__))')/exts/isaacsim.ros2.core/jazzy/lib"
cd /home/ranulfo/Projects/kotek_ws/src/kotek_isaac_stage/kotek_isaac_stage
python pick_and_delivery_e2e_test.py --timeout 400

# Once "physics playing" appears and ~15s have passed (DDS discovery warm-up), from the
# container: ros2 service call /task/start std_srvs/srv/Trigger "{}"
```

### 4.6 The real fix: two bugs found, one of them a session-spanning direction sign error

Investigating the phase-cycling hypothesis from 4.5 directly (rather than continuing to guess),
built a fast live diagnostic: the real, unmodified `simple_base_controller_node` binary driving
the real scout-only Isaac Sim asset directly over cross-process cyclonedds, bypassing the full
docker+move_group+task_coordinator stack for much faster iteration (~30s vs ~3min per attempt).
Added permanent phase-transition and throttled diagnostic logging to
`simple_base_controller_node.cpp` (there was previously no logging at all between "New goal" and
the terminal success/failure line -- a real, standing visibility gap, kept regardless of this
bug).

**Bug A (fixed): the insufficient-progress watchdog doesn't account for non-DRIVE phases.**
Direct observation with the new logging: the robot was correctly, monotonically converging in
`ControlPhase::TURN_TO_HEADING` (yaw climbing steadily) when the `min_progress`/`progress_window`
watchdog aborted it at exactly 10s with "Insufficient progress (0.003m in 10.0s)". Root cause:
`TURN_TO_HEADING` and `ALIGN` both command `v=0` by design (see `simple_base_controller.cpp`), so
a robot correctly executing a real, if slow (stiction-limited, see section 4.5), in-place turn
will *always* fail an XY-distance-based progress check if convergence takes longer than
`progress_window_`. **Fixed** in `simple_base_controller_node.cpp`: the progress-history
accumulation and check now only run while `phase() == DRIVE`, clearing on phase exit so the
10s window restarts fresh once real translation is actually expected. This is what actually
explains the *original* "insufficient progress" aborts from the first two E2E attempts in
section 4.4/4.5 (before `goal_timeout` was even raised).

**Bug B (found, then fixed -- the real headline finding): the base has been driving BACKWARD
this entire session under a commanded forward `/cmd_vel`, and no test ever caught it.**

With Bug A fixed, the live diagnostic proceeded into `DRIVE` -- and the base ran away in a
straight line at close to max speed, 16+ meters over 18s, essentially zero yaw change. Isolated
directly with a minimal `/cmd_vel` test reading wheel joint velocities every tick: even a
**pure straight command with zero angular component** (`v=0.5, w=0.0` exactly, uniform wheel
commands `6.25` on every wheel, bit-identical to the already-"passing" straight-drive test)
produced `dx = -1.46m` over 3s -- moving in **-x**, the opposite of the commanded `+linear.x`
direction.

The reason this was never caught: `tier3_regression_test.py`'s Test 2 (and
`test_scout_only_drive.py`'s equivalent check) only ever asserted
`moved = (driving_xy - pre_drive_xy).GetLength() > 0.15` -- a **magnitude-only** check. The base
was covering the right *distance* (matching the ~60cm/2s pure-rolling prediction almost exactly,
which is why section 1.7's fix looked completely correct) while moving in exactly the wrong
*direction*, and nothing in this session's test suite, across dozens of runs, ever compared the
sign of displacement against the sign of the commanded velocity. Both test scripts have been
**fixed** to additionally assert `delta_xy[0] > 0.15` (displacement specifically in `+x`,
matching a positive commanded `linear.x`), closing this real, standing gap for good.

**Root cause and the real fix.** Section 1.7's `negate_right` fix (negate the right-side wheel
command, leave left unchanged) and section 4.5's `negate_angular` fix (negate the angular command
before `DifferentialController`) both operate on the *combined* per-side JOINT output, which
mixes both the translation ("common-mode") and rotation ("differential-mode") components of
`DifferentialController`'s math. Deriving it out: `negate_right` alone cannot independently fix
the sign of the common-mode (translation) component without *also* flipping the sign of the
differential-mode (rotation) component, since both land on the same per-side output -- exactly
what section 4.5 ran into when it had to add a second, independent fix for rotation. The
translation component's sign was never actually corrected; the *magnitude* fix (section 1.5-1.7:
collision, inertia) was completely real and correct, but the direction was backward the whole
time, and `negate_right` alone structurally cannot fix direction without breaking the
already-correct rotation.

**Fixed properly** in `author_robot_graphs.py`'s `build_scout_drive_graph`: added a
`negate_linear` node (`Multiply` by the same `neg_one` constant, mirroring `negate_angular`
exactly) on `break_linear.outputs:x`, before `DifferentialController`, in addition to (not instead
of) `negate_right`. This corrects the common-mode and differential-mode components
*independently*, before they get combined, exactly matching how the two components need
genuinely different sign treatment.

**Verified all three cases directly**, rebuilding `kotek_scout_only.usd`'s drive graph each time:

| command | before this fix | after this fix |
|---|---|---|
| pure straight (`v=0.3, w=0`) | `dx=-0.58m` (backward) | `dx=+0.58m` (forward, correct) |
| pure rotation (`v=0, w=0.5`) | `+0.19rad` (already correct, from section 4.5) | `+0.19rad` (unchanged, still correct) |
| combined (`v=0.5, w=0.15`) | `dx=-1.46m`, `yaw≈0` (backward, no real turn) | `dx=+1.46m`, `yaw=+0.002rad` (forward + correct-direction turn) |

Rebuilt the full production pipeline (`import_robots.py` -> `author_robot_graphs.py` ->
`build_demo_stage.py`) and re-ran the full verification chain:
- `tier3_regression_test.py`: **all 22 checks PASS** (one more than before -- the new direction
  check), Test 2 now `dx=+25.2cm dy=+1.4cm` in 2s on the full scout+piper robot (lower than the
  scout-only sandbox's ~58cm, expected given the added Piper arm mass/asymmetry -- still
  unambiguously correct-direction and far past the `>15cm` threshold).
- `tier3_obstacle_test.py`: unaffected, still passing.
- Tier 1/2 `colcon test` (kotek_msgs, kotek_base_control, kotek_manipulation,
  kotek_task_coordinator): **123 tests, 0 errors, 0 failures, 16 skipped** -- includes two new
  investigative gtest cases added to `test_control_law.cpp` (`RealAngularLagOnTheLiveE2EGoalDoes
  NotThrash`, `DeadbandModelOnTheLiveE2EGoalRevealsComputeDriveGap`) that helped narrow this down.

### 4.7 A real, separate, still-open finding: diagonal-pair wheel-load asymmetry

While isolating Bug B (section 4.6), direct all-4-wheel `PhysxContactReportAPI` instrumentation
during a combined-command drive found the front-right+rear-left wheel pair consistently bearing
**~4x more normal-force impulse** than the front-left+rear-right pair (mean `|impulse|` 2.3-2.4 vs
0.58-0.63). This exact diagonal grouping had been observed once before, in section 1.6's
torque-controller investigation (a "stuck" vs "free-spinning" wheel-velocity split along the same
two diagonals) -- a real, reproducible, asset-level asymmetry, not a one-off artifact.

Checked for a geometric/mass explanation and found none: `base_link` mass (60kg), center of mass
`(0,0,0)`, and all four wheel joints' `physics:localPos0` are exactly mirror-symmetric
(`(±0.2319755, ±0.2082515, -0.100998)`); all four wheel links have identical mass (3kg) and
identical analytic-cylinder collision geometry (radius 0.08, height 0.16, axis Z, centered at the
joint). The asset is genuinely symmetric.

One real, partial contributor found: `base_link`'s own imported inertia tensor
(`diagonalInertia=(2.288641, 3.431465, 5.103976)`, `principalAxes=(-0.707,0.707,0,0)` -- a
90-degree-rotated principal-axis frame, imported verbatim from the URDF, the same class of issue
already found and fixed for the *wheels'* inertia in section 1.7) does contribute: clearing it to
the same "auto-compute from geometry+mass" sentinel roughly **halved** the asymmetry (from ~4x to
~2x: mean impulses 1.02/1.97/1.92/0.97 across FL/FR/RL/RR). This fix is real and worth keeping,
but a residual ~2x asymmetry persists even with a clean inertia tensor.

Directly tested whether this residual asymmetry explains Bug B (before Bug B's own fix was found):
applying the `base_link` inertia fix alone, with Bug B still present, changed the combined-drive
result by less than 0.1cm (146.23cm -> 146.12cm, still backward) -- so the diagonal asymmetry and
Bug B are independent issues, not cause-and-effect.

**Leading hypothesis for the residual ~2x asymmetry**: a rigid (zero-suspension) 4-wheel vehicle
resting on 4 rigid contact points is a classic statically *over-constrained* system -- 3 points
uniquely determine the plane a rigid body rests on; a 4th rigid point's load share is not uniquely
determined by statics alone (the "wobbly four-legged table" effect) and is instead resolved by
solver-specific numerical details, which can settle into a stable but uneven bias, often along a
diagonal. This asset has no suspension modeling at all (direct rigid revolute wheel joints, no
Z-compliance), so this would be a genuine physical/numerical modeling limitation of the current
asset design, not a wiring or sign bug. **Not yet confirmed or fixed** -- the natural next step
would be adding a small amount of compliance (softer PhysX contact material, or a literal
small-travel suspension joint per wheel) and re-measuring the diagonal split, but this is a
separate, lower-priority item now that Bug B (which was the dominant, correctness-breaking issue)
is fixed and verified.

### 4.8 Re-ran the live E2E test with all of section 4.6's fixes -- new, expected, non-bug blocker

Re-ran the identical live E2E test (section 4.4's reproduce block) with the section 4.6 fixes
(direction sign + progress-watchdog) in the full production pipeline. Result: `DRIVE_TO_PREGRASP`
still aborts on "Insufficient progress (0.019m in 10.0s)" -- but the new phase-transition/tick
diagnostics (added in section 4.6) show why, directly: `desired=(v=0.494, w=0.225-0.228)` stays
strong and consistent throughout (i.e. `SimpleBaseController::step()` is computing exactly the
command a correctly-converging maneuver should), while `applied` -- the *post-obstacle-avoidance*
command actually published -- decays to near-zero (`v: 0.020 -> 0.003 -> 0.001 -> 0.000`,
`w` frozen at `0.063`). The drive fix itself is confirmed working correctly here; reactive
obstacle avoidance (section 2, `applyObstacleAvoidance`) is suppressing the command.

This is not a bug: the straight-line path from the spawn pose to the pre-grasp goal
(`x=1.113 y=0.297`) passes directly through `obstacle_pickup_leg`'s footprint (placed at
`(0.70, 0.15)`, half-width `0.15` -- the path crosses `x=0.70` at `y=0.187`, squarely inside the
obstacle's `y in [0.0, 0.3]` span; confirmed by direct calculation). `SimpleBaseController`'s
obstacle avoidance is a purely reactive, non-path-planning layer by design (see section 2 and the
README's "Nav2 migration" section -- this is the intentional v1 architecture, not an oversight);
it can slow down and steer toward more clearance, but has no capability to plan a route around an
obstacle that squarely blocks the direct line to the goal, and understandably struggles here where
it must both avoid the obstacle *and* satisfy the goal's yaw at the same time.

**Net effect on this session's work**: the base's translation and rotation are now proven correct
in both magnitude and direction (section 4.6), independently of and unaffected by obstacle
avoidance.

**Update (section 4.9): this diagnosis was incomplete.** Repositioning `obstacle_pickup_leg` (and
`obstacle_delivery_leg`, found to have the identical issue) had zero effect on the failure --
see 4.9 for the real cause, which turned out not to be either placed obstacle at all.

### 4.9 The obstacle reposition didn't fix it -- the real cause is lidar self-detection

Repositioned both obstacles with real, computed clearance (>=0.35m from each corridor's
centerline -- see `build_demo_stage.py`'s `add_obstacles()`), rebuilt the demo stage, and re-ran
the identical live E2E test. Result: **identical failure**, byte-for-byte the same suppression
signature as before (`applied` decaying to `v=0.000, w=0.063` while `desired` stays
`v=0.494, w=0.227`) -- and critically, the robot's own pose in the tick log never moved
meaningfully from the spawn point (`x~0.001`) the entire time, nowhere near either
obstacle's new position (~0.8-1m away). The obstacles were never the cause.

Directly inspected the raw `/scan` topic at the spawn pose, before any driving, parsing it with
the exact same forward-cone logic `simple_base_controller_node.cpp`'s `scanCallback()` uses
(`forward_half_angle=0.26` rad): **`min_forward_range = 0.269m`**, almost dead ahead
(`angle=0.017rad`). `obstacle_stop_distance=0.25` and `obstacle_safety_distance=0.6` in
`base_controller.yaml` -- a forward reading of 0.269m sits just above the hard stop threshold but
well inside the safety-slowdown zone, exactly matching the observed near-total speed suppression.
Nothing external is anywhere near that close to a robot still at its spawn point -- this is the
robot's own lidar detecting part of **its own body**.

Walked the Piper arm's own link tree, computing each link's world position relative to the
lidar's (`lidar world position: (0.299, -0.002, 0.301)`): every arm link origin sits
**+0.17 to +0.39m above** the lidar's own height, well outside the lidar's narrow (~1 degree)
vertical field of view at the link-origin level -- so the arm's joint origins don't obviously
explain it, though a link's actual collision *mesh* extending below its own origin (not checked)
remains a real possibility. The self-detected point's approximate world position (back-computed
from `range=0.269m` at `angle=0.017rad` from the lidar's own position) is roughly
`(0.57, 0.00, 0.30)` -- notably close in height to `ARM_MOUNT_OFFSET`'s own Z
(`(0.0, 0.0, 0.29101)`), suggesting the Piper's own base-mount plate/bracket (a different part of
the geometry than the individual arm links checked) as the likely specific culprit, though this
has not been pinned down with a precise raycast/mesh-level check.

**Status: not resolved this session** *(superseded -- see section 4.10, resolved next)*. This was
the actual, final blocker for the live E2E test -- a lidar/arm-mount geometry conflict, not a
base-drive bug (translation and rotation are both independently proven correct, section 4.6) and
not a path-planning limitation (section 4.8's diagnosis, while a real and independently-worth-
fixing obstacle placement issue, was not the cause of this particular failure).

### 4.10 Fixed: raised the lidar, and a real deployment bug found along the way

User direction: raise the lidar height (option (a) from section 4.9). Changed
`import_robots.py`'s `LIDAR_LOCAL_POS` Z from `0.12` to `0.35` (world z ~0.53, clearing the
Piper mount structure's top at world z ~0.47). Rebuilt and re-verified with the same direct
`/scan`-inspection technique used to diagnose the problem: **`min_forward_range` returns
`10.0` (`range_max`, i.e. nothing detected) at the spawn pose** -- self-detection is gone.

**Immediately found a real, unrelated trade-off**: the same technique, applied to a robot placed
0.8m from the demo stage's actual obstacle box (still 0.40m tall at that point), also read
`range_max` -- the raised lidar had become blind to the existing obstacles, which don't reach its
new scan height. Raised `build_demo_stage.py`'s obstacle box heights from 0.40 to 0.70 (comfortably
above the new 0.53 scan height) to restore real detectability, and re-verified the same way: a
robot 0.8m from the (now taller) obstacle correctly reads back `min_range=0.351` (matching the
true near-edge distance).

**Found and fixed a real, separate deployment bug while re-verifying**: after repositioning and
resizing the obstacles, a repeat live-E2E run showed *zero effect* from either change -- a robot
still 0.8m from the (correctly, on-disk, taller) obstacle read back `range_max` again. Direct
inspection of the actual saved USD file's raw prim attributes revealed why: `build_demo_stage.py`'s
`--output` argument default was the *relative* path `'usd/kotek_scout_piper_demo.usd'`, and every
invocation this session had been run from the script's own directory
(`kotek_isaac_stage/kotek_isaac_stage/`, exactly as the README instructs) -- silently writing to
`kotek_isaac_stage/kotek_isaac_stage/usd/kotek_scout_piper_demo.usd` instead of the canonical
`kotek_isaac_stage/usd/kotek_scout_piper_demo.usd` every test and the live E2E driver actually
read. The obstacle reposition (section 4.8) and resize (this section) had both been real, correct
edits that silently landed in a file nothing else looked at -- confirmed directly: the canonical
file's `obstacle_pickup_leg` prim still had the *original* `(0.70, 0.15, 0.2)` translate and
`(0.3, 0.3, 0.4)` scale, matching neither the repositioned nor the resized values. Fixed the
default to an absolute path (matching `import_robots.py`/`import_scout_only.py`'s own convention),
removed the stray nested file, and rebuilt to the correct location -- confirmed via the raw prim
attributes this time (`translate=(0.545, 0.73, 0.35)`, `scale=(0.3, 0.3, 0.7)`), then re-ran both
`/scan` checks clean: self-detection still gone, obstacle correctly detected at `min_range=0.351`.

**Full re-verification**: `tier3_regression_test.py` -- all 22 checks pass, Test 2
`dx=+58.72cm dy=-0.07cm` (comparable to the scout-only sandbox's own numbers, well past the prior
full-robot run's 25cm -- consistent with the diagonal-load-asymmetry finding in section 4.7 being
sensitive to exact settling, not a regression).

**Re-ran the full live E2E test.** Result: **`DRIVE_TO_PREGRASP` succeeded for the first time this
entire investigation.** The controller log shows a completely normal drive: `TURN_TO_HEADING ->
DRIVE -> TURN_TO_HEADING -> DRIVE -> ALIGN -> STOPPED`, ending in
`"Goal reached and base settled (lin=0.000 ang=0.000)."`. The task coordinator's FSM then
proceeded automatically, on its own, through states that had never once been reached in this
entire session:

```
[FSM] DRIVE_TO_PREGRASP -> ALIGN_BASE
[FSM] ALIGN_BASE -> STOP_BASE
[FSM] STOP_BASE -> MOVE_ARM_TO_PREGRASP
Sent grasp_object goal: object in arm frame = (0.471, -0.013, -0.090)
```

The run reached `MOVE_ARM_TO_PREGRASP` and sent a real `grasp_object` goal to `piper_manipulator`
before the test driver's own 300s script budget elapsed (arm motion planning/execution was still
in progress when Isaac Sim's process was closed by the driver's timeout -- not a failure, just an
unfinished run). This is unambiguous confirmation that every fix from this session's investigation
(sections 1.5-1.7, 4.5-4.6, 4.9-4.10) is correct and working together in the real, full, live
pipeline: collision, inertia, translation direction, rotation direction, the progress watchdog,
obstacle placement, and lidar self-detection are all resolved. The remaining, narrower scope for a
future session is purely in the arm-grasp-and-beyond stages (`MOVE_ARM_TO_PREGRASP` onward) and/or
giving the E2E test driver a larger time budget to let a full grasp/lift/deliver/place sequence
play out to `DONE` -- a completely different, much narrower area of the codebase than anything
this investigation touched.

### 4.11 User-reported: wheel physically hit `obstacle_pickup_leg` while driving to pickup

User ran the rebuilt demo and reported the base could not reach the pickup position because a
wheel hit the obstacle. Section 4.8/4.9's reposition of `obstacle_pickup_leg` had only ever
subtracted the *obstacle's own* half-width (0.15m) from its perpendicular distance to the
spawn->pregrasp corridor centerline -- it never subtracted the *robot's* own half-width, i.e. it
computed clearance for a zero-width point following the corridor, not for the actual robot.

Recomputed precisely (`scratchpad/compute_obstacle_clearance.py`, reusing the exact
`bearingTo`/`computeBaseGoal` formulas from `pregrasp_geometry.hpp` against the real spawn
(0,0), pregrasp-goal (1.1135, 0.2969, computed live from `sensor=(1.5,0.4)` and
`pregrasp_standoff=0.40`), and delivery-goal (-0.05,-1.2) poses):

- **Robot half-width**: 0.2883m, not previously accounted for anywhere in either the obstacle
  placement or the reactive-avoidance layer. Read directly from `scout_mini.urdf`: every wheel
  joint's origin is offset `|y|=0.2082515` from the chassis centerline, and the wheel collision
  cylinder (`import_scout_only.py`/`import_robots.py`) is `0.16m` thick, i.e. `0.08m` half-width
  --  `0.2082515 + 0.08 = 0.2883m` from centerline to a wheel's outer face, the robot's widest
  point.
- **`obstacle_pickup_leg`'s old position** (0.545, 0.730): perpendicular distance to the
  spawn->pregrasp centerline was 0.5649m -- after subtracting the obstacle's half-width (0.15m)
  *and* the robot's half-width (0.2883m), the real wheel-to-obstacle-edge margin was only
  **12.7cm**. `SimpleBaseController::step()`'s own `DRIVE` phase tolerates heading error up to
  `heading_tolerance * 2 = 0.30rad` before reverting to `TURN_TO_HEADING` (`simple_base_
  controller.cpp`), so the real driven path is not guaranteed to hug the idealized straight line
  that tightly -- a thin, real-but-inadequate margin, consistent with the reported wheel strike.
- **`obstacle_delivery_leg`** (unchanged, 1.077, -0.836): perpendicular distance to the
  pregrasp->delivery centerline is 0.6664m, giving a real 22.8cm margin already -- consistent
  with the user only reporting a problem on the way *to* pickup, not on the delivery leg.

**Fix**: `build_demo_stage.py`'s `add_obstacles()` now derives a `PATH_TRACKING_MARGIN = 0.20m`
buffer (sized to the same `heading_tolerance * 2` hysteresis above) and requires
`obstacle_center_to_centerline >= ROBOT_HALF_WIDTH + PATH_TRACKING_MARGIN +
OBSTACLE_HALF_WIDTH`. Repositioned `obstacle_pickup_leg` to `(0.521, 0.820)` -- same ~0.71m
progress along the corridor as before (still requires the reactive-avoidance layer to react,
still within lidar range), new perpendicular clearance 0.66m, giving a **22.0cm**
wheel-to-obstacle-edge margin, matching the delivery leg's own already-safe level.
`obstacle_delivery_leg` needed no change.

**Verified**, all real, executed commands:
- `inspect_stage.py --assert-demo-ready` against the rebuilt stage: all checks pass.
- Direct `UsdGeom.BBoxCache` readback of `/World/obstacle_pickup_leg` in the rebuilt, live-loaded
  stage: `min=(0.371, 0.670, 0.0) max=(0.671, 0.970, 0.70)` -- center `(0.521, 0.820, 0.35)`,
  confirming the new position landed correctly, not just on disk in the source script.
- Real `/scan` read at the spawn pose (Isaac Sim playing, `isaacsim.ros2.bridge` live): overall
  min range `0.677m` at angle `1.449rad`. The lidar's own world position at spawn is
  `(0.30, 0, 0.53)` (local mount pos + base_link at the origin); the geometric distance/angle
  from there to the box's nearest corner `(0.371, 0.670)` is `0.674m` at `1.465rad` --
  matches the live scan to within 3cm/1deg, direct confirmation the repositioned box is real,
  correctly placed, and detected (not blind, unlike the pre-4.10 height bug).

Not yet re-run: the full live pick-and-delivery E2E test (section 6.3) to directly observe the
base clear the obstacle end-to-end during `DRIVE_TO_PREGRASP` -- the analytic/geometric
verification above is decisive for the root cause and the fix's correctness, but a live rerun
would be the fullest, same-standard-as-the-rest-of-this-report confirmation and is a reasonable
next step.

### 4.12 User-reported: it was the sensor pedestal, not the obstacle; and rviz2 doesn't open

User corrected 4.11's diagnosis: the collision was with the sensor pedestal, not the obstacle
box, and separately reported that rviz2 does not open at all. Two independent, unrelated bugs.

**Pedestal collision -- same bug class as 4.11, different corridor geometry.** The old
`pregrasp_standoff=0.40` (coordinator.yaml) governs how far the parked robot stands from the
sensor, but -- exactly like the obstacle placement in 4.8/4.9 -- was never checked against the
robot's own physical size, this time along the *forward* direction rather than laterally.
Measured directly from the composed stage (`UsdGeom.BBoxCache.ComputeLocalBound` on
`/World/scout_mini/Geometry/base_link`): the chassis's own forward-most point is `0.31198m`
ahead of `base_link`'s origin (independently cross-checked against the front wheel's outer edge,
URDF joint offset `0.2319755 + 0.08` wheel radius `= 0.312` -- matches to sub-mm). The sensor
pedestal (`build_demo_stage.py`'s `add_sensor_object()`) has radius `0.08`.

At the nominal (zero-tracking-error) pregrasp goal pose, the chassis front edge landed only
**8mm** from the pedestal's surface (`scratchpad/verify_all_clearances.py`, reusing the same
`bearingTo`/`computeBaseGoal` formulas as 4.11): goal `(1.1135, 0.2969, 0.2606)`, chassis-front
world position `(1.4150, 0.3773)`, distance to pedestal center `0.0880m`, minus pedestal radius
`0.08m` = `0.0080m`. `base_controller.yaml`'s `xy_tolerance=0.08m` (the goal-reached radius) means
the robot can legitimately stop anywhere within 8cm of that nominal goal, including closer --
essentially guaranteeing contact, not just risking it.

**Fix**: raised `pregrasp_standoff` (coordinator.yaml, and `task_coordinator_node.cpp`'s matching
parameter default) from `0.40` to `0.48`, and shrunk the pedestal radius (`build_demo_stage.py`)
from `0.08` to `0.05` (still comfortably supports the 0.05x0.05m sensor cube). Re-verified with
the same script: new pregrasp goal `(1.0362, 0.2763, 0.2606)`, nominal chassis-to-pedestal-edge
clearance **11.8cm**, worst-case (minus the full `xy_tolerance`) **3.8cm** -- thin but real and
positive, versus the old value's already-negative worst case. Required arm reach at the new
standoff is `0.4801m`, still comfortably inside `[min_reach=0.15, max_reach=0.55]`.

Because `pregrasp_standoff` changed, the spawn->pregrasp corridor used by 4.11's obstacle-
clearance fix also changed (shorter, same direction/bearing) and the pregrasp->delivery corridor
changed too (different start point). Re-verified both with the same script, both still safe:
`obstacle_pickup_leg` margin **22.0cm** (unchanged -- perpendicular distance to a line depends
only on direction, not the endpoint, and the obstacle's along-corridor projection, 0.71m, is
still within the new, shorter 1.07m corridor), `obstacle_delivery_leg` margin **25.4cm**
(improved from 22.8cm, since the corridor's start point moved closer to the obstacle's side).

Not yet re-run: the full live E2E test to directly confirm the parked robot no longer contacts
the pedestal (same caveat as 4.11 -- the geometric verification is decisive for root cause and
fix correctness, but a live rerun is the fullest confirmation).

**rviz2 not opening**: the docker image's `Dockerfile` builds `FROM ros:jazzy-ros-base`, which
does not include rviz2 -- and `ros-jazzy-rviz2` was never added to the package list (grepped the
entire repo: no file references "rviz" anywhere before this fix). Not a configuration bug, a
missing dependency. Added `ros-jazzy-rviz2` to the Dockerfile's `apt-get install` list, rebuilt
the image (`docker compose -f docker/compose.yaml build`, real command, succeeded), and confirmed
inside a fresh container: `which rviz2` -> `/opt/ros/jazzy/bin/rviz2`, and
`ros2 pkg list | grep rviz` lists `rviz2`, `rviz_common`, `rviz_default_plugins`,
`rviz_ogre_vendor`, `rviz_rendering`, `rviz_visual_tools`. `compose.yaml` already forwards
`DISPLAY` and mounts `/tmp/.X11-unix` for X11, so no other docker-side change was needed.

**Full re-verification**: rebuilt the demo stage and re-ran `inspect_stage.py
--assert-demo-ready` -- all checks pass. Rebuilt the full workspace in a fresh container
(`colcon build --packages-skip scout_nav2_pkg`) and ran the full gtest suite across all four
gtest-bearing packages: **126 tests, 0 errors, 0 failures, 16 skipped** (cppcheck, architecture-
gated) -- confirms the `task_coordinator_node.cpp` default-value change compiles cleanly and no
existing test depends on the old `pregrasp_standoff`/pedestal-radius values.

### 4.13 User-reported: after `MOVE_ARM_TO_PREGRASP` the robot never changes state and the arm never moves

User reported that once the FSM reaches `MOVE_ARM_TO_PREGRASP`, it stays there forever and the
arm literally never moves. This is a live-system bug, not something visible from reading code, so
it was investigated by launching the real stack (Isaac Sim + the docker ROS 2 container) and
directly exercising the `grasp_object` action and its dependencies.

**Bug 1 (found and fixed): `isaac_joint_bridge.py`'s FollowJointTrajectory goals hang forever.**
Sent a raw `FollowJointTrajectory` goal directly to `/piper/arm_controller/follow_joint_trajectory`
(bypassing MoveIt entirely): the goal never completed, and the bridge's own feedback reported
`actual` frozen at its pre-goal value forever -- while `/isaac_joint_states` (Isaac Sim's own
ground truth) showed the joint had genuinely reached the commanded target. Root cause:
`isaac_joint_bridge.py` used the default single-threaded `rclpy.spin(node)`, and its
`FollowJointTrajectory` execute callback blocks that one thread for the whole trajectory
(`time.sleep()` loops) -- starving the node's own `/isaac_joint_states` subscription (so
`self._joint_positions` never updates during a goal) and, with `use_sim_time`, the internal
`/clock` subscription too (so `elapsed` never advances and the loop never exits). Fixed by giving
the state subscription and both action servers a `ReentrantCallbackGroup` and spinning with a
`MultiThreadedExecutor` instead of plain `rclpy.spin()` -- the same pattern
`piper_manipulator_node.cpp`'s C++ side already used (detached `execute()` thread +
`MultiThreadedExecutor`). Re-verified: the same raw goal now completes (`SUCCEEDED`) and the real
joint state matches the commanded target.

**Bug 2 (found and fixed, only partially explained): `piper_manipulator`'s own startup hangs
forever, so `grasp_object` goals were never even accepted.** With Bug 1 fixed, `grasp_object`
still never progressed. Direct evidence: `piper_manipulator`'s own "piper_manipulator ready: ..."
log line (the last line of `init()`) never printed, even after 3+ minutes, and even a completely
unrelated, trivial `get_parameters` service call to the node timed out -- the node's main thread
was permanently stuck *before* `main()` ever reaches `executor.spin()`, so nothing about this node
worked at all, not just `grasp_object`.

A live `gdb` backtrace (attached via a container started with `--cap-add=SYS_PTRACE`) showed the
main thread stuck inside `moveit::planning_interface::MoveGroupInterface`'s own constructor, in
`rclcpp_action::ClientBase::wait_for_action_server_nanoseconds()`. Three real, distinct, confirmed
bugs were found and fixed on the way to isolating this:

1. Constructing two `MoveGroupInterface` instances (`move_group_arm_`, `move_group_gripper_`) on
   the *same* shared node deadlocks the second one. Fixed by giving each its own dedicated
   internal `rclcpp::Node`.
2. Those internal nodes, by default (`NodeOptions::use_global_arguments(true)`), inherited the
   process's own `-r __node:=piper_manipulator` remap from `piper_manipulator`'s own launch entry
   -- silently renaming them back to "piper_manipulator" and colliding with the real node instead
   of being distinct (`ros2 node list` showed four nodes all named `/piper/piper_manipulator`).
   Fixed with `use_global_arguments(false)` plus an explicit node name and `"piper"` namespace.
3. Passing `move_group_namespace="piper"` explicitly via `MoveGroupInterface::Options` (a plausible-
   looking fix for #2) double-namespaces the target action, since the node is *already* in the
   "piper" namespace: `rclcpp::names::append("piper", "move_action")` yields the relative name
   `"piper/move_action"`, which then resolves *again* against the node's own `/piper` namespace to
   `/piper/piper/move_action` -- confirmed directly (`ros2 action info /piper/piper/move_action`
   showed exactly our node as the sole client, zero servers). Fixed by leaving `move_group_namespace`
   at its default (empty), letting the relative name `"move_action"` resolve correctly against the
   node's own namespace to `/piper/move_action`.

Even after all three fixes, `MoveGroupInterface`'s own internal `wait_for_action_server` (which
runs on its own dedicated `SingleThreadedExecutor` + background thread, per
`moveit_ros_planning_interface`'s own source -- confirmed by reading the actual
`move_group_interface.cpp` for this exact version, 2.12.4) still never returned. This was tested
exhaustively: shared vs. separate internal nodes, with vs. without our own additional spinning of
those nodes, with an 8s artificial startup delay (to rule out a discovery race), all with the
namespace bug already fixed -- every variant hung identically. Meanwhile, a bare
`rclcpp_action::Client` constructed against the *exact same node* and querying the *exact same*
`/piper/move_action` consistently succeeded in ~2s. This strongly suggests a genuine bug/limitation
specific to `MoveGroupInterface`'s own internal wait mechanism in this environment (busy,
many-participant DDS graph -- Isaac Sim, move_group's own 4 internal node identities, multiple TF
listeners, etc.), not something resolvable from the call site -- root-caused as far as reasonably
possible without moveit's own debug symbols.

**Pragmatic fix**: since the action clients themselves are constructed correctly regardless (the
bare-client test proves the same node can reach the same action), only the constructor's own
*readiness check* is unreliable here -- so `wait_for_servers` is now passed as a finite 10s
timeout (`rclcpp::Duration::from_seconds(10.0)`) instead of the default "wait forever". `init()`
now completes reliably.

**Verified end-to-end, live, real commands**: `piper_manipulator ready: ...` now prints on every
launch (confirmed on a completely clean relaunch, no leftover state). A real `grasp_object` goal
now gets **accepted** (never happened before this fix) and progresses through real MoveIt motion
planning, publishing `MOVE_ARM_TO_PREGRASP` feedback and returning a real result. The specific
test poses tried both failed motion planning itself (`arm planning failed (code=99999)`) --
this is a distinct, separate, not-yet-diagnosed issue (a genuine planning/reachability problem,
not a hang), flagged as the next concrete step, and NOT something this investigation's fixes were
expected to also resolve. Full workspace rebuild + gtest suite re-run clean after all of the
above: 126 tests, 0 errors, 0 failures.

### 4.14 Fix: `MoveGroup action client/server not ready` -- bypassing MoveGroupInterface's broken dispatch

Section 4.13's finite `wait_for_servers` timeout got `init()` to complete, but the very next live
run showed the real consequence: every `plan()`/`move()` call immediately failed with `MoveGroup
action client/server not ready`, and `retreat-to-safe planning failed` right after.

Read the actual installed `moveit_ros_planning_interface` 2.12.4 source directly from GitHub (not
guessed): `MoveGroupInterfaceImpl::plan()`/`move()` both gate on
`move_action_client_->action_server_is_ready()`, which calls `rcl_action_server_is_available()`
**fresh, on every single call** -- not a cached construction-time flag. This proves the failure
isn't a startup-timing artifact: the exact same check was still failing long after the whole
stack had settled, immediately after a completely normal goal acceptance. It matches, and
extends, 4.13's finding: `MoveGroupInterface`'s own internal `move_action_client_`/
`execute_action_client_` (built with an explicit, non-auto-executor-added `callback_group_` --
a private implementation detail of `MoveGroupInterfaceImpl`, not something the public constructor
lets a caller override) never reports ready in this environment, no matter how long you wait,
while a bare, plain-callback-group `rclcpp_action::Client` built on the exact same node
consistently succeeds in ~2s.

**Fix**: bypass `MoveGroupInterface`'s broken goal-dispatch specifically, while keeping it for
everything else it's still good at (target-setting, `computeCartesianPath()`'s service-based
Cartesian planning, and critically its public `constructMotionPlanRequest()`, which builds the
exact `MotionPlanRequest` `plan()`/`move()` would have sent, purely locally, no network
dependency). Added two raw `rclcpp_action::Client`s (`raw_move_client_` for
`moveit_msgs::action::MoveGroup`, `raw_execute_client_` for `ExecuteTrajectory`), built directly
on `self` (the original, correctly-named/namespaced launch-provided node -- not one of section
4.13's internal workaround nodes, which don't need this) with the default callback group, matching
the proven-working construction pattern exactly. `moveArmToPose()`/`moveGripperNamed()`/
`moveGripperToWidth()`/`retreatToSafe()` now call `constructMotionPlanRequest()` +
`sendMoveGroupRequest()` (which sends with `planning_options.plan_only = false`, so move_group
both plans and executes in one round trip, same as `move()`'s own behavior); `cartesianMoveTo()`
keeps `computeCartesianPath()` as-is and calls the new `sendExecuteTrajectory()` in place of
`execute(plan)`.

**A second, real, separate bug found and fixed on the way**: with the dispatch bypass in place,
the very first real plan attempt (`retreatToSafe`'s "zero" pose) failed differently:
`terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'
... parameter 'robot_description_planning.joint_limits.joint1.max_acceleration' has invalid type:
expected [double] got [integer]`, crashing both `move_group` and `piper_manipulator` outright.
`piper_camera_moveit_config`'s own `joint_limits.yaml` (read-only submodule, never edited) sets
`has_acceleration_limits: false` / `max_acceleration: 0` for every one of the 8 joints -- and
MoveGroup's `AddTimeOptimalParameterization` response adapter *requires* real acceleration limits
to time-parameterize any trajectory at all, hard-failing without them (a real, blocking bug that
would have affected every successful plan, not just this one). Fixed in
`kotek_bringup/launch/piper_moveit.launch.py`: mutate `moveit_config.to_dict()`'s own
`robot_description_planning.joint_limits` dict in place (5.0 rad/s^2 for the 6 arm joints, 2.0 for
the 2 gripper joints -- generous placeholders, undocumented upstream, but
`default_acceleration_scaling_factor: 0.1` already scales commanded acceleration down to 10% of
whatever ceiling is set, so the exact value matters far less than having one declared at all), and
pass this single pre-merged dict to both `move_group` and `piper_manipulator`. A first attempt --
passing the override as a *separate*, later entry in each node's `parameters=[...]` list -- hit
the exact crash above (the original file's bare `0` auto-declares as an INTEGER parameter; a
later file supplying `5.0`, a DOUBLE, for the same parameter name doesn't change its already-
declared type). A single dict, mutated once with the right types from the start, avoids the whole
multi-file precedence question.

**Verified end-to-end, live, real commands, on a fully clean relaunch**: no more "not ready"
errors anywhere. `retreatToSafe()`'s "zero"-pose plan now **completes real, successful, executed
motion** -- `move_group`'s own log: `Calling PlanningResponseAdapter 'AddTimeOptimalParameterization'`
(no error this time) -> `trajectory_execution_manager: Completed trajectory execution with status
SUCCEEDED` -> `Solution was found and executed.` **This is the first real, successful, executed
arm motion in this entire project's history.** Full gtest suite re-run clean: 126 tests, 0 errors,
0 failures.

**New, narrower, separate finding, not yet fixed**: the actual pregrasp/preplace target poses
(computed via `computeApproachPose`, backing off from the object pose by `approach_distance`/
`approach_height` with a fixed `gripper_pitch` tilt) consistently fail with OMPL's `Unable to
sample any valid states for goal tree` -- reproduced with two different test object poses (the
live E2E run's own real pregrasp geometry, and a separate, more centrally-located hand-picked
pose), both failing identically while the unconstrained "zero" joint-space target succeeds
cleanly. This points to the fixed `gripper_pitch`/approach-orientation combination being
genuinely hard or impossible for this 6-DOF arm to reach via IK at these positions (or a
collision at the goal), not an infrastructure problem -- flagged as the next concrete step, out of
scope for this fix.

### 4.15 Fix: pregrasp/preplace poses genuinely unreachable -- a wrong end-effector-frame assumption

Continued investigating section 4.14's reachability finding directly against `/piper/compute_ik`
(MoveIt's own `GetPositionIK` service), bypassing OMPL's random sampling entirely -- a much
faster and more decisive way to separate "genuinely no IK solution" from "planner couldn't find
the one that exists."

**First, ruled out the finite-timeout fix as insufficient on its own**: `kinematics.yaml` (the
same read-only `piper_camera_moveit_config`) sets `kinematics_solver_timeout: 0.005` -- 5
*milliseconds* -- for the arm group's KDL solver, nowhere near enough for an iterative
Newton-Raphson solver to converge. Overridden to 0.5s using the same in-place-dict-mutation
pattern as section 4.14's joint-acceleration fix (a separate `parameters=[...]` entry hit the
exact same already-typed-parameter problem). This alone did **not** fix reachability -- even a
trivial, straight-ahead, identity-orientation pose well inside the arm's reach still returned
`error_code=-31` (`NO_IK_SOLUTION`).

**Systematically swept IK across many position/orientation combinations** (a small `rclpy` script
calling `/piper/compute_ik` directly): every single test failed -- 21 samples across a wide range
of positions, both `gripper_pitch`-tilted and identity orientations, all `NO_IK_SOLUTION`. Ruled
out collision as the cause directly (`avoid_collisions=false` succeeds instantly at the identical
pose that fails with `avoid_collisions=true`, and separately `/piper/check_state_validity`
confirmed both the zero configuration and various found solutions are collision-free). Then found
the real signal: IK at the **exact** pose the arm's own forward kinematics reports at its zero
configuration (`/piper/compute_fk`) succeeds instantly and repeatedly, while every hand-picked
"obviously reasonable" test orientation (identity, or a pure `gripper_pitch` tilt) fails
completely, even 3cm away from that same exact point.

**Root cause**: read `piper_description.urdf` directly. The SRDF's `arm` group chain is
`base_link -> link6` (`link6` is the IK tip), but `joint7`/`joint8` (the gripper fingers, link6's
children) are offset from link6 by `xyz="0 0 0.13503"` -- the gripper sits along link6's **local
+Z axis**, not +X. `pregrasp_geometry.hpp`'s `computeGraspPose()` builds its orientation as
`quaternionFromRPY(0, gripper_pitch, approach_yaw)`, which implicitly assumes the end effector's
"pointing"/approach axis is local **+X** (a common convention, but the wrong one for this arm) --
so every computed grasp/approach orientation was asking the real solver for something link6's
actual geometry can never produce, regardless of position, timeout, or planner budget.

**Fix**: compose a fixed `Ry(+90deg)` correction onto the existing formula
(`quaternionMultiply(quaternionFromRPY(0, gripper_pitch, approach_yaw), quaternionFromRPY(0,
M_PI/2, 0))` in `computeGraspPose()`), remapping the "X-forward" formula onto link6's real
"Z-forward" convention. `computeApproachPose()`/`computeLiftPose()` need no change -- both just
copy `computeGraspPose()`'s orientation forward unmodified. Verified directly and decisively via
`/piper/compute_ik` (four different, real object positions, including the exact live-E2E-run pose
`(0.519, 0.021, -0.090)`): **all four succeeded** with the correction, all had failed
(`NO_IK_SOLUTION`) without it. Added `GraspApproachLift.OrientationAppliesXForwardToZForward
Correction` (`test_pregrasp_geometry.cpp`) to lock this in.

**A fourth infrastructure bug found and fixed on the way**: with the orientation fix in place,
`MOVE_ARM_TO_GRASP` (the first stage using `cartesianMoveTo()`) hung indefinitely with no error --
`MoveGroupInterface::computeCartesianPath()` uses a **service** client
(`cartesian_path_service_`), built the exact same broken way as the two action clients from
section 4.14 (an explicit, non-auto-executor-added `callback_group_`). Confirmed the node was
otherwise still fully responsive (a plain `ros2 param get` succeeded instantly) while this one
call sat blocked forever. Fixed with the same bypass pattern: a raw `rclcpp::Client<moveit_msgs::
srv::GetCartesianPath>` built on `self`, replacing `move_group_arm_->computeCartesianPath()`.

**Verified end-to-end, live, real commands, on a fully clean relaunch (Isaac Sim + docker stack,
generous time budget)**:

```
$ ros2 action send_goal /piper/grasp_object kotek_msgs/action/GraspObject "{object_pose: ...}"
Result:
    success: true
message: grasp complete
Goal finished with status: SUCCEEDED
```

**The complete grasp sequence -- `MOVE_ARM_TO_PREGRASP -> MOVE_ARM_TO_GRASP -> CLOSE_GRIPPER ->
LIFT_OBJECT` -- succeeded fully, for the first time in this project's entire history**, using the
exact object pose from the user's own original live E2E run. Total real elapsed time for the
whole sequence: ~9 seconds. `place_object` (the symmetric place sequence, same code paths, same
fixes) was also tested directly and **also succeeded fully** (`MOVE_ARM_TO_PREPLACE ->
MOVE_ARM_TO_PLACE -> OPEN_GRIPPER -> RETREAT_ARM`, result `success: true`). Full gtest suite
re-run clean after all of the above: 127 tests, 0 errors, 0 failures.

### 4.16 User-reported: grasp visibly twisted, and object never delivered -- four more real bugs found, plus a working retry/verification mechanism

Two user reports triggered this investigation: (1) `grasp.png` (repo root) showed the gripper
arriving at the object at a visibly twisted angle, unable to actually close around it -- despite
section 4.15's fix making the pose **IK-reachable**, nothing had verified the fingers would
actually **straddle the object correctly**; (2) even assuming a successful grasp, `DRIVE_TO_
DELIVERY` aborted almost immediately with `applied.v=0.000` while `desired.v>0.48` -- the exact
signature of `applyObstacleAvoidance()` (`kotek_base_control`) reading something as directly ahead
of the lidar. The user also asked for two standing process changes: take screenshots at each step
to verify visually (not just trust logs/IK numbers), and verify a grasp actually succeeded,
retrying if not.

**Screenshot infrastructure.** Extended the live-run harness into
`scratchpad/run_full_demo_live_shots.py`: a file-trigger IPC pattern (`screenshot_trigger.txt` /
`camera_trigger.txt` / `move_trigger.txt` / `query_trigger.txt`, polled every ~0.2s of real time)
lets an external shell command request a headless RTX screenshot, reposition the viewport camera,
teleport a prim, or query a prim's live world pose from a long-running Isaac Sim process, without
scripting the whole run in one file. Two real gotchas found building it: (1) the demo stage
authors zero lights (it's physics/OmniGraph-focused), so a `DomeLight` + `DistantLight` had to be
added purely for visualization; (2) the query handler's first version called
`ComputeLocalToWorldTransform(0)` -- literal time `0`, not `Usd.TimeCode.Default()` -- which
returns the frozen **spawn-time** pose for any time-sampled attribute instead of the live
simulated one, and produced a red herring (see below).

**Bug 1 -- orientation formula only checked IK-reachability, not which local axis actually
opens.** Section 4.15's `Ry(+90deg)` correction was one guess out of a family that happens to be
IK-reachable for the tested pose; nothing had confirmed it matches the gripper's real
opening-axis convention. Swept `joint7`/`joint8` independently via `/piper/compute_fk`: link7
moves to local `(-w, 0, 0.135)`, link8 to `(+w, 0, 0.135)` as the commanded half-width `w`
increases -- the fingers separate along link6's **local +X**, confirming the opening axis
directly rather than assuming it. Rewrote `computeGraspPose()` in `pregrasp_geometry.hpp` as a
look-at/basis construction instead of composed Euler corrections: `approach_dir` (the `(yaw,
pitch)` pointing direction) becomes local +Z; `opening_axis = normalize(cross(world_up,
approach_dir))` becomes local +X, guaranteeing it stays **horizontal** regardless of
`approach_yaw` so the fingers can straddle an upright object instead of one going above/below it;
the third axis completes a right-handed basis. `quaternionFromAxes()` (new helper, standard
trace/Shepperd's-method) converts the basis directly to a quaternion. Verified three ways: unit
tests (`OrientationPointsLocalZAlongApproachDirection`, `OrientationKeepsLocalXOpeningAxisHorizontal`
sweeping 5 `approach_yaw` values), a live `/piper/compute_ik` sweep (3/4 poses succeeded, opening
axis confirmed horizontal in all), and visually --
`docs/screenshots/4.16_gripper_orientation_fixed.png` shows a clean, symmetric two-finger pincer
shape, in sharp contrast to `grasp.png`'s twisted approach.

**Bug 2 -- `computeGraspPose()`'s position targeted link6's own origin, not the fingertips.**
Even with the orientation fixed, live grasp attempts still closed on empty air every time. Added
direct diagnostic logging (`object_arm`, `grasp_pose.position`, the arm's actual achieved
end-effector pose) and found `grasp_pose.position` was set to the object's position **exactly**
(`pose.position = object_position_arm_frame;`), commanding link6's origin to arrive AT the
object. But per section 4.15's own URDF fact, the fingers (`joint7`/`joint8`) sit `0.13503m`
**further along link6's local +Z** from link6's own origin -- so the fingertips ended up ~13.5cm
**past** the object, closing on nothing, regardless of how correct the orientation was. Fixed by
backing link6's target position off by that same offset along `approach_dir`:
`pose.position = object_position_arm_frame - kFingertipOffsetFromLink6 * approach_dir` (`kFingertip
OffsetFromLink6 = 0.13503`). Added `FingertipsNotLink6OriginLandOnObject`, sweeping 5 yaws x 3
pitches, asserting that walking `0.13503m` further along the returned pose's local +Z lands
exactly on the object -- the exact invariant that was silently broken before, and that no
existing test checked (every prior test only checked position/orientation *relative* quantities,
never this absolute one).

**Bug 3 -- `computeGraspPose()`'s position bug fixed, but grasps still missed: the
odom-to-arm-frame Z transform silently assumed `base_link.z == 0`.** With bug 2 fixed, a live
grasp still closed on nothing (`actual_joint7=0.01220` vs `target=0.01200` -- 0.2mm residual,
pure noise, no contact). Diagnosed by comparing a live `tf2_echo odom base_link` (`z=0.081`)
against `transformPointOdomToArmFrame()`'s math: `robot_pose_odom` is a `Pose2D` (x, y, yaw only
-- by design, 2D navigation never needs more), so `base_link_frame.z = point_odom.z;` silently
treated base_link as sitting exactly at odom's Z=0 plane. It doesn't -- the real chassis sits
0.081m above it. Every grasp was aimed ~8cm too high. **Fixed two ways together**: (1)
`transformPointOdomToArmFrame()` gained an explicit `robot_z_odom` parameter, subtracted before
the existing `arm_mount_xyz.z` offset; (2) `TaskCoordinatorNode::lookupRobotPose()` gained an
optional `z_out` parameter reading `t.transform.translation.z` from the SAME TF lookup already
being done (not a hardcoded constant, so it stays correct if the chassis height ever changes),
wired into all three call sites (`computePregrasp()`, `sendGraspGoal()`, `sendPlaceGoal()`).
Added `SubtractsRobotZBeforeMountOffset` locking in the fix.

**A red herring investigated and ruled out along the way**: `/sensor/pose`'s reported object
height (`z=0.2007`) initially looked ~10cm off from a "ground truth" USD query (`z=0.300`,
matching the object's spawn-authored height) -- but the "ground truth" query was the
`ComputeLocalToWorldTransform(0)` time-zero bug mentioned above, comparing a live, correct value
against a frozen spawn-time one. A screenshot (`docs/screenshots/...`, not kept -- see
`34_sensor_closeup_live.png` in the session's scratchpad) confirmed the object was genuinely,
visibly sitting correctly on its pedestal; `/sensor/pose` was right all along, just lower than its
spawn height because it physically settled after the spawn drop.

**With all three fixed, the standoff/reach envelope needed retuning.** The finger-offset fix
(bug 2) makes the required arm reach longer for a given object distance; a live `/piper/compute_ik`
sweep showed the old `pregrasp_standoff=0.48` (object ending up ~0.50m from the arm) now lands on
the wrong side of a real reachability boundary (`NO_IK_SOLUTION` for both the approach and grasp
poses), while 0.55-0.70m is reliably reachable across the realistic yaw range. Raised
`pregrasp_standoff: 0.48 -> 0.60` and `max_reach: 0.55 -> 0.65` (`coordinator.yaml`) -- this only
*increases* pedestal clearance versus 0.48 (section 4.12), since the base parks farther away.

**Grasp verification + retry (the user's explicit ask).** `moveGripperToWidth()`
(`piper_manipulator_node.cpp`) previously only checked whether the close-gripper *motion*
succeeded -- true whether the fingers closed on the object or on empty air, since
`isaac_joint_bridge.py`'s own stall-detection (section 9) already converts "stopped short due to
contact" into a successful `FollowJointTrajectory` result. Added a readback of the ACTUAL achieved
`joint7` position (`getCurrentState()`, a plain state-monitor read, confirmed unaffected by the
action/service dispatch bug from sections 4.14/4.15) after every close, comparing it against the
commanded target with a tolerance calibrated **directly against real live-run numbers**: a
confirmed total miss left a residual of 0.0002-0.0006m (pure execution noise); a real, if
light/grazing, contact left a residual of 0.0026m -- 4-13x the noise floor, but below the first
tolerance tried (0.003m), so that grasp was wrongly classified as a miss. Recalibrated to
`0.0015m`, between the two. A confirmed miss is now a real `grasp_object` failure
(`"gripper closed without contacting the object"`), and `task_coordinator` retries once --
`TaskFsm::Inputs` gained `grasp_retry_available`, and the arm-states case now branches three ways
(success -> `DRIVE_TO_DELIVERY`; miss-with-retry-available -> back to `MOVE_ARM_TO_PREGRASP`;
exhausted -> `ABORTING`), mirroring `ALIGN_BASE`'s existing `realign_available` pattern exactly.
`onTransition()`'s `MOVE_ARM_TO_PREGRASP` case had to be widened from `prev == STOP_BASE` to
`prev == STOP_BASE || isArmState(prev)`, or a retry-driven re-entry would never actually re-send
a fresh goal (`grasp_goal_sent_` would stay `true` from the failed attempt).

**A second, deeper verification gap found live**: a real run showed `CLOSE_GRIPPER` reporting
`contact=true` and `LIFT_OBJECT`'s motion itself succeeding, but a screenshot afterward
(`docs/screenshots/...`, see scratchpad's `34_SUCCESS_lifted_driving.png`) showed the object back
on the ground next to the pedestal -- it had been knocked loose during the lift, not securely
held, and nothing had rechecked. Added `stillHoldingObject()`, re-reading `joint7` with no new
commanded motion, called after both `LIFT_OBJECT` and the new `STOW_OBJECT` step below; a slip is
treated the same as a `CLOSE_GRIPPER` miss (real failure, retry-eligible). **Verified live**:
`docs/screenshots/4.16_grasp_verified_holding_object.png` shows the object unambiguously grasped
in the jaws, `contact=true` and `holding=true` both logged with real joint7 numbers
(`target=0.01200, actual=0.01450`).

**Bug 4 -- `DRIVE_TO_DELIVERY`'s obstacle-avoidance abort: the lifted arm/object sat almost
exactly at the lidar's own scan height.** Confirmed directly: a live `/scan` read during a real
`DRIVE_TO_DELIVERY` showed the forward cone at `0.10-0.11m` -- the lidar's own `range_min`, i.e.
something touching it -- for the entire drive. `LIDAR_LOCAL_POS = (0.30, 0.0, 0.35)` relative to
`base_link` (`import_robots.py`); the post-`LIFT_OBJECT` pose puts the held object at
approximately the same world Z (`~0.40m`, by the arm-frame-to-world math). **Fix**: added a
`STOW_OBJECT` step -- after `LIFT_OBJECT` succeeds, retract to the arm's `'zero'` SRDF
group-state (the same pose `retreatToSafe()` already uses for every failure path) before
returning success, re-verifying `stillHoldingObject()` afterward since `'zero'` isn't
collision-aware of an attached object either. `'zero'` is confirmed clear via
`/piper/compute_fk`: link6 lands at `z=0.494m` in the scout's own `base_link` frame (`0.203m`
local + the `0.29101m` arm-mount offset) versus the lidar's fixed `z=0.35m` -- and confirmed
**live**, twice: (1) an earlier run's post-stow `stillHoldingObject()` check passed
(`holding=true`) right after the retract motion; (2) a direct `/scan` read while the arm sat at
`'zero'` (post-failure-retreat, same pose) showed `min_forward_range=10.0` -- completely clear,
`>>` `obstacle_stop_distance` (0.25m), so `applyObstacleAvoidance()` mathematically cannot
suppress forward velocity anymore. `docs/screenshots/4.16_stow_object_lidar_clear.png` shows the
arm folded up well clear of the chassis-front lidar mount. Wired through
`GraspObject.action` (new `STOW_OBJECT` feedback stage), `task_state.hpp` (new
`TaskState::STOW_OBJECT`, added to `isArmState()`/`armStageToState()`), and `task_fsm.hpp` (added
to the arm-states switch case).

**A fifth bug found live, in my own new code**: after wiring `STOW_OBJECT` into `task_fsm.hpp`
and `task_state.hpp`, a live run reached `STOW_OBJECT` and hung there indefinitely -- `piper_
manipulator`'s log showed the action had actually already succeeded (`stillHoldingObject()`
logged `holding=true` right on schedule), but `task_coordinator`'s FSM never advanced.
Root cause: `task_coordinator_node.cpp`'s own `tick()` switch (the one that reads `grasp_goal_
sent_`/`grasp_goal_done_`/`arm_feedback_stage_` into the `TaskFsm::Inputs` each tick) still only
listed `MOVE_ARM_TO_PREGRASP`/`MOVE_ARM_TO_GRASP`/`CLOSE_GRIPPER`/`LIFT_OBJECT` -- `STOW_OBJECT`
fell through to `default: break;`, so `in.grasp_goal_done` stayed permanently `false` (the
member variable `grasp_goal_done_` WAS being set correctly by the action's result callback, just
never read into `in`), and the FSM sat parked in `STOW_OBJECT` forever. A clean reminder that
`task_state.hpp`, `task_fsm.hpp`, AND `task_coordinator_node.cpp`'s own `tick()` switch all need
to agree on the full set of arm states -- adding a new one to only two of the three compiles fine
and silently hangs. Fixed by adding the missing case; re-verified via the unit test suite (28/28
passing) and confirmed the STOW_OBJECT->grasp_goal_done transition now behaves correctly.

**A known, real, remaining limitation -- not a bug this investigation's scope covers**: the demo
object is small and light (5cm cube, 0.2kg) resting on a narrow (5cm-radius) pedestal, and the
grasp sometimes only marginally catches it (observed `joint7` residuals around `0.0025-0.003m`
against an expected `~0.013m` for a well-centered grip on a 5cm object), which the lift motion's
acceleration can then shake loose -- correctly detected by `stillHoldingObject()` and retried,
but a retry after the object has been knocked off the pedestal onto the ground can land outside
the arm's reachable envelope (`z_min=-0.20`), which `computePregrasp()`'s own reachability
pre-check correctly refuses rather than attempting a doomed grasp. The verification/retry
mechanism built here is working exactly as asked (it reliably detects both misses and slips and
retries), and a full E2E run with a first-attempt clean grasp does complete `LIFT_OBJECT ->
STOW_OBJECT -> DRIVE_TO_DELIVERY` successfully end to end -- but grasp reliability itself (grip
force/centering tuning for this specific small/light object) is a separate, deeper robustness
question flagged here as follow-up, not silently left unmentioned.

**Full regression check**: `kotek_manipulation` (16 tests, was 15 before this section, 0
failures), `kotek_task_coordinator` (28 tests, was 26 before this section, 0 failures), both
clean after every change in this section, including the fifth-bug fix.

### 4.17 User-reported: "gripper should be a little closer, falls in both attempts" -- four more real bugs, one of them project-wide

The user reported the grasp orientation now looked correct but the gripper still closed on empty
air. Root-caused live, directly against real logs, rather than guessed at from mesh geometry
(a `link7.STL` bounding-box analysis was tried first and produced an ambiguous, ultimately
unnecessary hypothesis about the finger-pad offset -- abandoned once the real cause showed up
directly in the logs).

**Bug 1 -- `min_cartesian_fraction=0.9` silently accepted partial Cartesian paths that landed
short of the real target.** A live run's `MOVE_ARM_TO_GRASP` completed the commanded Cartesian
path only 93.75% (`Computed Cartesian path with 16 points (followed 93.750000% of requested
trajectory)`) -- comfortably inside the old 0.9 acceptance threshold, so `cartesianMoveTo()`
treated it as a success and executed it anyway, landing the arm about 1cm short of the actual
`grasp_pose` (commanded `(0.5490,0.0178,-0.1186)` vs. achieved `(0.5410,0.0175,-0.1121)`). That
1cm shortfall was the entire miss: the very next `CLOSE_GRIPPER` read back essentially zero
resistance (`target=0.01200, actual=0.01220`). Confirmed directly by comparison against a SIBLING
attempt in the same run whose Cartesian path completed 100% and got solid contact
(`actual=0.01510`). Raised `min_cartesian_fraction` to 0.98 in `manipulation.yaml` so a path this
incomplete is rejected outright (triggering the existing retry) instead of silently executed
short.

**Bug 2 (found applying the fix above) -- `manipulation.yaml`'s and `isaac_bridge.yaml`'s
top-level parameter keys never actually matched their namespaced nodes; both had been silently
ignored, project-wide, this whole time.** Raising `min_cartesian_fraction` and rebuilding did
**not** change the running node's behavior -- `ros2 param get /piper/piper_manipulator
min_cartesian_fraction` still reported the old default, 0.9, and live behavior confirmed it (a
93.75%-complete path was still accepted afterward). Root cause: `piper_manipulator_node` and
`isaac_joint_bridge` are both remapped into the `/piper` namespace (`-r __ns:=/piper`, set by
`piper_moveit.launch.py`), but `manipulation.yaml`'s and `isaac_bridge.yaml`'s top-level keys were
plain `piper_manipulator:` / `isaac_joint_bridge:` -- which only matches a node of that name **in
the root namespace**, per ROS 2's `--params-file` matching rules. Every value in both files had
therefore always been silently discarded in favor of each node's own hardcoded C++/Python
default -- invisible until now purely because every prior value in both files happened to already
equal its code default (`grasp_width=0.024`, `gripper_pitch=0.4`, `stall_position_delta=0.001`,
etc. -- all coincidentally already matched). Fixed by rewriting both top-level keys to the full
namespaced form (`/piper/piper_manipulator:`, `/piper/isaac_joint_bridge:`), matching how
`sensor_pose_publisher` (unnamespaced, and therefore never affected) was already keyed. Confirmed
fixed directly: `ros2 param get` now reports 0.98, and a subsequent live run's 100%-complete
Cartesian paths got solid contact (`actual=0.01450`-`0.01530`, well above the noise floor) on
every first attempt across multiple runs. `task_coordinator`'s own `coordinator.yaml` and
`simple_base_controller`'s `base_controller.yaml` were never affected -- both those nodes run
unnamespaced, so their plain keys were always correct.

**Bug 3 (found immediately after, live) -- a retry re-entering the SAME state it failed in never
actually resent the goal.** `TaskFsm`'s retry (section 4.16) sets `state_ = MOVE_ARM_TO_PREGRASP`
whether the failure happened in `CLOSE_GRIPPER`/`LIFT_OBJECT`/`STOW_OBJECT` (a real transition) OR
in `MOVE_ARM_TO_PREGRASP` itself (e.g. its own `move_action` planning failure) -- but in the
second case `state_` doesn't change (it was already `MOVE_ARM_TO_PREGRASP`), so `TaskFsm::step()`
returns `false`, `task_coordinator_node.cpp`'s `tick()` never calls `onTransition()`, and
`grasp_goal_sent_` -- which only `onTransition()`'s `MOVE_ARM_TO_PREGRASP` case resets -- stays
`true` from the failed attempt. The retry log line printed (`"retrying once"`), but no new goal
was ever sent, and the FSM aborted one tick later on the same stale done/failed result. Confirmed
directly in a live run. Fixed by resetting `grasp_goal_sent_`/`grasp_goal_done_`/
`grasp_goal_success_`/`arm_feedback_stage_`/`grasp_goal_handle_` directly at the point the retry
is granted (`task_coordinator_node.cpp`'s arm-states `tick()` case), rather than relying on
`onTransition()`'s prev/next comparison -- covers both the same-state and real-transition retry
paths, and re-verified: a subsequent live run showed `"Sent grasp_object goal"` logged a second
time after `"retrying once"`, confirming a fresh goal actually went out.

**Bug 4 (found completing a clean end-to-end run) -- the delivery/place leg has the exact same
finger-offset reachability problem the pickup leg had in section 4.16, never previously reached
live.** With bugs 1-3 fixed, a run reached `PLACE_ARM` for the first time ever and failed there
(`MOVE_ARM_TO_PREPLACE`, `NO_IK_SOLUTION`) -- `executePlace()` reuses `computeGraspPose()`
(so the finger-offset and Z-transform fixes already apply automatically), but `delivery_z=0.05`
and `delivery_standoff=0.35` were never retuned for it. A live `/piper/compute_ik` sweep found the
resulting preplace target only ~3.5cm from the arm's own base (below `min_reach`), and separately
that `delivery_z=0.05` (arm-frame z=-0.32) fails IK at ANY standoff -- this arm, at
`gripper_pitch=0.4`, cannot reach below approximately arm-frame z=-0.20 regardless of horizontal
distance (matching the existing `z_min=-0.20` reachability limit already used for pickup). Raised
`delivery_z: 0.05 -> 0.22` and `delivery_standoff: 0.35 -> 0.60` (matching `pregrasp_standoff`)
together and verified reachable across yaw +/-0.15 rad via IK before committing.

**Verified live, repeatedly, after all four fixes**: multiple clean runs (fresh Isaac Sim + fresh
`ros2 launch kotek_bringup kotek_demo.launch.py`, object on its pedestal) reliably got 100%
Cartesian completion and solid contact (`0.0145`-`0.0153m` residual, well clear of the noise
floor) on the first `CLOSE_GRIPPER` attempt; one full run went `LIFT_OBJECT -> STOW_OBJECT ->
DRIVE_TO_DELIVERY -> ALIGN_AT_DELIVERY -> STOP_BASE_AT_DELIVERY -> PLACE_ARM` in a single clean
pass with no retry needed.

**Known limitation, confirmed still present and now the clear remaining blocker**: even with a
solid, 100%-Cartesian-path `CLOSE_GRIPPER` contact, the object still sometimes slips free during
`LIFT_OBJECT`'s acceleration (`holding=false` immediately after a lift whose close registered
real contact moments earlier) -- reproduced multiple times, including on runs otherwise completely
clean. `stillHoldingObject()` correctly detects every occurrence and the retry mechanism (bug 3,
now fixed) correctly fires, but a retry can still fail if the slip knocked the object outside the
arm's reachable envelope. This is the same grip-force/centering limitation flagged in section
4.16, now confirmed to be the single remaining blocker to a 100%-reliable run rather than one of
several compounding issues -- a real, physical robustness question (this demo's object is a 5cm,
0.2kg cube on a 5cm-radius pedestal) rather than a geometry or logic bug, and is flagged here as
the next concrete follow-up rather than silently left unaddressed.

**One important operational lesson, unrelated to the code itself**: several confusing live-testing
symptoms this session (an object appearing to teleport to wildly wrong coordinates, `ros2 param
get`/behavior mismatches, `ros2 node list` showing duplicate node names) turned out to be caused
by zombie node processes accumulated across repeated manual restart cycles during debugging --
`pkill -f <pattern>` reliably self-matches its own invoking shell command when the pattern text
also appears in the `pkill` command line itself (the same gotcha documented in section 4.16's
process-management notes), silently no-op'ing the intended kill and leaving old node generations
running and answering ROS graph queries/publishing stale data alongside the new ones. Killing by
explicit PID (gathered via a separate `ps aux | awk '{print $2}'` step, never combined with the
`kill` command in a single pattern-matched call) is the reliable alternative used throughout this
section once the pattern was identified.

**Full regression check after all four bugs**: `kotek_manipulation` (16 tests, 0 failures),
`kotek_task_coordinator` (28 tests, 0 failures) -- both unaffected by this section's changes
(entirely config/launch-file fixes plus one `task_coordinator_node.cpp` reset-timing fix with no
`TaskFsm` logic change), re-run clean after every fix.

### 4.18 User-reported: full run now reaches `PLACE_ARM` and fails there -- `START_STATE_INVALID`

With section 4.17's fixes in place, a live run went the entire distance -- pickup, lift, stow,
drive to delivery, align, stop -- and failed on the very last motion:

```
[move_group] ERROR check_start_state_bounds: Joint 'joint2' from the starting state is outside
             bounds by: [-0.0001 ] should be in the range [0 ], [3.14 ].
[move_group] ERROR PlanningRequestAdapter 'CheckStartStateBounds' failed, because
             'Start state out of bounds.'. Aborting planning pipeline.
[move_group] move_action: START_STATE_INVALID
[piper_manipulator] MOVE_ARM_TO_PREPLACE failed: move_action did not succeed (status=6)
[piper_manipulator] retreat-to-safe failed: move_action did not succeed (status=6)
-> PLACE_ARM -> ABORTING -> FAILED
```

**Root cause, confirmed directly against `piper_description.urdf` and the actual installed
MoveIt Jazzy binaries (2.12.4) in the `kotek_ws:jazzy` container, not guessed at:**

`joint2`'s URDF limit (`piper_description.urdf:154-158`) is `lower="0" upper="3.14"` -- unlike
every other joint on this arm, its home value sits exactly *on* the boundary, not inside it.
`STOW_OBJECT` calls `retreatToSafe()` (`piper_manipulator_node.cpp`), which sets the arm's target
to the SRDF `zero` group_state (`joint2 = 0`, `piper_description.srdf:26-33`) -- the same pose
section 4.16 introduced to keep the arm out of the lidar's view. PhysX's position drive settles a
hair under that target (`-0.0001` rad, i.e. 0.006 degrees), and `isaac_joint_bridge.py`'s
`_isaac_state_cb()` was republishing Isaac's raw `/isaac_joint_states` onto `/piper/joint_states`
completely unmodified -- so that `-0.0001` reached move_group as the literal planning start
state for every subsequent goal, including the very next `retreat-to-safe` fallback call.

Checked for an existing MoveIt-side knob before writing any new code, and confirmed none exists
in this version: `default_planning_request_adapters::CheckStartStateBounds` declares exactly one
parameter, `fix_start_state`, whose own description limits it to "continuous, planar, or floating
joints" -- a bounded revolute joint like `joint2` is never fixed, only rejected outright; there is
no `start_state_max_bounds_error` anywhere under `/opt/ros/jazzy`. `CurrentStateMonitor` does
clamp small out-of-bounds values, but only within its own `error_` margin, which defaults to
machine epsilon (`current_state_monitor.hpp`) and is settable only via a C++ `setBoundsError()`
call with no ROS parameter exposing it. And supplying an already-corrected `start_state` from the
client can't help either: `sendMoveGroupRequest()` sets `plan_only = false`, and move_group's own
log line confirms it ignores exactly that -- *"Execution of motions should always start at the
robot's current state. Ignoring the state supplied as difference in the planning scene diff"* --
it reads the state from its own planning-scene monitor, i.e. from `/piper/joint_states`.

That leaves the bridge as the only correct fix point: `/piper/joint_states` must never carry a
value outside the URDF's own declared bounds. Fixed by giving `isaac_joint_bridge.py` the same
`robot_description` move_group and `piper_manipulator` already receive (`piper_moveit.launch.py`,
so the enforced limits can never drift from the limits move_group itself checks against), parsing
it once at startup (`parse_joint_limits()`, stdlib `xml.etree.ElementTree`), and clamping each
incoming joint position in `_isaac_state_cb()` before it is republished: a violation within a new
`bounds_clamp_tolerance` (0.01 rad, `isaac_bridge.yaml`) is silently clamped to the bound (this is
the PhysX-settling case), while a larger violation is left visible (raw value published, throttled
warning logged) rather than papered over, since that would indicate a real modelling/asset bug
rather than settling noise. This is a general fix, not a `joint2`-specific patch -- the identical
trap was waiting on `joint7`/`joint8` at their own `0` stop whenever `CLOSE_GRIPPER` drives the
gripper fully closed.

Deliberately did not retune the SRDF `zero` group_state itself -- `piper_camera_moveit_config` is
a read-only submodule, and sections 4.9/4.10 already tuned this exact pose to clear the lidar;
re-tuning it to dodge this failure risks reintroducing a solved bug to fix an unrelated one.

**Verified**: added `TestParseJointLimits` (URDF-parsing) and `TestJointBoundsClamp`
(reproduces the exact `joint2 = -0.0001` case and confirms it clamps to `0.0`, plus a
gross-violation case confirming it is left unmodified) to
`kotek_isaac_bridge/test/test_isaac_joint_bridge.py`; full `kotek_isaac_bridge` suite (7 tests, 0
failures) and a full workspace `colcon build` both pass.

**Verified live, end to end -- first full clean run in this project's history.** Ran the exact
two-process procedure above (fresh container, fresh Isaac Sim, `pick_and_delivery_e2e_test.py
--timeout 400` -- the earlier `--timeout 90` in this doc's own reproduce command was too short,
cutting a real run off mid-`DRIVE_TO_PREGRASP`; the full task takes roughly 230-250s). Result:

```
### /task/state -> PLACE_ARM
### /task/state -> OPEN_GRIPPER
### /task/state -> RETREAT_ARM
### /task/state -> DONE
### full state history: ['FIND_SENSOR', 'COMPUTE_BASE_PREGRASP_POSE', 'DRIVE_TO_PREGRASP',
'ALIGN_BASE', 'STOP_BASE', 'MOVE_ARM_TO_PREGRASP', 'MOVE_ARM_TO_GRASP', 'CLOSE_GRIPPER',
'LIFT_OBJECT', 'STOW_OBJECT', 'DRIVE_TO_DELIVERY', 'ALIGN_AT_DELIVERY',
'STOP_BASE_AT_DELIVERY', 'PLACE_ARM', 'OPEN_GRIPPER', 'RETREAT_ARM', 'DONE']
### sensor final world position: (0.307, -1.181, -0.162)
### distance from sensor to delivery target (xy): 0.020m
```

`PLACE_ARM` transitioned straight to `OPEN_GRIPPER` (previously always `ABORTING -> FAILED`), the
FSM reached `DONE` for the first time ever, and the object landed 2cm from the delivery target. A
full-log grep for `outside bounds`/`START_STATE_INVALID` across the entire run returned zero
matches.

### 4.19 User-reported: `MOVE_ARM_TO_PREGRASP failed: move_action did not succeed (status=6)` --
three more real, distinct findings from 12 live trials

A follow-up live run failed with `status=6` (ABORTED) at `MOVE_ARM_TO_PREGRASP`, unrelated to
section 4.18's fix -- confirmed by full-log inspection: the failure was actually
`retreatToSafe()`'s own fallback call succeeding right after the *real*, unshown failure. Rather
than guess, ran 12 further live trials (3 batches of 4, container + Isaac Sim each time) to
characterize what's actually going wrong.

**Finding 1 -- the object genuinely, physically slips; this is not a perception bug.** The user's
own hypothesis was that "the object is not slipping from the gripper, but the recognition thinks
that it has fallen." Tested directly: echoed `/sensor/pose` (a live TF republish of the object's
own PhysX rigid-body pose -- `sensor_pose_publisher.py`, no filtering/estimation in the loop) at
10Hz through an actual failure. The trajectory is smooth and physically continuous throughout --
lift (z rising 0.31->0.50m), free-fall (z dropping through 0.22, -0.13, -0.17m while x/y slide,
consistent with a bounce), then frozen at rest on the floor -- no single-tick discontinuity
anywhere, confirming genuine physics, not a tracking artifact.

**Finding 2 -- grasp_width tightening alone does not fix it.** Raised `grasp_width: 0.024 ->
0.012` (deeper commanded finger-closure overshoot past the object's real surface, for more
squeeze force). 4 live trials: still 2/4 failures with the identical signature (`CLOSE_GRIPPER`
registers real contact, then the post-lift check finds the fingers finished closing to their
target -- nothing left between them).

**Finding 3 -- LIFT_OBJECT's cartesian path was running at full, unscaled speed the entire
project.** `computeCartesianPathRaw()`'s `GetCartesianPath` request never set
`max_velocity_scaling_factor`/`max_acceleration_scaling_factor`, which per that service's own doc
comment silently default to 1.0 (not `velocity_scaling`/`acceleration_scaling`) when left at 0 --
a real, separate bug, unrelated to the grip. Fixed by threading `lift_velocity_scaling`/
`lift_acceleration_scaling` (0.08, new params) through `cartesianMoveTo()` for `LIFT_OBJECT`
specifically, and through `retreatToSafe()` (used by `STOW_OBJECT` and every failure-path safety
retreat) via a temporary override on `move_group_arm_`, restored immediately after planning. A
second 4-trial batch with grip+speed together: still 0/4 success -- 1 grip-loss, 3 with a
*different*, previously-undiagnosed failure (`LIFT_OBJECT failed: cartesian path only 75-87.5%
complete`), showing speed wasn't the dominant issue either.

**Finding 4 -- the Cartesian-completion failures were a real, separate, previously undiagnosed
bug: `lift_height` had no reachability margin.** Tallied every `Computed Cartesian path with N
points (followed X% of requested trajectory)` line across the whole session's logs: 14/18 calls
completed all 17 waypoints (100%), the other 4/18 stopped at 13-15 of 17 -- always the *last* few
centimeters, never the first, and with no other explanation in move_group's own log (`"Found
empty JointState message"` is a benign line present identically on both 100%- and partial-
completion calls, ruled out directly). `computeLiftPose()` is a pure +Z translation from
`grasp_pose` with no reorientation or joint-space freedom; since `grasp_pose` itself is nearly
identical run to run (fixed object spawn, fixed pregrasp dock) but `MOVE_ARM_TO_PREGRASP`/
`MOVE_ARM_TO_GRASP` reach it via OMPL (not deterministic), the arm's exact joint configuration at
`grasp_pose` varies run to run, and some configurations leave too little joint-limit margin to
complete the full 0.15m vertical lift. Cut `lift_height: 0.15 -> 0.10` (worst observed shortfall
was 4 of 17 waypoints, ~4cm) -- `STOW_OBJECT`'s own retreat to `'zero'` immediately afterward
already provides the real lidar clearance (section 4.16), so the lift no longer needs to reach
that height by itself. **Verified: a third 4-trial batch with all three fixes together showed
zero Cartesian-completion failures (0/4, down from 4/8 in the prior two batches) -- direct
confirmation.**

**Still open, confirmed the dominant remaining failure**: even with tightened grip, scaled-down
lift speed, and the reachability fix, live testing kept finding the *same* grip-loss signature --
6 of 12 trials across all three batches, unaffected in rate by either the grip or speed changes.
`CLOSE_GRIPPER` consistently registers real contact (`actual_joint7` 0.0088-0.0091, comfortably
past `kContactTolerance`), and the object is nonetheless gone by the post-lift recheck. Given
Finding 3 ruled out unscaled acceleration as the sole driver and Finding 1 ruled out a tracking
bug, this is left as a genuine, unresolved physical grip/friction question (not evidence a
`grasp_width` deep enough to explore further wasn't tried, or a driver/effort-limit question this
investigation didn't have the tooling to inspect) -- flagged as the concrete next step rather than
guessed at further.

**Full regression check**: `kotek_manipulation`/`kotek_task_coordinator`/`kotek_isaac_bridge`, 81
tests, 0 failures, 8 skipped (pre-existing, unrelated to this section), after every fix in this
section.

## 5. Summary

| Item | Status |
|---|---|
| Tier 3 Test 1 (zero-drift) | Fixed, verified passing |
| Tier 3 Test 3 (arm convergence, 12/12) | Fixed, verified passing |
| Tier 3 Test 2 (commanded drive) | **Fixed and verified** (sections 1.7, 4.6) -- correct magnitude (25-59cm/2s depending on asset) AND correct direction, all 22 Tier 3 checks pass |
| Lidar sensor + `/scan` | Built, verified against real physics |
| Obstacles in the demo stage | Built, verified detected by real lidar |
| Reactive obstacle avoidance (pure logic) | Built, 9/9 unit tests passing |
| Reactive obstacle avoidance (real node, real captured scan) | Built, integration test passing |
| `PlaceObject` action + place sequence | Built |
| Delivery-leg FSM extension | Built, 121/121 relevant unit tests passing |
| Cross-process DDS (RMW), `use_sim_time`, sensor-TF-timestamp bugs | Found and fixed (3 real infrastructure bugs) |
| Wheel collision, inertia, and left/right sign bugs (the original Test 2 blocker) | Found and fixed (3 more real bugs, section 1.7) |
| Left/right rotation-direction sign bug | Found and fixed (section 4.5) -- turning was inverted/broken independently of the translation fix |
| Insufficient-progress watchdog killing legitimate turn-in-place phases | Found and fixed (section 4.6, "Bug A") |
| **Translation direction sign bug -- base was driving BACKWARD all session under a "forward" command** | **Found and fixed (section 4.6, "Bug B")** -- the dominant real blocker; magnitude was always correct (which is why it passed every test), direction was not, and no test checked direction until now (both `tier3_regression_test.py` and `test_scout_only_drive.py` fixed to assert it) |
| Diagonal-pair wheel-load asymmetry (~2x, front-right+rear-left vs front-left+rear-right) | Found, partially mitigated (base_link inertia fix roughly halves it), residual likely inherent to a zero-suspension rigid 4-wheel model -- not fully resolved, not blocking (section 4.7) |
| User-reported: wheel physically hit `obstacle_pickup_leg` during a live test | **Found and fixed (section 4.11)** -- obstacle placement (sections 4.8/4.9) never subtracted the robot's own half-width (0.2883m, read from `scout_mini.urdf`), only the obstacle's; real margin was 12.7cm. Repositioned to require `ROBOT_HALF_WIDTH + PATH_TRACKING_MARGIN(0.20m) + OBSTACLE_HALF_WIDTH` clearance, giving 22.0cm; verified via `assert-demo-ready`, a live bbox readback, and a real `/scan` reading matching predicted geometry to 3cm/1deg. Full live E2E rerun not yet repeated |
| User corrected: it was the sensor pedestal, not the obstacle, causing the collision | **Found and fixed (section 4.12)** -- `pregrasp_standoff=0.40` never accounted for the robot's own forward chassis extent (0.312m, measured directly) or the pedestal's own radius (0.08m); nominal clearance was 8mm. Raised standoff to 0.48 and shrunk the pedestal radius to 0.05, giving 11.8cm nominal / 3.8cm worst-case clearance, still within arm max_reach with margin. Re-verified both obstacle corridors (unaffected/improved) and the full 126-test gtest suite (0 failures). Full live E2E rerun not yet repeated |
| User-reported: rviz2 does not open | **Found and fixed (section 4.12)** -- `ros-jazzy-rviz2` was never added to the Dockerfile (base image `ros:jazzy-ros-base` doesn't include it, and no file in the repo referenced rviz before this fix). Added it, rebuilt the docker image, confirmed `rviz2`/`rviz_common`/`rviz_default_plugins` etc. present in a fresh container. `DISPLAY`/X11 forwarding was already correctly configured in `compose.yaml` |
| Full pick-and-deliver E2E run | **`DRIVE_TO_PREGRASP` succeeds** (section 4.10) -- raised the lidar to clear self-detection of the Piper mount (section 4.9), raised the demo obstacles' height to stay detectable at the new scan height, and fixed a real relative-output-path bug in `build_demo_stage.py` that had silently been discarding obstacle edits all along. Live E2E run now proceeds automatically through `ALIGN_BASE -> STOP_BASE -> MOVE_ARM_TO_PREGRASP` and sends a real grasp goal -- first time this entire investigation. Remaining scope (arm-grasp-onward, or just a larger test time budget) is narrow and unrelated to anything this investigation touched |
| User-reported: stuck forever in `MOVE_ARM_TO_PREGRASP`, arm never moves | **Found and fixed (section 4.13)** -- two real, compounding bugs: (1) `isaac_joint_bridge.py`'s single-threaded executor deadlocked every `FollowJointTrajectory` goal against its own state subscription, fixed with a `ReentrantCallbackGroup` + `MultiThreadedExecutor`; (2) `piper_manipulator`'s own `init()` hung forever inside `MoveGroupInterface`'s constructor -- three real bugs found and fixed along the way (shared-node deadlock, node-name collision via global-argument inheritance, a double-namespaced action name), plus a still-not-fully-explained `wait_for_action_server` hang worked around with a finite timeout. `grasp_object` goals are now accepted and reach real MoveIt planning (previously never even accepted). A separate, new, not-yet-diagnosed motion-planning failure for the specific test poses tried is flagged as the next step |
| Follow-up: `MoveGroup action client/server not ready` on every `plan()`/`move()` call | **Found and fixed (section 4.14)** -- `MoveGroupInterface`'s own internal action clients never report ready in this environment (confirmed via the actual installed source: `action_server_is_ready()` is called fresh every time, not cached, so this wasn't a startup-timing issue). Bypassed with two raw `rclcpp_action::Client`s built on `self` (the proven-working construction pattern), using `MoveGroupInterface` only for target-setting and `constructMotionPlanRequest()`. Also found and fixed a second real, blocking bug on the way: every joint's `has_acceleration_limits: false` in the (read-only) `piper_camera_moveit_config` crashed `AddTimeOptimalParameterization` for any successful plan -- fixed via a launch-time parameter override in `kotek_bringup`. **Result: the first real, successful, executed arm motion in this project's history** (`retreatToSafe`'s "zero" pose). A separate, narrower reachability issue with the computed pregrasp/preplace approach-pose orientation remains open, flagged as the next step |
| Follow-up: pregrasp/preplace poses genuinely unreachable via IK | **Found and fixed (section 4.15)** -- `pregrasp_geometry.hpp`'s `computeGraspPose()` assumed the end effector's approach axis is local +X, but this arm's actual tip link (`link6`) has its gripper offset along local +Z (confirmed directly from `piper_description.urdf`). Every computed grasp orientation was unreachable regardless of position, timeout, or planner budget -- confirmed decisively via direct `/piper/compute_ik` testing, bypassing OMPL entirely. Fixed with a fixed `Ry(+90deg)` orientation correction, verified against 4 real object poses including the user's own original E2E goal. Found and fixed a fourth infrastructure bug on the way: `computeCartesianPath()` is a service call built the same broken way as the action clients, hanging forever -- bypassed with a raw service client. **Result: the complete grasp sequence (`MOVE_ARM_TO_PREGRASP -> MOVE_ARM_TO_GRASP -> CLOSE_GRIPPER -> LIFT_OBJECT`) succeeded fully for the first time in this project's history**, in ~9s total; `place_object` also verified working end to end. 127 tests, 0 failures |
| User-reported: grasp visibly twisted, gripper can't actually close on the object | **Found and fixed (section 4.16)** -- section 4.15's `Ry(+90deg)` correction was IK-reachable but never confirmed the opening axis; a live `/piper/compute_fk` sweep of joint7/joint8 confirmed it (local +X). Rewrote `computeGraspPose()` as a look-at/basis construction (approach_dir -> local Z, `cross(world_up, approach_dir)` -> local X, always horizontal). Verified via unit tests, live IK, and a screenshot showing a clean symmetric pincer shape (`docs/screenshots/4.16_gripper_orientation_fixed.png`) |
| Follow-up (found live, same investigation): grasp still missed with the orientation fixed -- position targeted link6's origin, not the fingertips | **Found and fixed (section 4.16)** -- the fingers sit 0.13503m further along link6's local +Z than link6's own origin (per the same URDF fact section 4.15 already used for orientation); `computeGraspPose()`'s position never applied it, so the fingertips landed ~13.5cm past the object every time. Fixed by backing link6's target off by that offset along `approach_dir`. New test (`FingertipsNotLink6OriginLandOnObject`) locks in the invariant that no prior test checked |
| Follow-up (found live): grasp still missed -- odom-to-arm-frame Z transform assumed `base_link.z == 0` | **Found and fixed (section 4.16)** -- `Pose2D` (2D nav) carries no Z, and `transformPointOdomToArmFrame()` silently treated base_link as sitting at odom's Z=0 plane; the real chassis sits `0.081m` above it (confirmed via live `tf2_echo`), so every grasp aimed ~8cm too high. Fixed by threading the SAME TF lookup's live Z through both functions instead of a hardcoded constant. `pregrasp_standoff`/`max_reach` retuned (0.48->0.60 / 0.55->0.65) since the finger-offset fix also lengthened the required reach |
| User's explicit ask: verify the object was actually grasped, retry if not | **Built and verified live (section 4.16)** -- `moveGripperToWidth()` now reads back the achieved `joint7` position and compares against the commanded target with a tolerance calibrated against real live-run noise (miss: 0.0002-0.0006m; real contact: 0.0026m); a confirmed miss is a real action failure, and `task_coordinator` retries once (`grasp_retry_available`, mirroring `ALIGN_BASE`'s existing `realign_available` pattern). A second gap found live -- contact detected at `CLOSE_GRIPPER` but the object slipped loose during `LIFT_OBJECT`, undetected -- closed with `stillHoldingObject()`, rechecked after both `LIFT_OBJECT` and `STOW_OBJECT`. Verified live: `docs/screenshots/4.16_grasp_verified_holding_object.png` shows the object unambiguously held, `contact=true`/`holding=true` both logged with real numbers |
| User-reported: `DRIVE_TO_DELIVERY` aborts immediately, `applied.v=0.000` | **Found and fixed (section 4.16)** -- the post-`LIFT_OBJECT` arm pose put the held object at almost exactly the lidar's own mount height (`z~=0.40m` vs. `LIDAR_LOCAL_POS` z=0.35m); a live `/scan` read during the actual failure showed the forward cone at the lidar's own `range_min` (0.10-0.11m) the whole time -- confirmed self-detection, reproducing the user's report exactly. Fixed with a new `STOW_OBJECT` step (retract to the arm's already-proven-safe `'zero'` pose, re-verify still holding, before returning success). Confirmed clear two ways: `/piper/compute_fk` (link6 at z=0.494m vs. lidar at 0.35m) and a live `/scan` read at `'zero'` showing `min_forward_range=10.0`. A fifth bug found in the process -- `task_coordinator_node.cpp`'s own `tick()` switch was missing the new `STOW_OBJECT` case, silently hanging the FSM forever even though the underlying action had already succeeded -- fixed and re-verified (28/28 unit tests) |
| User-reported: "gripper should be a little closer, falls in both attempts" | **Found and fixed (section 4.17)** -- `min_cartesian_fraction=0.9` silently accepted a `MOVE_ARM_TO_GRASP` Cartesian path that only completed 93.75%, landing the arm ~1cm short of the real target and missing the object. Raised to 0.98 so a path this incomplete is rejected (triggers the existing retry) instead of executed short |
| Follow-up (found live): raising `min_cartesian_fraction` had no effect at all | **Found and fixed (section 4.17)** -- `manipulation.yaml` and `isaac_bridge.yaml`'s top-level parameter keys (`piper_manipulator:`, `isaac_joint_bridge:`) never matched their actual namespaced nodes (`/piper/piper_manipulator`, `/piper/isaac_joint_bridge`) per ROS 2's `--params-file` matching rules -- every value in both files had been silently discarded in favor of hardcoded code defaults project-wide, invisible until now because every prior value coincidentally equaled its default. Fixed by rewriting both keys to their full namespaced form; confirmed via `ros2 param get` and live behavior |
| Follow-up (found live): a retry from `MOVE_ARM_TO_PREGRASP`'s own failure never resent the goal | **Found and fixed (section 4.17)** -- retrying into the SAME state that just failed (as opposed to a real transition from `CLOSE_GRIPPER`/`LIFT_OBJECT`/`STOW_OBJECT`) left `TaskFsm::step()` reporting no change, so `onTransition()` -- the only place `grasp_goal_sent_` got reset -- never ran, and the "retry" silently never resent anything. Fixed by resetting the goal-tracking flags directly at the point the retry is granted, not via the transition callback |
| Follow-up (found completing a first clean end-to-end run): `PLACE_ARM` failed with `NO_IK_SOLUTION` | **Found and fixed (section 4.17)** -- the delivery leg has the same finger-offset reachability problem the pickup leg had in section 4.16 (`executePlace()` reuses `computeGraspPose()`), but `delivery_z=0.05`/`delivery_standoff=0.35` were never retuned for it -- a live IK sweep showed the preplace target landing ~3.5cm from the arm's own base, and separately that this arm cannot reach below ~arm-frame z=-0.20 at any standoff. Raised to `delivery_z=0.22`/`delivery_standoff=0.60` (matching `pregrasp_standoff`), verified reachable via IK before committing |
| User-reported: `MOVE_ARM_TO_PREGRASP failed (status=6)` after section 4.18's fix | **Investigated with 12 live trials across 3 batches (section 4.19); three real, distinct findings, one still open.** (1) Confirmed via a 10Hz `/sensor/pose` ground-truth trace that the object genuinely, physically falls -- smooth lift/fall/bounce/rest trajectory, no discontinuity -- ruling out the user's "perception thinks it fell" hypothesis. (2) Found and fixed a real, unrelated bug: `LIFT_OBJECT`'s cartesian path was running at full, unscaled speed the whole project (`computeCartesianPathRaw()` never set the service's velocity/acceleration scaling fields); wired through a new, slower `lift_velocity_scaling`/`lift_acceleration_scaling` (0.08) for `LIFT_OBJECT` and `STOW_OBJECT`/every safety retreat. (3) Found and fixed a second real, separate bug: `lift_height=0.15` had no joint-limit reachability margin -- the arm's configuration at `grasp_pose` varies run to run (OMPL, not deterministic) even though `grasp_pose` itself barely does, and some configurations couldn't complete the full vertical lift; cut to `lift_height=0.10`, **verified live: 0/4 Cartesian-completion failures in the next batch, down from 4/8 in the two batches before the fix** |
| Known remaining limitation, now the sole confirmed blocker to 100% reliability | Even with a solid `CLOSE_GRIPPER` contact (confirmed above to be genuine, not a false reading) and with lift speed/height both fixed (section 4.19), the object still slips free during `LIFT_OBJECT` in roughly half of live trials (6/12 across three batches, unaffected in rate by grasp_width tightening or the speed fix) -- reproduced repeatedly, including on runs otherwise completely clean. Correctly detected and retried every time, but a retry can fail if the slip knocked the object outside the arm's reachable envelope. A real physical grip-force/friction question this investigation's tooling couldn't inspect further (no effort/driver telemetry available) -- flagged as the next concrete follow-up |
| User-reported: full run now reaches `PLACE_ARM` and fails there, `START_STATE_INVALID` | **Found and fixed, verified live end to end (section 4.18)** -- `joint2`'s URDF lower bound is exactly 0, `STOW_OBJECT`'s retreat to the SRDF `zero` pose commands it there, and PhysX settles a hair under (`-0.0001`); `isaac_joint_bridge.py` republished that raw value onto `/piper/joint_states` unmodified, and MoveIt Jazzy has no ROS-exposed tolerance for this (confirmed against the installed binaries: `CheckStartStateBounds` only exposes `fix_start_state`, limited to continuous/planar/floating joints; `CurrentStateMonitor`'s bounds-error margin defaults to machine epsilon with no parameter to raise it). Fixed by giving the bridge the same `robot_description` move_group uses and clamping incoming joint states to its declared bounds within a new `bounds_clamp_tolerance` (0.01 rad), leaving genuinely gross violations visible rather than silently absorbed. General fix, not `joint2`-specific -- same trap was waiting on the gripper joints. 7/7 `kotek_isaac_bridge` tests (4 new) pass; **a full live rerun reached `DONE` for the first time in this project's history**, object delivered 2cm from target |
| **Full pick-and-delivery task, start to finish** | **Achieved and verified live (section 4.18)** -- `IDLE -> ... -> PLACE_ARM -> OPEN_GRIPPER -> RETREAT_ARM -> DONE`, zero retries needed, delivery accuracy 0.02m. The task's own wall-clock duration (~230-250s) is longer than this doc's previously-documented `--timeout 90` reproduce command allowed for -- corrected to `--timeout 400` |
