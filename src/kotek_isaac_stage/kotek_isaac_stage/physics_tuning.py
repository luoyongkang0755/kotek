#!/usr/bin/env python3
"""Single source of truth for the PhysX tuning that makes the grasp actually hold.

Imported by import_robots.py (robot asset), build_demo_stage.py (demo scene),
inspect_stage.py (regression assertions) and ~/Projects/kotek_sim (the non-ROS
implementation), so the ROS and non-ROS pipelines cannot drift apart.

WHY THIS FILE EXISTS
--------------------
docs/pick_and_delivery_report.md section 4.19 ran 12 live trials of the grasp,
found a 50% failure rate where the object is physically ejected from the
gripper, and concluded: "a genuine, unresolved physical grip/friction
question... no effort/driver telemetry available". The drive gains ARE that
missing telemetry, and they were never inspected. The numbers found there:

1. THE GRIPPER WAS A ~900 N VISE ON A 0.2 kg OBJECT.
   import_robots.py applied one constant set to every joint:
       JOINT_STIFFNESS = 1.0e5;  JOINT_MAX_FORCE = 1.0e3
   JOINT_MAX_FORCE was chosen for joint3's gravity-holding TORQUE in N*m
   (see its comment there), then blanket-applied to the PRISMATIC fingers,
   where the same number means NEWTONS. The URDF says effort="100".
   CLOSE_GRIPPER commands joint7=0.006; the logs quoted in manipulation.yaml
   show the fingers stall at 0.0088-0.0151, i.e. a 3-9 mm steady-state drive
   error against k=1e5 N/m:
       F = min(1e5 * 0.009, 1000) = ~900 N per finger
   and, because isaac_joint_bridge stops publishing once the trajectory ends
   and Isaac's articulation controller latches the last position target, that
   900 N is held continuously through LIFT_OBJECT, STOW_OBJECT and the whole
   delivery drive.

2. TWO INDEPENDENTLY-DRIVEN FINGERS. moveGripperToWidth() sets joint7=+w and
   joint8=-w as separate targets; PhysX solves them independently. A 5%
   asymmetry in convex-hull penetration at 900 N/side is ~45 N net lateral on
   a 0.2 kg body -> 225 m/s^2. That is the observed "jump".

3. NOTHING BOUNDED HOW PHYSX RESOLVED THE PENETRATION. /World/sensor carried
   UsdPhysics.RigidBodyAPI + CollisionAPI and nothing else -- verified against
   the binary stage, `strings` yields zero physx* tokens on that prim. So
   maxDepenetrationVelocity, solver iteration counts and sleep/stabilization
   thresholds were all at defaults, and contactOffset defaulted to 0.02 m,
   which is 40% of the object's own 5 cm width.

4. THE SCENE AND ARTICULATIONS WERE ENTIRELY UNTUNED. PhysxSceneAPI was
   applied with ZERO attributes, and at the time every runner used World()
   with no physics_dt -> 60 Hz, no CCD, no stabilization, and PhysX's default 4
   position / 1 velocity solver iterations. ~/Projects/hsr_sim -- the working
   reference implementation on this same host -- needed 32/8 at 120 Hz to
   stop its own articulation exploding (hsr_sim/scene.py:64-81 and
   hsr_urdf_model.py:186-194 document that trail). That fix was simply never
   applied here. NOTE (2026-09-10): even after the USD was tuned, the runners'
   bare World() calls kept re-introducing this state at runtime -- World()'s
   default set_defaults=True makes PhysicsContext.__init__ run
   set_physics_dt(1/60) AND enable_stabilization(False) on the open stage,
   overriding the tuned USD in memory (on-disk value stays 120; observed live
   as /clock at ~55-60 Hz). The wall-demo runners now pass
   set_defaults=False + physics_dt=1/EFFECTIVE_PHYSICS_RATE_HZ (60 on this
   host: 120 Hz is unreachable in real time here and freezes the
   magnet-damped boxes -- see that constant's note); README 7.4 has the full
   story.

5. THE FRICTION WAS NOT WHAT THE FILE SAID. The fingers had no physics
   material bound, so the sensor's authored staticFriction=1.0 was averaged
   against PhysX's default 0.5 under the default combine mode -> ~0.7.

Corroborating evidence already in the report, which only makes sense under
this diagnosis: section 4.19 Finding 2 -- tightening grasp_width 0.024->0.012,
i.e. INCREASING the squeeze from ~300 N to ~900 N, did not change the failure
rate (consistent with already saturating maxForce); Finding 3 -- cutting the
lift speed to 0.08 scaling did nothing (consistent with the ejection being
contact-solver-driven, not inertial).

RAW ATTRIBUTE NAMES, NOT pxr.PhysxSchema
----------------------------------------
Everything here is authored via raw USD attribute names plus
Usd.Prim.AddAppliedSchema(), NOT via pxr.PhysxSchema. PhysxSchema ships as a
Kit extension (extscache/omni.usd.schema.physx-*) and is NOT importable from
plain usd-core, which is the environment build_demo_stage.py and
inspect_stage.py are designed to run under (README section 4). Verified
directly: `from pxr import PhysxSchema` raises ImportError there, and forcing
it onto sys.path fails on libphysxSchema.so / libusd_tf.so / a Boost.Python
base-class-wrapper ordering error in turn.

This is already the established pattern in this codebase -- import_robots.py's
set_drive_gains() writes drive:<axis>:physics:<prop> as raw attributes for the
same class of reason (the importer's own API silently unit-converted them).
"""
from pxr import Sdf, UsdPhysics, UsdShade

# --- physics scene ------------------------------------------------------
# Mirrors hsr_sim/scene.py:55-87, the configuration proven to work on this
# host. Min*IterationCount is scene-wide and applies to NON-articulation
# bodies too -- that is the path by which it reaches the sensor object, and
# it is why hsr_sim sets it at the scene level rather than only per-body.
PHYSICS_TIME_STEPS_PER_SECOND = 120
SOLVER_POSITION_ITERATIONS = 32   # PhysX default is 4
SOLVER_VELOCITY_ITERATIONS = 8    # PhysX default is 1
SOLVER_TYPE = 'TGS'

# Effective step rate the RUNNERS configure (physics_dt=1/this), overriding the
# stage-authored 120 above. Measured on this host (2026-09-10, README 7.4):
#  * At 120 Hz the wall stage achieves only ~52 physics steps per wall-second
#    (sim runs at 0.43x real-time; ~17 ms of the per-frame budget is
#    render+graph overhead, physics itself is only ~1 ms/step), so the whole
#    ROS stack -- which runs on sim time -- would creep at half speed.
#  * Worse, the magnet graph's script-applied linear velocity-feedback damping
#    (MAGNET_DAMPING_COEFF through apply_forces_and_torques_at_pos) FREEZES the
#    sensor boxes solid at 120 Hz (Bug 9's Newton-freeze signature: pose
#    bit-identical, velocity readback pinned at F/c; bisected live -- killing
#    the damping unfreezes, restoring it refreezes). At 60 Hz the same 2.0
#    coefficient is freeze-free, exactly as historically calibrated.
#  * All three magnet probes PASS at 60 Hz + stabilization ON
#    (weld at tick 32 d=0.0088m; 30-deg arrival welds at 6.6 deg), identical
#    to the historical calibration runs.
# Keep the stage USD at 120 (solver/iteration tuning stays the single source
# of truth); runners pass physics_dt=1/EFFECTIVE_PHYSICS_RATE_HZ with
# set_defaults=False.
EFFECTIVE_PHYSICS_RATE_HZ = 60

# --- Piper arm drive (joint1..joint6, all REVOLUTE) ---------------------
# The asset authors stiffness=1e5 / damping=1e2 on these. Two things about
# that are wrong, and the second one is what actually knocks the object off
# its pedestal.
#
# 1. UNITS. UsdPhysics' "angular" DriveAPI is per-DEGREE, so an authored 1e5
#    is 1e5 * 57.2958 = 5.73e6 N*m/rad in the solver. Measured directly:
#    Articulation.get_dof_gains() -- which IS radian-consistent -- reports
#    stiffness=5.73e6 and damping=5730 for every arm joint against authored
#    values of 1e5 and 1e2. hsr_sim documents the same trap
#    (hsr_urdf_model._patch_drive_stiffness) and scales by 1/degrees(1) at
#    authoring time. The values below are left in authored (per-degree) units
#    because that is what set_drive_gains() writes; what matters is the RATIO,
#    which is unit-independent.
#
# 2. DAMPING RATIO. damping/stiffness = 0.001 here. hsr_sim's proven arm gains
#    sit at ~0.45 on every joint, and its robot_config records why: "a
#    persistent, NON-DECAYING steady-state velocity ... a genuine sustained
#    limit cycle from cross-joint coupling". kotek has exactly that. Tracing
#    the fingertip through a commanded straight 10 cm descent showed joint1
#    oscillating at ~3 Hz and swinging the gripper +/-6-7 cm LATERALLY -- far
#    enough to sweep the finger bodies into the object and knock it off its
#    pedestal before the gripper was ever commanded to close. The object
#    moving during the approach, not during the squeeze, is what
#    tests/test_grasp.py's [MOVE] line reports.
#
# ARM_DAMPING was chosen by measurement, not by copying hsr_sim's 0.45.
# probe_arm_damping.py sweeps the ratio and reports the peak lateral
# excursion away from the commanded straight-line descent:
#
#     ratio 0.001 (the asset)   max|y| = 0.0499 m   max lateral = 0.0503 m
#     ratio 0.01                max|y| = 0.0312 m   max lateral = 0.0312 m
#     ratio 0.1                 max|y| = 0.0001 m   max lateral = 0.0022 m
#     ratio 0.45                max|y| = 0.0013 m   max lateral = 0.0163 m
#     ratio 1.0                 max|y| = 0.0079 m   max lateral = 0.0444 m
#
# 0.1 is the optimum for this arm -- a 500x reduction in lateral excursion,
# and the best final tracking error of the sweep. hsr_sim's 0.45 is NOT best
# here, which is exactly why this was measured rather than inherited.
# Stated in RADIAN scale -- the units the runtime tensor API uses and the
# units these values were measured in (probe_arm_damping.py drives the
# articulation through Articulation.set_dof_gains, which is radian-consistent).
ARM_STIFFNESS = 1.0e5              # N*m/rad
ARM_DAMPING = 1.0e4                # N*m*s/rad  (was the equivalent of 5730)
ARM_DAMPING_RATIO = ARM_DAMPING / ARM_STIFFNESS   # 0.1, the measured optimum

# UsdPhysics' "angular" DriveAPI is per-DEGREE, so anything written to
# drive:angular:physics:{stiffness,damping} must be scaled DOWN by
# degrees-per-radian to land on the radian-scale values above. Verified
# directly against this asset: authoring 1e5 produced a reported (radian)
# stiffness of 5729578 = 1e5 * 57.2958, and 1e2 produced 5729.578.
#
# Getting this wrong is not cosmetic -- it is a 57x error in the gain that was
# actually validated. The measured sweep in probe_arm_damping.py was run in
# radian scale, so the asset has to author the SCALED values to reproduce it.
# hsr_sim documents the same trap and applies the same 1/degrees(1) factor at
# authoring time (hsr_urdf_model._patch_drive_stiffness).
#
# maxForce is a torque, not a per-angle gain, and is NOT converted.
DEGREES_PER_RADIAN = 57.29577951308232
ARM_STIFFNESS_AUTHORED = ARM_STIFFNESS / DEGREES_PER_RADIAN   # ~1745.3
ARM_DAMPING_AUTHORED = ARM_DAMPING / DEGREES_PER_RADIAN       # ~174.5
# Unchanged: sized for joint3's genuine gravity-holding torque insufficiency
# (see import_robots.py's JOINT_MAX_FORCE comment). This is a real N*m torque
# limit for a revolute joint and is not the bug -- the bug was reusing it for
# the prismatic fingers, where it means newtons.
ARM_MAX_FORCE = 1.0e3

# --- Teleop-only per-joint effort caps (kotek_teleop demo) ---------------
# ARM_MAX_FORCE above (1e3 N*m, ~10x the URDF's own effort=100 per joint) is
# deliberately generous for the autonomous pick demo, where joint3 needs
# extra authority just to hold the arm's own weight against gravity in a
# reaching pose. It is wrong for teleop: with that much authority, the
# simulated arm does not stall against an obstacle, it simply overpowers it
# with near-unbounded force, which makes any contact-force -> leader-torque
# mapping physically meaningless (found during kotek_teleop implementation,
# before any torque ever reached the real leader arm).
#
# These values are NOT taken from an AgileX Piper datasheet -- no exact
# per-joint torque spec was available at authoring time. They are chosen to
# be plausible for a small collaborative arm (heavier authority on the
# shoulder/elbow joints that carry the most static load, lighter on the
# wrist), and exist so contact against a static obstacle produces a visible
# stall rather than penetration. Tune them during kotek_teleop's Phase 2
# verification (tier3_contact_test.py / driving the sim arm into
# obstacle_pickup_leg by hand) if the stall behavior looks wrong; they are
# applied at RUNTIME by teleop_runner.py via Articulation.set_dof_max_efforts,
# never authored into the USD, so changing them needs no stage rebuild and
# does not affect the autonomous demo's ARM_MAX_FORCE at all.
TELEOP_ARM_MAX_EFFORTS = (50.0, 50.0, 40.0, 15.0, 15.0, 15.0)  # N*m, joint1..joint6
# TELEOP_GRIPPER_MAX_EFFORT is defined below, right after GRIPPER_MAX_FORCE
# (which it equals) -- see that constant's comment.

# --- Piper gripper drive (joint7 / joint8, both PRISMATIC) ---------------
# Sized from first principles rather than inherited from the arm:
#   holding SENSOR_MASS at mu = SENSOR_DYNAMIC_FRICTION needs, per finger,
#       F = m*g / (2*mu) = 0.2*9.81 / (2*0.9) = 1.09 N
#   GRIPPER_MAX_FORCE = 20 N is ~18x that margin, and matches the gripper
#   max_force hsr_sim proved out (robot_config.URDF_DRIVE_GAINS).
#   GRIPPER_STIFFNESS = 1e3 N/m over the observed ~9 mm stall error gives
#   ~9 N -- comfortably INSIDE maxForce, so the squeeze is set by the grasp
#   geometry rather than by clipping at the force limit. That distinction is
#   what makes grasp_width a meaningful tuning knob again; at 1e5 N/m it was
#   not, which is exactly why report section 4.19 Finding 2 saw no effect
#   from changing it.
#   GRIPPER_DAMPING = 50 N*s/m is near-critical for the 0.0265 kg finger at
#   this stiffness (2*sqrt(k*m) = 2*sqrt(1e3*0.0265) = 10.3), i.e.
#   deliberately overdamped, so the fingers never overshoot into the object.
GRIPPER_STIFFNESS = 1.0e3          # N/m    (was 1.0e5)
GRIPPER_DAMPING = 5.0e1            # N*s/m  (was 1.0e2)
GRIPPER_MAX_FORCE = 2.0e1          # N      (was 1.0e3; URDF effort=100)
# Already sized from first principles for this scene's SENSOR_MASS /
# SENSOR_DYNAMIC_FRICTION above -- no separate teleop override needed.
TELEOP_GRIPPER_MAX_EFFORT = GRIPPER_MAX_FORCE
# The asset carried 57.29578, which is degrees(1 rad) -- the importer's
# radian->degree conversion leaking onto a PRISMATIC joint, where it means
# metres per second and is effectively unbounded finger speed. The URDF says
# velocity="1" (m/s); 0.2 m/s is used instead, which still closes the 40 mm
# of finger travel in 0.2 s and keeps the approach speed well inside what the
# contact solver resolves cleanly. Measured to be non-critical once
# SENSOR_MAX_DEPENETRATION_VELOCITY is right (see its table: 0.05 and 0.2 m/s
# behave identically at depenV >= 0.5), so this is chosen for physical
# plausibility rather than to paper over a solver limit.
GRIPPER_MAX_JOINT_VELOCITY = 0.2   # m/s    (was 57.29578)

# --- the graspable "sensor" object --------------------------------------
# SENSOR_SIZE is authored with UsdGeom.Cube.CreateSizeAttr and NO scale op.
# The previous asset used a size-1.0 cube with a NON-UNIFORM
# AddScaleOp(0.05, 0.05, 0.08): a scaled analytic collider, gripped on its
# narrow face with its long axis vertical, which maximises the tipping
# moment about any off-centre contact. hsr_sim deliberately sizes its own
# object via CreateSizeAttr and uses AddScaleOp only on static tables
# (hsr_sim/scene.py:128-150) -- that distinction is followed here.
SENSOR_SIZE = 0.05                 # m, true cube
SENSOR_MASS = 0.2                  # kg
SENSOR_STATIC_FRICTION = 1.0
SENSOR_DYNAMIC_FRICTION = 0.9
SENSOR_RESTITUTION = 0.0
# Was PhysX's 0.02 m default -- 40% of the object's own width, so the fingers
# began generating contact forces while still 2 cm clear of it.
SENSOR_CONTACT_OFFSET = 0.002      # m
SENSOR_REST_OFFSET = 0.0           # m
# The static wall collider (/World/wall/geom) was left at PhysX's 0.02 m
# default contactOffset -- verified un-authored via usd-core ("NOT AUTHORED
# (PhysX default contactOffset=0.02!)"; sensor_cam_1 by contrast carries an
# explicit 0.002). A static body's contactOffset still controls WHERE contact
# forces begin: with the boxes' own 0.002, the pair began interacting 2.2 cm
# off the wall face -- an invisible "pillow" 2 cm thick. Measured consequences
# (2026-09-07, dipole-model probe runs): boxes froze in dynamic equilibrium ON
# the pillow (d = 7.5-9.6 mm, speeds and angular speeds pinned at nonzero
# values, quat constant bit-for-bit for thousands of ticks, never reaching the
# weld gate), and every legacy E2E "weld" sat 1-4 cm off the wall -- the weld
# gate of 0.04 fired while the box hovered on the pillow lip. Same fix
# rationale as SENSOR_CONTACT_OFFSET above; restOffset stays 0 so faces truly
# touch at contact.
WALL_CONTACT_OFFSET = 0.002        # m
WALL_REST_OFFSET = 0.0             # m
# Caps how fast PhysX may push overlapping bodies apart, so a solver-resolved
# penetration cannot become a metres-per-second ejection.
#
# This value is a genuine trade-off in BOTH directions and was set by
# measurement, not by picking a conservative-looking number. The same cap that
# limits an ejection also limits how fast the solver may resolve ordinary
# contact: set it too low and the fingers plunge into the object faster than
# PhysX is allowed to push back, so the contact force available to hold it is
# throttled to almost nothing. probe_grasp_tuning.py sweeps it against the
# finger closing speed, closing on a 51.4 mm object and then lifting:
#
#   depenV  closeV  stall width  squeeze   rise      slip     held
#   0.05    0.05    0.0291 m     8.5 N     -0.0000   0.1045   no
#   0.05    0.20    0.0296 m     8.8 N     +0.0000   0.1028   no
#   0.50    0.05    0.0519 m    20.0 N     +0.1071   0.0049   YES
#   0.50    0.20    0.0518 m    19.9 N     +0.1051   0.0028   YES
#   5.00    0.05    0.0519 m    20.0 N     +0.1071   0.0049   YES
#   5.00    0.20    0.0519 m    20.0 N     +0.1051   0.0030   YES
#
# At 0.05 the fingers close to 29 mm on a 51 mm object and the object never
# moves at all -- the signature of contact being resolved far too weakly, not
# of a slip. At 0.5 the stall width lands exactly on the object's real extent
# (0.0519 vs 0.0514 measured), which is what a correctly-resolved contact
# looks like. 0.5 is chosen over 5.0 as the smallest value that works, i.e.
# the tightest remaining cap against ejection.
SENSOR_MAX_DEPENETRATION_VELOCITY = 0.5    # m/s
SENSOR_MAX_LINEAR_VELOCITY = 2.0           # m/s
SENSOR_MAX_ANGULAR_VELOCITY = 10.0         # rad/s

# --- Piper finger contact surfaces --------------------------------------
# Bound so the pair-combine is 0.9-against-0.9 instead of 0.9-against-PhysX's
# default 0.5. frictionCombineMode=min is PhysX's own default; 'multiply'
# would give 0.81 and 'average' 0.9 -- 'max' is chosen so the authored value
# is the one that actually applies regardless of what the other shape carries.
FINGER_STATIC_FRICTION = 1.0
FINGER_DYNAMIC_FRICTION = 0.9
FINGER_RESTITUTION = 0.0
FRICTION_COMBINE_MODE = 'max'
# A convex hull of the fork-shaped link7.STL / link8.STL is a solid wedge, so
# contact happens on a surface that does not exist in the visual mesh.
FINGER_COLLISION_APPROXIMATION = 'convexDecomposition'
FINGER_COLLISION_PRIMS = (
    '/World/piper/Geometry/base_link/link1/link2/link3/link4/link5/link6'
    '/link7/link7_1/link7_1',
    '/World/piper/Geometry/base_link/link1/link2/link3/link4/link5/link6'
    '/link8/link8_1/link8_1',
)

PHYSICS_SCENE_PATH = '/World/physicsScene'
SENSOR_PRIM_PATH = '/World/sensor'

# --- wall-mount task: the 4 camera-sensor objects ------------------------
# A true 8x3.5x4 cm box (not a scaled analytic cube on a demo cube's size=1
# convention -- these ARE that shape, so CreateSizeAttr(1.0) + a non-uniform
# AddScaleOp is the correct authoring here, the one case in this codebase
# where that pattern is right rather than the bug SENSOR_SIZE's comment
# documents). x=long edge (8cm), y=short edge (3.5cm), z=height (4cm).
SENSOR_CAM_SIZE_XYZ = (0.08, 0.035, 0.04)   # m
# Not measured against a real sensor -- a plausible mass for this volume
# (112 cm^3) at a density a bit above water, matching how SENSOR_MASS/
# SENSOR_SIZE were originally picked before being tuned against live
# physics. Adjust via probe_magnet_tuning.py if the grasp/weld numbers
# below don't hold up under a real run.
SENSOR_CAM_MASS = 0.05                      # kg
SENSOR_CAM_STATIC_FRICTION = 1.0
SENSOR_CAM_DYNAMIC_FRICTION = 0.9
SENSOR_CAM_RESTITUTION = 0.0
# Same reasoning as SENSOR_CONTACT_OFFSET: a fraction of the object's own
# smallest dimension (4 cm here), not PhysX's 0.02 m default (half the
# object's own height).
SENSOR_CAM_CONTACT_OFFSET = 0.002           # m
SENSOR_CAM_REST_OFFSET = 0.0                # m
# NOT the generic SENSOR_MAX_LINEAR_VELOCITY (2.0 m/s, tuned for the
# original pick-place object, which is never expected to move fast in a
# narrow force field). Found via a live probe_magnet_tuning.py failure: at
# 2.0 m/s, a sensor-cam object can cross the entire 3cm MAGNET_ATTRACT_RANGE
# in ~0.015s (~2 physics ticks at 120Hz) -- not enough time for
# MAGNET_DAMPING_COEFF to meaningfully arrest the object before it either
# overshoots past the target and exits the attract zone on the far side, or
# reaches WELD_DISTANCE already far above WELD_MAX_SPEED. Capped low enough
# that crossing the same 3cm zone takes ~0.1s (~12 ticks) -- comfortable
# room for damping to act -- while still fast enough not to look sluggish
# during normal arm-carried handling (which this cap also governs).
SENSOR_CAM_MAX_LINEAR_VELOCITY = 0.3        # m/s

# apply_graspable_rigid_body_tuning() also sets sleepThreshold=0.0 and
# stabilizationThreshold=0.0 on every graspable object (needed so a resting
# object responds immediately to grasp forces instead of first "waking up"
# from PhysX's sleep state) -- but that also means it NEVER stops
# integrating, even at near-zero residual velocity, so it never settles to a
# hard rest the way a normally-sleeping body would. The original pedestal
# demo cube never surfaced this: it sits directly on a static pedestal for
# only the few seconds between spawn and grasp. The wall-mount task's
# sensor-cams rest loose on the chassis-mounted riser -- itself subject to
# continuous small suspension/solver noise from a live articulation -- for
# however long Isaac Sim has been playing before the task runs, sometimes
# minutes. A live wall-mount E2E run's own magnet-graph position telemetry
# (which logs each sensor's pose every 60 ticks regardless of task state)
# caught this directly: sensor_cam_1 sat within ~1mm of its spawn position
# at t=0.5s, but had drifted ~3.6cm by t=~165s -- a slow but relentless
# random-walk creep, not a one-time settle -- and a live grasp attempt at
# that drifted position closed on empty air (no linear/angular damping was
# set anywhere in apply_graspable_rigid_body_tuning(), so PhysX's own
# defaults applied: near-zero, nowhere near enough to bleed off a
# persistent low-amplitude disturbance over that many ticks). Added as new,
# task-specific overrides (default 0.0, byte-for-byte preserving every
# existing caller's behavior -- SENSOR_CAM_STATIC_FRICTION etc. above are
# already quite high and did not stop the drift on their own, since
# friction is a reactive threshold force, not a continuous velocity-bleed
# term the way damping is) rather than changed as defaults, so the original
# pedestal demo's tuned dynamics are untouched.
SENSOR_CAM_LINEAR_DAMPING = 5.0
SENSOR_CAM_ANGULAR_DAMPING = 5.0

# HISTORY (2026-08-25): a "riser-hold" subsystem used to live here -- a
# breakable FixedJoint per sensor-cam (box <-> base_link,
# SENSOR_CAM_RISER_HOLD_BREAK_FORCE/TORQUE) plus a time-gated release
# ScriptNode (author_riser_release_graph.py, RISER_HOLD_RELEASE_TIME) --
# added to pin the boxes through a measured "settle offset" of up to 4.3cm.
# The whole subsystem is REMOVED, for two independently-verified reasons:
#
# 1. It could never release. PhysX joints anchored to an ARTICULATION link
#    (base_link is one) ignore breakForce entirely AND every runtime release
#    path -- physics:jointEnabled=False and even stage.RemovePrim were both
#    confirmed inert on a live sim (probe_release_live.py: a 5N pull, 5x the
#    authored break threshold, moved the box 0.0mm in all three states; the
#    identical world-anchored joint breaks instantly). The release graph's
#    "released 4/4" print only ever edited USD; the running sim never saw
#    it. THIS was the real cause of the "LIFT_OBJECT slip": the grasp itself
#    is strong (a live probe measured ~10N of squeeze holding a 43N pull),
#    but the object stayed bolted to the chassis, so the position-driven
#    lift pried the fingers off an immovable box every single time.
#
# 2. It is not needed. The original 4.3cm "settle offset" was measured in
#    WORLD coordinates and is chassis roll/tilt/creep, which the boxes (and
#    the arm) all share; the box-vs-chassis RELATIVE drift with no joints at
#    all is 0.1-0.2mm across the full settle transient and beyond
#    (probe_drift_rel_no_joints.py, 120s) -- the SENSOR_CAM_*_DAMPING and
#    friction tuning above is sufficient on its own. Grasp targets live in
#    the arm frame, which rides the same chassis, so only relative drift
#    ever mattered.
#
# inspect_stage.py asserts the joints and the release graph are ABSENT, so a
# stale stage build or a re-added articulation-anchored hold fails fast.

# --- wall-mount task: simulated magnetism ---------------------------------
# Magnets exist ONLY on the sensor's bottom (8x3.5cm) face and on the wall's
# 4 target patches -- nothing else in the scene is magnetic. The magnet
# graph (author_magnet_graph.py) enforces this by iterating a fixed list of
# (sensor_prim, wall_target_pose) pairs, not a general "any two magnetic
# surfaces attract" system.
#
# MODEL (2026-09-07): magnetic dipole-dipole interaction, replacing the
# original point-attraction law F = clamp(GAIN/d**2, 0, MAX). Each magnet is
# a dipole moment of magnitude MAGNET_DIPOLE_MOMENT: the wall patch's moment
# points along WALL_INWARD_DIR, the sensor's along its bottom-face outward
# normal (local -Z), so the two moments are PARALLEL when the sensor is
# correctly mounted and the interaction is then purely attractive. Per tick,
# per (sensor, target) pair, with r the dipole-to-dipole vector (wall patch
# -> sensor bottom face), K = 1e-7 * MAGNET_DIPOLE_MOMENT**2:
#     B_w = 1e-7 / r^3 * [3 (m_w . r_hat) r_hat - m_w]
#     F   = 3e-7 / r^4 * [(m_w . r_hat) m_s + (m_s . r_hat) m_w
#                         + (m_w . m_s) r_hat
#                         - 5 (m_w . r_hat) (m_s . r_hat) r_hat]
#     tau = m_s x B_w
# Two behaviors FALL OUT of the formula instead of being hand-tuned -- this
# is the entire point of the rewrite:
#   1. ALIGNMENT: near the mount axis B_w points along +x, so tau = m_s x B_w
#      rotates the sensor's bottom-face normal onto the wall direction and
#      holds it there. The old model's separate torqueGain alignment knob
#      (always disabled -- see the Bug-6-era history below and
#      author_magnet_graph.py) is gone. A backwards-mounted sensor (moments
#      anti-parallel) is REPELLED -- real dipole physics, not a special case.
#   2. LATERAL CAPTURE: off-axis, B_w tilts toward the mount axis, so tau
#      also swings the sensor's pull direction toward its own target point,
#      not just toward the wall plane.
# Three supporting effects, unchanged from the old model and still driven
# by the constants below:
#   1. ATTRACT: the dipole F/tau above are gated on d < MAGNET_ATTRACT_RANGE
#      and tapered to zero over the range (SAME factor on both, preserving
#      their physical ratio -- the dipole law is singular at r=0 and is not
#      a valid near-field model anyway).
#   2. DAMP + GRAVITY FEEDFORWARD: while in range, subtract
#      MAGNET_DAMPING_COEFF * velocity (Bug 2/4 -- a constant attractive
#      force with no velocity feedback is bang-bang and cannot converge)
#      and add a constant world-+Z force equal to the object's weight
#      (Bug 3 -- the radial interaction must never have to "accidentally"
#      cover gravity). Bug 9 (2026-09-08) retired ONLY the ANGULAR
#      damping twin of this term: it is the sole Newton freeze trigger;
#      the linear term below is freeze-free (live bisect) and keeps its
#      load-bearing role.
#   3. WELD: once d < WELD_DISTANCE and the linear and angular speeds are
#      below WELD_MAX_SPEED / WELD_MAX_ANG_SPEED, author a breakable
#      UsdPhysics.FixedJoint (one-shot,
#      not re-authored every tick) with CreateBreakForceAttr/
#      CreateBreakTorqueAttr -- holds indefinitely under normal handling,
#      detaches under a strong enough pull.
#
# MEASURED against live physics via probe_magnet_tuning.py --mode in-range
# (kotek_scout_piper_wall_demo.usd, real physics runs, not static authoring
# checks) -- TWO real bugs found and fixed here, in sequence, not guessed.
# Bug 1 was specific to the old force law's shape and is kept below as
# history; Bugs 2-8 are about the damping/taper/weld-gate stabilizers that
# the dipole model still needs and still uses:
#
# Bug 1 (shape, HISTORICAL -- superseded by the dipole model above): the
# first point-attraction formula sized its gain as
# MAX_FORCE * WELD_DISTANCE**2 (reaching the force cap only right at
# WELD_DISTANCE, ramping smoothly the rest of the way), giving F(d) =
# k/d**2 only ~0.03-0.05 N at d close to MAGNET_ATTRACT_RANGE -- against
# SENSOR_CAM_MASS's own weight of 0.05*9.81 = 0.4905 N, roughly 10x weaker
# than gravity at the distance where "attract" is supposed to start
# working. The object free-fell straight to the ground (z: 0.36 -> 0.04 m)
# instead of being pulled in. The fix then was GAIN = MAX_FORCE *
# MAGNET_ATTRACT_RANGE**2 (flat, constant pull across the whole range).
# Both the constant and the law it sized are GONE with the dipole rewrite;
# MAGNET_DIPOLE_MOMENT below is calibrated to reproduce the same 2 N scale
# at the attract-range edge, so the measured lesson carries over: the
# interaction must already be at full strength at the OUTER edge of the
# range, not bite only in the final millimetres.
#
# Bug 2 (no damping): with bug 1 fixed and MAGNET_MAX_FORCE=3.0 N (a
# roughly constant 60 m/s^2 = ~6g on the 0.05 kg object), the sensor DID get
# pulled in fast -- d=0.0058m by tick 15 (0.125s) -- but with nothing
# opposing its velocity it shot straight through the target, oscillated
# (speed spiking to 1.8+ m/s), eventually overshot past
# MAGNET_ATTRACT_RANGE entirely, and fell to the ground uncontested. A
# constant-magnitude attractive force with no velocity feedback is a
# bang-bang controller -- it has no mechanism to converge, only to
# overshoot repeatedly, so no value of MAGNET_MAX_FORCE alone fixes this.
# Fixed by adding MAGNET_DAMPING_COEFF as a linear drag term active
# whenever d < MAGNET_ATTRACT_RANGE: F_total = F_attract - c*v.
# MAGNET_DAMPING_COEFF is sized well under the explicit-Euler stability
# bound for this mass/timestep (c < 2*m/dt = 2*0.05*120 = 12 N*s/m at
# 120Hz) -- a damping term that stiff for the timestep can itself inject
# energy instead of removing it, so headroom under that bound is
# deliberate, not just "as strong as possible."
#
# Bug 3 (angle-dependent margin, caught before it shipped): an intermediate
# version dropped MAGNET_MAX_FORCE 3.0 -> 1.5 N alongside adding damping,
# reasoning that 1.5 N (~3x weight) was still comfortable margin per bug
# 1's analysis -- but that analysis implicitly assumed the force points
# straight at the target. At this probe's actual approach angle, the pull
# direction was only ~32% vertical (direction_z=0.323), so the vertical
# component of a 1.5 N cap is 1.5*0.323 = 0.485 N against 0.4905 N of
# weight -- net vertical force ~0, and the object sank at nearly free-fall
# rate. Raising the cap to compensate would only fix THIS probe's angle;
# the real 4 pickup/mount geometries (from the /piper/compute_ik sweep,
# still pending) will have their own angles, so a cap tuned to one angle
# doesn't generalize. Fixed by decoupling gravity compensation from the
# radial attraction force entirely: MAGNET_GRAVITY_FEEDFORWARD_Z adds a
# constant world-+Z force equal to the object's own weight whenever the
# magnet is active (same d < MAGNET_ATTRACT_RANGE gate), so the radial
# k/d**2 term only has to do horizontal capture work and never has to
# "accidentally" cover gravity by being aimed steeply enough -- MAGNET_MAX_FORCE
# can then be sized for radial capture strength alone, independent of
# approach angle.
#
# Bug 4 (velocity cap + bang-bang chatter near contact): with bugs 1-3
# fixed, a live probe showed the object genuinely CAPTURED -- held in a
# tight, stable orbit at d=0.002-0.003m from target for the full 1.5s
# window, no longer falling or escaping -- but never welding. Its speed sat
# pinned at 0.2978-0.3000 m/s (never decaying) for 300+ consecutive ticks.
# Two compounding causes, found in order:
#   (a) SENSOR_CAM objects inherited the generic SENSOR_MAX_LINEAR_VELOCITY
#       (2.0 m/s, tuned for the ORIGINAL pick-place object, which never
#       operates inside a 3cm force field). At 2.0 m/s an object crosses the
#       entire MAGNET_ATTRACT_RANGE in ~2 ticks at 120Hz -- far too fast for
#       MAGNET_DAMPING_COEFF to arrest before overshoot/escape (this was the
#       proximate cause of bug 2's fall-through). Fixed with a dedicated,
#       much lower SENSOR_CAM_MAX_LINEAR_VELOCITY = 0.3 m/s.
#   (b) Even after (a), speed stayed pinned -- now at the NEW 0.3 m/s cap --
#       because the radial force never actually weakens close to the
#       target: under the old inverse-square law any d small enough to
#       matter had GAIN/d**2 >> MAGNET_MAX_FORCE,
#       so the force was fully saturated (bang-bang) for the whole final
#       approach. (The dipole law's 1/r**4 is even steeper, so the taper
#       is just as load-bearing for it -- only now it tapers F and tau
#       together, preserving their physical ratio.) At 120Hz that saturated
#       force re-accelerates the object
#       back toward the velocity cap every tick, faster than damping can
#       remove it -- raising MAGNET_DAMPING_COEFF 6.0 -> 10.0 made NO
#       difference (identical stuck-at-cap result both times), confirming
#       the force was the limiter, not damping strength. Fixed by adding
#       MAGNET_TAPER_DISTANCE: the RADIAL force (only -- NOT the damping or
#       gravity feedforward terms) is scaled by
#       clamp((d - WELD_DISTANCE) / (MAGNET_TAPER_DISTANCE - WELD_DISTANCE), 0, 1),
#       i.e. full strength at MAGNET_TAPER_DISTANCE, tapering linearly to
#       EXACTLY ZERO at WELD_DISTANCE. Sized to 3x WELD_DISTANCE so the
#       taper zone comfortably covers the observed 0.002-0.003m oscillation
#       band -- the object now coasts the last few mm on damping and gravity
#       feedforward alone, with nothing re-accelerating it into another
#       orbit.
# Bug 7 (damping too strong, not too weak): with Bug 6's implied-torque fix
# in place (radial force at COM, alignment torque off), speed STILL stayed
# pinned at the linear velocity cap indefinitely with MAGNET_DAMPING_COEFF
# =10.0 -- but this time with ang_speed confirmed genuinely 0.0000 throughout
# (vector print, not just norm), ruling rotation out completely. Printing
# the actual applied com_force vector pointed at the real cause: at c=10.0,
# the damping force computed from an ALREADY-CAPPED velocity (0.3 m/s ->
# 3.0 N) was large enough, relative to this object's tiny mass (0.05 kg)
# and the 120Hz timestep, to overshoot rather than settle -- a classic
# explicit-Euler over-damping instability, not the earlier stability-bound
# arithmetic's "c < 2m/dt = 12" per-force-alone bound accounting for the
# OTHER forces (radial + gravity feedforward) also acting the same tick.
# Cutting MAGNET_DAMPING_COEFF 10.0 -> 2.0 (below, no other change) fixed it
# completely: speed decayed smoothly to exactly 0.0000 by tick 45 (0.30 ->
# 0.12 -> 0.08 -> 0.04 -> ... -> 0.0000), a genuine damped settle, not
# another lock. It settled at d=0.0063m with com_force stabilizing to a
# small residual (~0.245N) rather than continuing in to WELD_DISTANCE's
# original 0.003m -- a real, stable, zero-velocity fixed point, just
# outside the old tolerance. WELD_DISTANCE widened to 0.01m (below) to
# comfortably include this converged state -- relaxing an engineering
# tolerance to match genuinely-converged physics, not compensating for a
# bug.
# Bug 8 (WELD_DISTANCE was secretly doing double duty): a second run at
# WELD_DISTANCE=0.01 settled at a DIFFERENT equilibrium (d=0.0125m, not
# 0.0063m) -- not solver noise. The radial-force taper (author_magnet_graph.
# py's compute()) tapers to zero exactly AT weld_distance, by formula, so
# WELD_DISTANCE was controlling both "how close counts as welded" AND the
# force law's own zero-crossing point at the same time. Moving one moved
# the other's equilibrium along with it -- the 0.0063m settle point at
# WELD_DISTANCE=0.003 stopped being reachable once that same value became
# 0.01 (0.0063 now sits INSIDE the force-dead zone), so the system found a
# new equilibrium further out; widening to 0.02 pushed the dead zone far
# enough to straddle the new equilibrium and produced an oscillation
# instead of a settle. Fixed by decoupling: MAGNET_TAPER_FLOOR (below) is
# now a fixed, independent constant the taper formula tapers to zero
# against -- reusing the original 0.003 value, i.e. the exact configuration
# already proven to converge cleanly to speed=0.0000 at d=0.0063m. WELD_
# DISTANCE goes back to being a pure gate with no effect on the force law's
# shape, and is kept at 0.01 (already comfortable margin above 0.0063).
# 0.003 -> 0.02 -> back to 0.003 (2026-09-07, dipole model, both moves
# measured): raising the floor to 0.02 kept the box OUT of the torsionally
# stiff zone but created a worse problem -- with the force dead at 2cm the
# box dwells for seconds right at the band edge (d oscillating 1.7-2.1cm,
# speed 0.08-0.3 m/s, never coasting to rest) and the clamped torque
# operating on the r-hat feedback through the moving dipole point sustained
# a cap-pinned spin (ang_speed pinned at exactly SENSOR_MAX_ANGULAR_
# VELOCITY = 10 rad/s) that no admissible damping could remove. The FORCE
# law therefore keeps the old model's proven shape (full strength at the
# range edge, zero-crossing at 3mm -- Bug 1/5/7/8 territory, settled at
# d~6-12mm and pulled into contact). The near-field torque problem is
# handled separately by MAGNET_TORQUE_CUTOFF below, not by moving the
# force's zero-crossing.
MAGNET_TAPER_FLOOR = 0.003                   # m, fixed -- NOT the same knob as
                                              # WELD_DISTANCE (see Bug 8 above)
# Torque-only near-field cutoff (2026-09-07): the alignment torque tapers
# to zero by THIS distance (linearly, from MAGNET_ATTRACT_RANGE down),
# independently of the force taper. Measured reason: the point-dipole
# torsional stiffness grows as 1/r**3 -- inside ~3cm the discrete 120Hz
# integration of the tau = m_s x B_w feedback loop (stiffness fed back
# through r_hat, which the rotating dipole point keeps changing) goes
# unstable and pins ang_speed at the velocity cap regardless of the torque
# clamp value (0.02 and 0.002 N*m both measured to fail). Above 3cm the
# stiffness is integration-safe and the clamped torque still has real
# alignment authority. The final flattening against the wall is done by
# CONTACT (the force law below still pulls the box into the wall face;
# 0.9-friction contact at a ~2N press out-torques the 0.002 N*m clamp by
# an order of magnitude -- which is also how a real fridge magnet aligns).
MAGNET_TORQUE_CUTOFF = 0.01                  # m, torque tapers to zero by here
# 0.03 -> 0.01 (2026-09-08): the "inside 3cm the 120Hz tau feedback is
# unstable at any clamp" evidence that set 0.03 was CONTAMINATED -- every
# run that showed the cap-pinned spin also had Bug 9's script damping
# active, which freezes the body and pins |w| at the velocity cap
# regardless of the torque law. With damping disabled (Bug 9 fix) the
# freeze is gone, and re-measurement showed the OPPOSITE failure: with
# torque dead below 3cm, a box spun up by the 3-5cm band torque tumbles
# FREELY through the final 3cm (nothing opposes rotation there -- the
# body-authored angular damping is far too weak vs the accumulated
# spin) and arrives at the wall at up to 90 deg misalignment, where the
# weld gate -- legitimately passed at rest -- freezes it crooked. The
# alignment torque must therefore stay live almost to contact; 1cm
# leaves the last centimetre to wall-contact friction, which is also
# how a real fridge magnet seats. The clamp (MAGNET_MAX_TORQUE) bounds
# the near-field 1/r**3 stiffness, so the discrete-time loop sees a
# BOUNDED torque -- stability is re-verified by the misaligned probe,
# not assumed.
# 0.01 -> 0.028 (2026-08-25): with the release point moved 4.5cm short of
# the wall (see wall_mount_task.py's _WALL_X comment) the box arrives
# carrying the grasp's yaw and place_pitch=0.7 tilt, touches the wall EDGE
# first, and contact friction locks it there (the alignment torque is
# deliberately disabled -- see author_magnet_graph.py's torqueGain comment --
# and cannot flatten it): a live 4-cycle run left two boxes magnet-held
# against the wall, stable and at rest, with face centers 2-3cm from their
# targets and therefore never welded under the old 0.01 gate. Bug 8 already
# decoupled this gate from the force law, so widening it only changes what
# counts as "arrived": an edge-resting, slightly crooked box now welds where
# it stands (a crooked fridge magnet, visually), instead of hanging on
# attraction forever. Flattening the arrival (re-enabling a damped
# alignment torque) stays the polish item it already was.
# ...and 0.028 -> 0.04 after the next full run: all 4 boxes were delivered
# and hung stably magnet-held at the wall, but face-center distances settled
# at ~3.0-3.5cm (edge-resting, tilted -- see above), still outside 0.028;
# only sensor 1 welded. 0.04 covers every measured stable arrival. Paired
# with WELD_MAX_SPEED dropping 0.05 -> 0.01: a gate this wide is now crossed
# DURING the gripped place descent, and welding a box still held by the
# gripper would start an arm-vs-weld tug-of-war that snaps the weld
# (rigid-rigid break DOES work) and, since state.welded[i] latches, never
# re-welds -- the near-zero speed threshold restricts welding to boxes
# genuinely at rest, which the gripped carry never is and the released,
# magnet-settled hover always is.
WELD_DISTANCE = 0.04                         # m, contact tolerance for the weld
# Bug 5 (cap locks at whatever value it's set to, not "0.3 specifically"):
# a live A/B test with SENSOR_CAM_MAX_LINEAR_VELOCITY temporarily patched to
# 5.0 m/s (effectively off) showed damping DOES converge speed to ~0 cleanly
# on its own (0.0006-0.0014 m/s by tick 30) -- ruling out the force/damping
# math. But uncapped, the object blew past the ENTIRE attract range within
# ~15 ticks (full MAGNET_MAX_FORCE the whole way, since the taper only
# covered the last 3xWELD_DISTANCE=9mm) and free-fell, same as bug 2. A
# second test at an intermediate cap (1.0 m/s) reproduced the SAME
# stuck-at-cap lock as 0.3 m/s did -- speed pinned at ~1.0 for 150+ ticks,
# not decaying -- proving the lock tracks whatever the cap IS, not a
# resonance at 0.3 specifically. Isaac/PhysX's maxLinearVelocity behaves
# like a hard renormalization under continuous active force here, not a
# soft ceiling that only trims excess -- so no cap value alone fixes this,
# only keeping the object's OWN (undamped) speed well below whatever cap is
# chosen does. Fixed by widening the taper to span the WHOLE attract range
# (was 3x WELD_DISTANCE = 9mm) instead of only the last few mm, so the
# radial force starts weakening from the outer edge and speed never has
# room to build toward extreme values in the first place -- the cap should
# now rarely if ever actually engage.
WELD_MAX_SPEED = 0.01                        # m/s, at-rest threshold -- see
                                              # WELD_DISTANCE's 0.04 note
# Companion gate to WELD_MAX_SPEED, new with the dipole model (2026-09-07):
# the magnet now exerts real alignment torque, so a box can be LINEARLY at
# rest (passes WELD_MAX_SPEED) while still rotating into alignment --
# welding it mid-swing would freeze it crooked by construction. The old
# point-attraction model never produced torque, so its gates never needed
# this. 0.5 rad/s sits far above a damped settle (~0) and far below the
# several-rad/s spin the early torque experiments produced, and like
# WELD_MAX_SPEED it also keeps a gripped, still-descending place from
# welding (the carry never has zero angular velocity for long).
WELD_MAX_ANG_SPEED = 0.5                     # rad/s, at-rest threshold
# Misalignment gate, new 2026-09-13 (measured, E2E): the 2026-09-10 stab-ON
# E2E welded sensor 1 at 45.8 deg misalignment (d=0.0316) -- the d/speed/
# ang_speed gates alone happily weld a box that is still toppling or is
# jammed flat against the wall, freezing it crooked by construction (the
# 2026-08-25 "crooked fridge magnet" acceptance was written under the
# torque-free model; the dipole model CAN align in free space but has no
# authority against wall-contact friction, so a crooked weld is a permanent
# defect, not a cosmetic one). Only weld once the bottom face is already
# within this angle of the wall normal. 15 deg sits between the misaligned
# probe's measured free-space settle (30 deg arrival -> 6.6 deg at weld) and
# every observed bad weld (>= 45.8 deg), so it kills crooked welds without
# blocking legitimate ones. A box that arrives worse than this must be
# flattened by the alignment torque BEFORE it may weld -- if it never gets
# there, that is a delivery-pipeline bug to fix at the source, not to mask
# with a permissive gate.
# 15 -> 23 deg (2026-09-14, measured, 50-run batch docs/wall_mount_e2e_50runs/):
# the gate as introduced sat exactly in the middle of the dipole's natural
# equilibrium band. All 134 welded boxes settled at 13.6-15.0 deg, while the 31
# gate_borderline rejects sat at 15.1-22.1 deg (median 15.5) -- same capture
# dynamics, same weld quality, coin-flip outcome against a 15 deg line. The
# distribution is cleanly bimodal: jammed boxes start at 29.5 deg (2x) and the
# original 45.8 deg crooked weld is far above that, so 23 deg accepts the
# entire legitimate equilibrium band (<=22.1 observed) and still blocks every
# jammed/skewed weld the gate was created for. Re-validated with a full 50-run
# batch after the change (see docs/wall_mount_e2e_50runs/).
WELD_MAX_MISALIGN_DEG = 23.0                 # deg, bottom-normal vs wall
# 0.03 -> 0.05 (2026-08-25): with the parking brake pinning the chassis at
# its true authored standoff, the place release point had to move 4.5cm
# short of the wall face (the carried box's pitched corner was being driven
# into the wall -- see wall_mount_task.py's _WALL_X comment), so the magnet
# must capture from ~3-5cm out instead of ~1-3cm. The taper already spans
# the whole range (Bug 5), and the dipole calibration below derives from
# this value the same way the old gain did, so the force law scales with
# it; weld gate/equilibrium unchanged.
# 0.05 -> 0.08 (2026-09-13, measured): the stab-ON E2E's misalignment
# post-mortem (docs/handoff data, /tmp/box_trace_stab_on.csv) showed the
# delivered boxes tumbling to 39-90 deg -- far outside the dipole capture
# cone (a probe at 90 deg flat / d=0.075 was REPULSED: d grew 0.075 ->
# 0.085 and the box escaped). The tumble comes from the 45-deg V-grip at
# grasp (see piper_manipulator's grasp_yaw_snap_step), but even the
# reoriented vertical delivery lands its magnet face ~2.5-5cm off the wall,
# and at 90 deg / 5cm the old 5cm range + 4.6 moment had zero authority.
# The vertical_65 probe (2026-09-13, box vertical at magnet-face d=0.026,
# overrides range=0.08 / moment=11.7) captured, pulled in, and welded at
# 9.7 deg misalignment within 0.6s -- that range is a validated working
# point.
# 0.08 -> 0.10 (2026-09-13): the reoriented vertical delivery can only
# release from above the arm's lower workspace boundary (link6 z ~ -0.05 m,
# measured by an OMPL plan sweep), putting the box's magnet face ~7-9cm
# from its patch at the moment of release -- outside a 0.08 range. See the
# moment recalibration below (same 2 N at the new outer edge).
MAGNET_ATTRACT_RANGE = 0.10                  # m
MAGNET_TAPER_DISTANCE = MAGNET_ATTRACT_RANGE   # interaction ramps down over
                                                # the WHOLE attract range,
                                                # not just the last few mm --
                                                # see Bug 5 above
MAGNET_MAX_FORCE = 2.0                       # N, clamps |F_dipole| (gravity
                                              # handled separately below)
# Dipole moment magnitude of EACH magnet (sensor bottom face + wall patch),
# in A*m^2. Calibration, matching the old law's measured strength: with
# K = 1e-7 * MAGNET_DIPOLE_MOMENT**2 (N*m^4) the axial (face-on) attraction
# of the parallel pair is 6*K/r**4, so setting
# 6*K / MAGNET_ATTRACT_RANGE**4 = MAGNET_MAX_FORCE reproduces the old 2 N
# pull at the attract-range edge (Bug 1's lesson: full strength at the
# OUTER edge, not just near contact): K = 2.0 * 0.05**4 / 6 = 2.08e-6 N*m^4
# -> MAGNET_DIPOLE_MOMENT = sqrt(K / 1e-7) = 4.56 -> 4.6.
# 4.6 -> 11.7 (2026-09-13, recalibrated for MAGNET_ATTRACT_RANGE 0.05 ->
# 0.08): K = 2.0 * 0.08**4 / 6 = 1.365e-5 -> moment = sqrt(K/1e-7) = 11.68.
# 11.7 -> 18.3 (2026-09-13, second recalibration for 0.08 -> 0.10):
# K = 2.0 * 0.10**4 / 6 = 3.33e-5 -> moment = sqrt(K/1e-7) = 18.26. The
# reoriented delivery releases the box ~7cm ABOVE its patch (the arm's
# lower workspace boundary, link6 z ~ -0.05 m measured by OMPL sweep,
# forbids delivering any lower), so the attract range must cover that
# gap; capture from the new 0.8x-range spawn distance (0.08m) is
# validated by probe_magnet_tuning.py's in-range/misaligned modes.
# Same 2 N at the new outer edge; 6x stronger at any FIXED distance (the
# 1/r**4 law against a larger range), which is what gives the magnet
# authority over a tumbled box and over the ~2.5-5cm reoriented delivery
# gap -- measured, not assumed: the vertical_65 probe (moment override
# 11.7) captured a vertical box from magnet-face d=0.026 and welded it at
# 9.7 deg in 0.6s, while the production 4.6 values let the 90-deg flat
# deliveries in the stab-ON E2E jam at d=0.046-0.050 forever unwelded.
# NOTE: the alignment-torque taper (MAGNET_TAPER_FLOOR /
# MAGNET_TORQUE_TAPER_DISTANCE) still bottoms out at 3cm -- at larger d
# the torque-per-degree is WEAKER than before at the same distance
# (K up 6.6x but taper window 0.08-0.003 vs 0.05-0.003). The misaligned
# probe must be re-run (stage-D validation) to confirm free-space tilt
# recovery still converges.
#
# SIZING TENSION, measured against the formula, not assumed: the point-
# dipole torque from the SAME coupling is |tau| = 2*K*sin(theta)/r**3 near
# the axis -- already 0.045 N*m at r = 4cm / 45 deg, and 0.11 N*m at r =
# 3cm / 45 deg. Against I_yy = 3.33e-5 kg*m^2 that is 1300-3300 rad/s^2,
# and the linearized torsional stiffness 2*K/r**3 (~50-160 N*m/rad in the
# working range) puts the undamped natural frequency at 1200-2200 rad/s --
# ABOVE the 120Hz timestep, so explicit integration would alias-and-chatter
# long before damping (hard-capped at 2*I/dt ~= 0.008 N*m*s/rad by the
# explicit-Euler bound) could do anything about it. That is why
# MAGNET_MAX_TORQUE below is NOT sized "2x above the working range" as a
# never-engaging safety: it is the FINITE-SIZE SATURATION of the magnet.
# These magnets are extended bodies (8x3.5cm faces) and the working range
# (3-5cm) is comparable to the magnet's own size -- the regime where the
# point-dipole 1/r**3 torque systematically OVERestimates a real magnet's
# restoring torque (the pole distribution is not two point charges; once
# face overlap stops changing with angle, real torque saturates -- a real
# fridge magnet clicks into place, it does not slam-align at hundreds of
# rad/s^2). The clamp is how that finite-size physics enters the model.
# Validated by probe_magnet_tuning.py --mode misaligned.
MAGNET_DIPOLE_MOMENT = 18.3                  # A*m^2, each magnet
# 0.02 -> 0.002 (2026-09-07, measured): the first clamp value, picked as
# "2x above plausible", turned out 10-100x too hot for this moment of
# inertia. A live in-range probe (which starts with the bottom face
# STRAIGHT at the wall) showed the box spun up to ang_speed 5-10 rad/s
# within 0.5s of entering the range and stayed tumbling for the full 3s
# window -- misalignment GROWING to ~20 deg -- because MAGNET_MAX_TORQUE
# 0.02 N*m on I_yy = 3.33e-5 kg*m^2 injects up to 0.02*0.5 = 0.01 J over a
# 30-deg swing while the largest kinetic energy the weld gate tolerates is
# 0.5*I*0.5^2 = 4e-6 J; no damping the explicit integrator can carry
# (bounded at 2*I/dt ~= 0.008 N*m*s/rad) can absorb that gap -- the
# measured terminal angular speed under the old 0.01-N*m torqueGain
# experiments ("several rad/s") had already hinted at exactly this. 0.002
# N*m bounds the peak swing energy to ~1e-3 J (omega ~ 8 rad/s worst case,
# bled back below the 0.5 rad/s weld gate by MAGNET_ANGULAR_DAMPING_COEFF
# at zeta ~ 0.4 within ~0.2s) while still aligning a 30-deg arrival in
# ~0.2-0.4s -- well inside the probe/E2E settling windows. For reference,
# cm-scale uniformly-magnetized blocks in these fields saturate at
# ~1e-3-1e-2 N*m, so 0.002 is also the physically plausible range for the
# finite-size effect this clamp models.
MAGNET_MAX_TORQUE = 0.002                    # N*m, clamps |tau_dipole|
# 2.0 -> 0.0 -> back to 2.0 (2026-09-08, Bug 9 -- MEASURED, bisected live
# per-component): the Newton freeze (pose frozen bit-for-bit for hundreds
# of ticks, velocity readback pinned at a constant nonzero value) is
# triggered by the ANGULAR velocity-feedback damping term ALONE. Bisect
# (node attrs toggled live, one component at a time): both-damping-zero
# -> body integrates but the undamped approach slams into the 0.3 m/s
# velocity cap, spins up and escapes the attract band; LINEAR damping
# 2.0 alone -> controlled approach, deceleration through the band, clean
# weld at d=0.0093 (no freeze); ANGULAR damping 0.003 (with linear off)
# -> the original freeze. So MAGNET_DAMPING_COEFF keeps its Bug-2/4
# load-bearing role (a constant attractive force with no velocity
# feedback is bang-bang and cannot converge), while
# MAGNET_ANGULAR_DAMPING_COEFF stays retired at 0.0 -- angular settling
# falls to the body-authored SENSOR_CAM_ANGULAR_DAMPING=5.0 (inside the
# integrator, demonstrably freeze-free) plus the dipole alignment torque
# itself below MAGNET_TORQUE_CUTOFF. Bug 2's explicit-Euler sizing note
# below is moot for the retired term; for this one 2.0 is well under the
# 2*m/dt = 12 N*s/m bound it cites.
MAGNET_DAMPING_COEFF = 2.0                     # N*s/m, linear drag while in attract range
MAGNET_GRAVITY_FEEDFORWARD_Z = SENSOR_CAM_MASS * 9.81   # N, world +Z, cancels weight
                                                          # while the magnet is active
# Bug 6 (implied torque from an off-center force, unmasked by fixing Bug 5's
# degree/radian cap): with the maxAngularVelocity unit bug fixed (see
# apply_graspable_rigid_body_tuning's comment -- physxRigidBody:
# maxAngularVelocity is in DEGREES/s, was silently capping rotation to
# ~0.1745 rad/s = 10 deg/s regardless of any authored "10.0" rad/s value),
# the OLD model's radial attraction force -- applied at bottom_world, ~2cm
# off the object's COM -- was free to spin the object at several rad/s with
# nothing to arrest it (MAGNET_DAMPING_COEFF only ever opposed LINEAR
# velocity). Fixed two ways together back then: (1) the radial force moved
# to the COM (see author_magnet_graph.py's history), removing the
# implied-torque source entirely for that term; (2) MAGNET_ANGULAR_DAMPING_
# COEFF opposes whatever rotation remains the same way MAGNET_DAMPING_COEFF
# opposes linear velocity. The dipole rewrite (2026-09-07) TRIED restoring
# application at the dipole point -- bottom face center -- on the theory
# that tau = m_s x B_w is the exact physical counterpart of that r x F
# term. Measured against the live in-range probe it failed for the
# discrete-time reason above: the instantaneous r x F at 120Hz (up to
# 0.02m lever arm x 2N clamped force = 0.04 N*m, swinging with the body's
# own rotation) pumped angular energy ~20x beyond the physical torque
# clamp (0.002 N*m) and spun even a straight-on arrival up to 5-10 rad/s
# of permanent tumble. So COM application stays; the dipole model's
# physical content lives entirely in the orientation-dependent force
# DIRECTION and the pure torque tau = m_s x B_w. Sized well under the
# explicit-Euler stability
# bound for this object's moment of inertia:
# I_yy = m*(a^2+c^2)/12 for the 8x4cm cross-section (rotation about the
# short/Y axis, the axis actually observed spinning) = 0.05*(0.08^2+0.04^2)/12
# ~= 3.33e-5 kg*m^2; bound is c < 2*I/dt = 2*3.33e-5*120 ~= 0.008 N*m*s/rad.
# 0.003 -> 0.0 (2026-09-08, Bug 9): this ANGULAR velocity-feedback term is
# the sole freeze trigger (see MAGNET_DAMPING_COEFF's Bug-9 note for the
# live per-component bisect). Angular settling is handled by the
# body-authored SENSOR_CAM_ANGULAR_DAMPING=5.0 inside the integrator
# plus, below MAGNET_TORQUE_CUTOFF, the dipole alignment torque itself.
MAGNET_ANGULAR_DAMPING_COEFF = 0.0              # N*m*s/rad, DISABLED -- freeze trigger (Bug 9)
# Sized to hold firmly through normal handling (~10x the object's own
# weight) while still breaking under a deliberate pull -- not sized to be
# unbreakable.
MAGNET_BREAK_FORCE = 5.0                     # N
MAGNET_BREAK_TORQUE = 0.05                   # N*m


def _set(prim, name, type_name, value):
    """Authors a raw attribute, creating it if the schema didn't."""
    prim.CreateAttribute(name, type_name).Set(value)


def _uint(prim, name, value):
    _set(prim, name, Sdf.ValueTypeNames.UInt, int(value))


def _float(prim, name, value):
    _set(prim, name, Sdf.ValueTypeNames.Float, float(value))


def _bool(prim, name, value):
    _set(prim, name, Sdf.ValueTypeNames.Bool, bool(value))


def _token(prim, name, value):
    _set(prim, name, Sdf.ValueTypeNames.Token, value)


def apply_physx_scene_tuning(prim):
    """PhysxSceneAPI on the physics scene prim.

    The scene previously had PhysxSceneAPI applied with zero attributes (it
    was added only so omni.physx.tensors would recognise the scene as active
    -- see import_robots.py's comment), leaving solver type, iteration
    counts, CCD, stabilization and the step rate all at PhysX defaults.
    """
    prim.AddAppliedSchema('PhysxSceneAPI')
    _token(prim, 'physxScene:solverType', SOLVER_TYPE)
    _uint(prim, 'physxScene:minPositionIterationCount', SOLVER_POSITION_ITERATIONS)
    _uint(prim, 'physxScene:minVelocityIterationCount', SOLVER_VELOCITY_ITERATIONS)
    _uint(prim, 'physxScene:timeStepsPerSecond', PHYSICS_TIME_STEPS_PER_SECOND)
    _bool(prim, 'physxScene:enableCCD', True)
    _bool(prim, 'physxScene:enableStabilization', True)
    return prim


def apply_physx_articulation_tuning(prim):
    """PhysxArticulationAPI on an articulation root.

    Per hsr_urdf_model.py:186-194, whose docstring records a catastrophic
    articulation explosion (joint values reaching 1e7-1e13 within ~130
    physics steps) caused by nothing other than the default 4/1 solver
    iteration counts, confirmed fixed by bumping them alone.
    """
    prim.AddAppliedSchema('PhysxArticulationAPI')
    _uint(prim, 'physxArticulation:solverPositionIterationCount',
          SOLVER_POSITION_ITERATIONS)
    _uint(prim, 'physxArticulation:solverVelocityIterationCount',
          SOLVER_VELOCITY_ITERATIONS)
    _bool(prim, 'physxArticulation:enabledSelfCollisions', False)
    _float(prim, 'physxArticulation:sleepThreshold', 0.0)
    _float(prim, 'physxArticulation:stabilizationThreshold', 0.0)
    return prim


def apply_graspable_rigid_body_tuning(
        prim,
        max_depenetration_velocity=SENSOR_MAX_DEPENETRATION_VELOCITY,
        max_linear_velocity=SENSOR_MAX_LINEAR_VELOCITY,
        max_angular_velocity=SENSOR_MAX_ANGULAR_VELOCITY,
        contact_offset=SENSOR_CONTACT_OFFSET,
        rest_offset=SENSOR_REST_OFFSET,
        linear_damping=0.0,
        angular_damping=0.0):
    """PhysxRigidBodyAPI + PhysxCollisionAPI on the object being grasped.

    Defaults reproduce the original single-object behaviour exactly (every
    existing caller passes no overrides). The wall-mount task's 4 sensor-cam
    boxes are smaller than the original demo cube (4cm vs 5cm), so they pass
    their own SENSOR_CAM_CONTACT_OFFSET -- contactOffset is meant to be a
    fraction of the object's OWN size (see SENSOR_CONTACT_OFFSET's comment),
    not a fixed absolute value borrowed from a differently-sized object.
    They also pass SENSOR_CAM_LINEAR_DAMPING/SENSOR_CAM_ANGULAR_DAMPING (see
    that constant's own comment for why a resting-drift problem this
    function's sleepThreshold=0.0/stabilizationThreshold=0.0 below
    tolerates fine for a briefly-resting pedestal cube needed damping here).
    """
    prim.AddAppliedSchema('PhysxRigidBodyAPI')
    _bool(prim, 'physxRigidBody:enableCCD', True)
    _uint(prim, 'physxRigidBody:solverPositionIterationCount',
          SOLVER_POSITION_ITERATIONS)
    _uint(prim, 'physxRigidBody:solverVelocityIterationCount',
          SOLVER_VELOCITY_ITERATIONS)
    _float(prim, 'physxRigidBody:sleepThreshold', 0.0)
    _float(prim, 'physxRigidBody:stabilizationThreshold', 0.0)
    _float(prim, 'physxRigidBody:maxDepenetrationVelocity',
           max_depenetration_velocity)
    _float(prim, 'physxRigidBody:maxLinearVelocity', max_linear_velocity)
    # physxRigidBody:maxAngularVelocity is in DEGREES/second, not radians --
    # a THIRD instance of this codebase's recurring degree/radian trap (see
    # ARM_STIFFNESS's per-degree DriveAPI note and GRIPPER_MAX_JOINT_VELOCITY's
    # "degrees(1 rad) leaking" note above). Found via the wall-mount task's
    # magnet probe: SENSOR_MAX_ANGULAR_VELOCITY=10.0 (authored assuming
    # rad/s) produced an object whose angular speed was hard-pinned at
    # EXACTLY 0.17453292 rad/s in a live run -- which is exactly 10 DEGREES
    # per second (pi/18) to 7+ significant figures, not a coincidence.
    # Confirmed by raising the raw authored value to 1000.0 and observing
    # angular speed jump into the several-rad/s range (previously
    # impossible under the "10.0" cap) -- the attribute was always being
    # read as degrees, capping every SENSOR/SENSOR_CAM object's rotation to
    # roughly 1/57th of what every constant in this file assumes. Fixed by
    # converting at the authoring boundary, same DEGREES_PER_RADIAN pattern
    # already used for ARM_STIFFNESS_AUTHORED -- SENSOR_MAX_ANGULAR_VELOCITY
    # itself stays in rad/s (the unit every caller and comment already
    # assumes), only the raw USD attribute gets the conversion.
    _float(prim, 'physxRigidBody:maxAngularVelocity',
           max_angular_velocity * DEGREES_PER_RADIAN)
    _float(prim, 'physxRigidBody:linearDamping', linear_damping)
    _float(prim, 'physxRigidBody:angularDamping', angular_damping)

    prim.AddAppliedSchema('PhysxCollisionAPI')
    _float(prim, 'physxCollision:contactOffset', contact_offset)
    _float(prim, 'physxCollision:restOffset', rest_offset)
    return prim


def apply_collision_contact_tuning(prim, contact_offset, rest_offset):
    """PhysxCollisionAPI alone, for a static/kinematic collider prim.

    Counterpart of apply_graspable_rigid_body_tuning for bodies that carry
    no rigid-body dynamics (the wall cube). The wall previously shipped only
    UsdPhysics.CollisionAPI, leaving PhysX's 0.02 m default contactOffset
    un-authored -- an invisible 2 cm contact "pillow" the sensor boxes then
    hung and welded on (see WALL_CONTACT_OFFSET's comment).
    """
    prim.AddAppliedSchema('PhysxCollisionAPI')
    _float(prim, 'physxCollision:contactOffset', contact_offset)
    _float(prim, 'physxCollision:restOffset', rest_offset)
    return prim


def define_physics_material(stage, path, static_friction, dynamic_friction,
                            restitution=0.0,
                            friction_combine_mode=FRICTION_COMBINE_MODE):
    """Defines a UsdShade.Material carrying UsdPhysics.MaterialAPI + PhysX
    combine modes. Returns the UsdShade.Material."""
    material = UsdShade.Material.Define(stage, path)
    prim = material.GetPrim()
    mat_api = UsdPhysics.MaterialAPI.Apply(prim)
    mat_api.CreateStaticFrictionAttr(float(static_friction))
    mat_api.CreateDynamicFrictionAttr(float(dynamic_friction))
    mat_api.CreateRestitutionAttr(float(restitution))
    prim.AddAppliedSchema('PhysxMaterialAPI')
    _token(prim, 'physxMaterial:frictionCombineMode', friction_combine_mode)
    _token(prim, 'physxMaterial:restitutionCombineMode', 'min')
    return material


def bind_physics_material(prim, material):
    """Binds `material` to `prim` with materialPurpose='physics'."""
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, materialPurpose='physics')
    return prim
