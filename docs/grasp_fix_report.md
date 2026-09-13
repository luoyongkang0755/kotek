# Why the gripper threw the sensor, and what actually fixed it

Companion to `pick_and_delivery_report.md`. That report's section 4.19 ran 12 live
trials of the grasp, established that the object genuinely leaves the gripper
(a smooth lift → free-fall → bounce → rest trajectory in the object's own
ground-truth pose, not a perception artefact), ruled out grip width and lift
speed, and closed with:

> a genuine, unresolved physical grip/friction question … no effort/driver
> telemetry available

The telemetry existed. `Articulation.get_dof_gains()` reports it, and reading it
showed the cause was not friction. This document records what the numbers were,
how each was measured, and what changed.

Everything below is reproducible with `~/Projects/kotek_sim/tests/test_grasp.py`,
which runs one isolated grasp — approach, close, hold, lift, hold — in two modes
that differ **only** in the physics, with identical arm motion, object placement
and timing.

---

## The headline measurement

```
                        --mode legacy        --mode fixed
peak object speed       4.29 m/s             0.18 m/s
finger stall width      0.0400 m (on air)    0.0520 m (= the object)
slip after lift+hold    0.1125 m             0.0011 m
object rise             -0.325 m             +0.109 m
object final position   (0.03, -2.05, 0.03)  (0.30, 0.00, 0.46)
verdict                 GRASP FAILED         GRASP HELD
```

In legacy mode the object is ejected at 4.3 m/s and comes to rest on the floor
more than two metres away. In fixed mode it is picked up and held.

---

## Five faults, all in the asset

### 1. The gripper was a ~900 N vise on a 0.2 kg object

`import_robots.py` applied one constant set to every joint:

```python
JOINT_STIFFNESS = 1.0e5;  JOINT_DAMPING = 1.0e2;  JOINT_MAX_FORCE = 1.0e3
for gripper_joint in ('joint7', 'joint8'):
    set_drive_gains(..., True, JOINT_STIFFNESS, JOINT_DAMPING, JOINT_MAX_FORCE)
```

`JOINT_MAX_FORCE = 1.0e3` is documented in that file as being chosen for
**joint3's gravity-holding torque, in N·m**. `joint7`/`joint8` are *prismatic*,
so the same number means **newtons**. The URDF says `effort="100"`.

`CLOSE_GRIPPER` commanded `joint7 = 0.006` and the logs quoted in
`manipulation.yaml` show the fingers stalling at 0.0088–0.0151 — a 3–9 mm
steady-state drive error against 1e5 N/m:

    F = min(1e5 × 0.009, 1000) ≈ 900 N per finger

held continuously, because `isaac_joint_bridge` stops publishing at the end of a
trajectory and Isaac's articulation controller latches the last position target.

This also explains section 4.19's Finding 2 — tightening `grasp_width`
0.024 → 0.012, which *increases* the commanded overshoot, changed nothing. Both
settings were already saturating `maxForce`.

**Fixed:** `physics_tuning.GRIPPER_{STIFFNESS,DAMPING,MAX_FORCE}` = 1e3 N/m,
50 N·s/m, 20 N. Holding 0.2 kg at µ = 0.9 needs 1.09 N per finger, so 20 N is
~18× margin.

### 2. The arm was in a sustained limit cycle, and that is what moved the object first

This was the real surprise. `tests/test_grasp.py` reports the phase in which the
object *first* moves, and with the drive gains corrected it was still moving
during the **approach**, before the gripper was ever commanded to close.

Tracing the fingertip through a commanded straight 10 cm descent:

```
n=260  y=-0.0157   joint1=-0.010
n=280  y=+0.0608   joint1=+0.043
n=300  y=-0.0672   joint1=-0.045
n=320  y=+0.0668   joint1=+0.030
n=340  y=-0.0594   joint1=+0.005
```

`joint1` oscillating at ~3 Hz, swinging the gripper ±6–7 cm sideways — far
enough to sweep the finger bodies into the object and knock it off its pedestal.

The cause is the arm's damping ratio: `damping/stiffness = 1e2/1e5 = 0.001`.
`probe_arm_damping.py` sweeps it and measures the peak lateral excursion away
from the commanded straight line:

```
ratio 0.001 (the asset)   max|y| = 0.0499 m   max lateral = 0.0503 m
ratio 0.01                max|y| = 0.0312 m   max lateral = 0.0312 m
ratio 0.1                 max|y| = 0.0001 m   max lateral = 0.0022 m
ratio 0.45                max|y| = 0.0013 m   max lateral = 0.0163 m
ratio 1.0                 max|y| = 0.0079 m   max lateral = 0.0444 m
```

**Fixed:** ratio 0.1 — a 500× reduction, and the best tracking error in the
sweep. Note this is *not* `hsr_sim`'s proven 0.45, which is why it was measured
rather than inherited.

This also retroactively explains a "reach limit" that was not one: before the fix,
an IK reachability scan reported 4.6 cm residuals and failures at low targets;
after it, the same scan reaches **9/9** targets — including (0.48, 0, 0.30), the
real standoff and object height — with 2–9 mm of error.

### 3. Units: the angular drives are per-DEGREE

`UsdPhysics`' angular `DriveAPI` is per-degree. Measured directly on this asset:
authoring `1e5` produced a reported (radian-scale) stiffness of **5729578**, i.e.
`1e5 × 57.2958`, and `1e2` produced `5729.578`.

This is a 57× error in the gain that gets validated, so `physics_tuning` now
states the arm gains in radian scale and authors `ARM_*_AUTHORED = value /
57.2958`. The same unit leak put `physxJoint:maxJointVelocity = 57.29578` on the
*prismatic* finger joints — 57 m/s of finger speed — now 0.2 m/s.

The **wheel** drives are deliberately left unconverted: `WHEEL_DAMPING = 1e6` is
the value the base-drive fix was separately validated against (58–60 cm in 2 s,
after eleven ruled-out hypotheses in report section 1.3). Rescaling it on the
strength of an argument rather than a measurement would be a regression risk for
no benefit.

### 4. `/World/sensor` had no PhysX properties at all

The prim carried `UsdPhysics.RigidBodyAPI` + `CollisionAPI` and nothing else —
verified against the binary stage, where `strings` yields **zero `physx*` tokens**
on it. So `maxDepenetrationVelocity`, the solver iteration counts and the
sleep/stabilization thresholds were all at defaults, and `contactOffset` sat at
PhysX's 0.02 m — **40% of the object's own 5 cm width**.

The scene was equally untuned: `PhysxSceneAPI` applied with **zero attributes**
(4 position / 1 velocity iterations at 60 Hz), and neither articulation carried
`PhysxArticulationAPI` at all. Without the scene/articulation fix the arm does
not merely grasp badly — it diverges outright, ending 1.53 m from its IK target
(`test_grasp.py --legacy-scene`).

**`maxDepenetrationVelocity` is a genuine trade-off in both directions**, and
getting it wrong the *other* way cost a full debugging cycle here. The same cap
that limits an ejection also limits how fast ordinary contact is resolved:

```
depenV  closeV  stall width  squeeze   rise      slip     held
0.05    0.05    0.0291 m     8.5 N     -0.0000   0.1045   no
0.05    0.20    0.0296 m     8.8 N     +0.0000   0.1028   no
0.50    0.05    0.0519 m    20.0 N     +0.1071   0.0049   YES
0.50    0.20    0.0518 m    19.9 N     +0.1051   0.0028   YES
5.00    0.20    0.0519 m    20.0 N     +0.1051   0.0030   YES
```

At 0.05 the fingers close to 29 mm on a 51 mm object and the object never moves
at all — contact resolved far too weakly, which looks nothing like a slip. At
0.5 the stall width lands on the object's real extent. 0.5 is chosen as the
smallest value that works, i.e. the tightest remaining cap against ejection.

### 5. Friction was not what the file said, and the object was the wrong shape

The fingers had **no physics material bound**, so the object's authored
`staticFriction = 1.0` was combined against PhysX's default 0.5. And the "cube"
was a size-1.0 `UsdGeom.Cube` with a **non-uniform** `AddScaleOp(0.05, 0.05,
0.08)` — a scaled analytic collider, gripped on its narrow face with its long
axis vertical, maximising the tipping moment.

**Fixed:** high-friction material bound to the fingers too (with
`frictionCombineMode = max`), `convexDecomposition` instead of a convex hull of
the fork-shaped finger mesh, and a true unscaled 5 cm cube.

---

## What did NOT need to change

`piper_manipulator`, `isaac_joint_bridge`, `task_coordinator` and
`kotek_base_control` are untouched. Every fault above is in the asset, so the ROS
pipeline inherits the fix by rebuilding the stage. The only ROS-side edit is
`manipulation.yaml`'s `grasp_width`, and that is about *release*, not hold: at
the old 0.012 the residual saturates the 20 N cap, and opening from a saturated
squeeze throws the object ~0.5 m. At 0.040 the squeeze is ~5.9 N — still 5.5×
what is needed to hold 0.2 kg — and the release is clean.

## Regression guards

`inspect_stage.py --assert-demo-ready` now asserts every number above directly:
the PhysX API schemas are present, the solver iteration counts and step rate are
raised, `maxDepenetrationVelocity` and `contactOffset` are at their measured
values, the finger drives are at 1e3 N/m / 20 N, `maxJointVelocity` is a sane
finger speed, and the arm damping ratio and *authored* per-degree stiffness are
correct. That last one is what catches the 57× unit error.

This whole class of bug — "authored nothing, silently got a default" — leaves no
trace in the file, which is why it is asserted rather than trusted.

## Reproducing

```bash
cd ~/Projects/kotek_sim
./run.sh tests/test_grasp.py --mode legacy   # ejects the object, exit 1
./run.sh tests/test_grasp.py --mode fixed    # holds it,          exit 0
./run.sh probe_arm_damping.py                # the damping sweep
./run.sh probe_grasp_tuning.py               # the depenetration sweep
./run.sh main.py --screenshots               # full no-ROS pick and place
```
