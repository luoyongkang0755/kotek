#!/usr/bin/env python3
"""Authors the simulated-magnetism graph for the wall-mount task on the
wall-demo stage produced by build_wall_stage.py.

Magnets exist ONLY on the sensor's bottom face and the wall's 4 target
patches -- nothing else in the scene is magnetic. This graph enforces that
by iterating a FIXED list of (sensor_prim, wall_target) pairs, authored as
dynamic input attributes on the node (inspectable via USD, not baked into
the Python source string), not a general "any two magnetic surfaces
attract" system.

Every other OmniGraph in this codebase (author_robot_graphs.py,
author_camera_graphs.py) uses only built-in node types wired with
omni.graph.core.Controller.edit(). This is the first departure from that:
the magnet law needs genuine per-tick Python (a magnetic dipole-dipole
force AND torque pair in the original design; the contact-holddown force
law + aligning torque of the current one) that no built-in node provides,
so it is implemented as a single omni.graph.scriptnode.ScriptNode node
instead.

USER DECISION 2026-10-04 -- NO WELD: the weld (a one-shot dynamically
authored UsdPhysics.FixedJoint) was the last non-physical element. A real
weak magnet on a metal wall has no weld: the box stays up purely by
contact friction under the magnet's normal holddown force (3N x mu 0.9 =
2.7N vs the 0.49N shear weight). The joint authoring is REMOVED; the old
weld gates (distance/speed/ang_speed/misalign + gripper-open) are kept
verbatim but now DECLARE a friction hold, with one physically necessary
addition -- d < attract_range, because a box declared held must be inside
the magnet's active range (the weld era masked the [attract_range,
weld_distance) band: the joint held even where the magnet force is zero).
After declaration the magnet force simply keeps acting every tick -- that
IS the hold -- and a slip watchdog reports any drift. The
gravity-feedforward stabilizer is retired with the same decision: a real
magnet does not actively cancel weight; friction does the job. The node
inputs (weld*, gravityFeedforwardZ, break*) stay for graph-interface
stability and are ignored or re-interpreted in compute().

Runs INSIDE Isaac Sim, same headless invocation as author_camera_graphs.py:
    KOTEK_WITH_ROS=1 ./run_isaac.sh \\
        src/kotek_isaac_stage/kotek_isaac_stage/author_magnet_graph.py

No ordering dependency on author_camera_graphs.py -- run either first, both
before the /piper/compute_ik reachability sweep and the live E2E test (see
the wall-mount task plan's build/verification sequencing).
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WALL_DEMO_STAGE = (
    '/home/trs/kotek_ws/src/kotek_isaac_stage/usd/'
    'kotek_scout_piper_wall_demo.usd')

# The direction from the robot's side of the room toward the wall -- i.e.
# the direction the attraction force pulls sensors, and the direction each
# sensor's bottom-face outward normal should end up pointing once mounted.
# Matches build_wall_stage.py's WALL_FACE_X being positive-x, the robot
# starting at the world origin behind it.
WALL_INWARD_DIR = (1.0, 0.0, 0.0)

MAGNET_NODE_NAME = 'kotek_magnet'
MAGNET_GRAPH_PATH = '/World/kotek_magnet_graph'

# --- the embedded per-tick script -----------------------------------------
# A real, formatted, reviewable Python module (this file), not a hand-typed
# one-liner -- assigned to inputs:script below, per the ScriptNode
# framework's setup(db)/compute(db)/cleanup(db) contract
# (omni.graph.scriptnode's own example scripts document this exact shape).
MAGNET_SCRIPT_SOURCE = '''
"""Per-tick magnet logic for the wall-mount task. See
author_magnet_graph.py (which authors this node) for the full design
rationale -- this string is that file's own MAGNET_SCRIPT_SOURCE constant,
kept here only because ScriptNode requires the script as node data.

PHYSICS: magnetic dipole-dipole interaction (rewritten 2026-09-07, replacing
the old point-attraction law F = clamp(GAIN/d**2, 0, MAX) plus a separate,
never-enabled torqueGain alignment knob). Each (sensor, wall target) pair
is two dipoles of equal moment magnitude `dipoleMoment`: the wall patch's
moment points along `wallInwardDir`, the sensor's along its own bottom-face
outward normal (local -Z), so the two moments are PARALLEL once the sensor
is correctly mounted and the interaction is then purely attractive. With
r_vec the dipole-to-dipole vector (wall patch -> sensor bottom face) and
MU0_OVER_4PI = 1e-7 T*m/A:

    B_w = MU0_OVER_4PI / r^3 * [3 (m_w . r_hat) r_hat - m_w]
    F   = 3 MU0_OVER_4PI / r^4 * [(m_w . r_hat) m_s
          + (m_s . r_hat) m_w + (m_w . m_s) r_hat
          - 5 (m_w . r_hat) (m_s . r_hat) r_hat]
    tau = m_s x B_w

Two behaviors EMERGE from these formulas -- this is why the model was
rewritten:

  * ALIGNMENT: near the mount axis B_w points along +x, so tau rotates the
    bottom-face normal onto the wall direction and holds it there (a
    backwards-mounted sensor, moments anti-parallel, is REPELLED -- real
    dipole physics, not a special case).
  * LATERAL CAPTURE: off-axis, B_w tilts toward the mount axis, so the
    sensor is swung toward its own target point, not just the wall plane.

Engineering constraints carried over from the old model (see
physics_tuning.py's bug history -- measured, not guessed):

  * The FORCE is tapered to zero as r approaches taperFloor (the old
    model's proven shape -- Bug 4/8 history there). The TORQUE has its
    own, earlier cutoff at torqueCutoff: inside ~3cm the discrete 120Hz
    tau feedback loop is integration-unstable at any clamp value measured
    (0.02 and 0.002 N*m both ended in cap-pinned spin), so the alignment
    authority lives in the 3-5cm band and the final flattening is done by
    wall CONTACT (the force law still pulls the box into the face; 0.9
    friction at a ~2N press out-torques the clamp by an order of
    magnitude). This deliberately does NOT preserve the F:tau ratio in
    the near field -- a documented deviation from point-dipole physics,
    chosen because the point-dipole model is not valid at separations
    comparable to the magnet's own size anyway.
  * |F| is clamped to maxForce. |tau| is clamped to maxTorque, which --
    unlike the force cap -- DOES engage in the working range, deliberately:
    it is the finite-size saturation of the magnet. The point-dipole 1/r**3
    torque overestimates a real extended magnet (8x3.5cm faces) once the
    separation is comparable to the magnet's own size, and the raw torque
    would torsionally stiffen the box far above the 120Hz timestep (see
    physics_tuning.py's MAGNET_DIPOLE_MOMENT sizing-tension note).
  * Everything is gated on d < attractRange; the linear damping and
    gravity-feedforward stabilizers are STILL applied (Bugs 2/3/4 -- a
    constant attractive force with no velocity feedback is bang-bang and
    cannot converge; the radial force must never cover gravity). Only
    the ANGULAR damping twin is retired, zero since physics_tuning.py's
    Bug 9 (2026-09-08): it is the sole Newton freeze trigger (live
    per-component bisect); angular settling falls to the body-authored
    SENSOR_CAM_ANGULAR_DAMPING plus the dipole alignment torque.
  * F is applied at the CENTER OF MASS, tau as a pure torque. Continuum
    mechanics says a dipole force acts at the dipole point with the r x F
    moment about the COM as real physics -- and the FIRST version of this
    rewrite did exactly that. It failed MEASURED, not in theory: a live
    in-range probe starting with the bottom face STRAIGHT at the wall spun
    the box up to 5-10 rad/s and kept it tumbling (misalignment growing
    to ~20 deg, never passing the weld gate), because the discrete-time
    r x F term at 120Hz -- up to 0.02m lever arm x 2N clamped force =
    0.04 N*m, swinging with the body's own rotation -- pumps angular
    energy far faster than the physical tau (clamped 100x lower at
    0.002 N*m) or any admissible damping can remove. This is
    physics_tuning.py Bug 6's lesson repeating at a new angle: in this
    integrator, any force whose application point is not the COM is a
    wildcard. The physical content of the dipole model (orientation-
    dependent force direction + tau = m_s x B_w) is fully retained; only
    the application point moves to the COM.
  * Weld logic: REMOVED 2026-10-04 (user decision, see the module
    docstring) -- no more FixedJoint. The old gate conditions are kept as
    the FRICTION-HOLD DECLARATION (plus d < attract_range), after which the
    magnet force keeps acting every tick; a slip watchdog (10mm drift from
    the declared position, or lost wall contact) reports any failure of
    the friction hold loudly.
    gate (physics_tuning.py's WELD_MAX_MISALIGN_DEG) stops a still-toppling
    or wall-jammed box from being declared held while crooked -- a
    declaration on a crooked box would just be the friction hold failing
    loudly a moment later instead of being frozen permanently.

Reads the fixed (sensor, target) pairs and every tunable constant from
this node's own input attributes -- see author_magnet_graph.py for where
those are set from physics_tuning.py.
"""
import numpy as np


def _rotate_vec_by_quat_wxyz(q, v):
    """Rotates 3-vector v by quaternion q=(w,x,y,z). Standard formula, no
    external dependency -- this runs inside an embedded script node, which
    should not assume anything beyond numpy is importable."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    vv = np.array(v, dtype=float)
    t = 2.0 * np.cross(qv, vv)
    return vv + w * t + np.cross(qv, t)


def setup(db):
    state = db.per_instance_state
    state.rigid_prims = None       # lazily constructed on first compute() --
                                    # RigidPrim needs physics playing already
    state.welded = None            # bool per sensor, set once rigid_prims exists
    state.held_pos = None          # world pos at friction-hold declaration
                                   # (watchdog anchor), same lazy init
    state.slip_reported = None     # bool per sensor, latches the one-shot
                                   # SLIPPED report so it prints once
    state.tick_count = 0           # diagnostic heartbeat -- see compute() below


def cleanup(db):
    state = db.per_instance_state
    state.rigid_prims = None
    state.welded = None
    state.held_pos = None
    state.slip_reported = None


def compute(db):
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics
    from isaacsim.core.experimental.prims import RigidPrim

    # 1e-7 T*m/A by the SI definition of mu0 (1.00000000055e-7 since the
    # 2019 redefinition -- 0.55 ppb, far below every other uncertainty in
    # this sim).
    MU0_OVER_4PI = 1.0e-7

    sensor_paths = list(db.inputs.sensorPaths)
    n = len(sensor_paths)
    if n == 0:
        return True

    state = db.per_instance_state
    state.tick_count += 1

    if state.rigid_prims is None:
        try:
            state.rigid_prims = RigidPrim(sensor_paths)
            state.welded = [False] * n
            state.held_pos = [None] * n
            state.slip_reported = [False] * n
        except Exception as exc:  # noqa: BLE001 -- physics not playing yet
            # Plain print(), not db.log_warning() -- a live probe run showed
            # NEITHER this message nor the weld-success one below ever
            # reaching a captured stdout/stderr log across ~180 ticks, while
            # this same script's own print()-based diagnostics from the
            # calling probe DID show up -- strong evidence db.log_warning()
            # routes to a different sink (Carb's own log system) than
            # what run_isaac.sh's caller captures, not that compute() never
            # ran. print(flush=True) matches this whole codebase's own
            # convention for exactly this reason.
            print(f'### kotek_magnet[tick={state.tick_count}]: '
                  f'RigidPrim not ready yet ({exc})', flush=True)
            return True

    flat_targets = list(db.inputs.wallTargets)
    targets = [tuple(flat_targets[3 * i:3 * i + 3]) for i in range(n)]
    inward = np.array(list(db.inputs.wallInwardDir), dtype=float)
    inward = inward / max(np.linalg.norm(inward), 1e-9)

    attract_range = float(db.inputs.attractRange)
    max_force = float(db.inputs.maxForce)
    moment = float(db.inputs.dipoleMoment)
    max_torque = float(db.inputs.maxTorque)
    damping_coeff = float(db.inputs.dampingCoeff)
    gravity_feedforward_z = float(db.inputs.gravityFeedforwardZ)
    # RETIRED 2026-10-04 (user decision, weld-free friction hold): the
    # feedforward actively cancelled the box's weight while pressed -- a
    # real wall magnet does no such thing; the 0.49N shear is carried by
    # contact friction (mu 0.9 x the 3N normal = 2.7N). Zeroed here; the
    # input stays only for graph-interface stability (re-authoring the
    # node to drop it would churn the stage for no behavioral gain).
    gravity_feedforward_z = 0.0
    angular_damping_coeff = float(db.inputs.angularDampingCoeff)
    weld_distance = float(db.inputs.weldDistance)
    taper_distance = float(db.inputs.taperDistance)
    taper_floor = float(db.inputs.taperFloor)
    torque_cutoff = float(db.inputs.torqueCutoff)
    weld_max_speed = float(db.inputs.weldMaxSpeed)
    weld_max_ang_speed = float(db.inputs.weldMaxAngSpeed)
    weld_max_misalign = float(db.inputs.weldMaxMisalign)
    break_force = float(db.inputs.breakForce)
    break_torque = float(db.inputs.breakTorque)
    bottom_local_z = float(db.inputs.bottomLocalOffsetZ)   # negative half-height
    wall_prim_path = str(db.inputs.wallStaticPrim)
    # Gripper-open weld gate (2026-09-22, contact-placement rework): weld
    # only when joint7 says the fingers are OPEN. The contact-placement
    # sequence presses the box against the wall with the fingers closed,
    # and during that press the box is linearly AND angularly at rest at
    # d~0.006 -- every existing weld gate passes, so without this gate the
    # box welds mid-hold: the breakable weld then snaps in the arm-vs-weld
    # tug when the gripper retreats, and state.welded latches, never re-
    # welding (physics_tuning.py's WELD_DISTANCE note). joint states come
    # from /piper/joint_states via a ROS2SubscribeJointState node (the same
    # type author_robot_graphs.py uses); both arrays are wired in and joint7
    # is located by name at runtime (robust to the bridge's filtering). If
    # no joint states ever arrive the position input keeps its authored
    # default (1.0, above the threshold = "open") so probes and non-ROS
    # runs weld normally. Threshold: SRDF gripper states are close=0 /
    # open=0.04, measured grasp stall ~0.019 -- 0.03 separates them 2x over.
    joint_names = list(db.inputs.jointNamesObserved)
    joint_positions = list(db.inputs.jointPositions)
    joint7_position = 1.0   # default: open (welding allowed)
    if 'joint7' in joint_names:
        joint7_position = float(joint_positions[joint_names.index('joint7')])
    weld_enable = joint7_position > 0.03

    positions, orientations = state.rigid_prims.get_world_poses()
    linear_vel, angular_vel = state.rigid_prims.get_velocities()
    positions = np.asarray(positions)
    orientations = np.asarray(orientations)
    linear_vel = np.asarray(linear_vel)
    angular_vel = np.asarray(angular_vel)

    dipole_forces = np.zeros((n, 3))    # physical F -- applied at the
                                        # magnet FACE, not the COM (see the
                                        # apply call below, 2026-10-04)
    dipole_torques = np.zeros((n, 3))   # physical tau = m_s x B_w (pure torque)
    stabilizer_forces = np.zeros((n, 3))    # linear drag, at the COM
    stabilizer_torques = np.zeros((n, 3))   # angular damping (pure torque)
    force_points = np.array(positions, dtype=float)  # per-sensor application
                                        # point; default COM, set to the
                                        # magnet face when pressed
    any_active = False

    stage = omni.usd.get_context().get_stage()

    for i in range(n):
        pos = positions[i]
        quat = orientations[i]   # (w, x, y, z)
        bottom_local = np.array([0.0, 0.0, bottom_local_z])
        # The sensor's dipole moment rides its bottom-face outward normal
        # (local -Z) -- see the module docstring for the convention (moment
        # parallel to the wall patch's when correctly mounted -> purely
        # attractive).
        current_normal = _rotate_vec_by_quat_wxyz(
            quat, np.array([0.0, 0.0, -1.0]))
        bottom_world = pos + _rotate_vec_by_quat_wxyz(quat, bottom_local)
        force_points[i] = bottom_world   # the holddown F acts at the magnet
                                         # face, not the COM (apply call
                                         # below, 2026-10-04)
        target = np.array(targets[i], dtype=float)
        r_vec = bottom_world - target   # wall dipole -> sensor dipole
        d = float(np.linalg.norm(r_vec))   # used for the range gate, the
                                           # taper and the weld gate; the
                                           # force itself is APPLIED at the
                                           # magnet face (see the apply call)
        f_mag = 0.0     # actual applied |F| after taper+clamp -- read by the
        tau_mag = 0.0   # heartbeat below; same for |tau|.
        cos_mis = float(np.dot(current_normal, inward))
        misalign_deg = float(np.degrees(np.arccos(np.clip(cos_mis, -1.0, 1.0))))

        # CONTACT-HOLDDOWN model (2026-09-23, user-directed route A):
        # the magnet does NOT exist before the box face touches the wall.
        # No distance-triggered dipole force at all -- the six
        # contact_place batches proved every pre-contact force level
        # fails in this grip/wall system: 100% yanks the 50g box out of
        # the fingers (1.8N > finger friction, box spinning at 2.7 rad/s
        # when the grip opens), 40% repels tilted boxes, 15% crawls at
        # 0.2mm/s. Placement and positioning are the ARM's job (light
        # contact -> position confirm -> deliberate press); the magnet
        # only provides what a real weak magnet on a metal wall provides
        # ONCE TOUCHING: a normal holddown force (the wall's contact
        # normal takes the load; the force keeps contact pressure so
        # friction resists the box's 0.49N shear weight) plus a small
        # aligning torque that seats the face (a real fridge magnet
        # clicks flat; the probes measured 0.0deg). There is no radial
        # capture, no lateral pull to a patch point, no pre-contact
        # anything -- and therefore no oscillation, no yank, no repulsion.
        #
        # INPUT REPURPOSING (same node inputs, new semantics -- avoids an
        # author-script interface change): attractRange = contact
        # threshold (face gap below which the magnet is ON); maxForce =
        # holddown normal force; maxTorque = aligning torque cap;
        # dipoleMoment / taper* / soft* are retired with the dipole law.
        face_gap = float(target[0] - bottom_world[0])   # wall face x minus
                                            # magnet-face-center x; box
                                            # approaches from -x
        d = abs(face_gap)                   # weld gate distance: face-to-
                                            # wall-plane (point patch
                                            # distance retired with the
                                            # dipole era)
        pressed = face_gap < attract_range  # attractRange input = threshold
        f_mag = 0.0
        tau_mag = 0.0
        if pressed:
            dipole_forces[i] = inward * max_force
            f_mag = max_force
            # Aligning torque about the axis that rotates the magnet-face
            # normal onto the wall normal; magnitude capped (finite-size
            # saturation -- the contact finishes the seating).
            align_axis = np.cross(current_normal, inward)
            axis_n = float(np.linalg.norm(align_axis))
            if axis_n > 1e-9:
                dipole_torques[i] = (align_axis / axis_n) * min(
                    max_torque, max_torque * axis_n / 0.5)
                tau_mag = float(np.linalg.norm(dipole_torques[i]))
            # Stabilizer while pressed: linear drag (Bug 2/4 semantics).
            # The gravity feedforward was retired with the weld (see the
            # input read above): on the wall the box's weight is carried
            # by contact friction, not by an explicit upward force.
            stabilizer_forces[i] = (
                -damping_coeff * linear_vel[i]
                + np.array([0.0, 0.0, gravity_feedforward_z]))
            any_active = True

        # Friction-hold watchdog (2026-10-04, weld removal): once a sensor
        # is declared held there is no joint to trust -- the magnet force
        # above IS the hold, and these lines are its honest verifier. Drift
        # beyond SLIP_DRIFT_M from the declared position, or losing wall
        # contact entirely, prints a one-shot (latched) SLIPPED report.
        # The 10mm threshold sits far above normal seat noise (post-hold
        # drift measured at sub-mm across the F-group batches) and far
        # below a box visibly sagging on the wall.
        SLIP_DRIFT_M = 0.01
        if state.welded[i]:
            if state.tick_count % 60 == 0:
                drift = float(np.linalg.norm(pos - state.held_pos[i]))
                lost_contact = not pressed and d > attract_range * 1.5
                if (drift > SLIP_DRIFT_M or lost_contact) \
                        and not state.slip_reported[i]:
                    state.slip_reported[i] = True
                    print(f'### kotek_magnet[tick={state.tick_count}]: '
                          f'SLIPPED sensor {i + 1}: drift={drift:.4f}m '
                          f'd={d:.4f}m pos={pos} (friction hold failed)',
                          flush=True)
                else:
                    print(f'### kotek_magnet[tick={state.tick_count}] '
                          f'sensor{i + 1} HELD: d={d:.4f}m '
                          f'drift={drift:.4f}m '
                          f'speed={float(np.linalg.norm(linear_vel[i])):.4f}',
                          flush=True)
            continue

        # Diagnostic heartbeat for every not-yet-welded sensor, every 60
        # ticks (~2/sec at 120Hz) -- see the print()-vs-log_warning() note
        # above for why this is print(). Sensor 0 alone was enough for the
        # original probe runs (single-sensor capture debugging), but a full
        # 4-cycle E2E needs per-sensor d/speed/f_mag to explain why an
        # individual sensor never passes the weld gate -- e.g. a box that
        # arrives at d just outside weld_distance and hangs on attraction
        # forever looks identical to a welded one in the task logs.
        # misalign_deg and tau_mag are the dipole model's own health
        # signal: alignment torque doing its job shows up as misalignment
        # collapsing while d shrinks, BEFORE the weld fires.
        if not state.welded[i] and state.tick_count % 60 == 0:
            in_range = d < attract_range
            ang_speed = float(np.linalg.norm(angular_vel[i]))
            print(f'### kotek_magnet[tick={state.tick_count}] sensor{i + 1}: '
                  f'd={d:.4f}m in_range={in_range} f_mag={f_mag:.3f}N '
                  f'tau_mag={tau_mag:.4f}Nm misalign_deg={misalign_deg:.1f} '
                  f'pos={pos} bottom_world={bottom_world} quat={quat} '
                  f'speed={float(np.linalg.norm(linear_vel[i])):.4f} '
                  f'ang_speed={ang_speed:.4f} ang_vel_vec={angular_vel[i]} '
                  f'|com_force|={float(np.linalg.norm(stabilizer_forces[i])):.4f}N',
                  flush=True)

        speed = float(np.linalg.norm(linear_vel[i]))
        ang_speed_weld = float(np.linalg.norm(angular_vel[i]))
        # The angular gate is new with the dipole model: real alignment
        # torque means a box can be LINEARLY at rest yet still rotating
        # into alignment -- welding it mid-swing would freeze it crooked
        # by construction (see physics_tuning.py's WELD_MAX_ANG_SPEED
        # note). The old torque-free model never needed this. The
        # MISALIGNMENT gate (2026-09-13, WELD_MAX_MISALIGN_DEG) is the same
        # lesson one level up: even a linearly AND angularly settled box
        # can be resting crooked (toppled, or jammed flat against the wall
        # where the clamped alignment torque has no authority against
        # contact friction) -- measured live: sensor 1 welded at 45.8 deg.
        # A crooked weld is permanent, so the gate refuses it and lets the
        # torque keep working; a box that NEVER aligns is a delivery bug
        # to fix at the source, not to mask with a permissive weld gate.
        # FRICTION-HOLD DECLARATION (was: FixedJoint weld -- REMOVED
        # 2026-10-04 per user decision, see the module docstring). The old
        # gates are kept verbatim, plus one physically necessary addition:
        # d < attract_range. A declared-held box must be INSIDE the
        # magnet's active range, otherwise the holddown force is zero and
        # the declaration is a lie -- the weld era masked the
        # [attract_range, weld_distance) band by holding the box where the
        # magnet force is off. (E2E data shows boxes seat at d=0.0000, so
        # the stricter bound rejects nothing in practice; it closes a real
        # physical hole.) After declaration the magnet force above simply
        # keeps acting every tick -- that IS the hold -- and state.held_pos
        # anchors the slip watchdog.
        if (not state.welded[i] and weld_enable and d < attract_range
                and d < weld_distance and speed < weld_max_speed
                and ang_speed_weld < weld_max_ang_speed
                and misalign_deg < weld_max_misalign):
            state.welded[i] = True
            state.held_pos[i] = pos.copy()
            print(f'### kotek_magnet[tick={state.tick_count}]: '
                  f'held sensor {i + 1} by friction (weld removed) at '
                  f'd={d:.4f}m misalign_deg={misalign_deg:.1f} '
                  f'pos={pos}', flush=True)

    if any_active:
        # TWO calls, split by application point (2026-10-04):
        #   1. holddown F + aligning tau at the MAGNET FACE. A real magnet
        #      pulls at its pole face; the old COM application point loaded
        #      the rigid wall contact eccentrically (COM force + face
        #      reaction = a rocking couple), and the first weld-free batch
        #      measured the artifact: after the task ended, long-idle held
        #      boxes (always the outermost, most-stretched target) snapped
        #      12-18 mm along the wall in a single 1 s heartbeat -- speed
        #      0.0001 m/s, d pinned 0.0000, i.e. a contact-manifold re-
        #      resolution pop, not sliding. With the force at the face the
        #      loading is concentric (a purely normal face force has zero
        #      lever arm while the face is parallel), removing the couple
        #      that drove the penetration asymmetry. Bug 6's "COM-only"
        #      lesson does NOT apply here: that was a force whose DIRECTION
        #      swung with the body (dipole r x F pumping at 120 Hz); this
        #      force is world-fixed +X, torque-free by construction.
        #   2. stabilizers (linear drag) at the COM.
        state.rigid_prims.apply_forces_and_torques_at_pos(
            forces=dipole_forces, torques=dipole_torques,
            positions=force_points, local_frame=False)
        state.rigid_prims.apply_forces_and_torques_at_pos(
            forces=stabilizer_forces, torques=stabilizer_torques,
            positions=positions, local_frame=False)
    return True
'''


def build_magnet_graph(og, graph_path, sensor_paths, wall_targets_flat):
    """One omni.graph.scriptnode.ScriptNode, ticked every frame, holding
    all tunable constants and the fixed (sensor, target) list as its own
    input attributes."""
    import physics_tuning as pt

    keys = og.Controller.Keys
    node_ref = f'{graph_path}/{MAGNET_NODE_NAME}'

    (graph, _nodes, _, _) = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('tick', 'omni.graph.action.OnPlaybackTick'),
                (MAGNET_NODE_NAME, 'omni.graph.scriptnode.ScriptNode'),
            ],
            keys.CREATE_ATTRIBUTES: [
                (f'{MAGNET_NODE_NAME}.inputs:sensorPaths', 'token[]'),
                (f'{MAGNET_NODE_NAME}.inputs:wallTargets', 'double[]'),
                (f'{MAGNET_NODE_NAME}.inputs:wallInwardDir', 'double[]'),
                (f'{MAGNET_NODE_NAME}.inputs:wallStaticPrim', 'token'),
                (f'{MAGNET_NODE_NAME}.inputs:attractRange', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:maxForce', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:dipoleMoment', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:maxTorque', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:dampingCoeff', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:gravityFeedforwardZ', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:angularDampingCoeff', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:weldDistance', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:taperDistance', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:taperFloor', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:torqueCutoff', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxSpeed', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxAngSpeed', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxMisalign', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:breakForce', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:breakTorque', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:bottomLocalOffsetZ', 'double'),
                (f'{MAGNET_NODE_NAME}.inputs:jointNamesObserved', 'token[]'),
                (f'{MAGNET_NODE_NAME}.inputs:jointPositions', 'double[]'),
            ],
            keys.SET_VALUES: [
                # Authored defaults = empty: with no joint states wired in
                # (probes, non-ROS runs) joint7_position falls back to the
                # 1.0 open-default inside compute() and welding works as
                # before.
                (f'{MAGNET_NODE_NAME}.inputs:jointNamesObserved', []),
                (f'{MAGNET_NODE_NAME}.inputs:jointPositions', []),
                (f'{MAGNET_NODE_NAME}.inputs:script', MAGNET_SCRIPT_SOURCE),
                (f'{MAGNET_NODE_NAME}.inputs:sensorPaths', list(sensor_paths)),
                (f'{MAGNET_NODE_NAME}.inputs:wallTargets', list(wall_targets_flat)),
                (f'{MAGNET_NODE_NAME}.inputs:wallInwardDir', list(WALL_INWARD_DIR)),
                (f'{MAGNET_NODE_NAME}.inputs:wallStaticPrim', '/World/wall'),
                (f'{MAGNET_NODE_NAME}.inputs:attractRange', pt.MAGNET_ATTRACT_RANGE),
                (f'{MAGNET_NODE_NAME}.inputs:maxForce', pt.MAGNET_MAX_FORCE),
                (f'{MAGNET_NODE_NAME}.inputs:dipoleMoment', pt.MAGNET_DIPOLE_MOMENT),
                # Clamps |tau| = |m_s x B_w| as a safety bound on the dipole
                # 1/r**3 singularity -- NOT a tuning knob. The old model's
                # maxTorque was the ceiling of a hand-tuned alignment
                # torqueGain that was always set to 0.0 (see physics_tuning.
                # py's Bug-6 history); the alignment torque now falls out of
                # the dipole formula itself and is sized by
                # MAGNET_DIPOLE_MOMENT alone.
                (f'{MAGNET_NODE_NAME}.inputs:maxTorque', pt.MAGNET_MAX_TORQUE),
                (f'{MAGNET_NODE_NAME}.inputs:dampingCoeff', pt.MAGNET_DAMPING_COEFF),
                (f'{MAGNET_NODE_NAME}.inputs:gravityFeedforwardZ', pt.MAGNET_GRAVITY_FEEDFORWARD_Z),
                (f'{MAGNET_NODE_NAME}.inputs:angularDampingCoeff', pt.MAGNET_ANGULAR_DAMPING_COEFF),
                (f'{MAGNET_NODE_NAME}.inputs:weldDistance', pt.WELD_DISTANCE),
                (f'{MAGNET_NODE_NAME}.inputs:taperDistance', pt.MAGNET_TAPER_DISTANCE),
                (f'{MAGNET_NODE_NAME}.inputs:taperFloor', pt.MAGNET_TAPER_FLOOR),
                (f'{MAGNET_NODE_NAME}.inputs:torqueCutoff', pt.MAGNET_TORQUE_CUTOFF),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxSpeed', pt.WELD_MAX_SPEED),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxAngSpeed', pt.WELD_MAX_ANG_SPEED),
                (f'{MAGNET_NODE_NAME}.inputs:weldMaxMisalign', pt.WELD_MAX_MISALIGN_DEG),
                (f'{MAGNET_NODE_NAME}.inputs:breakForce', pt.MAGNET_BREAK_FORCE),
                (f'{MAGNET_NODE_NAME}.inputs:breakTorque', pt.MAGNET_BREAK_TORQUE),
                (f'{MAGNET_NODE_NAME}.inputs:bottomLocalOffsetZ',
                 -pt.SENSOR_CAM_SIZE_XYZ[2] / 2.0),
            ],
        },
    )

    og.Controller.edit(
        graph,
        {keys.CONNECT: [(f'{graph_path}/tick.outputs:tick', f'{node_ref}.inputs:execIn')]},
    )

    # Gripper-open weld gate (2026-09-22): see the compute() comment on
    # db.inputs.joint7Position. ROS2SubscribeJointState -- the same node type
    # author_robot_graphs.py already uses -- filtered to joint7, wired
    # straight into the magnet node. When no publisher is alive the
    # connection never fires and the authored 1.0 default stands (probes /
    # non-ROS runs weld normally).
    og.Controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                ('ros2_context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('joint_states_sub', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
            ],
            keys.SET_VALUES: [
                ('joint_states_sub.inputs:topicName', '/piper/joint_states'),
                ('joint_states_sub.inputs:queueSize', 1),
            ],
            keys.CONNECT: [
                (f'{graph_path}/tick.outputs:tick', 'joint_states_sub.inputs:execIn'),
                ('ros2_context.outputs:context', 'joint_states_sub.inputs:context'),
                ('joint_states_sub.outputs:jointNames', f'{node_ref}.inputs:jointNamesObserved'),
                ('joint_states_sub.outputs:positionCommand', f'{node_ref}.inputs:jointPositions'),
            ],
        },
    )
    return graph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', default=WALL_DEMO_STAGE)
    parser.add_argument('--headless', action='store_true', default=True)
    args = parser.parse_args()

    simulation_app = SimulationApp({'renderer': 'RayTracedLighting', 'headless': args.headless})

    try:
        import omni.graph.core as og
        import omni.usd
        from isaacsim.core.utils import extensions
        from pxr import Sdf

        extensions.enable_extension('isaacsim.ros2.bridge')
        extensions.enable_extension('omni.graph.scriptnode')
        simulation_app.update()
        print('### ros2 bridge + scriptnode extensions enabled', flush=True)

        omni.usd.get_context().open_stage(args.stage)
        simulation_app.update()
        print('### stage opened', flush=True)

        stage = omni.usd.get_context().get_stage()

        import build_wall_stage as ws
        sensor_paths = ws.SENSOR_CAM_PATHS
        for p in sensor_paths:
            if not stage.GetPrimAtPath(Sdf.Path(p)).IsValid():
                raise RuntimeError(
                    f'{p} not found -- run build_wall_stage.py against this stage first')
        wall_targets_flat = []
        for y in ws.WALL_TARGET_Y:
            wall_targets_flat += [ws.WALL_FACE_X, y, ws.WALL_TARGET_Z]

        # Idempotent re-authoring: og.Controller.edit()'s CREATE_NODES step
        # fails outright ("A graph already exists at this path") if this
        # script has already been run once against this stage -- expected
        # when re-running after a physics_tuning.py constant change (e.g.
        # the dipole-moment calibration) or a model rewrite like the
        # 2026-09-07 point-attraction -> dipole-dipole switch. Drop the old
        # graph prim first so this script stays safely re-runnable, same
        # spirit as build_wall_stage.py's --output always writing a fresh
        # stage.
        existing = stage.GetPrimAtPath(Sdf.Path(MAGNET_GRAPH_PATH))
        if existing.IsValid():
            stage.RemovePrim(Sdf.Path(MAGNET_GRAPH_PATH))
            print(f'### removed pre-existing graph at {MAGNET_GRAPH_PATH}', flush=True)

        build_magnet_graph(og, MAGNET_GRAPH_PATH, sensor_paths, wall_targets_flat)
        simulation_app.update()
        print(f'### magnet graph authored: {len(sensor_paths)} sensor/target pairs', flush=True)

        omni.usd.get_context().save_as_stage(args.stage)
        print(f'### saved to {args.stage}', flush=True)
        print('### DONE', flush=True)

    except Exception:
        import traceback
        print('### EXCEPTION', flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close()


if __name__ == '__main__':
    main()
