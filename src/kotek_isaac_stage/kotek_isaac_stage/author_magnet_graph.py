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
"attract, align, then weld with a breakable joint" needs genuine per-tick
Python (a magnetic dipole-dipole force AND torque pair, a one-shot
dynamically-authored UsdPhysics.FixedJoint) that no built-in node provides,
so it is implemented as a single omni.graph.scriptnode.ScriptNode node
instead. The physics is the exact dipole-dipole interaction -- force and
alignment torque both fall out of B = mu0/4pi * [3(m.r̂)r̂ - m]/r^3 and
F/tau formulas, no hand-tuned alignment gain -- see MAGNET_SCRIPT_SOURCE
below and physics_tuning.py's "simulated magnetism" section for the model,
its calibration and the measured bug history of the stabilizing terms it
still uses. The tunable physics constants (dipole moment, attract range,
force/torque caps, weld thresholds, break force/torque) are set as ordinary
node input attributes sourced from physics_tuning.py -- the script reads
them each tick rather than having them baked into its own source string,
so re-authoring after a physics_tuning.py change does not require editing
this file.

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
  * Weld logic: a one-shot breakable UsdPhysics.FixedJoint once d <
    weldDistance, speed < weldMaxSpeed, ang_speed < weldMaxAngSpeed AND
    misalign_deg < weldMaxMisalign -- the 2026-09-13 addition of the last
    gate (physics_tuning.py's WELD_MAX_MISALIGN_DEG) stops a still-toppling
    or wall-jammed box from being frozen crooked by construction.

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
    state.tick_count = 0           # diagnostic heartbeat -- see compute() below


def cleanup(db):
    state = db.per_instance_state
    state.rigid_prims = None
    state.welded = None


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

    positions, orientations = state.rigid_prims.get_world_poses()
    linear_vel, angular_vel = state.rigid_prims.get_velocities()
    positions = np.asarray(positions)
    orientations = np.asarray(orientations)
    linear_vel = np.asarray(linear_vel)
    angular_vel = np.asarray(angular_vel)

    dipole_forces = np.zeros((n, 3))    # physical F, applied at the COM
    dipole_torques = np.zeros((n, 3))   # physical tau = m_s x B_w (pure torque)
    stabilizer_forces = np.zeros((n, 3))    # damping + gravity ff, at the COM
    stabilizer_torques = np.zeros((n, 3))   # angular damping (pure torque)
    any_active = False

    stage = omni.usd.get_context().get_stage()

    for i in range(n):
        if state.welded[i]:
            continue
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
        target = np.array(targets[i], dtype=float)
        r_vec = bottom_world - target   # wall dipole -> sensor dipole
        d = float(np.linalg.norm(r_vec))   # used for the range gate, the
                                           # taper and the weld gate -- the
                                           # force itself is APPLIED at the
                                           # COM (see the docstring)
        f_mag = 0.0     # actual applied |F| after taper+clamp -- read by the
        tau_mag = 0.0   # heartbeat below; same for |tau|.
        cos_mis = float(np.dot(current_normal, inward))
        misalign_deg = float(np.degrees(np.arccos(np.clip(cos_mis, -1.0, 1.0))))

        if d < attract_range and d > 1e-6:
            r_hat = r_vec / d
            m_w = moment * inward
            m_s = moment * current_normal
            m_w_r = float(np.dot(m_w, r_hat))
            m_s_r = float(np.dot(m_s, r_hat))
            m_dot = float(np.dot(m_w, m_s))
            b_field = (MU0_OVER_4PI / d ** 3) * (3.0 * m_w_r * r_hat - m_w)
            f_vec = ((3.0 * MU0_OVER_4PI / d ** 4)
                     * (m_w_r * m_s + m_s_r * m_w + m_dot * r_hat
                        - 5.0 * m_w_r * m_s_r * r_hat))
            tau_vec = np.cross(m_s, b_field)
            # Near-contact taper on the FORCE -- see the docstring: the
            # dipole law is singular at r=0 and invalid in the near field,
            # and the old model's bang-bang near-contact oscillation
            # (physics_tuning.py Bug 4) would be even worse under a
            # 1/r**4 law without it. taperFloor stays decoupled from
            # weldDistance (Bug 8) -- a gate, not the zero-crossing moved.
            taper = 0.0
            if d > taper_floor:
                taper = min(1.0, (d - taper_floor) / max(taper_distance - taper_floor, 1e-9))
            # Separate, EARLIER cutoff for the TORQUE (physics_tuning.py's
            # MAGNET_TORQUE_CUTOFF -- measured): inside ~3cm the discrete
            # 120Hz tau feedback goes unstable at any clamp value tried
            # (0.02 and 0.002 N*m both pinned ang_speed at the velocity
            # cap). This deliberately breaks the F:tau physical ratio in
            # the near field; the wall contact finishes the alignment.
            tau_taper = 0.0
            if d > torque_cutoff:
                tau_taper = min(1.0, (d - torque_cutoff) / max(attract_range - torque_cutoff, 1e-9))
            # Clamp BEFORE the taper, in that order -- this is the old
            # model's semantics (min(GAIN/d**2, MAX) * taper), and the
            # order matters twice as much here: the dipole raw force at
            # 5mm is ~1.7e4 N, so tapering FIRST and clamping AFTER pins
            # |F| at maxForce for the entire near field (measured: box
            # pressed into the wall at a constant 2N, contact solver in a
            # permanent fight, pose frozen while both velocity caps
            # renormalized every tick, never welding). Clamp-first reduces
            # the near-field press to maxForce*taper (~0.1N at 5mm --
            # exactly the old model's proven contact behavior), and the
            # direction physics of the dipole formula is untouched.
            f_n = float(np.linalg.norm(f_vec))
            if f_n > max_force:
                f_vec *= max_force / f_n
                f_n = max_force
            t_n = float(np.linalg.norm(tau_vec))
            if t_n > max_torque:
                tau_vec *= max_torque / t_n
                t_n = max_torque
            f_vec *= taper
            f_mag = f_n * taper
            tau_vec *= tau_taper
            tau_mag = t_n * tau_taper
            dipole_forces[i] = f_vec
            dipole_torques[i] = tau_vec
            # Stabilizers, applied at the COM: linear drag (Bug 2/4 --
            # load-bearing: a constant attractive force with no velocity
            # feedback is bang-bang and cannot converge; freeze-free per
            # Bug 9's live bisect, which retired only the ANGULAR twin)
            # plus the gravity feedforward (Bug 3: the radial term must
            # never be relied on to "accidentally" cover gravity).
            # Deliberately NOT tapered -- gravity and momentum do not turn
            # themselves off near contact.
            stabilizer_forces[i] = (
                -damping_coeff * linear_vel[i]
                + np.array([0.0, 0.0, gravity_feedforward_z]))
            stabilizer_torques[i] = -angular_damping_coeff * angular_vel[i]
            any_active = True

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
        if (d < weld_distance and speed < weld_max_speed
                and ang_speed_weld < weld_max_ang_speed
                and misalign_deg < weld_max_misalign):
            joint_path = f'/World/wall/_magnet_weld_{i + 1}'
            if not stage.GetPrimAtPath(joint_path).IsValid():
                wall_prim = stage.GetPrimAtPath(wall_prim_path)
                wall_pos = wall_prim.GetAttribute('xformOp:translate').Get()
                wall_pos = np.array([wall_pos[0], wall_pos[1], wall_pos[2]])
                joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
                joint.CreateBody0Rel().SetTargets([Sdf.Path(sensor_paths[i])])
                joint.CreateBody1Rel().SetTargets([Sdf.Path(wall_prim_path)])
                joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                local_pos1 = pos - wall_pos   # wall carries no rotation, so this
                                              # is already in its local frame
                joint.CreateLocalPos1Attr().Set(
                    Gf.Vec3f(float(local_pos1[0]), float(local_pos1[1]), float(local_pos1[2])))
                joint.CreateLocalRot1Attr().Set(
                    Gf.Quatf(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
                joint.CreateBreakForceAttr().Set(break_force)
                joint.CreateBreakTorqueAttr().Set(break_torque)
                state.welded[i] = True
                print(f'### kotek_magnet[tick={state.tick_count}]: '
                      f'welded sensor {i + 1} at d={d:.4f}m '
                      f'misalign_deg={misalign_deg:.1f} '
                      f'pos={pos} wall_pos={wall_pos} local_pos1={local_pos1} '
                      f'quat={quat}', flush=True)

    if any_active:
        # ONE call, everything at the COM: the physical dipole F, the
        # stabilizers (linear drag, gravity feedforward), and all pure
        # torques (physical tau = m_s x B_w, angular damping). The dipole
        # force is NOT applied at the bottom-face dipole point even though
        # continuum mechanics would put it there -- see the docstring's
        # measured r x F pumping note (physics_tuning.py Bug 6's lesson:
        # in this integrator only the COM is a safe application point).
        state.rigid_prims.apply_forces_and_torques_at_pos(
            forces=dipole_forces + stabilizer_forces,
            torques=dipole_torques + stabilizer_torques,
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
            ],
            keys.SET_VALUES: [
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
