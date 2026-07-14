"""Constrained planning examples.

Plans with manifold constraints by passing `constraints=[...]` to the regular planner
entry points (rrtc, aorrtc, grrtstar) and simplify:

- line: the Panda's end-effector may only translate along its approach axis.
- plane: the Panda's end-effector slides in a fixed-orientation plane through a sphere cage.
- bimanual: the two arms of the bimanual Panda hold a fixed relative grasp transform.
- screw: the Panda's end-effector advances along its approach axis locked to rotation
  about it (lead-screw coupling), threading the arm past sphere obstacles. The coupling
  row is Pfaffian -- it restricts velocities, not positions -- so this mode requires
  --flask; the reported drift of the screw invariant h measures how well the chart
  machinery holds the coupling. The twist-based row is smooth for any rotation
  (--turn sets the total, default 5 rad); --holonomic swaps it for its integrable
  level constraint as an oracle, which uses the log map and so needs --turn below pi.
- knife: the Panda drags a knife edge across a virtual board: the blade may advance
  along its edge and yaw about the board normal, but its lateral (body-y) velocity is
  pinned to zero -- the classic nonintegrable unicycle row, built from the generic
  TwistConstraint with runtime coefficients and no dedicated kernel. The goal is
  laterally offset from the start at the same heading, a displacement that is
  forbidden pointwise, so the plan must steer (yaw, cut, yaw back); the reported
  net slip measures how well the chart machinery holds the row (a broken row
  would show the full offset as slip, and without the ChartSettings slip tolerance
  shortcutting realizes ~40% of it by slip). Nonintegrable, so no holonomic
  oracle exists.

Start and goal must lie on the constraint manifold: the planners raise ValueError
otherwise, so this script projects them first with the module's project() helper.

With --flask, the examples plan kinodynamically instead: the chart-LQMT planner on
the robot's flask submodule steers rest-to-rest (q, qdot) states along time-optimal
cubics on the constraint manifold. Constraints are still built from the ambient
submodule; the executed trajectory is reconstructed with lift_edge.

With --retime, the examples plan geometrically as usual and then retime the path on
the flask sibling: waypoints are lifted to rest states and the flask simplify's
C_loc shortcutting (validated chart-LQMT edges on the manifold) blends the
stop-at-every-waypoint lift into flowing motion.
"""

from functools import partial
from pathlib import Path
import time

import numpy as np
import vamp
from vamp import flask as vf
from vamp import transformations as tr
from fire import Fire

# Sphere cage for the plane example (from the original constrained-planning demos).
PLANE_PROBLEM = [
    [0.56, 0, 0.450],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Allowed end-effector travel along the line constraint's axis, in meters.
LINE_EXTENT = 0.6

# Reference pose (qw, qx, qy, qz, x, y, z) of the plane constraint: the end-effector
# slides in the local x-y plane of this frame with fixed orientation.
PLANE_POSE = [0.0, 0.707107, 0.0, 0.707107, 0.354, 0.7, 0.243]

# Lead-screw coupling for the screw example: advance per full turn (m). The Pfaffian
# twist row is smooth for any rotation; only the holonomic level constraint (seed pinning
# and --holonomic) uses the log map, which wraps at a half turn from its anchor. The
# default 5 rad turn advances 0.279 m; much beyond ~0.32 m the arm runs out of reach
# along the fixed approach axis and goal projection stalls short.
SCREW_PITCH = 0.35

# Sphere obstacle for the screw example (position, radius): hovers just above where
# the wrist ends up late in the default 5 rad turn on the start's (negative) swivel
# side. Grid-probing the manifold's (phase, swivel) free set shows it blocks the
# swivel crossing for phases past about half the default turn while leaving the exact
# h = 0 manifold connected: the elbow must cross to the goal side early, then finish
# the advance there. At --turn 2 the wrist never rises into it, so the holonomic
# oracle plans the same scene. (An obstacle on the unswiveled elbow corridor instead
# blocks all swivels at every phase -- the arm barely descends -- making h = 0
# infeasible; the Pfaffian planner then "solves" it only by drifting h across leaves.)
SCREW_OBSTACLES = [([-0.01, 0.07, 0.84], 0.035)]

# Elbow-swivel bias (rad, applied to the base joint) separating the screw seeds:
# start on the obstacle's (negative) side, goal on the free (positive) side.
SCREW_SWIVEL = 1.2

# Blade pose anchor for the knife example: the ready pose, approach axis (body z)
# pointing down at the virtual board; the blade edge lies along body x.
KNIFE_NOMINAL = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

# Lateral (body-y) displacement of the knife goal, in meters: entirely along the
# forbidden direction, so it can only be realized by steering.
KNIFE_OFFSET = 0.2

# Playback rate for geometric (non-flask) paths, which carry no timing.
PLAYBACK_FPS = 30.0


def lift_path(module, path, constraints, constraint_settings, chart_settings):
    """Lift every edge of a z-path through the chart machinery; per edge pick the
    direction whose reconstructed endpoint matches (goal-tree edges lift backward and
    are executed in reverse). The reconstruction shoots toward the recorded endpoint
    and stops once attained, so misses up to ~reached_pos_tol are inherent; beyond 2x
    the edge is considered unreconstructible. Returns (dense states, per-state times,
    max constraint error, duration, per-edge endpoint misses, per-edge sample counts)."""
    n_q = len(path[0]) // 2
    dense = []
    times = []
    max_err = 0.0
    total_T = 0.0
    misses = []
    counts = []
    for i in range(len(path) - 1):
        a, b = np.asarray(path[i]), np.asarray(path[i + 1])
        best = None
        for frm, tgt, fwd, endpoint in ((a, b, True, b), (b, a, False, a)):
            try:
                states, errs, T, _ = module.lift_edge(
                    frm, tgt, constraints, forward=fwd, n_samples=32,
                    constraint_settings=constraint_settings, chart_settings=chart_settings)
            except ValueError:
                continue
            miss = float(np.linalg.norm(np.asarray(states[-1])[:n_q] - endpoint[:n_q]))
            if best is None or miss < best[0]:
                best = (miss, states, errs, T, fwd)
        if best is None:
            raise RuntimeError(f"edge {i} of the path could not be lifted")
        miss, states, errs, T, fwd = best
        if miss > 2.0 * chart_settings.reached_pos_tol:
            raise RuntimeError(f"lifted edge {i} misses its endpoint by {miss:.2e}")
        dense.extend(np.asarray(s) for s in (states if fwd else states[::-1]))
        # lift_edge samples at uniform times in [0, T], so timestamps are recoverable.
        times.extend(total_T + np.linspace(0.0, float(T), len(states)))
        max_err = max(max_err, float(max(errs)))
        total_T += float(T)
        misses.append(miss)
        counts.append(len(states))
    return np.array(dense, dtype=np.float32), np.array(times), max_err, total_T, misses, counts


def heal_path(module, path, environment, constraints, constraint_settings, chart_settings):
    """Close interior connection gaps by chaining executed arrivals in execution order.
    A forward-replayable edge is lifted from the previous edge's (chained) executed
    arrival toward the ORIGINAL stored waypoint and the arrival -- on-manifold
    position, tangent-projected velocity -- adopted as the new waypoint: replaying it
    shoots at its own arrival (an exactly attainable chart point), so that junction's
    seam closes to numerical noise, and re-targeting the originals keeps every
    per-edge miss at O(reached_pos_tol) rather than accumulating. A backward-only
    edge (goal-tree orientation: its arc chains exactly at its EXIT waypoint and
    shoots at its entry) cannot chain forward; instead its backward arc is verified
    shooting at the true chained junction state, which collapses the two stacked
    misses of opposite-orientation replay into the single shooting miss, and its exit
    waypoint is kept exactly. The goal waypoint is never replaced: a fixed-chart arc
    cannot land across a Pfaffian row normal, so a final miss remains and lift_path
    reports it honestly (a backward goal edge instead lands exactly and carries the
    miss at its entry). An accepted arc must arrive within 2x reached_pos_tol,
    on-manifold, and collision-free at all samples; failing both directions, the edge
    reverts to its original endpoints. Returns (healed waypoints, arrivals adopted,
    backward edges verified, skip reasons)."""
    n_q = len(path[0]) // 2
    tol = 2.0 * chart_settings.reached_pos_tol
    original = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]

    def lift(frm, tgt, forward):
        states, _, _, _ = module.lift_edge(
            frm, tgt, constraints, forward=forward, n_samples=32,
            constraint_settings=constraint_settings, chart_settings=chart_settings)
        # The last emitted state is the shooting arrival near the lift's target,
        # regardless of direction.
        arrival = np.asarray(states[-1], dtype=np.float32)
        miss = float(np.linalg.norm(arrival[:n_q] - np.asarray(tgt)[:n_q]))
        if miss > tol:
            return None, f"miss {miss:.2e} > 2x tol"
        # Collision depends on position only; validating a full flask state sweeps a
        # spurious to-itself LQMT loop from its velocity.
        pos = np.array(states, dtype=np.float32)
        pos[:, n_q:] = 0.0
        if not all(module.validate(p, environment) for p in pos):
            return None, "arc in collision"
        return arrival, None

    healed = [original[0]]
    n_forward = n_backward = 0
    reasons = []
    for i in range(len(path) - 1):
        tgt = original[i + 1]
        last = i + 1 == len(path) - 1
        try:
            arrival, why_fwd = lift(healed[-1], tgt, forward=True)
            if arrival is not None and not module.satisfied(
                    arrival, constraints, constraint_settings):
                arrival, why_fwd = None, "arrival off manifold"
        except ValueError as exc:
            arrival, why_fwd = None, f"lift failed ({exc})"
        if arrival is not None:
            healed.append(tgt if last else arrival)
            n_forward += 0 if last else 1
            continue
        # Backward arc: lift from the stored exit waypoint, shooting at the chained
        # junction state (states[-1] is the entry-side arrival in execution order).
        try:
            entry, why_bwd = lift(tgt, healed[-1], forward=False)
        except ValueError as exc:
            entry, why_bwd = None, f"lift failed ({exc})"
        if entry is not None:
            healed.append(tgt)
            n_backward += 1
            continue
        # Both directions failed: revert this edge's start too, so the edge is the
        # original (known replayable) one; that junction keeps its original seam.
        healed[-1] = original[i]
        healed.append(tgt)
        reasons.append(f"edge {i}: forward {why_fwd}; backward {why_bwd}")
    return healed, n_forward, n_backward, reasons


def pose_to_transform(pose):
    """4x4 matrix -> (qw, qx, qy, qz, x, y, z)."""
    pose = np.asarray(pose)
    x, y, z, w = tr.quaternion_from_matrix(pose)
    return [w, x, y, z, *pose[:3, 3]]


def screw_invariant_series(module, reference_inv, qs):
    """Screw invariant h = [t]_z - (pitch / 2 pi) theta of the end-effector pose in the
    reference frame along a densely-sampled trajectory, computed independently of the
    generated kernels. The rotation angle theta about the reference z-axis is
    accumulated with np.unwrap, so multi-turn screws (beyond the log map's half-turn
    wrap) report correctly. Returns (h, theta)."""
    z, theta = [], []
    for q in qs:
        rTe = reference_inv @ np.array(module.eefk(q))
        rot = rTe[:3, :3]
        theta.append(np.arctan2(rot[1, 0] - rot[0, 1], rot[0, 0] + rot[1, 1]))
        z.append(rTe[2, 3])
    theta = np.unwrap(theta)
    return np.asarray(z) - SCREW_PITCH / (2.0 * np.pi) * theta, theta


def screw_transform(theta):
    """Displacement of the lead screw at rotation theta: advance (pitch / 2 pi) theta
    along z while rotating theta about it."""
    t = tr.rotation_matrix(theta, [0.0, 0.0, 1.0])
    t[2, 3] = SCREW_PITCH / (2.0 * np.pi) * theta
    return t


def screw_nominal(turn):
    """Unswiveled configuration whose end-effector pose anchors the screw: the ready
    pose with the wrist roll (aligned with the approach axis) at +turn/2, so the turn
    splits symmetrically between the roll's joint limits (+-2.8973)."""
    return np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, turn / 2.0], dtype=np.float32)


def screw_problem(module, turn):
    # The end-effector rotates by `turn` about the approach (local z) axis of the
    # nominal pose while advancing along it, coupled as a lead screw: SCREW_PITCH
    # meters per full turn. The TSR pins the off-axis rows; the coupling row is
    # Pfaffian. The seeds are elbow-swiveled to opposite sides; the obstacle blocks
    # the swivel crossing late in the turn, so the plan must swing the elbow to the
    # goal side early and finish the advance there.
    nominal = screw_nominal(turn)
    start_seed = nominal.copy()
    start_seed[0] -= SCREW_SWIVEL
    goal_seed = nominal.copy()
    goal_seed[0] += SCREW_SWIVEL
    goal_seed[6] -= turn

    reference = np.array(module.eefk(nominal))

    # The TSR is anchored mid-screw so its z-translation row brackets the whole advance
    # symmetrically; the row must not be flagged tight (bound width < 0.5) or its
    # gradient joins the chart basis permanently and freezes the screw's only DOF. The
    # rz bounds are never active (|log3_z| <= pi always): rotation is the coupling
    # row's job, not the TSR's.
    advance = SCREW_PITCH * turn / (2.0 * np.pi)
    assert advance / 2.0 < 0.3
    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        pose_to_transform(reference @ screw_transform(-turn / 2.0)),
        [-0.01, -0.01, -0.3, -0.01, -0.01, -3.2],
        [0.01, 0.01, 0.3, 0.01, 0.01, 3.2],
    )

    screw = module.LeadScrewConstraint(IDENTITY, pose_to_transform(reference), SCREW_PITCH)

    e = vamp.Environment()
    for position, radius in SCREW_OBSTACLES:
        e.add_sphere(vamp.Sphere(position, radius))

    return start_seed, goal_seed, [tsr, screw], e


def knife_problem(module):
    # The blade's contact frame may advance along its edge (body x) and yaw about the
    # board normal, but its lateral body-y velocity is zero: a unicycle row, expressed
    # directly through the generic TwistConstraint's body-frame coefficients. The TSR
    # holds the blade at board height and vertical (z, rx, ry tight) while x, y, and
    # heading stay free (wide slab bounds, width >= 0.5 so they never join the chart basis).
    # Both seeds are the nominal config; main() pins the goal seed to the laterally
    # offset pose with a narrow TSR before planning.
    reference = np.array(module.eefk(KNIFE_NOMINAL))

    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        pose_to_transform(reference),
        [-0.3, -0.3, -0.01, -0.01, -0.01, -3.2],
        [0.3, 0.3, 0.01, 0.01, 0.01, 3.2],
    )

    knife = module.TwistConstraint(
        IDENTITY, pose_to_transform(reference),
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    )

    return KNIFE_NOMINAL.copy(), KNIFE_NOMINAL.copy(), [tsr, knife], vamp.Environment()


def line_problem(module):
    # The end-effector may only translate along the approach (local z) axis of its
    # starting pose; position off-axis and orientation are held to +- 0.01.
    start_seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
    goal_seed = start_seed + np.array([0.0, 0.35, 0.0, 0.45, 0.0, -0.4, 0.0], dtype=np.float32)

    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        pose_to_transform(module.eefk(start_seed)),
        [-0.01, -0.01, -LINE_EXTENT, -0.01, -0.01, -0.01],
        [0.01, 0.01, LINE_EXTENT, 0.01, 0.01, 0.01],
    )

    return start_seed, goal_seed, [tsr], vamp.Environment()


def plane_problem(module):
    # The end-effector slides in the x-y plane of a fixed reference frame (free in two
    # translation axes) with fixed orientation, through a cage of spheres.
    start_seed = np.array([-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761], dtype=np.float32)
    goal_seed = np.array([-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034], dtype=np.float32)

    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        PLANE_POSE,
        [-10.01, -10.01, -0.01, -0.01, -0.01, -0.01],
        [10.01, 10.01, 0.01, 0.01, 0.01, 0.01],
    )

    e = vamp.Environment()
    for sphere in PLANE_PROBLEM:
        e.add_sphere(vamp.Sphere(sphere, 0.15))

    return start_seed, goal_seed, [tsr], e


# Shelf for the bimanual example: two boards, a center divider, and the ground
# (center, full extents).
BIMANUAL_SHELF = [
    ([0.5, 0.0, 0.2], [0.3, 3.0, 0.014]),
    ([0.5, 0.0, 0.4], [0.3, 3.0, 0.014]),
    ([0.5, 0.0, 0.3], [0.3, 0.01, 0.15]),
    ([0.0, 0.0, -0.2], [5.0, 5.0, 0.2]),
]


def bimanual_problem(module):
    # The right end-effector holds a fixed transform relative to the left, as if both
    # arms grasp one rigid object.
    # Both seeds keep >5cm clearance from the shelf: the object starts held low in
    # front of the shelf and ends held above the top board.
    start_seed = np.array(
        [-1.388458, 1.789655, 0.526891, -2.779171, -0.986079, 3.079894, -0.75567,
         1.630401, 0.982874, -0.542026, -2.682339, -0.376891, 2.048735, 0.422199],
        dtype=np.float32,
    )
    goal_seed = np.array(
        [-2.118829, 0.419675, 2.249477, -2.045575, 1.324726, 1.762167, -0.726496,
         1.423746, 1.290487, -2.148522, -2.168713, -0.143205, 2.446808, -1.55776],
        dtype=np.float32,
    )

    relative = module.BimanualTaskSpaceConstraint(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.221814],
        [-0.001] * 6,
        [0.001] * 6,
    )

    e = vamp.Environment()
    for center, extents in BIMANUAL_SHELF:
        e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], [x / 2.0 for x in extents]))

    return start_seed, goal_seed, [relative], e


PROBLEMS = {
    "line": ("panda", line_problem, "panda_spherized.urdf"),
    "plane": ("panda", plane_problem, "panda_spherized.urdf"),
    "bimanual": ("bimanual_panda", bimanual_problem, "bipanda_spherized.urdf"),
    "screw": ("panda", screw_problem, "panda_spherized.urdf"),
    "knife": ("panda", knife_problem, "panda_spherized.urdf"),
}


def main(
    mode: str = "plane",  # One of line, plane, bimanual, screw, knife.
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float | None = None,  # Planner range; constrained steps should stay small.
    flask: bool = False,  # Plan kinodynamically on the manifold with the chart-LQMT planner.
    retime: bool = False,  # Plan geometrically, then retime on the flask sibling.
    rho: float | None = None,  # LQMT time weight in C = rho * T + int |u|^2 (flask/retime).
    max_kinetic_energy: float | None = None,  # Phase constraint: kinetic energy cap in J (flask/retime).
    max_eef_speed: float | None = None,  # Phase constraint: end-effector speed cap in m/s (flask/retime).
    sample_energy: float | None = None,  # Shape sampled kinetic energy to [0, cap] J (flask only).
    heal: bool = False,  # Close interior connection gaps by chaining executed arrivals (flask/retime).
    holonomic: bool = False,  # Swap the screw's Pfaffian row for its holonomic representation (screw only).
    turn: float = 5.0,  # Total screw rotation in rad (screw only).
    visualize: bool = False,
    **kwargs,
):
    robot_name, problem, urdf = PROBLEMS[mode]
    if range_ is None:
        # An exact connect's arrival misses its target by ~ arc length x Pfaffian row
        # rotation (a frozen-chart arc cannot land across the row normal), so the
        # knife's rapidly turning row needs shorter arcs for connects to attain the
        # tightened reached_pos_tol below.
        range_ = 0.1 if mode == "knife" else 0.5
    if mode == "screw":
        problem = partial(screw_problem, turn=turn)
    if flask and retime:
        raise ValueError("--flask already plans kinodynamically; --retime retimes a geometric plan")
    kinodynamic = flask or retime
    if rho is not None and not kinodynamic:
        raise ValueError("--rho only applies to the chart-LQMT machinery; pass --flask or --retime")
    if (max_kinetic_energy is not None or max_eef_speed is not None) and not kinodynamic:
        raise ValueError("phase constraints only apply to the chart-LQMT machinery; pass --flask or --retime")
    if sample_energy is not None and not flask:
        raise ValueError("--sample_energy only applies to the chart-LQMT planner; pass --flask")
    if heal and not kinodynamic:
        raise ValueError("--heal only applies to the chart-LQMT machinery; pass --flask or --retime")
    if mode in ("screw", "knife") and not flask:
        raise ValueError(
            f"{mode} mode requires --flask: the Pfaffian coupling row restricts velocities "
            "only, which the geometric projection-based pipeline ignores")
    if holonomic and mode != "screw":
        raise ValueError("--holonomic only applies to screw mode")
    if holonomic and turn + 0.4 >= np.pi:
        raise ValueError(
            "--holonomic plans on the log-map level constraint, which wraps at a half turn "
            "from its anchor; pass --turn below pi - 0.4 (the TSR's rz slack)")
    if flask:
        kwargs.setdefault("max_iterations", 50000)
        kwargs.setdefault("max_samples", 50000)
        robot_name = f"{robot_name}.flask"
        if mode == "bimanual" and max_kinetic_energy is None:
            # Unbounded sampled velocities make LQMT steer arcs sweep wide and clip
            # the shelf boards; a kinetic-energy gate rest-biases steers enough to
            # thread the below->above passage (unsolvable without it).
            max_kinetic_energy = 1.0

    (module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot_name, planner, **kwargs)

    # The module that owns the final path: with --retime the flask sibling (whose simplify
    # retimes the geometric plan); otherwise the planning module itself.
    path_module, flask_simp_settings = module, simp_settings
    if retime:
        (path_module, _, _, flask_simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
            f"{robot_name}.flask", planner, **kwargs)
        flask_simp_settings.operations = vf.RETIME_OPERATIONS
    if kinodynamic and rho is not None:
        path_module.set_rho(rho)

    if planner == "rrtc" or planner == "grrtstar":
        plan_settings.range = range_
    elif planner == "aorrtc":
        plan_settings.rrtc.range = range_

    if flask:
        # Flat z-space sample distances dwarf the geometric dynamic-domain radius
        # default (4.0), which starves the trees after the first trapped chart steer.
        if planner == "rrtc":
            plan_settings.radius = 8.0
        elif planner == "grrtstar":
            plan_settings.dd_radius = 8.0
        elif planner == "aorrtc":
            plan_settings.rrtc.radius = 8.0

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = 50

    # Constraints always come from the ambient (geometric) robot's submodule; a flask
    # robot is a nested submodule of its ambient parent.
    ambient = getattr(vamp, robot_name.partition(".")[0])
    start_seed, goal_seed, constraints, e = problem(ambient)
    n_q = len(start_seed)

    # The Pfaffian screw row restricts directions but cannot pin which leaf of the
    # foliation a state is on, so each seed is projected with a holonomic level constraint
    # pinning the invariant h = 0 at an anchor within a half turn of that seed (the
    # log-map kernel wraps beyond that): the start constraint at the start pose, the goal
    # constraint a full `turn` down the screw. The level constraint pins the helix but not the
    # phase along it -- projection can slide the wrist back while it advances the arm
    # -- so each seed set also carries a seed-anchored TSR whose narrow rz rows pin
    # the phase (its log map sees that seed at rotation ~0, so no wrap).
    start_constraints = goal_constraints = constraints
    seed_settings = constraint_settings
    if mode == "screw":
        screw_reference = np.array(ambient.eefk(screw_nominal(turn)))

        def level_at(theta):
            anchor = screw_reference @ screw_transform(theta)
            return ambient.LeadScrewLevelConstraint(
                IDENTITY, pose_to_transform(anchor), SCREW_PITCH, 0.0)

        def pin_tsr_at(theta):
            return ambient.TaskSpaceConstraint(
                IDENTITY,
                pose_to_transform(screw_reference @ screw_transform(theta)),
                [-0.01, -0.01, -0.3, -0.01, -0.01, -0.01],
                [0.01, 0.01, 0.3, 0.01, 0.01, 0.01],
            )

        start_constraints = [pin_tsr_at(0.0), level_at(0.0)]
        goal_constraints = [pin_tsr_at(-turn), level_at(-turn)]
        # Alternating projection between the narrow TSR rows and the level row
        # converges slowly (each drags the other off slightly), and the goal
        # projection must pull the arm through the screw's whole advance; give the
        # one-off seed projections a much bigger budget than the planner's
        # per-sample one.
        seed_settings = vamp.ConstraintSettings()
        seed_settings.max_iterations = 500
        if holonomic:
            # The level constraint's log map wraps at a half turn from its start anchor, so
            # unlike the Pfaffian row it cannot leave the TSR's rz rows fully free:
            # samples rotated past pi see a spurious pitch-shifted helix. Cap rz so
            # all reachable rotations stay within the wrap radius.
            capped_tsr = ambient.TaskSpaceConstraint(
                IDENTITY,
                pose_to_transform(screw_reference @ screw_transform(-turn / 2.0)),
                [-0.01, -0.01, -0.3, -0.01, -0.01, -(turn / 2.0 + 0.4)],
                [0.01, 0.01, 0.3, 0.01, 0.01, turn / 2.0 + 0.4],
            )
            constraints = [capped_tsr, level_at(0.0)]
    elif mode == "knife":
        # The unicycle row restricts directions only; positions are pinned by narrow
        # seed TSRs: the start at the nominal blade pose, the goal displaced purely
        # along the blade's lateral (body-y) axis at the same heading.
        knife_reference = np.array(ambient.eefk(KNIFE_NOMINAL))
        goal_pose = knife_reference.copy()
        goal_pose[:3, 3] += knife_reference[:3, :3] @ [0.0, KNIFE_OFFSET, 0.0]

        def pin_tsr(pose):
            return ambient.TaskSpaceConstraint(
                IDENTITY, pose_to_transform(pose), [-0.01] * 6, [0.01] * 6)

        start_constraints = [pin_tsr(knife_reference)]
        goal_constraints = [pin_tsr(goal_pose)]
        seed_settings = vamp.ConstraintSettings()
        seed_settings.max_iterations = 500

    chart_kwargs = {}
    if kinodynamic:
        chart_settings = vamp.ChartSettings()
        if mode in ("screw", "knife"):
            chart_settings.reached_pos_tol = 0.05
        chart_kwargs["chart_settings"] = chart_settings
        gates = vf.phase_constraints(path_module, max_kinetic_energy, max_eef_speed)
        if gates:
            chart_kwargs["phase_constraints"] = gates

    # The geometric pipeline in retime mode takes no chart kwargs; only flask planning
    # and the retiming simplify do.
    plan_chart_kwargs = chart_kwargs if flask else {}

    if flask:
        start_seed, goal_seed = vf.rest_state(start_seed), vf.rest_state(goal_seed)

    # Strict start/goal policy: seeds must be projected onto the manifold explicitly.
    start = module.project(start_seed, start_constraints, seed_settings, **plan_chart_kwargs)
    goal = module.project(goal_seed, goal_constraints, seed_settings, **plan_chart_kwargs)

    if mode in ("screw", "knife"):
        # The seeds were projected in seed-anchored frames whose TSR bounds differ
        # slightly from the planning TSR's ones; polish them onto the planning set
        # (whose rows barely move them, so the pinned poses survive).
        start = module.project(start, constraints, seed_settings, **plan_chart_kwargs)
        goal = module.project(goal, constraints, seed_settings, **plan_chart_kwargs)

    for name, q in (("start", start), ("goal", goal)):
        if not module.validate(q, e):
            raise RuntimeError(f"projected {name} configuration is in collision")

    sampler = module.halton()
    if sample_energy is not None:
        sampler = module.ke_shaped(sampler, float(sample_energy))
    elapsed = time.perf_counter()
    result = planner_func(
        start, goal, e, plan_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings, **plan_chart_kwargs)
    elapsed = time.perf_counter() - elapsed

    if not result.solved:
        raise RuntimeError("planning failed")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings, **plan_chart_kwargs)

    path = simple.path

    tag = " (flask)" if flask else " (retime)" if retime else ""
    print(f"{mode} with {planner}{tag}: solved in {elapsed * 1e3:.1f} ms")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")

    if retime:
        # Retime on the flask sibling: lift the geometric waypoints to rest states and
        # let the flask simplify's C_loc shortcutting blend the stops into flowing
        # motion. Shortcut edges are chart-LQMT cubics validated on the manifold.
        lifted = vf.lift(path_module, path)
        rest_cost = lifted.cost()
        retime_elapsed = time.perf_counter()
        flask_simple = path_module.simplify(
            lifted, e, flask_simp_settings, path_module.halton(),
            constraints=constraints, constraint_settings=constraint_settings, **chart_kwargs)
        retime_elapsed = time.perf_counter() - retime_elapsed

        path = flask_simple.path
        print(f"retime: {len(lifted)} -> {len(path)} states, "
              f"cost {rest_cost:.4f} -> {path.cost():.4f}, in {retime_elapsed * 1e3:.1f} ms")

    if heal:
        cs = chart_kwargs["chart_settings"]
        _, _, _, _, misses_before, _ = lift_path(
            path_module, path, constraints, constraint_settings, cs)
        heal_elapsed = time.perf_counter()
        path, n_forward, n_backward, reasons = heal_path(
            path_module, path, e, constraints, constraint_settings, cs)
        heal_elapsed = time.perf_counter() - heal_elapsed
        print(f"heal: {n_forward} arrivals chained, {n_backward} backward edges "
              f"re-shot at the chained junction, {len(reasons)} reverted, in "
              f"{heal_elapsed * 1e3:.1f} ms")
        print("  per-edge miss before: " + " ".join(f"{m:.1e}" for m in misses_before))
        for reason in reasons:
            print(f"  heal reverted -- {reason}")

    on_manifold = all(
        path_module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))
    print(f"all waypoints on manifold: {on_manifold}")

    dense = dense_t = None
    if kinodynamic:
        dense, dense_t, max_err, total_T, misses, counts = lift_path(
            path_module, path, constraints, constraint_settings, chart_kwargs["chart_settings"])
        vmax = np.abs(dense[:, n_q:]).max(axis=0)
        vel_ratio = float((vmax / np.array(path_module.velocity_limits())).max())
        print(f"trajectory: {total_T:.2f} s over {len(path) - 1} segments, "
              f"max constraint dist^2 {max_err ** 2:.2e}, max velocity ratio {vel_ratio:.2f}, "
              f"max endpoint miss {max(misses):.2e}")
        if heal:
            print("  per-edge miss after:  " + " ".join(f"{m:.1e}" for m in misses))
        if mode == "screw":
            # Drift of the screw invariant along the executed trajectory: zero up to
            # per-sample tangent projection and chart tolerance for the Pfaffian row,
            # and up to projection tolerance for the holonomic representation.
            reference_inv = np.linalg.inv(screw_reference)
            h, theta = screw_invariant_series(ambient, reference_inv, dense[:, :n_q])
            kind = "holonomic level row" if holonomic else "Pfaffian row"
            print(f"screw invariant ({kind}): h start {h[0]:+.2e} m, "
                  f"max drift {np.abs(h - h[0]).max():.2e} m, "
                  f"rotation {theta[-1]:+.2f} rad (target {-turn:+.2f})")
        if mode == "knife":
            # Blade-frame lateral motion along the executed trajectory. The goal
            # displacement is purely lateral, so the net (signed) slip is displacement
            # stolen along the forbidden direction: without the chart machinery's slip
            # gate, shortcutting realizes ~40% of the offset by same-signed slip
            # instead of steering; with it, net slip sits at integration-noise level
            # and the offset is earned by the heading swing. The unsigned total is
            # first-order arc wobble that largely cancels.
            poses = [np.array(ambient.eefk(q)) for q in dense[:, :n_q]]
            # Sample pairs that straddle an edge junction are seam teleports (never
            # executed, invisible to the slip tolerance); the rest is executed motion.
            junctions = set(np.cumsum(counts[:-1]))
            slip = net = travel = seam_net = 0.0
            for j, (a, b) in enumerate(zip(poses, poses[1:])):
                d_body = a[:3, :3].T @ (b[:3, 3] - a[:3, 3])
                slip += abs(float(d_body[1]))
                net += float(d_body[1])
                travel += float(np.linalg.norm(d_body[:2]))
                if j + 1 in junctions:
                    seam_net += float(d_body[1])
            heading = np.unwrap([np.arctan2(p[1, 0], p[0, 0]) for p in poses])
            rel_end = np.linalg.inv(knife_reference) @ poses[-1]
            print(f"knife edge (Pfaffian row): lateral offset {rel_end[1, 3]:+.3f} m "
                  f"(target {KNIFE_OFFSET:+.3f}), net slip {net:+.2e} m "
                  f"(executed {net - seam_net:+.2e}, seam teleports {seam_net:+.2e}; "
                  f"unsigned {slip:.2e}) over {travel:.2f} m travel, "
                  f"heading swing {heading.max() - heading.min():.2f} rad")
        if "phase_constraints" in chart_kwargs:
            # Enforcement is at the planner's validation samples, so uniformly-resampled
            # lift samples may peak slightly between them.
            max_ke = max(float(path_module.kinetic_energy(z)) for z in dense)
            max_ee = max(vf.max_eef_speed(path_module, z) for z in dense)
            ke_cap = f" (cap {max_kinetic_energy:g} J)" if max_kinetic_energy is not None else ""
            ee_cap = f" (cap {max_eef_speed:g} m/s)" if max_eef_speed is not None else ""
            print(f"phase: max kinetic energy {max_ke:.3f} J{ke_cap}, "
                  f"max EEF speed {max_ee:.3f} m/s{ee_cap}")

    if visualize:
        from viser import transforms as tf
        from viser_utils import setup_viser_with_robot, add_point_cloud, add_spheres, add_trajectory

        robot_dir = Path(__file__).parents[1] / "resources" / "panda"
        server, robot = setup_viser_with_robot(robot_dir, urdf)
        robot.update_cfg(np.asarray(start)[:n_q])

        if e.spheres:
            add_spheres(
                server,
                [s.position for s in e.spheres],
                [s.r for s in e.spheres],
                prefix="/environment/sphere",
            )

        for i, c in enumerate(e.cuboids + e.z_aligned_cuboids):
            axes = np.array([[getattr(c, f"axis_{j}_{k}") for j in (1, 2, 3)] for k in "xyz"])
            server.scene.add_box(
                f"/environment/cuboid_{i}",
                color=(160, 160, 160),
                dimensions=tuple(2.0 * getattr(c, f"axis_{j}_r") for j in (1, 2, 3)),
                wxyz=tf.SO3.from_matrix(axes).wxyz,
                position=np.array([c.x, c.y, c.z]),
            )

        if mode == "line":
            # The line constraint is anchored at the end-effector pose of the start seed;
            # motion is allowed only along the local z (approach) axis.
            reference = np.array(ambient.eefk(start_seed[:n_q]))
            rotation, origin = reference[:3, :3], reference[:3, 3]
            axis = rotation[:, 2]
            server.scene.add_line_segments(
                "/constraint/line",
                points=np.array([[origin - LINE_EXTENT * axis, origin + LINE_EXTENT * axis]]),
                colors=(255, 140, 0),
                line_width=4.0,
            )
            server.scene.add_frame(
                "/constraint/reference",
                wxyz=tf.SO3.from_matrix(rotation).wxyz,
                position=origin,
                axes_length=0.1,
                axes_radius=0.004,
            )
        elif mode == "plane":
            server.scene.add_box(
                "/constraint/plane",
                color=(255, 140, 0),
                dimensions=(1.4, 1.4, 0.002),
                opacity=0.25,
                wxyz=np.array(PLANE_POSE[:4]),
                position=np.array(PLANE_POSE[4:]),
            )
        elif mode == "knife":
            # The virtual board: the blade's contact point slides in this plane.
            rotation, origin = knife_reference[:3, :3], knife_reference[:3, 3]
            server.scene.add_box(
                "/constraint/board",
                color=(255, 140, 0),
                dimensions=(0.8, 0.8, 0.002),
                opacity=0.25,
                wxyz=tf.SO3.from_matrix(rotation).wxyz,
                position=origin,
            )
        elif mode == "screw":
            rotation, origin = screw_reference[:3, :3], screw_reference[:3, 3]
            axis = rotation[:, 2]
            extent = SCREW_PITCH * (turn + 0.1) / (2.0 * np.pi)
            server.scene.add_line_segments(
                "/constraint/screw_axis",
                points=np.array([[origin - extent * axis, origin + extent * axis]]),
                colors=(255, 140, 0),
                line_width=4.0,
            )
            server.scene.add_frame(
                "/constraint/reference",
                wxyz=tf.SO3.from_matrix(rotation).wxyz,
                position=origin,
                axes_length=0.1,
                axes_radius=0.004,
            )

        # Geometric paths are already dense (planners emit whole projected waypoint
        # chains); flask paths are lifted to the executed trajectory. Either way, no
        # interpolation: linear interpolation leaves the manifold.
        # numpy() is a read-only view and the bindings only take writable arrays, so copy.
        waypoints = dense[:, :n_q] if kinodynamic else path.numpy().copy()

        if mode != "bimanual":
            # End-effector positions along the path; these should all lie on the constraint.
            ee_trace = np.array([np.array(ambient.eefk(q))[:3, 3] for q in waypoints])
            add_point_cloud(server, ee_trace, colors=[0, 255, 0], point_size=0.008, prefix="/ee_trace")

        if mode == "screw":
            # The end-effector origin lies on the screw axis, so its own trace is a
            # straight line; an off-axis point of the hand traces the actual helix.
            helix = np.array(
                [(np.array(ambient.eefk(q)) @ [0.06, 0.0, 0.0, 1.0])[:3] for q in waypoints])
            add_point_cloud(
                server, helix, colors=[255, 0, 255], point_size=0.006, prefix="/helix_trace")

        slider = add_trajectory(server, waypoints, robot, [], [[]])
        play = server.gui.add_checkbox("Play", initial_value=True)

        # Playback schedule: flask trajectories replay on the lifted wall-clock timing;
        # geometric paths carry no timing, so they step at a constant rate.
        play_times = dense_t if kinodynamic else np.arange(len(waypoints)) / PLAYBACK_FPS
        period = play_times[-1] + 1.0  # Hold the goal pose for a beat before looping.

        print(f"visualization at http://localhost:{server.get_port()}; ctrl-c to exit")
        t0 = time.perf_counter()
        was_playing = True
        while True:
            if play.value:
                if not was_playing:  # Resume from wherever the slider was scrubbed to.
                    t0 = time.perf_counter() - play_times[int(slider.value)]
                t = min((time.perf_counter() - t0) % period, play_times[-1])
                idx = int(np.searchsorted(play_times, t, side="right") - 1)
                if idx != int(slider.value):
                    slider.value = idx  # Fires the slider callback, which moves the robot.
            was_playing = play.value
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    Fire(main)
