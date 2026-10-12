"""Utilities for FLASK (differentially flat) robot modules.

Flask robot modules plan over flat states z = (q, qdot); edges are
minimum-acceleration cubics with optimal duration T*. These helpers
reconstruct the time-parameterized trajectory from a planned path of
z waypoints using the per-robot bindings (optimal_time, eval, torques).
"""

from typing import Any, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from ._core import SimplifyRoutine

# Simplify schedule for retiming lifted geometric paths. Lifted paths are dense
# rest-state chains, so a second BSPLINE round before VELOPT beats the direct
# default: on full MBM it matches its cost while cutting densified collision
# risk (1.9% vs 4.6%) and time (see 2026-07 sweep).
RETIME_OPERATIONS = [
    SimplifyRoutine.SHORTCUT,
    SimplifyRoutine.BSPLINE,
    SimplifyRoutine.BSPLINE,
    SimplifyRoutine.VELOPT,
    SimplifyRoutine.REDUCE,
]


def rest_state(q: Any) -> NDArray[np.float32]:
    """Lift a configuration q to the rest flat state z = (q, 0)."""
    q = np.asarray(q, dtype=np.float32)
    return np.concatenate([q, np.zeros_like(q)])


def parse_flask_robot(robot: str, rho: Optional[float] = None) -> Tuple[str, Any]:
    """Resolve a "<base>.flask" robot name to (base name, ambient geometric module).

    The flask robot is a nested submodule of its ambient geometric parent, which
    shares the URDF/sphere model. Optionally overrides the flask sibling's LQMT
    cost weight rho.
    """
    import vamp

    if not robot.endswith(".flask"):
        raise RuntimeError(f"Robot {robot} is not a flask robot!")

    base_robot = robot[: -len(".flask")]
    if base_robot not in vamp.robots:
        raise RuntimeError(f"Robot {base_robot} does not exist in VAMP!")
    geo_module = getattr(vamp, base_robot)

    if rho is not None:
        geo_module.flask.set_rho(float(rho))

    return base_robot, geo_module


def phase_constraints(
    robot_module: Any,
    max_kinetic_energy: Optional[float] = None,
    max_eef_speed: Optional[float] = None,
) -> List[Any]:
    """Phase constraints for the given caps; empty when no cap is given.

    Raises RuntimeError if a cap is requested but the robot was not generated
    with the corresponding phase-constraint kernel.
    """
    constraints = []
    for cap, kernel in (
        (max_kinetic_energy, "KineticEnergyConstraint"),
        (max_eef_speed, "EEFSpeedConstraint"),
    ):
        if cap is None:
            continue
        if not hasattr(robot_module, kernel):
            raise RuntimeError(f"Robot {robot_module.__name__} has no {kernel} kernel!")
        constraints.append(getattr(robot_module, kernel)(float(cap)))

    return constraints


def max_eef_speed(robot_module: Any, z: Any) -> float:
    """Largest per-end-effector linear speed of a flat state z (the quantity the
    EEF-speed phase constraint caps: each end-effector's speed separately)."""
    v = robot_module.eef_velocity(np.asarray(z, dtype=np.float32))
    return float(np.linalg.norm(np.reshape(v, (-1, 3)), axis=1).max())


def segment_times(robot_module: Any, path: Any) -> NDArray[np.float64]:
    """Optimal durations T* for each segment of a flask path."""
    waypoints = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]
    return np.array(
        [robot_module.optimal_time(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])],
        dtype=np.float64,
    )


def densify(
    robot_module: Any,
    path: Any,
    samples_per_segment: int = 32,
    environment: Optional[Any] = None,
) -> Tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Densify a flask path of z waypoints into trajectory arrays.

    Returns (t, q, qd, qdd, tau): times are cumulative across segments,
    each segment sampled at samples_per_segment points plus the final state.
    Torques include the payloads of `environment`'s attachments when given.
    """
    waypoints = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]
    if len(waypoints) < 2:
        raise ValueError("path must have at least two waypoints")

    ts, qs, qds, qdds, taus = [], [], [], [], []

    def emit(t: float, a: NDArray[np.float32], b: NDArray[np.float32], T: float, frac: float) -> None:
        y, yd, ydd = robot_module.eval(a, b, T, frac)
        x = np.concatenate([y, yd, ydd]).astype(np.float32)
        tau = robot_module.torques(x) if environment is None else robot_module.torques(x, environment)
        ts.append(t)
        qs.append(y)
        qds.append(yd)
        qdds.append(ydd)
        taus.append(tau)

    t_offset = 0.0
    T = 0.0
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        T = robot_module.optimal_time(a, b)
        for frac in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
            emit(t_offset + frac * T, a, b, T, float(frac))
        t_offset += T

    emit(t_offset, waypoints[-2], waypoints[-1], T, 1.0)

    return (
        np.array(ts, dtype=np.float64),
        np.array(qs, dtype=np.float64),
        np.array(qds, dtype=np.float64),
        np.array(qdds, dtype=np.float64),
        np.array(taus, dtype=np.float64),
    )


def jerk_metrics(robot_module: Any, path: Any) -> Tuple[float, float, float]:
    """Jerk statistics of a flask path's cubic trajectory.

    Each LQMT cubic has constant jerk (qdd is linear in t) and acceleration is
    discontinuous at interior waypoints (impulsive jerk). Returns
    (max_jerk, rms_jerk, max_accel_jump): largest per-joint |jerk| (rad/s^3),
    time-weighted RMS of the jerk vector norm, and largest per-joint
    acceleration discontinuity at a junction (rad/s^2).
    """
    waypoints = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]
    if len(waypoints) < 2:
        raise ValueError("path must have at least two waypoints")

    max_jerk = 0.0
    rms_num = 0.0
    total_time = 0.0
    max_jump = 0.0
    previous_end_qdd = None
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        T = robot_module.optimal_time(a, b)
        _, _, qdd0 = robot_module.eval(a, b, T, 0.0)
        _, _, qdd1 = robot_module.eval(a, b, T, 1.0)
        jerk = (np.asarray(qdd1) - np.asarray(qdd0)) / T
        max_jerk = max(max_jerk, float(np.max(np.abs(jerk))))
        rms_num += T * float(np.dot(jerk, jerk))
        total_time += T
        if previous_end_qdd is not None:
            max_jump = max(max_jump, float(np.max(np.abs(np.asarray(qdd0) - previous_end_qdd))))
        previous_end_qdd = np.asarray(qdd1)

    return max_jerk, float(np.sqrt(rms_num / total_time)), max_jump


def path_length(robot_module: Any, path: Any, samples_per_segment: int = 64) -> float:
    """Arc length in configuration space (rad) of the cubic path through q."""
    _, q, _, _, _ = densify(robot_module, path, samples_per_segment)
    return float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))


def lift(robot_module: Any, path: Any) -> Any:
    """Lift a geometric path to a flask Path of rest states z = (q, 0).

    A rest-to-rest LQMT segment is a smooth-step along the straight line, so the
    lifted path traces the geometric path exactly (stop-at-every-waypoint).
    Consecutive duplicate configurations are dropped. Retiming is then just the
    flask robot's simplify: shortcut states sampled along segment interiors carry
    the cubic's nonzero velocities, so accepted shortcuts (validated at full
    resolution, scored by the LQMT cost C_loc) blend waypoint stops into flowing
    motion.
    """
    lifted = robot_module.Path()
    previous = None
    for i in range(len(path)):
        q = np.asarray(path[i], dtype=np.float32)
        if previous is not None and np.array_equal(q, previous):
            continue
        lifted.append(rest_state(q))
        previous = q
    return lifted


def path_cost(robot_module: Any, path: Any) -> float:
    """Total LQMT cost of a flask path (sum of segment costs C(z_i -> z_{i+1}))."""
    waypoints = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]
    return float(sum(robot_module.cost(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])))


def _lp2_extreme(A: NDArray, b: NDArray, objective: NDArray, maximize: bool) -> Optional[NDArray]:
    """Optimum of objective . (u, x) over {A (u, x) <= b} by vertex enumeration (2 variables)."""
    i, j = np.triu_indices(len(b), 1)
    det = A[i, 0] * A[j, 1] - A[i, 1] * A[j, 0]
    ok = np.abs(det) > 1e-12
    i, j, det = i[ok], j[ok], det[ok]
    u = (b[i] * A[j, 1] - A[i, 1] * b[j]) / det
    x = (A[i, 0] * b[j] - b[i] * A[j, 0]) / det
    pts = np.stack([u, x], axis=1)
    feasible = np.all(pts @ A.T <= b + 1e-9 * (1.0 + np.abs(b)), axis=1)
    if not np.any(feasible):
        return None
    values = pts[feasible] @ objective
    return pts[feasible][np.argmax(values) if maximize else np.argmin(values)]


def topp_retime(
    robot_module: Any,
    q: Any,
    environment: Optional[Any] = None,
    grid: int = 800,
    effort_fraction: float = 0.98,
    end_speed: float = 0.0,
    substeps: int = 4,
) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Time-optimal retiming of a dense joint path under torque and joint-velocity limits.

    Time-optimal path parameterization by reachability analysis (Pham & Pham, "A new approach
    to time-optimal path parameterization based on reachability analysis", T-RO 2018). The path
    q(s) is a chord-length natural cubic spline through `q` (e.g. a densified executed trajectory, so
    constraint manifolds and collision clearance are those of the given geometry), sampled at
    `grid` points. Torques are affine in (sdd, sd^2), tau = a sdd + b sd^2 + c, with a, b, c
    from the robot's payload-aware RNEA (`environment`'s attachments); they must stay within
    effort_fraction of the (scaled) effort limits at both ends of every grid interval (the
    first-order interpolation discretization), and joint speeds within their limits. The path starts at rest and ends with path speed at most `end_speed` (path
    parameter units per second; 0 arrives at rest). Statically infeasible stretches are kept
    when the velocity terms make them feasible. Constraints hold at the grid points; between them
    (constant sdd per grid interval, sd^2 linear in s) effort_fraction is the margin. Returns
    (t, q, qd, qdd) of that executed motion at `substeps` points per grid interval; raises
    ValueError if no feasible timing exists.
    """
    from scipy.interpolate import CubicSpline

    # Densified trajectories repeat (nearly) the junction states of consecutive edges; such
    # near-duplicates put spikes in the interpolating spline's curvature, so keep only points at
    # least a quarter of the median spacing from the previous kept one.
    q = np.asarray(q, dtype=np.float64)
    gap = 0.25 * np.median(np.linalg.norm(np.diff(q, axis=0), axis=1))
    kept = [q[0]]
    for point in q[1:-1]:
        if np.linalg.norm(point - kept[-1]) >= gap:
            kept.append(point)
    if np.linalg.norm(q[-1] - kept[-1]) < gap and len(kept) > 1:
        kept.pop()
    kept.append(q[-1])
    q = np.array(kept)
    chord = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    spline = CubicSpline(chord / chord[-1], q, axis=0, bc_type="natural")
    s = np.linspace(0.0, 1.0, grid)
    qs, dq, ddq = spline(s), spline(s, 1), spline(s, 2)
    step = s[1] - s[0]
    n = q.shape[1]

    limit = effort_fraction * np.asarray(robot_module.effort_limits()) * robot_module.effort_scale()
    vmax = np.asarray(robot_module.velocity_limits())
    zero = np.zeros(n)

    def tau(x: NDArray) -> NDArray:
        x = x.astype(np.float32)
        t = robot_module.torques(x) if environment is None else robot_module.torques(x, environment)
        return np.asarray(t, dtype=np.float64)

    c = np.array([tau(np.concatenate([qi, zero, zero])) for qi in qs])
    a = np.array([tau(np.concatenate([qi, zero, di])) for qi, di in zip(qs, dq)]) - c
    b = np.array([tau(np.concatenate([qi, di, ddi])) for qi, di, ddi in zip(qs, dq, ddq)]) - c
    with np.errstate(divide="ignore"):
        x_cap = np.min(np.where(np.abs(dq) > 1e-12, (vmax / np.abs(dq)) ** 2, np.inf), axis=1)

    def rows(i: int) -> Tuple[NDArray, NDArray]:
        # Interpolation discretization: interval i's (u, x) satisfies a u + b x + c within +-limit at
        # its start, and at its end with sd^2 = x + 2 step u; 0 <= x <= x_cap at both. A (u, x) <= rhs.
        j = min(i + 1, grid - 1)
        ends = [np.stack([a[i], b[i]], 1), np.stack([a[j] + 2 * step * b[j], b[j]], 1)]
        A = np.vstack([ends[0], -ends[0], ends[1], -ends[1], [[0.0, -1.0]], [[0.0, 1.0]], [[2 * step, 1.0]]])
        rhs = np.concatenate([limit - c[i], limit + c[i], limit - c[j], limit + c[j], [0.0], [x_cap[i]], [x_cap[j]]])
        return A, rhs

    # Backward pass: controllable sets K_i = [lo_i, hi_i] of sd^2.
    lo, hi = np.zeros(grid), np.zeros(grid)
    hi[-1] = min(end_speed**2, x_cap[-1])
    for i in range(grid - 2, -1, -1):
        A, rhs = rows(i)
        A = np.vstack([A, [[2 * step, 1.0]], [[-2 * step, -1.0]]])
        rhs = np.concatenate([rhs, [hi[i + 1]], [-lo[i + 1]]])
        top = _lp2_extreme(A, rhs, np.array([0.0, 1.0]), True)
        bottom = _lp2_extreme(A, rhs, np.array([0.0, 1.0]), False)
        if top is None or bottom is None:
            raise ValueError(f"no feasible timing: the path is uncontrollable at grid point {i}")
        lo[i], hi[i] = max(bottom[1], 0.0), top[1]
    if lo[0] > 1e-12:
        raise ValueError("no feasible timing from rest at the start")

    # Forward pass: greedy maximum sdd within the controllable sets.
    x = np.zeros(grid)
    u = np.zeros(grid)
    for i in range(grid - 1):
        A, rhs = rows(i)
        A = np.vstack([A, [[2 * step, 1.0]], [[-2 * step, -1.0]]])
        rhs = np.concatenate([rhs, [hi[i + 1]], [-lo[i + 1]]])
        upper = np.inf
        lower = -np.inf
        for coeff, bound in zip(A[:, 0], rhs - A[:, 1] * x[i]):
            if coeff > 1e-12:
                upper = min(upper, bound / coeff)
            elif coeff < -1e-12:
                lower = max(lower, bound / coeff)
        u[i] = upper if upper >= lower else lower
        x[i + 1] = min(max(x[i] + 2 * step * u[i], lo[i + 1]), hi[i + 1])
        u[i] = (x[i + 1] - x[i]) / (2 * step)

    # Executed motion: within grid interval i, sdd = u_i and sd^2 = x_i + 2 u_i (s - s_i).
    frac = np.arange(substeps) / substeps
    ss = (s[:-1, None] + step * frac[None, :]).ravel()
    xs = np.maximum((x[:-1, None] + 2 * u[:-1, None] * step * frac[None, :]).ravel(), 0.0)
    us = np.repeat(u[:-1], substeps)
    ss, xs, us = np.append(ss, 1.0), np.append(xs, x[-1]), np.append(us, u[-2])
    sd = np.sqrt(xs)
    dt = 2 * np.diff(ss) / np.maximum(sd[:-1] + sd[1:], 1e-12)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    dqs, ddqs = spline(ss, 1), spline(ss, 2)
    return t, spline(ss), dqs * sd[:, None], dqs * us[:, None] + ddqs * xs[:, None]
