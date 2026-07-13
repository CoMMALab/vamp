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
    """Phase gates for the given caps; empty when no cap is given.

    Raises RuntimeError if a cap is requested but the robot was not generated
    with the corresponding phase-constraint kernel.
    """
    gates = []
    for cap, kernel in (
        (max_kinetic_energy, "KineticEnergyConstraint"),
        (max_eef_speed, "EEFSpeedConstraint"),
    ):
        if cap is None:
            continue
        if not hasattr(robot_module, kernel):
            raise RuntimeError(f"Robot {robot_module.__name__} has no {kernel} kernel!")
        gates.append(getattr(robot_module, kernel)(float(cap)))

    return gates


def max_eef_speed(robot_module: Any, z: Any) -> float:
    """Largest per-end-effector linear speed of a flat state z (the quantity the
    EEF-speed phase gate caps: each end-effector's speed separately)."""
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
    """
    waypoints = [np.asarray(path[i], dtype=np.float32) for i in range(len(path))]
    if len(waypoints) < 2:
        raise ValueError("path must have at least two waypoints")

    ts, qs, qds, qdds, taus = [], [], [], [], []

    def emit(t: float, a: NDArray[np.float32], b: NDArray[np.float32], T: float, frac: float) -> None:
        y, yd, ydd = robot_module.eval(a, b, T, frac)
        tau = robot_module.torques(np.concatenate([y, yd, ydd]).astype(np.float32))
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
