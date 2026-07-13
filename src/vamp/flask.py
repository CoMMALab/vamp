"""Utilities for FLASK (differentially flat) robot modules.

Flask robot modules plan over flat states z = (q, qdot); edges are
minimum-acceleration cubics with optimal duration T*. These helpers
reconstruct the time-parameterized trajectory from a planned path of
z waypoints using the per-robot bindings (optimal_time, eval, torques).
"""

from typing import Any, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray


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


def path_length(robot_module: Any, path: Any, samples_per_segment: int = 64) -> float:
    """Arc length in configuration space (rad) of the cubic path through q."""
    _, q, _, _, _ = densify(robot_module, path, samples_per_segment)
    return float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))
