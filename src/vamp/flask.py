"""Utilities for FLASK (differentially flat) robot modules.

Flask robot modules plan over flat states z = (q, qdot); edges are
minimum-acceleration cubics with optimal duration T*. These helpers
reconstruct the time-parameterized trajectory from a planned path of
z waypoints using the per-robot bindings (optimal_time, eval, torques).
"""

from typing import Any, Tuple

import numpy as np
from numpy.typing import NDArray


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
