"""Task-space planning example for BimanualIiwa: demonstrates both of its task-space
parameterizations --

  * `bimanualiiwa.parameterized_space` (BimanualIiwa::ParameterizedSpace): RBY1-style
    mid-pose parameterization. Samples a shared SE3 mid-frame (position + orientation)
    plus one self-motion-manifold redundancy parameter per arm (left_psi, right_psi),
    and IK-resolves both arms from fixed, once-derived hand-to-mid-frame offsets
    (t_mid_left / t_mid_right, via compute_mid_pose()).

  * `bimanualiiwa.leader_follower_space` (BimanualIiwa::LeaderFollowerSpace):
    sample-one-arm/FK/IK-the-follower parameterization. Samples the leader (left) arm's
    7 joint angles directly (no IK) plus one follower self-motion-manifold parameter,
    and IK-resolves the follower (right) arm's pose from a fixed, once-derived
    leader-to-follower hand offset (rel_pose, via compute_rel_pose()).

bimanual_iiwa.hh is newly generated and has no curated problem set yet (unlike
resources/ruby or resources/iiwa_marker), so this script derives everything it needs at
runtime: it samples the ambient joint space directly for a self-collision-free reference
configuration (used to derive each space's fixed offset), then rejection-samples
task-space states with each space's own Halton stream until both a start and a goal
resolve through IK and validate collision-free.

Usage:
    python scripts/bimanual_iiwa_task_space_example.py
    python scripts/bimanual_iiwa_task_space_example.py --space leader_follower --n_trials 5
"""

import time

import numpy as np
import vamp
from fire import Fire

np.set_printoptions(precision=3, suppress=True)


def find_reference_configuration(ambient, environment, max_attempts: int = 2000) -> np.ndarray:
    """Sample the ambient joint space directly until a self-collision-free configuration
    turns up; used as the "home" whole-body config that compute_mid_pose()/
    compute_rel_pose() derive each space's fixed offset from."""
    sampler = ambient.halton()
    for _ in range(max_attempts):
        q = np.asarray(sampler.next(), dtype=np.float32)
        if ambient.validate(q, environment):
            return q
    raise RuntimeError(f"no self-collision-free ambient configuration found in {max_attempts} samples")


def find_valid_task_state(param, ambient, environment, max_attempts: int = 2000):
    """Rejection-sample a task-space state (via the space's own Halton stream) until it
    IK-resolves and the resolved ambient configuration is collision-free. Returns
    (state, ambient_config)."""
    sampler = param.halton()
    for _ in range(max_attempts):
        state = sampler.next()
        valid, ambient_config = param.resolve(state)
        if not valid:
            continue
        ambient_config = np.asarray(ambient_config, dtype=np.float32)
        if ambient.validate(ambient_config, environment):
            return np.asarray(state, dtype=np.float32), ambient_config
    raise RuntimeError(f"no valid task-space state found in {max_attempts} samples")


def run_space(name, param, ambient, environment, n_trials: int, range_: float, max_iterations: int):
    task_dimension = len(np.asarray(param.halton().next()))
    print(f"\n=== {name} (task-space dimension {task_dimension}) ===")

    solved = 0
    for trial in range(n_trials):
        start_state, _ = find_valid_task_state(param, ambient, environment)
        goal_state, _ = find_valid_task_state(param, ambient, environment)

        sampler = param.halton()
        settings = vamp.RRTCSettings()
        settings.range = range_
        settings.max_iterations = max_iterations

        t0 = time.perf_counter()
        result = param.rrtc(start_state, goal_state, environment, settings, sampler)
        elapsed_ms = (time.perf_counter() - t0) * 1e3

        status = "solved" if result.solved else "failed"
        path_size = len(result.path) if result.solved else 0
        print(
            f"  trial {trial}: {status} in {elapsed_ms:.1f} ms, "
            f"{result.iterations} iterations, path size {path_size}")

        if result.solved:
            solved += 1
            cost_before = result.path.cost()
            changed = param.shortcut(result.path, environment)
            print(
                f"    shortcut changed={changed}, cost {cost_before:.3f} -> "
                f"{result.path.cost():.3f}, path size {len(result.path)}")

    print(f"{name}: solved {solved}/{n_trials}")


def main(
    space: str = "both",  # "mid_pose", "leader_follower", or "both"
    n_trials: int = 3,
    range_: float = 0.5,
    max_iterations: int = 200_000,
    seed: int = 0,
):
    np.random.seed(seed)

    ambient = vamp.bimanualiiwa
    environment = vamp.Environment()  # empty; add obstacles as needed

    print(f"Ambient joint-space dimension: {ambient.dimension()}")

    reference = find_reference_configuration(ambient, environment)
    print(f"Reference (home) ambient configuration: {reference}")

    if space in ("mid_pose", "both"):
        mid = ambient.parameterized_space
        mid.compute_mid_pose(reference)
        run_space(
            "ParameterizedSpace (mid-pose)", mid, ambient, environment, n_trials, range_, max_iterations)

    if space in ("leader_follower", "both"):
        lf = ambient.leader_follower_space
        lf.compute_rel_pose(reference)
        run_space(
            "LeaderFollowerSpace (leader-follower)", lf, ambient, environment, n_trials, range_,
            max_iterations)


if __name__ == "__main__":
    Fire(main)
