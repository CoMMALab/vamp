"""Leader-follower task-space planning over BimanualIiwa's shelf problem, ported from the
`iiwa_parameterized_ik_planner` branch's scripts/bimanual_iiwa.py (old flat
`use_parameterized_ik`/`ik_parameters` mechanism) onto the current
`bimanualiiwa.leader_follower_space` bindings (BimanualIiwa::LeaderFollowerSpace).

The old branch's BimanualIiwa::parameterized_ik took an 18-element input: 8 free
variables (Robot::dimension: the left/leader arm's 7 joint angles + 1 self-motion-
manifold parameter psi) followed by Robot::num_ik_parameters = 10 fixed closure
parameters (default {1.0, 1.0, -1.0, 0.0, 0.0, 0.6, 0.927184, -0.374607, 0.0, 0.0} --
see scripts/cpp/iiwa_try.cc's start_array/goal_array construction). That 8+10 split is
structurally identical to LeaderFollowerSpace: the first 8 are exactly its State (leader
joints + follower psi -- same joint-limit bounds, verified against
BimanualIiwa::LeaderFollowerSpace::sample() in bimanual_iiwa.hh), and the trailing 10
split into the first 3 (the follower's self-motion-manifold branch, i.e. `smm` /
set_smm()) and the last 7 (the fixed leader-to-follower hand offset, i.e. `rel_pose` /
set_rel_pose()). This script sets those two the same way the old branch's default
ik_parameters did, then reuses its three named problem states (q_tilde_bottom/middle/
top, from the old scripts/bimanual_iiwa.py and resources/iiwa/example_points.txt) as
LeaderFollowerSpace task-space states, planning random pairs of them through the same
shelf environment (resources/iiwa/cuboids/shelf_drake.txt, inlined below since that
resource doesn't exist on this branch).

Usage:
    python scripts/bimanual_iiwa_leader_follower_shelf.py
    python scripts/bimanual_iiwa_leader_follower_shelf.py --n_trials 20 --range_ 0.3
"""

import random
import time

import numpy as np
import pandas as pd
import vamp
from fire import Fire

np.set_printoptions(precision=3, suppress=True)

# Default closure parameters from the old branch's BimanualIiwa::ik_parameters
# ({1.0, 1.0, -1.0, 0.0, 0.0, 0.6, 0.927184, -0.374607, 0.0, 0.0}): first 3 are the
# follower's self-motion-manifold branch (LeaderFollowerSpace::smm / set_smm), last 7 are
# the fixed leader-to-follower hand offset (LeaderFollowerSpace::rel_pose / set_rel_pose),
# as (x, y, z, qx, qy, qz, qw).
DEFAULT_SMM = [1.0, 1.0, -1.0]
DEFAULT_REL_POSE = [0.0, 0.0, 0.6, 0.927184, -0.374607, 0.0, 0.0]

# The three named problem states from the old branch's scripts/bimanual_iiwa.py /
# resources/iiwa/example_points.txt: each is (7 leader/left-arm joint angles, psi) --
# exactly LeaderFollowerSpace::State's 8-element layout.
Q_TILDE_BOTTOM = [
    -0.6430910102907225, 1.9156121024586796, -1.7968254667817805, 1.2945447141185198,
    -0.023834531305537934, -0.876966810663043, -1.7041643160834519, 1.45,
]
Q_TILDE_MIDDLE = [
    -0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483,
    -0.04421061906813227, -0.8793712572715165, -1.1603461715511334, 1.45,
]
Q_TILDE_TOP = [
    -0.1994994216078726, 0.9140739951190965, -2.236618320862171, 0.5238879195899456,
    0.7998441913611017, -1.3575398006936048, -1.0153092816310436, 2.41,
]
NAMED_STATES = {
    "bottom": np.asarray(Q_TILDE_BOTTOM, dtype=np.float32),
    "middle": np.asarray(Q_TILDE_MIDDLE, dtype=np.float32),
    "top": np.asarray(Q_TILDE_TOP, dtype=np.float32),
}

# Shelf cuboids from the old branch's resources/iiwa/cuboids/shelf_drake.txt: each row is
# (x, y, z, dx, dy, dz) -- position + *full* extents (halved below to match vamp.Cuboid's
# half-extent convention, same as the old scripts/bimanual_iiwa.py did).
SHELF_CUBOIDS = [
    [0.8, 0.3825, 0.3, 0.4, 1.0, 0.014],
    [0.8, 0.3825, 0.58, 0.4, 1.0, 0.014],
    [1.0, 0.3825, 0.45, 0.03, 1.0, 0.9],
    [0.4, 0.3825, -0.2, 5, 5, 0.2],
]


def build_environment() -> vamp.Environment:
    env = vamp.Environment()
    for cuboid in SHELF_CUBOIDS:
        position = cuboid[:3]
        half_extents = [d / 2.0 for d in cuboid[3:6]]
        env.add_cuboid(vamp.Cuboid(position, [0.0, 0.0, 0.0], half_extents))
    return env


def resolve_and_validate(param, ambient, state: np.ndarray, environment: vamp.Environment):
    """IK-resolve a task-space state and collision-check the resulting ambient config.
    Returns (valid, ambient_config or None)."""
    valid, ambient_config = param.resolve(state)
    if not valid:
        return False, None
    ambient_config = np.asarray(ambient_config, dtype=np.float32)
    if not ambient.validate(ambient_config, environment):
        return False, None
    return True, ambient_config


def main(
    n_trials: int = 100,
    range_: float = 0.5,
    max_iterations: int = 1_000_000,
    seed: int = 0,
):
    random.seed(seed)
    np.random.seed(seed)

    ambient = vamp.bimanualiiwa
    param = ambient.leader_follower_space

    param.set_smm(DEFAULT_SMM)
    param.set_rel_pose(DEFAULT_REL_POSE)

    environment = build_environment()
    print(
        f"Environment has {len(environment.cuboids) + len(environment.z_aligned_cuboids)} "
        "cuboids (shelf).")

    print("\n--- Checking the three named problem states ---")
    valid_states = {}
    for label, state in NAMED_STATES.items():
        valid, ambient_config = resolve_and_validate(param, ambient, state, environment)
        print(f"{label}: resolve+validate = {valid}")
        if valid:
            valid_states[label] = state

    if len(valid_states) < 2:
        print("Fewer than two named states are valid under this rel_pose/smm; nothing to plan.")
        return

    names = list(valid_states.keys())
    settings = vamp.RRTCSettings()
    settings.range = range_
    settings.max_iterations = max_iterations

    results = []
    for trial in range(n_trials):
        start_label, goal_label = random.sample(names, 2) if len(names) > 1 else (names[0], names[0])
        start_state = valid_states[start_label]
        goal_state = valid_states[goal_label]

        sampler = param.halton()

        t0 = time.perf_counter()
        result = param.rrtc(start_state, goal_state, environment, settings, sampler)
        elapsed_ns = int((time.perf_counter() - t0) * 1e9)

        entry = {
            "trial": trial,
            "start": start_label,
            "goal": goal_label,
            "solved": result.solved,
            "iterations": result.iterations,
            "planning_time_ns": result.nanoseconds,
            "python_wall_ns": elapsed_ns,
        }

        if result.solved:
            entry["initial_path_cost"] = result.path.cost()
            changed = param.shortcut(result.path, environment)
            entry["shortcut_changed"] = changed
            entry["simplified_path_cost"] = result.path.cost()

        results.append(entry)

    df = pd.DataFrame.from_dict(results)
    print(f"\nSolved {df['solved'].sum()} / {len(df)} trials.")

    solved_df = df[df["solved"]]
    if len(solved_df):
        solved_df = solved_df.copy()
        solved_df["planning_time_us"] = solved_df["planning_time_ns"] / 1e3
        stats = solved_df[[
            "planning_time_us",
            "iterations",
            "initial_path_cost",
            "simplified_path_cost",
        ]].describe()
        print(stats)


if __name__ == "__main__":
    Fire(main)
