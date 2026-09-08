"""Task-space (TSR) parameterized-planning variant of evaluate_mbm.py.

Instead of planning directly in the robot's joint configuration space, this evaluator
plans over Panda's `ParameterizedSpace` (end-effector pose + redundancy angle "psi"),
via `vamp.panda.parameterized_space` -- see scripts/fr3_marker_maze_example.py for the
FR3-marker analogue this mirrors. EEF_TO_OFFSET and WORLD_TO_REFERENCE are left at
identity (the eef pose IS the task-space pose, expressed directly in world coordinates),
and the TSR bound is a wide-open +-1.5m translation box with the full [-pi, pi] so(3)
log-map box on every rotation axis -- i.e. no real task constraint, just a bounding
region for the TSR-informed sampler to draw from. So every solved MBM problem is still
the same reachability problem, just sampled/interpolated as SE(3) poses and IK-resolved
on the fly (`param.resolve()`) rather than rejecting joint-space samples on a downstream
IK check.

Each MBM problem stores its start/goal(s) as joint configurations, not task-space poses,
so start/goal task-space states are derived per problem:
  1. FK the joint configuration (`vamp.panda.eefk`) to get its eef pose.
  2. Panda's IK is redundant beyond the eef pose + psi (see panda.hh's ParameterizedSpace
     comments): resolving a pose also requires picking a (case6_sel, case1_sel) branch
     ("smm"). Since MBM configs don't record which branch produced them, every branch in
     BRANCH_ORDER is tried (fixed order) until one resolves (and collision-validates) both
     the start and at least one goal.
  3. For a given branch, psi (index 7, literally the joint-7 angle for Panda) is tried at
     the config's own joint-7 value first -- the branch that actually produced the saved
     config should resolve exactly there -- falling back to a coarse grid (PSI_GRID) only
     if that fails.
A problem is "unresolvable" (distinct from MBM's own 'valid' flag) if no branch resolves
both endpoints this way; it's skipped, same as an MBM-invalid problem, just counted
separately.

Usage:
    python scripts/evaluate_mbm_parameterized.py --problem box --trials 1
    python scripts/evaluate_mbm_parameterized.py --print_failures
"""

import math
import time
from typing import List, Union

import numpy as np
import pandas as pd
from fire import Fire
from scipy.spatial.transform import Rotation
from tqdm import tqdm

import vamp
from vamp import mbm

# Identity offset/reference frames: the eef pose IS the task-space pose, and the TSR
# bound below is expressed directly in world coordinates.
EEF_TO_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
WORLD_TO_REFERENCE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# Wide-open TSR: +-1.5m translation box (comfortably covers Panda's ~0.85m reach) and the
# full [-pi, pi] so(3) log-map box on every axis, i.e. unconstrained orientation sampling.
TSR_LOWER = [-1.5, -1.5, -1.5, -math.pi, -math.pi, -math.pi]
TSR_UPPER = [1.5, 1.5, 1.5, math.pi, math.pi, math.pi]

# The 4 (case6_sel, case1_sel) IK branches searched per problem, fixed order starting at
# (0, 0); the 3rd smm slot (q_actual_0) is unused for the fr3_se3 IK family and left at 0.
BRANCH_ORDER = [(1.0, 0.0), (1.0, 1.0), (0.0, 0.0), (0.0, 1.0)]

# Fallback psi grid, tried only if the problem's own joint-7 angle doesn't resolve on a
# given branch (it should, on whichever branch produced the saved configuration).
PSI_GRID = np.linspace(-math.pi, math.pi, 512, endpoint = False)


def eef_pose_state(ambient_module, q: np.ndarray, psi: float) -> np.ndarray:
    """FK a joint configuration to an 8-vector task-space state (x, y, z, qx, qy, qz, qw, psi)."""
    mat = np.asarray(ambient_module.eefk(np.asarray(q, dtype = np.float32)))
    quat = Rotation.from_matrix(mat[:3, :3]).as_quat()  # (x, y, z, w)
    return np.array([*mat[:3, 3], *quat, psi], dtype = np.float32)


def resolve_config(param, ambient_module, environment, q: np.ndarray):
    """Search psi values (the config's own joint-7 angle first, then PSI_GRID) for one that
    IK-resolves (and collision-validates) `q`'s eef pose on the currently-set smm branch.
    Returns the resolved 8-vector task-space state, or None."""
    for psi in (q[6], *PSI_GRID):
        state = eef_pose_state(ambient_module, q, psi)
        valid, ambient_config = param.resolve(state)
        if valid and ambient_module.validate(np.asarray(ambient_config, dtype = np.float32), environment):
            return state

    return None


def resolve_start_goal(param, ambient_module, environment, start: np.ndarray, goals: List[np.ndarray]):
    """Search BRANCH_ORDER for the first (case6_sel, case1_sel) branch that resolves both
    the start config and at least one goal config to valid task-space states. Returns
    (start_state, [goal_state, ...], branch) or None if no branch works for start + >=1 goal."""
    for branch in BRANCH_ORDER:
        param.set_smm([branch[0], branch[1], 0.0])

        start_state = resolve_config(param, ambient_module, environment, start)
        if start_state is None:
            continue

        goal_states = [
            goal_state for goal in goals
            if (goal_state := resolve_config(param, ambient_module, environment, goal)) is not None
            ]

        if goal_states:
            return start_state, goal_states, branch

    return None


def main(
    robot: str = "panda",                  # Robot to plan for (must have a ParameterizedSpace)
    planner: str = "rrtc",                 # Planner name to use (only "rrtc" is bound for task-space)
    dataset: str = "problems.pkl",         # Pickled MBM dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Inner sampler for the TSR-informed sampler
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out invalid/unresolvable/failed problems
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    ambient_module = getattr(vamp, robot)
    if not hasattr(ambient_module, "parameterized_space"):
        raise RuntimeError(f"Robot {robot} does not have a parameterized_space -- rebuild vamp?")

    problems, problem = mbm.load_problems(robot, dataset, problem)

    (param, planner_func, plan_settings,
     _) = vamp.configure_robot_and_planner_with_kwargs(f"{robot}.parameterized_space", planner, **kwargs)

    inner_sampler = getattr(param, sampler)()

    total_problems = 0
    valid_problems = 0          # MBM-valid AND some branch resolves start + >=1 goal
    unresolvable_problems = 0   # MBM-valid but no branch resolves start + a goal
    failed_problems = 0         # Resolved, but the planner didn't find a solution

    tick = time.perf_counter()
    results = []
    for name, pset in problems.items():
        if name not in problem:
            continue

        failures = []
        invalids = []
        unresolvable = []
        print(f"Evaluating {robot} (parameterized) on {name}: ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            environment = vamp.problem_dict_to_vamp(data)

            start = np.asarray(data['start'], dtype = np.float32)
            goals = [np.asarray(goal, dtype = np.float32) for goal in data['goals']]

            resolved = resolve_start_goal(param, ambient_module, environment, start, goals)
            if resolved is None:
                unresolvable.append(i)
                continue

            valid_problems += 1
            start_state, goal_states, branch = resolved

            tsr_sampler = param.TaskSpaceInformedSampler(
                EEF_TO_OFFSET, WORLD_TO_REFERENCE, TSR_LOWER, TSR_UPPER, environment, inner_sampler)

            tsr_sampler.reset()
            tsr_sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = planner_func(start_state, goal_states, environment, plan_settings, tsr_sampler)
                if not result.solved:
                    failures.append(i)
                    break

                shortcut_t0 = time.perf_counter()
                changed = param.shortcut(result.path, environment)
                shortcut_time = pd.Timedelta(seconds = time.perf_counter() - shortcut_t0)

                results.append({
                    'problem': name,
                    'index': i,
                    'branch': branch,
                    'planning_time': pd.Timedelta(nanoseconds = result.nanoseconds),
                    'planning_iterations': result.iterations,
                    'planning_graph_size': sum(result.size),
                    'initial_path_vertices': len(result.path),
                    'shortcut_time': shortcut_time,
                    'shortcut_changed': changed,
                    'shortcut_path_vertices': len(result.path),
                    'shortcut_path_cost': result.path.cost(),
                    'total_time': pd.Timedelta(nanoseconds = result.nanoseconds) + shortcut_time,
                    })

        failed_problems += len(failures)
        unresolvable_problems += len(unresolvable)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")
            if unresolvable:
                print(f"  Unresolvable (no IK branch for start + goal) problems: {unresolvable}")
            if failures:
                print(f"  Failed on {failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    if not df.empty:
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["shortcut_time"] = df["shortcut_time"].dt.microseconds
        df["total_time"] = df["total_time"].dt.microseconds
        df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]

        mbm.print_stats_table(
            df, {
                'planning_time': 'Planning Time (μs)',
                'shortcut_time': 'Shortcut Time (μs)',
                'total_time': 'Total Time (μs)',
                'planning_iterations': 'Planning Iters.',
                'avg_time_per_iteration': 'Time per Iter. (μs)',
                }
            )

        mbm.print_stats_table(
            df, {
                'shortcut_path_cost': 'Shortcut Path Cost (SE3)',
                'shortcut_path_vertices': 'Shortcut Path Vertices',
                }
            )

    print(
        f"Solved / IK-Resolved / Valid / Total # Problems: "
        f"{valid_problems - failed_problems} / {valid_problems} / "
        f"{valid_problems + unresolvable_problems} / {total_problems}"
        )
    if not df.empty:
        print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
