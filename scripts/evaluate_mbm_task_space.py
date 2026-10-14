"""MBM evaluation with task-space goals: the goal configuration is only used to compute an exact SE(3)
goal pose (forward kinematics), then TaskSpaceRRTC plans from the start to that pose with no goal
configuration. Same dataset handling and reporting as evaluate_mbm.py.

Usage: python scripts/evaluate_mbm_task_space.py --problem=cage --method=BoundedLS
"""
import time
from typing import List, Union

import numpy as np
import pandas as pd
from fire import Fire
from tqdm import tqdm

import vamp
from vamp import mbm


def rotation_to_quaternion(R):
    """(qw, qx, qy, qz) of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        return [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]

    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0] * 4
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def goal_region(vamp_module, goal):
    """Exact TSR at the end-effector pose of `goal`: identity offset, zero-width bounds."""
    T = np.asarray(vamp_module.eefk(goal), dtype=np.float64)
    pose = [*rotation_to_quaternion(T[:3, :3]), *T[:3, 3]]
    return vamp_module.TaskSpaceConstraint([1, 0, 0, 0, 0, 0, 0], pose, [0.0] * 3 + [-0.0] * 3, [0.0] * 3 + [0.0] * 3)


def main(
    robot: str = "panda",                  # Robot to plan for
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    method: str = "InnerLM",               # Projection method: InnerLM, OuterLM, GradDesc, BoundedLS
    range_: float = 0.5,                   # Maximum step length of the planner
    max_greedy_steps: int = 20,
    max_iterations: int = 1000000,
    max_samples: int = 1000000,
    simplify: bool = False,                # Simplify the path (unconstrained at the end point)
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    problems, problem = mbm.load_problems(robot, dataset, problem)

    # Only used for the simplification settings
    (vamp_module, _, _, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, "rrtc", **kwargs)

    sampler = getattr(vamp_module, sampler)()

    plan_settings = vamp.TaskSpaceRRTCSettings()
    plan_settings.range = range_
    plan_settings.max_greedy_steps = max_greedy_steps
    plan_settings.max_iterations = max_iterations
    plan_settings.max_samples = max_samples

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.method = getattr(vamp.ProjMethod, method)

    total_problems = 0
    valid_problems = 0
    failed_problems = 0

    tick = time.perf_counter()
    results = []
    for name, pset in problems.items():
        if name not in problem:
            continue

        failures = []
        invalids = []
        print(f"Evaluating {robot} on {name} (task-space goals, {method}): ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            env = vamp.problem_dict_to_vamp(data)
            start = np.asarray(data['start'], dtype=np.float32)
            goal = np.asarray(data['goals'][0], dtype=np.float32)
            tsr = goal_region(vamp_module, goal)

            sampler.reset()
            sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = vamp_module.task_space_rrtc(
                    start, [tsr], env, plan_settings, sampler, constraint_settings)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(result.path, env, simp_settings, sampler) if simplify else result
                results.append(vamp.results_to_dict(result, simple))

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")

    tock = time.perf_counter()

    if not results:
        print(f"No problems solved ({failed_problems} failed); nothing to report.")
        return

    df = pd.DataFrame.from_dict(results)

    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]
    df["total_time"] = df["total_time"].dt.microseconds

    mbm.print_stats_table(
        df, {
            'planning_time': 'Planning Time (μs)',
            'total_time': 'Total Time (μs)',
            'planning_iterations': 'Planning Iters.',
            'avg_time_per_iteration': 'Time per Iter. (μs)',
            }
        )

    mbm.print_stats_table(df, {'initial_path_cost': ' Initial Cost (L2)', 'simplified_path_cost': '    Simplified Cost (L2)'})

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
