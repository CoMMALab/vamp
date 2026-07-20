import time
from tqdm import tqdm
import numpy as np
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp
from vamp import mbm
from vamp import pointcloud as vpc


def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    structure: str = "capt",               # Pointcloud collision structure to use (capt or mvt)
    samples_per_object: int = 10000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = True,              # Cull pointcloud around robot by maximum distance
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    problems, problem = mbm.load_problems(robot, dataset, problem)

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    sampler = getattr(vamp_module, sampler)()

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
        print(f"Evaluating {robot} on {name}: ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            if pointcloud:
                r_min, r_max = vamp_module.min_max_radii()
                (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
                    robot,
                    r_min,
                    r_max,
                    data,
                    samples_per_object,
                    filter_radius,
                    filter_cull,
                    structure,
                    )

                pointcloud_results = {
                    'original_pointcloud_size': len(original_pc),
                    'filtered_pointcloud_size': len(filtered_pc),
                    'filter_time': pd.Timedelta(nanoseconds = filter_time),
                    'build_time': pd.Timedelta(nanoseconds = build_time)
                    }
            else:
                env = vamp.problem_dict_to_vamp(data)

            start = np.asarray(data['start'], dtype = np.float32)
            goals = [np.asarray(goal, dtype = np.float32) for goal in data['goals']]

            sampler.reset()
            sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = planner_func(start, goals, env, plan_settings, sampler)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(result.path, env, simp_settings, sampler)

                trial_result = vamp.results_to_dict(result, simple)
                if pointcloud:
                    trial_result.update(pointcloud_results)

                results.append(trial_result)

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]

    # Pointcloud data
    if pointcloud:
        df["total_build_and_plan_time"] = df["total_time"] + df["filter_time"] + df["build_time"]
        df["filter_time"] = df["filter_time"].dt.microseconds / 1e3
        df["build_time"] = df["build_time"].dt.microseconds / 1e3
        df["total_build_and_plan_time"] = df["total_build_and_plan_time"].dt.microseconds / 1e3

    df["total_time"] = df["total_time"].dt.microseconds

    mbm.print_stats_table(
        df, {
            'planning_time': 'Planning Time (μs)',
            'simplification_time': 'Simplification Time (μs)',
            'total_time': 'Total Time (μs)',
            'planning_iterations': 'Planning Iters.',
            'avg_time_per_iteration': 'Time per Iter. (μs)',
            }
        )

    mbm.print_stats_table(
        df, {
            'initial_path_cost': ' Initial Cost (L2)',
            'simplified_path_cost': '    Simplified Cost (L2)',
            }
        )

    if pointcloud:
        mbm.print_stats_table(
            df, {
                'filter_time': '  Filter Time (ms)',
                'build_time': f'    {structure.upper()} Build Time (ms)',
                'total_build_and_plan_time': 'Total Time (ms)',
                }
            )

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
