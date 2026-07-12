import pickle
import time
from tabulate import tabulate
from tqdm import tqdm
from pathlib import Path
import numpy as np
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp
from vamp import flask


def main(
    robot: str = "panda.flask",            # Flask robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    samples_per_segment: int = 64,         # Trajectory samples per segment for post-hoc checks
    rho: Union[float, None] = None,        # Override the LQMT cost weight rho for this robot
    **kwargs,
    ):

    if not robot.endswith(".flask"):
        raise RuntimeError(f"Robot {robot} is not a flask robot!")

    # Ambient geometric parent: shares the URDF/sphere model; used for problems and
    # post-hoc collision checks. The flask robot is its nested submodule.
    base_robot = robot[:-len(".flask")]
    if base_robot not in vamp.robots:
        raise RuntimeError(f"Robot {base_robot} does not exist in VAMP!")
    geo_module = getattr(vamp, base_robot)

    if rho is not None:
        geo_module.flask.set_rho(float(rho))

    problems_dir = Path(__file__).parent.parent / 'resources' / base_robot / 'problems'
    with open(problems_dir.parent / dataset, 'rb') as f:
        problems = pickle.load(f)

    problem_names = list(problems['problems'].keys())
    if isinstance(problem, str):
        problem = [problem]

    if not problem:
        problem = problem_names
    else:
        for problem_name in problem:
            if problem_name not in problem_names:
                raise RuntimeError(
                    f"Problem `{problem_name}` not available! Available problems: {problem_names}"
                    )

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    n_q = vamp_module.flat_dimension()
    velocity_limits = np.array(vamp_module.velocity_limits())
    effort_limits = np.array(vamp_module.effort_limits())

    def lift(q):
        """Lift a configuration q to the rest flat state z = (q, 0)."""
        return np.concatenate([np.asarray(q, dtype = np.float32),
                               np.zeros(n_q, dtype = np.float32)])

    sampler = getattr(vamp_module, sampler)()

    total_problems = 0
    valid_problems = 0
    failed_problems = 0

    tick = time.perf_counter()
    results = []
    for name, pset in problems['problems'].items():
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

            env = vamp.problem_dict_to_vamp(data)

            start = lift(data['start'])
            goals = [lift(goal) for goal in data['goals']]

            sampler.reset()
            sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = planner_func(start, goals, env, plan_settings, sampler)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(result.path, env, simp_settings, sampler)

                trial_result = vamp.results_to_dict(result, simple)

                # Post-hoc trajectory checks: densely resample the cubic trajectory and
                # verify collision-freedom (geometric twin) and dynamic limits
                t, q, qd, _, tau = flask.densify(vamp_module, simple.path, samples_per_segment)
                collided = not all(
                    geo_module.validate(qi.astype(np.float32), env) for qi in q
                    )

                path_cost = float(
                    sum(
                        vamp_module.cost(
                            np.asarray(simple.path[k], np.float32),
                            np.asarray(simple.path[k + 1], np.float32),
                            ) for k in range(len(simple.path) - 1)
                        )
                    )

                trial_result.update(
                    {
                        'problem': name,
                        'trajectory_duration': t[-1],
                        'trajectory_cost': path_cost,
                        'trajectory_length': float(
                            np.sum(np.linalg.norm(np.diff(q, axis = 0), axis = 1))
                            ),
                        'collision_risk': float(collided),
                        'max_velocity_ratio': float(np.max(np.abs(qd) / velocity_limits)),
                        'max_effort_ratio': float(np.max(np.abs(tau) / effort_limits)),
                        }
                    )

                results.append(trial_result)

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to milliseconds to match the FLASK paper's tables
    for field in ["planning_time", "simplification_time", "total_time"]:
        df[field] = df[field].dt.total_seconds() * 1e3

    percentiles = [0.25, 0.5, 0.75, 0.95]
    paper_metrics = {
        'total_time': 'Total planning time (ms)',
        'trajectory_length': 'Final traj. length (rad)',
        'trajectory_duration': 'Trajectory duration (s)',
        'trajectory_cost': 'Trajectory cost (C_loc)',
        'collision_risk': 'Collision risk',
        }

    def describe(sub: pd.DataFrame) -> pd.DataFrame:
        stats = sub[list(paper_metrics)].describe(percentiles = percentiles)
        stats = stats.drop(index = ["count", "min", "max"]).T
        stats.index = [paper_metrics[m] for m in stats.index]
        return stats

    for name in problem:
        sub = df[df['problem'] == name]
        if sub.empty:
            continue

        print(f"\n{name} ({len(sub)} trials)")
        print(tabulate(describe(sub), headers = 'keys', tablefmt = 'github', floatfmt = '.2f'))

    print(f"\nAll environments ({len(df)} trials)")
    print(tabulate(describe(df), headers = 'keys', tablefmt = 'github', floatfmt = '.2f'))

    time_stats = df[[
        "planning_time",
        "simplification_time",
        "total_time",
        "planning_iterations",
        ]].describe(percentiles = percentiles)
    time_stats.drop(index = ["count"], inplace = True)

    print()
    print(
        tabulate(
            time_stats,
            headers = [
                'Planning Time (ms)',
                'Simplification Time (ms)',
                'Total Time (ms)',
                'Planning Iters.',
                ],
            tablefmt = 'github'
            )
        )

    print(
        f"\nMax velocity / effort limit ratio over all trajectories: "
        f"{df['max_velocity_ratio'].max():.3f} / {df['max_effort_ratio'].max():.3f}"
        )
    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum():.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
