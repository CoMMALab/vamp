"""Motion Benchmaker evaluation using EnvironmentCollisionConstraint instead of (or in
addition to) ordinary hard collision checking: the planner treats "stay `margin` away from
every obstacle" as a manifold constraint (see
vamp::planning::constraint::EnvironmentCollisionConstraint / cricket's "env_collision"
recipe key), projecting samples and connect attempts back off the environment using one
representative ("middle") sphere per link rather than checking every sphere.

Only robots generated with `"env_collision": true` in their cricket recipe expose
`EnvironmentCollisionConstraint`; run cricket's fkcc_gen with that key set (and rebuild
vamp) for the robot you want to benchmark before using this script -- see
scripts/evaluate_mbm.py for the equivalent hard-collision-checking benchmark, and
scripts/digit_example.py for the general constrained-planning call pattern this mirrors.

Note: EnvironmentCollisionConstraint's signed-distance query only covers primitive
geometry (spheres, capsules, cuboids, heightfields) -- point-cloud obstacles contribute no
gradient (see sphere_environment_min_distance_gradient in validity.hh), so this script does
not support `pointcloud=True` environments.
"""

import time
from tqdm import tqdm
import numpy as np
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp
from vamp import mbm

PROJ_METHODS = {
    "InnerLM": vamp.ProjMethod.InnerLM,
    "OuterLM": vamp.ProjMethod.OuterLM,
    "GradDesc": vamp.ProjMethod.GradDesc,
}


def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    margin: float = 0.05,                  # EnvironmentCollisionConstraint distance margin
    method: str = "InnerLM",               # Projection method: InnerLM, OuterLM, GradDesc
    descend_rate: float = 1.0,             # Step size along the projection gradient
    projection_iterations: int = 10,       # Iteration cap for a single projection
    seed_projection_iterations: int = 100, # Iteration cap for projecting start/goal seeds
    emit_all_waypoints: bool = False,      # Emit every waypoint of a projected chain
    connect_slack: float = 2.0,            # Greedy-connect step budget multiplier (unconstrained default: 1.0)
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    problems, problem = mbm.load_problems(robot, dataset, problem)

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    if not hasattr(vamp_module, "EnvironmentCollisionConstraint"):
        raise RuntimeError(
            f"{robot} was not generated with cricket's `env_collision` recipe key -- "
            "EnvironmentCollisionConstraint is unavailable for this robot. Regenerate its "
            "header with `\"env_collision\": true` in the recipe and rebuild vamp."
            )

    if method not in PROJ_METHODS:
        raise RuntimeError(f"Unknown projection method `{method}`: expected one of {list(PROJ_METHODS)}")

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.method = PROJ_METHODS[method]
    constraint_settings.descend_rate = descend_rate
    constraint_settings.max_iterations = projection_iterations
    constraint_settings.emit_all_waypoints = emit_all_waypoints
    # Disable steer()'s per-lane perturbation (default 0.1): every extend attempt uses only
    # the unperturbed direct-line target (lane 0), same single-candidate behavior as the
    # unconstrained local planner, instead of trying rake-1 additional perturbed fallbacks.
    constraint_settings.perturbation_scale = 0.1
    constraint_settings.connect_slack = connect_slack

    # Seed projection (start/goal) gets its own, more generous iteration budget, same
    # policy as digit_example.py: seeds must land on the manifold before planning starts.
    seed_settings = vamp.ConstraintSettings()
    seed_settings.method = PROJ_METHODS[method]
    seed_settings.max_iterations = seed_projection_iterations
    seed_settings.descend_rate = descend_rate


    # plan_settings.range=1.25

    sampler = getattr(vamp_module, sampler)()

    if hasattr(vamp_module, "reset_constrained_edge_stats"):
        vamp_module.reset_constrained_edge_stats()

    total_problems = 0
    valid_problems = 0
    failed_problems = 0
    all_rejected_seeds = []  # (problem index, 'start'/'goal', squared_error), across all problems

    tick = time.perf_counter()
    results = []
    for name, pset in problems.items():
        if name not in problem:
            continue

        failures = []
        invalids = []
        rejected_seeds = []  # (problem index, 'start'/'goal', squared_error) for each rejection
        print(f"Evaluating {robot} on {name}: ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            env = vamp.problem_dict_to_vamp(data)
            constraints = [vamp_module.EnvironmentCollisionConstraint(env, margin)]

            start = np.asarray(data['start'], dtype = np.float32)
            goals = [np.asarray(goal, dtype = np.float32) for goal in data['goals']]

            # A ValueError here just means projection didn't converge within
            # seed_projection_iterations -- report how far off the manifold the seed
            # actually was (summed squared violation, in the "distance - margin" units of
            # each violated env-collision row) rather than only counting the failure.
            rejected = False
            try:
                start_c = vamp_module.project(start, constraints, seed_settings)
            except ValueError:
                violation = vamp_module.squared_error(start, constraints, seed_settings)
                rejected_seeds.append((i, 'start', violation))
                rejected = True

            goals_c = []
            for goal in goals:
                try:
                    goals_c.append(vamp_module.project(goal, constraints, seed_settings))
                except ValueError:
                    violation = vamp_module.squared_error(goal, constraints, seed_settings)
                    rejected_seeds.append((i, 'goal', violation))
                    rejected = True

            if rejected:
                failures.append(i)
                continue

            sampler.reset()
            sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = planner_func(
                    start_c, goals_c, env, plan_settings, sampler,
                    constraints = constraints, constraint_settings = constraint_settings)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(
                    result.path, env, simp_settings, sampler,
                    constraints = constraints, constraint_settings = constraint_settings)

                results.append(vamp.results_to_dict(result, simple))

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if rejected_seeds:
                print("  Seed projection failed (index, start/goal, squared violation):")
                for i, which, violation in rejected_seeds:
                    print(f"    [{i}] {which}: squared_error = {violation:.6f}"
                          f" (L2 = {violation ** 0.5:.4f})")

            if failures:
                print(f"  Failed on {failures}")

        all_rejected_seeds.extend(rejected_seeds)

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]
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

    if all_rejected_seeds:
        violations = np.array([v for _, _, v in all_rejected_seeds])
        l2 = violations ** 0.5
        print(
            f"\nSeed projection rejected {len(all_rejected_seeds)} start/goal(s) "
            f"(valid under hard collision checking, but violate margin={margin}): "
            f"squared_error min/mean/max = {violations.min():.6f} / {violations.mean():.6f} / "
            f"{violations.max():.6f} (L2 min/mean/max = {l2.min():.4f} / {l2.mean():.4f} / {l2.max():.4f})"
            )
        print("Pass --print_failures to see the per-problem breakdown, or lower --margin.")

    if hasattr(vamp_module, "constrained_edge_stats"):
        sdf_ok, sdf_fail, fkcc_fail_after_sdf_ok = vamp_module.constrained_edge_stats()
        pct = f" ({100 * fkcc_fail_after_sdf_ok / sdf_ok:.2f}% of SDF successes)" if sdf_ok else ""
        print(
            f"\nEdge checks -- SDF projection succeeded: {sdf_ok}, failed: {sdf_fail}; "
            f"collision-checking failed despite SDF success: {fkcc_fail_after_sdf_ok}{pct}"
            )

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
