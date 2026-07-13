"""Evaluate flask kinodynamic planning on MBM. Two pipelines share the metrics,
so the tables are directly comparable:

- Direct (default): plan and simplify with the flask robot itself, from rest
  states, entirely in phase space.
- Plan-then-retime (--retime): plan with the ambient geometric parent, simplify,
  lift the path to flask rest states (vamp.flask.lift), then retime it with the
  flask robot's own simplify -- shortcut states sampled along segment interiors
  carry the LQMT cubic's velocities, so accepted shortcuts (validated at full
  resolution, scored by C_loc) blend waypoint stops into flowing motion."""

import time
from tqdm import tqdm
import numpy as np
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp
from vamp import flask, mbm


def main(
    robot: str = "panda.flask",            # Flask robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    retime: bool = False,                  # Plan with the ambient geometric parent, then retime
    samples_per_segment: int = 64,         # Trajectory samples per segment for post-hoc checks
    rho: Union[float, None] = None,        # Override the LQMT cost weight rho for this robot
    sample_energy: Union[float, None] = None,  # Shape sampled kinetic energy to [0, cap] J (direct only)
    **kwargs,
    ):

    # Ambient geometric parent: shares the URDF/sphere model; used for problems and
    # post-hoc collision checks. The flask robot is its nested submodule.
    base_robot, geo_module = flask.parse_flask_robot(robot, rho)

    problems, problem = mbm.load_problems(base_robot, dataset, problem)

    (flask_module, planner_func, plan_settings,
     flask_simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    # With --retime the ambient parent plans and simplifies the geometric path; the
    # flask sibling only retimes the lifted path with its own simplifier.
    plan_module, simp_settings = flask_module, flask_simp_settings
    if retime:
        (plan_module, planner_func, plan_settings,
         simp_settings) = vamp.configure_robot_and_planner_with_kwargs(base_robot, planner, **kwargs)
        flask_simp_settings.operations = flask.RETIME_OPERATIONS

    velocity_limits = np.array(flask_module.velocity_limits())
    effort_limits = np.array(flask_module.effort_limits())

    plan_sampler = getattr(plan_module, sampler)()
    flask_sampler = getattr(flask_module, sampler)() if retime else None
    if sample_energy is not None and not retime:
        plan_sampler = flask_module.ke_shaped(plan_sampler, float(sample_energy))

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
        retime_failures = []
        tag = f"{robot} (retime after {base_robot} {planner})" if retime else robot
        print(f"Evaluating {tag} on {name}: ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            env = vamp.problem_dict_to_vamp(data)

            if retime:
                start = np.asarray(data['start'], dtype = np.float32)
                goals = [np.asarray(goal, dtype = np.float32) for goal in data['goals']]
            else:
                start = flask.rest_state(data['start'])
                goals = [flask.rest_state(goal) for goal in data['goals']]

            plan_sampler.reset()
            plan_sampler.skip(skip_rng_iterations)
            if retime:
                flask_sampler.reset()
                flask_sampler.skip(skip_rng_iterations)

            for _ in range(trials):
                result = planner_func(start, goals, env, plan_settings, plan_sampler)
                if not result.solved:
                    failures.append(i)
                    break

                simple = plan_module.simplify(result.path, env, simp_settings, plan_sampler)

                trial_result = vamp.results_to_dict(result, simple)

                flask_path = simple.path
                if retime:
                    retime_tick = time.perf_counter()
                    lifted = flask.lift(flask_module, simple.path)
                    rest_duration = float(np.sum(flask.segment_times(flask_module, lifted)))
                    flask_simple = flask_module.simplify(
                        lifted, env, flask_simp_settings, flask_sampler)
                    retime_time = time.perf_counter() - retime_tick

                    flask_path = flask_simple.path
                    retime_valid = flask_path.validate(env)
                    if not retime_valid:
                        retime_failures.append(i)

                    trial_result.update(
                        {
                            'retime_time': pd.Timedelta(seconds = retime_time),
                            'retime_valid': float(retime_valid),
                            'rest_duration': rest_duration,
                            }
                        )

                # Post-hoc trajectory checks: densely resample the cubic trajectory and
                # verify collision-freedom (geometric twin) and dynamic limits
                t, q, qd, _, tau = flask.densify(flask_module, flask_path, samples_per_segment)
                collided = not all(
                    geo_module.validate(qi.astype(np.float32), env) for qi in q
                    )
                max_jerk, rms_jerk, max_accel_jump = flask.jerk_metrics(flask_module, flask_path)

                trial_result.update(
                    {
                        'problem': name,
                        'trajectory_duration': t[-1],
                        'trajectory_cost': flask.path_cost(flask_module, flask_path),
                        'trajectory_length': float(
                            np.sum(np.linalg.norm(np.diff(q, axis = 0), axis = 1))
                            ),
                        'collision_risk': float(collided),
                        'max_velocity_ratio': float(np.max(np.abs(qd) / velocity_limits)),
                        'max_effort_ratio': float(np.max(np.abs(tau) / effort_limits)),
                        'max_jerk': max_jerk,
                        'rms_jerk': rms_jerk,
                        'max_accel_jump': max_accel_jump,
                        }
                    )

                results.append(trial_result)

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")

            if retime_failures:
                print(f"  Retime invalid on {retime_failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to milliseconds to match the FLASK paper's tables
    time_fields = ["planning_time", "simplification_time", "total_time"]
    if retime:
        time_fields.append("retime_time")
    mbm.to_milliseconds(df, time_fields)

    paper_metrics = {
        'total_time': 'Total planning time (ms)',
        'trajectory_length': 'Final traj. length (rad)',
        'trajectory_duration': 'Trajectory duration (s)',
        }
    time_columns = {
        'planning_time': 'Planning Time (ms)',
        'simplification_time': 'Simplification Time (ms)',
        }
    if retime:
        # Pipeline total: geometric plan + simplify + lift + flask simplify
        df["total_time"] = df["total_time"] + df["retime_time"]
        paper_metrics['rest_duration'] = 'Stop-at-waypoints dur. (s)'
        time_columns['retime_time'] = 'Retime Time (ms)'

    paper_metrics.update(
        {
            'trajectory_cost': 'Trajectory cost (C_loc)',
            'collision_risk': 'Collision risk',
            }
        )
    time_columns.update(
        {
            'total_time': 'Total Time (ms)',
            'planning_iterations': 'Planning Iters.',
            }
        )

    mbm.print_metric_tables(df, problem, paper_metrics)
    mbm.print_stats_table(df, time_columns)

    print()
    if retime:
        speedup = df['rest_duration'] / df['trajectory_duration']
        print(f"Retime valid: {int(df['retime_valid'].sum())} / {len(df)}")
        print(
            f"Duration speedup over stop-at-waypoints: "
            f"median {speedup.median():.2f}x, min {speedup.min():.2f}x, max {speedup.max():.2f}x"
            )
    print(
        f"Max velocity / effort limit ratio over all trajectories: "
        f"{df['max_velocity_ratio'].max():.3f} / {df['max_effort_ratio'].max():.3f}"
        )
    print(
        f"Max |jerk| (rad/s^3): median {df['max_jerk'].median():.1f}, "
        f"p95 {df['max_jerk'].quantile(0.95):.1f}, max {df['max_jerk'].max():.1f}; "
        f"RMS jerk median {df['rms_jerk'].median():.1f}"
        )
    print(
        f"Junction accel jump (rad/s^2): median {df['max_accel_jump'].median():.2f}, "
        f"p95 {df['max_accel_jump'].quantile(0.95):.2f}, max {df['max_accel_jump'].max():.2f}"
        )
    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum():.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
