import pickle
import time
from pathlib import Path
from typing import Union
import numpy as np

from fire import Fire

import vamp
from vamp import flask
from vamp import pybullet_interface as vpb


def main(
    robot: str = "panda.flask",            # Flask robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: str = "",                     # Problem name
    index: int = 1,                        # Problem index
    sampler_name: str = "halton",          # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    display_object_names: bool = False,    # Display object names over geometry
    fps: float = 60.0,                     # Animation frames per second of trajectory time
    rho: Union[float, None] = None,        # Override the LQMT cost weight rho for this robot
    retime: bool = False,                  # Plan with the ambient geometric parent, then retime
    max_kinetic_energy: Union[float, None] = None,    # Phase constraint: kinetic energy cap (J)
    max_eef_speed: Union[float, None] = None,         # Phase constraint: end-effector speed cap (m/s)
    sample_energy: Union[float, None] = None,         # Shape sampled kinetic energy to [0, cap] J
    **kwargs,
    ):

    # Ambient geometric parent: shares the URDF/sphere model; used for problems and
    # state display. The flask robot is its nested submodule.
    base_robot, geo_module = flask.parse_flask_robot(robot, rho)

    robot_dir = Path(__file__).parent.parent / 'resources' / base_robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )

    # With --retime the ambient parent plans and simplifies the geometric path; the
    # flask sibling only retimes the lifted path with its own simplifier.
    plan_module, flask_simp_settings = vamp_module, simp_settings
    if retime:
        (plan_module, planner_func, plan_settings,
         simp_settings) = vamp.configure_robot_and_planner_with_kwargs(base_robot, planner, **kwargs)
        flask_simp_settings.operations = flask.RETIME_OPERATIONS

    n_q = vamp_module.flat_dimension()
    velocity_limits = np.array(vamp_module.velocity_limits())
    effort_limits = np.array(vamp_module.effort_limits())

    phase_constraints = flask.phase_constraints(vamp_module, max_kinetic_energy, max_eef_speed)
    phase_kwargs = {"phase_constraints": phase_constraints} if phase_constraints else {}

    if not problem:
        problem = list(data['problems'].keys())[0]

    if problem not in data['problems']:
        raise RuntimeError(
            f"""No problem with name {problem}!
Existing problems: {list(data['problems'].keys())}"""
            )

    problems = data['problems'][problem]
    try:
        problem_data = next(problem for problem in problems if problem['index'] == index)
    except StopIteration:
        raise RuntimeError(f"No problem in {problem} with index {index}!")

    env = vamp.problem_dict_to_vamp(problem_data)

    start = np.asarray(problem_data['start'], dtype = np.float32)
    goals = [np.asarray(goal, dtype = np.float32) for goal in problem_data['goals']]
    valid = problem_data['valid']

    sampler = getattr(plan_module, sampler_name)()
    if sample_energy is not None and not retime:
        sampler = vamp_module.ke_shaped(sampler, float(sample_energy))
    sampler.skip(skip_rng_iterations)

    flask_sampler = None
    if retime:
        flask_sampler = getattr(vamp_module, sampler_name)()
        flask_sampler.skip(skip_rng_iterations)

    if valid:
        if retime:
            result = planner_func(start, goals, env, plan_settings, sampler)
        else:
            result = planner_func(
                flask.rest_state(start), [flask.rest_state(goal) for goal in goals],
                env, plan_settings, sampler, **phase_kwargs
                )
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
        simplify = plan_module.simplify(
            result.path, env, simp_settings, sampler, **({} if retime else phase_kwargs)
            )

        stats = vamp.results_to_dict(result, simplify)

        flask_path = simplify.path
        if retime:
            retime_tick = time.perf_counter()
            lifted = flask.lift(vamp_module, simplify.path)
            flask_simple = vamp_module.simplify(
                lifted, env, flask_simp_settings, flask_sampler, **phase_kwargs
                )
            retime_time = time.perf_counter() - retime_tick

            flask_path = flask_simple.path
            retime_valid = flask_path.validate(env)
            print(
                f"Retime: {len(lifted)} -> {len(flask_path)} states, "
                f"C_loc {lifted.cost():5.3f} -> {flask_path.cost():5.3f}, "
                f"{'valid' if retime_valid else 'INVALID'}, {retime_time * 1e3:5.1f}ms"
                )

        # Reconstruct the time-parameterized cubic trajectory from the z waypoints
        t, q, qd, _, tau = flask.densify(vamp_module, flask_path, 64)
        length = float(np.sum(np.linalg.norm(np.diff(q, axis = 0), axis = 1)))

        cost_space = "geometric L2" if retime else "z-space L2"
        print(
            f"""
Planning Time: {stats['planning_time'].microseconds:8d}μs
Simplify Time: {stats['simplification_time'].microseconds:8d}μs
   Total Time: {stats['total_time'].microseconds:8d}μs

Planning Iters: {stats['planning_iterations']}
n Graph States: {stats['planning_graph_size']}

Path Cost ({cost_space}):
   Initial: {stats['initial_path_cost']:5.3f}
Simplified: {stats['simplified_path_cost']:5.3f}

Trajectory Duration: {t[-1]:5.3f}s
  Trajectory Length: {length:5.3f}rad
 Max Velocity Ratio: {np.max(np.abs(qd) / velocity_limits):5.3f}
   Max Effort Ratio: {np.max(np.abs(tau) / effort_limits):5.3f}"""
            )

        if phase_constraints:
            # Enforcement is at the planner's validation samples, so densified samples
            # between them may peak slightly above the caps.
            zs = np.hstack([q, qd]).astype(np.float32)
            max_ke = max(float(vamp_module.kinetic_energy(z)) for z in zs)
            max_ee = max(flask.max_eef_speed(vamp_module, z) for z in zs)
            ke_cap = f" (cap {max_kinetic_energy:g}J)" if max_kinetic_energy is not None else ""
            ee_cap = f" (cap {max_eef_speed:g}m/s)" if max_eef_speed is not None else ""
            print(
                f" Max Kinetic Energy: {max_ke:5.3f}J{ke_cap}\n"
                f"      Max EEF Speed: {max_ee:5.3f}m/s{ee_cap}"
                )

            # 5% slack so marginal densified-sample peaks don't trip the warning.
            violated = (max_kinetic_energy is not None and max_ke > 1.05 * max_kinetic_energy) \
                or (max_eef_speed is not None and max_ee > 1.05 * max_eef_speed)
            if retime and violated:
                print(
                    "WARNING: retimed trajectory violates the phase constraints. Gates only"
                    " reject simplify moves; they cannot slow the lifted input path, whose"
                    " speed profile is fixed by rho. Lower rho to slow the base motion."
                    )

        # Resample uniformly in time so playback speed follows the velocity profile
        frames = np.arange(0.0, t[-1], 1.0 / fps)
        plan = [
            np.array([np.interp(f, t, q[:, j]) for j in range(n_q)], dtype = np.float32)
            for f in frames
            ]
        plan.append(q[-1].astype(np.float32))

    if valid and not solved:
        print("Failed to solve problem! Displaying start and goals.")
        print(start)
        for goal in goals:
            print(goal)

        print(
            f"""
Planning Time: {int(result.nanoseconds / 1000):8d}μs
Planning Iters: {result.iterations}
n Graph States: {result.size}
"""
            )

    if not solved:
        plan = [start, *goals]

    sim = vpb.PyBulletSimulator(
        str(robot_dir / f"{base_robot}_spherized.urdf"), vamp_module.joint_names(), True
        )
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    if not valid:
        for state in [start, *goals]:
            if not geo_module.validate(state, env):
                print(f"Displaying colliding spheres for first invalid state: {state}")
                debug = geo_module.debug(state, env)
                invalid = set([x[0] for x in filter(lambda x: x[1], enumerate(debug[0]))])

                for (a, b) in debug[1]:
                    invalid.add(a)
                    invalid.add(b)

                spheres = geo_module.fk(state)
                for i in range(len(spheres)):
                    sphere = spheres[i]
                    if i in invalid:
                        sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 0., 0., 1.])
                    else:
                        sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 1., 1., 1.])

                break

    sim.animate(plan)


if __name__ == "__main__":

    Fire(main)
