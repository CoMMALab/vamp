import pickle
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
    **kwargs,
    ):

    if not robot.endswith(".flask"):
        raise RuntimeError(f"Robot {robot} is not a flask robot!")

    # Ambient geometric parent: shares the URDF/sphere model; used for problems and
    # state display. The flask robot is its nested submodule.
    base_robot = robot[:-len(".flask")]
    if base_robot not in vamp.robots:
        raise RuntimeError(f"Robot {base_robot} does not exist in VAMP!")
    geo_module = getattr(vamp, base_robot)

    if rho is not None:
        geo_module.flask.set_rho(float(rho))

    robot_dir = Path(__file__).parent.parent / 'resources' / base_robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )

    n_q = vamp_module.flat_dimension()
    velocity_limits = np.array(vamp_module.velocity_limits())
    effort_limits = np.array(vamp_module.effort_limits())

    def lift(q):
        """Lift a configuration q to the rest flat state z = (q, 0)."""
        return np.concatenate([np.asarray(q, dtype = np.float32),
                               np.zeros(n_q, dtype = np.float32)])

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

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    if valid:
        result = planner_func(lift(start), [lift(goal) for goal in goals], env, plan_settings, sampler)
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
        simplify = vamp_module.simplify(result.path, env, simp_settings, sampler)

        stats = vamp.results_to_dict(result, simplify)

        # Reconstruct the time-parameterized cubic trajectory from the z waypoints
        t, q, qd, _, tau = flask.densify(vamp_module, simplify.path, 64)
        length = float(np.sum(np.linalg.norm(np.diff(q, axis = 0), axis = 1)))

        print(
            f"""
Planning Time: {stats['planning_time'].microseconds:8d}μs
Simplify Time: {stats['simplification_time'].microseconds:8d}μs
   Total Time: {stats['total_time'].microseconds:8d}μs

Planning Iters: {stats['planning_iterations']}
n Graph States: {stats['planning_graph_size']}

Path Cost (z-space L2):
   Initial: {stats['initial_path_cost']:5.3f}
Simplified: {stats['simplified_path_cost']:5.3f}

Trajectory Duration: {t[-1]:5.3f}s
  Trajectory Length: {length:5.3f}rad
 Max Velocity Ratio: {np.max(np.abs(qd) / velocity_limits):5.3f}
   Max Effort Ratio: {np.max(np.abs(tau) / effort_limits):5.3f}"""
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
