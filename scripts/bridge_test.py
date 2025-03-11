import pickle
from pathlib import Path
import numpy as np

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb
from vamp import pointcloud as vpc


def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: str = "",                     # Problem name
    index: int = 1,                        # Problem index
    sampler_name: str = "halton",          # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    display_object_names: bool = False,    # Display object names over geometry
    samples: int = 100,                    # Number of bridge test samples
    bridge_range: float = 0.01,            # Scale of logistic distribution to use in bridge test
    max_attempts: int = 10000,             # Number of attempts to use for bridge test sampling
    **kwargs,
    ):

    if robot not in vamp.ROBOT_JOINTS:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )

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

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    bridge_samples = list(
        vamp_module.batch_bridge_test_sample(samples, bridge_range, max_attempts, sampler, env)
        )

    print(f"Got {len(bridge_samples)} samples!")

    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp.ROBOT_JOINTS[robot], True)
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    sim.animate(bridge_samples)


if __name__ == "__main__":

    Fire(main)
