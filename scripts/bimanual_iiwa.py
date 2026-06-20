import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire

q_tilde_bottom = np.array([-0.6430910102907225, 1.9156121024586796, -1.7968254667817805, 1.2945447141185198, -0.023834531305537934, -0.876966810663043, -1.7041643160834519, 1.45])
q_tilde_middle = np.array([-0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483, -0.04421061906813227, -0.8793712572715165, -1.1603461715511334, 1.45])
q_tilde_top = np.array([-0.1994994216078726, 0.9140739951190965, -2.236618320862171, 0.5238879195899456, 0.7998441913611017, -1.3575398006936048, -1.0153092816310436, 2.41])

# Problem specification: a list of sphere centers
problem = [
    # [0.55, 0, 0.25],
    # [0.35, 0.35, 0.25],
    # [0, 0.55, 0.25],
    # [-0.55, 0, 0.25],
    # [-0.35, -0.35, 0.25],
    # [0, -0.55, 0.25],
    # [0.35, -0.35, 0.25],
    # [0.35, 0.35, 0.8],
    # [0, 0.55, 0.8],
    # [-0.35, 0.35, 0.8],
    # [-0.55, 0, 0.8],
    # [-0.35, -0.35, 0.8],
    # [0, -0.55, 0.8],
    # [0.35, -0.35, 0.8],
    ]


def main(
    variation: float = 0.01,
    benchmark: bool = True,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = False,
    planner: str = "rrtc",
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("bimanualiiwa", planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    cuboids_file = Path(__file__).parent.parent / "resources" / "iiwa" / "cuboids" / "shelf_drake.txt"
    cuboids = np.loadtxt(cuboids_file, delimiter=",")

    if benchmark:
        random.seed(0)
        np.random.seed(0)

        results = []
        for _ in range(n_trials):
            e = vamp.Environment()
            for cuboid in cuboids:
                e.add_cuboid(vamp.Cuboid(cuboid[:3], np.array([0.0, 0.0, 0.0]), cuboid[3:6]/2))

            # print("Check if all three are valid in the first place")
            # print(vamp.bimanualiiwa.validate(q_tilde_bottom, e))
            # print(vamp.bimanualiiwa.validate(q_tilde_middle, e))
            # print(vamp.bimanualiiwa.validate(q_tilde_top, e))

            # check the parameterization of the three configurations
            # print(vamp.bimanualiiwa.parameterized_ik(q_tilde_top)[1])
            # print(vamp.bimanualiiwa.parameterized_ik(q_tilde_middle)[1])
            # print(vamp.bimanualiiwa.parameterized_ik(q_tilde_bottom)[1])

            # vamp.bimanualiiwa.set_ik_parameters([1.0, 1.0, -1.0, 0.6])


            # pick two random configurations of the three and make sure they are different
            start = random.choice([q_tilde_bottom, q_tilde_middle, q_tilde_top])
            goal = random.choice([q_tilde_bottom, q_tilde_middle, q_tilde_top])
            while np.allclose(start, goal):
                goal = random.choice([q_tilde_bottom, q_tilde_middle, q_tilde_top])



            if vamp.bimanualiiwa.validate(start, e) and vamp.bimanualiiwa.validate(goal, e):
                result = planner_func(start, goal, e, plan_settings, sampler)
                simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
                results.append(vamp.results_to_dict(result, simple))

        df = pd.DataFrame.from_dict(results)

        # Convert to microseconds
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["simplification_time"] = df["simplification_time"].dt.microseconds

        # Get summary statistics
        stats = df[[
            "planning_time",
            "simplification_time",
            "initial_path_cost",
            "simplified_path_cost",
            "planning_iterations"
            ]].describe()

        print(stats)


if __name__ == "__main__":
    Fire(main)
