import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path
import itertools
import vamp
from fire import Fire
import time

np.set_printoptions(suppress = True, precision=4)

def main(
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,

    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("g1_unitree", "crrtc", **kwargs)
    )
    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [1.,     0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0],
        [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001],
        [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        [0.08, -0.045, 0.08, 0.045, 0.06, 0.045, 0.06, -0.045],
    )


    feet_tsr_constraint = vamp_module.TaskSpaceConstraint(
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0], [1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0]],
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.001, -0.001, -0.001, -0.005, -0.005, -0.005, -0.001, -0.001, -0.001, -0.005, -0.005, -0.005],
        [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.001, 0.001, 0.001, 0.005, 0.005, 0.005, 0.001, 0.001, 0.001, 0.005, 0.005, 0.005]
    )


    constraints = vamp_module.Composable_TaskSpaceConstraint_CoMTaskSpaceConstraint_BimanualTaskSpaceConstraint(feet_tsr_constraint, com_constraint, bimanual_constraint)



    e = vamp.Environment()
    problem_cuboids = np.loadtxt('resources/environments/cuboids/humanoid_shelf.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    goal = [0.042186737, 4.2915344e-06, 0.72114706, -3.3705605e-06, 0.03055413, -2.003611e-05, -0.30831593, -0.011244297, 0.0050789025, 0.6614378, -0.3893154, 0.012345497, -0.30834094, 0.011252999, -0.0050491523, 0.66147137, -0.38932344, -0.0123702455, -2.5435329e-05, 8.368492e-06, -0.24609257, 0.008559517, -0.0059906836, -0.0037853387, -0.00089317013, 0.0042787157, -0.0032093797, 0.008020755, 0.002362628, 0.0043031597, -0.005259495, 0.0026970499, -0.0058816765, -0.00028618058, 0.014119654]

    start = [-0.005940199, -0.00012779236, 0.5253687, -0.0005400387, 0.013260534, -0.00089769263, -0.8943771, -0.007457934, 0.015044728, 1.758194, -0.87267, 0.017328043, -0.8946327, 0.010154848, -0.015723394, 1.7575748, -0.87267, -0.01812963, 1.5915728, 0.13870776, 0.37364945, 0.0016204158, -0.007157203, -0.0032037592, 0.0036042016, 0.002070119, -0.008524217, 0.0055902405, -0.0023131033, -0.0070516965, -0.0035007126, -0.0020867267, 0.0020755264, 0.0010021132, 0.005703019]

    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    c1 = constraints.projectConfiguration(np.array(start), 0, 10.0, 0.75, 50, True)
    c2 = constraints.projectConfiguration(np.array(goal), 0, 10.0, 0.75, 50, True)

    # print(", ".join(map(str, c1)))
    # print(", ".join(map(str, c2)))
    # stop


    # np.set_printoptions(precision = 4, suppress = True)
    # print(np.array(vamp_module.eefk(start)))
    # print(np.array(vamp_module.eefk(goal)))

    # robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    # server, robot = setup_viser_with_robot(robot_dir, "bipanda_spherized.urdf")
    # robot.update_cfg(start)

    # floor_grid = server.scene.add_grid(
    #     name="/floor_grid",
    #     width=10.0,
    #     height=10.0,
    #     plane="xy",
    #     position=(0.0, 0.0, 0.0),
    #     cell_color=(200, 200, 200),
    #     section_color=(140, 140, 140),
    # )


    # leaf = server.scene.add_frame(
    #     "/tree/branch/leaf",
    #     wxyz=(0, 0.707107, 0, 0.707107),
    #     position=(0.354, 0.7, 0.243),

    # )
    # leaf = server.scene.add_frame(
    #     "/tree/branch/origin",
    #     wxyz=(1, 0, 0, 0),
    #     position=(0, 0, 0),

    # )
    ranges = [0.2, 0.5, 0.75, 1.0, 1.5]
    dyndoms = [False]
    all_combinations = list(itertools.product(ranges, dyndoms))
    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(10):
        for combination in all_combinations:
            plan_settings.rrtc_settings.range = combination[0]
            plan_settings.rrtc_settings.dynamic_domain = combination[1]
            plan_settings.constraint_settings.insert_all_to_tree = True
            plan_settings.constraint_settings.std_dev_scaling_factor = 0.1
            plan_settings.constraint_settings.num_projection_iterations = 15
            plan_settings.constraint_settings.descend_rate = 1.0
            result = planner_func(start, goal, e, plan_settings, constraints, sampler)

            planning_times[combination].append(result.nanoseconds/1e6)
    print("Execution completed")
    print("Planning times for each combination:")
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")


    # times = []
    # for _ in range(20):
    #     t1 = time.perf_counter_ns()
    #     result = planner_func(start, goal, e, plan_settings, constraints, sampler)
    #     print((time.perf_counter_ns() - t1) / 1e6)
    #     times.append((time.perf_counter_ns() - t1) / 1e6)
    # add_trajectory(
    #     server, result.path.numpy(), robot, [], [[]]
    # )
    # print(f"All times : {times}")
    # print(f"Average time: {np.mean(times):.2f} ms")
    # print(f"Standard deviation: {np.std(times):.2f} ms")
    # while True:
    #     continue


if __name__ == '__main__':
    Fire(main)
