import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path

import vamp
from fire import Fire
import time

def main(
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,

    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("dual_panda", "rrtc", **kwargs)
    )

    # plan_settings.rrtc_settings.range = 1.0
    # plan_settings.rrtc_settings.dynamic_domain = False
    # plan_settings.rrtc_settings.radius = 4.0
    # plan_settings.constraint_settings.num_projection_iterations = 15
    # plan_settings.constraint_settings.std_dev_scaling_factor = 0.1

    # for attr in dir(plan_settings.rrtc_settings):
    #     print(f"{attr}: {getattr(plan_settings.rrtc_settings, attr)}")
    # for attr in dir(plan_settings.constraint_settings):
    #     print(f"{attr}: {getattr(plan_settings.constraint_settings, attr)}")


    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [0.00, 0.0, 1.00, 0.00, 0.0, 0.0, 0.221814],
        [-10.001, -10.001, -10.001, -10.001, -10.001, -10.001],
        [10.001, 10.001, 10.001, 10.001, 10.001, 10.001]
    )

    constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(bimanual_constraint)


    start = [1.153085, 1.0687546, -0.72878325, -1.7417533, -0.5460635, 1.5697869, -1.5375385, 0.0, 0.0, 1.2295971, 1.0459367, -0.718508, -1.7925525, -0.58791673, 1.6164308, -1.5082302, 0.0, 0.0]
    goal = [2.4776876, 0.80449617, -2.2346807, -2.521999, -1.046068, 1.9215181, -2.0181684, 0.0, 0.0, 2.6297247, 0.20511132, -2.1690536, -2.1782806, -1.3524126, 1.5270882, -1.8656551, 0.0, 0.0]


    start_eef_constraints = vamp_module.TaskSpaceConstraint(
        [[-0.70701665, -0.7070191 ,  0.01122052, 0.01120981, 0.26074907,  0.14300023,  0.8614942], [0.00504418, 0.00504418,  0.7070877, 0.70708996, 0.28567183, -0.35142893,  0.8594931]],
        [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-0.001, -0.001, -0.001, -0.005, -0.005, -0.005, -0.001, -0.001, -0.001, -0.005, -0.005, -0.005],
        [0.001, 0.001, 0.001, 0.005, 0.005, 0.005, 0.001, 0.001, 0.001, 0.005, 0.005, 0.005]
    )

    start_eef_constraints = vamp_module.Composable_TaskSpaceConstraint(start_eef_constraints)

    c1 = start_eef_constraints.projectConfiguration(np.array(start), 0, 10.0, 0.5, 500, True)
    print(", ".join(map(str, c1[:18])))




    goal_eef_constraints = vamp_module.TaskSpaceConstraint(
        [[-0.6998372, -0.69984126, -0.10111418, -0.10112438, 0.16705255,  0.02162239,  1.25], [-0.10722477, -0.10722317,  0.69892895, 0.6989309, 0.2809522 , -0.28401366,  1.25]],
        [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-0.001, -0.001, -0.001, -0.005, -0.005, -0.005, -0.001, -0.001, -0.001, -0.005, -0.005, -0.005],
        [0.001, 0.001, 0.001, 0.005, 0.005, 0.005, 0.001, 0.001, 0.001, 0.005, 0.005, 0.005]
    )

    goal_eef_constraints = vamp_module.Composable_TaskSpaceConstraint(goal_eef_constraints)
    c2 = goal_eef_constraints.projectConfiguration(np.array(goal), 0, 10.0, 0.5, 500, True)
    print(", ".join(map(str, c2[:18])))

    e = vamp.Environment()
    # problem_cuboids = np.loadtxt('resources/environments/cuboids/shelf_drake.txt', delimiter = ",")
    # for cuboid in problem_cuboids:
    #     e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]



    np.set_printoptions(precision = 4, suppress = True)
    print(np.array(vamp_module.eefk(start)))
    print(np.array(vamp_module.eefk(goal)))

    print(vamp_module.debug(goal, e))
    print(vamp_module.debug(start, e))

    # print(vamp_module.fkcc(start))
    # print(vamp_module.fkcc(goal))
    # result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    # stop

    robot_dir = Path(__file__).parents[1] / "resources" / "rlbench_panda"
    server, robot = setup_viser_with_robot(robot_dir, "dualpanda_exported_spherized.urdf")
    robot.update_cfg(start)

    floor_grid = server.scene.add_grid(
        name="/floor_grid",
        width=10.0,
        height=10.0,
        plane="xy",
        position=(0.0, 0.0, 0.0),
        cell_color=(200, 200, 200),
        section_color=(140, 140, 140),
    )


    leaf = server.scene.add_frame(
        "/righteef",
        wxyz=(0.00504418, 0.00504418,  0.7070877, 0.70708996),
        position=(0.28567183, -0.35142893,  0.8594931),

    )
    leaf = server.scene.add_frame(
        "/leftteef",
        wxyz=(-0.70701665, -0.7070191 ,  0.01122052, 0.01120981),
        position=(0.26074907,  0.14300023,  0.8614942),

    )

    result_path = np.linspace(start, goal, num=100)

    add_trajectory(
        server, result_path, robot, [], [[]]
    )

    times = []
    for _ in range(20):
        t1 = time.perf_counter_ns()
        result = planner_func(start, goal, e, plan_settings, sampler)
        print((time.perf_counter_ns() - t1) / 1e6)
        times.append((time.perf_counter_ns() - t1) / 1e6)
    
    result.path.interpolate_to_resolution(vamp_module.resolution())
    add_trajectory(
        server, result.path.numpy(), robot, [], [[]]
    )
    print(f"All times : {times}")
    print(f"Average time: {np.mean(times):.2f} ms")
    print(f"Standard deviation: {np.std(times):.2f} ms")
    while True:
        continue


if __name__ == '__main__':
    Fire(main)
