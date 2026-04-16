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
        vamp.configure_robot_and_planner_with_kwargs("bimanual_panda", "crrtc", **kwargs)
    )

    plan_settings.rrtc_settings.range = 1.0
    plan_settings.rrtc_settings.dynamic_domain = False
    plan_settings.rrtc_settings.radius = 4.0
    plan_settings.constraint_settings.num_projection_iterations = 15
    plan_settings.constraint_settings.std_dev_scaling_factor = 0.1

    for attr in dir(plan_settings.rrtc_settings):
        print(f"{attr}: {getattr(plan_settings.rrtc_settings, attr)}")
    for attr in dir(plan_settings.constraint_settings):
        print(f"{attr}: {getattr(plan_settings.constraint_settings, attr)}")


    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [0.00, 0.0, 1.00, 0.00, 0.0, 0.0, 0.221814],
        [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001],
        [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    )

    constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(bimanual_constraint)



    e = vamp.Environment()
    problem_cuboids = np.loadtxt('resources/environments/cuboids/shelf_drake.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    start = [-1.362, 1.319, 1.064, -2.486, 0.518, 2.481, -1.459, 1.327, 1.260, -1.048, -2.481, -0.644, 2.444, -0.011 ]
    goal = [-2.143, 0.395, 2.249, -2.043, 1.320, 1.772, -0.697, 1.359, 1.320, -2.092, -2.138, -0.140, 2.437, -1.547]
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    np.set_printoptions(precision = 4, suppress = True)
    print(np.array(vamp_module.eefk(start)))
    print(np.array(vamp_module.eefk(goal)))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "bipanda_spherized.urdf")
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
        "/tree/branch/leaf",
        wxyz=(0, 0.707107, 0, 0.707107),
        position=(0.354, 0.7, 0.243),

    )
    leaf = server.scene.add_frame(
        "/tree/branch/origin",
        wxyz=(1, 0, 0, 0),
        position=(0, 0, 0),

    )

    times = []
    for _ in range(20):
        t1 = time.perf_counter_ns()
        result = planner_func(start, goal, e, plan_settings, constraints, sampler)
        print((time.perf_counter_ns() - t1) / 1e6)
        times.append((time.perf_counter_ns() - t1) / 1e6)
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
