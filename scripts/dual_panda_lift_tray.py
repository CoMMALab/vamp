import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import vamp
from fire import Fire
import time

np.set_printoptions(precision=3, suppress=True, linewidth=200)


def project_config_to_pose(goal_poses, vamp_module, start_config):
    """
    Goal pose is in the form [[qw, qx, qy, qz, x, y, z] * num_eef].
    Returns a config that satisfies the pose
    """
    arg0 = [[float(x) for x in row] for row in goal_poses]
    constraint = vamp_module.TaskSpaceConstraint(
        goal_poses,
        [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * len(goal_poses),
        [-0.0001] * 6 * len(goal_poses),
        [0.0001] * 6 * len(goal_poses)
    )
    composable_constraint = vamp_module.Composable_TaskSpaceConstraint(constraint)
    projected_config = composable_constraint.projectConfiguration(np.array(start_config), 0, 10.0, 0.5, 50, True)
    return projected_config


def se3_matrix_to_se3_vec(se3_matrix):
    """
    Convert a 4x4 SE3 matrix to a 7D vector of the form [qw, qx, qy, qz, x, y, z]
    """
    rotation_matrix = se3_matrix[:3, :3]
    translation = se3_matrix[:3, 3]
    quat = R.from_matrix(rotation_matrix).as_quat(scalar_first = True)  # returns in (w, x, y, z) format
    return np.concatenate([quat, translation])

def main(
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,

    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("dual_panda", "crrtc", **kwargs)
    )
    start = [1.153085, 1.0687546, -0.72878325, -1.7417533, -0.5460635, 1.5697869, -1.5375385, 0.03999999910593033, 0.03999999910593033, 1.2295971, 1.0459367, -0.718508, -1.7925525, -0.58791673, 1.6164308, -1.5082302, 0.03999999910593033, 0.03999999910593033]
    goal = [2.4776876, 0.80449617, -2.2346807, -2.521999, -1.046068, 1.9215181, -2.0181684, 0.03999999910593033, 0.03999999910593033, 2.6297247, 0.20511132, -2.1690536, -2.1782806, -1.3524126, 1.5270882, -1.8656551, 0.03999999910593033, 0.03999999910593033]


    plan_settings.rrtc_settings.balance = True
    plan_settings.rrtc_settings.tree_ratio = -0.001
    plan_settings.rrtc_settings.start_tree_first = False




    initial_config = [0.01000453345477581, 0.1852446049451828, 0.010011476464569569, -0.8631004691123962, 0.009994401596486568, 1.2315564155578613, 0.7953943014144897, 0.03999999910593033, 0.03999999910593033, 0.009997142478823662, 0.18523173034191132, 0.010000032372772694, -0.8630699515342712, 0.009978427551686764, 1.2315738201141357, 0.7953969240188599, 0.03999999910593033, 0.03999999910593033]


    waypoint_0 = [
        [-0.7037,-0.7037,-0.0689,-0.0689,0.0496,0.1174,0.8615], [-0.0750,-0.0750,0.7031,0.7031,0.1850,-0.3588,0.8595]
    ]
    waypoint_1 = [
         [-0.7037,-0.7037,-0.0689,-0.0689,0.0740,0.0422,0.8615], [-0.0750,-0.0750,0.7031,0.7031,0.1594,-0.2725,0.8595],
    ]

    waypoint_2 = [
        [-0.7037,-0.7037,-0.0689,-0.0689,0.0740,0.0423,1.2500], [-0.0750,-0.0750,0.7031,0.7031,0.1594,-0.2725,1.2500]
    ]

    
    e = vamp.Environment()
    # problem_cuboids = np.loadtxt('resources/environments/cuboids/shelf_drake.txt', delimiter = ",")
    # for cuboid in problem_cuboids:
    #     e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()


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

    no_bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-10.0] * 6,
        [10.0] * 6
    )
    no_constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(no_bimanual_constraint)


    planned_traj = []

    t1 = time.time()

    # find config for waypoint 0
    waypoint_0_config = project_config_to_pose(waypoint_0, vamp_module, start)
    t2 = time.time()
    print(f"Time taken to project waypoint 0 config: {(t2 - t1) * 1000:.3f} milliseconds")
    result = planner_func(initial_config, waypoint_0_config, e, plan_settings, no_constraints, sampler)
    t3 = time.time()
    print(f"Time taken to plan to waypoint 0: {(t3 - t2) * 1000:.3f} milliseconds")
    result.path.interpolate_to_resolution(vamp_module.resolution())
    planned_traj.extend(result.path.numpy())

    fk1 = vamp_module.eefk(waypoint_0_config)
    # compute relative transform between fk1[0] and fk[1]
    relative_transform = np.linalg.inv(fk1[0]) @ fk1[1]


    # now compute for waypoint 1 from waypoint_0
    t3 = time.time()
    waypoint_1_config = project_config_to_pose(waypoint_1, vamp_module, waypoint_0_config)
    t4 = time.time()
    print(f"Time taken to project waypoint 1 config: {(t4 - t3) * 1000:.3f} milliseconds")
    result = planner_func(waypoint_0_config, waypoint_1_config, e, plan_settings, no_constraints, sampler)
    t5 = time.time()
    print(f"Time taken to plan to waypoint 1: {(t5 - t4) * 1000:.3f} milliseconds")
    result.path.interpolate_to_resolution(vamp_module.resolution())
    planned_traj.extend(result.path.numpy())
    fk2 = vamp_module.eefk(waypoint_1_config)
    # compute relative transform between fk2[0] and fk2[1]
    relative_transform_1_2 = np.linalg.inv(fk2[0]) @ fk2[1]


    # relative_transform_se3 = [0.0] * 7
    # # first 4 values are wxyz, compute from relative_transform
    # relative_transform_se3[:4] = R.from_matrix(relative_transform[:3, :3]).as_quat().tolist()
    # # last 3 values are translation, compute from relative_transform
    # relative_transform_se3[4:] = relative_transform[:3, 3].tolist()

    relative_transform_se3 = se3_matrix_to_se3_vec(relative_transform_1_2)
    print("Relative transform between waypoint 1 and 2: ", relative_transform_se3)


    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        relative_transform_se3,
        [-0.001] * 6,
        [0.001] * 6
    )
    constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(bimanual_constraint)

    t5 = time.time()
    # now compute for waypoint 2 from waypoint_1
    waypoint_2_config = project_config_to_pose(waypoint_2, vamp_module, goal)
    t6 = time.time()
    print(f"Time taken to project waypoint 2 config: {(t6 - t5) * 1000:.3f} milliseconds")

    fk3 = vamp_module.eefk(waypoint_2_config)
    relative_transform_2_3 = np.linalg.inv(fk3[0]) @ fk3[1]

    # compute relative transform between relative_transform_1_2 and relative_transform, relative_transform_1_2 @ np.linalg.inv(relative_transform)
    print("Relative transform between waypoint 2 and 3: ", se3_matrix_to_se3_vec(relative_transform_2_3))
    


    result = planner_func(waypoint_1_config, waypoint_2_config, e, plan_settings, constraints, sampler)
    t7 = time.time()
    print(f"Time taken to plan to waypoint 2 with bimanual constraint: {(t7 - t6) * 1000:.3f} milliseconds")
    result.path.interpolate_to_resolution(vamp_module.resolution())
    planned_traj.extend(result.path.numpy())


    print(f"Time taken for planning: {(time.time() - t1) * 1000:.3f} milliseconds")


    result_path = np.linspace(start, goal, num=100)

    add_trajectory(
        server, planned_traj, robot, [], [[]]
    )

    add_trajectory(
        server, result_path, robot, [], [[]]
    )

    # times = []
    # for _ in range(20):
    #     t1 = time.perf_counter_ns()
    #     result = planner_func(start, goal, e, plan_settings, sampler)
    #     print((time.perf_counter_ns() - t1) / 1e6)
    #     times.append((time.perf_counter_ns() - t1) / 1e6)
    
    # result.path.interpolate_to_resolution(vamp_module.resolution())
    # add_trajectory(
    #     server, result.path.numpy(), robot, [], [[]]
    # )
    # print(f"All times : {times}")
    # print(f"Average time: {np.mean(times):.2f} ms")
    # print(f"Standard deviation: {np.std(times):.2f} ms")
    while True:
        continue


if __name__ == '__main__':
    Fire(main)
