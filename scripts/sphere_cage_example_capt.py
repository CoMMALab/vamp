import numpy as np
from viser import transforms as tf
import os
from viser_utils import (
    setup_viser_with_robot,
    add_point_cloud,
    add_spheres,
    add_trajectory,
    )
from pathlib import Path

import vamp
import vamp.pointcloud

from fire import Fire

# Starting configuration
a = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

# Goal configuration
b = np.array([2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785], dtype=np.float32)

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]


def main(
    obstacle_radius: float = 0.2,
    points_per_sphere: int = 1000,
    attachment_radius: float = 0.07,
    attachment_offset: float = 0.02,
    planner: str = "rrtc",
    structure: str = "capt",
    filter_voxel_size: float = 0.0,
    **kwargs,
    ):

    point_cloud = np.vstack(
        [
            vamp.pointcloud.sphere_sample_surface(center, obstacle_radius, points_per_sphere, 0.0)
            for center in problem
            ]
        ).astype(np.float32)

    workspace_min = point_cloud.min(axis=0) - 0.1
    workspace_max = point_cloud.max(axis=0) + 0.1

    if filter_voxel_size > 0.0:
        filtered, filter_ns = vamp.filter_pointcloud_centervox(
            point_cloud,
            filter_voxel_size,
            10.0,
            [0.0, 0.0, 0.0],
            workspace_min.tolist(),
            workspace_max.tolist(),
            )
        print(f"centervox filter: {point_cloud.shape[0]} -> {len(filtered)} points "
              f"({filter_ns / 1e6:.2f} ms)")
        point_cloud = np.array(filtered, dtype=np.float32)

    point_cloud_colors = np.random.randint(100, 200, (point_cloud.shape[0], 3))

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot.update_cfg(a)

    e = vamp.Environment()

    r_min, r_max = vamp_module.min_max_radii()
    if structure == "capt":
        build_ns = e.add_pointcloud(point_cloud.tolist(), r_min, r_max, 0.01)
    elif structure == "mvt":
        build_ns = e.add_pointcloud_mvt(
            point_cloud.tolist(),
            r_max,
            workspace_min.tolist(),
            workspace_max.tolist(),
            0.01,
            )
    else:
        raise ValueError(f"unknown structure {structure!r}: expected 'capt' or 'mvt'")

    print(f"{structure} build: {point_cloud.shape[0]} points ({build_ns / 1e6:.2f} ms)")

    _problem_point_cloud_handles = add_point_cloud(server, point_cloud, point_cloud_colors)

    # Plan and display
    sampler = vamp_module.halton()
    result = planner_func(a, b, e, plan_settings, sampler)
    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    add_trajectory(server, simple.path.numpy(), robot, [], [[]])

    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
