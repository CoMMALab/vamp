import numpy as np
from viser import transforms as tf
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path

import vamp
from fire import Fire

# Starting configuration
a = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Goal configuration
b = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]

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
    attachment_radius: float = 0.07,
    attachment_offset: float = 0.02,
    planner: str = "rrtc",
    **kwargs,
    ):

    robot_dir = Path(__file__).parents[1] / "resources" / "iiwa_marker"
    server, robot = setup_viser_with_robot(robot_dir, "iiwa_marker_spherized_collision_as_visual.urdf")
    robot.update_cfg(a)

    server.scene.add_frame(
        "/world_origin",
        position=(0.0, 0.0, 0.0),
        wxyz=(1.0, 0.0, 0.0, 0.0),
        axes_length=0.3,
        axes_radius=0.01,
    )

    trajectory = np.array([
        [-0.590226, 2.38491, 0.490872, 1.63233, 2.69949, -2.27675, 1.88928],
        [0.543891, 2.45087, 1.26009, 2.36853, 2.14838, -2.3226, 1.70952],
        [0.181092, 1.63625, 0.992812, 0.0141434, 2.14979, -1.52007, -2.97467]
    ])

    add_trajectory(server, trajectory, robot, [], [[]])

    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
