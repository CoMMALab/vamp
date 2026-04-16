import numpy as np
import os
from pathlib import Path
import vamp
import meshcat_viz_utils as viz
from fire import Fire
import json

# Starting configuration
a = [-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294]

# Goal configuration
b = [1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238]

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


def add_json_cuboids(e, filename):
    """
    Add cuboids from a JSON file. The JSON should be a list of objects, each with keys:
    'name', 'x', 'y', 'z', 'dx', 'dy', 'dz' and optional 'roll','pitch','yaw'.

    This function reads the JSON, iterates over entries, and adds a box for each item.
    Dimensions are used as provided in the JSON (assumed full extents).
    """
    with open(filename, "r") as f:
        data = json.load(f)

    cuboids = []
    colors = []
    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x = obj.get("x", 0.0)
        y = obj.get("y", 0.0)
        z = obj.get("z", 0.0)
        dx = obj.get("dx", 0.0)
        dy = obj.get("dy", 0.0)
        dz = obj.get("dz", 0.0)

        e.add_cuboid(vamp.Cuboid([x, y, z], [0.0, 0.0, 0.0], [dx / 2, dy / 2, dz / 2]))
        cuboids.append([x, y, z, 0.0, 0.0, 0.0, dx, dy, dz])  # Store for visualization

        # if it is a wall, dark brown else light brown
        if "wall" in name.lower():
            colors.append((101, 67, 33))  # Dark Brown
        else:
            colors.append((210, 180, 140))  # Light Brown
    
    return (cuboids, colors)


def get_eef_of_waypoints(waypoints, robot, planner):
    (vamp_module, _, _, _) = vamp.configure_robot_and_planner_with_kwargs(robot, planner)
    eef_poses = []
    for idx, waypoint in enumerate(waypoints):
        eef_pose = [pose.tolist() for pose in vamp_module.eefk(waypoint)]
        eef_poses.extend(eef_pose)
    return np.array(eef_poses)


def main(
    obstacle_radius: float = 0.2,
    attachment_radius: float = 0.01,
    planner: str = "crrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    # Create an attachment offset on the Z-axis from the end-effector frame
    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, 0])
    attachment = vamp.Attachment(tf)

    attach_positions = np.zeros((10, 3))
    attach_positions[:, 2] = np.linspace(0, 0.16, len(attach_positions))

    # Add a single sphere to the attachment - spheres are added in the attachment's local frame
    attachment.add_spheres(
        [
            vamp.Sphere(pos, attachment_radius) for pos in attach_positions
        ]
    )

    tsr = vamp_module.TaskSpaceConstraint(
        [[1, 0, 0, 0, 0, 0, 0]],
        [[0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783]],
        [-10.01, -10.01, -0.001, -0.01, -0.01, -10.01],
        [10.01, 10.01, 0.001, 0.01, 0.01, 10.01]
    )
    constraints = vamp_module.Composable_TaskSpaceConstraint(tsr)

    e = vamp.Environment()

    env_cuboids, env_colors = add_json_cuboids(e, "resources/environments/cuboids/real_maze.json")

    # Add the attchment to the VAMP environment
    e.attach(attachment, 0)

    if planner == "crrtc":
        plan_settings.rrtc_settings.range = 0.5
    else:
        plan_settings.range = 0.5

    sampler = vamp_module.halton()
    result = planner_func(a, b, e, plan_settings, constraints, sampler)
    simple = vamp_module.simplify_with_constraints(result.path, e, constraints, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    print("Planned path with", len(result.path), "waypoints, simplified to", len(simple.path), "waypoints in ", result.nanoseconds / 1e6, "ms")

    # display in meshcat
    viz_instance = viz.MeshcatViz()
    viz_instance.init_viz("resources/panda/panda_spherized.urdf", "resources/panda/meshes/")
    viz_instance.clear_all_waypoints()

    viz_instance.add_cuboids(env_cuboids, colors=env_colors)
    # optional: render the waypoints as spheres in meshcat
    eef_poses = get_eef_of_waypoints(simple.path.numpy(), "panda", planner)
    viz_instance.render_eefs(eef_poses)

    viz_instance.animate(simple.path.numpy(), np.arange(0, len(simple.path.numpy()), dtype=np.float64) / 50, loop=True)
    


if __name__ == "__main__":
    Fire(main)