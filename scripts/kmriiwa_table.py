"""
Mobile manipulation with the KMR iiwa: drive around a table to reach an object.

The KMR iiwa is an iiwa arm on a mobile base. It is planned for as one 10-DoF robot: base x, y, yaw, then
the seven arm joints. Its collision model mixes primitives: the arm is covered by spheres and the base is
a single cuboid. That cuboid is what keeps the base honest around the table; the table is a big solid
obstacle and the base is a large box (1.14 x 0.64 m), so a sphere cover would be both looser and much
more expensive.

The task: the robot starts on the north side of the table with its arm upright. The goal is on the south
side, facing the table, with the tool held above the object on the tabletop. The straight line between
them goes through the table, so the planner has to drive the base around it.

Run it with

    python scripts/kmriiwa_table.py

and open the printed viser URL (usually http://localhost:8080). Useful options:

    --visualize=False          just plan and print results (no viser)
    --planner=aorrtc           any planner VAMP provides for this robot (rrtc, prm, fcit, aorrtc, ...)
    --urdf=/path/kmriiwa.urdf  also draw the robot's visual meshes from this URDF (optional; by default
                               the robot is drawn from its collision model, which is what VAMP checks)
    --safety_margin=0.02       how far to pad obstacles while planning, in meters

Base yaw limits: the planner treats the base yaw as an ordinary coordinate in [lower, upper]. If the robot
model gives it a huge range (e.g., +-1000 rad), paths can spin the base many times. Give the yaw joint a
limit of +-pi in the URDF and regenerate the robot with cricket; this script warns if it is wider.
"""

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from fire import Fire

import vamp

ROBOT = "kmriiwa_tool"

# Joint order: base x, base y, base yaw, then the seven iiwa joints
ARM_UPRIGHT = [0.0] * 7
# Found by a local search for a top-down grasp pose (tool z pointing down) above the object
ARM_REACH = [1.48, 0.72, 0.53, -1.15, -0.25, 1.25, -0.6]

START = [0.0, 1.15, -math.pi / 2] + ARM_UPRIGHT  # north of the table, facing south
GOAL = [0.0, -1.05, math.pi / 2] + ARM_REACH  # south of the table, facing north, reaching over it

# name, center, half-extents, color
Box = Tuple[str, Tuple[float, float, float], Tuple[float, float, float], Tuple[int, int, int]]
SCENE: List[Box] = [
    # A solid table (it is closed at the bottom, so the base cannot drive under it)
    ("table", (0.0, 0.0, 0.375), (0.5, 0.3, 0.375), (160, 120, 80)),
    # The object to reach for, on the table
    ("object", (0.0, -0.05, 0.80), (0.05, 0.05, 0.05), (220, 60, 60)),
    ]


def build_environment(margin: float = 0.0) -> "vamp.Environment":
    """The scene as a VAMP environment, with every box grown by `margin` meters on all sides."""
    env = vamp.Environment()
    for _, center, half, _ in SCENE:
        env.add_cuboid(vamp.Cuboid(list(center), [0.0, 0.0, 0.0], [h + margin for h in half]))

    return env


def plan(planner: str, safety_margin: float, seed_skip: int = 0, n_states: int = 200, **kwargs) -> Dict:
    """Plans from START to GOAL, simplifies the path, and checks it against the true (unpadded) scene."""
    robot = getattr(vamp, ROBOT)

    lower, upper = np.array(robot.lower_bounds()), np.array(robot.upper_bounds())
    if upper[2] - lower[2] > 2 * math.pi + 1e-3:
        print(
            f"WARNING: base yaw range is [{lower[2]:.0f}, {upper[2]:.0f}] rad. Planned paths may spin the "
            "base around many times; limit that joint to +-pi in the URDF and regenerate the robot."
            )

    # Plan with obstacles padded by a small margin. Collision checking is discrete, so without padding a
    # path can graze an obstacle between two checks. The path is then verified against the true scene.
    planning_env = build_environment(safety_margin)
    true_env = build_environment(0.0)

    for name, config in (("start", START), ("goal", GOAL)):
        if not robot.validate(config, planning_env):
            raise RuntimeError(f"The {name} configuration is in collision (with a {safety_margin} m margin).")

    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs(ROBOT, planner, **kwargs)
        )

    sampler = module.halton()
    for _ in range(seed_skip):
        sampler.next()

    result = planner_func(START, GOAL, planning_env, plan_settings, sampler)
    if not result.solved:
        raise RuntimeError("The planner did not find a path. Try more iterations or another planner.")

    simplified = module.simplify(result.path, planning_env, simp_settings, sampler)
    path = simplified.path
    path.interpolate_to_n_states(n_states)
    waypoints = path.numpy()

    # Check the final path against the real obstacles, not the padded ones
    waypoints_valid = all(robot.validate(c, true_env) for c in waypoints)
    motions_valid = all(robot.validate_motion(a, b, true_env) for a, b in zip(waypoints[:-1], waypoints[1:]))

    return {
        "waypoints": waypoints,
        "planning_ms": result.nanoseconds / 1e6,
        "iterations": result.iterations,
        "initial_cost": result.path.cost(),
        "simplified_cost": simplified.path.cost(),
        "valid": bool(waypoints_valid and motions_valid),
        "yaw_range": (float(waypoints[:, 2].min()), float(waypoints[:, 2].max())),
        }


def rotation_to_wxyz(rotation: np.ndarray) -> np.ndarray:
    from viser import transforms as tf
    return tf.SO3.from_matrix(rotation).wxyz


def show_in_viser(waypoints: np.ndarray, urdf: Optional[str], block: bool = True):
    """Shows the scene, the robot's collision model, and the path in viser."""
    import viser

    robot = getattr(vamp, ROBOT)

    server = viser.ViserServer()
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/floor", width = 6.0, height = 6.0, plane = "xy", cell_size = 0.5)

    @server.on_client_connect
    def _(client: viser.ClientHandle):
        client.camera.position = (3.2, -3.4, 2.6)
        client.camera.look_at = (0.0, 0.0, 0.5)

    # The scene
    for name, center, half, color in SCENE:
        server.scene.add_box(
            f"/scene/{name}",
            color = color,
            dimensions = tuple(2 * h for h in half),
            position = center,
            )

    # Start and goal markers on the floor
    for label, config, color in (("start", START, (60, 160, 60)), ("goal", GOAL, (60, 60, 220))):
        server.scene.add_label(f"/markers/{label}", label, position = (config[0], config[1], 0.02))
        server.scene.add_icosphere(
            f"/markers/{label}_dot",
            radius = 0.06,
            color = color,
            position = (config[0], config[1], 0.06),
            )

    # Where the tool and the base travel
    tool = np.array([robot.eefk(c)[:3, 3] for c in waypoints])
    base = np.stack([waypoints[:, 0], waypoints[:, 1], np.full(len(waypoints), 0.03)], axis = 1)
    path_handles = [
        server.scene.add_spline_catmull_rom(
            "/paths/tool", positions = tool, color = (255, 160, 0), line_width = 3.0
            ),
        server.scene.add_spline_catmull_rom(
            "/paths/base", positions = base, color = (60, 160, 60), line_width = 3.0
            ),
        ]

    # The robot's collision model, exactly the geometry VAMP checks: spheres for the arm, a cuboid for the base
    first_spheres = robot.fk(waypoints[0])
    sphere_handles = [
        server.scene.add_icosphere(
            f"/collision/sphere_{i}",
            radius = float(s.r),
            color = (70, 130, 220),
            position = (s.x, s.y, s.z),
            opacity = 0.55,
            ) for i, s in enumerate(first_spheres)
        ]
    cuboid_handles = [
        server.scene.add_box(
            f"/collision/cuboid_{i}",
            color = (240, 150, 50),
            dimensions = (2 * c.axis_1_r, 2 * c.axis_2_r, 2 * c.axis_3_r),
            opacity = 0.6,
            ) for i, c in enumerate(robot.cuboid_fk(waypoints[0]))
        ]

    # Optionally, the robot's visual meshes
    urdf_viewer = None
    joint_order = None
    if urdf:
        import yourdfpy
        from viser.extras import ViserUrdf

        model = yourdfpy.URDF.load(urdf)
        names = list(robot.joint_names())
        missing = [n for n in model.actuated_joint_names if n not in names]
        if missing:
            raise RuntimeError(f"URDF joints {missing} are not joints of the {ROBOT} robot: {names}")

        joint_order = [names.index(n) for n in model.actuated_joint_names]
        urdf_viewer = ViserUrdf(
            server, model, load_meshes = True, load_collision_meshes = False, root_node_name = "/robot"
            )

    def show(index: int):
        config = waypoints[index]

        for handle, sphere in zip(sphere_handles, robot.fk(config)):
            handle.position = (sphere.x, sphere.y, sphere.z)

        for handle, cuboid in zip(cuboid_handles, robot.cuboid_fk(config)):
            # The cuboid's axes are the columns of its rotation matrix
            rotation = np.array(
                [
                    [cuboid.axis_1_x, cuboid.axis_2_x, cuboid.axis_3_x],
                    [cuboid.axis_1_y, cuboid.axis_2_y, cuboid.axis_3_y],
                    [cuboid.axis_1_z, cuboid.axis_2_z, cuboid.axis_3_z],
                    ],
                )
            handle.position = (cuboid.x, cuboid.y, cuboid.z)
            handle.wxyz = rotation_to_wxyz(rotation)

        if urdf_viewer is not None:
            urdf_viewer.update_cfg(config[joint_order])

        x, y, yaw = config[:3]
        info.content = f"Base: x={x:+.2f} m, y={y:+.2f} m, yaw={math.degrees(yaw):+.0f} deg"

    with server.gui.add_folder("Path"):
        slider = server.gui.add_slider(
            "Waypoint", min = 0, max = len(waypoints) - 1, step = 1, initial_value = 0
            )
        play = server.gui.add_button("Play / pause")
        speed = server.gui.add_slider("Speed (waypoints/s)", min = 5, max = 120, step = 5, initial_value = 40)
        info = server.gui.add_markdown("")

    with server.gui.add_folder("Display"):
        show_collision = server.gui.add_checkbox("Collision model", initial_value = urdf is None)
        show_paths = server.gui.add_checkbox("Tool and base paths", initial_value = True)

    state = {"playing": True}

    @slider.on_update
    def _(_) -> None:
        show(int(slider.value))

    @play.on_click
    def _(_) -> None:
        state["playing"] = not state["playing"]

    @show_collision.on_update
    def _(_) -> None:
        for handle in sphere_handles + cuboid_handles:
            handle.visible = show_collision.value

    @show_paths.on_update
    def _(_) -> None:
        for handle in path_handles:
            handle.visible = show_paths.value

    show(0)
    show_collision.value = urdf is None

    print(f"\nOpen the viser URL above to watch the {len(waypoints)}-waypoint path (Ctrl-C to quit).")
    if not block:
        return server

    position = 0.0
    last = time.time()
    while True:
        now = time.time()
        if state["playing"]:
            position += (now - last) * speed.value
            if position >= len(waypoints):
                position = 0.0

            index = int(position)
            if index != int(slider.value):
                slider.value = index

        last = now
        time.sleep(0.01)


def main(
    planner: str = "rrtc",
    visualize: bool = True,
    urdf: Optional[str] = None,
    safety_margin: float = 0.01,
    n_states: int = 200,
    seed_skip: int = 0,
    **kwargs,
    ):
    result = plan(planner, safety_margin, seed_skip, n_states, **kwargs)

    print(f"Planner:             {planner}")
    print(f"Planning time:       {result['planning_ms']:.2f} ms ({result['iterations']} iterations)")
    print(f"Path cost:           {result['initial_cost']:.2f} -> {result['simplified_cost']:.2f} (simplified)")
    print(f"Waypoints:           {len(result['waypoints'])}")
    print(f"Base yaw range:      [{result['yaw_range'][0]:+.2f}, {result['yaw_range'][1]:+.2f}] rad")
    print(f"Collision-free:      {result['valid']} (checked against the unpadded scene)")

    if visualize:
        show_in_viser(result["waypoints"], urdf)


if __name__ == "__main__":
    Fire(main)
