"""
Mobile manipulation with the KMR iiwa: pick up a block and carry it around a table.

The KMR iiwa is an iiwa arm on a mobile base. It is planned for as one 10-DoF robot: base x, y, yaw, then
the seven arm joints. Its collision model mixes primitives: the arm is covered by spheres and the base is
a single cuboid. That cuboid is what keeps the base honest around the table; the table is a big solid
obstacle and the base is a large box (1.14 x 0.64 m), so a sphere cover would be both looser and much
more expensive.

The task has three motions:

  1. Approach. The robot starts on the north side of the table with its arm upright and drives around
     to the south side, where it reaches over the table and positions the tool above a block.
  2. Lift. The robot picks the block up and raises it a few centimeters, in a straight line in joint
     space. It is checked against the real scene instead of planned, since it is short.
  3. Carry. The robot carries the block around to the east side of the table, where it holds it over the
     tabletop. The block is attached to the tool as a cuboid, so while carrying it the planner checks
     the block itself, not just the robot, against the table (and the block against the robot).

The approach has to go around the table: the straight line between its poses goes through it. The carry
then moves to the next side of the table while holding the block.

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

# The poses below were found with a local search for top-down grasp poses (tool z pointing down)
START = [0.0, 1.15, -math.pi / 2] + ARM_UPRIGHT  # north of the table, facing south
# South of the table, facing north, tool just above the block
GRASP = [0.0, -1.05, math.pi / 2] + [2.06, 0.81, -0.32, -1.22, 0.27, 1.08, -0.86]
# The same, with the block lifted 4.5 cm off the table
LIFT = [0.0, -1.05, math.pi / 2] + [2.055, 0.748, -0.359, -1.215, 0.335, 1.136, -0.851]
# East of the table, facing (nearly) west, holding the block above the far end of the tabletop
PLACE = [1.35, 0.0, 3.04] + [1.13, 0.81, 1.1, -1.48, -0.59, 1.23, -1.19]

# name, center, half-extents, color
Box = Tuple[str, Tuple[float, float, float], Tuple[float, float, float], Tuple[int, int, int]]
# A solid table (it is closed at the bottom, so the base cannot drive under it)
TABLE: Box = ("table", (0.0, 0.0, 0.375), (0.5, 0.3, 0.375), (160, 120, 80))
# The block to pick up. It rests 5 mm above the table top (0.75 m) so that picking it up does not start in
# contact with the table, which collision checking treats as a collision.
BLOCK: Box = ("block", (0.0, -0.05, 0.795), (0.04, 0.04, 0.04), (220, 60, 60))


def cuboid(box: Box, margin: float = 0.0) -> "vamp.Cuboid":
    _, center, half, _ = box
    return vamp.Cuboid(list(center), [0.0, 0.0, 0.0], [h + margin for h in half])


def build_environment(margin: float = 0.0, attachment: Optional["vamp.Attachment"] = None):
    """
    The scene as a VAMP environment, with every box grown by `margin` meters on all sides.

    The block is not in the scene: it is the target of the approach, and once picked up it is carried by
    the robot (the attachment), so it is never an obstacle.
    """
    env = vamp.Environment()
    env.add_cuboid(cuboid(TABLE, margin))
    if attachment is not None:
        env.attach(attachment)

    return env


def block_in_tool_frame() -> np.ndarray:
    """The block's pose relative to the tool at the grasp, as a 4x4 transform."""
    robot = getattr(vamp, ROBOT)
    block_in_world = np.identity(4)
    block_in_world[:3, 3] = BLOCK[1]
    return np.linalg.inv(robot.eefk(GRASP)) @ block_in_world


def make_attachment(block_in_tool: np.ndarray, margin: float = 0.0) -> "vamp.Attachment":
    """The block as a cuboid attached to the tool, which moves with it."""
    attachment = vamp.Attachment(block_in_tool)
    # The attachment's frame is the block's center, so the block is centered on the origin
    attachment.add_cuboid(vamp.Cuboid([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [h + margin for h in BLOCK[2]]))
    return attachment


def plan_motion(
    label: str,
    start: List[float],
    goal: List[float],
    planning_env,
    true_env,
    planner: str,
    seed_skip: int,
    n_states: int,
    **kwargs,
    ) -> Dict:
    """Plans one motion, simplifies it, and checks the result against the true (unpadded) scene."""
    robot = getattr(vamp, ROBOT)

    for name, config in (("start", start), ("goal", goal)):
        if not robot.validate(config, planning_env):
            raise RuntimeError(f"{label}: the {name} configuration is in collision.")

    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs(ROBOT, planner, **kwargs)
        )

    sampler = module.halton()
    for _ in range(seed_skip):
        sampler.next()

    result = planner_func(start, goal, planning_env, plan_settings, sampler)
    if not result.solved:
        raise RuntimeError(f"{label}: the planner did not find a path. Try more iterations or another planner.")

    simplified = module.simplify(result.path, planning_env, simp_settings, sampler)
    path = simplified.path
    path.interpolate_to_n_states(n_states)
    waypoints = path.numpy()

    # Check the final path against the real obstacles, not the padded ones
    waypoints_valid = all(robot.validate(c, true_env) for c in waypoints)
    motions_valid = all(robot.validate_motion(a, b, true_env) for a, b in zip(waypoints[:-1], waypoints[1:]))

    return {
        "label": label,
        "waypoints": waypoints,
        "planning_ms": result.nanoseconds / 1e6,
        "iterations": result.iterations,
        "initial_cost": result.path.cost(),
        "simplified_cost": simplified.path.cost(),
        "valid": bool(waypoints_valid and motions_valid),
        "yaw_range": (float(waypoints[:, 2].min()), float(waypoints[:, 2].max())),
        }


def plan_lift(block_in_tool: np.ndarray, n_states: int) -> Dict:
    """
    The lift: a straight line in joint space from the grasp pose to the lifted pose, with the block in hand.
    It is short and starts with the block resting almost on the table, so it is checked against the true
    scene instead of planned with a safety margin.
    """
    robot = getattr(vamp, ROBOT)
    true_env = build_environment(0.0, make_attachment(block_in_tool, 0.0))

    start, goal = np.array(GRASP), np.array(LIFT)
    waypoints = np.array([start + t * (goal - start) for t in np.linspace(0.0, 1.0, max(2, n_states // 8))])
    valid = all(robot.validate(c, true_env) for c in waypoints) and robot.validate_motion(
        list(start), list(goal), true_env
        )

    return {
        "label": "lift",
        "waypoints": waypoints,
        "valid": bool(valid),
        "yaw_range": (float(waypoints[:, 2].min()), float(waypoints[:, 2].max())),
        }


def plan(planner: str, safety_margin: float, seed_skip: int = 0, n_states: int = 120, **kwargs) -> Dict:
    """Plans the approach, the lift and the carry. Returns the motions and the block's pose in the tool frame."""
    robot = getattr(vamp, ROBOT)

    lower, upper = np.array(robot.lower_bounds()), np.array(robot.upper_bounds())
    if upper[2] - lower[2] > 2 * math.pi + 1e-3:
        print(
            f"WARNING: base yaw range is [{lower[2]:.0f}, {upper[2]:.0f}] rad. Planned paths may spin the "
            "base around many times; limit that joint to +-pi in the URDF and regenerate the robot."
            )

    # Plan with obstacles padded by a small margin. Collision checking is discrete, so without padding a
    # path can graze an obstacle between two checks. Paths are then verified against the true scene.
    # The carried block is padded the same way.
    block_in_tool = block_in_tool_frame()

    approach = plan_motion(
        "approach",
        START,
        GRASP,
        build_environment(safety_margin),
        build_environment(0.0),
        planner,
        seed_skip,
        n_states,
        **kwargs,
        )

    lift = plan_lift(block_in_tool, n_states)

    carry = plan_motion(
        "carry",
        LIFT,
        PLACE,
        build_environment(safety_margin, make_attachment(block_in_tool, safety_margin)),
        build_environment(0.0, make_attachment(block_in_tool, 0.0)),
        planner,
        seed_skip,
        n_states,
        **kwargs,
        )

    return {"approach": approach, "lift": lift, "carry": carry, "block_in_tool": block_in_tool}


def rotation_to_wxyz(rotation: np.ndarray) -> np.ndarray:
    from viser import transforms as tf
    return tf.SO3.from_matrix(rotation).wxyz


def show_in_viser(result: Dict, urdf: Optional[str], block: bool = True):
    """Shows the scene, the robot's collision model, the carried block, and the paths in viser."""
    import viser

    robot = getattr(vamp, ROBOT)

    waypoints = np.concatenate(
        [result["approach"]["waypoints"], result["lift"]["waypoints"], result["carry"]["waypoints"]]
        )
    grasp_index = len(result["approach"]["waypoints"]) - 1
    block_in_tool = result["block_in_tool"]

    block_at_rest = np.identity(4)
    block_at_rest[:3, 3] = BLOCK[1]

    server = viser.ViserServer()
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/floor", width = 6.0, height = 6.0, plane = "xy", cell_size = 0.5)

    @server.on_client_connect
    def _(client: viser.ClientHandle):
        client.camera.position = (3.2, -3.4, 2.6)
        client.camera.look_at = (0.0, 0.0, 0.5)

    # The table
    server.scene.add_box(
        "/scene/table",
        color = TABLE[3],
        dimensions = tuple(2 * h for h in TABLE[2]),
        position = TABLE[1],
        )

    # The block: it sits on the table until it is picked up, then moves with the tool
    block_handle = server.scene.add_box(
        "/scene/block",
        color = BLOCK[3],
        dimensions = tuple(2 * h for h in BLOCK[2]),
        position = BLOCK[1],
        )

    # The start, grasp and place poses, marked on the floor
    for label, config, color in (
        ("start", START, (60, 160, 60)),
        ("grasp", GRASP, (220, 160, 40)),
        ("place", PLACE, (60, 60, 220)),
        ):
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

        for handle, cuboid_pose in zip(cuboid_handles, robot.cuboid_fk(config)):
            # The cuboid's axes are the columns of its rotation matrix
            rotation = np.array(
                [
                    [cuboid_pose.axis_1_x, cuboid_pose.axis_2_x, cuboid_pose.axis_3_x],
                    [cuboid_pose.axis_1_y, cuboid_pose.axis_2_y, cuboid_pose.axis_3_y],
                    [cuboid_pose.axis_1_z, cuboid_pose.axis_2_z, cuboid_pose.axis_3_z],
                    ],
                )
            handle.position = (cuboid_pose.x, cuboid_pose.y, cuboid_pose.z)
            handle.wxyz = rotation_to_wxyz(rotation)

        # The block rides on the tool once it is picked up
        carrying = index >= grasp_index
        pose = robot.eefk(config) @ block_in_tool if carrying else block_at_rest
        block_handle.position = tuple(pose[:3, 3])
        block_handle.wxyz = rotation_to_wxyz(pose[:3, :3])

        if urdf_viewer is not None:
            urdf_viewer.update_cfg(config[joint_order])

        x, y, yaw = config[:3]
        phase = "carrying the block" if carrying else "approaching the block"
        info.content = f"{phase}  \nBase: x={x:+.2f} m, y={y:+.2f} m, yaw={math.degrees(yaw):+.0f} deg"

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
    n_states: int = 120,
    seed_skip: int = 0,
    **kwargs,
    ):
    result = plan(planner, safety_margin, seed_skip, n_states, **kwargs)

    print(f"Planner: {planner}")
    for key in ("approach", "lift", "carry"):
        motion = result[key]
        print(f"\n{key.capitalize()}")
        if "planning_ms" in motion:
            print(f"  Planning time:     {motion['planning_ms']:.2f} ms ({motion['iterations']} iterations)")
            print(
                f"  Path cost:         {motion['initial_cost']:.2f} -> {motion['simplified_cost']:.2f} (simplified)"
                )
        else:
            print("  Straight line in joint space (not planned)")
        print(f"  Waypoints:         {len(motion['waypoints'])}")
        print(f"  Base yaw range:    [{motion['yaw_range'][0]:+.2f}, {motion['yaw_range'][1]:+.2f}] rad")
        print(f"  Collision-free:    {motion['valid']} (checked against the unpadded scene)")

    if visualize:
        show_in_viser(result, urdf)


if __name__ == "__main__":
    Fire(main)
