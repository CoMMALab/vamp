"""Planning with two attachments on a bimanual iiwa, visualized with viser.

The `bimanual_iiwa_tool` robot has two end-effectors, the tool frames of the left and right gripper
(`end_effectors()` lists them). A held object is an `Attachment` riding on one of them: here the
left gripper holds a bottle (three stacked spheres) and the right gripper holds a ball. Both are
checked against the environment and the robot while planning, and against each other, because
they are attached to different end-effectors (attachments on the same end-effector move rigidly
together and are never tested against each other).

The task: both arms hold their object in front of a pillar standing between them (on a table); the
goal has both objects behind it, so each arm has to carry its object around the pillar on its own
side without touching it, the table, the other arm, or the other object.

This robot is not built by default. Install it when you install vamp, for example
    pip install . -C cmake.define.VAMP_ROBOT_MODULES='panda;bimanual_iiwa_tool' \
                   -C cmake.define.VAMP_ROBOT_STRUCTS='Panda;BimanualIiwaTool'

Run:
    python scripts/bimanual_attachments_example.py [--planner rrtc|grrtstar] [--visualize]

`aorrtc` also works but keeps improving the path until max_iterations (`--max_iterations=...`).
"""

import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

ROBOT = "bimanual_iiwa_tool"
ROBOT_DIR = Path(__file__).parents[1] / "resources" / "bimanual_iiwa_tool"
ROBOT_URDF = "iiwa_bimanual_spherized.urdf"

# Joint order: left arm (7), right arm (7).
START = [
    *[-0.109, -0.509, 0.289, -1.881, -0.118, 0.944, -2.659],
    *[1.232, -0.970, -0.660, -1.881, -0.710, 1.154, 0.772],
]
GOAL = [
    *[1.109, 1.689, -1.786, -1.772, -0.043, -0.577, -1.166],
    *[-0.714, 0.533, -2.561, 1.570, -1.298, 0.244, 0.692],
]

# Attachments, in the frame of the end-effector they ride on: (sphere centers, radius).
LEFT_OBJECT = ([[0.0, 0.0, 0.10], [0.0, 0.0, 0.18], [0.0, 0.0, 0.26]], 0.04)  # bottle
RIGHT_OBJECT = ([[0.0, 0.0, 0.12]], 0.06)  # ball

# Obstacles: (center, radius) spheres forming a pillar between the arms, and a table cuboid.
PILLAR = [([0.65, 0.385, z], 0.07) for z in (0.2, 0.35, 0.5, 0.65, 0.8)]
TABLE = ([0.55, 0.385, -0.16], [0.6, 0.8, 0.1])  # (center, half extents)


def make_attachment(spheres, radius, end_effector):
    """An attachment on the given end-effector with the object's spheres in its frame."""
    attachment = vamp.Attachment(np.identity(4), end_effector)
    attachment.add_spheres([vamp.Sphere(center, radius) for center in spheres])
    return attachment


def make_environment(attachments):
    e = vamp.Environment()
    for center, radius in PILLAR:
        e.add_sphere(vamp.Sphere(center, radius))

    center, half_extents = TABLE
    e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], half_extents))
    for attachment in attachments:
        e.attach(attachment)

    return e


def main(
    planner: str = "rrtc",  # One of rrtc, grrtstar, aorrtc.
    visualize: bool = False,
    **kwargs,
):
    module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(
        ROBOT, planner, **kwargs)

    ends = module.end_effectors()
    print(f"end-effectors: {ends}")

    left = make_attachment(*LEFT_OBJECT[:1], LEFT_OBJECT[1], 0)
    right = make_attachment(*RIGHT_OBJECT[:1], RIGHT_OBJECT[1], 1)
    e = make_environment([left, right])

    start = np.array(START, dtype=np.float32)
    goal = np.array(GOAL, dtype=np.float32)
    for name, q in (("start", start), ("goal", goal)):
        if not module.validate(q, e):
            raise RuntimeError(f"{name} configuration is in collision")

    # The attachments are what makes the arms interact: with the objects removed the same
    # configuration may be valid, with them it is not if they touch each other or the robot.
    sampler = module.halton()
    elapsed = time.perf_counter()
    result = planner_func(start, goal, e, plan_settings, sampler)
    elapsed = time.perf_counter() - elapsed
    if not result.solved:
        raise RuntimeError("planning failed")

    simple = module.simplify(result.path, e, simp_settings, sampler)
    path = simple.path
    path.interpolate_to_resolution(module.resolution())

    print(f"solved in {elapsed * 1e3:.1f} ms, {result.iterations} iterations; "
          f"path {len(result.path)} -> {len(path)} states, cost {path.cost():.3f}")

    trajectory = path.numpy()
    print(f"every waypoint valid with both attachments: "
          f"{all(module.validate(q, e) for q in trajectory)}")

    if visualize:
        show_in_viser(module, trajectory, (left, right), (LEFT_OBJECT, RIGHT_OBJECT))


def show_in_viser(module, trajectory, attachments, objects):
    import viser
    import yourdfpy
    from viser.extras import ViserUrdf

    server = viser.ViserServer()
    urdf = yourdfpy.URDF.load(
        str(ROBOT_DIR / ROBOT_URDF), load_meshes=False, build_collision_scene_graph=True)
    robot = ViserUrdf(
        server, urdf, load_meshes=False, load_collision_meshes=True, root_node_name="/robot")

    # yourdfpy's actuated joint order can differ from the module's.
    names = list(module.joint_names())
    remap = [names.index(n) for n in urdf.actuated_joint_names]

    for i, (center, radius) in enumerate(PILLAR):
        server.scene.add_icosphere(
            f"/environment/pillar_{i}", radius=radius, position=tuple(center), color=(160, 110, 60))

    center, half_extents = TABLE
    server.scene.add_box(
        "/environment/table",
        dimensions=tuple(2.0 * h for h in half_extents),
        position=tuple(center),
        color=(120, 120, 130))

    handles = []
    for k, (spheres, radius) in enumerate(objects):
        color = (220, 60, 60) if k == 0 else (60, 90, 220)
        handles.append([
            server.scene.add_icosphere(
                f"/attachment_{k}/sphere_{j}", radius=radius, position=tuple(c), color=color)
            for j, c in enumerate(spheres)
        ])

    def show(q):
        q = np.asarray(q, dtype=np.float32)
        robot.update_cfg(q[remap])
        for k, attachment in enumerate(attachments):
            attachment.set_ee_pose(module.eefk(q, attachment.end_effector))
            for handle, sphere in zip(handles[k], attachment.posed_spheres):
                handle.position = tuple(sphere.position)

    show(trajectory[0])
    slider = server.gui.add_slider(
        "Current Waypoint", min=0, max=len(trajectory) - 1, step=1, initial_value=0)
    autoplay = server.gui.add_checkbox("Autoplay", initial_value=True)
    rate = server.gui.add_slider(
        "Playback Rate (Hz)", min=1.0, max=60.0, step=1.0, initial_value=20.0)

    @slider.on_update
    def _(event):
        show(trajectory[int(event.target.value)])

    print("visualization at http://localhost:8080; ctrl-c to exit")
    while True:
        if autoplay.value:
            slider.value = (slider.value + 1) % len(trajectory)

        time.sleep(1.0 / rate.value)


if __name__ == "__main__":
    Fire(main)
