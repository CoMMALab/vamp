"""Planning for the mobile bimanual `ridgeback_dual_panda` (Ridgeback base + two Pandas), visualized with viser.

The robot has 17 joints: base x, y and yaw (3), the left arm (7) and the right arm (7). The task: start
with the base at the origin and both arms folded in the SRDF "ready" pose; the goal drives the base
forward and to the side, turns it, and opens both arms up a little. A pillar and a crate stand between
the two base poses, so the base has to drive around them.

This robot is not built by default. Install it when you install vamp, for example
    pip install . -C cmake.define.VAMP_ROBOTS="panda;ridgeback_dual_panda"

Run:
    python scripts/ridgeback_dual_panda_example.py [--planner rrtc|grrtstar] [--visualize]
"""

import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

ROBOT = "ridgeback_dual_panda"
ROBOT_DIR = Path(__file__).parents[1] / "resources" / "ridgeback_dual_panda"
ROBOT_URDF = "ridgeback_dual_panda_full_primitive.urdf"

# Joint order: base (x, y, yaw), left arm (7), right arm (7).
READY_ARM = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
REACH_ARM = [0.0, -0.45, 0.0, -1.9, 0.0, 1.9, 0.785]  # ready, with the arm opened up a little
START = [0.0, 0.0, 0.0, *READY_ARM, *READY_ARM]
GOAL = [2.0, 0.9, 0.8,*REACH_ARM, *REACH_ARM]


# Obstacles, between the start and the goal base poses: a pillar of spheres (center, radius) and a crate
# (center, half extents).
PILLAR = [([0.9, 0.45, z], 0.12) for z in (0.15, 0.35, 0.55, 0.75, 0.95, 1.15)]
CRATE = ([0.9, -0.7, 0.3], [0.2, 0.2, 0.3])


def make_environment():
    e = vamp.Environment()
    for center, radius in PILLAR:
        e.add_sphere(vamp.Sphere(center, radius))

    center, half_extents = CRATE
    e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], half_extents))
    return e


def main(
    planner: str = "rrtc",  # One of rrtc, grrtstar, aorrtc.
    visualize: bool = False,
    **kwargs,
):
    module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(
        ROBOT, planner, **kwargs)

    print(f"end-effectors: {module.end_effectors()}")
    e = make_environment()

    start = np.array(START, dtype=np.float32)
    goal = np.array(GOAL, dtype=np.float32)
    for name, q in (("start", start), ("goal", goal)):
        if not module.validate(q, e):
            raise RuntimeError(f"{name} configuration is in collision")

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
    print(f"every waypoint valid: {all(module.validate(q, e) for q in trajectory)}")

    if visualize:
        show_in_viser(module, trajectory)


def show_in_viser(module, trajectory):
    import viser
    import yourdfpy
    from viser.extras import ViserUrdf

    server = viser.ViserServer()
    urdf = yourdfpy.URDF.load(
        str(ROBOT_DIR / ROBOT_URDF), load_meshes=False, build_collision_scene_graph=True)
    robot = ViserUrdf(
        server, urdf, load_meshes=False, load_collision_meshes=True, root_node_name="/robot")

    # yourdfpy's actuated joint order differs from the module's (right arm first).
    names = list(module.joint_names())
    remap = [names.index(n) for n in urdf.actuated_joint_names]

    for i, (center, radius) in enumerate(PILLAR):
        server.scene.add_icosphere(
            f"/environment/pillar_{i}", radius=radius, position=tuple(center), color=(160, 110, 60))

    center, half_extents = CRATE
    server.scene.add_box(
        "/environment/crate",
        dimensions=tuple(2.0 * h for h in half_extents),
        position=tuple(center),
        color=(120, 120, 130))

    def show(q):
        robot.update_cfg(np.asarray(q, dtype=np.float32)[remap])

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
