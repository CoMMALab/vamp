"""Constrained planning with a bimanual constraint on the bimanual iiwa with grippers.

Both grippers hold one rigid object: a BimanualTaskSpaceConstraint keeps the right gripper at a
fixed pose in the left gripper's frame (+-1 mm, +-1 mrad) while the pair moves through a shelf.
The constraint is passed as `constraints=[...]` to the regular planner entry points and to simplify.

Port of the McVAMP experiment iiwa_bimanual_planning (start/goal and shelf are the same). That
experiment uses the `bimanual_iiwa` robot, whose end-effectors are the arms' link 7; here the
end-effectors are the gripper tool frames (`bimanual_iiwa_tool`), so the held pose is taken from the
start configuration instead of the link-7 constant.

Start and goal must lie on the constraint manifold: the planners raise ValueError otherwise, so
this script projects them first with the module's project() helper.

The robot is not built by default and its header must be generated with the constraint kernels
(cricket recipe `bimanual_iiwa_tool.json` has `"constraints": true`). Install with
    pip install . -C cmake.define.VAMP_ROBOT_MODULES='panda;bimanual_iiwa_tool' \
                   -C cmake.define.VAMP_ROBOT_STRUCTS='Panda;BimanualIiwaTool'

Run:
    python scripts/bimanual_iiwa_tool_constrained_example.py [--planner rrtc|grrtstar] [--visualize]
"""

import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

ROBOT = "bimanual_iiwa_tool"
ROBOT_DIR = Path(__file__).parents[1] / "resources" / "bimanual_iiwa_tool"
ROBOT_URDF = "iiwa_bimanual_spherized.urdf"

# Joint order: left arm (7), right arm (7). First two lines of McVAMP's bimanual_iiwa_start_goal.txt.
START = [
    -0.1994994216078726, 0.9140739951190965, -2.236618320862171, 0.5238879195899456,
    0.7998441913611017, -1.3575398006936048, -1.0153092816310436,
    0.24160750214358093, 0.9022665362154509, 2.2897413507442623, 0.5286854254175202,
    -0.8636815969096344, -1.4123134496102092, 1.7961899529805394,
]
GOAL = [
    -0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483,
    -0.04421061906813227, -0.8793712572715165, -1.1603461715511334,
    0.6478085507050495, 1.5420182953666222, 1.4012033615433137, 1.2931428651414443,
    0.14691416900781198, -0.9348727702323719, 1.864705746202578,
]

# Shelf (center xyz, full extents): two boards, a back wall and the floor.
SHELF_CUBOIDS = [
    ([0.8, 0.3825, 0.3], [0.4, 1.0, 0.014]),
    ([0.8, 0.3825, 0.57], [0.4, 1.0, 0.014]),
    ([1.0, 0.3825, 0.45], [0.03, 1.0, 0.9]),
    ([0.4, 0.3825, -0.2], [5.0, 5.0, 0.2]),
]

PROJ_METHODS = {
    "InnerLM": vamp.ProjMethod.InnerLM,
    "OuterLM": vamp.ProjMethod.OuterLM,
    "GradDesc": vamp.ProjMethod.GradDesc,
}


def quaternion_from_rotation(R):
    """(w, x, y, z) of a rotation matrix."""
    t = np.trace(R)
    if t > 0.0:
        s = 2.0 * np.sqrt(t + 1.0)
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])

    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def held_pose(module, q):
    """Pose (qw, qx, qy, qz, x, y, z) of the right gripper in the left gripper's frame at q."""
    left = np.array(module.eefk(q, 0))
    right = np.array(module.eefk(q, 1))
    relative = np.linalg.inv(left) @ right
    return [*quaternion_from_rotation(relative[:3, :3]), *relative[:3, 3]]


def main(
    planner: str = "rrtc",  # One of rrtc, grrtstar, aorrtc.
    range_: float = 1.0,  # Planner range; constrained steps should stay small.
    dynamic_domain: bool = False,
    method: str = "InnerLM",  # Projection method: InnerLM, OuterLM or GradDesc.
    descend_rate: float = 1.0,
    projection_iterations: int = 25,
    emit_all_waypoints: bool = False,  # Keep whole projected chains: dense on-manifold paths.
    visualize: bool = False,
    **kwargs,
):
    kwargs.setdefault("max_iterations", 100000)
    module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(
        ROBOT, planner, **kwargs)

    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc_settings.range = range_
    rrtc_settings.dynamic_domain = dynamic_domain
    for name in ("radius", "dd_radius"):  # Dynamic-domain radius: named differently per planner.
        if hasattr(rrtc_settings, name):
            setattr(rrtc_settings, name, 4.0)

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.method = PROJ_METHODS[method]
    constraint_settings.descend_rate = descend_rate
    constraint_settings.max_iterations = projection_iterations
    constraint_settings.emit_all_waypoints = emit_all_waypoints

    e = vamp.Environment()
    for center, extents in SHELF_CUBOIDS:
        e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], [d / 2.0 for d in extents]))

    start = np.array(START, dtype=np.float32)
    goal = np.array(GOAL, dtype=np.float32)

    # Hold the right gripper at its start pose in the left gripper's frame, within 1 mm / 1 mrad.
    tolerance = [0.0001] * 6
    constraints = [
        module.BimanualTaskSpaceConstraint(
            held_pose(module, start), [-t for t in tolerance], tolerance)
    ]

    seed_settings = vamp.ConstraintSettings()
    seed_settings.method = PROJ_METHODS[method]
    seed_settings.max_iterations = 100
    start_c = module.project(start, constraints, seed_settings)
    goal_c = module.project(goal, constraints, seed_settings)
    print(f"projection moved start by {np.linalg.norm(start_c - start):.5f}, "
          f"goal by {np.linalg.norm(goal_c - goal):.5f}")

    for name, q in (("start", start_c), ("goal", goal_c)):
        if not module.validate(q, e):
            raise RuntimeError(f"projected {name} configuration is in collision")

    sampler = module.halton()
    elapsed = time.perf_counter()
    result = planner_func(
        start_c, goal_c, e, plan_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    elapsed = time.perf_counter() - elapsed
    if not result.solved:
        raise RuntimeError("planning failed")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    path = simple.path
    on_manifold = all(
        module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))

    print(f"solved in {elapsed * 1e3:.1f} ms, {result.iterations} iterations and {result.nanoseconds:.1f} ns")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")
    print(f"all waypoints on manifold: {on_manifold}")

    if visualize:
        show_in_viser(module, path.numpy().copy())


def show_in_viser(module, trajectory):
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

    for i, (center, extents) in enumerate(SHELF_CUBOIDS):
        server.scene.add_box(
            f"/environment/cuboid_{i}", dimensions=tuple(extents), position=tuple(center),
            color=(160, 110, 60))

    # The two gripper tool frames: the constraint holds the right one rigid in the left one.
    left_axes = server.scene.add_frame("/left_gripper", axes_length=0.08, axes_radius=0.004)
    right_axes = server.scene.add_frame("/right_gripper", axes_length=0.08, axes_radius=0.004)

    def show(q):
        q = np.asarray(q, dtype=np.float32)
        robot.update_cfg(q[remap])
        poses = [np.array(module.eefk(q, i)) for i in range(2)]
        for frame, pose in zip((left_axes, right_axes), poses):
            frame.position = tuple(pose[:3, 3])
            frame.wxyz = tuple(quaternion_from_rotation(pose[:3, :3]))

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
