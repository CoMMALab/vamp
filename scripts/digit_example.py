"""Whole-body constrained planning on the Digit humanoid.

Port of the original scripts/cpp/planning_with_constraints/digit_example.cc: plan a
box-transport task (both hands holding a box, moving it from a shelf pickup to a rack)
under the whole-body constraint stack, passed as `constraints=[...]` to the regular
planner entry points:

- TaskSpaceConstraint: both feet are pinned to their stance poses (arms are left free).
- CoMConstraint: the center of mass stays over the support polygon between the feet.
- ClosedLoopConstraint: the two four-bar linkage rods in the legs stay closed.
- BimanualTaskSpaceConstraint (transport only): the right hand holds a fixed transform
  relative to the left, as if both grasp one rigid box.

Start and goal must lie on the constraint manifold: the planners raise ValueError
otherwise, so this script projects them first with the module's project() helper.

Run:
    python scripts/digit_example.py [--planner rrtc|aorrtc|grrtstar] [--visualize]
"""

import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

# Free-flyer q layout: base translation (x y z), base quaternion (qx qy qz qw), then the
# 24 actuated joints (left leg, left arm, right leg, right arm).
CONFIGS = {
    "standing": [
        0.028748, -0.0189052, -0.0145299, 0.00483, 0.00413, -0.00106, 0.99998,
        0.397209, -0.0082068, 0.284002, 0.290042, -0.0213535, -0.235481, -0.031921, 0.00127043,
        -0.0782953, 1.04159, 0.0284093, -0.0123198,
        -0.358284, -0.013361, -0.278441, -0.268447, 0.0177607, 0.212721, -0.0692397, -0.000343739,
        0.068962, -1.23829, 0.0369306, -0.00862385],
    "box_top_shelf_pickup": [
        0.00779, -0.02074, 0.00461, -0.00213, -0.00449, -0.00263, 0.99998,
        0.38120, 0.00154, 0.28644, 0.32323, -0.00734, -0.29696, 0.07485, 0.01125,
        0.08865, -0.26348, -0.00645, 0.11539,
        -0.37495, -0.00330, -0.29074, -0.31914, 0.01103, 0.28118, -0.13812, 0.00576,
        0.16191, 0.27674, 0.06755, -0.09470],
    "rack_2": [
        0.00927, -0.01219, -0.47283, -0.00698, -0.01481, -0.00458, 0.99986,
        0.40005, 0.01408, -0.23717, -0.84704, -0.02429, 0.90190, -0.48186, 0.04543,
        -0.17059, -0.36363, 0.06204, 1.09005,
        -0.36164, -0.00267, 0.22962, 0.84538, 0.02448, -0.90900, 0.40052, 0.03680,
        -0.29646, 0.37895, -0.15478, -1.16821],
}

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Stance poses (qw, qx, qy, qz, x, y, z) of the toe-roll frames.
LEFT_FOOT = [0.59, 0.38, 0.4, 0.59, -0.02070, 0.06015, -0.95335]
RIGHT_FOOT = [0.61, -0.36, 0.35, -0.61, -0.02228, -0.11609, -0.94832]

FREE = [10.0] * 6
PINNED = [0.001, 0.001, 0.001, 0.1, 0.1, 0.1]

# Pose (qw, qx, qy, qz, x, y, z) of the right hand in the left-hand frame while both
# grasp the box: position and roll are held, the other rotations are left free.
BOX_GRASP = [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356]
BOX_GRASP_LOWER = [-0.001, -0.001, -0.001, -0.1, -10.1, -10.1]
BOX_GRASP_UPPER = [0.001, 0.001, 0.001, 0.1, 10.1, 10.1]

# Support polygon (counterclockwise, in the feet-midpoint frame of the generated
# CoM function) that the xy projection of the center of mass must stay inside.
SUPPORT_POLYGON = [(0.01, -0.01), (0.01, 0.03), (-0.05, 0.03), (-0.05, -0.01)]

# Shelf scene as (center xyz, full extents): three boards and a back wall in front of the
# robot, plus a rack at its left. Coordinates are ground-relative (from the G1 humanoid
# shelf scene); SHELF_SHIFT drops them by digit's ground offset and nudges the scene back
# in x so the pickup and rack seeds clear the boards and back wall (minimum dx is 0.13).
SHELF_CUBOIDS = [
    ([0.45, 0.0, 0.4], [0.385, 1.0, 0.015]),
    ([0.45, 0.0, 0.79], [0.385, 1.0, 0.015]),
    ([0.45, 0.0, 1.2], [0.385, 1.0, 0.015]),
    ([0.6, 0.0, 1.0], [0.03, 1.0, 2.0]),
    ([0.0, 0.4, 0.225], [0.4, 0.3, 0.45]),
]
SHELF_SHIFT = [0.15, 0.0, -0.95]

PROJ_METHODS = {
    "InnerLM": vamp.ProjMethod.InnerLM,
    "OuterLM": vamp.ProjMethod.OuterLM,
    "GradDesc": vamp.ProjMethod.GradDesc,
}

ROBOT_DIR = Path(__file__).parents[1] / "resources" / "digit"
ROBOT_URDF = "digit_model_spherized.urdf"  # Fixed-base variant: viser drives the base frame.


def make_constraints(module, transport):
    # Module end-effector order: left arm, right arm, left toe roll, right toe roll.
    feet = module.TaskSpaceConstraint(
        [IDENTITY] * 4,
        [IDENTITY, IDENTITY, LEFT_FOOT, RIGHT_FOOT],
        [[-b for b in FREE], [-b for b in FREE], [-b for b in PINNED], [-b for b in PINNED]],
        [FREE, FREE, PINNED, PINNED],
    )
    com = module.CoMConstraint(SUPPORT_POLYGON)
    loops = module.ClosedLoopConstraint()

    constraints = [feet, com, loops]
    if transport:
        constraints.append(
            module.BimanualTaskSpaceConstraint(BOX_GRASP, BOX_GRASP_LOWER, BOX_GRASP_UPPER))

    return constraints


def main(
    start: str = "box_top_shelf_pickup",
    goal: str = "rack_2",
    transport: bool = True,  # Include the bimanual box-hold constraint.
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 0.75,  # Planner range; constrained steps should stay small.
    dynamic_domain: bool = False,
    method: str = "InnerLM",  # Projection method: InnerLM, OuterLM, or GradDesc.
    descend_rate: float = 1.0,
    projection_iterations: int = 25,
    emit_all_waypoints: bool = True,  # Keep whole projected chains: dense on-manifold paths.
    output: str = "",  # Write the simplified path to this file as CSV.
    interpolate: bool = False,  # Interpolate the saved path to the collision resolution.
    visualize: bool = False,
    **kwargs,
):
    kwargs.setdefault("max_iterations", 100000)
    (module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("digit", planner, **kwargs)

    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc_settings.range = range_
    rrtc_settings.dynamic_domain = dynamic_domain
    rrtc_settings.radius = 10.0

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.method = PROJ_METHODS[method]
    constraint_settings.descend_rate = descend_rate
    constraint_settings.max_iterations = projection_iterations
    constraint_settings.emit_all_waypoints = emit_all_waypoints

    constraints = make_constraints(module, transport)
    e = vamp.Environment()
    for center, extents in SHELF_CUBOIDS:
        shifted = [c + s for c, s in zip(center, SHELF_SHIFT)]
        e.add_cuboid(vamp.Cuboid(shifted, [0.0, 0.0, 0.0], [d / 2.0 for d in extents]))

    # Strict start/goal policy: seeds must be projected onto the manifold explicitly.
    # Projection gets a generous iteration budget independent of the planning settings.
    seed_settings = vamp.ConstraintSettings()
    seed_settings.method = PROJ_METHODS[method]
    seed_settings.max_iterations = 100
    seed_settings.descend_rate = 0.75
    start_c = module.project(np.array(CONFIGS[start], dtype=np.float32), constraints, seed_settings)
    goal_c = module.project(np.array(CONFIGS[goal], dtype=np.float32), constraints, seed_settings)

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

    print(f"{start} -> {goal} with {planner}: solved in {elapsed * 1e3:.1f} ms, "
          f"{result.iterations} iterations")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")
    print(f"all waypoints on manifold: {on_manifold}")

    if output:
        out = path
        if interpolate:
            # Linear interpolation leaves the constraint manifold: only use this for
            # consumers that reproject or track the path with a controller.
            out.interpolate_to_resolution(module.resolution())
        np.savetxt(output, out.numpy(), delimiter=",")
        print(f"wrote {len(out)} states to {output}")

    if visualize:
        from viser_utils import setup_viser_with_robot

        server, robot = setup_viser_with_robot(ROBOT_DIR, ROBOT_URDF)
        base_frame = server.scene.add_frame("/robot", show_axes=False)

        for i, (center, extents) in enumerate(SHELF_CUBOIDS):
            server.scene.add_box(
                f"/environment/cuboid_{i}",
                dimensions=tuple(extents),
                position=tuple(c + s for c, s in zip(center, SHELF_SHIFT)),
                color=(160, 110, 60),
            )

        # yourdfpy's actuated joint order can differ from the module's joint order; the
        # module's first seven names are the floating base (x y z qx qy qz qw).
        import yourdfpy

        urdf = yourdfpy.URDF.load(
            str(ROBOT_DIR / ROBOT_URDF), load_meshes=False, build_collision_scene_graph=False)
        names = list(module.joint_names())[7:]
        remap = [names.index(n) for n in urdf.actuated_joint_names]

        # The support polygon lives in world axes about the feet midpoint; draw it on
        # the ground between the feet.
        midpoint = 0.5 * (np.array(LEFT_FOOT[4:]) + np.array(RIGHT_FOOT[4:]))
        corners = [midpoint + [vx, vy, 0.0] for vx, vy in SUPPORT_POLYGON]
        server.scene.add_line_segments(
            "/constraint/support_polygon",
            points=np.array([[corners[i], corners[(i + 1) % len(corners)]]
                             for i in range(len(corners))]),
            colors=(255, 140, 0),
            line_width=4.0,
        )

        trajectory = path.numpy()

        def show(q):
            q = np.asarray(q)
            base_frame.position = q[0:3]
            base_frame.wxyz = np.array([q[6], q[3], q[4], q[5]])
            robot.update_cfg(q[7:][remap])

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
