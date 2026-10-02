"""Constrained planning examples.

Plans with manifold constraints by passing `constraints=[...]` to the regular planner
entry points (rrtc, aorrtc, grrtstar) and simplify:

- line: the Panda's end-effector may only translate along its approach axis.
- plane: the Panda's end-effector slides in a fixed-orientation plane through a sphere cage.
- bimanual: the two arms of the bimanual Panda hold a fixed relative grasp transform.

Start and goal must lie on the constraint manifold: the planners raise ValueError
otherwise, so this script projects them first with the module's project() helper.
"""

from pathlib import Path
import time

import numpy as np
import vamp
from vamp import transformations as tr
from fire import Fire

# Sphere cage for the plane example (from the original constrained-planning demos).
PLANE_PROBLEM = [
    [0.56, 0, 0.450],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Allowed end-effector travel along the line constraint's axis, in meters.
LINE_EXTENT = 0.6

# Reference pose (qw, qx, qy, qz, x, y, z) of the plane constraint: the end-effector
# slides in the local x-y plane of this frame with fixed orientation.
PLANE_POSE = [0.0, 0.707107, 0.0, 0.707107, 0.354, 0.7, 0.243]

# Playback rate for visualization; paths carry no timing.
PLAYBACK_FPS = 30.0


def pose_to_transform(pose):
    """4x4 matrix -> (qw, qx, qy, qz, x, y, z)."""
    pose = np.asarray(pose)
    x, y, z, w = tr.quaternion_from_matrix(pose)
    return [w, x, y, z, *pose[:3, 3]]


def line_problem(module):
    # The end-effector may only translate along the approach (local z) axis of its
    # starting pose; position off-axis and orientation are held to +- 0.01.
    start_seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
    goal_seed = start_seed + np.array([0.0, 0.35, 0.0, 0.45, 0.0, -0.4, 0.0], dtype=np.float32)

    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        pose_to_transform(module.eefk(start_seed)),
        [-0.01, -0.01, -LINE_EXTENT, -0.01, -0.01, -0.01],
        [0.01, 0.01, LINE_EXTENT, 0.01, 0.01, 0.01],
    )

    return start_seed, goal_seed, [tsr], vamp.Environment()


def plane_problem(module):
    # The end-effector slides in the x-y plane of a fixed reference frame (free in two
    # translation axes) with fixed orientation, through a cage of spheres.
    start_seed = np.array([-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761], dtype=np.float32)
    goal_seed = np.array([-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034], dtype=np.float32)

    tsr = module.TaskSpaceConstraint(
        IDENTITY,
        PLANE_POSE,
        [-10.01, -10.01, -0.01, -0.01, -0.01, -0.01],
        [10.01, 10.01, 0.01, 0.01, 0.01, 0.01],
    )

    e = vamp.Environment()
    for sphere in PLANE_PROBLEM:
        e.add_sphere(vamp.Sphere(sphere, 0.15))

    return start_seed, goal_seed, [tsr], e


# Shelf for the bimanual example: two boards, a center divider, and the ground
# (center, full extents).
BIMANUAL_SHELF = [
    ([0.5, 0.0, 0.2], [0.3, 3.0, 0.014]),
    ([0.5, 0.0, 0.4], [0.3, 3.0, 0.014]),
    ([0.5, 0.0, 0.3], [0.3, 0.01, 0.15]),
    ([0.0, 0.0, -0.2], [5.0, 5.0, 0.2]),
]


def bimanual_problem(module):
    # The right end-effector holds a fixed transform relative to the left, as if both
    # arms grasp one rigid object.
    # Both seeds keep >5cm clearance from the shelf: the object starts held low in
    # front of the shelf and ends held above the top board.
    start_seed = np.array(
        [-1.388458, 1.789655, 0.526891, -2.779171, -0.986079, 3.079894, -0.75567,
         1.630401, 0.982874, -0.542026, -2.682339, -0.376891, 2.048735, 0.422199],
        dtype=np.float32,
    )
    goal_seed = np.array(
        [-2.118829, 0.419675, 2.249477, -2.045575, 1.324726, 1.762167, -0.726496,
         1.423746, 1.290487, -2.148522, -2.168713, -0.143205, 2.446808, -1.55776],
        dtype=np.float32,
    )

    relative = module.BimanualTaskSpaceConstraint(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.221814],
        [-0.001] * 6,
        [0.001] * 6,
    )

    e = vamp.Environment()
    for center, extents in BIMANUAL_SHELF:
        e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], [x / 2.0 for x in extents]))

    return start_seed, goal_seed, [relative], e


PROBLEMS = {
    "line": ("panda", line_problem, "panda_spherized.urdf"),
    "plane": ("panda", plane_problem, "panda_spherized.urdf"),
    "bimanual": ("bimanual_panda", bimanual_problem, "bipanda_spherized.urdf"),
}


def main(
    mode: str = "plane",  # One of line, plane, bimanual.
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 0.5,  # Planner range; constrained steps should stay small.
    visualize: bool = False,
    # ConstraintSettings (None keeps the library default, except max_iterations).
    constraint_method: str | None = None,  # InnerLM, OuterLM or GradDesc.
    constraint_descend_rate: float | None = None,
    constraint_tolerance: float | None = None,  # Squared-violation convergence threshold.
    constraint_max_iterations: int = 50,  # Projection iteration cap (library default is 15).
    constraint_perturbation_scale: float | None = None,
    constraint_emit_all_waypoints: bool | None = None,
    constraint_connect_slack: float | None = None,
    constraint_reached_radius2: float | None = None,
    constraint_endpoint_tolerance2: float | None = None,
    constraint_hold_satisfied_rows: bool | None = None,  # Keep (library default: True) or drop satisfied rows in the projection Jacobian.
    **kwargs,
):
    robot_name, problem, urdf = PROBLEMS[mode]

    (module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot_name, planner, **kwargs)

    if planner == "rrtc" or planner == "grrtstar":
        plan_settings.range = range_
    elif planner == "aorrtc":
        plan_settings.rrtc.range = range_

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = constraint_max_iterations
    if constraint_method is not None:
        constraint_settings.method = getattr(vamp.ProjMethod, constraint_method)
    for name, value in (
        ("descend_rate", constraint_descend_rate),
        ("tolerance", constraint_tolerance),
        ("perturbation_scale", constraint_perturbation_scale),
        ("emit_all_waypoints", constraint_emit_all_waypoints),
        ("connect_slack", constraint_connect_slack),
        ("reached_radius2", constraint_reached_radius2),
        ("endpoint_tolerance2", constraint_endpoint_tolerance2),
        ("hold_satisfied_rows", constraint_hold_satisfied_rows),
    ):
        if value is not None:
            setattr(constraint_settings, name, value)

    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    planner_fields = " ".join(
        f"{name}={getattr(rrtc_settings, name, 'n/a')}"
        for name in ("range", "dynamic_domain", "radius", "dd_radius", "max_iterations", "max_samples")
        if hasattr(rrtc_settings, name))
    print(f"planner settings ({planner}): {planner_fields}")
    print(
        "constraint settings: " +
        " ".join(f"{name}={getattr(constraint_settings, name)}" for name in (
            "method", "descend_rate", "tolerance", "max_iterations", "perturbation_scale",
            "emit_all_waypoints", "connect_slack", "reached_radius2", "endpoint_tolerance2",
            "hold_satisfied_rows")))

    start_seed, goal_seed, constraints, e = problem(module)
    n_q = len(start_seed)

    # Strict start/goal policy: seeds must be projected onto the manifold explicitly.
    def check(label, q):
        """Print and return (on_manifold, collision_free) for a configuration."""
        on_manifold = bool(module.satisfied(q, constraints, constraint_settings))
        collision_free = bool(module.validate(q, e))
        print(f"  {label:<16} on_manifold={'PASS' if on_manifold else 'FAIL'}  "
              f"collision_free={'PASS' if collision_free else 'FAIL'}")
        return on_manifold, collision_free

    print("start/goal checks:")
    check("start (seed)", start_seed)
    check("goal (seed)", goal_seed)

    start = module.project(start_seed, constraints, constraint_settings)
    goal = module.project(goal_seed, constraints, constraint_settings)

    failed = []
    for name, seed, q in (("start", start_seed, start), ("goal", goal_seed, goal)):
        on_manifold, collision_free = check(f"{name} (projected)", q)
        print(f"  {name} moved by projection: {float(np.linalg.norm(np.asarray(q) - np.asarray(seed))):.5f}")
        if not on_manifold:
            failed.append(f"{name} is not on the constraint manifold")
        if not collision_free:
            failed.append(f"{name} is in collision")

    if failed:
        raise RuntimeError("projected start/goal check failed: " + "; ".join(failed))

    sampler = module.halton()
    elapsed = time.perf_counter()
    result = planner_func(
        start, goal, e, plan_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    elapsed = time.perf_counter() - elapsed

    if not result.solved:
        raise RuntimeError("planning failed")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)

    path = simple.path

    print(f"{mode} with {planner}: solved in {elapsed * 1e3:.1f} ms with {result.iterations} iterations")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")

    on_manifold = all(
        module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))
    print(f"all waypoints on manifold: {on_manifold}")

    if visualize:
        from viser import transforms as tf
        from viser_utils import setup_viser_with_robot, add_point_cloud, add_spheres, add_trajectory

        robot_dir = Path(__file__).parents[1] / "resources" / "panda"
        server, robot = setup_viser_with_robot(robot_dir, urdf)
        robot.update_cfg(np.asarray(start)[:n_q])

        if e.spheres:
            add_spheres(
                server,
                [s.position for s in e.spheres],
                [s.r for s in e.spheres],
                prefix="/environment/sphere",
            )

        for i, c in enumerate(e.cuboids + e.z_aligned_cuboids):
            axes = np.array([[getattr(c, f"axis_{j}_{k}") for j in (1, 2, 3)] for k in "xyz"])
            server.scene.add_box(
                f"/environment/cuboid_{i}",
                color=(160, 160, 160),
                dimensions=tuple(2.0 * getattr(c, f"axis_{j}_r") for j in (1, 2, 3)),
                wxyz=tf.SO3.from_matrix(axes).wxyz,
                position=np.array([c.x, c.y, c.z]),
            )

        if mode == "line":
            # The line constraint is anchored at the end-effector pose of the start seed;
            # motion is allowed only along the local z (approach) axis.
            reference = np.array(module.eefk(start_seed[:n_q]))
            rotation, origin = reference[:3, :3], reference[:3, 3]
            axis = rotation[:, 2]
            server.scene.add_line_segments(
                "/constraint/line",
                points=np.array([[origin - LINE_EXTENT * axis, origin + LINE_EXTENT * axis]]),
                colors=(255, 140, 0),
                line_width=4.0,
            )
            server.scene.add_frame(
                "/constraint/reference",
                wxyz=tf.SO3.from_matrix(rotation).wxyz,
                position=origin,
                axes_length=0.1,
                axes_radius=0.004,
            )
        elif mode == "plane":
            server.scene.add_box(
                "/constraint/plane",
                color=(255, 140, 0),
                dimensions=(1.4, 1.4, 0.002),
                opacity=0.25,
                wxyz=np.array(PLANE_POSE[:4]),
                position=np.array(PLANE_POSE[4:]),
            )

        # Paths are already dense (planners emit whole projected waypoint chains): no
        # interpolation, since linear interpolation leaves the manifold.
        # numpy() is a read-only view and the bindings only take writable arrays, so copy.
        waypoints = path.numpy().copy()

        if mode != "bimanual":
            # End-effector positions along the path; these should all lie on the constraint.
            ee_trace = np.array([np.array(module.eefk(q))[:3, 3] for q in waypoints])
            add_point_cloud(server, ee_trace, colors=[0, 255, 0], point_size=0.008, prefix="/ee_trace")

        slider = add_trajectory(server, waypoints, robot, [], [[]])
        play = server.gui.add_checkbox("Play", initial_value=True)

        # Paths carry no timing, so they step at a constant rate.
        play_times = np.arange(len(waypoints)) / PLAYBACK_FPS
        period = play_times[-1] + 1.0  # Hold the goal pose for a beat before looping.

        print(f"visualization at http://localhost:{server.get_port()}; ctrl-c to exit")
        t0 = time.perf_counter()
        was_playing = True
        while True:
            if play.value:
                if not was_playing:  # Resume from wherever the slider was scrubbed to.
                    t0 = time.perf_counter() - play_times[int(slider.value)]
                t = min((time.perf_counter() - t0) % period, play_times[-1])
                idx = int(np.searchsorted(play_times, t, side="right") - 1)
                if idx != int(slider.value):
                    slider.value = idx  # Fires the slider callback, which moves the robot.
            was_playing = play.value
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    Fire(main)
