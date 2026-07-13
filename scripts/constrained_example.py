"""Constrained planning examples.

Plans with manifold constraints by passing `constraints=[...]` to the regular planner
entry points (rrtc, aorrtc, grrtstar) and simplify:

- line: the Panda's end-effector may only translate along its approach axis.
- plane: the Panda's end-effector slides in a fixed-orientation plane through a sphere cage.
- bimanual: the two arms of the bimanual Panda hold a fixed relative grasp transform.

Start and goal must lie on the constraint manifold: the planners raise ValueError
otherwise, so this script projects them first with the module's project() helper.

With --flask, the examples plan kinodynamically instead: the chart-LQMT planner on
the robot's flask submodule steers rest-to-rest (q, qdot) states along time-optimal
cubics on the constraint manifold. Constraints are still built from the ambient
submodule; the executed trajectory is reconstructed with lift_edge.

With --retime, the examples plan geometrically as usual and then retime the path on
the flask sibling: waypoints are lifted to rest states and the flask simplify's
C_loc shortcutting (validated chart-LQMT edges on the manifold) blends the
stop-at-every-waypoint lift into flowing motion.
"""

from pathlib import Path
import time

import numpy as np
import vamp
from vamp import flask as vf
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

# Playback rate for geometric (non-flask) paths, which carry no timing.
PLAYBACK_FPS = 30.0


def lift_path(module, path, constraints, constraint_settings, chart_settings):
    """Lift every edge of a z-path through the chart machinery; per edge pick the
    direction whose reconstructed endpoint matches (goal-tree edges lift backward and
    are executed in reverse). The reconstruction shoots toward the recorded endpoint
    and stops once attained, so misses up to ~reached_pos_tol are inherent; beyond 2x
    the edge is considered unreconstructible. Returns (dense states, per-state times,
    max constraint error, duration, max endpoint miss)."""
    n_q = len(path[0]) // 2
    dense = []
    times = []
    max_err = 0.0
    total_T = 0.0
    max_miss = 0.0
    for i in range(len(path) - 1):
        a, b = np.asarray(path[i]), np.asarray(path[i + 1])
        best = None
        for frm, tgt, fwd, endpoint in ((a, b, True, b), (b, a, False, a)):
            try:
                states, errs, T, _ = module.lift_edge(
                    frm, tgt, constraints, forward=fwd, n_samples=32,
                    constraint_settings=constraint_settings, chart_settings=chart_settings)
            except ValueError:
                continue
            miss = float(np.linalg.norm(np.asarray(states[-1])[:n_q] - endpoint[:n_q]))
            if best is None or miss < best[0]:
                best = (miss, states, errs, T, fwd)
        if best is None:
            raise RuntimeError(f"edge {i} of the path could not be lifted")
        miss, states, errs, T, fwd = best
        if miss > 2.0 * chart_settings.reached_pos_tol:
            raise RuntimeError(f"lifted edge {i} misses its endpoint by {miss:.2e}")
        dense.extend(np.asarray(s) for s in (states if fwd else states[::-1]))
        # lift_edge samples at uniform times in [0, T], so timestamps are recoverable.
        times.extend(total_T + np.linspace(0.0, float(T), len(states)))
        max_err = max(max_err, float(max(errs)))
        total_T += float(T)
        max_miss = max(max_miss, miss)
    return np.array(dense, dtype=np.float32), np.array(times), max_err, total_T, max_miss


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
    flask: bool = False,  # Plan kinodynamically on the manifold with the chart-LQMT planner.
    retime: bool = False,  # Plan geometrically, then retime on the flask sibling.
    rho: float | None = None,  # LQMT time weight in C = rho * T + int |u|^2 (flask/retime).
    max_kinetic_energy: float | None = None,  # Phase gate: kinetic energy cap in J (flask/retime).
    max_eef_speed: float | None = None,  # Phase gate: end-effector speed cap in m/s (flask/retime).
    sample_energy: float | None = None,  # Shape sampled kinetic energy to [0, cap] J (flask only).
    visualize: bool = False,
    **kwargs,
):
    robot_name, problem, urdf = PROBLEMS[mode]
    if flask and retime:
        raise ValueError("--flask already plans kinodynamically; --retime retimes a geometric plan")
    kinodynamic = flask or retime
    if rho is not None and not kinodynamic:
        raise ValueError("--rho only applies to the chart-LQMT machinery; pass --flask or --retime")
    if (max_kinetic_energy is not None or max_eef_speed is not None) and not kinodynamic:
        raise ValueError("phase gates only apply to the chart-LQMT machinery; pass --flask or --retime")
    if sample_energy is not None and not flask:
        raise ValueError("--sample_energy only applies to the chart-LQMT planner; pass --flask")
    if flask:
        kwargs.setdefault("max_iterations", 50000)
        kwargs.setdefault("max_samples", 50000)
        robot_name = f"{robot_name}.flask"
        if mode == "bimanual" and max_kinetic_energy is None:
            # Unbounded sampled velocities make LQMT steer arcs sweep wide and clip
            # the shelf boards; a kinetic-energy gate rest-biases steers enough to
            # thread the below->above passage (unsolvable without it).
            max_kinetic_energy = 1.0

    (module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot_name, planner, **kwargs)

    # The module that owns the final path: with --retime the flask sibling (whose simplify
    # retimes the geometric plan); otherwise the planning module itself.
    path_module, flask_simp_settings = module, simp_settings
    if retime:
        (path_module, _, _, flask_simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
            f"{robot_name}.flask", planner, **kwargs)
    if kinodynamic and rho is not None:
        path_module.set_rho(rho)

    if planner == "rrtc" or planner == "grrtstar":
        plan_settings.range = range_
    elif planner == "aorrtc":
        plan_settings.rrtc.range = range_

    if flask:
        # Flat z-space sample distances dwarf the geometric dynamic-domain radius
        # default (4.0), which starves the trees after the first trapped chart steer.
        if planner == "rrtc":
            plan_settings.radius = 8.0
        elif planner == "grrtstar":
            plan_settings.dd_radius = 8.0
        elif planner == "aorrtc":
            plan_settings.rrtc.radius = 8.0

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = 10

    # Constraints always come from the ambient (geometric) robot's submodule; a flask
    # robot is a nested submodule of its ambient parent.
    ambient = getattr(vamp, robot_name.partition(".")[0])
    start_seed, goal_seed, constraints, e = problem(ambient)
    n_q = len(start_seed)

    chart_kwargs = {}
    if kinodynamic:
        chart_kwargs["chart_settings"] = vamp.ChartSettings()
        gates = vf.phase_constraints(path_module, max_kinetic_energy, max_eef_speed)
        if gates:
            chart_kwargs["phase_constraints"] = gates

    # The geometric pipeline in retime mode takes no chart kwargs; only flask planning
    # and the retiming simplify do.
    plan_chart_kwargs = chart_kwargs if flask else {}

    if flask:
        start_seed, goal_seed = vf.rest_state(start_seed), vf.rest_state(goal_seed)

    # Strict start/goal policy: seeds must be projected onto the manifold explicitly.
    start = module.project(start_seed, constraints, constraint_settings, **plan_chart_kwargs)
    goal = module.project(goal_seed, constraints, constraint_settings, **plan_chart_kwargs)

    for name, q in (("start", start), ("goal", goal)):
        if not module.validate(q, e):
            raise RuntimeError(f"projected {name} configuration is in collision")

    sampler = module.halton()
    if sample_energy is not None:
        sampler = module.ke_shaped(sampler, float(sample_energy))
    elapsed = time.perf_counter()
    result = planner_func(
        start, goal, e, plan_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings, **plan_chart_kwargs)
    elapsed = time.perf_counter() - elapsed

    if not result.solved:
        raise RuntimeError("planning failed")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings, **plan_chart_kwargs)

    path = simple.path

    tag = " (flask)" if flask else " (retime)" if retime else ""
    print(f"{mode} with {planner}{tag}: solved in {elapsed * 1e3:.1f} ms")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")

    if retime:
        # Retime on the flask sibling: lift the geometric waypoints to rest states and
        # let the flask simplify's C_loc shortcutting blend the stops into flowing
        # motion. Shortcut edges are chart-LQMT cubics validated on the manifold.
        lifted = vf.lift(path_module, path)
        rest_cost = lifted.cost()
        retime_elapsed = time.perf_counter()
        flask_simple = path_module.simplify(
            lifted, e, flask_simp_settings, path_module.halton(),
            constraints=constraints, constraint_settings=constraint_settings, **chart_kwargs)
        retime_elapsed = time.perf_counter() - retime_elapsed

        path = flask_simple.path
        print(f"retime: {len(lifted)} -> {len(path)} states, "
              f"cost {rest_cost:.4f} -> {path.cost():.4f}, in {retime_elapsed * 1e3:.1f} ms")

    on_manifold = all(
        path_module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))
    print(f"all waypoints on manifold: {on_manifold}")

    dense = dense_t = None
    if kinodynamic:
        dense, dense_t, max_err, total_T, max_miss = lift_path(
            path_module, path, constraints, constraint_settings, chart_kwargs["chart_settings"])
        vmax = np.abs(dense[:, n_q:]).max(axis=0)
        vel_ratio = float((vmax / np.array(path_module.velocity_limits())).max())
        print(f"trajectory: {total_T:.2f} s over {len(path) - 1} segments, "
              f"max constraint dist^2 {max_err ** 2:.2e}, max velocity ratio {vel_ratio:.2f}, "
              f"max endpoint miss {max_miss:.2e}")
        if "phase_constraints" in chart_kwargs:
            # Enforcement is at the planner's validation samples, so uniformly-resampled
            # lift samples may peak slightly between them.
            max_ke = max(float(path_module.kinetic_energy(z)) for z in dense)
            max_ee = max(vf.max_eef_speed(path_module, z) for z in dense)
            ke_cap = f" (cap {max_kinetic_energy:g} J)" if max_kinetic_energy is not None else ""
            ee_cap = f" (cap {max_eef_speed:g} m/s)" if max_eef_speed is not None else ""
            print(f"phase: max kinetic energy {max_ke:.3f} J{ke_cap}, "
                  f"max EEF speed {max_ee:.3f} m/s{ee_cap}")

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
            reference = np.array(ambient.eefk(start_seed[:n_q]))
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

        # Geometric paths are already dense (planners emit whole projected waypoint
        # chains); flask paths are lifted to the executed trajectory. Either way, no
        # interpolation: linear interpolation leaves the manifold.
        # numpy() is a read-only view and the bindings only take writable arrays, so copy.
        waypoints = dense[:, :n_q] if kinodynamic else path.numpy().copy()

        if mode != "bimanual":
            # End-effector positions along the path; these should all lie on the constraint.
            ee_trace = np.array([np.array(ambient.eefk(q))[:3, 3] for q in waypoints])
            add_point_cloud(server, ee_trace, colors=[0, 255, 0], point_size=0.008, prefix="/ee_trace")

        slider = add_trajectory(server, waypoints, robot, [], [[]])
        play = server.gui.add_checkbox("Play", initial_value=True)

        # Playback schedule: flask trajectories replay on the lifted wall-clock timing;
        # geometric paths carry no timing, so they step at a constant rate.
        play_times = dense_t if kinodynamic else np.arange(len(waypoints)) / PLAYBACK_FPS
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
