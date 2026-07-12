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


def quat_from_matrix(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
    q = [0.0] * 4
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return q


def pose_to_transform(pose):
    """4x4 matrix -> (qw, qx, qy, qz, x, y, z)."""
    return [*quat_from_matrix(np.array(pose)[:3, :3]), *np.array(pose)[:3, 3]]


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


def bimanual_problem(module):
    # The right end-effector holds a fixed transform relative to the left, as if both
    # arms grasp one rigid object.
    start_seed = np.array(
        [-1.362, 1.319, 1.064, -2.486, 0.518, 2.481, -1.459,
         1.327, 1.260, -1.048, -2.481, -0.644, 2.444, -0.011],
        dtype=np.float32,
    )
    goal_seed = np.array(
        [-2.143, 0.395, 2.249, -2.043, 1.320, 1.772, -0.697,
         1.359, 1.320, -2.092, -2.138, -0.140, 2.437, -1.547],
        dtype=np.float32,
    )

    relative = module.BimanualTaskSpaceConstraint(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.221814],
        [-0.001] * 6,
        [0.001] * 6,
    )

    return start_seed, goal_seed, [relative], vamp.Environment()


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
    constraint_settings.max_iterations = 10

    start_seed, goal_seed, constraints, e = problem(module)

    # Strict start/goal policy: seeds must be projected onto the manifold explicitly.
    start = module.project(start_seed, constraints, constraint_settings)
    goal = module.project(goal_seed, constraints, constraint_settings)

    for name, q in (("start", start), ("goal", goal)):
        if not module.validate(q, e):
            raise RuntimeError(f"projected {name} configuration is in collision")

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
    on_manifold = all(
        module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))

    print(f"{mode} with {planner}: solved in {elapsed * 1e3:.1f} ms")
    print(f"path: {len(result.path)} -> {len(path)} states, "
          f"cost {result.path.cost():.4f} -> {path.cost():.4f}")
    print(f"all waypoints on manifold: {on_manifold}")

    if visualize:
        from viser import transforms as tf
        from viser_utils import setup_viser_with_robot, add_point_cloud, add_spheres, add_trajectory

        robot_dir = Path(__file__).parents[1] / "resources" / "panda"
        server, robot = setup_viser_with_robot(robot_dir, urdf)
        robot.update_cfg(np.array(start))

        if e.spheres:
            add_spheres(
                server,
                [s.position for s in e.spheres],
                [s.r for s in e.spheres],
                prefix="/environment/sphere",
            )

        if mode == "line":
            # The line constraint is anchored at the end-effector pose of the start seed;
            # motion is allowed only along the local z (approach) axis.
            reference = np.array(module.eefk(start_seed))
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

        if mode != "bimanual":
            # End-effector positions along the path; these should all lie on the constraint.
            ee_trace = np.array([np.array(module.eefk(q))[:3, 3] for q in path])
            add_point_cloud(server, ee_trace, colors=[0, 255, 0], point_size=0.008, prefix="/ee_trace")

        # No interpolation: linear interpolation leaves the manifold, and constrained
        # paths are already dense (planners emit whole projected waypoint chains).
        add_trajectory(server, path.numpy(), robot, [], [[]])

        print("visualization at http://localhost:8080; ctrl-c to exit")
        while True:
            time.sleep(1.0)


if __name__ == "__main__":
    Fire(main)
