"""Plans along a spline from Python: vamp.panda.task_space_rrtc with a SplineConstraint as the manifold.

A cubic B-spline is fitted through the first 20 end-effector positions of the first trajectory in a file written by
evaluate_mbm.py --save_eef (default /tmp/eef_bookshelf_small.json). The tree starts at that trajectory's start
configuration, which is on the curve, and every state of it stays on the curve: extensions toward random samples and
the goal-directed steps are projected onto it (manifold_constraints). The goal is the end of the curve, P(1): the
planner holds the manifold's own constraint at t = 1 as the goal region (fixed-parameter steps, never the solve for
t), so no goal constraint and no goal configuration is given.

With --shape s the curve is instead a synthetic S, 20 waypoints at equal arc length along two joined arcs (radius
--s-radius, default 10 cm) in a vertical plane in front of the Panda, centred on the end-effector of the ready pose:
by default 0.84 m long, 20 cm wide and 40 cm tall, with two turns of 240 degrees. The goal is the lower-left end,
and the pull toward it is not monotone along the curve (the path first leaves the goal's side), so this is the hard
case for the goal walk. The environment is empty then, and the start configuration is the ready pose projected onto
a point region at the S's start (about 17 cm from where the ready pose's end-effector is).

The environment is the MBM problem the trajectory was planned for (default bookshelf_small, index 0), or empty with
--no-env. Checked: the planner solves; every state of the path is on the curve and collision-free; the path ends
at P(1). The path is written to <prefix>_path.json, in the format of cricket's tests/view_spline.py
(the curve, waypoints and control points, the Panda at each state, the curve point each is matched to):

    python3 scripts/spline_task_space_rrtc_example.py --view
    python3 cricket/tests/view_spline.py spline_task_space_rrtc_py_path.json

With --qt the planner works in (q, t) instead (vamp.panda.task_space_rrtc_spline): states carry the curve
parameter t, which is sampled with the joints and held fixed while a state is projected onto the curve, so no
closest point of the curve is searched for during planning. The path then comes with the t of every state.

With --curve catmull-rom the curve is a Catmull-Rom curve through the same points (vamp.panda.CatmullRomConstraint,
centripetal by default, --alpha) instead of the fitted B-spline; it is only planned in (q, t), so --qt is implied.
Its curve and points are written to the dump as the "spline" (the viewer draws the curve through the points).

With --curve linear the curve is the straight-line interpolation of the points (vamp.panda.PolylineConstraint, t the
chord-length parameter): the baseline for how much the smooth curves differ from joining the dots. Also only planned in
(q, t).

With --curve linear-se3 the curve is a pose curve in R^3 x SO(3) (vamp.panda.PosePolylineConstraint): the end-effector's
position follows the polyline of the points and its rotation the slerp between keyframe rotations, which start at the
start's rotation and turn about the world z axis by --turn degrees in equal steps per keyframe. The error has six rows,
the rotation ones weighted by --rot-weight (m/rad), which is also the weight of rotation in the metric of t. The goal
is the last keyframe's pose, and the path is checked for the curve's rotation as well as its position. Planned in
(q, t) too.

With --slide (linear and linear-se3) the planner does not hold a state at the point P(t): t only picks the segment the
state is on, and every projection lets the end-effector (its pose, for linear-se3) slide along that segment, the state
then taking the t the projection left it at. Compare the two with and without it: the goal is still the end of the
curve.

Needs a Panda built with a header generated with "spline_constraint": true (cricket); with --curve catmull-rom also
"catmull_rom_constraint": true, with --curve linear also "position_target_constraint": true; --curve linear-se3 also
needs "pose_slide_constraint": true (it uses the task-space region's kernel, and its own solvers with t as an unknown).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import vamp
from vamp import mbm

import spline_constraint_example as sce

N = sce.N


READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

# The S is two arcs of 240 degrees each, so its length is 2 * (4 pi / 3) radii
S_LENGTH_PER_RADIUS = 8.0 * np.pi / 3.0

# The Panda's shoulder (joint 2) is 0.333 m above its base; the arm reaches about 0.855 m from it
SHOULDER = np.array([0.0, 0.0, 0.333])
REACH = 0.855


def s_curve_points(radius=0.10, count=N):
    """An S in a vertical plane (x constant), centred on the end-effector of the ready pose.

    Two arcs of radius `radius` joined smoothly at the middle, which is the centre: an upper one
    (counter-clockwise, 30 to 270 degrees) and a lower one (clockwise, from the middle to 210 degrees). It is 4 radii
    tall and 2 wide, and 8.4 radii long. Returns `count` points at equal arc length.
    """
    upper_angle = np.linspace(np.pi / 6, 3 * np.pi / 2, 1200)
    upper = np.stack([radius * np.cos(upper_angle), radius + radius * np.sin(upper_angle)], axis=1)
    lower_angle = np.linspace(np.pi / 2, -5 * np.pi / 6, 1200)
    lower = np.stack([radius * np.cos(lower_angle), -radius + radius * np.sin(lower_angle)], axis=1)
    dense = np.vstack([upper, lower[1:]])  # (y, z)

    arc_length = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
    targets = np.linspace(0.0, arc_length[-1], count)
    yz = np.stack([np.interp(targets, arc_length, dense[:, 0]), np.interp(targets, arc_length, dense[:, 1])], axis=1)

    anchor = sce.end_effector(READY)
    return np.column_stack([np.full(count, anchor[0]), yz + anchor[1:3]])


def start_at(point):
    """A configuration whose end-effector is at `point`: the ready pose projected onto a position-only region there."""
    region = vamp.panda.TaskSpaceConstraint(
        [1, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, *np.asarray(point).tolist()],
        [0.0, 0.0, 0.0, -3.2, -3.2, -3.2],
        [0.0, 0.0, 0.0, 3.2, 3.2, 3.2],
    )
    settings = vamp.ConstraintSettings()
    settings.tolerance = 1e-9
    settings.max_iterations = 100
    return np.asarray(vamp.panda.project(READY, [region], settings), dtype=np.float32)


def rotation_to_quaternion(matrix):
    """The unit quaternion (w, x, y, z) of a rotation matrix."""
    m = np.asarray(matrix, dtype=np.float64)
    w = np.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0
    x = np.copysign(x, m[2, 1] - m[1, 2])
    y = np.copysign(y, m[0, 2] - m[2, 0])
    z = np.copysign(z, m[1, 0] - m[0, 1])
    quaternion = np.array([w, x, y, z])
    return quaternion / np.linalg.norm(quaternion)


def quaternion_product(a, b):
    """The Hamilton product of quaternions (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def end_effector_quaternion(q):
    return rotation_to_quaternion(np.asarray(vamp.panda.eefk(np.asarray(q, dtype=np.float32)), dtype=np.float64)[:3, :3])


def rotation_angle(a, b):
    """The angle (rad) of the rotation between two unit quaternions."""
    return 2.0 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))


def keyframe_rotations(start, count, turn_degrees):
    """`count` keyframe rotations: the start's end-effector rotation, turned about the world z axis in equal steps up
    to `turn_degrees` at the last keyframe."""
    start_rotation = end_effector_quaternion(start)
    rotations = []
    for i in range(count):
        half = np.radians(turn_degrees) * i / (count - 1) / 2.0
        rotations.append(quaternion_product(np.array([np.cos(half), 0.0, 0.0, np.sin(half)]), start_rotation))

    return np.array(rotations)


def write_curve_dump(path, label, points, manifold, trail, position_at=None, extra=None):
    """The dump in the format of cricket's tests/view_spline.py, for a curve that passes through its points and
    evaluates with manifold.value(t) (Catmull-Rom, polyline): the points stand for the control points and the knot
    points. `position_at(t)` gives the curve's position when value(t) is more than that (a pose curve). `extra` is one
    dict per trail entry of more fields for the viewer's step (a pose curve's "eef_quaternion", "target_quaternion",
    "position_error" and "rotation_error")."""
    position_at = position_at or manifold.value
    ts = np.linspace(0.0, 1.0, 401)
    curve = np.array([position_at(t) for t in ts])

    iterations = []
    for q, t, squared_error in trail:
        tangent = (np.array(position_at(t + 1e-5)) - np.array(position_at(t - 1e-5))) / 2e-5
        iterations.append(
            {
                "t": float(t),
                "q": [float(v) for v in q],
                "arm": [sce.end_effector(q).tolist()],
                "curve_point": list(position_at(t)),
                "tangent": tangent.tolist(),
                "error_norm": float(np.sqrt(max(squared_error, 0.0))),
            }
        )

    for iteration, fields in zip(iterations, extra or []):
        iteration.update(fields)

    with open(path, "w") as f:
        json.dump(
            {
                "label": label,
                "nq": 7,
                "waypoints": np.asarray(points).tolist(),
                "control_points": np.asarray(points).tolist(),
                "knots": [],
                "parameters": [],
                "curve_t": ts.tolist(),
                "curve": curve.tolist(),
                "knot_points": np.asarray(points).tolist(),
                "iterations": iterations,
            },
            f,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, default=sce.DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--shape", choices=["trajectory", "s"], default="trajectory", help="the curve: the trajectory's points, or a synthetic S")
    parser.add_argument("--s-radius", type=float, default=0.10, help="radius of the S's arcs (m), for --shape s")
    parser.add_argument(
        "--s-spacing",
        type=float,
        default=None,
        help="instead of --s-radius: the distance (m) between consecutive points of the S, which sets its size "
        "(the default radius gives 0.044 m; 0.08 makes the gaps, and so the difference between straight lines and "
        "the smooth curve, much larger)",
    )
    parser.add_argument("--problem", default="bookshelf_small", help="MBM problem set the trajectory was planned for")
    parser.add_argument("--index", type=int, default=0, help="problem index within the set")
    parser.add_argument("--no-env", action="store_true", help="plan in an empty environment")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension (rad)")
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--qt", action="store_true", help="plan in (q, t): task_space_rrtc_spline")
    parser.add_argument("--curve", choices=["bspline", "catmull-rom", "linear", "linear-se3"], default="bspline", help="the curve through the points: linear-se3 is the straight-line interpolation of poses in R^3 x SO(3) (the rotation along a slerp)")
    parser.add_argument("--alpha", type=float, default=0.5, help="Catmull-Rom parametrization: 0 uniform, 0.5 centripetal, 1 chordal")
    parser.add_argument("--turn", type=float, default=60.0, help="with linear-se3: the rotation about the world z axis (degrees) from the first keyframe to the last")
    parser.add_argument("--rot-weight", type=float, default=0.1, help="with linear-se3: the rotation weight (m/rad) of the error and of the parameter t's metric")
    parser.add_argument("--slide", action="store_true", help="with linear or linear-se3: the t of a state only says which segment it is on, and the end-effector slides along that segment while a projection runs (the state takes the t the projection leaves it at), instead of being held at the point P(t)")
    parser.add_argument(
        "--method",
        choices=["BoundedLS", "InnerLM", "OuterLM", "GradDesc"],
        default="BoundedLS",
        help="the projection solver of the manifold and of the goal region (BoundedLS keeps the joint limits in the step)",
    )
    parser.add_argument("--parameter-step", type=float, default=0.05, help="with --qt: t advance of each goal-directed step")
    parser.add_argument("--no-project-samples", action="store_true", help="with --qt: do not project random samples onto the curve")
    parser.add_argument("--prefix", default="spline_task_space_rrtc_py", help="prefix of the output JSON file")
    parser.add_argument("--view", action="store_true", help="start the viewer on the path when done")
    args = parser.parse_args()
    host_curve = args.curve != "bspline"  # evaluated on the host, through value(t)
    if args.slide and args.curve not in ("linear", "linear-se3"):
        sys.exit("--slide is for --curve linear and --curve linear-se3")

    se3 = args.curve == "linear-se3"  # the curve has rotations: value(t) is (position, quaternion)
    args.qt = args.qt or host_curve  # those constraints are only for planning in (q, t)

    if args.shape == "s":
        if args.s_spacing is not None:
            args.s_radius = args.s_spacing * (N - 1) / S_LENGTH_PER_RADIUS

        points = s_curve_points(args.s_radius)
        farthest = float(np.linalg.norm(points - SHOULDER, axis=1).max())
        if farthest > 0.95 * REACH:
            print(f"warning: the S reaches {farthest:.2f} m from the shoulder (the arm's reach is about {REACH} m)")

        start = start_at(points[0])
        print(
            f"spline: an S of radius {args.s_radius} m, {N} waypoints at equal arc length, "
            f"{np.linalg.norm(np.diff(points, axis=0), axis=1).sum():.2f} m long; "
            "start: the ready pose projected onto its first point"
        )
    else:
        if not args.trajectory.exists():
            sys.exit(f"no trajectory file at {args.trajectory}: write one with evaluate_mbm.py --save_eef")

        entry = next(iter(json.load(open(args.trajectory))["trajectories"].values()))[0]
        points = np.array(entry["eef_positions"][:N], dtype=np.float64)
        start = np.array(entry["start"], dtype=np.float32)
        print(f"spline: the first {N} end-effector positions of {args.trajectory.name}; start: its start configuration")

    if args.curve == "catmull-rom":
        manifold = vamp.panda.CatmullRomConstraint(points.tolist(), args.alpha)
    elif args.curve == "linear":
        manifold = vamp.panda.PolylineConstraint(points.tolist(), args.slide)
    elif args.curve == "linear-se3":
        rotations = keyframe_rotations(start, len(points), args.turn)
        manifold = vamp.panda.PosePolylineConstraint(points.tolist(), rotations.tolist(), args.rot_weight, args.slide)
    else:
        control_points, knots, parameters = sce.fit_cubic_bspline(points)
        manifold = vamp.panda.SplineConstraint(
            control_points.tolist(), knots.tolist(), points.tolist(), parameters.tolist()
        )

    # the goal is the end of the curve, P(1): the planner holds the manifold's own constraint at t = 1 as the goal
    # region, so there is no separate goal constraint
    goal = points[-1]

    # the environment
    if args.no_env or args.shape == "s":
        env = vamp.Environment()
    else:
        problems, _ = mbm.load_problems("panda", "problems.pkl", [args.problem])
        data = problems[args.problem][args.index]
        env = vamp.problem_dict_to_vamp(data)
        if not np.allclose(np.asarray(data["start"], dtype=np.float32), start, atol=1e-4):
            print("warning: the trajectory's start differs from the problem's start: is it the same problem?")

    # the start has to be on the manifold
    t, d2 = manifold.closest_t(start.tolist())
    print(f"start: closest t {t:.4f}, squared distance to the curve {d2:.2e}; goal P(1) = {np.round(goal, 3).tolist()}")
    sce.check(d2 < 1e-8, f"the start is on the curve (squared distance {d2:.2e})")

    plan_settings = vamp.TaskSpaceRRTCSettings()
    plan_settings.range = args.range
    plan_settings.max_greedy_steps = 20
    plan_settings.max_iterations = args.max_iterations
    plan_settings.max_samples = 20000

    constraint_settings = vamp.ConstraintSettings()  # for the manifold and the goal region (the manifold's constraint at t = 1)
    constraint_settings.method = getattr(vamp.ProjMethod, args.method)
    constraint_settings.tolerance = 1e-7
    constraint_settings.max_iterations = 50

    sampler = vamp.panda.halton()
    if args.qt:
        plan_settings.project_samples = not args.no_project_samples
        plan_settings.parameter_step = args.parameter_step
        result = vamp.panda.task_space_rrtc_spline(
            start, env, plan_settings, sampler, [manifold], constraint_settings, start_t=t
        )
    else:
        result = vamp.panda.task_space_rrtc(
            start, [], env, plan_settings, sampler, constraint_settings, manifold_constraints=[manifold]
        )

    name = "task_space_rrtc_spline" if args.qt else "task_space_rrtc"
    print(f"{name}: solved {result.solved}, {result.iterations} iterations, {result.nanoseconds / 1e3:.2f} us")
    sce.check(result.solved, "the planner solved")

    if result.solved:
        path = np.asarray(result.path) if args.qt else result.path.numpy()
        trail = []
        worst = 0.0
        worst_angle = 0.0
        for i, q in enumerate(path):
            t, d2 = manifold.closest_t(np.asarray(q, dtype=np.float32).tolist())
            if args.qt:
                t = result.t[i]  # the state's own t, which the planner held fixed
            if se3:
                # the position and the rotation of the state against the curve's pose at its own t
                position, rotation = manifold.value(t)
                d2 = float(np.sum((sce.end_effector(q) - np.array(position)) ** 2))
                worst_angle = max(worst_angle, rotation_angle(end_effector_quaternion(q), np.array(rotation)))
            worst = max(worst, np.sqrt(d2))
            trail.append((q, t, d2))

        final_distance = float(np.linalg.norm(sce.end_effector(path[-1]) - goal))
        print(
            f"path: {len(path)} states, t {trail[0][1]:.4f} -> {trail[-1][1]:.4f}, worst distance to the curve "
            f"{worst:.2e} m, final distance to P(1) {final_distance:.2e} m"
        )

        sce.check(worst < 1e-3, f"every path state is on the curve (worst distance {worst:.2e} m)")
        sce.check(final_distance < 2e-3, f"the path ends at P(1) (distance {final_distance:.2e} m)")
        if se3:
            final_angle = rotation_angle(end_effector_quaternion(path[-1]), np.array(manifold.value(1.0)[1]))
            print(f"rotation: worst angle to the curve {worst_angle:.2e} rad, final angle to P(1) {final_angle:.2e} rad")
            sce.check(worst_angle < 1e-2, f"every path state has the curve's rotation (worst angle {worst_angle:.2e} rad)")
            sce.check(final_angle < 2e-2, f"the path ends at P(1)'s rotation (angle {final_angle:.2e} rad)")
        sce.check(
            all(vamp.panda.validate(np.asarray(q, dtype=np.float32), env) for q in path),
            "every path state is collision-free",
        )

        output = f"{args.prefix}_path.json"
        if host_curve:
            position_at = (lambda t: manifold.value(t)[0]) if se3 else None
            write_curve_dump(output, f"TaskSpaceRRTC (q, t) {args.curve} path", points, manifold, trail, position_at)
        else:
            label = "TaskSpaceRRTC (q, t) path" if args.qt else "TaskSpaceRRTC path"
            sce.write_dump(output, label, points, control_points, knots, parameters, trail)
        print(f"written to {output}")

    if sce.failures:
        print(f"\nspline_task_space_rrtc_example: {sce.failures} check(s) failed", file=sys.stderr)
        sys.exit(1)

    print("\nspline_task_space_rrtc_example: all checks passed")

    if args.view and result.solved:
        subprocess.run([sys.executable, str(sce.VIEWER), f"{args.prefix}_path.json"], check=False)


if __name__ == "__main__":
    main()
