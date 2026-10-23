"""Plans along a spline from Python: vamp.panda.task_space_rrtc with a SplineConstraint as the manifold.

A cubic B-spline is fitted through the first 20 end-effector positions of the first trajectory in a file written by
evaluate_mbm.py --save_eef (default /tmp/eef_bookshelf_small.json). The tree starts at that trajectory's start
configuration, which is on the curve, and every state of it stays on the curve: extensions toward random samples and
the goal-directed steps are projected onto it (manifold_constraints). The goal is the end of the curve, a
position-only task-space region at P(1); no goal configuration is given.

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

Needs a Panda built with a header generated with "spline_constraint": true (cricket).
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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, default=sce.DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--shape", choices=["trajectory", "s"], default="trajectory", help="the curve: the trajectory's points, or a synthetic S")
    parser.add_argument("--s-radius", type=float, default=0.10, help="radius of the S's arcs (m), for --shape s")
    parser.add_argument("--problem", default="bookshelf_small", help="MBM problem set the trajectory was planned for")
    parser.add_argument("--index", type=int, default=0, help="problem index within the set")
    parser.add_argument("--no-env", action="store_true", help="plan in an empty environment")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension (rad)")
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--qt", action="store_true", help="plan in (q, t): task_space_rrtc_spline")
    parser.add_argument("--parameter-step", type=float, default=0.05, help="with --qt: t advance of each goal-directed step")
    parser.add_argument("--no-project-samples", action="store_true", help="with --qt: do not project random samples onto the curve")
    parser.add_argument("--prefix", default="spline_task_space_rrtc_py", help="prefix of the output JSON file")
    parser.add_argument("--view", action="store_true", help="start the viewer on the path when done")
    args = parser.parse_args()

    if args.shape == "s":
        points = s_curve_points(args.s_radius)
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

    control_points, knots, parameters = sce.fit_cubic_bspline(points)
    manifold = vamp.panda.SplineConstraint(
        control_points.tolist(), knots.tolist(), points.tolist(), parameters.tolist()
    )

    # the goal: the end of the curve, position only (the rotation bounds are wider than the widest rotation error)
    goal = points[-1]
    goal_region = vamp.panda.TaskSpaceConstraint(
        [1, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, *goal.tolist()],
        [0.0, 0.0, 0.0, -3.2, -3.2, -3.2],
        [0.0, 0.0, 0.0, 3.2, 3.2, 3.2],
    )

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

    constraint_settings = vamp.ConstraintSettings()  # used for the manifold and for the goal region
    constraint_settings.method = vamp.ProjMethod.InnerLM
    constraint_settings.tolerance = 1e-7
    constraint_settings.max_iterations = 50

    sampler = vamp.panda.halton()
    if args.qt:
        plan_settings.project_samples = not args.no_project_samples
        plan_settings.parameter_step = args.parameter_step
        result = vamp.panda.task_space_rrtc_spline(
            start, [goal_region], env, plan_settings, sampler, [manifold], constraint_settings, start_t=t
        )
    else:
        result = vamp.panda.task_space_rrtc(
            start, [goal_region], env, plan_settings, sampler, constraint_settings, manifold_constraints=[manifold]
        )

    name = "task_space_rrtc_spline" if args.qt else "task_space_rrtc"
    print(f"{name}: solved {result.solved}, {result.iterations} iterations, {result.nanoseconds / 1e3:.2f} us")
    sce.check(result.solved, "the planner solved")

    if result.solved:
        path = np.asarray(result.path) if args.qt else result.path.numpy()
        trail = []
        worst = 0.0
        for i, q in enumerate(path):
            t, d2 = manifold.closest_t(np.asarray(q, dtype=np.float32).tolist())
            if args.qt:
                t = result.t[i]  # the state's own t, which the planner held fixed
            worst = max(worst, np.sqrt(d2))
            trail.append((q, t, d2))

        final_distance = float(np.linalg.norm(sce.end_effector(path[-1]) - goal))
        print(
            f"path: {len(path)} states, t {trail[0][1]:.4f} -> {trail[-1][1]:.4f}, worst distance to the curve "
            f"{worst:.2e} m, final distance to P(1) {final_distance:.2e} m"
        )

        sce.check(worst < 1e-3, f"every path state is on the curve (worst distance {worst:.2e} m)")
        sce.check(final_distance < 2e-3, f"the path ends at P(1) (distance {final_distance:.2e} m)")
        sce.check(
            all(vamp.panda.validate(np.asarray(q, dtype=np.float32), env) for q in path),
            "every path state is collision-free",
        )

        output = f"{args.prefix}_path.json"
        sce.write_dump(output, "TaskSpaceRRTC (q, t) path" if args.qt else "TaskSpaceRRTC path", points, control_points, knots, parameters, trail)
        print(f"written to {output}")

    if sce.failures:
        print(f"\nspline_task_space_rrtc_example: {sce.failures} check(s) failed", file=sys.stderr)
        sys.exit(1)

    print("\nspline_task_space_rrtc_example: all checks passed")

    if args.view and result.solved:
        subprocess.run([sys.executable, str(sce.VIEWER), f"{args.prefix}_path.json"], check=False)


if __name__ == "__main__":
    main()
