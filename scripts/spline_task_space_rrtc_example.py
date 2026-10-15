"""Plans along a spline from Python: vamp.panda.task_space_rrtc with a SplineConstraint as the manifold.

A cubic B-spline is fitted through the first 20 end-effector positions of the first trajectory in a file written by
evaluate_mbm.py --save_eef (default /tmp/eef_bookshelf_small.json). The tree starts at that trajectory's start
configuration, which is on the curve, and every state of it stays on the curve: extensions toward random samples and
the goal-directed steps are projected onto it (manifold_constraints). The goal is the end of the curve, a
position-only task-space region at P(1); no goal configuration is given.

The environment is the MBM problem the trajectory was planned for (default bookshelf_small, index 0), or empty with
--no-env. Checked: the planner solves; every state of the path is on the curve and collision-free; the path ends
at P(1). The path is written to <prefix>_path.json, in the format of cricket's tests/view_spline.py
(the curve, waypoints and control points, the Panda at each state, the curve point each is matched to):

    python3 scripts/spline_task_space_rrtc_example.py --view
    python3 cricket/tests/view_spline.py spline_task_space_rrtc_py_path.json

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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, default=sce.DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--problem", default="bookshelf_small", help="MBM problem set the trajectory was planned for")
    parser.add_argument("--index", type=int, default=0, help="problem index within the set")
    parser.add_argument("--no-env", action="store_true", help="plan in an empty environment")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension (rad)")
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--prefix", default="spline_task_space_rrtc_py", help="prefix of the output JSON file")
    parser.add_argument("--view", action="store_true", help="start the viewer on the path when done")
    args = parser.parse_args()

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
    if args.no_env:
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
    result = vamp.panda.task_space_rrtc(
        start, [goal_region], env, plan_settings, sampler, constraint_settings, manifold_constraints=[manifold]
    )
    print(f"task_space_rrtc: solved {result.solved}, {result.iterations} iterations")
    sce.check(result.solved, "the planner solved")

    if result.solved:
        path = result.path.numpy()
        trail = []
        worst = 0.0
        for q in path:
            t, d2 = manifold.closest_t(q.tolist())
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
        sce.write_dump(output, "TaskSpaceRRTC path", points, control_points, knots, parameters, trail)
        print(f"written to {output}")

    if sce.failures:
        print(f"\nspline_task_space_rrtc_example: {sce.failures} check(s) failed", file=sys.stderr)
        sys.exit(1)

    print("\nspline_task_space_rrtc_example: all checks passed")

    if args.view and result.solved:
        subprocess.run([sys.executable, str(sce.VIEWER), f"{args.prefix}_path.json"], check=False)


if __name__ == "__main__":
    main()
