"""Plans along the spline of an MBM solution, in the same environment.

For an MBM problem and index (--problem, --index; --index all runs every problem of the set):
  1. a reference trajectory is planned with RRTC (and simplified, and interpolated to the robot's resolution);
  2. forward kinematics gives the end-effector's pose along it; the position (orientation is not constrained yet) is
     resampled to 20 waypoints at equal arc length and a cubic B-spline is fitted through them;
  3. TaskSpaceRRTC plans from the problem's start with the spline as the manifold (every state of the tree stays on
     the curve) and the end of the curve, the reference trajectory's final end-effector position, as the goal region.
     The environment is the problem's own: the planner checks every state against it.

Checked, per problem: the reference planned; the planner solves; every state of its path is on the curve and
collision-free; the path ends at the end of the curve. The spline is a smoothing of the reference path, so it need not
be collision-free itself between waypoints: the printed deviation shows how far it is from the reference's
end-effector path.

One dump is written per problem, <prefix>_<problem>_<index>.json, in the format of cricket's tests/view_spline.py: the
curve, the Panda along the planned path, the environment, the reference trajectory's end-effector path in grey, and
stats (planning time in milliseconds, whether it solved, ...). A problem whose planner failed still gets a dump (the
start, the environment and the reference), marked as failed. With --index all, the viewer has a dropdown of the problems:

    python3 scripts/spline_mbm_planner_example.py --problem bookshelf_small --index all --view
    python3 cricket/tests/view_spline.py "spline_mbm_planner_bookshelf_small_*.json"

Problems whose reference planner fails or whose end-effector barely moves are skipped (listed in the summary).
The planner has no time limit, only an iteration limit (--max-iterations; a smaller default with --index all): a
problem it cannot solve costs the whole budget, so a run over a whole set can take a while.

Needs a Panda built with a header generated with "spline_constraint": true (cricket).
"""

import argparse
import statistics
import subprocess
import sys

import numpy as np
import vamp
from vamp import mbm

import spline_constraint_example as sce

N = sce.N


def resample_by_arc_length(points, count):
    """`count` points at equal arc length along a polyline (the first and last points are kept exactly)."""
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate([[True], steps > 1e-9])  # drop repeated points: arc length must increase
    points = points[keep]
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    targets = np.linspace(0.0, arc[-1], count)
    return np.stack([np.interp(targets, arc, points[:, axis]) for axis in range(3)], axis=1)


def obstacles_of(data):
    """The problem's obstacles, as the viewer draws them."""
    return {
        "boxes": [
            {
                "position": box["position"],
                "half_extents": box["half_extents"],
                "quaternion_xyzw": box["orientation_quat_xyzw"],
            }
            for box in data["box"]
        ],
        "cylinders": [
            {
                "position": cylinder["position"],
                "radius": cylinder["radius"],
                "length": cylinder["length"],
                "quaternion_xyzw": cylinder["orientation_quat_xyzw"],
            }
            for cylinder in data["cylinder"]
        ],
        "spheres": [{"position": sphere["position"], "radius": sphere["radius"]} for sphere in data["sphere"]],
    }


def solve_problem(problems, name, index, args, vamp_module, planner, planner_settings, simplify_settings, verbose):
    """Plan along the spline of problem `index`. Returns a record of what happened (see the summary table)."""
    record = {"index": index, "status": "", "messages": [], "file": None}

    def log(message):
        if verbose:
            print(message)

    def fail(message):
        record["messages"].append(message)
        print(f"  FAIL ({name} #{index}): {message}", file=sys.stderr)

    data = problems[name][index]
    if not data["valid"]:
        record["status"] = "invalid problem"
        return record

    env = vamp.problem_dict_to_vamp(data)
    start = np.asarray(data["start"], dtype=np.float32)
    goals = [np.asarray(goal, dtype=np.float32) for goal in data["goals"]]
    log(f"{name} #{index}: {len(data['box'])} boxes, {len(data['cylinder'])} cylinders, {len(data['sphere'])} spheres")

    # 1. the reference trajectory
    sampler = vamp_module.halton()
    result = planner(start, goals, env, planner_settings, sampler)
    if not result.solved:
        record["status"] = "reference failed"
        return record

    record["reference_ms"] = result.nanoseconds / 1e6
    simplified = vamp_module.simplify(result.path, env, simplify_settings, sampler)
    simplified.path.interpolate_to_resolution(vamp_module.resolution()//8)
    reference = simplified.path.numpy().copy()
    log(f"reference trajectory: {len(reference)} states, joint-space length {simplified.path.cost():.3f} rad")

    # 2. forward kinematics: the end-effector's pose along it; the spline is fitted through its positions
    poses = np.array([np.asarray(vamp_module.eefk(np.asarray(q, dtype=np.float32)), dtype=np.float64) for q in reference])
    reference_positions = poses[:, :3, 3]
    path_length = np.linalg.norm(np.diff(reference_positions, axis=0), axis=1).sum()
    if path_length < 0.05:
        record["status"] = "end-effector barely moves"
        return record

    points = resample_by_arc_length(reference_positions, N)
    control_points, knots, parameters = sce.fit_cubic_bspline(points)
    if max(np.linalg.norm(sce.evaluate(control_points, knots, parameters[i]) - points[i]) for i in range(N)) >= 1e-9:
        fail("the fitted spline does not pass through its waypoints")

    # how faithful the spline is to the reference's end-effector path
    curve = np.array([sce.evaluate(control_points, knots, t) for t in np.linspace(0.0, 1.0, 2001)])
    deviation = np.linalg.norm(reference_positions[:, None, :] - curve[None, :, :], axis=2).min(axis=1).max()
    record["deviation_mm"] = float(deviation * 1000)
    log(
        f"spline: {N} waypoints at equal arc length along the reference's end-effector path "
        f"({path_length:.2f} m, {path_length / (N - 1) * 1000:.0f} mm apart); the reference's end-effector path is at "
        f"most {deviation * 1000:.1f} mm from the curve"
    )

    stats = {"problem": name, "index": index, "reference_ms": record["reference_ms"], "deviation_mm": record["deviation_mm"]}
    extra = {"obstacles": obstacles_of(data), "reference_path": reference_positions.tolist()}
    label = f"{name} #{index}"
    file = f"{args.prefix}_{name}_{index:03d}.json"

    def write(trail, status):
        stats["status"] = status
        record["status"] = status
        sce.write_dump(
            file,
            label,
            points,
            control_points,
            knots,
            parameters,
            trail,
            extra={**extra, "stats": stats},
        )
        record["file"] = file

    try:
        manifold = vamp.panda.SplineConstraint(
            control_points.tolist(), knots.tolist(), points.tolist(), parameters.tolist()
        )

        # the goal: the end of the curve, position only (the rotation bounds are wider than the widest rotation error)
        goal_position = points[-1]
        goal_region = vamp.panda.TaskSpaceConstraint(
            [1, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, *goal_position.tolist()],
            [0.0, 0.0, 0.0, -3.2, -3.2, -3.2],
            [0.0, 0.0, 0.0, 3.2, 3.2, 3.2],
        )

        # the start is the problem's start, whose end-effector is the curve's first waypoint
        t0, d2 = manifold.closest_t(start.tolist())
        log(f"start: closest t {t0:.4f}, squared distance to the curve {d2:.2e}; goal P(1) = {np.round(goal_position, 3).tolist()}")
        if d2 >= 1e-8:
            fail(f"the start is not on the curve (squared distance {d2:.2e})")

        # 3. plan along the spline, in the same environment
        plan_settings = vamp.TaskSpaceRRTCSettings()
        plan_settings.range = args.range
        plan_settings.max_greedy_steps = 20
        plan_settings.max_iterations = args.max_iterations
        plan_settings.max_samples = 20000

        constraint_settings = vamp.ConstraintSettings()  # used for the manifold and for the goal region
        constraint_settings.method = vamp.ProjMethod.InnerLM
        constraint_settings.tolerance = 1e-7
        constraint_settings.max_iterations = 50

        sampler = vamp_module.halton()
        planned = vamp_module.task_space_rrtc(
            start, [goal_region], env, plan_settings, sampler, constraint_settings, manifold_constraints=[manifold]
        )
        stats["planning_ms"] = planned.nanoseconds / 1e6
        stats["iterations"] = int(planned.iterations)
        record["planning_ms"] = stats["planning_ms"]
        record["iterations"] = stats["iterations"]
        log(f"task_space_rrtc along the spline: solved {planned.solved}, {planned.iterations} iterations, {stats['planning_ms']:.1f} ms")

        if not planned.solved:
            # a dump with the start, the environment and the reference: to see why
            write([(start, t0, d2)], "planner failed")
            return record

        path = planned.path.numpy().copy()
        trail = []
        worst = 0.0
        for q in path:
            t, d2 = manifold.closest_t(q.tolist())
            worst = max(worst, np.sqrt(d2))
            trail.append((q, t, d2))

        final_distance = float(np.linalg.norm(sce.end_effector(path[-1]) - goal_position))
        stats["path"] = {"states": len(path), "joint_length": float(planned.path.cost())}
        stats["worst_mm"] = float(worst * 1000)
        record["path_states"] = len(path)
        record["joint_length"] = float(planned.path.cost())
        record["worst_mm"] = stats["worst_mm"]
        log(
            f"path: {len(path)} states, joint-space length {planned.path.cost():.3f} rad, "
            f"t {trail[0][1]:.4f} -> {trail[-1][1]:.4f}, worst distance to the curve {worst:.2e} m, "
            f"final distance to P(1) {final_distance:.2e} m"
        )

        if worst >= 1e-3:
            fail(f"a path state is off the curve (worst distance {worst:.2e} m)")
        if final_distance >= 2e-3:
            fail(f"the path does not end at P(1) (distance {final_distance:.2e} m)")
        if not all(vamp_module.validate(np.asarray(q, dtype=np.float32), env) for q in path):
            fail("a path state is in collision in the problem's environment")

        write(trail, "solved" if not record["messages"] else "solved, checks failed")
    except Exception as error:  # a binding raising (the start off the curve, ...) must not stop a run over a set
        fail(f"{type(error).__name__}: {error}")
        if record["file"] is None:
            record["status"] = "error"

    return record


def print_summary(name, records):
    print(f"\n{name}: {len(records)} problems")
    print(f"  {'#':>3}  {'status':<26} {'planning ms':>11} {'iters':>7} {'ref ms':>8} {'dev mm':>7} {'worst mm':>8}")
    for r in records:
        def cell(key, fmt):
            return format(r[key], fmt) if key in r else "-"

        print(
            f"  {r['index']:>3}  {r['status']:<26} {cell('planning_ms', '11.1f'):>11} {cell('iterations', '7d'):>7} "
            f"{cell('reference_ms', '8.1f'):>8} {cell('deviation_mm', '7.1f'):>7} {cell('worst_mm', '8.2f'):>8}"
        )

    solved = [r for r in records if r["status"] == "solved"]
    print(f"\nsolved {len(solved)} of {len(records)}")
    if solved:
        times = [r["planning_ms"] for r in solved]
        print(f"planning time of the solved ones (ms): median {statistics.median(times):.1f}, mean {statistics.mean(times):.1f}, max {max(times):.1f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--problem", default="bookshelf_small", help="MBM problem set (bookshelf_small, cage, ...)")
    parser.add_argument("--index", default="0", help="problem index within the set, or `all`")
    parser.add_argument("--limit", type=int, default=0, help="with --index all: only the first this many problems")
    parser.add_argument("--dataset", default="problems.pkl", help="pickled MBM dataset of the Panda")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension of the spline planner (rad)")
    parser.add_argument(
        "--max-iterations", type=int, default=None, help="iterations of the spline planner (default 20000; 5000 with --index all)"
    )
    parser.add_argument("--prefix", default="spline_mbm_planner", help="prefix of the output JSON files")
    parser.add_argument("--view", action="store_true", help="start the viewer on the dumps when done")
    args = parser.parse_args()

    problems, _ = mbm.load_problems("panda", args.dataset, [args.problem])
    count = len(problems[args.problem])

    run_all = args.index == "all"
    if run_all:
        indices = list(range(count))
        if args.limit:
            indices = indices[: args.limit]
    else:
        try:
            index = int(args.index)
        except ValueError:
            sys.exit(f"--index is an integer or `all`, not `{args.index}`")
        if not 0 <= index < count:
            sys.exit(f"index {index} is out of range: {args.problem} has {count} problems")
        indices = [index]

    if args.max_iterations is None:
        args.max_iterations = 5000 if run_all else 20000

    vamp_module, planner, planner_settings, simplify_settings = vamp.configure_robot_and_planner_with_kwargs(
        "panda", "rrtc"
    )

    records = []
    for position, index in enumerate(indices):
        if run_all:
            print(f"[{position + 1}/{len(indices)}] {args.problem} #{index} ...", flush=True)

        record = solve_problem(
            problems, args.problem, index, args, vamp_module, planner, planner_settings, simplify_settings,
            verbose=not run_all,
        )
        records.append(record)
        if run_all:
            planning = f", {record['planning_ms']:.1f} ms" if "planning_ms" in record else ""
            print(f"    {record['status']}{planning}")

    files = [r["file"] for r in records if r["file"]]
    if run_all:
        print_summary(args.problem, records)

    if files:
        print(f"\n{len(files)} dump(s) written: view with\n    python3 cricket/tests/view_spline.py \"{args.prefix}_{args.problem}_*.json\"")

    if not run_all:
        if records[0]["status"] != "solved":
            print(f"\nspline_mbm_planner_example: {records[0]['status']}", file=sys.stderr)
            sys.exit(1)

        print("\nspline_mbm_planner_example: all checks passed")

    if args.view and files:
        subprocess.run([sys.executable, str(sce.VIEWER), *files], check=False)


if __name__ == "__main__":
    main()
