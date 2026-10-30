"""One projection of a random sample onto a polyline, step by step: q, qt or qs (--mode), for viser.

The curve (--curve) is the straight-line interpolation of points (the S, or the first points of a trajectory), with t
the chord-length parameter: `linear` in R^3 (position only), or `linear-se3` in R^3 x SO(3) (the rotation along a slerp
between keyframe rotations, see spline_task_space_rrtc_example.py; only qt and qs, the pose curve has no q projection). A random sample is a configuration q and a curve parameter t, as the planner draws them (the
same --seed gives the same sample, so the three modes can be run one after the other on it). --mode picks the
projection:

  q    no t is given: the projection seeds t at the closest point of the whole polyline to the end-effector and then
       solves for the joints and t together (t bounded to [0, 1])           PolylineConstraint(points, whole_curve=True)
  qt   t is given: the end-effector must reach the point P(t), the joints
       alone are solved for                                                 PolylineConstraint(points)
  qs   t is given but only picks the segment: the end-effector may slide
       along that segment, t is solved for next to the joints, bounded to the
       segment, and the state takes the t where it ends                     PolylineConstraint(points, slide=True)

(for linear-se3 the same two with PosePolylineConstraint(points, rotations, rot_weight, slide).)

Each iteration is printed (t, the segment and the fraction s along it, the error, the end-effector; for linear-se3 also
the position and rotation parts of the error) and written to
<prefix>_<mode>.json in the format of cricket's tests/view_spline.py, which shows the arm at that iteration, the point
P(t) it is being matched to on the curve, and the segment t is on, highlighted:

    python3 scripts/projection_example.py --mode qs --shape s --s-spacing 0.08 --view
    python3 cricket/tests/view_spline.py projection_qs.json

Needs a Panda built with a header generated with "spline_constraint" and "position_target_constraint"; linear-se3 also
"constraints" and "pose_slide_constraint".
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import vamp

import spline_constraint_example as sce
import spline_task_space_rrtc_example as ex
from pose_projection_example import pose_fields


def chord_parameters(points):
    """The chord-length parameter of each point of a polyline, 0 to 1 (the t the constraint uses)."""
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    tau = np.concatenate([[0.0], np.cumsum(lengths)])
    return tau / tau[-1]


def pose_parameters(points, rotations, weight):
    """The parameter of each keyframe of a pose polyline, 0 to 1: segment lengths sqrt(|dp|^2 + w^2 theta^2), theta the
    angle between the keyframe rotations (the t the pose constraint uses)."""
    lengths = []
    for i in range(len(points) - 1):
        theta = ex.rotation_angle(rotations[i], rotations[i + 1])
        lengths.append(np.sqrt(np.sum((points[i + 1] - points[i]) ** 2) + (weight * theta) ** 2))

    tau = np.concatenate([[0.0], np.cumsum(lengths)])
    return tau / tau[-1]


def segment_fields(points, tau, trail, fixed_segment=False):
    """The viewer's highlight of the segment t is on, and the fraction s along it, for each iterate. With
    `fixed_segment` (qs) every iterate is on the segment of the first one: t is bounded to it, so a t at its end is
    s = 1 of that segment, not s = 0 of the next."""
    fields = []
    segment = None
    for _, t, _ in trail:
        if segment is None or not fixed_segment:
            segment = int(np.clip(np.searchsorted(tau, t, side="right") - 1, 0, len(points) - 2))

        s = float((t - tau[segment]) / (tau[segment + 1] - tau[segment]))
        fields.append(
            {
                "segment": [points[segment].tolist(), points[segment + 1].tolist()],
                "segment_index": segment,
                "s": s,
            }
        )

    return fields


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["q", "qt", "qs"], required=True, help="the projection to run")
    parser.add_argument("--curve", choices=["linear", "linear-se3"], default="linear", help="the curve: positions only, or poses in R^3 x SO(3) (qt and qs)")
    parser.add_argument("--turn", type=float, default=60.0, help="with linear-se3: the rotation about the world z axis (degrees) from the first keyframe to the last")
    parser.add_argument("--rot-weight", type=float, default=0.1, help="with linear-se3: the rotation weight (m/rad)")
    parser.add_argument("--trajectory", type=Path, default=sce.DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--shape", choices=["trajectory", "s"], default="s", help="the curve: the trajectory's points, or a synthetic S")
    parser.add_argument("--s-radius", type=float, default=0.10)
    parser.add_argument("--s-spacing", type=float, default=None, help="instead of --s-radius: the distance (m) between consecutive points of the S")
    parser.add_argument("--method", choices=["BoundedLS", "InnerLM", "OuterLM", "GradDesc"], default="InnerLM")
    parser.add_argument("--sample", choices=["random", "near-start"], default="random", help="q uniform in the joint limits, or the start configuration plus noise")
    parser.add_argument("--perturb", type=float, default=0.15, help="with near-start: the noise (rad, per joint)")
    parser.add_argument("--t-offset", type=float, default=None, help="instead of a random t: the closest t of the sample's q plus this (a planner's t is near its q's)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--tolerance", type=float, default=1e-8, help="squared error at which the projection has converged")
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--alpha", type=float, default=1.0, help="step size")
    parser.add_argument("--repeat", type=int, default=200, help="times the projection is repeated to time it (0 for no timing)")
    parser.add_argument("--prefix", default="projection")
    parser.add_argument("--view", action="store_true", help="start the viewer on the dump when done")
    args = parser.parse_args()

    if args.shape == "s":
        if args.s_spacing is not None:
            args.s_radius = args.s_spacing * (ex.N - 1) / ex.S_LENGTH_PER_RADIUS

        points = ex.s_curve_points(args.s_radius)
        start = ex.start_at(points[0])
    else:
        if not args.trajectory.exists():
            sys.exit(f"no trajectory file at {args.trajectory}: write one with evaluate_mbm.py --save_eef")

        entry = next(iter(json.load(open(args.trajectory))["trajectories"].values()))[0]
        points = np.array(entry["eef_positions"][: ex.N], dtype=np.float64)
        start = np.array(entry["start"], dtype=np.float32)

    se3 = args.curve == "linear-se3"
    if se3 and args.mode == "q":
        sys.exit("--mode q needs a projection over the whole curve, which the pose curve does not have: use qt or qs")

    # the chord-length parameter of the pose curve has the rotation in its metric: its segments' t-ranges come from it
    tau = chord_parameters(points)
    slide = args.mode == "qs"
    if se3:
        rotations = ex.keyframe_rotations(start, len(points), args.turn)
        manifold = vamp.panda.PosePolylineConstraint(points.tolist(), rotations.tolist(), args.rot_weight, slide)
        tau = pose_parameters(points, rotations, args.rot_weight)
    elif args.mode == "q":
        manifold = vamp.panda.PolylineConstraint(points.tolist(), False, True)
    else:
        manifold = vamp.panda.PolylineConstraint(points.tolist(), slide, False)

    method = getattr(vamp.ProjMethod, args.method)

    # the random sample: a configuration and a curve parameter
    rng = np.random.default_rng(args.seed)
    if args.sample == "random":
        lower = np.asarray(vamp.panda.lower_bounds(), dtype=np.float64)
        upper = np.asarray(vamp.panda.upper_bounds(), dtype=np.float64)
        q = rng.uniform(lower, upper).astype(np.float32)
    else:
        q = (start + args.perturb * rng.standard_normal(7)).astype(np.float32)

    t = float(rng.uniform(0.0, 1.0))
    closest_t, distance2 = manifold.closest_t(q.tolist())
    if args.t_offset is not None:
        t = float(min(max(closest_t + args.t_offset, 0.0), 1.0))

    print(
        f"sample: t = {t:.4f}, end-effector at {np.round(sce.end_effector(q), 3).tolist()}, closest point of the curve at "
        f"t = {closest_t:.4f} ({np.sqrt(distance2):.3f} {'m, with the rotation weighted' if se3 else 'm'} away)"
    )

    trail = manifold.projection_trail(q.tolist(), t, method, args.tolerance, args.max_iterations, args.alpha)
    fields = segment_fields(points, tau, trail, fixed_segment=args.mode == "qs")

    shown = "t is not given: seeded at the closest point" if args.mode == "q" else f"t given: {t:.4f}"
    print(f"\n{args.mode}, {args.method} ({shown})")
    pose = pose_fields(manifold, trail) if se3 else None
    print("  iter        t  segment       s       |err|" + ("  position (m)  rotation (rad)" if se3 else " (m)") + "   end-effector")
    for i, (qi, ti, squared_error) in enumerate(trail):
        position = sce.end_effector(qi)
        errors = f"  {pose[i]['position_error']:12.3e}  {pose[i]['rotation_error']:14.3e}" if se3 else ""
        print(
            f"  {i:4d}  {ti:7.4f}  {fields[i]['segment_index']:7d}  {fields[i]['s']:6.3f}  "
            f"{np.sqrt(max(squared_error, 0.0)):11.3e}{errors}   {np.round(position, 3).tolist()}"
        )

    converged = trail[-1][2] <= args.tolerance
    print(
        f"-> {'converged' if converged else 'did NOT converge'} after {len(trail) - 1} iterations, "
        f"final |err| {np.sqrt(max(trail[-1][2], 0.0)):.2e} m, t {trail[-1][1]:.4f}"
    )

    # the time of the projection: the same call repeated, the mean of the repeats (it includes the call's overhead
    # from Python and the copying of the iterates, the same for every mode)
    if args.repeat > 0:
        arguments = (q.tolist(), t, method, args.tolerance, args.max_iterations, args.alpha)
        manifold.projection_trail(*arguments)  # warm up
        started = time.perf_counter_ns()
        for _ in range(args.repeat):
            manifold.projection_trail(*arguments)

        per_projection = (time.perf_counter_ns() - started) / args.repeat / 1e3
        iterations = max(len(trail) - 1, 1)
        print(
            f"time: {per_projection:.1f} us per projection ({args.repeat} repeats), {iterations} iterations, "
            f"{per_projection / iterations:.1f} us per iteration"
        )

    # a projection that stalls above the tolerance is often a joint held at its limit
    final = np.asarray(trail[-1][0], dtype=np.float64)
    lower = np.asarray(vamp.panda.lower_bounds(), dtype=np.float64)
    upper = np.asarray(vamp.panda.upper_bounds(), dtype=np.float64)
    at_limit = [f"joint {j} at {'lower' if final[j] - lower[j] < 1e-3 else 'upper'} limit ({final[j]:.3f})"
                for j in range(len(final)) if final[j] - lower[j] < 1e-3 or upper[j] - final[j] < 1e-3]
    print("joints at a limit: " + (", ".join(at_limit) if at_limit else "none"))

    output = f"{args.prefix}_{args.mode}.json"
    if se3:
        fields = [{**a, **b} for a, b in zip(fields, pose)]

    position_at = (lambda t: manifold.value(t)[0]) if se3 else None
    ex.write_curve_dump(
        output, f"{args.mode}, {args.curve}, {args.method}, sample t = {t:.4f}", points, manifold, trail, position_at, fields
    )
    print(f"written to {output}")

    if args.view:
        subprocess.run([sys.executable, str(sce.VIEWER), output], check=False)


if __name__ == "__main__":
    main()
