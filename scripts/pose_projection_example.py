"""Projection steps onto a pose curve in R^3 x SO(3), held at the point P(t) and sliding along its segment, for viser.

Takes the pose polyline of spline_task_space_rrtc_example.py --curve linear-se3 (positions along a polyline, rotations
along a slerp), a configuration near its start, and a parameter t that a planner would give a state, and runs the
projection onto the curve the way the planner does (vamp.panda.PosePolylineConstraint.projection_trail):

  fixed   the end-effector's pose must equal the curve's pose at t, the parameter given
  slide   t only picks the segment: the pose may slide along it, t being an unknown of the projection, bounded to the
          segment (the generated solvers with a column for t), and the state takes the t where it ends

The candidate is the start configuration plus Gaussian noise (--perturb rad per joint), the parameter is the start's
closest t plus --t-offset: a state's t that is off from where its configuration is, as after an extension that
interpolated t. Both projections start from the same candidate and the same t.

Prints the iterates of the first trial (error, t, the position and rotation parts of the error) and, over --trials
trials, how many projections converged and how many iterations they took. Writes <prefix>_fixed.json and
<prefix>_slide.json in the format of cricket's tests/view_spline.py (which draws the end-effector's and the curve's
frames, so the rotations show):

    python3 scripts/pose_projection_example.py --shape s --s-spacing 0.08 --view
    python3 cricket/tests/view_spline.py pose_projection_fixed.json pose_projection_slide.json

Needs a Panda built with a header generated with "spline_constraint", "constraints" and "pose_slide_constraint".
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import vamp

import spline_constraint_example as sce
import spline_task_space_rrtc_example as ex


def clip(t):
    return float(min(max(t, 0.0), 1.0))


def pose_fields(manifold, trail):
    """The fields a pose curve adds to the viewer's steps: the frames of the end-effector and of the curve at t, and the
    position and rotation errors."""
    fields = []
    for q, t, _ in trail:
        position, rotation = manifold.value(t)
        ee_rotation = ex.end_effector_quaternion(q)
        fields.append(
            {
                "eef_quaternion": ee_rotation.tolist(),
                "target_quaternion": list(rotation),
                "position_error": float(np.linalg.norm(sce.end_effector(q) - np.array(position))),
                "rotation_error": float(ex.rotation_angle(ee_rotation, np.array(rotation))),
            }
        )

    return fields


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, default=sce.DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--shape", choices=["trajectory", "s"], default="s", help="the curve: the trajectory's points, or a synthetic S")
    parser.add_argument("--s-radius", type=float, default=0.10)
    parser.add_argument("--s-spacing", type=float, default=None, help="instead of --s-radius: the distance (m) between consecutive points of the S")
    parser.add_argument("--turn", type=float, default=60.0, help="rotation about the world z axis (degrees) from the first keyframe to the last")
    parser.add_argument("--rot-weight", type=float, default=0.1, help="rotation weight (m/rad)")
    parser.add_argument("--method", choices=["BoundedLS", "InnerLM", "OuterLM", "GradDesc"], default="InnerLM")
    parser.add_argument("--perturb", type=float, default=0.1, help="noise (rad, per joint) added to the start configuration")
    parser.add_argument("--t-offset", type=float, default=0.1, help="the state's t is the start's closest t plus this")
    parser.add_argument("--trials", type=int, default=50, help="random candidates to compare the two projections on")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--tolerance", type=float, default=1e-8, help="squared error at which a projection has converged")
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--alpha", type=float, default=1.0, help="step size")
    parser.add_argument("--prefix", default="pose_projection")
    parser.add_argument("--view", action="store_true", help="start the viewer on the two dumps when done")
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

    rotations = ex.keyframe_rotations(start, len(points), args.turn)
    fixed = vamp.panda.PosePolylineConstraint(points.tolist(), rotations.tolist(), args.rot_weight, False)
    slide = vamp.panda.PosePolylineConstraint(points.tolist(), rotations.tolist(), args.rot_weight, True)
    method = getattr(vamp.ProjMethod, args.method)

    t_start, distance2 = fixed.closest_t(start.tolist())
    print(f"start: closest t {t_start:.4f} (squared distance {distance2:.2e}), method {args.method}")

    rng = np.random.default_rng(args.seed)
    results = {"fixed": [], "slide": []}
    first = {}
    for trial in range(args.trials):
        q = (start + args.perturb * rng.standard_normal(7)).astype(np.float32)
        t_hint = clip(t_start + args.t_offset)

        for name, manifold in (("fixed", fixed), ("slide", slide)):
            trail = manifold.projection_trail(q.tolist(), t_hint, method, args.tolerance, args.max_iterations, args.alpha)
            converged = trail[-1][2] <= args.tolerance
            results[name].append((converged, len(trail) - 1))
            if trial == 0:
                first[name] = (manifold, trail)

    # the iterates of the first trial
    for name in ("fixed", "slide"):
        manifold, trail = first[name]
        fields = pose_fields(manifold, trail)
        print(f"\n{name}: first trial, t starts at {trail[0][1]:.4f}")
        print("  iter        t   |err|^2      position (m)  rotation (rad)")
        for i, (q, t, squared_error) in enumerate(trail):
            print(
                f"  {i:4d}  {t:7.4f}  {squared_error:9.2e}  "
                f"{fields[i]['position_error']:12.3e}  {fields[i]['rotation_error']:14.3e}"
            )

        label = f"{name}, {args.method}, candidate = start + {args.perturb} rad noise, t = closest + {args.t_offset}"
        output = f"{args.prefix}_{name}.json"
        ex.write_curve_dump(output, label, points, manifold, trail, lambda t: manifold.value(t)[0], fields)
        print(f"written to {output}")

    # over the trials
    print(f"\nover {args.trials} trials (tolerance {args.tolerance:g}, at most {args.max_iterations} iterations):")
    for name, outcomes in results.items():
        converged = [iterations for ok, iterations in outcomes if ok]
        text = f"{len(converged)}/{args.trials} converged"
        if converged:
            text += f", iterations: mean {np.mean(converged):.1f}, median {np.median(converged):.0f}, max {max(converged)}"
        print(f"  {name:5s}  {text}")

    if args.view:
        subprocess.run(
            [sys.executable, str(sce.VIEWER), f"{args.prefix}_fixed.json", f"{args.prefix}_slide.json"], check=False
        )


if __name__ == "__main__":
    main()
