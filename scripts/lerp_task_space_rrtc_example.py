"""Plans along a straight-line (lerp) curve in (q, t) from Python: vamp.<robot>.task_space_rrtc_curve.

The curve is the straight-line interpolation of lerp_points() keyframes that start at the end-effector's pose at the Panda's ready
configuration and wander from there in a gentle S: positions in R^3 (--curve r3, vamp.panda.LerpR3Constraint) or poses in
SO(3) x R^3 with the rotation slerped, turning about the world z axis (--curve so3r3, vamp.panda.LerpSO3R3Constraint,
with an adjustable --rot-weight in m / rad). t in [0, 1] is the normalized chord length. States of the tree carry the t
they sit at, sampled with the joints and held fixed while a state is projected onto the curve; the goal is the end of
the curve, which the planner holds a copy of the constraint at t = 1 for, so no goal constraint is given.

With --segment the manifold's constraint is in its segment form (qs): t only picks the segment of a state, a projection
solves for t as well, bounded to the segment, and every state takes the t its projection ended at; the goal is still the
curve at exactly t = 1. The checks are the same.

With --traj vertical_s the curve is instead the vertical S of the old spline examples (--s-radius, a default per robot,
or --s-spacing, the distance between consecutive points; radius = spacing * (points - 1) / 8.4): two arcs of 240 degrees
joined smoothly in a plane in front of the robot, centred on the end-effector of the ready pose, 4 radii tall and 2 wide. The start is the old one too: the
ready pose projected onto a point region (position only) at the S's first point. For --curve so3r3 the keyframes'
rotations start from that start configuration's own orientation and turn about the world z axis by --turn degrees, so
the start is on the pose curve. The environment is empty.

Checked: the planner solves; every state of the path is on the curve at its own t; the path ends at the curve's last
pose; every state is collision-free. The whole path is printed and written to <prefix>_<robot>_<curve>_<qt|qs>_path.json, in the format
of cricket's tests/view_lerp.py:

    python3 scripts/lerp_task_space_rrtc_example.py --curve so3r3 --view
    python3 scripts/lerp_task_space_rrtc_example.py --traj vertical_s --curve so3r3 --view
    python3 cricket/tests/view_lerp.py lerp_panda_so3r3_qt_path.json

Needs the robot built with a header generated with "lerp_r3_constraint": true and "lerp_so3r3_constraint": true (cricket;
see resources/panda.json).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import vamp

# Per robot: the ready configuration the curves start from, the shoulder and reach (for a printout), and the radius of
# the vertical S (m) that fits the workspace in front of the ready pose
ROBOTS = {
    "panda": dict(
        ready=np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32),
        shoulder=np.array([0.0, 0.0, 0.333]),
        reach=0.855,
        s_radius=0.27,
    ),
}
module = None  # vamp.<robot>, set by main
READY = None  # the robot's ready configuration, set by main
VIEWER = Path(__file__).resolve().parents[2] / "cricket" / "tests" / "view_lerp.py"

# The vertical S is two arcs of 240 degrees each, so its length is 2 * (4 pi / 3) radii
S_LENGTH_PER_RADIUS = 8.0 * np.pi / 3.0

failures = 0


def check(condition, message):
    global failures
    print(f"{'ok  ' if condition else 'FAIL'} {message}")
    if not condition:
        failures += 1


def quaternion_from_matrix(m):
    """The unit quaternion (w, x, y, z) of a rotation matrix."""
    m = np.asarray(m, dtype=np.float64)
    trace = np.trace(m)
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]

    q = np.array(q)
    return q / np.linalg.norm(q)


def quaternion_product(a, b):
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


def rotation_angle(a, b):
    """The angle (rad) between two rotations given as unit quaternions."""
    return 2.0 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))


def end_effector(q):
    """The end-effector's position and quaternion (w, x, y, z) at q."""
    pose = module.eefk(np.asarray(q, dtype=np.float32))
    return np.asarray(pose[:3, 3], dtype=np.float64), quaternion_from_matrix(pose[:3, :3])


def rotations_about_z(count, turn_degrees, rotation):
    """The quaternions of the curve's keyframes: `rotation` turning about the world z axis by `turn_degrees` over the
    curve."""
    rotations = []
    for i in range(count):
        angle = np.radians(turn_degrees) * i / (count - 1)
        about_z = np.array([np.cos(0.5 * angle), 0.0, 0.0, np.sin(0.5 * angle)])
        rotations.append(quaternion_product(about_z, rotation))

    return np.array(rotations)


def s_curve_points(radius, count):
    """The old vertical S: an S in a vertical plane (x constant), centred on the end-effector of the ready pose.

    Two arcs of radius `radius` joined smoothly at the middle, which is the centre: an upper one (counter-clockwise, 30 to
    270 degrees) and a lower one (clockwise, from the middle to 210 degrees). It is 4 radii tall and 2 wide, and 8.4 radii
    long. Returns `count` points at equal arc length.
    """
    upper_angle = np.linspace(np.pi / 6, 3 * np.pi / 2, 1200)
    upper = np.stack([radius * np.cos(upper_angle), radius + radius * np.sin(upper_angle)], axis=1)
    lower_angle = np.linspace(np.pi / 2, -5 * np.pi / 6, 1200)
    lower = np.stack([radius * np.cos(lower_angle), -radius + radius * np.sin(lower_angle)], axis=1)
    dense = np.vstack([upper, lower[1:]])  # (y, z)

    arc_length = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
    targets = np.linspace(0.0, arc_length[-1], count)
    yz = np.stack([np.interp(targets, arc_length, dense[:, 0]), np.interp(targets, arc_length, dense[:, 1])], axis=1)

    anchor = end_effector(READY)[0]
    return np.column_stack([np.full(count, anchor[0]), yz + anchor[1:3]])


def start_at(point):
    """The old start: a configuration whose end-effector is at `point`, the ready pose projected onto a position-only
    region there. If that one projection does not converge (the point is far from the ready pose's end-effector), the
    region walks there from the ready pose's end-effector in moves of at most 5 cm, each projection starting from the
    last one's solution."""
    settings = vamp.ConstraintSettings()
    settings.tolerance = 1e-9
    settings.max_iterations = 100

    def project(q, target):
        region = module.TaskSpaceConstraint(
            [1, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, *np.asarray(target).tolist()],
            [0.0, 0.0, 0.0, -3.2, -3.2, -3.2],
            [0.0, 0.0, 0.0, 3.2, 3.2, 3.2],
        )
        return np.asarray(module.project(q, [region], settings), dtype=np.float32)

    try:
        return project(READY, point)
    except ValueError:
        pass

    print("the start: one projection onto the S's first point did not converge; walking there in steps")
    origin = end_effector(READY)[0]
    steps = max(1, int(np.ceil(np.linalg.norm(np.asarray(point) - origin) / 0.05)))
    q = READY
    for k in range(1, steps + 1):
        target = origin + (np.asarray(point) - origin) * k / steps
        try:
            q = project(q, target)
        except ValueError:
            raise SystemExit(
                f"could not move the ready pose's end-effector to {np.round(point, 3).tolist()}: stopped at step {k} of "
                f"{steps}, target {np.round(target, 3).tolist()}; try a smaller --s-spacing"
            )

    return q


def keyframes(count, turn_degrees, scale=1.0):
    """Positions and quaternions of the curve: from the ready pose's end-effector pose, a gentle S in position, and the
    rotation turning about the world z axis by `turn_degrees` over the curve."""
    position, rotation = end_effector(READY)
    positions = []
    rotations = []
    for i in range(count):
        s = i / (count - 1)
        positions.append(position + scale * np.array([0.20 * s, 0.12 * np.sin(2.0 * np.pi * s), 0.08 * s]))
        angle = np.radians(turn_degrees) * s
        about_z = np.array([np.cos(0.5 * angle), 0.0, 0.0, np.sin(0.5 * angle)])
        rotations.append(quaternion_product(about_z, rotation))

    return np.array(positions), np.array(rotations)


def main():
    global module, READY
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="panda", help="the robot (its vamp module has to be built)")
    parser.add_argument("--curve", choices=["r3", "so3r3"], default="r3", help="positions in R^3, or poses in SO(3) x R^3")
    parser.add_argument("--segment", action="store_true", help="the segment form (qs): t is solved for on its segment, not held fixed")
    parser.add_argument("--rot-weight", type=float, default=0.3, help="with so3r3: the weight of the rotation error (m / rad)")
    parser.add_argument("--scale", type=float, default=1.0, help="the size of the curve: its S has extents 20 x 12 x 8 cm at scale 1 (0.54 m long); larger makes the segments longer, but the curve leaves the arm's reach beyond about 2")
    parser.add_argument("--traj", choices=["default", "vertical_s"], default="default", help="the curve: the gentle S (--scale), or the old vertical S (--s-spacing)")
    parser.add_argument("--s-radius", type=float, default=None, help="with vertical_s: the radius (m) of the S's arcs, which sets its size (default: the robot's)")
    parser.add_argument("--s-spacing", type=float, default=None, help="with vertical_s: instead of --s-radius, the distance (m) between consecutive points of the S")
    parser.add_argument("--turn", type=float, default=60.0, help="with so3r3: the rotation about the world z axis (degrees) over the curve")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension (in the metric of the run)")
    parser.add_argument("--parameter-weight", type=float, default=0.0, help="the weight of t in the metric (rad per unit of t); 0 is 1 / the number of joints")
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--project-samples", action="store_true", help="project random samples onto the curve before using them")
    parser.add_argument("--prefix", default="lerp", help="prefix of the output JSON file")
    parser.add_argument("--view", action="store_true", help="start the viewer on the path when done")
    args = parser.parse_args()

    robot = ROBOTS[args.robot]
    if not hasattr(vamp, args.robot):
        raise SystemExit(
            f"vamp has no module {args.robot!r}: build VAMP with it, for example "
            f"pip install . -C cmake.define.VAMP_ROBOTS=\"{args.robot}\""
        )

    module = getattr(vamp, args.robot)
    READY = robot["ready"]
    count = module.lerp_points()
    start = READY
    if args.traj == "vertical_s":
        if args.s_spacing is not None:
            radius = args.s_spacing * (count - 1) / S_LENGTH_PER_RADIUS
        else:
            radius = args.s_radius if args.s_radius is not None else robot["s_radius"]
        positions = s_curve_points(radius, count)
        farthest = float(np.linalg.norm(positions - robot["shoulder"], axis=1).max())
        print(
            f"vertical S: radius {radius:.3f} m, {np.linalg.norm(np.diff(positions, axis=0), axis=1).sum():.2f} m long, "
            f"the farthest point {farthest:.2f} m from the shoulder (the arm reaches about {robot['reach']} m)"
        )

        # the old start, and the rotations of the pose curve from its orientation
        start = start_at(positions[0])
        rotations = rotations_about_z(count, args.turn, end_effector(start)[1])
    else:
        positions, rotations = keyframes(count, args.turn, args.scale)
    pose_curve = args.curve == "so3r3"
    form = "qs" if args.segment else "qt"
    if pose_curve:
        manifold = module.LerpSO3R3Constraint(positions.tolist(), rotations.tolist(), args.rot_weight, args.segment)
        print(f"lerp curve in SO(3) x R^3 through {count} poses, rotation weight {args.rot_weight}, {form}")
    else:
        manifold = module.LerpR3Constraint(positions.tolist(), args.segment)
        print(f"lerp curve in R^3 through {count} points, {form}")

    taus = np.array(manifold.parameters())

    def curve_pose(t):
        """The pose of the curve at t: the position and, for poses, the quaternion (identity in R^3)."""
        if pose_curve:
            position, rotation = manifold.value(t)
            return np.array(position), np.array(rotation)

        return np.array(manifold.value(t)), np.array([1.0, 0.0, 0.0, 0.0])

    def errors(q, t):
        """The end-effector's position error and (weighted) rotation error against the curve at t."""
        position, rotation = end_effector(q)
        target_position, target_rotation = curve_pose(t)
        rotation_error = args.rot_weight * rotation_angle(rotation, target_rotation) if pose_curve else 0.0
        return float(np.linalg.norm(position - target_position)), float(rotation_error)

    env = vamp.Environment()  # empty

    # the start has to be on the manifold
    t0, d2 = manifold.closest_t(start.tolist())
    print(f"start: closest t {t0:.4f}, squared distance to the curve {d2:.2e}")
    check(d2 < 1e-8, f"the start is on the curve (squared distance {d2:.2e})")

    plan_settings = vamp.TaskSpaceRRTCSettings()
    plan_settings.range = args.range
    plan_settings.max_greedy_steps = 20
    plan_settings.max_iterations = args.max_iterations
    plan_settings.max_samples = 20000
    plan_settings.project_samples = args.project_samples
    plan_settings.parameter_weight = args.parameter_weight

    constraint_settings = vamp.ConstraintSettings()  # for the manifold and the goal region
    constraint_settings.method = vamp.ProjMethod.BoundedLS
    constraint_settings.tolerance = 1e-7
    constraint_settings.max_iterations = 50

    result = module.task_space_rrtc_curve(
        start, env, plan_settings, module.halton(), [manifold], constraint_settings, start_t=t0
    )
    print(f"task_space_rrtc_curve: solved {result.solved}, {result.iterations} iterations, {result.nanoseconds / 1e6:.2f} ms")
    check(result.solved, "the planner solved")

    if result.solved:
        path = np.asarray(result.path)
        ts = list(result.t)

        print("\n  state        t  segment       s     end-effector (m)            position err  rotation err (weighted)")
        iterations = []
        worst = 0.0
        for i, (q, t) in enumerate(zip(path, ts)):
            segment = int(np.clip(np.searchsorted(taus, t, side="right") - 1, 0, count - 2))
            s = float((t - taus[segment]) / (taus[segment + 1] - taus[segment]))
            position, rotation = end_effector(q)
            target_position, target_rotation = curve_pose(t)
            position_error, rotation_error = errors(q, t)
            worst = max(worst, float(np.hypot(position_error, rotation_error)))
            print(
                f"  {i:5d}  {t:7.4f}  {segment:7d}  {s:6.3f}   "
                f"{position[0]:7.3f} {position[1]:7.3f} {position[2]:7.3f}   {position_error:12.2e}  {rotation_error:12.2e}"
            )

            step = {
                "t": float(t),
                "q": [float(v) for v in q],
                "arm": [position.tolist()],
                "curve_point": target_position.tolist(),
                "segment": segment,
                "s": s,
                "error_norm": float(np.hypot(position_error, rotation_error)),
                "position_error": position_error,
            }
            if pose_curve:
                step["rotation_error"] = rotation_error
                step["eef_quaternion"] = rotation.tolist()
                step["curve_quaternion"] = target_rotation.tolist()

            iterations.append(step)

        final_error = float(np.hypot(*errors(path[-1], 1.0)))
        print(
            f"\npath: {len(path)} states, t {ts[0]:.4f} -> {ts[-1]:.4f}, worst error to the curve at its own t "
            f"{worst:.2e}, final error to the last pose {final_error:.2e}"
        )

        check(worst < 1e-3, f"every path state is on the curve at its own t (worst error {worst:.2e})")
        check(final_error < 2e-3, f"the path ends at the curve's last pose (error {final_error:.2e})")
        check(
            all(module.validate(np.asarray(q, dtype=np.float32), env) for q in path),
            "every path state is collision-free",
        )

        ts_curve = np.linspace(0.0, 1.0, 401)
        output = f"{args.prefix}_{args.robot}_{args.curve}_{form}_path.json"
        with open(output, "w") as f:
            json.dump(
                {
                    "kind": args.curve,
                    "mode": "path",
                    "label": f"{args.robot} lerp {args.curve}, {form}, path ({len(path)} states)",
                    "nq": len(start),
                    "weight": args.rot_weight if pose_curve else 0.0,
                    "taus": taus.tolist(),
                    "keyframes": [curve_pose(t)[0].tolist() for t in taus],
                    **({"keyframe_quaternions": [curve_pose(t)[1].tolist() for t in taus]} if pose_curve else {}),
                    "curve_t": ts_curve.tolist(),
                    "curve": [curve_pose(t)[0].tolist() for t in ts_curve],
                    "iterations": iterations,
                },
                f,
            )

        print(f"written to {output}")

    if failures:
        print(f"\nlerp_task_space_rrtc_example: {failures} check(s) failed", file=sys.stderr)
        sys.exit(1)

    print("\nlerp_task_space_rrtc_example: all checks passed")

    if args.view and result.solved:
        subprocess.run([sys.executable, str(VIEWER), output], check=False)


if __name__ == "__main__":
    main()
