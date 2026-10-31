"""Plans along a straight-line (lerp) curve in (q, t) from Python: vamp.panda.task_space_rrtc_curve.

The curve is the straight-line interpolation of 20 keyframes that start at the end-effector's pose at the Panda's ready
configuration and wander from there in a gentle S: positions in R^3 (--curve r3, vamp.panda.LerpR3Constraint) or poses in
SO(3) x R^3 with the rotation slerped, turning about the world z axis (--curve so3r3, vamp.panda.LerpSO3R3Constraint,
with an adjustable --rot-weight in m / rad). t in [0, 1] is the normalized chord length. States of the tree carry the t
they sit at, sampled with the joints and held fixed while a state is projected onto the curve; the goal is the end of
the curve, which the planner holds a copy of the constraint at t = 1 for, so no goal constraint is given.

Checked: the planner solves; every state of the path is on the curve at its own t; the path ends at the curve's last
pose; every state is collision-free. The whole path is printed and written to <prefix>_<curve>_path.json, in the format
of cricket's tests/view_lerp.py:

    python3 scripts/lerp_task_space_rrtc_example.py --curve so3r3 --view
    python3 cricket/tests/view_lerp.py lerp_so3r3_path.json

Needs a Panda built with a header generated with "lerp_r3_constraint": true and "lerp_so3r3_constraint": true (cricket).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import vamp

READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
VIEWER = Path(__file__).resolve().parents[2] / "cricket" / "tests" / "view_lerp.py"

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
    pose = vamp.panda.eefk(np.asarray(q, dtype=np.float32))
    return np.asarray(pose[:3, 3], dtype=np.float64), quaternion_from_matrix(pose[:3, :3])


def keyframes(count, turn_degrees):
    """Positions and quaternions of the curve: from the ready pose's end-effector pose, a gentle S in position, and the
    rotation turning about the world z axis by `turn_degrees` over the curve."""
    position, rotation = end_effector(READY)
    positions = []
    rotations = []
    for i in range(count):
        s = i / (count - 1)
        positions.append(position + np.array([0.20 * s, 0.12 * np.sin(2.0 * np.pi * s), 0.08 * s]))
        angle = np.radians(turn_degrees) * s
        about_z = np.array([np.cos(0.5 * angle), 0.0, 0.0, np.sin(0.5 * angle)])
        rotations.append(quaternion_product(about_z, rotation))

    return np.array(positions), np.array(rotations)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--curve", choices=["r3", "so3r3"], default="r3", help="positions in R^3, or poses in SO(3) x R^3")
    parser.add_argument("--rot-weight", type=float, default=0.3, help="with so3r3: the weight of the rotation error (m / rad)")
    parser.add_argument("--turn", type=float, default=60.0, help="with so3r3: the rotation about the world z axis (degrees) over the curve")
    parser.add_argument("--range", type=float, default=0.3, help="longest extension (in the metric of the run)")
    parser.add_argument("--parameter-weight", type=float, default=0.0, help="the weight of t in the metric (rad per unit of t); 0 is 1 / the number of joints")
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--project-samples", action="store_true", help="project random samples onto the curve before using them")
    parser.add_argument("--prefix", default="lerp", help="prefix of the output JSON file")
    parser.add_argument("--view", action="store_true", help="start the viewer on the path when done")
    args = parser.parse_args()

    count = vamp.panda.lerp_points()
    positions, rotations = keyframes(count, args.turn)
    pose_curve = args.curve == "so3r3"
    if pose_curve:
        manifold = vamp.panda.LerpSO3R3Constraint(positions.tolist(), rotations.tolist(), args.rot_weight)
        print(f"lerp curve in SO(3) x R^3 through {count} poses, rotation weight {args.rot_weight}")
    else:
        manifold = vamp.panda.LerpR3Constraint(positions.tolist())
        print(f"lerp curve in R^3 through {count} points")

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

    # the start has to be on the manifold: the ready pose is where the curve starts
    start = READY
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

    result = vamp.panda.task_space_rrtc_curve(
        start, env, plan_settings, vamp.panda.halton(), [manifold], constraint_settings, start_t=t0
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
            all(vamp.panda.validate(np.asarray(q, dtype=np.float32), env) for q in path),
            "every path state is collision-free",
        )

        ts_curve = np.linspace(0.0, 1.0, 401)
        output = f"{args.prefix}_{args.curve}_path.json"
        with open(output, "w") as f:
            json.dump(
                {
                    "kind": args.curve,
                    "mode": "path",
                    "label": f"lerp {args.curve}, path ({len(path)} states)",
                    "nq": 7,
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
