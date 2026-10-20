"""Exercises vamp.panda.SplineConstraint from Python and writes the projections for the viser viewer.

A cubic B-spline is fitted through 20 end-effector points, wrapped in a SplineConstraint, and a configuration off the
curve is projected onto it with each method. Checked:
  1. closest_t(q): the closest curve point to the end-effector and its squared distance, against a dense scan here;
  2. projection_trail(q, method): the constraint's own step-by-step projection lands on the curve, t stays in [0, 1];
  3. vamp.panda.project / satisfied: the generic API agrees (the result is on the curve and satisfied).

The 20 points are the first 20 end-effector positions of the first trajectory in a file written by
evaluate_mbm.py --save_eef (default: /tmp/eef_bookshelf_small.json), and the start configuration is that
trajectory's 10th configuration (on the curve) perturbed by noise so that it is off it. Without the file, an arc of
points near the Panda's start end-effector and the Panda's start configuration are used.

Each method's projection is written to <prefix>_<method>.json, in the format of cricket's tests/view_spline.py
(the curve, waypoints and control points, the Panda at each step, the end-effector and the curve point it is matched
to). With --view the viewer is started on the inner-LM projection once the checks are done:

    python3 scripts/spline_constraint_example.py --view
    python3 cricket/tests/view_spline.py spline_constraint_py_bounded_ls.json      # any of the files, port 8080

Needs a Panda built with a header generated with "spline_constraint": true (cricket).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import vamp

N = vamp.panda.spline_control_points()
REPO = Path(__file__).resolve().parents[2]
DEFAULT_TRAJECTORY = Path("/tmp/eef_bookshelf_small.json")  # written by evaluate_mbm.py --save_eef
VIEWER = REPO / "cricket" / "tests" / "view_spline.py"

failures = 0


def check(condition, message):
    global failures
    if not condition:
        failures += 1
        print(f"FAIL: {message}", file=sys.stderr)


# ---- the spline on the host (numpy, float64): the same fit and evaluation as cricket's tests/spline.hh ----


def basis(knots, span, t):
    """The four basis values that are non-zero on a span (Cox-de Boor)."""
    left, right, values = np.zeros(4), np.zeros(4), np.zeros(4)
    values[0] = 1.0
    for j in range(1, 4):
        left[j] = t - knots[span + 1 - j]
        right[j] = knots[span + j] - t
        saved = 0.0
        for r in range(j):
            term = values[r] / (right[r + 1] + left[j - r])
            values[r] = saved + right[r + 1] * term
            saved = left[j - r] * term
        values[j] = saved
    return values


def find_span(knots, count, t):
    if t >= knots[count]:
        return count - 1
    return max(3, int(np.searchsorted(knots[3 : count + 1], t, side="right")) + 2)


def evaluate(control_points, knots, t):
    t = min(max(float(t), 0.0), 1.0)
    span = find_span(knots, len(control_points), t)
    return basis(knots, span, t) @ control_points[span - 3 : span + 1]


def fit_cubic_bspline(points):
    """Global interpolation with chord-length parameters and averaged clamped knots."""
    count = len(points)
    chords = np.linalg.norm(np.diff(points, axis=0), axis=1)
    parameters = np.concatenate([[0.0], np.cumsum(chords) / chords.sum()])
    parameters[-1] = 1.0

    knots = np.zeros(count + 4)
    knots[count:] = 1.0
    for j in range(1, count - 3):
        knots[j + 3] = parameters[j : j + 3].mean()

    system = np.zeros((count, count))
    for k, t in enumerate(parameters):
        span = find_span(knots, count, t)
        system[k, span - 3 : span + 1] = basis(knots, span, t)

    control_points = np.linalg.solve(system, points)
    return control_points, knots, parameters


def closest_on_curve(control_points, knots, x, samples=20001):
    """The closest curve point to x by a dense scan: (t, squared distance)."""
    ts = np.linspace(0.0, 1.0, samples)
    curve = np.array([evaluate(control_points, knots, t) for t in ts])
    squared = np.sum((curve - x) ** 2, axis=1)
    best = int(np.argmin(squared))
    return float(ts[best]), float(squared[best])


def end_effector(q):
    return np.asarray(vamp.panda.eefk(np.asarray(q, dtype=np.float32)), dtype=np.float64)[:3, 3]


# ---- the viewer's JSON ----


def write_dump(path, label, points, control_points, knots, parameters, trail, extra=None):
    ts = np.linspace(0.0, 1.0, 401)
    curve = np.array([evaluate(control_points, knots, t) for t in ts])
    knot_points = np.array([evaluate(control_points, knots, knots[i]) for i in range(3, N + 1)])

    iterations = []
    for q, t, squared_error in trail:
        tangent = (evaluate(control_points, knots, t + 1e-5) - evaluate(control_points, knots, t - 1e-5)) / 2e-5
        iterations.append(
            {
                "t": float(t),
                "q": [float(v) for v in q],
                "arm": [end_effector(q).tolist()],
                "curve_point": evaluate(control_points, knots, t).tolist(),
                "tangent": tangent.tolist(),
                "error_norm": float(np.sqrt(max(squared_error, 0.0))),
            }
        )

    with open(path, "w") as f:
        json.dump(
            {
                **(extra or {}),
                "label": label,
                "nq": 7,
                "waypoints": np.asarray(points).tolist(),
                "control_points": control_points.tolist(),
                "knots": knots.tolist(),
                "parameters": parameters.tolist(),
                "curve_t": ts.tolist(),
                "curve": curve.tolist(),
                "knot_points": knot_points.tolist(),
                "iterations": iterations,
            },
            f,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY, help="evaluate_mbm.py --save_eef file")
    parser.add_argument("--prefix", default="spline_constraint_py", help="prefix of the output JSON files")
    parser.add_argument("--noise", type=float, default=0.15, help="joint noise (rad) that takes the start off the curve")
    parser.add_argument("--view", action="store_true", help="start the viewer on the inner-LM projection when done")
    args = parser.parse_args()

    rng = np.random.default_rng(0)

    # the points to fit and the start configuration
    if args.trajectory.exists():
        entry = next(iter(json.load(open(args.trajectory))["trajectories"].values()))[0]
        points = np.array(entry["eef_positions"][:N], dtype=np.float64)
        on_curve = np.array(entry["configurations"][10], dtype=np.float32)
        q = (on_curve + args.noise * rng.standard_normal(7)).astype(np.float32)
        print(f"points: the first {N} end-effector positions of {args.trajectory.name}, start: its configuration 10 + noise")
    else:
        q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
        center = end_effector(q)
        s = np.linspace(0.0, 1.0, N)
        points = center + np.stack([0.05 * (s - 0.5), 0.30 * (s - 0.5), 0.05 * np.sin(np.pi * s)], axis=1)
        print(f"points: an arc near the start end-effector (no trajectory file at {args.trajectory})")

    control_points, knots, parameters = fit_cubic_bspline(points)
    interpolation_error = max(
        np.linalg.norm(evaluate(control_points, knots, parameters[i]) - points[i]) for i in range(N)
    )
    check(interpolation_error < 1e-9, f"the fitted spline passes through the points (error {interpolation_error:.2e})")

    constraint = vamp.panda.SplineConstraint(
        control_points.tolist(), knots.tolist(), points.tolist(), parameters.tolist()
    )

    # 1. the distance function against a dense scan
    t, d2 = constraint.closest_t(q.tolist())
    host_t, host_d2 = closest_on_curve(control_points, knots, end_effector(q))
    print(f"closest t at the start: kernel t {t:.6f}, squared distance {d2:.6e}; host t {host_t:.6f}, {host_d2:.6e}")
    check(host_d2 > 1e-5, "the start configuration is off the curve")
    check(abs(d2 - host_d2) < 1e-6, f"squared distance {d2:.3e} vs the host's {host_d2:.3e}")
    check(abs(t - host_t) < 1e-3, f"closest t {t:.5f} vs the host's {host_t:.5f}")

    # 2.-3. projection with each method
    runs = [
        ("inner LM", "inner_lm", vamp.ProjMethod.InnerLM, True),
        ("outer LM", "outer_lm", vamp.ProjMethod.OuterLM, True),
        ("bounded LS", "bounded_ls", vamp.ProjMethod.BoundedLS, True),
        ("gradient descent", "gradient_descent", vamp.ProjMethod.GradDesc, False),
    ]

    for name, slug, method, must_converge in runs:
        trail = constraint.projection_trail(q.tolist(), method, 1e-8, 50, 1.0)
        final_q, final_t, final_error = trail[-1]
        host_t, host_d2 = closest_on_curve(control_points, knots, end_effector(final_q))
        print(
            f"{name:17s}: {len(trail) - 1:2d} steps, squared error {trail[0][2]:.3e} -> {final_error:.3e}, "
            f"host squared distance {host_d2:.3e}, t {final_t:.5f} (host closest {host_t:.5f})"
        )

        check(0.0 <= final_t <= 1.0, f"{name}: t {final_t} stays in [0, 1]")
        if must_converge:
            check(final_error <= 1e-8, f"{name}: converged (squared error {final_error:.2e})")
            check(host_d2 < 1e-7, f"{name}: the end-effector is on the curve (host squared distance {host_d2:.2e})")
            check(abs(final_t - host_t) < 1e-3, f"{name}: t {final_t:.5f} is the closest t {host_t:.5f}")

            # the generic API: project, then satisfied
            settings = vamp.ConstraintSettings()
            settings.method = method
            settings.tolerance = 1e-8
            settings.max_iterations = 50
            try:
                projected = vamp.panda.project(q, [constraint], settings)
                check(vamp.panda.satisfied(projected, [constraint], settings), f"{name}: project() result is satisfied")
                check(
                    closest_on_curve(control_points, knots, end_effector(projected))[1] < 1e-7,
                    f"{name}: project() result is on the curve",
                )
            except Exception as error:  # project raises when it does not converge
                check(False, f"{name}: vamp.panda.project raised: {error}")

        path = f"{args.prefix}_{slug}.json"
        write_dump(path, name, points, control_points, knots, parameters, trail)
        print(f"{'':17s}  written to {path}")

    if failures:
        print(f"\nspline_constraint_example: {failures} check(s) failed", file=sys.stderr)
        sys.exit(1)

    print("\nspline_constraint_example: all checks passed")

    if args.view:
        subprocess.run([sys.executable, str(VIEWER), f"{args.prefix}_inner_lm.json"], check=False)


if __name__ == "__main__":
    main()
