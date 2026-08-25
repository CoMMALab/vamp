"""Task-space planning benchmark for iiwa_marker: plan through every maze problem in
resources/iiwa_marker/maze_problems_checked_ik.json using the
`iiwamarker.parameterized_space` python bindings, and save results to a json matching
the shape scripts/cpp/iiwa_maze_solver_benchmark.cc writes (see
resources/iiwa_marker/maze_solver_benchmark_paths.json) -- run scripts/
compare_iiwa_maze_results.py against both to sanity-check the python bindings against
the reference C++ path.

This plans directly over end-effector poses (+ a redundancy parameter psi) instead of
joint configurations, using TaskSpaceInformedSampler to draw samples inside a Task Space
Region (TSR) instead of rejecting joint-space samples on a downstream IK check.

Usage:
    python scripts/iiwa_marker_maze_example.py
    python scripts/iiwa_marker_maze_example.py --limit 10 --output /tmp/quick_check.json
"""

import json
import math
import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

RESOURCES = Path(__file__).parents[1] / "resources"
MAZE_JSON = RESOURCES / "environments" / "real_maze.json"
DEFAULT_PROBLEMS_JSON = RESOURCES / "iiwa_marker" / "maze_problems_checked_ik.json"
DEFAULT_OUTPUT_JSON = RESOURCES / "iiwa_marker" / "maze_solver_benchmark_paths_python.json"

# Marker tip pose IS the eef pose, so eef_to_offset is identity (x, y, z, qx, qy, qz, qw).
EEF_TO_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# TSR reference frame: tool facing straight down (qx=1) at the maze's working height.
WORLD_TO_REFERENCE = [0.0, 0.0, 0.22607783, 0.0, 1.0, 0.0, 0.0]
Z_HEIGHT = 0.22607783
DOWN_QUAT = (0.0, -1.0, 0.0, 0.0)  # (qx, qy, qz, qw)

# Bound order is (dx, dy, dz, rx, ry, rz): translation box + so(3) log-map rotation box.
# Matches iiwa_maze_solver_benchmark.cc -- a narrower box starves the informed sampler of
# the space it needs to route around obstacles, even when start/goal are IK-valid.
TSR_LOWER = [-0.85, -0.7, 0.0, 0.0, 0.0, -math.pi]
TSR_UPPER = [0.0, 0.7, 0.0, 0.0, 0.0, math.pi]


def load_maze_environment() -> vamp.Environment:
    with open(MAZE_JSON) as f:
        cuboids = json.load(f)

    env = vamp.Environment()
    for c in cuboids:
        # Matches the C++ demo's slight forward push on x.
        position = [c["x"] + 0.05, c["y"], c["z"]]
        orientation = [c.get("roll", 0.0), c.get("pitch", 0.0), c.get("yaw", 0.0)]
        half_extents = [c["dx"] / 2.0, c["dy"] / 2.0, c["dz"] / 2.0]
        env.add_cuboid(vamp.Cuboid(position, orientation, half_extents))

    return env


def make_pose_array(eef_pos, psi: float) -> np.ndarray:
    return np.array([eef_pos[0], eef_pos[1], Z_HEIGHT, *DOWN_QUAT, psi], dtype=np.float32)


def resolve_and_check(param, ambient, state: np.ndarray, environment: vamp.Environment) -> bool:
    """IK-resolve a task-space state and collision-check the resulting ambient config."""
    valid, ambient_config = param.resolve(state)
    if not valid:
        return False
    return ambient.validate(np.asarray(ambient_config, dtype=np.float32), environment)


def resolve_ambient_path(param, states: np.ndarray) -> np.ndarray:
    """IK-resolve every task-space waypoint to its ambient (joint-space) configuration --
    this is what's actually physically reachable, so it's the representation "distance"
    analysis should really care about. `states` is an (n, dimension) array."""
    out = np.empty((len(states), vamp.iiwamarker.dimension()), dtype=np.float32)
    for i, state in enumerate(states):
        _, ambient_config = param.resolve(state)
        out[i] = np.asarray(ambient_config, dtype=np.float32)
    return out


def ambient_path_distance(ambient_path: np.ndarray) -> float:
    """Euclidean distance summed over consecutive ambient (joint-space) waypoints."""
    if len(ambient_path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(ambient_path, axis=0), axis=1).sum())


def se3_distance(ta, qa, tb, qb) -> float:
    """SE3 distance (translation distance and quaternion angle, combined in quadrature)
    between two eef poses; qa/qb are (w, x, y, z). Kept identical to the C++ benchmark's
    version so the two files' "eef distance" numbers are directly comparable."""
    translation_distance = float(np.linalg.norm(tb - ta))
    dot = min(1.0, abs(float(np.dot(qa, qb))))
    rotation_distance = 2.0 * math.acos(dot)
    return math.sqrt(translation_distance ** 2 + rotation_distance ** 2)


def path_se3_distance(path: np.ndarray) -> float:
    """Total SE3 distance along a task-space pose path. The state already IS the eef pose
    (x, y, z, qx, qy, qz, qw, psi) -- no FK needed -- and psi (index 7) is never read,
    since it isn't part of the eef pose."""
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        ta, qa = a[:3], np.array([a[6], a[3], a[4], a[5]])
        tb, qb = b[:3], np.array([b[6], b[3], b[4], b[5]])
        total += se3_distance(ta, qa, tb, qb)
    return total


def run_problem(param, ambient, environment, sampler, settings, problem: dict, index: int):
    """Plan + shortcut one problem. Returns a result dict (schema-compatible with the C++
    benchmark's output, plus a `python_shortcut_seconds` field the C++ side doesn't have,
    since the shortcut() binding returns only a changed-or-not bool, not timing) or None
    if the problem's start/goal isn't valid."""
    start_state = make_pose_array(problem["start_eef_pos"], problem["start_psi"])
    goal_state = make_pose_array(problem["goal_eef_pos"], problem["goal_psi"])

    if not (resolve_and_check(param, ambient, start_state, environment) and
            resolve_and_check(param, ambient, goal_state, environment)):
        return None

    # Restart the Halton sequence for each problem so results are reproducible per-problem
    # and independent of how many samples earlier problems in this run consumed -- matches
    # the C++ benchmark's task_sampler->reset() before every problem.
    sampler.reset()

    result = param.rrtc(start_state, goal_state, environment, settings, sampler)
    if not result.solved:
        return {"solved": False}

    raw_path = result.path.numpy().copy()  # shortcut() mutates result.path in place below
    raw_ambient_path = resolve_ambient_path(param, raw_path)

    shortcut_t0 = time.perf_counter()
    changed = param.shortcut(result.path, environment)
    shortcut_seconds = time.perf_counter() - shortcut_t0

    shortcut_path = result.path.numpy()
    shortcut_ambient_path = resolve_ambient_path(param, shortcut_path)

    return {
        "solved": True,
        "problem_index": index,
        "start_eef_pos": list(problem["start_eef_pos"]),
        "goal_eef_pos": list(problem["goal_eef_pos"]),
        "nanoseconds": result.nanoseconds,
        "iterations": result.iterations,
        "path": raw_path.tolist(),
        "ambient_path": raw_ambient_path.tolist(),
        "path_ambient_distance": ambient_path_distance(raw_ambient_path),
        "path_se3_distance": path_se3_distance(raw_path),
        "shortcut_changed": changed,
        "shortcut_python_seconds": shortcut_seconds,
        "shortcut_path": shortcut_path.tolist(),
        "shortcut_ambient_path": shortcut_ambient_path.tolist(),
        "shortcut_path_ambient_distance": ambient_path_distance(shortcut_ambient_path),
        "shortcut_path_se3_distance": path_se3_distance(shortcut_path),
    }


def print_summary(nanoseconds_per_problem, iterations_per_problem, total_problems, valid_problems):
    successful = len(nanoseconds_per_problem)
    print(f"Total problems: {total_problems}")
    print(f"Valid problems: {valid_problems}")
    print(f"Successful problems: {successful}")
    if valid_problems:
        print(f"Success rate: {successful / valid_problems * 100.0:.2f}%")
    if not successful:
        return

    ns = np.sort(np.asarray(nanoseconds_per_problem, dtype=np.float64))
    its = np.sort(np.asarray(iterations_per_problem, dtype=np.float64))

    def pct(arr, p):
        return arr[min(len(arr) - 1, int(p * len(arr)))]

    print(f"Average time (ms): {ns.mean() / 1e6:.3f}")
    print(f"Average iterations: {its.mean():.1f}")
    print(f"Median time (ms): {pct(ns, 0.5) / 1e6:.3f}")
    print(f"Median iterations: {pct(its, 0.5):.1f}")
    print(f"Minimum time (ms): {ns[0] / 1e6:.3f}")
    print(f"Minimum iterations: {its[0]:.1f}")
    print(f"Maximum time (ms): {ns[-1] / 1e6:.3f}")
    print(f"Maximum iterations: {its[-1]:.1f}")
    print(f"Q1 time (ms): {pct(ns, 0.25) / 1e6:.3f}")
    print(f"Q1 iterations: {pct(its, 0.25):.1f}")
    print(f"Q3 time (ms): {pct(ns, 0.75) / 1e6:.3f}")
    print(f"Q3 iterations: {pct(its, 0.75):.1f}")
    print(f"95th percentile time (ms): {pct(ns, 0.95) / 1e6:.3f}")
    print(f"95th percentile iterations: {pct(its, 0.95):.1f}")


def main(
    problems: str = str(DEFAULT_PROBLEMS_JSON),
    output: str = str(DEFAULT_OUTPUT_JSON),
    limit: int = None,  # Only run the first `limit` problems (for a quick check).
    range_: float = 0.75,  # RRTC range; matches iiwa_maze_solver_benchmark.cc.
    max_iterations: int = 500_000,
    max_samples: int = 500_000,
):
    environment = load_maze_environment()
    print(f"Loaded {len(environment.cuboids) + len(environment.z_aligned_cuboids)} cuboids from {MAZE_JSON.name}")

    with open(problems) as f:
        problem_list = json.load(f)
    if limit is not None:
        problem_list = problem_list[:limit]
    print(f"Loaded {len(problem_list)} problems from {problems}")

    ambient = vamp.iiwamarker
    param = ambient.parameterized_space

    inner = param.halton()
    sampler = param.TaskSpaceInformedSampler(
        EEF_TO_OFFSET, WORLD_TO_REFERENCE, TSR_LOWER, TSR_UPPER, environment, inner)

    settings = vamp.RRTCSettings()
    settings.range = range_
    settings.max_iterations = max_iterations
    settings.max_samples = max_samples
    settings.dynamic_domain = False

    all_results = []
    nanoseconds_per_problem = []
    iterations_per_problem = []
    valid_problems = 0

    for index, problem in enumerate(problem_list):
        print(f"Planning problem {index + 1} / {len(problem_list)}")
        entry = run_problem(param, ambient, environment, sampler, settings, problem, index)
        if entry is None:
            print("  skipping problem due to invalid start or goal configuration.")
            continue

        valid_problems += 1
        if not entry["solved"]:
            print("  unable to solve problem with start and goal configs.")
            continue

        nanoseconds_per_problem.append(entry["nanoseconds"])
        iterations_per_problem.append(entry["iterations"])
        all_results.append(entry)

        with open(output, "w") as f:
            json.dump(all_results, f, indent=4)

    print(f"\nSaved {len(all_results)} problem paths to {output}")
    print_summary(nanoseconds_per_problem, iterations_per_problem, len(problem_list), valid_problems)


if __name__ == "__main__":
    Fire(main)
