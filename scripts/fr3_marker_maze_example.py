"""Task-space planning benchmark for fr3_marker: plan through every maze problem in
resources/fr3_marker/maze_problems_checked_ik.json using the
`fr3marker.parameterized_space` python bindings, and save each solved problem's shortcut
ambient path to <trajectory_dir>/problem_<n>.txt -- one waypoint per line, comma-separated
joint values -- the same format scripts/cpp/fr3_maze_solver_benchmark.cc's
write_ambient_path writes, so scripts/fr3_marker_maze_viser.py can load either script's
output unchanged.

This plans directly over end-effector poses (+ FR3's joint-7 self-motion angle, called
"psi" here for consistency with the iiwa version) instead of joint configurations,
IK-resolving each sampled/interpolated task-space state on the fly instead of rejecting
joint-space samples on a downstream IK check.

Unlike iiwa_marker, FR3's IK is redundant beyond just q7: each problem's start/goal was
resolved by the generator on a particular (elbow_sel, shoulder_sel, wrist_sel)
self-motion-manifold branch, saved in the problem file as "smm". This script pins that
branch directly via parameterized_space.set_smm() per problem (matching the C++
benchmark's --use_smm --use_psi mode) rather than searching branches, so results are
directly comparable to a --use_smm --use_psi run of the C++ benchmark.

Uses TaskSpaceInformedSampler (the TSR sampler) to keep every sample on the maze's fixed
z-plane -- an earlier version of this script fell back to plain Halton sampling because
TSR only solves a fraction of these problems, but that was the wrong call: the file's
saved "smm" branch was only checked for IK-reachability by the generator, not for
full path-connectivity, so some problems are genuinely unsolvable on that specific
branch (no --use_smm/--use_psi run of the C++ benchmark could solve them either). Solving
~25% of the 200 problems here (spot-checked: 36 of the first 39 solved indices matched
resources/fr3_marker/maze_solver_benchmark_paths.json's reference set exactly) is the
correct, expected result -- plain Halton's 100% solve rate was the tell that something
was wrong: it was cheating by leaving the z-plane entirely instead of routing through
the maze.

Usage:
    python scripts/fr3_marker_maze_example.py
    python scripts/fr3_marker_maze_example.py --limit 10 --trajectory_dir /tmp/quick_check
"""

import json
import math
import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

RESOURCES = Path(__file__).parents[1] / "resources"
MAZE_JSON = RESOURCES / "environments" / "maze_cuboids.json"
DEFAULT_PROBLEMS_JSON = RESOURCES / "fr3_marker" / "maze_problems_checked_ik.json"
DEFAULT_TRAJECTORY_DIR = RESOURCES / "fr3_marker" / "maze_solver_benchmark_trajectories_python"

# Fixed height / orientation for the end effector -- matches
# scripts/cpp/fr3_maze_solver_benchmark.cc's kEefZ and make_pose_array.
Z_HEIGHT = 0.150519
DOWN_QUAT = (0.0, -1.0, 0.0, 0.0)  # (qx, qy, qz, qw)

# Marker tip pose IS the eef pose, so eef_to_offset is identity (x, y, z, qx, qy, qz, qw).
EEF_TO_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# TSR reference frame: tool facing straight down (qy=1) at the maze's working height.
# Matches scripts/cpp/fr3_maze_solver_benchmark.cc's world_to_reference.
WORLD_TO_REFERENCE = [0.0, 0.0, Z_HEIGHT, 0.0, 1.0, 0.0, 0.0]

# Bound order is (dx, dy, dz, rx, ry, rz): translation box + so(3) log-map rotation box,
# relative to WORLD_TO_REFERENCE. dz is pinned to 0 -- this is what keeps every sampled
# state exactly on the maze's z-plane. Matches fr3_maze_solver_benchmark.cc's tsr_lower/
# tsr_upper exactly.
TSR_LOWER = [-0.85, -0.7, 0.0, 0.0, 0.0, -math.pi]
TSR_UPPER = [0.0, 0.7, 0.0, 0.0, 0.0, math.pi]

# The 4 (case6_sel, case1_sel) branches, fixed order starting at (0, 0) -- q_actual_0 (3rd smm
# slot) is left at 0.0 since --use_fixed_order_smm here keeps the problem's saved psi fixed and
# only sweeps the branch, so there's no per-attempt q_actual_0 draw the way the C++ benchmark's
# full branch+psi search has (see fr3_maze_solver_benchmark.cc's find_valid_start_goal_on_branch).
BRANCH_ORDER = [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)]


def load_maze_environment() -> vamp.Environment:
    with open(MAZE_JSON) as f:
        cuboids = json.load(f)

    env = vamp.Environment()
    for c in cuboids:
        # Matches fr3_maze_solver_benchmark.cc's load_cuboids_from_json: push forward/up
        # and pad dz, on the assumption of a shared maze/mount frame with the iiwa rig.
        position = [c["x"] + 0.285 * 2, c["y"], c["z"] + 0.05]
        orientation = [c.get("roll", 0.0), c.get("pitch", 0.0), c.get("yaw", 0.0)]
        half_extents = [(c["dx"] + 0.01) / 2.0, (c["dy"] + 0.01) / 2.0, (c["dz"] + 0.01) / 2.0]
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
    out = np.empty((len(states), vamp.fr3marker.dimension()), dtype=np.float32)
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


def resolve_branch(param, ambient, environment, branch, start_state, goal_state) -> bool:
    """Try one (case6_sel, case1_sel) branch (q_actual_0 pinned to 0.0) against the given
    (already psi-filled) start/goal states."""
    param.set_smm([branch[0], branch[1], 0.0])
    return (resolve_and_check(param, ambient, start_state, environment) and
            resolve_and_check(param, ambient, goal_state, environment))


def run_problem(param, ambient, environment, sampler, settings, problem: dict, index: int,
                 use_fixed_order_smm: bool = False):
    """Plan + shortcut one problem. By default, pinned to the problem's saved (smm, psi) -- no
    branch or psi search. With use_fixed_order_smm, the saved smm is ignored and BRANCH_ORDER's
    4 branches are swept (in fixed order, starting at (0, 0)) with the saved psi held fixed,
    keeping the first branch that resolves both endpoints -- a simpler python-side stand-in for
    the C++ benchmark's --use_fixed_order_smm (which also searches psi per branch; here psi
    stays fixed, only the branch sweeps). Returns a result dict (schema-compatible with the C++
    benchmark's output, plus a `shortcut_python_seconds` field the C++ side doesn't have, since
    the shortcut() binding returns only a changed-or-not bool, not timing) or None if no branch
    resolves both endpoints."""
    start_state = make_pose_array(problem["start_eef_pos"], problem["start_psi"])
    goal_state = make_pose_array(problem["goal_eef_pos"], problem["goal_psi"])

    if use_fixed_order_smm:
        smm = None
        for branch in BRANCH_ORDER:
            if resolve_branch(param, ambient, environment, branch, start_state, goal_state):
                smm = [branch[0], branch[1], 0.0]
                break
        if smm is None:
            return None
    else:
        smm = problem["smm"]
        param.set_smm(smm)
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

    result.path.interpolate_to_resolution(32)  # match the C++ benchmark's output resolution

    shortcut_path = result.path.numpy()
    shortcut_ambient_path = resolve_ambient_path(param, shortcut_path)

    return {
        "solved": True,
        "problem_index": index,
        "start_eef_pos": list(problem["start_eef_pos"]),
        "goal_eef_pos": list(problem["goal_eef_pos"]),
        "smm": list(smm),
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


def write_ambient_path(waypoints: np.ndarray, path: Path) -> None:
    """One waypoint per line, comma-separated joint values -- matches
    fr3_maze_solver_benchmark.cc's write_ambient_path, so scripts/fr3_marker_maze_viser.py
    can load either script's trajectory files the same way."""
    with open(path, "w") as f:
        for waypoint in waypoints:
            f.write(",".join(str(float(v)) for v in waypoint) + "\n")


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
    trajectory_dir: str = str(DEFAULT_TRAJECTORY_DIR),
    limit: int = None,  # Only run the first `limit` problems (for a quick check).
    range_: float = 0.42,  # RRTC range; matches fr3_maze_solver_benchmark.cc.
    max_iterations: int = 100_000,
    max_samples: int = 100_000,
    radius: float = 1.0,  # Matches fr3_maze_solver_benchmark.cc's rrtc_settings.radius.
    use_fixed_order_smm: bool = False,  # Ignore each problem's saved smm and instead sweep
    # BRANCH_ORDER's 4 branches (fixed order, saved psi held fixed) until one resolves both
    # endpoints -- see run_problem's docstring for how this differs from the C++ benchmark's
    # fuller --use_fixed_order_smm (branch order matches, but psi isn't also searched).
):
    environment = load_maze_environment()
    print(f"Loaded {len(environment.cuboids) + len(environment.z_aligned_cuboids)} cuboids from {MAZE_JSON.name}")

    with open(problems) as f:
        problem_list = json.load(f)
    if limit is not None:
        problem_list = problem_list[:limit]
    print(f"Loaded {len(problem_list)} problems from {problems}")

    ambient = vamp.fr3marker
    param = ambient.parameterized_space

    inner = param.halton()
    sampler = param.TaskSpaceInformedSampler(
        EEF_TO_OFFSET, WORLD_TO_REFERENCE, TSR_LOWER, TSR_UPPER, environment, inner)

    settings = vamp.RRTCSettings()
    settings.range = range_
    settings.max_iterations = max_iterations
    settings.max_samples = max_samples
    settings.dynamic_domain = False
    settings.radius = radius

    trajectory_dir = Path(trajectory_dir)
    trajectory_dir.mkdir(parents=True, exist_ok=True)
    print(f"Writing shortcut trajectories to: {trajectory_dir}")

    nanoseconds_per_problem = []
    iterations_per_problem = []
    valid_problems = 0
    solved_problems = 0

    for index, problem in enumerate(problem_list):
        print(f"Planning problem {index + 1} / {len(problem_list)}")
        entry = run_problem(param, ambient, environment, sampler, settings, problem, index,
                             use_fixed_order_smm)
        if entry is None:
            print("  skipping problem: no branch resolved both start and goal."
                  if use_fixed_order_smm else
                  "  skipping problem: start or goal invalid on its saved branch.")
            continue

        valid_problems += 1
        if not entry["solved"]:
            print("  unable to solve problem with start and goal configs.")
            continue

        nanoseconds_per_problem.append(entry["nanoseconds"])
        iterations_per_problem.append(entry["iterations"])
        solved_problems += 1

        shortcut_ambient_path = np.asarray(entry["shortcut_ambient_path"], dtype=np.float32)
        write_ambient_path(shortcut_ambient_path, trajectory_dir / f"problem_{index}.txt")

    print(f"\nSaved {solved_problems} shortcut trajectory files to {trajectory_dir}")
    print_summary(nanoseconds_per_problem, iterations_per_problem, len(problem_list), valid_problems)


if __name__ == "__main__":
    Fire(main)
