"""Python port of scripts/cpp/rby1_task_space_planner.cc: task-space RRTC planning over
RBY1::ParameterizedSpace using the `rby1.parameterized_space` python bindings.

Reads start/goal problems (task-space states, ParameterizedSpace.dimension() = 19 floats
each) from an input json -- either a bare array or a {"problems": [...]} wrapper, each
entry `{"file": <optional string>, "start": [...], "goal": [...]}` -- and writes one
result per problem to an output json, matching the schema
resources/ruby/problem_set_skipped_intermediate_res_no_early_eef.json already uses, so
it's directly comparable to the C++ tool's output.

Everything ParameterizedLocalPlanner does internally to validate a task-space state
(IK-resolve, eef-collision prefilter, RBY1's support-polygon/CoM stability check,
fkcc/fkcc_attach) is deliberately not exposed to python as a single query -- only the
coarser resolve()/eefs_collision_free()/rrtc()/shortcut() entry points are (plus
set_support_polygon(), which configures the stability check's polygon but doesn't expose
running it standalone). So "*_satisfies_initial_conditions" below is a narrower proxy
(resolve + eef-collision prefilter + ambient validate) than the C++ side's
resolve_and_check(), which additionally checks stability; a state passing here but
failing that internal check would simply make RRTC (which does use the real internal
gauntlet, against whatever polygon set_support_polygon() last configured) fail to connect
it, not silently produce an invalid path.

Usage:
    python scripts/rby1_task_space_planner.py
    python scripts/rby1_task_space_planner.py --input_json path/to/problems.json \\
        --output_json /tmp/rby1_results.json
"""

import json
import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

RESOURCES = Path(__file__).parents[1] / "resources"
DEFAULT_INPUT_JSON = RESOURCES / "ruby" / "problem_set_skipped_intermediate.json"
DEFAULT_OUTPUT_JSON = RESOURCES / "ruby" / "problem_set_skipped_intermediate_res_python.json"

# --- compute_mid_pose(): derive the fixed T_mid -> hand offsets from a reference ambient
# configuration. Matches rby1_task_space_planner.cc's home_ambient.
HOME_AMBIENT = [
    0.0, 0.0, 1.0, 0.0,
    -2.16360474e-03, 1.45860954e+00, -1.97468998e+00, 1.55147668e+00, 2.66302773e-01, -5.04128611e-01,
    1.17522024e-01, 9.91603153e-02, 4.46071004e-01, -1.67540527e+00, 3.29611651e-01, 5.74187322e-01, 1.65740047e+00,
    -1.22668093e+00, 3.92945961e-02, 1.20823988e+00, -6.24967729e-01, -1.61156583e-01, 5.79106863e-01, -2.52512556e+00,
]

# --- smoke-test task-space state (unused for planning, just a sanity check).
SMOKE_TEST_STATE = [
    0.0, 0.0, 1.0, 0.0, 0.0207, -0.3034, 0.882, 1.1434, 0.0344, 0.0677,
    0.1354, -0.0944, 0.553, 0.0073, 0.22, 0.0, 0.0, 0.0, 1.0,
]

# ParameterizedSpace::dimension isn't bound to python (no "dimension" def on the
# parameterized_space submodule, unlike the ambient robot's dimension()); this matches
# RBY1::ParameterizedSpace::dimension = 19.
TASK_SPACE_DIMENSION = len(SMOKE_TEST_STATE)

# --- c-space (ambient joint-space) planning problem.
CSPACE_START = [
    0, 0, 1, 0, 0.1682, 0.7809, -1.3941, 0.8259, -0.1562, -0.0383, -0.652034, 0.846282, -0.3154, -1.71205,
    0.955445, 1.80577, 2.12459, -1.16081, -0.694043, 0.9617, -1.63612, -1.14521, 1.62616, -2.67506,
]
CSPACE_GOAL = [
    0, 0, 1, 0, -0.0021636, 1.45861, -1.97469, 1.55148, 0.266303, -0.504129, 0.117516, 0.0991325, 0.4461,
    -1.67542, 0.32963, 0.574213, 1.65735, -1.22666, 0.039279, 1.2082, -0.625015, -0.16113, 0.579141, -2.5251,
]

# --- static-stability support polygon: ground-contact points (right wheel, left wheel,
# left caster, right caster) in the mobile base's local xy frame. This is also
# ParameterizedLocalPlanner's built-in default (see set_support_polygon's docstring);
# kept here explicitly so the script has a concrete polygon to shrink via --inset (see
# inset_polygon below) and pass to set_support_polygon().
DEFAULT_SUPPORT_POLYGON_XY = [
    [0.228000, -0.265000],   # right wheel
    [0.228000, 0.265000],    # left wheel
    [-0.248686, 0.066310],   # left caster
    [-0.248686, -0.066310],  # right caster
]
np.set_printoptions(precision=3, suppress=True)


def inset_polygon(vertices, inset: float):
    """Shrink a convex polygon inward by `inset` meters (0 returns it unchanged; negative
    grows it outward instead). Offsets each edge inward along its normal by `inset`, then
    rebuilds vertices as consecutive offset-edge intersections -- the standard convex-
    polygon erosion construction. `vertices` must be in consistent winding order (either
    direction; the winding is detected from the vertices themselves via the shoelace
    formula, so it doesn't need to match any particular convention). Too large an inset
    can push edges past each other into a self-intersecting result -- this doesn't detect
    or guard against that, so keep it well under the polygon's narrowest half-width."""
    if inset == 0.0:
        return [list(v) for v in vertices]

    pts = np.asarray(vertices, dtype=np.float64)
    n = len(pts)

    signed_area = sum(
        pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
    winding = 1.0 if signed_area > 0 else -1.0  # +1 counterclockwise, -1 clockwise

    def offset_edge(p1, p2):
        edge = p2 - p1
        # Left normal of the edge direction; points into the interior for a
        # counterclockwise polygon, so `winding` flips it for a clockwise one.
        normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
        offset = normal * winding * inset
        return p1 + offset, p2 + offset

    def line_intersection(a1, a2, b1, b2):
        da, db = a2 - a1, b2 - b1
        denom = da[0] * db[1] - da[1] * db[0]
        if abs(denom) < 1e-9:  # parallel edges (degenerate); fall back to the shared point
            return a2
        t = ((b1[0] - a1[0]) * db[1] - (b1[1] - a1[1]) * db[0]) / denom
        return a1 + t * da

    offset_edges = [offset_edge(pts[i], pts[(i + 1) % n]) for i in range(n)]
    return [
        line_intersection(*offset_edges[i - 1], *offset_edges[i]).tolist()
        for i in range(n)
    ]

CONSERVATIVE_PAD_M = 0.02

TABLE_POSITION = [0.32, -0.79, 0.3667]
TABLE_HALF_EXTENTS = [0.60 + CONSERVATIVE_PAD_M, 0.30 + CONSERVATIVE_PAD_M,
                      0.3667 + CONSERVATIVE_PAD_M]

# Matches execute_planned_trajectory.py's SIZE / OFFSET (full box dims, box centre
# in the ee frame), plus CONSERVATIVE_PAD_M grown symmetrically on every face
# (2*pad added to each full-extent dimension; BOX_OFFSET -- the centre -- is
# unchanged, since symmetric growth needs no recentring).
#
# These were previously stale: BOX_SIZE=[0.3086, 0.2064, 0.11] and
# BOX_OFFSET=[-0.193, 0.0, -0.13] were the pre-remeasurement box geometry
# execute_planned_trajectory.py's own comments describe replacing (that file's
# comment on OFFSET literally names "-0.13" as "the previous" value, since
# corrected to -0.166). The stale -0.13 put this file's box 35 mm short of
# reaching as far below the gripper as Drake's model does -- enough on its own to
# let vamp validate a configuration where the box has already penetrated the
# table by ~28 mm per Drake's SceneGraph. The undersized x/y footprint
# (308.6x206.4 mm vs Drake's 386x264 mm) was a second, independent under-
# estimate on top of that.
BOX_SIZE = [0.386 + 2 * CONSERVATIVE_PAD_M, 0.264 + 2 * CONSERVATIVE_PAD_M,
           0.108 + 2 * CONSERVATIVE_PAD_M]
BOX_OFFSET = [-0.193, 0.0, -0.166]



def grid_centers(extent: float, radius: float) -> np.ndarray:
    half = extent / 2.0
    if half <= radius:
        return np.array([0.0])
    n = int(np.ceil(extent / (2.0 * radius))) + 1
    return -half + radius + (2.0 * (half - radius)) * np.arange(n) / (n - 1)


def build_environment() -> vamp.Environment:
    env = vamp.Environment()
    env.add_cuboid(vamp.Cuboid(TABLE_POSITION, [0.0, 0.0, 0.0], TABLE_HALF_EXTENTS))

    box_sphere_radius = BOX_SIZE[2] / 2.0
    attachment_tf = np.identity(4)
    attachment_tf[:3, 3] = BOX_OFFSET
    attachment = vamp.Attachment(attachment_tf, end_effector=1)  # right hand
    attachment.excluded_end_effectors = [0]  # left hand; see attachment.excluded_end_effectors docs

    spheres = [
        vamp.Sphere([cx, cy, 0.0], box_sphere_radius)
        for cx in grid_centers(BOX_SIZE[0], box_sphere_radius)
        for cy in grid_centers(BOX_SIZE[1], box_sphere_radius)
    ]
    attachment.add_spheres(spheres)
    env.attach(attachment)

    return env


# def resolve_and_report(param, state: np.ndarray, label: str, environment: vamp.Environment, check_env_cc=False):
#     """Mirrors resolve_and_report: IK-resolve a task-space state, optionally reporting the
#     eef-collision prefilter separately. Returns (valid, ambient_config or None)."""
#     # print(state, state.shape)
#     if check_env_cc:
#         print(f"{label} checking eef collision prefilter...")
#         eef_free = param.eefs_collision_free(state, environment)
#         print(f"{label} eefs in collision: {not eef_free}")
#         if not eef_free:
#             print(f"{label} eefs in collision: True")

#     print(f"{label} resolving task-space state... : ", ",".join(f"{x:.3f}" for x in state))
#     valid, ambient_config = param.resolve(state)
#     print(f"{label} resolve_block: valid={valid}, ambient_config={ambient_config}")
#     if not valid:
#         print(f"{label} resolve_block failed; invalid task-space state.")
#         return False, None
#     return True, np.asarray(ambient_config, dtype=np.float32)

def resolve_and_report(param, state: np.ndarray, label: str, environment: vamp.Environment, check_env_cc=False):
    """Mirrors resolve_and_report: IK-resolve a task-space state, optionally reporting the
    eef-collision prefilter separately. Returns (valid, ambient_config or None)."""
    # print(state, state.shape)
    if check_env_cc:
        print(f"{label} checking eef collision prefilter...")
        eef_free = param.eefs_collision_free(state, environment)
        print(f"{label} eefs in collision: {not eef_free}")
        if not eef_free:
            print(f"{label} eefs in collision: True")

    print(f"{label} resolving task-space state... : ", ",".join(f"{x:.3f}" for x in state))
    valid, ambient_config = param.resolve(state)
    print(f"{label} resolve_block: valid={valid}, ambient_config={ambient_config}")
    if not valid:
        print(f"{label} resolve_block failed; invalid task-space state.")
        return False, None
    return True, np.asarray(ambient_config, dtype=np.float32)



def check_collision_free(ambient, ambient_config: np.ndarray, label: str, environment: vamp.Environment) -> bool:
    collision_free = ambient.validate(ambient_config, environment)
    if not collision_free:
        print(f"{label} resolved configuration is in collision.")
    return collision_free


def satisfies_initial_conditions(param, ambient, state: np.ndarray, ambient_config, environment) -> bool:
    """Narrower proxy for TaskLocalPlanner::resolve_and_check(); see module docstring."""
    if ambient_config is None:
        return False
    return (
        param.eefs_collision_free(state, environment)
        and ambient.validate(ambient_config, environment)
    )


def run_cspace_smoke_test(ambient):
    print("\n--- C-space planning problem ---")
    start = np.asarray(CSPACE_START, dtype=np.float32)
    goal = np.asarray(CSPACE_GOAL, dtype=np.float32)
    environment = vamp.Environment()  # matches the C++ side's cspace check (no obstacles set up yet)

    if not (check_collision_free(ambient, start, "C-space start", environment) and
            check_collision_free(ambient, goal, "C-space goal", environment)):
        print("C-space start or goal is in collision; skipping c-space RRTC.")
        return

    sampler = ambient.halton()
    settings = vamp.RRTCSettings()
    result = ambient.rrtc(start, goal, environment, settings, sampler)

    print("\n--- C-space RRTC result ---")
    print(f"solved: {result.solved}")
    print(f"cost: {result.path.cost() if result.solved else float('nan')}")
    print(f"iterations: {result.iterations}")
    print(f"nanoseconds: {result.nanoseconds}")
    print(f"tree sizes (start, goal): {result.size[0]}, {result.size[1]}")
    print(f"path size: {len(result.path)}")


def run_cspace_planner(ambient, environment, start_ambient: np.ndarray, goal_ambient: np.ndarray) -> dict:
    """Plan directly in ambient joint space between two already-resolved configurations
    (e.g. a task-space problem's resolved start/goal) -- same idea as
    run_cspace_smoke_test, but for real problem-specific endpoints and the problem's own
    (populated) environment instead of a fixed hardcoded pair over an empty one."""
    sampler = ambient.halton()
    settings = vamp.RRTCSettings()

    t0 = time.perf_counter()
    result = ambient.rrtc(start_ambient, goal_ambient, environment, settings, sampler)
    elapsed_ns = int((time.perf_counter() - t0) * 1e9)

    entry = {
        "solved": result.solved,
        "iterations": result.iterations,
        "nanoseconds": result.nanoseconds,
        "python_wall_nanoseconds": elapsed_ns,
        "tree_size_start": result.size[0],
        "tree_size_goal": result.size[1],
        "path_size": len(result.path),
    }
    if not result.solved:
        return entry

    entry["cost"] = result.path.cost()

    # Unlike the task-space parameterized_space.shortcut() binding, the ambient robot's
    # simplify() returns a *new* PlanningResult rather than mutating result.path in place,
    # so the raw path is left untouched here -- no need to snapshot it first.
    simp_settings = vamp.SimplifySettings()
    simp_settings.operations = [vamp.SimplifyRoutine.SHORTCUT]
    shortcut_result = ambient.simplify(result.path, environment, simp_settings, sampler)

    entry["shortcut_path_size"] = len(shortcut_result.path)
    entry["shortcut_cost"] = shortcut_result.path.cost()

    # Densify both paths to Robot::resolution before writing them out, so the written
    # trajectories are at a fixed step resolution rather than just their own waypoints.
    result.path.interpolate_to_resolution(ambient.resolution())
    shortcut_result.path.interpolate_to_resolution(ambient.resolution())
    entry["raw_interpolated_path_size"] = len(result.path)
    entry["shortcut_interpolated_path_size"] = len(shortcut_result.path)

    entry["raw_trajectory"] = result.path.numpy().tolist()
    entry["shortcut_trajectory"] = shortcut_result.path.numpy().tolist()

    return entry


def load_problems(input_json: str):
    with open(input_json) as f:
        data = json.load(f)
    return data if isinstance(data, list) else data["problems"]


def run_problem(param, ambient, environment, dimension: int, problem: dict, index: int) -> dict:
    result_entry = {"index": index}
    file_field = problem.get("file", "")
    if file_field:
        result_entry["file"] = file_field

    start_vec = problem["start"]
    goal_vec = problem["goal"]
    if len(start_vec) != dimension or len(goal_vec) != dimension:
        print(f"Problem {index}: start/goal must have {dimension} elements; skipping.")
        result_entry["solved"] = False
        result_entry["error"] = "start or goal has the wrong number of elements"
        return result_entry

    start_state = np.asarray(start_vec, dtype=np.float32)
    goal_state = np.asarray(goal_vec, dtype=np.float32)

    # param.set_gcp(left=(-1, -1, 1), right=(-1, -1, 1))  # left/right hand: gcp index 0 (palm), 1 (thumb), 2 (fingers)

    start_valid, start_ambient = resolve_and_report(param, start_state, "Start", environment, check_env_cc=True)
    goal_valid, goal_ambient = resolve_and_report(param, goal_state, "Goal", environment, check_env_cc=True)
    result_entry["start_valid"] = start_valid
    result_entry["goal_valid"] = goal_valid

    if not (start_valid and goal_valid):
        print("Start or goal did not resolve through IK; skipping RRTC.")
        result_entry["solved"] = False
        result_entry["error"] = "start or goal did not resolve through IK"
        return result_entry

    start_collision_free = check_collision_free(ambient, start_ambient, "Start", environment)
    goal_collision_free = check_collision_free(ambient, goal_ambient, "Goal", environment)
    result_entry["start_collision_free"] = start_collision_free
    result_entry["goal_collision_free"] = goal_collision_free

    if not (start_collision_free and goal_collision_free):
        print("Start or goal resolves to a colliding ambient configuration; skipping RRTC.")
        result_entry["solved"] = False
        result_entry["error"] = "start or goal resolves to a colliding ambient configuration"
        return result_entry

    start_ok = satisfies_initial_conditions(param, ambient, start_state, start_ambient, environment)
    goal_ok = satisfies_initial_conditions(param, ambient, goal_state, goal_ambient, environment)
    result_entry["start_satisfies_initial_conditions"] = start_ok
    result_entry["goal_satisfies_initial_conditions"] = goal_ok

    if not (start_ok and goal_ok):
        print("Start or goal fails the initial-condition proxy checks; skipping RRTC.")
        result_entry["solved"] = False
        result_entry["error"] = "start or goal fails the initial-condition proxy checks"
        return result_entry

    # Sampler: FixedBaseSampler wraps a plain Halton<Robot, ParameterizedSpace> and
    # restricts it to this problem -- base fixed at the origin, t_mid_pose position
    # confined to +/-1.25 around the start pose, everything else unrestricted.
    inner = param.halton()
    start_position = [float(start_state[12]), float(start_state[13]), float(start_state[14])]
    sampler = param.FixedBaseSampler(inner, start_position, 1.25)

    settings = vamp.RRTCSettings()
    settings.range = 0.3
    settings.max_iterations = 1_000_000

    t0 = time.perf_counter()
    result = param.rrtc(start_state, goal_state, environment, settings, sampler)
    elapsed_ns = int((time.perf_counter() - t0) * 1e9)

    print(f"solved: {result.solved}")
    print(f"iterations: {result.iterations}")
    print(f"milliseconds: {result.nanoseconds / 1e6:.3f}")
    print(f"tree sizes (start, goal): {result.size[0]}, {result.size[1]}")

    result_entry["solved"] = result.solved
    result_entry["cost"] = result.path.cost() if result.solved else None
    result_entry["iterations"] = result.iterations
    result_entry["nanoseconds"] = result.nanoseconds
    result_entry["python_wall_nanoseconds"] = elapsed_ns
    result_entry["tree_size_start"] = result.size[0]
    result_entry["tree_size_goal"] = result.size[1]
    result_entry["path_size"] = len(result.path)

    if not result.solved:
        return result_entry

    # Snapshot the raw (pre-shortcut) path into its own Path object before shortcut()
    # mutates result.path in place below -- this is the only way to keep both versions,
    # since shortcut() has no non-mutating form.
    raw_path = param.Path()
    for state in result.path.numpy():
        raw_path.append(state)

    # --- Shortcut the resolved task-space path. shortcut() mutates result.path in place;
    # the same IK-resolve/eef-collision/stability/fkcc gauntlet RRTC used revalidates
    # every collapsed edge, so shortcutting can only remove waypoints, never bypass a check.
    cost_before_shortcut = result.path.cost()
    param.shortcut(result.path, environment)

    result_entry["shortcut_path_size"] = len(result.path)
    result_entry["shortcut_cost_before"] = cost_before_shortcut
    result_entry["shortcut_cost_after"] = result.path.cost()

    # Densify both paths to Robot::resolution before writing them out, so the written
    # trajectories are at a fixed step resolution rather than just their own waypoints.
    raw_path.interpolate_to_resolution(ambient.resolution())
    result.path.interpolate_to_resolution(ambient.resolution())
    result_entry["raw_interpolated_path_size"] = len(raw_path)
    result_entry["interpolated_path_size"] = len(result.path)

    def resolve_trajectory(path):
        trajectory = []
        for state in path.numpy():
            valid, ambient_config = param.resolve(state)
            trajectory.append({
                "resolved": valid,
                "ambient_configuration": np.asarray(ambient_config, dtype=np.float32).tolist(),
            })
        return trajectory

    result_entry["raw_trajectory"] = resolve_trajectory(raw_path)
    result_entry["trajectory"] = resolve_trajectory(result.path)

    # Also plan directly in ambient joint space between this problem's already-resolved
    # start/goal, as a point of comparison against the task-space plan above.
    result_entry["cspace"] = run_cspace_planner(ambient, environment, start_ambient, goal_ambient)

    return result_entry


def main(
    input_json: str = str(DEFAULT_INPUT_JSON),
    output_json: str = str(DEFAULT_OUTPUT_JSON),
    inset: float = 0.0,  # Shrink the default support polygon inward by this many meters.
):
    ambient = vamp.rby1
    param = ambient.parameterized_space

    print(f"Robot.dimension (ambient/joint space): {ambient.dimension()}")
    print(f"ParameterizedSpace.dimension (task space): {TASK_SPACE_DIMENSION}")

    param.compute_mid_pose(np.asarray(HOME_AMBIENT, dtype=np.float32))

    # Thread-local, like compute_mid_pose's t_mid_left/t_mid_right: takes effect for every
    # rrtc()/shortcut() call on this thread from here on, until set again.
    polygon_xy = inset_polygon(DEFAULT_SUPPORT_POLYGON_XY, inset)
    param.set_support_polygon(polygon_xy)
    print(f"Support polygon (base-local xy, inset {inset}): {polygon_xy}")

    environment = build_environment()
    print(f"Environment has {len(environment.cuboids) + len(environment.z_aligned_cuboids)} cuboids, "
          f"{len(environment.spheres)} spheres, and {len(environment.attachments)} attachments.")

    print("\n--- Smoke test: resolve() + validate() ---")
    smoke_state = np.asarray(SMOKE_TEST_STATE, dtype=np.float32)
    smoke_valid, smoke_ambient = resolve_and_report(param, smoke_state, "Smoke test", environment)
    if smoke_valid:
        check_collision_free(ambient, smoke_ambient, "Smoke test", environment)

    run_cspace_smoke_test(ambient)

    problems = load_problems(input_json)

    results = []
    for index, problem in enumerate(problems):
        label = problem.get("file", "")
        print(f"\n=== Planning problem {index}{' (' + label + ')' if label else ''} ===")
        results.append(run_problem(param, ambient, environment, TASK_SPACE_DIMENSION, problem, index))

    with open(output_json, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nWrote {len(results)} planning result(s) to {output_json}")


# def test_resolve_block():
#     ambient = vamp.rby1
#     param = ambient.parameterized_space
#     param.compute_mid_pose(np.asarray(HOME_AMBIENT, dtype=np.float32))
#     environment = build_environment()


if __name__ == "__main__":
    Fire(main)
