import sys
import time

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

# Start 44 (episode 4, start index 101) at L=16: vamp_tsr_ik and Drake both pass
# it, vamp_curve_rrtc's planner returns unsolved after ~2100 iterations with
# the 20000-sample tree full (rotation_weight 0.3, 3<->5 sphere pair disabled
# in the SRDF), so it is neither a tube failure nor an invalid start.
# Similar starts (tsr and Drake pass, tree full at ~1400-2800 iterations):
#   L=16: 28, 40, 232      L=32: 4, 60, 123, 251
# Start 63 (still invalid: iiwa_link_5 vs wsg/body spheres) fails at 20000
# iterations at L=8/16/32; start 191 (the old 3<->5 case) now passes.


PROBLEMS = [
    (63, 8, 6),    # 20001 iterations (whole budget), drake passes; still an invalid start (link 5 vs wsg/body)
    (234, 8, 6),   # 3644 iterations (tree full), drake fails
    (28, 16, 6),   # 2503 iterations (tree full), drake passes
    (40, 16, 6),   # 2522 iterations (tree full), drake passes
    (44, 16, 6),   # 2139 iterations (tree full), drake passes   <- done
    (63, 16, 6),   # 20001 iterations (whole budget), drake passes
    (86, 16, 6),   # 2691 iterations (tree full), drake fails
    (232, 16, 6),  # 2502 iterations (tree full), drake passes
    (4, 32, 6),    # 2583 iterations (tree full), drake passes
    (60, 32, 6),   # 2502 iterations (tree full), drake passes
    (63, 32, 6),   # 20001 iterations (whole budget), drake passes
    (123, 32, 6),  # 2829 iterations (tree full), drake passes
    (138, 32, 6),  # 1440 iterations (tree full), drake fails
    (248, 32, 6),  # 2588 iterations (tree full), drake fails
    (251, 32, 6),  # 1782 iterations (tree full), drake passes
]

problem_index = 5
START_ID, L, STRIDE = PROBLEMS[problem_index]

# START_ID = 44
# L, STRIDE = 16, 6

# Planner settings come from the benchmark method's own defaults, so this
# script and vamp_curve_rrtc cannot drift apart; override a line to experiment
# (e.g. MAX_ITERATIONS = MAX_SAMPLES = 2_000_000).
sys.path.insert(0, "/src/diffik-benchmark/src")
from diffik_benchmark.methods.vamp_curve_rrtc import (  # noqa: E402
    VampCurveRRTCParams, pad_keyframes, refine_path,
)

P = VampCurveRRTCParams()
ROT_WEIGHT = 0.3  # the benchmark default is P.rotation_weight
# TOL = P.tolerance
TOL = 1e-5

PROJ_METHOD = 'InnerLM'
RANGE = P.range
MAX_ITERATIONS = P.max_iterations
MAX_SAMPLES = 20_000  # the benchmark default is P.max_samples
MAX_GREEDY_STEPS = P.max_greedy_steps
PARAMETER_WEIGHT = P.parameter_weight
PROJECT_SAMPLES = P.project_samples
USE_BENCHMARK_SEGMENT = False  # plan on the benchmark-built segment instead
N_TRIALS = 20  # planner runs on the same input

# One planner run per (rotation weight, max_samples) pair, everything else as
# above, printed before the trials. Empty a list to skip the sweep.
SWEEP_WEIGHTS = [0.1, 0.3, 0.5, 1.0]
SWEEP_SAMPLES = [20_000, 200_000]
SWEEP_METHODS = ["InnerLM", "BoundedLS"]  # projection methods in the sweep
# > 0: before planning, move q_start away from the joint limits (to at least
# this margin, rad) along the null space of the start pose (the arm has one
# redundant DOF), so the start pose is unchanged. Tests whether a start pinned
# on a joint limit (25% of the benchmark's iiwa starts are) is what stops the
# planner. 0 = use q_start as the benchmark does.
START_INWARD_MARGIN = 0.0
# Plan on the first k waypoints of the segment (k = 2 .. L) from the same
# start: the smallest k the planner fails at localizes the part of the curve it
# cannot get through. The benchmark's L=4 and L=8 results for the same start
# say what to expect for small k.
PREFIX_SWEEP = True
URDF = "/src/diffik-benchmark/models/iiwa/external/iiwa_spherized.urdf"
starts = np.load("/src/diffik-benchmark/data/starts/iiwa/cup_in_the_wild.npz")
q0 = starts["q"][START_ID]
episode = int(starts["episode_index"][START_ID])
start_index = int(starts["start_index"][START_ID])

d = np.load(
    f"/src/diffik-benchmark/data/trajectories/cup_in_the_wild/ep_{episode:05d}.npz")
idx = start_index + STRIDE * np.arange(L)
pos  = d["ee_pos"][idx, 0]        # (L, 3)
quat = d["ee_quat_wxyz"][idx, 0]  # (L, 4), wxyz





# d = np.load("/src/diffik-benchmark/data/trajectories/cup_in_the_wild/ep_00000.npz")
# idx = 0 + 6 * np.arange(32)
# pos  = d["ee_pos"][idx, 0]        # (32, 3)
# quat = d["ee_quat_wxyz"][idx, 0]  # (32, 4), wxyz

# print(pos, quat)

# The same segment through the benchmark's own pipeline (what vamp_curve_rrtc
# is handed): any difference from the numbers above would explain a different
# planner outcome.
sys.path.insert(0, "/src/diffik-benchmark/src")
from diffik_benchmark.trajectories import Episode  # noqa: E402

bench = Episode.load(
    f"/src/diffik-benchmark/data/trajectories/cup_in_the_wild/ep_{episode:05d}.npz"
).segment(start_index, L, STRIDE).transformed(starts["X_W_ref"][START_ID])
dq = np.minimum(np.linalg.norm(bench.quat[:, 0] - quat, axis=1),
                np.linalg.norm(bench.quat[:, 0] + quat, axis=1))
print("benchmark segment vs this script: max |dpos| =",
      np.abs(bench.pos[:, 0] - pos).max(), "m, max |dquat| (up to sign) =",
      dq.max(), "| quaternion signs differ at",
      np.flatnonzero(np.sum(bench.quat[:, 0] * quat, axis=1) < 0).tolist())
if USE_BENCHMARK_SEGMENT:
    pos, quat = bench.pos[:, 0].copy(), bench.quat[:, 0].copy()

# START_ID = 2
# URDF = "/src/diffik-benchmark/models/iiwa/external/iiwa_spherized.urdf"
# starts = np.load("/src/diffik-benchmark/data/starts/iiwa/cup_in_the_wild.npz")
# q0 = starts["q"][START_ID]
# print("start", START_ID, "episode", starts["episode_index"][START_ID],
#       "start_index", starts["start_index"][START_ID], "q0", q0)




print("start", START_ID, "episode", starts["episode_index"][START_ID],
      "start_index", starts["start_index"][START_ID], "q0", q0)

# The trajectory is in the dataset (tag) frame; X_W_ref (p, wxyz) puts it in
# the robot's world frame. The single iiwa has no tool offset (X_tool_ref None).
def quat_mul(a, b):
    aw, av = a[..., :1], a[..., 1:]
    bw, bv = b[..., :1], b[..., 1:]
    return np.concatenate(
        [aw * bw - (av * bv).sum(-1, keepdims=True),
         aw * bv + bw * av + np.cross(av, bv)], -1)


def quat_rotate(q, v):
    qv = np.broadcast_to(q[1:], v.shape)
    t = 2.0 * np.cross(qv, v)
    return v + q[0] * t + np.cross(qv, t)


pW, qW = starts["X_W_ref"][START_ID][:3], starts["X_W_ref"][START_ID][3:]
pos = pW + quat_rotate(qW, pos)
quat = quat_mul(np.broadcast_to(qW, quat.shape), quat)
print(pos, quat)

server = viser.ViserServer()
server.scene.add_grid("/grid", width=2.0, height=2.0, plane="xy")
server.scene.add_frame("/world", axes_length=0.1, axes_radius=0.003)

# The spherized URDF has collision spheres only, so draw those.
urdf = yourdfpy.URDF.load(
    URDF,
    load_collision_meshes=True,
    build_collision_scene_graph=True,
)
robot = ViserUrdf(
    server,
    urdf,
    load_meshes=False,
    load_collision_meshes=True,
    root_node_name="/robot",
)
robot.update_cfg(q0)

# One slider per actuated joint (update_cfg takes them in this order).
joint_names = robot.get_actuated_joint_names()
limits = robot.get_actuated_joint_limits()
sliders = []
with server.gui.add_folder("Joints"):
    for name, q_i in zip(joint_names, q0):
        lo, hi = limits[name]
        sliders.append(server.gui.add_slider(
            name, min=float(lo), max=float(hi), step=1e-3,
            initial_value=float(np.clip(q_i, lo, hi)),
        ))
    reset = server.gui.add_button("Reset to q0")


def apply_sliders(_=None):
    robot.update_cfg(np.array([s.value for s in sliders]))


for s in sliders:
    s.on_update(apply_sliders)


@reset.on_click
def _(_):
    for s, q_i in zip(sliders, q0):
        s.value = float(q_i)
    apply_sliders()


# Straight lines between consecutive waypoints (the lerp part of the curve).
server.scene.add_line_segments(
    "/path",
    points=np.stack([pos[:-1], pos[1:]], axis=1),
    colors=(120, 120, 120),
    line_width=2.0,
)

for i, (p, q) in enumerate(zip(pos, quat)):
    server.scene.add_frame(
        f"/waypoints/{i:02d}",
        wxyz=q,
        position=p,
        axes_length=0.04,
        axes_radius=0.002,
        origin_radius=0.004,
    )
    server.scene.add_label(f"/waypoints/{i:02d}/label", text=str(i))


# ---------------------------------------------------------------------------
# Plan the whole segment on the lerp/slerp curve with the VAMP curve RRTC, the
# same steps as the benchmark's vamp_curve_rrtc (pad to 32 keyframes, project
# q0 onto waypoint 0, task_space_rrtc_curve, refine the chords).
# ---------------------------------------------------------------------------
import vamp  # noqa: E402

module = vamp.iiwa_tool
print("vamp joints", module.joint_names())
n_points = int(module.lerp_points())


def matrix_to_wxyz(R):
    """Quaternion (w, x, y, z) of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        r = np.sqrt(1.0 + t)
        w, x, y, z = 0.5 * r, *((np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                                           R[1, 0] - R[0, 1]])) * 0.5 / r)
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        v = np.zeros(3)
        v[i] = 0.5 * r
        v[j] = (R[j, i] + R[i, j]) * 0.5 / r
        v[k] = (R[k, i] + R[i, k]) * 0.5 / r
        w = (R[k, j] - R[j, k]) * 0.5 / r
        x, y, z = v
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# q0 is an IK solution accurate only to the start generator's tolerance, so
# waypoint 0 is not exactly where the arm is (3.5 mm / 0.02 rad), and
# sometimes not reachable exactly within the joint limits. Start the curve
# at the tool's actual pose instead; the rest of the curve is unchanged.
urdf.update_cfg(q0)
T0 = urdf.get_transform("wsg/tool", "world")
plan_pos, plan_quat = pos.copy(), quat.copy()
plan_pos[0] = T0[:3, 3]
plan_quat[0] = matrix_to_wxyz(T0[:3, :3])
if np.dot(plan_quat[0], quat[1]) < 0:
    plan_quat[0] = -plan_quat[0]
print("curve start moved by", np.linalg.norm(plan_pos[0] - pos[0]), "m")
server.scene.add_frame(
    "/start_actual", wxyz=plan_quat[0], position=plan_pos[0],
    axes_length=0.06, axes_radius=0.003, origin_radius=0.006)

kpos, kquat, key = pad_keyframes(plan_pos, plan_quat, n_points, ROT_WEIGHT)

env = vamp.Environment()
settings = vamp.ConstraintSettings()
settings.method = getattr(vamp.ProjMethod, PROJ_METHOD)
settings.tolerance = TOL
settings.max_iterations = P.proj_iterations
start_settings = vamp.ConstraintSettings()
start_settings.tolerance = P.start_tolerance
start_settings.max_iterations = P.start_iterations
plan = vamp.TaskSpaceRRTCSettings()
plan.range = RANGE
plan.max_greedy_steps = MAX_GREEDY_STEPS
plan.max_iterations = MAX_ITERATIONS
plan.max_samples = MAX_SAMPLES
plan.project_samples = PROJECT_SAMPLES
plan.parameter_weight = PARAMETER_WEIGHT

pose0 = [float(v) for v in (*plan_quat[0], *plan_pos[0])]  # wxyz, xyz
region = module.TaskSpaceConstraint(
    [[1, 0, 0, 0, 0, 0, 0]], [pose0], [[0.0] * 6], [[0.0] * 6])
q_start = np.array(module.project(
    np.asarray(q0, dtype=np.float32), [region], start_settings))
print("start moved by", np.abs(q_start - q0).max(), "rad")

q_lo = np.array([limits[n][0] for n in joint_names])
q_hi = np.array([limits[n][1] for n in joint_names])
if START_INWARD_MARGIN > 0:
    margin_before = np.minimum(q_start - q_lo, q_hi - q_start)
    q_try = np.clip(q_start, q_lo + START_INWARD_MARGIN, q_hi - START_INWARD_MARGIN)
    try:
        q_moved = np.array(module.project(
            q_try.astype(np.float32), [region], start_settings))
        print(f"START_INWARD_MARGIN={START_INWARD_MARGIN}: start moved "
              f"{np.abs(q_moved - q_start).max():.4f} rad inward along the start pose's "
              f"null space; min margin {margin_before.min():.4f} -> "
              f"{np.minimum(q_moved - q_lo, q_hi - q_moved).min():.4f} rad")
        q_start = q_moved
    except ValueError as e:
        print("could not move the start inward (the projection did not converge):", e)

manifold = module.LerpSO3R3Constraint(
    kpos.tolist(), kquat.tolist(), ROT_WEIGHT, False)
def make_settings(method_name):
    st = vamp.ConstraintSettings()
    st.method = getattr(vamp.ProjMethod, method_name)
    st.tolerance = TOL
    st.max_iterations = P.proj_iterations
    return st


def run_with(weight, max_samples, method_name=PROJ_METHOD):
    """One planner run with a given rotation weight, sample budget and
    projection method."""
    kp, kq, _ = pad_keyframes(plan_pos, plan_quat, n_points, weight)
    m = module.LerpSO3R3Constraint(kp.tolist(), kq.tolist(), weight, False)
    pl = vamp.TaskSpaceRRTCSettings()
    pl.range, pl.max_greedy_steps = RANGE, MAX_GREEDY_STEPS
    pl.max_iterations, pl.max_samples = MAX_ITERATIONS, max_samples
    pl.project_samples, pl.parameter_weight = PROJECT_SAMPLES, PARAMETER_WEIGHT
    return module.task_space_rrtc_curve(
        q_start.astype(np.float32), env, pl, module.halton(), [m],
        make_settings(method_name), start_t=0.0)


if SWEEP_WEIGHTS and SWEEP_SAMPLES and SWEEP_METHODS:
    print(f"sweep (max_iterations={MAX_ITERATIONS}):")
    for meth in SWEEP_METHODS:
        for w in SWEEP_WEIGHTS:
            for ms in SWEEP_SAMPLES:
                tag = f"  {meth:<9} rotation_weight={w:<4} max_samples={ms:<7}"
                try:
                    r = run_with(w, ms, meth)
                    print(f"{tag} solved={r.solved} iterations={r.iterations} "
                          f"ms={r.nanoseconds / 1e6:.1f}")
                except (ValueError, RuntimeError) as e:
                    print(f"{tag} error: {e}")


def run_prefix(k, max_samples=None):
    """One planner run on the first k waypoints (padded to the kernel size)."""
    kp, kq, _ = pad_keyframes(plan_pos[:k], plan_quat[:k], n_points, ROT_WEIGHT)
    m = module.LerpSO3R3Constraint(kp.tolist(), kq.tolist(), ROT_WEIGHT, False)
    pl = vamp.TaskSpaceRRTCSettings()
    pl.range, pl.max_greedy_steps = RANGE, MAX_GREEDY_STEPS
    pl.max_iterations, pl.max_samples = MAX_ITERATIONS, max_samples or MAX_SAMPLES
    pl.project_samples, pl.parameter_weight = PROJECT_SAMPLES, PARAMETER_WEIGHT
    return module.task_space_rrtc_curve(
        q_start.astype(np.float32), env, pl, module.halton(), [m],
        make_settings(PROJ_METHOD), start_t=0.0)


if PREFIX_SWEEP:
    print(f"prefix sweep (first k of {L} waypoints, max_samples={MAX_SAMPLES}):")
    for k in range(2, L + 1):
        try:
            r = run_prefix(k)
            print(f"  k={k:<3} solved={r.solved} iterations={r.iterations} "
                  f"ms={r.nanoseconds / 1e6:.1f}")
        except (ValueError, RuntimeError) as e:
            print(f"  k={k:<3} error: {e}")


# Probe: can the planner's basic move work from q_start? Project q_start onto
# the curve at fixed t for growing t (what each extension does), with both
# projection methods, and report convergence, the joint step and any joint
# that ends within 1e-3 rad of a limit.
print("q_start margin to the nearest limit (rad):",
      np.round(np.minimum(q_start - q_lo, q_hi - q_start), 6).tolist())
for method_name in ("InnerLM", "BoundedLS"):
    print(f"probe, {method_name}: t, converged, max |dq| (rad), joints within 1e-3 of a limit")
    for t_probe in (0.0, 1e-3, 1e-2, 0.05, 0.1, 0.25, 0.5, 1.0):
        trail = manifold.projection_trail(
            q_start.astype(np.float32), float(t_probe),
            getattr(vamp.ProjMethod, method_name), TOL, P.proj_iterations, 1.0)
        qv, tv, err = trail[-1]
        qv = np.asarray(qv, float)
        near = (np.flatnonzero(np.minimum(qv - q_lo, q_hi - qv) < 1e-3) + 1).tolist()
        print(f"  t={t_probe:<6} converged={err <= TOL!s:<5} err={err:.2e} "
              f"max|dq|={np.abs(qv - q_start).max():.4f} near-limit joints={near}")


# Probe 2: is the start (and the end of the curve) valid in VAMP's own model?
# The planner rejects every move from an invalid root at once, which is what
# 20000 iterations in 52 ms looks like. The benchmark's starts were generated
# against Drake's collision geometry (convex hulls), VAMP uses the spheres.
q_start32 = q_start.astype(np.float32)
print("VAMP validate(q_start), no bounds check:", module.validate(q_start32, env, False))
print("VAMP validate(q_start), bounds checked: ", module.validate(q_start32, env, True))
print("VAMP debug(q_start):", module.debug(q_start32, env))
trail_end = manifold.projection_trail(
    q_start32, 1.0, getattr(vamp.ProjMethod, PROJ_METHOD), TOL, P.proj_iterations, 1.0)
q_end = np.asarray(trail_end[-1][0], dtype=np.float32)
print("VAMP validate(q at t=1):", module.validate(q_end, env, True))
print("VAMP validate_motion(q_start -> q at t=1):",
      module.validate_motion(q_start32, q_end, env, True))


def plan_once():
    return module.task_space_rrtc_curve(
        q_start.astype(np.float32), env, plan, module.halton(), [manifold],
        settings, start_t=0.0)


# Same input, fresh sampler each time: is the outcome the same every time?
results = [plan_once() for _ in range(N_TRIALS)]
print(f"start {START_ID} L={L} {PROJ_METHOD} rotation_weight={ROT_WEIGHT} "
      f"range={RANGE} max_iterations={MAX_ITERATIONS} max_samples={MAX_SAMPLES}")
print(f"{N_TRIALS} trials: {sum(r.solved for r in results)} solved")
print("  solved:    ", [(int(r.iterations), round(r.nanoseconds / 1e6, 1))
                        for r in results if r.solved], "(iterations, ms)")
print("  not solved:", [(int(r.iterations), round(r.nanoseconds / 1e6, 1))
                        for r in results if not r.solved])
result = next((r for r in results if r.solved), results[0])
print("showing a", "solved" if result.solved else "unsolved", "trial:",
      "iterations", result.iterations, "ms", result.nanoseconds / 1e6)

path_q = None
if result.solved:
    states = np.array([np.asarray(q, float) for q in result.path])
    ts = np.asarray(list(result.t), float)

    def project(q, t):
        trail = manifold.projection_trail(
            np.asarray(q, dtype=np.float32), float(t),
            getattr(vamp.ProjMethod, PROJ_METHOD), TOL, 50, 1.0)
        qv, tv, err = trail[-1]
        return (np.asarray(qv, float), float(tv)) if err <= TOL else None

    path_q, path_t, n_bad = refine_path(states, ts, project, 0.05)
    path_q = np.vstack([q0, path_q])  # q0 first, as the benchmark does
    print(f"{len(states)} planner states -> {len(path_q)} knots "
          f"({n_bad} unprojected), t {path_t[0]:.3f} -> {path_t[-1]:.3f}")

    # End-effector trace of the planned path, by forward kinematics.
    def tool_position(q):
        urdf.update_cfg(np.asarray(q, float))
        return urdf.get_transform("wsg/tool", "world")[:3, 3]

    trace = np.array([tool_position(q) for q in path_q])
    urdf.update_cfg(q0)
    server.scene.add_line_segments(
        "/planned/ee_trace",
        points=np.stack([trace[:-1], trace[1:]], axis=1),
        colors=(40, 200, 80),
        line_width=3.0,
    )
    with server.gui.add_folder("Plan"):
        step = server.gui.add_slider(
            "path knot", min=0, max=len(path_q) - 1, step=1, initial_value=0)

    @step.on_update
    def _(_):
        q = path_q[int(step.value)]
        for s_, q_i in zip(sliders, q):
            s_.value = float(np.clip(q_i, s_.min, s_.max))
        apply_sliders()

print("viser running, Ctrl-C to quit")
while True:
    time.sleep(1.0)
