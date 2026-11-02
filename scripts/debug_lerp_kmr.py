"""Debug the VAMP curve planner on the kmriiwa (mobile base + iiwa), like
debug_lerp_example.py does for the fixed-base iiwa.

Shows in viser: the robot (collision spheres) at the start configuration with
joint sliders, the 32-waypoint reference as frames, and the planned path.
Prints whether the start is valid in VAMP's sphere model (and which link pairs
collide), plans the whole segment, and sweeps a few planner settings.

Why it exists: on the kmriiwa, vamp_curve_rrtc gets ~0.4 first-segment success
and vamp_tsr_ik ~0.99. 208 of the 320 starts (65%) are in self-collision in the
sphere model (almost all iiwa_link_1 against kmriiwa_base_link, overlaps up to
19 mm; some gripper fingers against the base, up to 89 mm), where Drake's hulls
do not touch, and the planner cannot leave an invalid root. On the valid starts
it solves 0.98 at L=4. This script shows that for one start at a time.
"""
import sys
import time

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf
import xml.etree.ElementTree as ET

sys.path.insert(0, "/src/diffik-benchmark/src")
from diffik_benchmark.methods.vamp_curve_rrtc import (  # noqa: E402
    VampCurveRRTCParams, pad_keyframes, refine_path,
)
from diffik_benchmark.trajectories import Episode  # noqa: E402

# (start_id, L, stride).
PROBLEMS = [
    # kmriiwa_base_link vs the left finger's tip sphere (sphere 69, r 35 mm) in
    # VAMP's sphere model; the base spheres are big (r 19-26 cm). Overlap in mm
    # (my numpy check), all 18 starts fail at L=4 in the planner and tsr passes:
    (58, 4, 6),   # 83
    (9, 4, 6),    # 63
    (157, 4, 6),  # 56
    (93, 4, 6),   # 55
    (89, 4, 6),   # 45
    (292, 4, 6),  # 41
    (165, 4, 6),  # 36
    (118, 4, 6),  # 29
    (25, 4, 6),   # 28
    (201, 4, 6),  # 26 (passes: the reference barely moves, held q0 is accepted)
    (8, 4, 6),    # 26
    (244, 4, 6),  # 23
    (57, 4, 6),   # 12
    (137, 4, 6),  # 9
    (277, 4, 6),  # 5
    (168, 4, 6),  # 5
    (312, 4, 6),  # 4
    (74, 4, 6),   # 2.5 (planner solves it)
    # Valid starts where the planner uses the whole 20000-iteration budget and
    # tsr passes:
    (144, 4, 6),
    (37, 16, 6), (148, 16, 6), (150, 16, 6), (184, 16, 6), (186, 16, 6),
    (218, 16, 6), (280, 16, 6), (281, 16, 6),
    (16, 32, 6), (18, 32, 6), (42, 32, 6), (53, 32, 6), (85, 32, 6),
    # iiwa_link_1 vs kmriiwa_base_link (pair now disabled in the cricket SRDF;
    # these should validate after the rebuild): 1, 4, 5, 7 at L=4.
    (1, 4, 6), (4, 4, 6), (5, 4, 6), (7, 4, 6),
]
problem_index = 0
START_ID, L, STRIDE = PROBLEMS[problem_index]
# START_ID, L, STRIDE = 1, 4, 6

# Planner settings come from the benchmark method's defaults; override a line to
# experiment.
P = VampCurveRRTCParams()
ROT_WEIGHT = P.rotation_weight
TOL = P.tolerance
PROJ_METHOD = P.proj_method
RANGE = P.range
MAX_ITERATIONS = P.max_iterations
MAX_SAMPLES = P.max_samples
MAX_GREEDY_STEPS = P.max_greedy_steps
PARAMETER_WEIGHT = P.parameter_weight
PROJECT_SAMPLES = P.project_samples
N_TRIALS = 5
SWEEP_WEIGHTS = [0.1, 0.3, 1.0]
SWEEP_SAMPLES = [20_000, 200_000]
PREFIX_SWEEP = False  # plan on the first k waypoints, k = 2 .. L

DATA = "/src/diffik-benchmark/data"
URDF = "/src/diffik-benchmark/models/kmriiwa/external/kmriiwa_spherized.urdf"
# Sphere order of the generated header = the order of the <collision> spheres in
# the spherized URDF cricket was given; used only to name colliding spheres.
SPHERE_URDF = URDF
# Slider ranges where the URDF gives none (base x, y are limited to +-2 m in
# VAMP, theta is unbounded).
BASE_RANGES = {"kmr_base/x": (-2.0, 2.0), "kmr_base/y": (-2.0, 2.0),
               "kmr_base/theta": (-2 * np.pi, 2 * np.pi)}

# --- data ------------------------------------------------------------------
starts = np.load(f"{DATA}/starts/kmriiwa/cup_in_the_wild.npz")
q0 = starts["q"][START_ID]  # x, y, theta, iiwa_joint_1..7
episode = int(starts["episode_index"][START_ID])
start_index = int(starts["start_index"][START_ID])
X_W_ref = starts["X_W_ref"][START_ID]
print(f"start {START_ID}: episode {episode}, start index {start_index}, L={L}, "
      f"stride {STRIDE}\nq0 = {np.round(q0, 4).tolist()}")

# The segment exactly as the benchmark hands it to the method.
bench = Episode.load(
    f"{DATA}/trajectories/cup_in_the_wild/ep_{episode:05d}.npz"
).segment(start_index, L, STRIDE).transformed(X_W_ref)
pos, quat = bench.pos[:, 0].copy(), bench.quat[:, 0].copy()

# --- viser -------------------------------------------------------------------
server = viser.ViserServer()
server.scene.add_grid("/grid", width=4.0, height=4.0, plane="xy")
server.scene.add_frame("/world", axes_length=0.1, axes_radius=0.003)
urdf = yourdfpy.URDF.load(URDF, load_collision_meshes=True,
                          build_collision_scene_graph=True)
robot = ViserUrdf(server, urdf, load_meshes=False, load_collision_meshes=True,
                  root_node_name="/robot")
joint_names = robot.get_actuated_joint_names()
print("URDF actuated joints:", joint_names)
assert len(joint_names) == len(q0), "joint count differs from the starts file"
robot.update_cfg(q0)

limits = robot.get_actuated_joint_limits()
q_lo = np.zeros(len(joint_names))
q_hi = np.zeros(len(joint_names))
for i, n in enumerate(joint_names):
    lo, hi = limits.get(n, (None, None))
    if n in BASE_RANGES or lo is None or hi is None or not np.isfinite([lo, hi]).all():
        lo, hi = BASE_RANGES.get(n, (-np.pi, np.pi))
    q_lo[i], q_hi[i] = lo, hi

sliders = []
with server.gui.add_folder("Joints"):
    for n, q_i, lo, hi in zip(joint_names, q0, q_lo, q_hi):
        sliders.append(server.gui.add_slider(
            n, min=float(lo), max=float(hi), step=1e-3,
            initial_value=float(np.clip(q_i, lo, hi))))
    reset = server.gui.add_button("Reset to q0")


highlight_hook = None  # set once VAMP is loaded: marks the colliding spheres


def apply_sliders(_=None):
    q = np.array([s.value for s in sliders])
    robot.update_cfg(q)
    if highlight_hook is not None:
        highlight_hook(q)


def set_sliders(q):
    for s_, q_i in zip(sliders, q):
        s_.value = float(np.clip(q_i, s_.min, s_.max))
    apply_sliders()


for s in sliders:
    s.on_update(apply_sliders)


@reset.on_click
def _(_):
    set_sliders(q0)


server.scene.add_line_segments("/path", points=np.stack([pos[:-1], pos[1:]], axis=1),
                               colors=(120, 120, 120), line_width=2.0)
for i, (p, q) in enumerate(zip(pos, quat)):
    server.scene.add_frame(f"/waypoints/{i:02d}", wxyz=q, position=p,
                           axes_length=0.04, axes_radius=0.002, origin_radius=0.004)
    server.scene.add_label(f"/waypoints/{i:02d}/label", text=str(i))

# --- planner ----------------------------------------------------------------
import vamp  # noqa: E402

module = vamp.kmriiwa_tool
print("vamp joints:", list(module.joint_names()))
assert module.dimension() == len(q0)
n_points = int(module.lerp_points())


def matrix_to_wxyz(R):
    t = np.trace(R)
    if t > 0:
        r = np.sqrt(1.0 + t)
        w = 0.5 * r
        x, y, z = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5 / r
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


# Start the curve at the tool's actual pose (the benchmark's start_at_actual).
urdf.update_cfg(q0)
T0 = urdf.get_transform("wsg/tool", "world")
plan_pos, plan_quat = pos.copy(), quat.copy()
plan_pos[0] = T0[:3, 3]
plan_quat[0] = matrix_to_wxyz(T0[:3, :3])
if np.dot(plan_quat[0], quat[1]) < 0:
    plan_quat[0] = -plan_quat[0]
print(f"curve start moved by {np.linalg.norm(plan_pos[0] - pos[0]) * 1e3:.2f} mm from waypoint 0")
server.scene.add_frame("/start_actual", wxyz=plan_quat[0], position=plan_pos[0],
                       axes_length=0.06, axes_radius=0.003, origin_radius=0.006)

env = vamp.Environment()


def make_settings(method_name=PROJ_METHOD):
    st = vamp.ConstraintSettings()
    st.method = getattr(vamp.ProjMethod, method_name)
    st.tolerance = TOL
    st.max_iterations = P.proj_iterations
    return st


start_settings = vamp.ConstraintSettings()
start_settings.tolerance = P.start_tolerance
start_settings.max_iterations = P.start_iterations
pose0 = [float(v) for v in (*plan_quat[0], *plan_pos[0])]
region = module.TaskSpaceConstraint([[1, 0, 0, 0, 0, 0, 0]], [pose0], [[0.0] * 6], [[0.0] * 6])
q_start = np.array(module.project(np.asarray(q0, dtype=np.float32), [region], start_settings))
print("start moved by", np.abs(q_start - q0).max(), "rad (max over the 10 joints)")
q_start32 = q_start.astype(np.float32)

# --- Is the start valid in VAMP's sphere model? ------------------------------
# An invalid root makes the planner reject every move at once (20000 iterations
# in ~50 ms). This is the main suspect on the kmriiwa.
sphere_links = []
sphere_geom = []  # (link, offset in the link frame, radius), in sphere order
for link in ET.parse(SPHERE_URDF).getroot().findall("link"):
    for col in link.findall("collision"):
        org = col.find("origin")
        xyz = np.array([float(x) for x in
                        (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
        radius = float(col.find("geometry").find("sphere").get("radius"))
        sphere_links.append(link.get("name"))
        sphere_geom.append((link.get("name"), xyz, radius))
valid = module.validate(q_start32, env, True)
print(f"VAMP validate(q_start): {valid}")
env_hits, self_pairs = module.debug(q_start32, env)
if self_pairs:
    named = sorted({tuple(sorted((sphere_links[a], sphere_links[b]))) for a, b in self_pairs})
    print("  colliding sphere pairs (index -> link, assuming the URDF's sphere order):")
    for a, b in self_pairs:
        print(f"    sphere {a} ({sphere_links[a]}) <-> sphere {b} ({sphere_links[b]})")
    print("  link pairs:", named)
if any(env_hits):
    print("  environment hits:", [i for i, h in enumerate(env_hits) if h])

# Highlight the colliding spheres (VAMP's own check) in red, for the current
# sliders; the text box lists the link pairs.
highlights = []
with server.gui.add_folder("Collisions"):
    collision_text = server.gui.add_text("VAMP self-collision", "", disabled=True)


def update_highlights(q):
    for h in highlights:
        h.remove()
    highlights.clear()
    urdf.update_cfg(np.asarray(q, float))
    _, pairs = module.debug(np.asarray(q, dtype=np.float32), env)
    names = set()
    for i in sorted({i for pr in pairs for i in pr}):
        link, off, rad = sphere_geom[i]
        T = urdf.get_transform(link, "world")
        highlights.append(server.scene.add_icosphere(
            f"/collision/sphere_{i}", radius=rad, color=(230, 40, 40), opacity=0.4,
            position=tuple(T[:3, :3] @ off + T[:3, 3])))
    for a_, b_ in pairs:
        names.add(" <-> ".join(sorted((sphere_links[a_].split("/")[-1],
                                       sphere_links[b_].split("/")[-1]))))
    collision_text.value = "; ".join(sorted(names)) or "none"


highlight_hook = update_highlights
update_highlights(q_start)

manifold = module.LerpSO3R3Constraint(
    *[x.tolist() for x in pad_keyframes(plan_pos, plan_quat, n_points, ROT_WEIGHT)[:2]],
    ROT_WEIGHT, False)


def plan_with(weight=ROT_WEIGHT, max_samples=MAX_SAMPLES, k=None, method_name=PROJ_METHOD):
    kp, kq, _ = pad_keyframes(plan_pos[:k], plan_quat[:k], n_points, weight)
    m = module.LerpSO3R3Constraint(kp.tolist(), kq.tolist(), weight, False)
    pl = vamp.TaskSpaceRRTCSettings()
    pl.range, pl.max_greedy_steps = RANGE, MAX_GREEDY_STEPS
    pl.max_iterations, pl.max_samples = MAX_ITERATIONS, max_samples
    pl.project_samples, pl.parameter_weight = PROJECT_SAMPLES, PARAMETER_WEIGHT
    return module.task_space_rrtc_curve(q_start32, env, pl, module.halton(), [m],
                                        make_settings(method_name), start_t=0.0)


def report(tag, r):
    print(f"{tag} solved={r.solved} iterations={r.iterations} ms={r.nanoseconds / 1e6:.1f}")


print(f"\nplanner: {PROJ_METHOD}, rotation_weight={ROT_WEIGHT}, range={RANGE}, "
      f"max_iterations={MAX_ITERATIONS}, max_samples={MAX_SAMPLES}, tolerance={TOL}")
if SWEEP_WEIGHTS and SWEEP_SAMPLES:
    print("sweep:")
    for w in SWEEP_WEIGHTS:
        for ms in SWEEP_SAMPLES:
            try:
                report(f"  rotation_weight={w:<4} max_samples={ms:<7}", plan_with(w, ms))
            except (ValueError, RuntimeError) as e:
                print(f"  rotation_weight={w:<4} max_samples={ms:<7} error: {e}")
if PREFIX_SWEEP:
    print(f"prefix sweep (first k of {L} waypoints):")
    for k in range(2, L + 1):
        try:
            report(f"  k={k:<3}", plan_with(k=k))
        except (ValueError, RuntimeError) as e:
            print(f"  k={k:<3} error: {e}")

# Projection probe: from q_start, project onto the curve at fixed t.
print(f"projection probe ({PROJ_METHOD}): t, converged, err, max |dq| (base x/y m, theta/joints rad)")
for t_probe in (0.0, 1e-3, 1e-2, 0.05, 0.1, 0.25, 0.5, 1.0):
    qv, tv, err = manifold.projection_trail(
        q_start32, float(t_probe), getattr(vamp.ProjMethod, PROJ_METHOD), TOL,
        P.proj_iterations, 1.0)[-1]
    print(f"  t={t_probe:<6} converged={err <= TOL!s:<5} err={err:.2e} "
          f"max|dq|={np.abs(np.asarray(qv, float) - q_start).max():.4f}")

results = [plan_with() for _ in range(N_TRIALS)]
print(f"\n{N_TRIALS} trials with the benchmark settings: {sum(r.solved for r in results)} solved; "
      f"iterations {[int(r.iterations) for r in results]}")
result = next((r for r in results if r.solved), results[0])

path_q = None
if result.solved:
    states = np.array([np.asarray(q, float) for q in result.path])
    ts = np.asarray(list(result.t), float)

    def project(q, t):
        qv, tv, err = manifold.projection_trail(
            np.asarray(q, dtype=np.float32), float(t),
            getattr(vamp.ProjMethod, PROJ_METHOD), TOL, P.proj_iterations, 1.0)[-1]
        return (np.asarray(qv, float), float(tv)) if err <= TOL else None

    path_q, path_t, n_bad = refine_path(states, ts, project, P.max_knot_step)
    path_q = np.vstack([q0, path_q])
    print(f"{len(states)} planner states -> {len(path_q)} knots ({n_bad} unprojected), "
          f"t {path_t[0]:.3f} -> {path_t[-1]:.3f}; base moves "
          f"{np.linalg.norm(path_q[-1, :2] - path_q[0, :2]):.3f} m, turns "
          f"{path_q[-1, 2] - path_q[0, 2]:+.3f} rad")

    def tool_position(q):
        urdf.update_cfg(np.asarray(q, float))
        return urdf.get_transform("wsg/tool", "world")[:3, 3]

    trace = np.array([tool_position(q) for q in path_q])
    urdf.update_cfg(q0)
    server.scene.add_line_segments("/planned/ee_trace",
                                   points=np.stack([trace[:-1], trace[1:]], axis=1),
                                   colors=(40, 200, 80), line_width=3.0)
    with server.gui.add_folder("Plan"):
        step = server.gui.add_slider("path knot", min=0, max=len(path_q) - 1, step=1,
                                     initial_value=0)

    @step.on_update
    def _(_):
        set_sliders(path_q[int(step.value)])

print("\nviser running, Ctrl-C to quit")
while True:
    time.sleep(1.0)
