"""Multi-step handrail climbing for R2C6, after the mmpx handrail_climbing2 problems:
the robot traverses all four handrails with an alternating gait, one single-mode plan
per step, until both feet grasp the last rail. In each step the stance foot stays
grasped on its rail and the torso is held upright (yaw free) while the swing foot
moves to its target rail; the step's goal is found by projecting seeds onto the mode
intersected with a grasp TSR on the target rail. Random floating sphere obstacles
fill the space the legs swing through.

Run:
    python scripts/r2_handrail_example.py [--planner rrtc|aorrtc] [--n_spheres 8]
                                          [--obstacle_seed 0] [--visualize]
"""

import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire
from scipy.spatial.transform import Rotation

# Scene: mmpx handrail_climbing2 problem_02_scene.yml. Rails run along world Y, spaced
# 0.7 m in X; the grasp line (rail axis) sits 0.12 m above the rail frame origin.
RAIL_X = [-0.4585, -1.1585, -1.85795, -2.5585]
RAIL_POSE_T = [0.0, -0.505, -1.223]
RAIL_QUAT_XYZW = [0.0, 0.0, 0.7071068, 0.7071068]
GRASP_OFFSET_T = [0.505, 0.0, 0.12]
GRASP_OFFSET_RPY = [0.0, 3.14, 1.57]
GRASP_Z = RAIL_POSE_T[2] + GRASP_OFFSET_T[2]
ALONG_RAIL = 0.49  # grasp line half-length; along-rail is the grasp frame's y axis
RAIL_RADIUS = 0.03
RAIL_HALF_LENGTH = 0.505
FLOOR_CENTER = [-1.5, 0.0, -1.3]
FLOOR_HALF_EXTENTS = [2.5, 1.5, 0.05]

START_RAIL = 0  # right foot holds this rail at the start; left foot starts free

# Random floating obstacles: sampled in the volume the body and swing leg move
# through, but kept out of a clearance corridor around every grasp line so each rail
# stays graspable and the feet can pass.
SPHERE_LOW = [-2.7, -0.7, -1.0]
SPHERE_HIGH = [0.2, 0.7, 0.3]
SPHERE_RADIUS_RANGE = (0.06, 0.12)
RAIL_CLEARANCE = 0.25

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
WIDE = [100.0] * 3 + [3.2] * 3  # unconstrained: |se(3) log| never exceeds these
# mmpx LineMode grasp tolerances: position pinned (5 mm slack along the gripper
# approach), orientation pinned.
PIN = [1e-4, 1e-4, 5e-3, 1e-2, 1e-2, 1e-2]
# Goal grasp: same, but free along the rail (grasp frame y).
GOAL_PIN = [1e-4, ALONG_RAIL, 5e-3, 1e-2, 1e-2, 1e-2]
# mmpx FixedOrientationMode: torso roll/pitch held, yaw half-free, position free.
WAIST_BOUND = [100.0] * 3 + [0.05, 0.05, 0.5]
WAIST_REFERENCE_RPY = [0.0, -3.14, 0.0]

FOOT_TIP = {"left": "r2/left_leg/gripper/tip", "right": "r2/right_leg/gripper/tip"}
EEF_INDEX = {"left": 0, "right": 1}  # constraint EEF order: left tip, right tip, waist
LEG_SLICE = {"left": slice(7, 14), "right": slice(14, 21)}

# Free-flyer q layout: x y z qx qy qz qw (dims 0-6), then left leg (7), right leg (7),
# waist (1), left arm (7), right arm (7). The URDF floating joint carries a +1.2 m z
# offset, so a base of (0, 0, -1.2) puts the robot at the mmpx world origin. Arms are
# tucked less than in mmpx to clear the conservative spherization.
START = np.array(
    [0, 0, -1.2, 0, 0, 1, 0]
    + [0, 0, 0, 1.6708, 0, 1.6708, 1.6708]
    + [0, 0, 0, 1.5708, 0, 1.5708, 1.5708]
    + [0]
    + [1.272665, -0.99626, -1.8326, -2.44346, 1.39626, 0, 0]
    + [-1.272665, -0.99626, 1.8326, -2.44346, -1.39626, 0, 0],
    dtype=np.float32,
)

BASE_Z_OFFSET = 1.2
ROBOT_DIR = Path(__file__).parents[1] / "resources" / "r2c6"
ROBOT_URDF = "r2c6_minimal_spherized.urdf"


def matrix_to_transform(mat):
    """4x4 matrix -> (qw qx qy qz x y z), the constraint transform layout."""
    q = Rotation.from_matrix(mat[:3, :3]).as_quat()
    return [q[3], q[0], q[1], q[2], *mat[:3, 3]]


def grasp_frame(rail_index):
    rail = np.eye(4)
    rail[:3, :3] = Rotation.from_quat(RAIL_QUAT_XYZW).as_matrix()
    rail[:3, 3] = RAIL_POSE_T
    rail[0, 3] += RAIL_X[rail_index]

    offset = np.eye(4)
    offset[:3, :3] = Rotation.from_euler("xyz", GRASP_OFFSET_RPY).as_matrix()
    offset[:3, 3] = GRASP_OFFSET_T
    return rail @ offset


class ForwardKinematics:
    """World-frame link poses for the free-flyer robot via the fixed-base URDF."""

    def __init__(self, joint_names):
        import yourdfpy

        self.urdf = yourdfpy.URDF.load(
            str(ROBOT_DIR / ROBOT_URDF), load_meshes=False,
            build_collision_scene_graph=False)
        names = list(joint_names)[7:]
        self.remap = [names.index(n) for n in self.urdf.actuated_joint_names]

    def base_pose(self, q):
        base = np.eye(4)
        base[:3, :3] = Rotation.from_quat(q[3:7]).as_matrix()
        base[:3, 3] = q[0:3]
        base[2, 3] += BASE_Z_OFFSET
        return base

    def link_pose(self, q, link):
        self.urdf.update_cfg(np.asarray(q)[7:][self.remap])
        return self.base_pose(q) @ self.urdf.get_transform(link, self.urdf.base_link)


def clear_of_rails(center, radius):
    if abs(center[1]) > RAIL_HALF_LENGTH + 0.1:
        return True

    return all(
        np.hypot(center[0] - x, center[2] - GRASP_Z) > radius + RAIL_CLEARANCE
        for x in RAIL_X)


def sample_spheres(module, n, rng):
    """Random floating obstacles clear of the grasp corridors and of the robot's
    stance regions: the scene is periodic along x, so the start pose translated to
    each rail stands in for the body volume the gait must occupy there."""
    stances = []
    for x in RAIL_X:
        stance = START.copy()
        stance[0] += x - RAIL_X[START_RAIL]
        stances.append(stance)

    spheres = []
    for _ in range(200 * n):
        center = rng.uniform(SPHERE_LOW, SPHERE_HIGH)
        radius = rng.uniform(*SPHERE_RADIUS_RANGE)
        if not clear_of_rails(center, radius):
            continue

        e = vamp.Environment()
        e.add_sphere(vamp.Sphere(center, radius))
        if all(module.validate(stance, e) for stance in stances):
            spheres.append((center, radius))
            if len(spheres) == n:
                return spheres

    raise RuntimeError("could not sample obstacles that keep the stance regions free")


def build_environment(spheres, exclude_rails):
    e = vamp.Environment()
    e.add_cuboid(vamp.Cuboid(FLOOR_CENTER, [0.0, 0.0, 0.0], FLOOR_HALF_EXTENTS))
    for i, x in enumerate(RAIL_X):
        if i in exclude_rails:  # a foot touches these rails during the step
            continue

        e.add_capsule(vamp.Cylinder(
            [x, -RAIL_HALF_LENGTH, GRASP_Z], [x, RAIL_HALF_LENGTH, GRASP_Z], RAIL_RADIUS))

    for center, radius in spheres:
        e.add_sphere(vamp.Sphere(center, radius))

    return e


def make_constraint(module, references, bounds):
    """References/bounds indexed by EEF: left tip, right tip, waist."""
    return module.TaskSpaceConstraint(
        [IDENTITY] * 3,
        [matrix_to_transform(r) if isinstance(r, np.ndarray) else r for r in references],
        [[-b for b in bound] for bound in bounds],
        list(bounds),
    )


def plan_step(
    module, planner_func, plan_settings, simp_settings, fk,
    config, pinned, moving, target_rail, e,
    constraint_settings, goal_settings, seed_attempts, plan_rounds, rng,
):
    """Plan one gait step: `pinned` foot stays grasped where it is, `moving` foot
    swings to `target_rail`. Returns the simplified path (ends at the step goal)."""
    waist_reference = np.eye(4)
    waist_reference[:3, :3] = Rotation.from_euler("xyz", WAIST_REFERENCE_RPY).as_matrix()

    pinned_pose = fk.link_pose(config, FOOT_TIP[pinned])
    references = [IDENTITY, IDENTITY, waist_reference]
    bounds = [WIDE, WIDE, WAIST_BOUND]
    references[EEF_INDEX[pinned]] = pinned_pose
    bounds[EEF_INDEX[pinned]] = PIN
    constraints = [make_constraint(module, references, bounds)]

    start = np.array(module.project(config, constraints, constraint_settings), dtype=np.float32)
    if not module.validate(start, e):
        raise RuntimeError("projected step start is in collision")

    # Goal manifold: the mode plus a grasp TSR on the target rail. The reference
    # orientation is the stance foot's grasp orientation (the rails are parallel, and
    # both grippers hold a rail the same way); along-rail stays free.
    target_grasp = pinned_pose.copy()
    target_grasp[:3, 3] = grasp_frame(target_rail)[:3, 3]
    goal_references = list(references)
    goal_bounds = list(bounds)
    goal_references[EEF_INDEX[moving]] = target_grasp
    goal_bounds[EEF_INDEX[moving]] = GOAL_PIN
    goal_constraints = [make_constraint(module, goal_references, goal_bounds)]

    # Random-restart projection: shift the base partway along the swing and perturb
    # the swing leg until seeds project onto the goal manifold collision-free. The
    # long two-rail swings are hard to connect, so gather several distinct goal
    # grasps and let the planner try them all; on failure, add more goals and replan
    # with the sampler continuing where it left off.
    moving_x = fk.link_pose(start, FOOT_TIP[moving])[0, 3]
    goals = []
    attempt = 0

    def top_up_goals():
        nonlocal attempt
        for _ in range(seed_attempts):
            seed = start.copy()
            seed[0] += 0.5 * (RAIL_X[target_rail] - moving_x)
            if attempt > 0:
                seed[0:3] += rng.normal(0.0, 0.1, size=3).astype(np.float32)
                # Spread seeds along the rail so the projected grasps cover the
                # goal TSR's free along-rail interval.
                seed[1] += rng.uniform(-0.35, 0.35)
                seed[LEG_SLICE[moving]] += rng.normal(0.0, 0.3, size=7).astype(np.float32)
                seed[3:7] /= np.linalg.norm(seed[3:7])

            attempt += 1
            try:
                candidate = np.array(
                    module.project(seed, goal_constraints, goal_settings), dtype=np.float32)
            except ValueError:
                continue

            if module.validate(candidate, e) and \
                    module.satisfied(candidate, constraints, constraint_settings):
                goals.append(candidate)

    sampler = module.halton()
    # --- per-query self-collision pair partition (m71), guarded by R2_PRUNE ---
    import os as _os
    _prune = {"active": None, "total": None, "ms": 0.0}
    if _os.environ.get("R2_PRUNE"):
        top_up_goals()
        if goals:
            import sys as _sys, time as _t
            _pp = str(Path(__file__).parents[1] / "experiments" / "jit" / "partition")
            if _pp not in _sys.path:
                _sys.path.insert(0, _pp)
            import selfpair_partition as _sp
            if not hasattr(_sp, "_R2C6_CACHE"):
                _sp._R2C6_CACHE = _sp.load_compact_self()
            _ent, _pa, _pb = _sp._R2C6_CACHE
            _cfgs = [np.array(start, dtype=np.float64)] + [np.array(g, dtype=np.float64) for g in goals]
            _t0 = _t.perf_counter()
            _active, _ = _sp.compute_active_self_pairs(module, _cfgs, _ent, _pa, _pb, len(start))
            _prune["ms"] = (_t.perf_counter() - _t0) * 1e3
            e.active_self_pairs = _active
            _prune["active"], _prune["total"] = len(_active), len(_ent)
    result = None
    elapsed = time.perf_counter()
    for _ in range(plan_rounds):
        top_up_goals()
        if not goals:
            continue

        result = planner_func(
            start, goals, e, plan_settings, sampler,
            constraints=constraints, constraint_settings=constraint_settings)
        if result.solved:
            break

    elapsed = time.perf_counter() - elapsed
    if not goals:
        raise RuntimeError(f"no valid goal grasp found on rail {target_rail}")

    if result is None or not result.solved:
        raise RuntimeError(f"planning failed for {moving} foot to rail {target_rail}")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)

    path = simple.path
    if not all(module.satisfied(path[i], constraints, constraint_settings)
               for i in range(len(path))):
        raise RuntimeError("simplified path leaves the mode manifold")

    # --- correctness verification (R2_VERIFY): re-validate the path under FULL self-collision ---
    if _os.environ.get("R2_VERIFY") and _prune["active"] is not None:
        _saved = list(e.active_self_pairs)
        e.active_self_pairs = []  # all self-pairs active == baseline collision semantics
        _fn = sum(0 if module.validate(np.array(path[i], dtype=np.float32), e) else 1
                  for i in range(len(path)))
        e.active_self_pairs = _saved
        _pr = 100.0 * (_prune["total"] - _prune["active"]) / _prune["total"]
        if _fn:
            print(f"  !!! CORRECTNESS FAIL: {_fn}/{len(path)} path states collide under FULL check "
                  f"(pruned {_pr:.0f}% pairs)")
        else:
            print(f"  CORRECTNESS OK: path valid under full self-collision; pruned "
                  f"{_prune['total'] - _prune['active']}/{_prune['total']} pairs ({_pr:.0f}%), "
                  f"partition {_prune['ms']:.1f}ms")

    drift = max(
        np.linalg.norm(fk.link_pose(np.array(path[i]), FOOT_TIP[pinned])[:3, 3]
                       - pinned_pose[:3, 3])
        for i in range(len(path)))
    landing = fk.link_pose(np.array(path[len(path) - 1]), FOOT_TIP[moving])[:3, 3]

    print(f"step: {moving} foot -> rail {target_rail}: solved in {elapsed * 1e3:.1f} ms "
          f"({len(goals)} goal grasps), {len(result.path)} -> {len(path)} states, "
          f"cost {path.cost():.4f}")
    print(f"  pinned {pinned} foot max drift {drift * 1e3:.2f} mm; "
          f"{moving} foot lands at {np.round(landing, 4)} "
          f"(rail x = {RAIL_X[target_rail]}, grasp z = {GRASP_Z})")

    return np.array([np.array(path[i]) for i in range(len(path))], dtype=np.float32)


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 2.0,  # Planner range; constrained steps should stay small.
    n_spheres: int = 8,
    obstacle_seed: int = 0,
    seed_attempts: int = 30,
    plan_rounds: int = 5,
    visualize: bool = False,
    coupled: bool = False,  # Coupled Gauss-Newton projection step.
    **kwargs,
):
    kwargs.setdefault("max_iterations", 10000)
    (module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("r2c6", planner, **kwargs)

    # Dynamic-domain's default radius (4) is far below typical sample-to-tree
    # distances in this robot's raw-unit configuration space, which silently skips
    # nearly every iteration.
    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc_settings.range = range_
    rrtc_settings.dynamic_domain = False

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = 50
    constraint_settings.coupled = coupled
    goal_settings = vamp.ConstraintSettings()
    goal_settings.max_iterations = 200
    goal_settings.coupled = coupled

    fk = ForwardKinematics(module.joint_names())
    rng = np.random.default_rng(obstacle_seed)
    spheres = sample_spheres(module, n_spheres, rng)

    # Alternating gait: the trailing foot swings two rails ahead (one rail on the
    # first step, where the left foot starts free beside the right foot's rail).
    # The last rail is targeted twice so the trailing foot joins the leading one
    # there and the robot finishes with both feet on the final rail.
    last = len(RAIL_X) - 1
    targets = list(range(START_RAIL + 1, len(RAIL_X))) + [last]
    feet_on = {"right": START_RAIL, "left": None}
    config = START
    waypoints = [START[np.newaxis, :]]
    for target_rail in targets:
        moving = "left" if feet_on["left"] is None or \
            feet_on["left"] < feet_on["right"] else "right"
        pinned = "right" if moving == "left" else "left"

        # Rails a foot touches during this step leave the collision environment: the
        # stance rail, the swing foot's source rail, and its target rail.
        exclude = {feet_on[pinned], feet_on[moving], target_rail} - {None}
        e = build_environment(spheres, exclude)

        path = plan_step(
            module, planner_func, plan_settings, simp_settings, fk,
            config, pinned, moving, target_rail, e,
            constraint_settings, goal_settings, seed_attempts, plan_rounds, rng)

        waypoints.append(path[1:] if np.allclose(path[0], config) else path)
        config = path[-1]
        feet_on[moving] = target_rail

    trajectory = np.vstack(waypoints)
    print(f"gait complete: {len(targets)} steps, "
          f"{len(trajectory)} total states; final stance "
          f"left = rail {feet_on['left']}, right = rail {feet_on['right']}")

    if visualize:
        from viser_utils import add_spheres, setup_viser_with_robot

        server, robot = setup_viser_with_robot(ROBOT_DIR, ROBOT_URDF)
        base_frame = server.scene.add_frame("/robot", show_axes=False)

        def show(q):
            base = fk.base_pose(q)
            base_frame.position = base[:3, 3]
            quat = Rotation.from_matrix(base[:3, :3]).as_quat()
            base_frame.wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
            robot.update_cfg(np.asarray(q)[7:][fk.remap])

        server.scene.add_box(
            "/environment/floor",
            color=(120, 120, 120),
            dimensions=tuple(2 * h for h in FLOOR_HALF_EXTENTS),
            position=np.array(FLOOR_CENTER),
        )
        for i, x in enumerate(RAIL_X):
            server.scene.add_box(
                f"/environment/rail_{i}",
                color=(150, 150, 160),
                dimensions=(2 * RAIL_RADIUS, 2 * RAIL_HALF_LENGTH, 2 * RAIL_RADIUS),
                position=np.array([x, 0.0, GRASP_Z]),
            )

        add_spheres(
            server,
            [center for center, _ in spheres],
            [radius for _, radius in spheres],
            colors=[[255, 140, 0]],
            prefix="/environment/obstacle",
        )

        show(trajectory[0])
        slider = server.gui.add_slider(
            "Current Waypoint", min=0, max=len(trajectory) - 1, step=1, initial_value=0)
        autoplay = server.gui.add_checkbox("Autoplay", initial_value=True)
        rate = server.gui.add_slider(
            "Playback Rate (Hz)", min=1.0, max=60.0, step=1.0, initial_value=20.0)

        @slider.on_update
        def _(event):
            show(trajectory[int(event.target.value)])

        print("visualization at http://localhost:8080; ctrl-c to exit")
        while True:
            if autoplay.value:
                slider.value = (slider.value + 1) % len(trajectory)
            time.sleep(1.0 / rate.value)


if __name__ == "__main__":
    Fire(main)
