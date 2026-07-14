"""Fruit ninja demonstration on the Panda's chart-LQMT (flask) planner.

An endless stream of spheres is thrown on ballistic arcs toward the Panda in
rapid fire, each leaving the thrower's hand while the arm is still swinging at
the previous one. The arm must intersect each fruit at its designed hit time
with a prescribed slicing velocity (the fastest swipe the arm can swing at that
posture) to cut it. Planning runs continuously: while the arm executes the
current cut, the next throw is sampled and its interception leg is planned.

Each throw is fixed at sample time; the planner has to match its flight:

- Cut states: for each fruit an ambient TSR projection places the end-effector at
  the nominal hit point (orientation free), the slice direction is chosen over
  the full unit sphere to maximize the end-effector speed achievable within
  SLICE_HEADROOM of the joint velocity limits at that posture (the fruit's own
  velocity is deliberately ignored: the swipe is as aggressive as the arm can
  swing, which favors tangential sweeps over the nearly-radial weak axis of the
  position Jacobian), and damped least-squares differential IK converts the
  slice vector into goal joint velocities.
- Timed interception: each cut-to-cut leg is planned with the flask RRT-Connect
  and simplified, then the time/effort trade-off rho of the LQMT cost
  C = rho T + integral |u|^2 is bisected until the leg's total optimal time
  equals the flight-time gap to sub-millisecond residual. rho only reshapes the
  timing of the arcs; the cut states' boundary velocities are untouched.
- Fruits are targets only, not collision objects, but the workspace holds static
  sphere obstacles: every cut-to-cut leg is planned and simplified against them,
  and sampled throws whose flight would pass through an obstacle are redrawn.

A cut lands if the executed end-effector position at the executed hit time is
inside the fruit and the slice-direction speed meets the prescription. Position
miss comes from the TSR box (corner up to ~9 mm) plus timing residual times fruit
speed (~5 mm/ms); both are printed honestly.
"""

from pathlib import Path
import multiprocessing
import queue
import signal
import time

import numpy as np
import vamp
from vamp import flask as vf
from vamp import transformations as tr
from fire import Fire

GRAVITY = np.array([0.0, 0.0, -9.81])

FRUIT_RADIUS = 0.06

# Fraction of the joint velocity limits the slicing joint velocity may use. The
# slice direction is chosen at each hit posture to maximize the end-effector
# speed achievable within it, so the prescribed speed is per-fruit. 0.9 is the
# ceiling: the arcs into and out of the cut wind a near-limit boundary velocity
# up and back down, and their interior Hermite overshoot already touches the
# limits (executed joint ratio up to 1.00) — any higher executes past them.
SLICE_HEADROOM = 0.9

# Throw stream: one thrower standing 6 m out at hand height keeps lobbing fruits
# (~7.3 m/s tosses at ~54 degrees, apex ~3 m), each new fruit leaving the hand
# while the arm swings at the previous one. Hit points are sampled from a box
# spanning the arm's workspace; samples the cut-state IK cannot reach, or whose
# legs cannot be planned or duration-matched, are redrawn. Consecutive hits are
# spaced at least 1.4 s apart: cut-to-cut legs that reverse a slice at
# SLICE_HEADROOM of the joint velocity limits have a velocity-limited floor of
# ~1.25 s between nearby hits, more when the cuts sit far apart or the leg has
# to detour around the obstacle cage.
THROWER = np.array([6.0, 0.0, 1.2])
SPAWN_SWAY = 0.5  # thrower sways up to +-0.5 m sideways between tosses
FLIGHT = 1.3  # s each fruit spends in the air
GAP_RANGE = (1.4, 2.0)  # s between consecutive hits
HIT_LO = np.array([0.35, -0.55, 0.35])
HIT_HI = np.array([0.65, 0.55, 0.80])

# Static sphere obstacles the cut-to-cut sweeps must plan around: a center ball
# in the middle of the hit box splitting the left/right halves of the
# workspace, two low guards under the box where the hand dips on low cuts
# (their tops sit below the lowest hit, so no pre-hit flight can clip them),
# an overhead ball above the shoulder that side-to-side transits must duck
# under (behind the box in x, so flights descending onto hits stay clear), and
# two side balls shaving the box rims. Sampled throws whose flight would pass
# through an obstacle are redrawn; cut debris falls through freely, like it
# already does through the robot.
OBSTACLES = [
    ([0.50, 0.0, 0.575], 0.12),
    ([0.50, -0.35, 0.15], 0.10),
    ([0.50, 0.35, 0.15], 0.10),
    ([0.15, 0.0, 1.05], 0.10),
    ([0.40, -0.70, 0.55], 0.10),
    ([0.40, 0.70, 0.55], 0.10),
]
OBSTACLE_COLOR = [70, 90, 130]

# Minimum surface distance (m) from a hit point to the nearest obstacle. The
# wind-down after a cut needs free space around the hit: a cut committed right
# against an obstacle plans fine INTO the cut but strands the stream on the way
# out (measured: a cut 0.17 m from a surface connected <10% of departures even
# at 15x the planning budget, stalling on collision-detour path shapes).
HIT_CLEARANCE = 0.18

FRUIT_COLORS = [[220, 30, 30], [240, 150, 30], [40, 170, 40]]

# Ready pose the arm starts from at rest (same nominal as the constrained demos).
READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

# TSR position box half-width (m) for the cut-state IK; orientation is free.
HIT_TOL = 0.005

# Seconds of room a cut posture must have inside the joint position limits to
# carry its slicing velocity through the hit. A posture pinned against a limit
# with velocity into it strands the planner: the outgoing leg starts with that
# same velocity, so every arc leaving the cut breaks the limit immediately.
FLOW_TIME = 0.15

# Log-rho bisection bracket and timing stop criterion for duration matching.
# Outside the bracket the trees starve: below, velocity-limited arcs stretch too
# long to chain; above, the near-time-optimal arcs get too aggressive to accept.
RHO_BOUNDS = (0.1, 1e3)
TIMING_TOL = 5e-4  # s

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

DENSE_DT = 1.0 / 240.0
FRAME_DT = 1.0 / 60.0

# Seconds of trajectory the planning process keeps buffered ahead of playback,
# absorbing the occasional slow leg (resampled throws, stalled bisections).
BUFFER_AHEAD = 5.0

# Seconds a cut (gray) fruit keeps falling before it is recycled.
FALL_TAIL = 0.75


def pose_to_transform(pose):
    """4x4 matrix -> (qw, qx, qy, qz, x, y, z)."""
    pose = np.asarray(pose)
    x, y, z, w = tr.quaternion_from_matrix(pose)
    return [w, x, y, z, *pose[:3, 3]]


class Throw:
    """One fruit's ballistic flight, back-solved so it passes through the nominal
    hit point exactly at launch + flight."""

    def __init__(self, launch, spawn, hit_point, flight):
        self.launch = launch
        self.spawn = np.asarray(spawn, dtype=np.float64)
        self.hit_point = np.asarray(hit_point, dtype=np.float64)
        self.flight = flight
        self.hit_time = launch + flight
        self.v0 = (self.hit_point - self.spawn - 0.5 * GRAVITY * flight**2) / flight

    def position(self, t):
        tau = t - self.launch
        if tau <= 0.0:
            return self.spawn.copy()
        return self.spawn + self.v0 * tau + 0.5 * GRAVITY * tau**2

    def velocity(self, t):
        return self.v0 + GRAVITY * (t - self.launch)


def flight_clear(throw):
    """True if the fruit stays clear of every obstacle from spawn to hit."""
    tau = np.linspace(0.0, throw.flight, int(throw.flight / DENSE_DT) + 1)
    points = throw.spawn + np.outer(tau, throw.v0) + 0.5 * np.outer(tau * tau, GRAVITY)
    return all(np.linalg.norm(points - center, axis=1).min() > radius + FRUIT_RADIUS
               for center, radius in OBSTACLES)


def unit_directions(n=2048):
    """Fibonacci-spiral sampling of the unit sphere."""
    i = np.arange(n)
    phi = np.pi * (3.0 - np.sqrt(5.0)) * i
    z = 1.0 - 2.0 * (i + 0.5) / n
    r = np.sqrt(1.0 - z * z)
    return np.stack([r * np.cos(phi), r * np.sin(phi), z], axis=1)


def position_jacobian(ambient, q, eps=1e-4):
    base = np.array(ambient.eefk(q))[:3, 3]
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.array(q, dtype=np.float64)
        dq[i] += eps
        J[:, i] = (np.array(ambient.eefk(dq.astype(np.float32)))[:3, 3] - base) / eps
    return J


def cut_state(ambient, environment, hit_point, limits, seed):
    """Hit configuration by TSR projection (position pinned to a HIT_TOL box,
    orientation free), slice direction chosen over the full unit sphere to
    maximize the end-effector speed achievable within SLICE_HEADROOM of the
    joint velocity limits at that posture (the fruit's own velocity is ignored:
    the swipe is as aggressive as the arm allows), and slicing joint velocity by
    damped least-squares differential IK. The cut is a flow-through state, so
    slice directions without FLOW_TIME of flow room (inside the joint position
    limits and clear of the obstacles) are skipped for the next-fastest ones;
    the projection is retried from READY when the warm seed fails. Returns
    (q, qdot, slice direction, prescribed speed, achieved end-effector
    velocity)."""
    target = np.eye(4)
    target[:3, 3] = hit_point
    tsr = ambient.TaskSpaceConstraint(
        IDENTITY, pose_to_transform(target),
        [-HIT_TOL] * 3 + [-3.2] * 3, [HIT_TOL] * 3 + [3.2] * 3)
    settings = vamp.ConstraintSettings()
    settings.max_iterations = 200
    lower = np.array(ambient.lower_bounds())
    upper = np.array(ambient.upper_bounds())
    directions = unit_directions()
    for ik_seed in (seed, READY):
        try:
            q = np.asarray(ambient.project(ik_seed, [tsr], settings),
                           dtype=np.float32)
        except ValueError:
            continue  # projection did not converge
        reached = np.array(ambient.eefk(q))[:3, 3]
        if np.linalg.norm(reached - hit_point) > np.sqrt(3.0) * HIT_TOL + 1e-4:
            continue  # projection did not converge onto the hit point
        if not ambient.validate(q, environment):
            continue
        J = position_jacobian(ambient, q)
        solve = np.linalg.inv(J @ J.T + 1e-6 * np.eye(3))
        qdot_per_speed = directions @ (J.T @ solve).T
        demand = np.abs(qdot_per_speed / limits).max(axis=1)  # joint ratio per m/s
        for pick in np.argsort(demand)[:512]:
            speed = SLICE_HEADROOM / float(demand[pick])
            d = directions[pick]
            qdot = speed * qdot_per_speed[pick]
            room = 0.05 + FLOW_TIME * np.abs(qdot)
            if np.any(q - room < lower) or np.any(q + room > upper):
                continue
            # The legs into and out of the cut hug the straight flow-through
            # line q + tau*qdot near the hit; a slice pointed into an obstacle
            # strands the planner exactly like a pinned joint limit, so fall
            # back to the next-fastest direction (usually the reverse swipe).
            # The check is asymmetric: a blocked incoming side only fails this
            # one sample (the leg into the cut cannot plan), so ~0.3 m of
            # clearance is enough, but a blocked outgoing side poisons every
            # leg AFTER the cut is committed, so the full wind-down line must
            # be clear.
            flow_in = min(FLOW_TIME, 0.3 / speed)
            if not all(ambient.validate((q + tau * qdot).astype(np.float32),
                                        environment)
                       for tau in np.linspace(-flow_in, FLOW_TIME, 15)):
                continue
            return q, qdot.astype(np.float32), d, speed, J @ qdot
    raise RuntimeError("no usable cut state at this hit point")


def hermite_arc(z0, z1, T, dt=DENSE_DT):
    """Dense (positions, velocities, times) of the LQMT arc between two flask
    states over its optimal time T. The minimum-effort arc with fixed boundary
    (q, qdot) and duration is the cubic Hermite interpolant."""
    n_q = len(z0) // 2
    q0, v0 = z0[:n_q], z0[n_q:]
    q1, v1 = z1[:n_q], z1[n_q:]
    ts = np.linspace(0.0, T, max(int(np.ceil(T / dt)) + 1, 2))
    s = ts / T
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    h11 = s**3 - s**2
    q = (np.outer(h00, q0) + np.outer(h10, T * v0)
         + np.outer(h01, q1) + np.outer(h11, T * v1))
    g00 = (6 * s**2 - 6 * s) / T
    g10 = 3 * s**2 - 4 * s + 1
    g01 = (-6 * s**2 + 6 * s) / T
    g11 = 3 * s**2 - 2 * s
    v = (np.outer(g00, q0) + np.outer(g10, v0)
         + np.outer(g01, q1) + np.outer(g11, v1))
    return q, v, ts


def plan_leg(module, planner_func, plan_settings, simp_settings, environment,
             z_from, z_to, target_duration):
    """Plan z_from -> z_to with the flask RRT-Connect and simplify, bisecting
    log-rho until the leg's total optimal time matches target_duration. The
    optimal times must be evaluated under the same rho as the plan, so each
    bisection step replans from a fresh deterministic sampler. Returns
    (waypoints, duration, rho, raw state count, iterations)."""

    def attempt(rho):
        module.set_rho(float(rho))
        result = planner_func(z_from, z_to, environment, plan_settings, module.halton())
        if not result.solved:
            return None
        simple = module.simplify(result.path, environment, simp_settings, module.halton())
        path = [np.asarray(q, dtype=np.float32) for q in simple.path]
        duration = sum(module.optimal_time(a, b) for a, b in zip(path, path[1:]))
        return path, duration, len(result.path)

    lo, hi = np.log(RHO_BOUNDS[0]), np.log(RHO_BOUNDS[1])
    fastest = None
    for _ in range(6):
        fastest = attempt(np.exp(hi))
        if fastest is not None:
            break
        # The fast end of the bracket can itself be unplannable when the goal
        # velocity is near the limits (arcs too aggressive to accept): walk it
        # down into the solvable range.
        hi -= np.log(2.0)
    if fastest is None:
        raise RuntimeError("planning failed across the rho bracket")
    path, duration, n_raw = fastest
    if duration > target_duration + TIMING_TOL:
        raise RuntimeError(
            f"leg infeasible: fastest duration {duration:.3f} s > gap {target_duration:.3f} s")
    best = (path, duration, np.exp(hi), n_raw)

    iteration = 0
    for iteration in range(1, 61):
        mid = 0.5 * (lo + hi)
        outcome = attempt(np.exp(mid))
        if outcome is None:
            # Unsolved legs inside the bracket sit on the slow side, where
            # velocity-limited arcs stretch too long to chain.
            lo = mid
            continue
        path, duration, n_raw = outcome
        if abs(duration - target_duration) < abs(best[1] - target_duration):
            best = (path, duration, np.exp(mid), n_raw)
        if abs(best[1] - target_duration) < TIMING_TOL:
            break
        if duration > target_duration:
            lo = mid  # Too slow: raise rho.
        else:
            hi = mid
        if hi - lo < 3e-5:
            # Bracket collapsed without converging: D(rho) is stuck on a path
            # shape change, and further replans at ~the same rho are identical.
            break
    path, duration, rho, n_raw = best
    if abs(duration - target_duration) > TIMING_TOL:
        # Simplified paths change shape between bisection steps, leaving
        # D(rho) too noisy to converge on some legs (obstacle detours make
        # this common). Retime the best path with its waypoints frozen: the
        # total optimal time is then smooth and monotone in rho, so this
        # bisection cannot stall. The retimed arcs differ slightly from the
        # ones the planner validated, so the dense trajectory is re-checked
        # by the caller before the leg is accepted.
        lo, hi = np.log(RHO_BOUNDS[0]), np.log(RHO_BOUNDS[1])
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            module.set_rho(float(np.exp(mid)))
            duration = sum(module.optimal_time(a, b) for a, b in zip(path, path[1:]))
            if abs(duration - target_duration) < TIMING_TOL:
                break
            if duration > target_duration:
                lo = mid
            else:
                hi = mid
        rho = float(np.exp(mid))
        if abs(duration - target_duration) > 10 * TIMING_TOL:
            raise RuntimeError(
                f"duration matching stalled: residual "
                f"{abs(duration - target_duration) * 1e3:.1f} ms")
    return path, duration, rho, n_raw, iteration


def main(visualize: bool = False, count: int = 0, seed: int = 0):
    """count fruits are thrown (0 = endless); seed drives the throw stream."""
    # Most legs connect quickly; a small budget rejects unplannable sampled
    # throws cheaply, which is what keeps the stream ahead of real time. But
    # weaving the obstacle cage sometimes genuinely needs more iterations, so
    # when every cheap try fails the retry loop escalates to the big budget.
    BUDGETS = (2000, 10000)
    (module, planner_func, plan_settings, simp_settings) = \
        vamp.configure_robot_and_planner_with_kwargs(
            "panda.flask", "rrtc",
            max_iterations=BUDGETS[0], max_samples=BUDGETS[0])
    plan_settings.range = 0.5
    # Flat z-space sample distances dwarf the geometric dynamic-domain radius
    # default, which starves the trees after the first trapped steer.
    plan_settings.radius = 8.0

    ambient = vamp.panda
    environment = vamp.Environment()
    for center, radius in OBSTACLES:
        environment.add_sphere(vamp.Sphere(center, radius))
    limits = np.array(module.velocity_limits())
    n_q = len(READY)
    rng = np.random.default_rng(seed)

    def next_fruit(z_from, clock):
        """Sample the next throw (hit point, spacing, thrower sway), synthesize
        its cut state, and plan the duration-matched interception leg. A sampled
        throw can be unreachable or its leg unplannable; resample a fresh one."""
        failures = []
        attempts = 0  # samples that survived the free geometric filters
        for tries in range(1, 201):
            if attempts >= 40:
                break
            gap = float(rng.uniform(*GAP_RANGE))
            hit_point = rng.uniform(HIT_LO, HIT_HI)
            if min(np.linalg.norm(hit_point - center) - radius
                   for center, radius in OBSTACLES) < HIT_CLEARANCE:
                continue
            spawn = THROWER + np.array([0.0, rng.uniform(-SPAWN_SWAY, SPAWN_SWAY), 0.0])
            throw = Throw(clock + gap - FLIGHT, spawn, hit_point, FLIGHT)
            if not flight_clear(throw):
                continue
            attempts += 1
            budget = BUDGETS[0] if attempts <= 20 else BUDGETS[1]
            plan_settings.max_iterations = budget
            plan_settings.max_samples = budget
            try:
                q, qdot, d, speed, ee_velocity = cut_state(
                    ambient, environment, hit_point, limits[:n_q], z_from[:n_q])
                z_to = np.concatenate([q, qdot])
                path, duration, rho, n_raw, _ = plan_leg(
                    module, planner_func, plan_settings, simp_settings,
                    environment, z_from, z_to, gap)
            except RuntimeError as failure:
                failures.append(str(failure).split(":")[0])
                continue
            # Dense Hermite evaluation of the leg under its matched rho,
            # appended to the one shared clock.
            module.set_rho(rho)
            qs, vs, ts = [], [], []
            leg_clock = clock
            for a, b in zip(path, path[1:]):
                qq, vv, tt = hermite_arc(a, b, module.optimal_time(a, b))
                qs.append(qq)
                vs.append(vv)
                ts.append(leg_clock + tt)
                leg_clock = ts[-1][-1]
            dense = (np.vstack(qs), np.vstack(vs), np.concatenate(ts))
            # The executed trajectory is these dense arcs, not the exact ones
            # the planner validated (retimed legs shift every arc's duration),
            # so check what will actually run. Velocity gets a small allowance:
            # interior Hermite overshoot already grazes the limits on legs the
            # planner itself accepts.
            if (np.abs(dense[1]) / limits[:n_q]).max() > 1.02 or not all(
                    ambient.validate(qc, environment)
                    for qc in dense[0].astype(np.float32)):
                failures.append("executed trajectory failed validation")
                continue
            leg = (rho, n_raw, len(path), duration, gap, tries)
            return throw, z_to, (d, speed, ee_velocity), leg, dense
        counts = {m: failures.count(m) for m in dict.fromkeys(failures)}
        raise RuntimeError(f"no plannable throw in {attempts} planning attempts: "
                           + "; ".join(f"{m} x{c}" for m, c in counts.items()))

    def report(index, throw, z_to, slice_info, leg_info, dense):
        d, speed, ee_velocity = slice_info
        rho, n_raw, n_simple, duration, gap, tries = leg_info
        t_hit = float(dense[2][-1])
        ee_position = np.array(ambient.eefk(z_to[:n_q]))[:3, 3]
        miss = float(np.linalg.norm(ee_position - throw.position(t_hit)))
        along = float(ee_velocity @ d)
        cut = miss < FRUIT_RADIUS and along >= 0.9 * speed
        vel_ratio = float((np.abs(dense[1]) / limits[:n_q]).max())
        sampled = f" [{tries} throws sampled]" if tries > 1 else ""
        print(f"fruit {index}: leg rho {rho:8.3f}, {n_raw} -> {n_simple} states, "
              f"duration {duration:.4f} s (gap {gap:.4f}, "
              f"residual {abs(duration - gap) * 1e3:.2f} ms); "
              f"hit at {t_hit:.3f} s, miss {miss * 1e3:.1f} mm, "
              f"slice {along:.2f} m/s, joint ratio {vel_ratio:.2f} "
              f"-> {'CUT' if cut else 'MISS'}{sampled}")
        return cut, t_hit

    if not visualize:
        z = np.asarray(vf.rest_state(READY), dtype=np.float32)
        clock, index, cuts = 0.0, 0, 0
        start = time.perf_counter()
        while count == 0 or index < count:
            index += 1
            throw, z_to, slice_info, leg_info, dense = next_fruit(z, clock)
            cut, clock = report(index, throw, z_to, slice_info, leg_info, dense)
            cuts += cut
            z = z_to
        wall = time.perf_counter() - start
        print(f"{cuts}/{index} fruits cut; {clock:.1f} s of motion planned "
              f"in {wall:.1f} s wall ({clock / wall:.0f}x real time)")
        return

    # The planner holds the GIL (measured: a leg's bisection blocks other
    # threads for tens of milliseconds per step), so an in-process planner
    # thread makes the render loop below stutter. Run planning in its own
    # forked process instead, streaming finished legs back over a queue. Fork
    # before the viser server exists so the child inherits no server threads.
    context = multiprocessing.get_context("fork")
    playhead = context.Value("d", 0.0)
    planned = context.Queue()

    def producer():
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # parent's ctrl-c cleans up
        z = np.asarray(vf.rest_state(READY), dtype=np.float32)
        clock, index, cuts = 0.0, 0, 0
        while count == 0 or index < count:
            if clock - playhead.value > BUFFER_AHEAD:
                time.sleep(0.05)
                continue
            index += 1
            throw, z_to, slice_info, leg_info, dense = next_fruit(z, clock)
            cut, t_hit = report(index, throw, z_to, slice_info, leg_info, dense)
            cuts += cut
            planned.put((throw, t_hit, dense[2], dense[0]))
            z, clock = z_to, t_hit
        print(f"throw stream finished: {cuts}/{index} fruits cut")

    producer_process = context.Process(target=producer, daemon=True)
    producer_process.start()

    from viser_utils import setup_viser_with_robot, add_spheres

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot.update_cfg(READY)

    # Fruits cycle through a fixed pool of colored/gray sphere pairs (colored
    # while airborne, gray falling away after the cut). With hits >= 1.3 s apart
    # and each fruit visible for flight + tail ~= 2 s, at most two fruits are on
    # screen at once, so consecutive fruits never share a pool slot.
    hidden = np.array([0.0, 0.0, -50.0])
    handles = add_spheres(
        server,
        [hidden] * (2 * len(FRUIT_COLORS)),
        [FRUIT_RADIUS] * (2 * len(FRUIT_COLORS)),
        colors=[c for color in FRUIT_COLORS for c in (color, [120, 120, 120])],
        prefix="/fruit",
    )
    pool = [(handles[2 * j], handles[2 * j + 1]) for j in range(len(FRUIT_COLORS))]

    add_spheres(
        server,
        [center for center, _ in OBSTACLES],
        [radius for _, radius in OBSTACLES],
        colors=[OBSTACLE_COLOR] * len(OBSTACLES),
        prefix="/obstacle",
    )

    segments = []  # contiguous (times, configs) chunks of the executed trajectory
    active = []  # fruit records: throw, executed hit time, pool slot

    play = server.gui.add_checkbox("Play", initial_value=True)
    print(f"visualization at http://localhost:{server.get_port()}; ctrl-c to exit")

    t0 = time.perf_counter()
    t = 0.0
    slot = 0
    was_playing = True
    while True:
        try:  # Drain newly planned legs from the planner process.
            while True:
                throw, t_hit, times, configs = planned.get_nowait()
                segments.append((times, configs))
                active.append((throw, t_hit, slot % len(pool)))
                slot += 1
        except queue.Empty:
            pass
        if not segments:  # First leg still planning: hold the clock at zero.
            t0 = time.perf_counter()
            time.sleep(FRAME_DT)
            continue
        if play.value:
            if not was_playing:  # Resume from wherever playback was paused.
                t0 = time.perf_counter() - t
            t = time.perf_counter() - t0
            end = segments[-1][0][-1]
            if t > end:  # Planner behind (or stream finished): hold here.
                t0 += t - end
                t = end
            while len(segments) > 1 and segments[0][0][-1] < t - 1.0:
                segments.pop(0)
            while active and active[0][1] + FALL_TAIL + 0.5 < t:
                active.pop(0)
            config = None
            for times, configs in segments:
                if t <= times[-1]:
                    config = configs[min(np.searchsorted(times, t), len(times) - 1)]
                    break
            playhead.value = t
            if config is not None:
                robot.update_cfg(config)
            positions = {}
            for throw, t_hit, slot_index in active:
                if throw.launch <= t < t_hit:
                    positions[(slot_index, 0)] = throw.position(t)
                elif t_hit <= t <= t_hit + FALL_TAIL:
                    positions[(slot_index, 1)] = throw.position(t)
            for slot_index, pair in enumerate(pool):
                for half, handle in enumerate(pair):
                    handle.position = positions.get((slot_index, half), hidden)
        was_playing = play.value
        time.sleep(FRAME_DT)


if __name__ == "__main__":
    Fire(main)
