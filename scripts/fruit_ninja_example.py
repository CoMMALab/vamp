"""Fruit ninja demonstration on the Panda's chart-LQMT (flask) planner.

Three spheres are thrown on ballistic arcs toward the Panda in rapid fire, each
leaving the thrower's hand just before the previous one is cut. The arm must
intersect each fruit at its designed hit time with a prescribed slicing velocity
(the fastest swipe the arm can swing at that posture) to cut it.

The throws are fixed up front; the planner has to match the flight times exactly:

- Cut states: for each fruit an ambient TSR projection places the end-effector at
  the nominal hit point (orientation free), the slice direction is chosen over
  the full unit sphere to maximize the end-effector speed achievable within
  SLICE_HEADROOM of the joint velocity limits at that posture (the fruit's own
  velocity is deliberately ignored: the swipe is as aggressive as the arm can
  swing, which favors tangential sweeps over the nearly-radial weak axis of the
  position Jacobian), and damped least-squares differential IK converts the
  slice vector into goal joint velocities.
- Timed interception: each leg (rest -> cut 1 -> cut 2 -> cut 3) is planned with
  the flask RRT-Connect and simplified, then the time/effort trade-off rho of the
  LQMT cost C = rho T + integral |u|^2 is bisected until the leg's total optimal
  time equals the flight-time gap to sub-millisecond residual. rho only reshapes
  the timing of the arcs; the cut states' boundary velocities are untouched.
- Fruits are targets only, not collision objects: the workspace is empty and the
  planner is exercised purely for the timed kinodynamic connections.

A cut lands if the executed end-effector position at the executed hit time is
inside the fruit and the slice-direction speed meets the prescription. Position
miss comes from the TSR box (corner up to ~9 mm) plus timing residual times fruit
speed (~5 mm/ms); both are printed honestly.
"""

from pathlib import Path
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
# speed achievable within it, so the prescribed speed is per-fruit. Beyond 0.8
# the legs into and out of the cut stretch past ~1.1 s (the arc must wind a
# near-limit boundary velocity up and back down without its interior overshoot
# breaking the limits), forcing loftier throws for no visible extra snap.
SLICE_HEADROOM = 0.8

# Volley design: (launch time s, spawn point, nominal hit point, flight time s).
# Rapid fire from one thrower standing 6 m out at hand height: ~7.3 m/s tosses at
# ~54 degrees, apex ~3 m, each fruit leaving the hand just before the previous
# cut so consecutive fruits overlap in the air. Flatter is not possible without
# breaking the overlap: cut-to-cut legs that reverse a slice at SLICE_HEADROOM
# of the joint velocity limits have a velocity-limited floor of ~1.15 s between
# hits, and a fruit aloft that long drops g tau^2 / 2 = 8 m without the launch
# lofting it back up.
VOLLEY = [
    (0.00, [6.0, -0.5, 1.2], [0.45, -0.3, 0.55], 1.3),
    (1.25, [6.0, 0.0, 1.2], [0.55, 0.0, 0.65], 1.3),
    (2.50, [6.0, 0.5, 1.2], [0.45, 0.3, 0.55], 1.3),
]

FRUIT_COLORS = [[220, 30, 30], [240, 150, 30], [40, 170, 40]]

# Ready pose the arm starts from at rest (same nominal as the constrained demos).
READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

# TSR position box half-width (m) for the cut-state IK; orientation is free.
HIT_TOL = 0.005

# Log-rho bisection bracket and timing stop criterion for duration matching.
# Outside the bracket the trees starve: below, velocity-limited arcs stretch too
# long to chain; above, the near-time-optimal arcs get too aggressive to accept.
RHO_BOUNDS = (0.1, 1e3)
TIMING_TOL = 5e-4  # s

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

DENSE_DT = 1.0 / 240.0
FRAME_DT = 1.0 / 60.0


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
    damped least-squares differential IK. Returns (q, qdot, slice direction,
    prescribed speed, achieved end-effector velocity)."""
    target = np.eye(4)
    target[:3, 3] = hit_point
    tsr = ambient.TaskSpaceConstraint(
        IDENTITY, pose_to_transform(target),
        [-HIT_TOL] * 3 + [-3.2] * 3, [HIT_TOL] * 3 + [3.2] * 3)
    settings = vamp.ConstraintSettings()
    settings.max_iterations = 200
    q = np.asarray(ambient.project(seed, [tsr], settings), dtype=np.float32)
    if not ambient.validate(q, environment):
        raise RuntimeError("cut configuration is in collision")

    J = position_jacobian(ambient, q)
    solve = np.linalg.inv(J @ J.T + 1e-6 * np.eye(3))
    directions = unit_directions()
    qdot_per_speed = directions @ (J.T @ solve).T
    demand = np.abs(qdot_per_speed / limits).max(axis=1)  # joint ratio per m/s
    pick = int(np.argmin(demand))
    speed = SLICE_HEADROOM / float(demand[pick])
    d = directions[pick]
    qdot = speed * qdot_per_speed[pick]
    return q, qdot.astype(np.float32), d, speed, J @ qdot


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
    path, duration, rho, n_raw = best
    return path, duration, rho, n_raw, iteration


def main(visualize: bool = False):
    (module, planner_func, plan_settings, simp_settings) = \
        vamp.configure_robot_and_planner_with_kwargs(
            "panda.flask", "rrtc", max_iterations=50000, max_samples=50000)
    plan_settings.range = 0.5
    # Flat z-space sample distances dwarf the geometric dynamic-domain radius
    # default, which starves the trees after the first trapped steer.
    plan_settings.radius = 8.0

    ambient = vamp.panda
    environment = vamp.Environment()
    limits = np.array(module.velocity_limits())
    n_q = len(READY)

    throws = [Throw(*design) for design in VOLLEY]

    # Cut states, seeded by the previous fruit's hit configuration.
    cuts = []
    seed = READY
    for throw in throws:
        q, qdot, d, speed, ee_velocity = cut_state(
            ambient, environment, throw.hit_point, limits[:n_q], seed)
        cuts.append((np.concatenate([q, qdot]), d, speed, ee_velocity))
        seed = q

    # Timed legs: rest -> cut 1 -> cut 2 -> cut 3, each leg's duration bisected
    # to the flight-time gap.
    states = [vf.rest_state(READY)] + [np.asarray(z, dtype=np.float32) for z, _, _, _ in cuts]
    gaps = np.diff([0.0] + [throw.hit_time for throw in throws])
    legs = []
    elapsed = time.perf_counter()
    for z_from, z_to, gap in zip(states, states[1:], gaps):
        legs.append(plan_leg(module, planner_func, plan_settings, simp_settings,
                             environment, z_from, z_to, gap))
    elapsed = time.perf_counter() - elapsed
    print(f"fruit ninja: 3 legs planned and duration-matched in {elapsed * 1e3:.1f} ms")

    # Executed trajectory: dense Hermite evaluation of every simplified edge under
    # its optimal time, chained on one clock starting at the first launch.
    dense_q, dense_v, dense_t = [], [], []
    clock = 0.0
    executed_hits = []
    for path, duration, rho, n_raw, iterations in legs:
        module.set_rho(rho)
        for a, b in zip(path, path[1:]):
            q, v, ts = hermite_arc(a, b, module.optimal_time(a, b))
            dense_q.append(q)
            dense_v.append(v)
            dense_t.append(clock + ts)
            clock = dense_t[-1][-1]
        executed_hits.append(clock)
    dense_q = np.vstack(dense_q)
    dense_v = np.vstack(dense_v)
    dense_t = np.concatenate(dense_t)

    for i, ((path, duration, rho, n_raw, iterations), gap) in enumerate(zip(legs, gaps)):
        print(f"leg {i + 1}: rho {rho:8.3f}, {n_raw} -> {len(path)} states, "
              f"duration {duration:.4f} s (gap {gap:.4f}, "
              f"residual {abs(duration - gap) * 1e3:.2f} ms, {iterations} bisections)")

    vel_ratio = float((np.abs(dense_v) / limits[:n_q]).max())
    print(f"trajectory: {dense_t[-1]:.3f} s, max joint velocity ratio {vel_ratio:.2f}")

    n_cut = 0
    for i, (throw, (z, d, speed, ee_velocity)) in enumerate(zip(throws, cuts)):
        t_hit = executed_hits[i]
        ee_position = np.array(ambient.eefk(z[:n_q]))[:3, 3]
        miss = float(np.linalg.norm(ee_position - throw.position(t_hit)))
        along = float(ee_velocity @ d)
        cut = miss < FRUIT_RADIUS and along >= 0.9 * speed
        n_cut += cut
        print(f"fruit {i + 1}: hit at {t_hit:.4f} s (designed {throw.hit_time:.4f}), "
              f"miss {miss * 1e3:.1f} mm (radius {FRUIT_RADIUS * 1e3:.0f}), "
              f"slice speed {along:.3f} m/s (prescribed {speed:.3f}) "
              f"-> {'CUT' if cut else 'MISS'}")
    print(f"{n_cut}/3 fruits cut")

    if visualize:
        from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory

        robot_dir = Path(__file__).parents[1] / "resources" / "panda"
        server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
        robot.update_cfg(READY)

        # One shared clock: uniform frames sampling the executed arm trajectory
        # (with a beat of tail to watch the last fruit fall away) and the fruits'
        # ballistic arcs. Each fruit swaps from its colored sphere to a gray one
        # at its cut time; hidden spheres are parked out of view.
        frame_times = np.arange(0.0, dense_t[-1] + 0.75, FRAME_DT)
        indices = np.clip(np.searchsorted(dense_t, frame_times), 0, len(dense_t) - 1)
        waypoints = dense_q[indices]

        hidden = np.array([0.0, 0.0, -50.0])
        fruit_positions = []
        for t in frame_times:
            frame = []
            for throw, t_hit in zip(throws, executed_hits):
                p = throw.position(t)
                frame.append(p if t < t_hit else hidden)
                frame.append(hidden if t < t_hit else p)
            fruit_positions.append(frame)
        fruit_positions = np.array(fruit_positions)

        handles = add_spheres(
            server,
            fruit_positions[0],
            [FRUIT_RADIUS] * 6,
            colors=[c for color in FRUIT_COLORS for c in (color, [120, 120, 120])],
            prefix="/fruit",
        )

        slider = add_trajectory(server, waypoints, robot, handles, fruit_positions)
        play = server.gui.add_checkbox("Play", initial_value=True)
        period = frame_times[-1] + 1.0

        print(f"visualization at http://localhost:{server.get_port()}; ctrl-c to exit")
        t0 = time.perf_counter()
        was_playing = True
        while True:
            if play.value:
                if not was_playing:  # Resume from wherever the slider was scrubbed to.
                    t0 = time.perf_counter() - frame_times[int(slider.value)]
                t = min((time.perf_counter() - t0) % period, frame_times[-1])
                idx = int(np.searchsorted(frame_times, t, side="right") - 1)
                if idx != int(slider.value):
                    slider.value = idx  # Fires the slider callback, which moves everything.
            was_playing = play.value
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    Fire(main)
