"""Line-constraint benchmark for the integrated chart-LQMT planner.

Port of the mcflask-branch prototype benchmark (MCVAMP paper line-problem recipe) to
the integrated API: end-effector constrained to a line (orientation free), start/goal
configurations sampled on the constraint by projection, random sphere obstacles with
clearance around the line.

panda.flask.rrtc plans rest-to-rest kinodynamic trajectories on the manifold
(chart LQMT cubics lifted through the SIMD projector, bidirectional trees);
panda.rrtc with the same constraints plans geometric paths (projection-based local
planner). Times are not apples-to-apples (different problems solved), but both are
reported, along with constraint adherence for both.

Prototype reference (mcflask branch, single tree): 92% success, ~0.5 ms median,
adherence <= 1e-6 squared on 31/33, max velocity ratio 0.46.

Velocity note: bounds are enforced on execution-frame velocities — each sample's
chart-center-frame velocity is re-projected into the tangent space at its own
lifted point before the limit check, matching the lift used at execution time —
so the velocity ratio gate holds at any chart radius. (The prototype enforced
chart-center-frame velocities only and hid the tangent seam with
velocity_scale=0.25, reporting 0.46.)
"""

import collections
import json
import time

import numpy as np

import vamp
from vamp import flask

LINES = [(0.0, 0.35), (0.25, 0.5), (-0.2, 0.45)]  # (y0, z0), line along world x
N_PAIRS = 4
OBSTACLE_LEVELS = [0, 3, 6]
N_RUNS = 10
LINE_CLEARANCE = 0.22
OBS_RADIUS = 0.12
SEED = 17

# Phase-capped configuration: one set of caps for the phase-gated chart runs. The gates
# bound execution-frame kinetic energy and EEF linear speed; enforcement is at the
# planner's validation samples, so uniformly-resampled lift samples may peak slightly
# between them.
KE_CAP = 1.0  # joules
EE_SPEED_CAP = 0.5  # m/s
PHASE_RATIO_TOL = 1.05


def make_line_constraint(pa, y0, z0):
    # TSR frame at (0, y0, z0), identity rotation; x-translation and all rotation free.
    return pa.TaskSpaceConstraint(
        [1, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0.0, y0, z0],
        [-10.01, -0.01, -0.01, -3.15, -3.15, -3.15],
        [10.01, 0.01, 0.01, 3.15, 3.15, 3.15],
    )


def sample_on_constraint(pa, constraints, rng, lo, hi, n, env):
    out = []
    tries = 0
    while len(out) < n and tries < 5000:
        tries += 1
        q = rng.uniform(lo, hi).astype(np.float32)
        try:
            qp = np.asarray(pa.project(q, constraints), dtype=np.float32)
        except ValueError:
            continue
        if not pa.satisfied(qp, constraints):
            continue
        if not pa.validate(qp, env):
            continue
        out.append(qp)
    return out


def line_point_distance(p, y0, z0):
    return float(np.hypot(p[1] - y0, p[2] - z0))


def lift_path(pf, path, constraints, chs, pos_tol):
    """Lift every edge of a z-path; per edge pick the direction whose endpoints match.

    Goal-tree edges live in the chart of their far endpoint, so a forward lift from
    path[i] may not reconstruct them; the backward lift from path[i+1] does.
    Lifted samples carry execution-frame velocities, so the kinetic energy and
    end-effector speed measured on them are the execution-frame quantities the phase
    gates bound. Returns (max hinged error, max |v| per joint, total duration,
    n samples, max kinetic energy, max EEF speed).
    """
    max_err = 0.0
    vmax_seen = None
    total_T = 0.0
    n_total = 0
    max_ke = 0.0
    max_ee = 0.0
    for i in range(len(path) - 1):
        a = np.asarray(path[i])
        b = np.asarray(path[i + 1])
        best = None
        for frm, tgt, fwd, endpoint in ((a, b, True, b), (b, a, False, a)):
            try:
                states, errs, T, cost = pf.lift_edge(frm, tgt, constraints, forward=fwd, n_samples=32)
            except ValueError:
                continue
            miss = float(np.linalg.norm(np.asarray(states[-1])[: len(a) // 2] - endpoint[: len(a) // 2]))
            if best is None or miss < best[0]:
                best = (miss, states, errs, T)
        if best is None:
            return None
        miss, states, errs, T = best
        if miss > pos_tol * (1.0 + 1e-3):
            return None
        max_err = max(max_err, float(max(errs)))
        v = np.abs(np.asarray(states)[:, len(a) // 2 :])
        vm = v.max(axis=0)
        vmax_seen = vm if vmax_seen is None else np.maximum(vmax_seen, vm)
        total_T += float(T)
        n_total += len(states)
        for st in states:
            sa = np.array(st, dtype=np.float32)
            max_ke = max(max_ke, float(pf.kinetic_energy(sa)))
            max_ee = max(max_ee, flask.max_eef_speed(pf, sa))
    return max_err, vmax_seen, total_T, n_total, max_ke, max_ee


def main():
    rng = np.random.default_rng(SEED)

    pa = vamp.panda
    pf = vamp.panda.flask

    settings = vamp.RRTCSettings()
    settings.range = 0.5
    settings.max_iterations = 50000
    settings.max_samples = 50000

    chs = vamp.ChartSettings()
    chs.eps_chart = 0.5

    phase_gates = flask.phase_constraints(pf, KE_CAP, EE_SPEED_CAP)

    gsettings = vamp.RRTCSettings()
    gsettings.range = 0.5
    gsettings.max_iterations = 50000
    gsettings.max_samples = 50000

    lo = np.array(pa.lower_bounds(), dtype=np.float64)
    hi = np.array(pa.upper_bounds(), dtype=np.float64)
    vlim = np.array(pf.velocity_limits(), dtype=np.float64)

    results = []

    for line_idx, (y0, z0) in enumerate(LINES):
        constraints = [make_line_constraint(pa, y0, z0)]
        empty = vamp.Environment()

        configs = sample_on_constraint(pa, constraints, rng, lo, hi, 2 * N_PAIRS + 4, empty)
        if len(configs) < 2 * N_PAIRS:
            print(f"line {line_idx}: only {len(configs)} valid configs, skipping some pairs")

        candidates = []
        for i in range(len(configs)):
            for j in range(i + 1, len(configs)):
                if np.linalg.norm(configs[i] - configs[j]) > 1.5:
                    candidates.append((configs[i], configs[j]))
        rng.shuffle(candidates)

        # Prescreen: keep pairs the geometric planner can solve in the empty scene.
        # The prototype's RNG stream is not reproducible through the new projector, and
        # unscreened draws can pick pairs straddling the robot base (the on-line EEF
        # would have to pass through the body at x=0), infeasible for any planner --
        # the prototype hit one such pair (line 0 pair 2).
        pairs = []
        for qs, qg in candidates:
            if len(pairs) == N_PAIRS:
                break
            sampler = pa.halton()
            if pa.rrtc(qs, qg, empty, gsettings, sampler, constraints).solved:
                pairs.append((qs, qg))

        for pair_idx, (qs, qg) in enumerate(pairs):
            zs, zg = flask.rest_state(qs), flask.rest_state(qg)
            for n_obs in OBSTACLE_LEVELS:
                env = vamp.Environment()
                added = 0
                otries = 0
                while added < n_obs and otries < 500:
                    otries += 1
                    c = rng.uniform([-0.7, -0.7, 0.05], [0.7, 0.7, 0.9])
                    if line_point_distance(c, y0, z0) < LINE_CLEARANCE + OBS_RADIUS:
                        continue
                    cand = vamp.Environment()
                    cand.add_sphere(vamp.Sphere(c.tolist(), OBS_RADIUS))
                    if not pa.validate(qs, cand) or not pa.validate(qg, cand):
                        continue
                    env.add_sphere(vamp.Sphere(c.tolist(), OBS_RADIUS))
                    added += 1

                row = dict(line=line_idx, pair=pair_idx, n_obs=added)

                # --- chart-LQMT (kinodynamic, on-manifold) ---
                times, succ = [], 0
                best = None
                for k in range(N_RUNS):
                    sampler = pf.halton()
                    sampler.skip(7919 * k)
                    res = pf.rrtc(zs, zg, env, settings, sampler, constraints, chart_settings=chs)
                    if res.solved:
                        succ += 1
                        times.append(res.nanoseconds / 1e6)
                        if best is None or len(res.path) < len(best.path):
                            best = res
                row["chart_success"] = succ / N_RUNS
                row["chart_ms_median"] = float(np.median(times)) if times else None
                if best is not None:
                    lifted = lift_path(pf, best.path, constraints, chs, chs.reached_pos_tol)
                    if lifted is not None:
                        max_err, vmax_seen, total_T, n_samp, max_ke, max_ee = lifted
                        row["chart_duration_s"] = total_T
                        row["chart_segments"] = len(best.path) - 1
                        row["chart_max_constraint_dist2"] = max_err * max_err
                        row["chart_max_vel_ratio"] = float((vmax_seen / vlim).max())
                        row["chart_max_ke"] = max_ke
                        row["chart_max_ee_speed"] = max_ee
                        row["chart_goal_err"] = float(
                            np.linalg.norm(np.asarray(best.path[len(best.path) - 1])[:7] - qg))
                    else:
                        row["chart_lift_failed"] = True

                # --- chart-LQMT with phase gates (capped config) ---
                times, succ = [], 0
                best = None
                for k in range(N_RUNS):
                    sampler = pf.halton()
                    sampler.skip(7919 * k)
                    res = pf.rrtc(
                        zs, zg, env, settings, sampler, constraints,
                        chart_settings=chs, phase_constraints=phase_gates)
                    if res.solved:
                        succ += 1
                        times.append(res.nanoseconds / 1e6)
                        if best is None or len(res.path) < len(best.path):
                            best = res
                row["phase_success"] = succ / N_RUNS
                row["phase_ms_median"] = float(np.median(times)) if times else None
                if best is not None:
                    lifted = lift_path(pf, best.path, constraints, chs, chs.reached_pos_tol)
                    if lifted is not None:
                        _, _, total_T, _, max_ke, max_ee = lifted
                        row["phase_duration_s"] = total_T
                        row["phase_max_ke_ratio"] = max_ke / KE_CAP
                        row["phase_max_ee_ratio"] = max_ee / EE_SPEED_CAP
                    else:
                        row["phase_lift_failed"] = True

                # --- geometric constrained RRTC baseline ---
                times, succ = [], 0
                cpath = None
                for k in range(N_RUNS):
                    sampler = pa.halton()
                    sampler.skip(7919 * k)
                    res = pa.rrtc(qs, qg, env, gsettings, sampler, constraints)
                    if res.solved:
                        succ += 1
                        times.append(res.nanoseconds / 1e6)
                        if cpath is None:
                            cpath = res.path.numpy()
                row["geo_success"] = succ / N_RUNS
                row["geo_ms_median"] = float(np.median(times)) if times else None
                if cpath is not None:
                    length = float(sum(np.linalg.norm(b - a) for a, b in zip(cpath[:-1], cpath[1:])))
                    row["geo_path_length"] = length

                results.append(row)
                print(
                    f"line {line_idx} pair {pair_idx} obs {added}: "
                    f"chart {row['chart_success']:.0%} "
                    f"{row.get('chart_ms_median') or float('nan'):8.2f}ms "
                    f"dur {row.get('chart_duration_s', float('nan')):6.2f}s "
                    f"cdist2 {row.get('chart_max_constraint_dist2', float('nan')):.2e} | "
                    f"phase {row['phase_success']:.0%} "
                    f"ke {row.get('phase_max_ke_ratio', float('nan')):.2f} "
                    f"ee {row.get('phase_max_ee_ratio', float('nan')):.2f} | "
                    f"geo {row['geo_success']:.0%} "
                    f"{row.get('geo_ms_median') or float('nan'):8.2f}ms",
                    flush=True,
                )

    with open("/tmp/chart_line_benchmark.json", "w") as f:
        json.dump(results, f, indent=1)

    by_obs = collections.defaultdict(list)
    for r in results:
        by_obs[r["n_obs"]].append(r)
    print("\n=== SUMMARY (per obstacle count) ===")
    for n_obs in sorted(by_obs):
        rows = by_obs[n_obs]
        mt = [r["chart_ms_median"] for r in rows if r["chart_ms_median"]]
        gt = [r["geo_ms_median"] for r in rows if r["geo_ms_median"]]
        ms = np.mean([r["chart_success"] for r in rows])
        gs = np.mean([r["geo_success"] for r in rows])
        md = [r["chart_max_constraint_dist2"] for r in rows if "chart_max_constraint_dist2" in r]
        dur = [r["chart_duration_s"] for r in rows if "chart_duration_s" in r]
        print(
            f"obs={n_obs}: chart succ {ms:.0%} median {np.median(mt) if mt else float('nan'):.2f}ms "
            f"traj {np.median(dur) if dur else float('nan'):.1f}s maxc {max(md) if md else 0:.1e} | "
            f"geo succ {gs:.0%} median {np.median(gt) if gt else float('nan'):.2f}ms"
        )

    # gate verdicts vs the mcflask prototype reference
    all_succ = np.mean([r["chart_success"] for r in results])
    all_med = np.median([r["chart_ms_median"] for r in results if r["chart_ms_median"]])
    adher = [r["chart_max_constraint_dist2"] for r in results if "chart_max_constraint_dist2" in r]
    n_tight = sum(a <= 1e-6 for a in adher)
    vr = [r["chart_max_vel_ratio"] for r in results if "chart_max_vel_ratio" in r]
    lift_fail = sum(1 for r in results if r.get("chart_lift_failed"))
    print("\n=== GATES (prototype: succ 92%, median ~0.5 ms, adherence <= 1e-6 on 31/33) ===")
    print(f"success:    {all_succ:.1%} {'OK' if all_succ >= 0.92 else 'FAIL'}")
    print(f"median:     {all_med:.2f} ms {'OK' if all_med <= 5.0 else 'FAIL'}")
    print(f"adherence:  {n_tight}/{len(adher)} <= 1e-6 sq (max {max(adher):.1e}) "
          f"{'OK' if adher and max(adher) <= 1e-4 else 'FAIL'}")
    print(f"lift recon: {lift_fail} paths failed edge reconstruction "
          f"{'OK' if lift_fail == 0 else 'FAIL'}")
    print(f"vel ratio:  max {max(vr):.2f} {'OK' if vr and max(vr) <= 1.0 else 'FAIL'}")

    # phase-gated verdicts (capped config); uncapped maxima reported for cap tuning
    uncapped_ke = [r["chart_max_ke"] for r in results if "chart_max_ke" in r]
    uncapped_ee = [r["chart_max_ee_speed"] for r in results if "chart_max_ee_speed" in r]
    p_succ = np.mean([r["phase_success"] for r in results])
    p_med = np.median([r["phase_ms_median"] for r in results if r["phase_ms_median"]])
    p_ke = [r["phase_max_ke_ratio"] for r in results if "phase_max_ke_ratio" in r]
    p_ee = [r["phase_max_ee_ratio"] for r in results if "phase_max_ee_ratio" in r]
    p_lift_fail = sum(1 for r in results if r.get("phase_lift_failed"))
    print(f"\n=== PHASE GATES (KE cap {KE_CAP} J, EEF speed cap {EE_SPEED_CAP} m/s) ===")
    print(f"uncapped:   max KE {max(uncapped_ke):.2f} J median {np.median(uncapped_ke):.2f} J | "
          f"max EEF {max(uncapped_ee):.2f} m/s median {np.median(uncapped_ee):.2f} m/s")
    print(f"success:    {p_succ:.1%} {'OK' if p_succ >= 0.92 else 'FAIL'}")
    print(f"median:     {p_med:.2f} ms {'OK' if p_med <= 5.0 else 'FAIL'}")
    print(f"KE ratio:   max {max(p_ke):.3f} "
          f"{'OK' if p_ke and max(p_ke) <= PHASE_RATIO_TOL else 'FAIL'}")
    print(f"EEF ratio:  max {max(p_ee):.3f} "
          f"{'OK' if p_ee and max(p_ee) <= PHASE_RATIO_TOL else 'FAIL'}")
    print(f"lift recon: {p_lift_fail} phase paths failed edge reconstruction "
          f"{'OK' if p_lift_fail == 0 else 'FAIL'}")


if __name__ == "__main__":
    main()
