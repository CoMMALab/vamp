# Affine-arithmetic reachability — tightness test (the interval-FK de-risk)

Question: can a *provable* directional reachability bound recover the sampled 2–4×
pruning, or does it collapse to VAMP's existing radial bound? Built an affine-arithmetic
(AA) forward-kinematics evaluator (`phase3/aa_reach.py`; scalar FK validated vs pinocchio
to 2.5e-16 in `aa_extract.py`) and compared per-link reachable-AABB volume to the true
(sampled) set and to the radial bound. Metric = bound-volume / sampled-volume (1 = tight).

## Result (`results/aa_reach.csv`)
| robot | region | AA/sampled | radial/sampled |
|-------|--------|-----------:|---------------:|
| Panda | full box | 25.7× | 7.1× |
| Panda | **corridor (15% ranges)** | **2.0×** | 3265× |
| UR5   | full box | 3.6× | 17.4× |
| UR5   | **corridor** | **1.8×** | 385× |

AA AABBs are **conservative** (contain the sampled extremes in every dimension) — valid bounds.

## Verdict
- **Corridor (JIT): AA wins decisively.** Tight (≤2× the true reachable set) while the radial
  bound is useless (385–3265×). This is the provable, query-time substrate the pruning levers
  need — far better than VAMP's radial early-exit over the queried region.
- **Full box (AOT): AA is unreliable** — beats radial for UR5 (6 joints) but blows up for Panda
  (7 joints × large ranges; AA error accumulates along the chain). AOT pruning should keep the
  sampled/radial bounds.

⇒ **Scope the reachability pass to the query corridor (start→goal + planner margin), not the
full joint box.** There it is provably tight and enables aggressive env-obstacle pruning,
self-collision pair-pruning, and a reachability filter for reordering — all from one pass.

## Notes / next
- The 2× is a first-cut AA (secant Chebyshev sin/cos, standard AA multiply); a refined AA
  (better sin/cos enclosures, symbol-reduction/recentering) would tighten toward 1×.
- Next: translate volume-tightness into a pruning factor on real corridors + obstacle sets,
  and (if pursued) port the AA evaluator to C++ / integrate with the JIT specialization path.

---

## Update — pruning-factor test (m15) walks back the volume result

Translating AA tightness into actual obstacle pruning over a **realistic start→goal
corridor** (per-joint box = |goal−start| + margin), `results/m15_aa_prune.csv`:

| robot | region | frac_kept AA / sampled / radial |
|-------|--------|--------------------------------|
| Panda | full box | 0.79 / 0.65 / 0.61 |
| Panda | corridor+0.1 | 0.76 / **0.42** / 0.59 |
| UR5 | corridor+0.1 | 0.61 / **0.23** / 0.96 |

**AA does NOT deliver.** A real start→goal corridor has a few joints spanning large
ranges (the ones that move, ~40%), and AA blows up on exactly those — the earlier
"2× tight" volume test used an artificial *uniform* 15%-range corridor. On real
corridors AA is far looser than sampled (UR5 0.61 vs 0.23) and for Panda's 7-joint
chain it's worse than radial. AA is conservative (a valid superset), just too loose.

**Fundamental tension = granularity:** AA is tight only over small joint ranges
(short path segments), but specialization must amortize over a whole query, which
spans large ranges. Sampled bounds *do* capture the corridor pruning (UR5 4.5×,
Panda 2.4× at tight corridor) but aren't provable.

**Verdict:** don't pursue AA/interval-FK. Env-pruning should use **sampled
reachability bounds with a safety margin + empirical validation** (Lever A already
showed 0 false negatives). The provability goal isn't reachable with standard AA;
it would need a fundamentally tighter method (Taylor models / per-edge bounds) whose
granularity doesn't fit query-level specialization.
