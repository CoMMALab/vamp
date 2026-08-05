# Per-edge broadphase — the right granularity (general, exact, tight)

The fkcc/motion-validation kernel is always called to validate an **edge** of bounded
length (≤ RRT `range`: ur5 1.5, panda 1.25, fetch 1.0, baxter 0.5 rad, L2). Within one
edge each robot sphere sweeps only a *bounded* region. So: compute each sphere's swept
AABB over the edge once, prune obstacles to it, and check all the edge's sub-configs
against only those. **Fully general** (any start A + vector v; nothing baked per scene),
and **exactly correct** — the kernel only checks the edge's discrete sub-configs, and the
swept AABB is the bbox of the sphere over exactly those sub-configs, so no collision can be
missed. No margin, no heuristic, no false-negative risk (unlike free-region estimation or
scene-baked pruning).

## Check-count measurement (`m16_edge_bp.py`, `results/m16_edge_bp.csv`)
Obstacle-checks per edge: current per-config radial early-exit vs per-edge broadphase.
| robot | edge L | sub-cfgs | check speedup | kept/sphere broadphase vs radial |
|-------|--------|---------:|--------------:|----------------------------------|
| Panda | 1.25 | 40 | 8.4× | 3.1 vs 131 |
| UR5   | 1.50 | 48 | **22.5×** | 4.2 vs 329 |
| Fetch | 1.00 | 32 | 15.9× | 2.6 vs 289 |

The swept AABB is 40–100× tighter than radial (directional + short motion) → ~1–4 of 500
obstacles kept per sphere; amortized over ~40 sub-configs it nets 8–22× fewer checks.

## Why this is the right lever (vs everything prior)
- **General**, not scene-baked: derived at runtime from (A, v). Works for any planner/query.
- **Exact**: conservative over exactly the checked sub-configs. No safety margin needed.
- **Tighter than full-box** (Lever A): the edge is short, so bounds are far tighter than
  per-scene joint-box reachability. Subsumes and beats Lever A.
- Correct granularity: the range is per-edge (too fine to *bake*), but the edge is exactly
  the kernel's call unit — so we optimize *inside* the kernel's own context, not across it.

## Caveats / next
- **Check-count ≠ time.** VAMP's radial+per-link-gate kernel is already SIMD/early-exit;
  the real speedup needs a C++ timing bench (this measures the pruning ceiling).
- **Flow change**: broadphase wants FK for the edge's sub-configs first (to bound the sweep),
  then pruned collision — vs the current fused compute-check-earlyexit. This helps FREE/long
  edges (full traversal) but the fused early-exit is better on edges that reject on
  sub-config 0. A real kernel may branch on edge length / do a cheap endpoint-based sweep
  bound (2 FK + deviation bound) to avoid full-FK-first. Timing bench is the decisive next step.
