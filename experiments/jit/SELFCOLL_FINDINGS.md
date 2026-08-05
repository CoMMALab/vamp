# Self-collision as a JIT/compilation target — findings

Follow-on to `PHASE2_FINDINGS.md` (FK gating refuted; self-collision identified as the
coupling that blocks it). Question: can compilation/JIT reduce self-collision itself?

## Cost (m10, `results/m10_selfcost.csv`)
Self-collision is **~40% of a check in sparse scenes** (136 ns of 351 ns — the
microsecond-planning regime), **~3% in dense** (env-collision dominates). Structurally
it is a **flat traversal of ~28–48 gated per-link-pair bound-checks**, done every config;
for the common free config the bound-checks all clear but are still all executed.

## Lever 1 — hierarchical self-culling (self-BVH): **REFUTED** (m12)
Idea: a coarse cluster-level bound (body / proximal-arm / distal-arm subtrees) checked
first, to cull a whole group of pair-checks at once. **Measured: coarse cluster bounds
overlap 100% of the time** over motion rakes — the arm is attached to the body and
sweeps close to it, so any multi-link enclosing sphere always overlaps. Adding the
cluster level does **more** work (117.9% of flat), never culls. There is no useful
intermediate granularity: the existing per-link-pair gating is already as coarse as you
can go and still separate.

## Lever 2 — reachability-based pair pruning: modest (m11, `results/m11_selfprune.csv`)
Drop pairs provably separated over the queried region (same interval-FK mechanism as
env-pruning). At coarse (9-link) granularity: **17% prunable over the full joint box**
(beyond the conservative SRDF — an AOT win), **27% over a start→goal corridor** (JIT).
Both under-estimate at finer granularity, but the geometry (arm sweeps near the body)
caps it. Net: ~17–27% fewer pair-checks → ~**10%** of the sparse-scene check. Real but
small, and it's the same reachability pass env-pruning already needs.

## Synthesis (the honest scope boundary)
VAMP's generated kernels are **already very well compiled**: tight per-sphere and
per-link-pair bounding-sphere gates, SRDF pair pruning, efficient SIMD FK. Across this
phase we found the exploitable compilation/JIT headroom is narrow:
- **FK reduction** (static hoist / bounding-gated FK): refuted (~1.5% / ≤1.00×).
- **Self-collision** (BVH / pair-pruning): BVH refuted; pruning ~10% (sparse).
- **Env-collision pruning (Lever A)**: the one clean win — 2–4× for **collision-bound**
  robots (UR5-class) under amortization; robot-dependent (broad-reach arms don't beat
  the existing early-exit; FK-bound robots gated by FK cost).

Recommended framing: the contribution is **scene-conditioned env-collision specialization
under a cost model / amortization**, with a rigorous characterization of *where it wins
and where the kernel is already optimal* (FK-bound robots, self-collision). The negative
results are scope boundaries that make the positive result credible. A unified
**interval-FK reachability pass in cricket** serves both env-pruning (primary win) and
self-collision pair-pruning (secondary), and is the single most reusable next build.
