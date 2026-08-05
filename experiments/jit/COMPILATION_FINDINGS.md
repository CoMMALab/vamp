# Compiling the ideas into the kernel — exploration

The ccfk template has two compilation strategies (`ccfk_template.hh`):
- **unrolled** (non-compact): straight-line, per-link gate blocks baked in emission order.
- **compact**: data-driven — a small `constexpr` table `cc_env_links` (15 links: order +
  fine-sphere ranges) + `cc_self_pairs` (48). A generic compiled loop consumes them.

The compact table is the natural "trace-it-in" hook: specialize by permuting/shrinking the
table (once per scene/robot), with NO per-config runtime partition (what killed the runtime
broadphase, m17). Table is tiny (compact kernel 183 KB vs 4.8 MB unrolled) -> cheap to re-JIT,
or make the table settable for zero-recompile runtime reorder.

## m18 (Fetch, env+self, shelf N=40, frac 0.57)
| variant | fkcc | vs unrolled |
|---------|-----:|------------:|
| unrolled           | 650 ns | 1.00 |
| compact default    | 824 ns | 0.79 (21% slower) |
| compact reordered  | 797 ns | 0.82 (reorder = **1.03x**) |

- Data-driven compact is **21% slower** than unrolled (loop indirection). Unrolled is the
  perf baseline; compact buys flexibility, not speed.
- **Reordering the link table = 1.03x (3%)** — tiny.

## The key insight (reframes earlier results)
Reordering barely helps because **VAMP's per-link bounding gate is already a per-config
broadphase**: a non-colliding link costs one cheap bounding-sphere check, so reordering only
saves those. The gate captures most of what reordering/pruning would add.

This reframes the earlier "wins": Lever A 2-4x (m25) and reordering 1.46x (m13) were measured
against **un-gated manual per-sphere loops**. Against VAMP's real gated `fkcc`, the headroom is
small (m17 broadphase slower; m18 reorder 3%). The gated kernel is a hard baseline precisely
because its hierarchical bounding gates already do cheap, effective, per-config broadphase.

## Still open to explore (compilation angles vs the REAL gated kernel)
- Prune `cc_self_pairs` (self-collision is ~40% of the sparse-scene check; remove pairs
  impossible over the joint box). Cheap table shrink.
- Attack the dominant cost (FK ~260 ns) via edge continuity (incremental/interpolated FK).
- Const-fold + re-vectorize obstacles over SIMD lanes (vs configs) for collision-bound cases.

---

## Edge-continuity FK: the trig recurrence (m19) — first EXACT win

Sub-configs along an edge are collinear (theta_j(t) = a_j + t*delta_j), so per joint the
angle is an arithmetic progression and sin/cos obey an exact 3-term recurrence
(rotate by rake*delta_j between rakes). Compute rake 0 full-SIMD, recur the rest (4 mul +
2 add/step, SIMD over lanes).

| edge rakes | full trig | recurrence | speedup | max err |
|-----------:|----------:|-----------:|--------:|--------:|
| 2 | 131.5 ns | 71.0 ns | 1.85x | 3.6e-6 |
| 4 | 263.0 ns | 80.1 ns | **3.28x** | 4.2e-6 |
| 8 | 525.9 ns | 98.8 ns | **5.32x** | 4.2e-6 |

- **Exact** (error = float sin precision), **general** (any edge), **scales with edge length**.
- Leaf trig is ~25% of Fetch per-rake FK -> recurrence ~= **15-20% off FK** (~8-10% off a free
  edge). Modest, but the FIRST exact win vs the real kernel -- it attacks FK, the dominant
  UN-gated cost, which the bounding gates don't touch. Composes with everything.

**Integration (codegen):** emit FK taking precomputed sin/cos v[] as inputs; the motion
validator computes rake-0 trig full, recurs subsequent rakes, and calls FK-with-precomputed-
trig per rake. cricket already traces v[]=sin(x[j])/cos(x[j]) as the leaves -> hoist them out
of the per-rake FK and feed the recurrence. Next: wire it and measure the real FK saving.

**Deeper (open):** the full sphere positions along the edge are trig polynomials in t; a
higher-order recurrence could compute more of FK, but frequency count grows with chain depth
-- the leaf-trig recurrence is the clean, cheap version.

---

## Integrated FK recurrence (m20) + per-sphere question (m21)

**m20 (real sphere_fk, leaf trig hoisted + recurred):** FK speedup 1.13x (n=2) ->
1.23x (n=8), positions exact (err ~1e-6). Real, modest, exact, scales with edge length.

**m21 (can we recur each SPHERE, not just the trig?):** measured the Hankel rank (=
exact recurrence order) of each sphere position along the edge. Striking result:
**every sphere is order <= 5-6 even at range scale (1.0-1.5 rad edges); order 3 at 1e-4.**
Over a bounded edge the motion is near-constant-twist (a screw), so each sphere traces
DC + ~one sinusoid. So per-sphere direct recurrence is mathematically viable.

**But it does NOT beat FK**, and the reason is instructive: FK amortizes the kinematic
chain across all 111 spheres (shared subexpressions -> ~4 ops/sphere for placement).
Advancing each sphere position independently by its screw is a full transform (~15
ops/sphere), and there are 111 of them (+3 coords) -> ~5-7x more ops than FK's shared
placement. The shared chain wins.

**Conclusion:** the extractable edge-continuity win is specifically in the LEAVES --
the sin/cos, which are the *expensive* ops (~4 ns each vs ~0.5 ns/mul). The leaf-trig
recurrence (m20, ~1.2x FK) captures it; the rest of FK is cheap shared arithmetic that
recompute handles better than independent per-sphere recurrence. Caveat: for robots with
little chain-sharing (few spheres/link) or where trig dominates more, per-sphere/per-link
screw advance could win -- worth checking per robot.
