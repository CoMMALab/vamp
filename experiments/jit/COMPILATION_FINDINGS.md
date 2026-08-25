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

---

## r2c6 test (m22): does per-sphere win for a high-DoF/low-sharing robot? NO.

r2c6 (36-DoF, 211 spheres, 29 revolute joints -> 58 leaf trig; median 3 spheres/link).
- Per-sphere recurrence order still <=3 even at range scale (near-screw holds).
- **Leaf-trig recurrence FK: 1.02x (n=2) -> 1.07x (n=8)** -- SMALLER than Fetch's 1.2x,
  because r2c6's FK is placement-dominated (211 spheres, deep chain, ~1360 ns/rake) so
  trig is only ~8.5% of FK (vs ~25% for Fetch). More trig, but even more placement.
- **Per-sphere / per-link screw advance still loses:** advancing 211 sphere positions
  independently (~15 ops each) or 65 link frames by screws (65 independent SE(3) mults)
  is more work than FK's shared chain (36 chained joint transforms + ~4-6 ops/sphere).

**Universal conclusion:** FK's shared kinematic chain is fundamentally more op-efficient
than any independent per-sphere/per-link recurrence, regardless of DoF or sphere count.
The extractable edge-continuity win is only the LEAVES (expensive sin/cos), and it's
LARGER for trig-heavy/placement-light robots (moderate sphere count) -- Fetch 1.2x,
r2c6 1.07x. The near-screw low-order structure (order<=3, robust across robots) is
mathematically elegant but not exploitable for speed given the shared chain.

---

## (d) Structured per-sphere position recurrence (m23): cheaper, but inaccurate

Challenge: advancing a sphere is just a point translation -> ~9 ops (1 mul + 2 add/coord
via p_{k+1}=a(p_k-p_{k-1})+p_{k-2}), not a 15-op transform. Confirmed on ops: seed 4 rakes
full + recur is 1.19x (n=8) -> 1.40x (n=16) faster than full FK.

BUT position error is 2.9 mm -> 1.6 cm (grows with edge) -- catastrophic for collision
(radii are cm-scale). NOT an alpha-conditioning issue (robust alpha didn't help). Two
fundamental causes:
  1. Some spheres are genuinely order 4 (not 3) at 1e-6 (m21) -> order-3 recurrence can't
     represent them.
  2. The recurrence is marginally stable (roots on the unit circle) -> any model mismatch
     ACCUMULATES over the edge.

Contrast with the leaf-trig recurrence (m20): sin(a+k*delta) is EXACTLY order-3 with an
EXACTLY-known coefficient 1+2cos(delta) (delta = edge step) -> exact, zero accumulation.
The sphere position is only approximately low-order with an ESTIMATED coefficient -> it
accumulates. **So the op-count was indeed too high (per-sphere is cheaper), but the
blocker is accuracy/stability, not ops. Exact collision needs the leaf-trig recurrence;
the sphere recurrence is a marginally-stable approximation that drifts to cm error.**

---

## (a)+(b) All robots at realistic RRTC edge lengths (m24)

RRTC extends by up to `range`; at resolution 32 an edge ~ ceil(range*32/rake) rakes.
Leaf-trig recurrence FK win, exact (err ~1e-7):
| robot | range | n_rakes | speedup |
|-------|------:|--------:|--------:|
| panda  | 1.25 | 5 | **1.33x** |
| fetch  | 1.0  | 4 | 1.22x |
| baxter | 0.5  | 2 | 1.23x |
| r2c6   | 1.0  | 4 | 1.05x |

Best for moderate-sphere workhorse arms; smallest for placement-dominated r2c6. Even
baxter's short (n=2) edges benefit (rake-0 full + 1 recurred). Realistic RRTC edges are
range-length extension steps, so these are representative; shorter connect/goal edges get
somewhat less.

---

## (c) Through the trace: cricket natively emits the compiled form

Wired the leaf-trig hoist into cricket's codegen (`src/codegen.cc`: regex-hoist
sin(x[j])/cos(x[j]) -> ps[j]/pc[j] from the traced spherefk code) + a `sphere_fk_pretrig`
in `fk_template.hh` (gated by `exists(spherefk_pretrig_code)`). `generate_robot_source`
now emits `sphere_fk_pretrig` natively. Verified (m25_native): the cricket-TRACED panda
kernel gives **1.30x FK, exact (err 1.3e-7)** driven by the recurrence -- matching the
post-transform result. The experiments are now demonstrated in their final compiled form.
Next for production: emit the recurrence preamble inside the generated motion validator
(compute rake-0 sin/cos, advance per rake) so validate_motion uses it end to end.

---

## Can the per-sphere error be absorbed as radius inflation? Conceptually yes, practically no (m26)

Inflating radius by the max deviation eps makes the approximate recurrence CONSERVATIVE
(no missed collisions -- the inflated approximate sphere contains the true sphere). Valid.
But eps must be small (cm inflation on cm-radius spheres over-rejects valid motions).

Measured the required inflation (= max accumulated position error) with crude vs best-fit
alpha:
| edge L | crude 4-pt alpha | best-fit alpha (LSQ over edge) |
|--------|-----------------:|-------------------------------:|
| 0.5    | 2.4 mm | 2.8 mm |
| 1.0    | 4.8 cm | 4.3 cm |
| 1.5    | 6.3 cm | 5.7 cm |

**Alpha quality doesn't help** -- the error is the order-3 MODEL, not estimation. Over a
range-length edge the twist is non-constant (not a single screw), so the true sphere motion
is genuinely order 4-5; the cheap order-3 (9-op) recurrence leaves cm residual. Getting to
sub-mm needs order 5-6 (~15-18 ops/sphere) ~ FK cost -> no win. Re-seeding bounds it but cuts
savings. Short edges (L<=0.3) would be mm/sub-mm (inflatable) but have few sub-configs (little
to save). So inflation is correct-but-impractical for realistic range-length edges. The EXACT
leaf-trig recurrence (leaves are exactly order-3 with exact coefficient -> no error, no
inflation) remains the right lever.

---

## What would order-4 take? (m27) — worse accuracy AND higher cost

Measured accumulated error for a general order-M per-sphere recurrence (coeffs by LSQ over
the edge), M=3..6, Fetch:
| edge L | M=3 | M=4 | M=5 | M=6 |
|--------|----:|----:|----:|----:|
| 0.5 | 16cm | 17cm | 28cm | 24cm |
| 1.0 | 30cm | 35cm | 20cm | 12cm |
| 1.5 | 49cm | 43cm | 41cm | 31cm |

Higher order is WORSE, not better -- fitting a general order-M recurrence to a near-low-order
signal is ill-conditioned, so the estimated coefficients put roots slightly OFF the unit
circle -> the recurrence is UNSTABLE and diverges over the steps. (The structured order-3
alpha form forces roots on the unit circle -> merely inaccurate, not divergent.)

**To do order-4 properly you must structure it: 2 sinusoids with KNOWN beat frequencies**
(roots pinned on the unit circle). That requires per-sphere beat-frequency analysis from the
kinematics (which sum/difference of joint rates dominates each sphere), and costs ~12-18
ops/sphere (2 phasors) -- comparable to FK's effective ~10-14 ops/sphere, so no speedup --
and it is STILL approximate (higher beats unmodeled -> needs inflation). Order M>=4 general
(LSQ) is 21-33 ops/sphere AND unstable.

**Bottom line:** there is no accurate-and-cheaper-than-FK sphere recurrence. FK's shared
chain is already at the op floor (~10-14 ops/sphere); any recurrence accurate enough
(stable order-4) costs about the same. The EXACT leaf-trig recurrence (known coefficient,
exactly-periodic leaves, roots exactly on the unit circle) is the only free lunch, and it's
the compiled lever.

---

## FLASK (polynomial edges): does the recurrence extend? (m28) Yes, but the win shrinks

FLASK edges are polynomial in t (position+velocity BCs -> cubic), so theta_j(t) is a
polynomial and sin(theta_j(t)) is a CHIRP, not a fixed sinusoid -- the linear-edge
recurrence breaks. It generalizes EXACTLY via the phasor cascade (finite-difference chirp
generator): maintain d+1 coupled unit phasors (d = polynomial degree), update w_m *= w_{m+1}
(m=0..d-1) per step -> w_0 = (cos,sin) exactly. Uses FLASK's known polynomial directly
(phase + velocity + accel = the forward differences). NOT spectral analysis -- a chirp has
continuous spectrum; the finite-difference phasor cascade is the exact tool.

| edge | degree | n=16 | n=48 |
|------|-------:|-----:|-----:|
| RRTC linear | 1 | 1.59x | 3.51x |
| FLASK cubic | 3 | 0.50x | 1.17x |
| quintic     | 5 | 0.22x | 0.61x |

**Counterintuitive:** FLASK's richer (polynomial) structure makes this HARDER, not easier.
Each degree adds a phasor -> d complex-mul/step, eroding the savings vs a single sin+cos. For
cubic it only wins on long edges (1.17x trig -> ~1.04x FK); quintic loses. And the phasors
drift off the unit circle (~n*eps: 2e-4 at n=48 cubic) -> needs periodic renormalization to
stay exact (more cost). The big win is specifically the LINEAR RRTC case (single fixed
rotation). So: yes it extends, elegantly and exactly, but the practical FK gain for FLASK is
marginal-to-negative unless edges are long -- the linearity of RRTC edges is what made it pay.

---

## Tricks to buy back accuracy on long edges (m29 + analysis)

Two distinct error sources on long edges, different fixes:

**(1) FP drift (phasor cascade leaves the unit circle): SOLVED by renormalization.**
Every ~8 steps apply w *= (3-|w|^2)/2 (first-order Newton for 1/sqrt, ~4 ops/phasor, no
sqrt). Cubic cascade error: no-renorm 2.3e-4 (n=48) / 4.6e-3 (n=192) -> with renorm ~1e-6
to 1e-5, flat in n. Cost ~0.5 op/phasor/step (negligible). So the cascade is exact-to-float
over arbitrarily long edges. (Alternatives: double-precision phasor state; compensated sum.)

**(2) Model error (per-sphere order-3 recurrence is order 4-5): only bounded, not cheaply fixed.**
- Re-seeding/restart every K rakes: bounds error to the per-segment (short-segment) error
  (~sub-mm), but the order-3 sphere recurrence only saves ~33%/rake vs FK (FK is already
  efficient at ~10 ops/sphere), and re-seeding erodes that toward ~= the EXACT leaf-trig
  recurrence (1.3x) -- while remaining approximate + needing inflation. Not a net win.
- Structured higher order (2 known beat frequencies): stable but ~FK cost.
- Defect/Richardson correction at checkpoints: marginal.

**Bottom line:** renormalization cleanly buys back the FP drift (the phasor-cascade / FLASK
long-edge issue). The per-sphere MODEL error has no cheap fix -- bounding it (re-seeding)
costs about what it saves. The EXACT leaf-trig recurrence (linear RRTC edges) needs NONE of
these -- it's exact by construction -- which is exactly why it, not the sphere recurrence,
is the lever.

---

## End-to-end: leaf-trig recurrence through the real fused kernel (m30)

The leaf-trig recurrence is now emitted NATIVELY by cricket's trace (not a hand-patched
header): `fkcc_pretrig(env, x, ps, pc)` -- the fused FK+CC kernel with `sin(x[j])/cos(x[j])`
regex-hoisted to `ps[j]/pc[j]` inputs (codegen.cc after the ccfk trace; template gated on
`exists("ccfk_pretrig_code")`). `gen_e2e.py` regenerates PandaE/FetchE; both emit it and
`sphere_fk_pretrig`, each with trig_of_x=0, ps/pc=14 (fully hoisted).

m30 validates random RRTC-style edges two ways on a 40-obstacle front-shelf scene, at
realistic edge length n=ceil(range*32/rake) rakes:
  - baseline: loop rakes, `fkcc(env, block)` (one sin/cos batch per rake)
  - recurrence: rake 0 -> `ps=sin,pc=cos`; rakes 1..n-1 -> exact complex advance
    (ps'=ps*C+pc*S, pc'=pc*C-ps*S, C/S = cos/sin of the 8-lane step); `fkcc_pretrig`
Verdicts bit-exact (mm=0, both robots), verified per edge.

| robot | dim | n | edge type | fkcc ns | recur ns | speedup |
|-------|----:|--:|-----------|--------:|---------:|--------:|
| panda | 7 | 5 | all       | 2359 | 2157 | 1.07x |
| panda | 7 | 5 | free      | 2656 | 2428 | 1.09x |
| panda | 7 | 5 | colliding | 830  | 779  | 1.06x |
| fetch | 8 | 4 | all       | 2344 | 2250 | 1.06x |
| fetch | 8 | 4 | free      | 2801 | 2672 | 1.03x |
| fetch | 8 | 4 | colliding | 985  | 961  | 1.03x |

**Honest system-level number: ~1.03-1.09x.** The isolated-FK figure (panda 1.33x) collapses
because (a) collision checking is now IN the timed path and dominates -- fetch's 111 spheres
dilute more than panda's 59, so fetch < panda; (b) real edges are short (n=4-5 rakes), so the
rake-0 seed still pays a full sin/cos and is a large fixed fraction of a 4-5 rake edge;
(c) colliding edges early-exit before most FK runs, so they benefit least. Free edges (full
traverse) benefit most, as expected. The win is exact and free (no accuracy cost, native in
the trace) -- but at the system level it is single-digit percent, not the isolated-FK 1.3x.
The remaining time is collision (broadphase + narrowphase sphere tests), which the trig
recurrence does not touch -- that is where a bigger end-to-end lever would have to come from.

---

## Where the free-edge time actually is (m31) -- the collision-side ceiling

Decompose a free-edge kernel: FK-only (`sphere_fk`, spheres forced live) vs full FK+CC
(`fkcc`), same edges, n=ceil(range*32/rake) rakes, 40-obstacle shelf.

| robot | n | FK-only | FK+CC | collision | collision % | cc-lever ceiling |
|-------|--:|--------:|------:|----------:|------------:|-----------------:|
| panda | 5 | 956 ns  | 2609 ns | 1653 ns | 63% | 2.73x |
| fetch | 4 | 997 ns  | 2747 ns | 1750 ns | 64% | 2.75x |

**Collision is ~63% of a free-edge kernel -- nearly 2x the FK cost.** The leaf-trig
recurrence attacks only the FK third (and only its trig-leaf slice), which is exactly why
the end-to-end win is ~1.05-1.09x. The real lever is the COLLISION side: a perfect
collision elision would give up to ~2.7x. The FK-side lever is now exhausted (exact + native,
but small); the open direction is compiling/tracing the collision check per-edge.

### Next compile/trace target: per-edge swept broadphase
VAMP already gates per-link (bounding sphere vs env) per rake, descending to per-sphere
narrowphase only on a hit -- so a free edge is mostly n x links broadphase tests. The edge
structure lets cricket emit a swept variant: for each link, enclose the bounding-sphere
CENTER trajectory over the whole edge in one inflated sphere (endpoint centers + chord/sagitta
bound) and test the environment ONCE. If clear, skip all n rakes of that link's broadphase
AND narrowphase. Needs only endpoint bounding-sphere FK (2 configs, bounding subset) -- a
`bound_fk` cricket could trace. Open question to probe first: on realistic free edges, what
fraction of (link x edge) stay entirely clear of the environment (skippable), and how much
inflation is needed to stay conservative -- that bounds the achievable slice of the 2.7x.

---

## Skippability + conservative advancement (m32) -- two levers, one wins

Probe of both user questions on free edges (40-obstacle shelf, true min clearance = min over
all leaf spheres x all obstacles; per-joint reach r_j from finite-diff FK; M = sum_j r_j*|dtheta_j|
= Cartesian sweep of the edge).

### Q1 skippability -- whole-robot NO, per-sphere YES
| view | panda | fetch |
|------|------:|------:|
| median min-clearance over free edge | 0.289 m | 0.173 m |
| whole-edge certified by 1 midpoint check | 0.8% | 0.8% |
| **per-sphere swept-certifiable (skip all n rakes)** | **95.9%** | **94.2%** |

A full-range RRTC edge sweeps ~1 m of Cartesian space (reach ~1 m/rad x ~1 rad) while
clearance is ~0.2 m, so the *closest* link kills any whole-robot skip. But ~95% of leaf
spheres never approach an obstacle -- one swept test (enclose the center trajectory: center =
midpoint, radius rho = max deviation; certified iff clearance(center) > rho) certifies each
for the WHOLE edge. Only ~5% (the approaching link) need per-rake checks. This is the
swept-broadphase lever, quantified: on a free edge ~95% of per-sphere collision work is
skippable with one test.

### Q2 "skip subsequent rakes if far from obstacles" = conservative advancement -- does NOT pay
CA: at clearance d, all configs within joint-motion d/M are provably free; jump d/M, re-eval.

vs edge LENGTH (range 0.5..2.5): CA evals AND n_blocks both scale with length (M ~ range ~ n),
so block-skip stays flat at ~0.6-0.8x -- **CA never wins as edges lengthen**.

vs RESOLUTION at fixed length 1.25 (M fixed, n = resmul x native):
| n_blocks | CA evals (loose-M) | block-skip |
|---------:|-------------------:|-----------:|
| 5  (native res 32) | 7.08 | 0.71x |
| 10 | 7.11 | 1.41x |
| 20 | 7.10 | 2.82x |
| 40 | 7.09 | 5.64x |

CA cost is ~M/clearance ~= 7 checks INDEPENDENT of sampling density. So CA only wins ABOVE
native resolution. **Why it doesn't help VAMP:** the 8-wide SIMD rake already bundles ~8
configs per check, which is about the CA skip distance at these clearances (config spacing
~M/N ~ 3 cm, clearance ~25 cm -> skip ~8 configs = 1 rake). VAMP's fixed resolution 32 sits
right at the point where per-config CA stops paying -- the rake captures the temporal-skip win
structurally, for free, via SIMD. A distance query (costlier than a broadphase block) can't
beat it. (CA would pay 1.4-5.6x only if VAMP had to check at 2-8x finer resolution, e.g. very
thin obstacles.)

**Takeaway:** temporal skipping (CA) is already done by the rake; the open lever is SPATIAL --
the per-link swept broadphase (skip the ~95% of links that stay clear across the whole edge).
Next: measure it at the BOUNDING-SPHERE level (VAMP prunes per-link, not per-leaf) via a
traced `bound_fk`, then build the swept-broadphase kernel.

---

## Per-LINK swept certifiability (m33) -- the number the swept broadphase actually acts on

VAMP prunes per-link bounding sphere, so the swept lever acts per link, not per leaf. Group
leaves by link (cricket per_link_spheres), build each link's bounding sphere per config
(centroid + enclosing radius), enclose the bs-center trajectory over the edge (center=midpoint,
rho=max deviation, R=max bs radius), certify free for the whole edge iff clearance(center)-R > rho.

| robot | links | per-LINK swept-certifiable | (per-sphere ref) | broadphase tests eliminated |
|-------|------:|---------------------------:|-----------------:|----------------------------:|
| panda | 11 | **93.5%** | 95.9% | ~74.8% |
| fetch | 15 | **85.1%** | 94.2% | ~63.8% |

Per-link is a bit below per-leaf (larger spheres sweep more, smaller clearance) but still
dominant: on a free edge ~85-94% of links are certified clear for the ENTIRE edge by one swept
test, so their per-rake broadphase (n tests -> 1) is eliminated -- ~64-75% of broadphase work.
With collision = 63% of the free-edge kernel (m31), this is the lever with real headroom.

**Cost model / next build.** The win requires computing each link's swept bound CHEAPLY (not
by FK-ing all n rakes). The swept bound needs: (a) the bs-center trajectory extent rho, and
(b) max bs radius R. Both are bounded analytically from the two endpoint configs + a curvature
(sagitta) term -- exactly what a traced `bound_fk` would emit: endpoint bounding-sphere FK
(2 configs, ~links spheres) + a per-link rho/R bound. Kernel sketch:
  1. bound_fk at edge endpoints -> per-link bs center_0, center_1, R_0, R_1.
  2. per link: e = midpoint, rho <= |center_1-center_0|/2 + sagitta, R = max(R_0,R_1)+growth.
  3. one sphere_environment test per link on the inflated (R+rho) swept sphere.
  4. certified links: skip entirely. Uncertified (~1-2 links): fall back to per-rake fkcc.
Ceiling: collapses the ~63% collision slice roughly in proportion to the certified fraction,
net of the 2-endpoint bound_fk. Open risk: the sagitta/growth bound's tightness (too loose ->
fewer certified); measure certified% under the analytic (endpoints-only) bound vs the exact
trajectory bound used here.

---

## De-risking the swept bound (m34) -- Lipschitz remainder was the only problem

The swept kernel must derive each link's bound (rho = trajectory extent, R = max bs radius)
from a few sampled configs, not all n rakes. Sweep K = 2,3,5,9 samples, two remainder models:
 - Lipschitz: rem = M_link*gap/2, M_link = sum_j reach[link][j]*|dtheta_j| (worst-case reach).
 - none (sampling ceiling): rho = max sampled deviation from midpoint, no safety term.

| robot | exact | Lipschitz K=2/3/5/9 | **no-remainder (ceiling) K=2/3/5/9** |
|-------|------:|---------------------|--------------------------------------|
| panda | 93.5% | 35 / 50 / 72 / 84%  | **93.5 / 93.5 / 93.5 / 93.5%** |
| fetch | 85.1% | 44 / 55 / 69 / 77%  | **85.1 / 85.1 / 85.1 / 85.1%** |

**The endpoints alone (K=2, no remainder) already hit the exact certified rate.** The
bounding-sphere CENTER trajectory of a rigid link over a short RRTC edge barely bulges off its
chord -- max deviation ~= endpoint deviation -- so 2 samples capture the extent. The Lipschitz
remainder was pure over-conservatism: it inflates rho by the worst-case sweep (M_link/2), which
a fast-but-far link never realizes, dropping certification to 35-44% at K=2.

**Consequence: the swept broadphase is viable and cheap.** Use a curvature-aware (2nd-difference
/ sagitta) remainder, NOT Lipschitz: from endpoints + midpoint bounding-sphere FK (K=3), the
observed sagitta |c0 - 2*cmid + c1| bounds the between-sample bulge to a few mm, so certified
rate ~= exact 85-94% rigorously. Cost = 2-3 bounding-sphere FK (11-15 spheres) + 1 env test per
link, vs n=4-5 SIMD broadphase tests per link today.

**Build plan (traced swept broadphase / `bound_fk`):**
  1. cricket emits `bound_fk(x)` -> per-link bounding sphere center+radius (link transforms only,
     no leaves). Evaluate at edge endpoints t=0,1 and midpoint t=0.5 (K=3).
  2. per link: e = c(0.5); rho = max(|c0-e|,|c1-e|) + |c0-2*e_chord+c1|/c ; R = max(R0,Rm,R1)+same.
  3. one sphere_environment test on the inflated (R+rho) swept sphere. Clear -> skip the link's
     whole-edge broadphase AND leaf narrowphase; else fall back to per-rake fkcc for that link.
**Rigor (build-time):** validate on COLLIDING/near edges that the swept bound NEVER skips a link
that actually collides (inflate the sagitta remainder until zero false-skips over a large
colliding-edge set). This probe only measured the certified RATE on free edges (the opportunity).

---

## Env vs self split (m35) -- the swept-env ceiling, and extending swept to self

The swept broadphase certifies clearance from the ENVIRONMENT; self-collisions are separate.
Decompose a free-edge kernel: FK (sphere_fk) | self (fkcc on EMPTY env - FK) | env (full - empty).

| robot | FK | self | env | swept-ENV-only ceiling |
|-------|---:|-----:|----:|-----------------------:|
| panda | 35% | 33% | 32% | 1.48x |
| fetch | 36% | ~0% | 68% | 3.14x |

(fetch self ~0 is real -- few/cheap self-pairs; the -4% is timing noise.) The earlier
"self-collision is the bottleneck" was scene-specific (self-tangled configs); on front-shelf
FREE edges env dominates for fetch and env~=self for panda. So swept-ENV alone caps at ~1.5x
(panda) to ~3.1x (fetch).

**Extend swept to SELF using the same bound_fk output:** a self-pair (link A vs B) is skippable
over the edge iff their swept bounding spheres stay disjoint (dist(eA,eB) > rhoA+RA+rhoB+RB).
Same per-link swept spheres, no extra FK. Covering env AND self raises the ceiling to the full
collision slice ~2.7-2.8x (m31). This is the build target.

---

## Swept broadphase BUILT and measured end-to-end (m36) -- rigorous, compact-penalty-limited

Traced two new cricket kernels: `bound_fk(x)` (per-link bounding spheres only, link transforms,
no leaves) and `fkcc_swept(env, x, env_clear, pair_clear)` (compact FK+CC that skips env links /
self pairs a per-edge swept test proved clear, keyed by bounding-sphere index). The validator
computes masks ONCE per edge: bound_fk at 3 configs (t=0,0.5,1) -> per-bs swept sphere (center
= midpoint, radius = max endpoint deviation + curvature remainder*SAFETY + max bs radius);
env_clear[bs] = swept sphere clear of obstacles; pair_clear[pi] = the pair's swept spheres
disjoint. Then fkcc_swept per rake.

Verdicts vs unrolled baseline, 3000 edges (free + colliding), SAFETY=2.0: **mismatch=0,
unsafe=0** -- the swept mask NEVER passes a colliding edge. Rigorous.

| robot | edge | unrolled (ships) | compact | swept | swept/unrolled | swept/compact |
|-------|------|-----------------:|--------:|------:|---------------:|--------------:|
| panda | free | 2610 | 4129 | 4056 | 0.64x | 1.02x |
| fetch | free | 2769 | 3500 | 2563 | **1.08x** | 1.37x |
| panda | colliding | 794 | 1198 | 1544 | 0.51x | 0.78x |
| fetch | colliding | 986 | 1245 | 1474 | 0.67x | 0.84x |

**The mask works (swept/compact 1.24-1.37x on free edges) but the COMPACT-mode kernel it is
built on is 1.5-1.6x slower than VAMP's UNROLLED kernel.** That penalty eats the algorithmic
savings on panda; only fetch (48 self-pairs, much to skip) nets +8% over the shipping kernel.
Colliding edges lose (bound_fk setup unamortized when the baseline early-exits).

### Why compact loses and what the full win needs
- **Unrolled masked kernel.** The mask must live in the UNROLLED ccfk (each per-link `if` block
  wrapped in `if (!env_clear[bs])`), keeping unrolled speed AND skipping certified links. The
  runtime branch is constant across the edge's rakes (well-predicted). This removes the 1.5-1.6x
  compact tax -> free-edge estimate ~1.5-2.2x (skip ~85% of collision = 65% of kernel).
- **Lazy leaf FK.** fkcc_swept still computes ALL leaves every rake though ~85% of links are
  skipped; leaves of skipped links are never read. Compute only bounding spheres per rake
  (bound_fk-style) + leaves lazily for active links -> also reclaims the FK third.
- **Colliding-edge guard.** Skip the swept setup when cheap (short edges) or fold bound_fk's
  first sample into rake 0; the setup only pays on longer free edges.
- **rho=0 sanity:** SAFETY=2.0 on the observed sagitta gave 0 unsafe over 3000 edges; a
  build must sweep SAFETY on a large colliding set to fix the conservative margin.

---

## Unrolled masked swept kernel (m36, unrolled) -- the real win

Moved fkcc_swept from compact loops to the UNROLLED form: each per-link env block / per-pair
self block wrapped in a runtime mask guard `if (not env_clear[bs])` / `if (not pair_clear[pi])`,
keeping VAMP's unrolled speed. Now fkcc, fkcc_swept, and bound_fk all live in ONE (non-compact)
class -> apples-to-apples vs the SHIPPING fkcc. Verdicts vs baseline, 3000 edges (free+colliding),
SAFETY=2.0: mismatch=0, unsafe=0.

| robot | free | colliding | all |
|-------|-----:|----------:|----:|
| fetch | **1.50x** | 0.83x | 1.36x |
| panda | 1.04x | 0.70x | 1.05x |

**Fetch: real 1.50x on free edges over the shipping unrolled kernel** (111 spheres, 48 self-pairs,
env=68% of kernel -> eliding ~85% of it is huge). **Panda: modest 1.04x** -- FK-dominated (35%)
with few links (11)/pairs (21) and a smaller collision base, so the bound_fk setup (1 SIMD FK of
11 bounding spheres + 11 env + 21 pair swept tests) erodes the win. Both **regress on colliding
edges** (0.70-0.83x): the swept setup is paid even when the baseline early-exits after ~1-2 rakes.

Blended 50/50 free:colliding -> fetch ~1.24x net, panda ~0.93x (net loss). So the swept
broadphase is a clear win for high-sphere robots (fetch, and by extension baxter) and marginal/
negative for low-sphere robots (panda) until two levers land:
- **Lazy leaf FK** (reclaim the 35% FK): fkcc_swept still computes ALL leaves every rake though
  ~85% of links are skipped and their leaves are never read. Compute bounding spheres per rake +
  leaves only for active links. Biggest help exactly where the swept win is small (panda).
- **Colliding-edge amortization**: fold bound_fk's first sample into rake-0 FK, or gate swept on
  edge length, so short/colliding edges don't pay the full setup. Removes the regression.
Both are the difference between "wins on fetch" and "wins everywhere".

---

## Colliding-edge amortization attempt (m36) -- backfires

Tried "rake-0-plain": run rake 0 with plain fkcc (no swept setup); only if it passes pay the
setup + run rakes 1..n-1 masked, so early collisions cost nothing extra.

| variant | panda free | panda coll | fetch free | fetch coll |
|---------|-----------:|-----------:|-----------:|-----------:|
| plain swept  | 1.05x | 0.71x | 1.46x | 0.80x |
| rake-0-plain | 0.96x | **0.89x** | 1.15x | **0.93x** |

It DOES help colliding (0.71->0.89, 0.80->0.93) but running 1 of 4-5 rakes unmasked robs
20-25% of the FREE-edge win, and free edges dominate total cost -> net "all" gets worse
(fetch 1.33->1.13). Bad trade; reverted to plain swept (kept in m36 as a documented variant).

**Root cause (same shape as the CA finding):** swept setup (bound_fk + swept tests) is O(1)
per edge; the savings is O(n) over the rakes. At VAMP's short edges (n=4-5) the setup barely
amortizes, and colliding edges early-exit before it can. Rake-0-plain just relocates the cost
onto the free edges that were paying off.

**The real fixes (don't rob free edges):**
- **Lazy leaf FK** -- reduces per-rake cost for ALL edges (compute leaves only on a bounding-
  sphere hit), so the setup amortizes better AND panda's 35% FK is reclaimed. This is the lever.
- **Planner-level bound_fk caching** -- each tree node is an endpoint of MANY edges; bound_fk at
  a node computed once and reused amortizes 2/3 of the setup (the two endpoint samples) across
  all its edges. Architectural (RRTC integration), not a per-edge-bench change.

---

## Lazy-leaf-FK ceiling (m37) -- helps fetch, NOT panda

Before building a lazy kernel: on free edges, how often would it need leaves (a non-certified
link's bounding sphere actually hits env, or a non-certified pair's bounding spheres overlap)?

| robot | rakes needing leaves | leaf-FK reclaimable | edges w/ 0 leaf-rakes |
|-------|---------------------:|--------------------:|----------------------:|
| panda | 100% | 0% | 0% |
| fetch | 42% | 58% | 40% |

**Panda's leaves are genuinely needed.** Its bounding spheres are loose (11 large links), so on
every free-edge rake SOME link's bounding sphere overlaps an obstacle -> broadphase descends ->
leaves used. VAMP's eager leaf FK is JUSTIFIED for panda; lazy FK reclaims nothing. This also
explains panda's weak swept win: it isn't wasting leaf FK, and its collision base is small.

**Fetch reclaims 58%** (finer spherization -> tighter bounding spheres -> fewer false broadphase
hits; 40% of edges never need leaves at all). So lazy FK, like the swept broadphase, is a lever
for FINELY-SPHERIZED robots. Points squarely at baxter (highest sphere count) as the ideal
target -- both levers should be strongest there.

---

## Baxter + the swept sweet spot (m36/m37) -- setup amortization is the binding constraint

Added baxter (dim 14, bimanual: 33 bounding spheres, 349 self-pairs, 75 leaves, RRTC range 0.5
-> n=2 rakes). All three robots, unrolled swept kernel, SAFETY=2.0, mismatch=0/unsafe=0:

| robot | n | bs | pairs | free | colliding | all | lazy-FK reclaimable |
|-------|--:|---:|------:|-----:|----------:|----:|--------------------:|
| panda | 5 | 11 | 21  | 1.08x | 0.68x | 1.04x | 0%  |
| fetch | 4 | 15 | 48  | **1.48x** | 0.80x | **1.34x** | 58% |
| baxter| 2 | 33 | 349 | 1.09x | 0.48x | **0.92x** | 58% |

**Baxter has the MOST to skip (349 self-pairs) but nets a LOSS.** The swept setup is O(bs+pairs)
per edge -- for baxter ~349 pair swept-tests + 33 bounding-FK, comparable to a whole rake of
work -- and baxter's edges are only n=2 rakes (range 0.5), so the setup can't amortize.
Colliding edges (setup wasted before early-exit) crater to 0.48x, dragging the net to 0.92x.

**The swept broadphase wins only in a sweet spot: moderate pair count AND enough rakes to
amortize the O(bs+pairs) setup.**
- fetch (48 pairs, n=4): hits it -> +48% free, +34% all. The clear win.
- panda (21 pairs, n=5, but coarse spheres/loose bounding -> little to skip + 100% leaf-need): flat.
- baxter (349 pairs, n=2): setup-bound -> net loss.

So the original "baxter is more interesting (higher sphere count)" hypothesis is only half right:
higher sphere count gives more to skip, but baxter's SHORT RRTC edges (range 0.5 -> n=2) defeat
the per-edge setup. Baxter would win with (a) longer edges, or (b) planner-level caching of
bound_fk + masks at tree nodes so the setup is shared across the many edges from each node --
that is the lever for high-pair robots, and it is an RRTC-integration change, not a kernel one.

### Net synthesis of the swept-broadphase thread
- Traced natively: `bound_fk` (bounding spheres only) + `fkcc_swept` (unrolled, per-link/pair
  mask guards). Rigorous: 0 unsafe skips over 9000 edges (3 robots x 3000, incl colliding).
- **Fetch: real +34% (free +48%) over the shipping kernel.** The deliverable.
- Panda flat, baxter net-loss -- both for structural reasons (leaf-need / setup-amortization),
  not bugs. Lazy leaf FK adds 58% leaf-FK reclaim for fetch/baxter (not panda).
- The single biggest remaining lever for BOTH baxter and colliding edges is amortizing the
  setup across edges (node-level bound_fk/mask caching in RRTC), not more kernel work.

---

## Node-level caching prototype (m38) -- flips baxter to a win

In RRTC an edge connects two EXISTING tree nodes, so both endpoints' bound_fk are cacheable.
Using endpoints-only swept spheres (m34: 2 samples capture the extent) + a per-link sagitta
remainder calibrated offline (C[k] = max bulge |c_mid-(c0+c1)/2| over sampled edges; REM=1.5x
safety), the per-edge setup needs ZERO FK -- both endpoint bounding spheres are cache hits.
Rigorous: mismatch=0, unsafe=0 across 9000 edges (3 robots x 3000, incl colliding).

| robot | free | colliding | all | vs per-edge swept (m36 all) |
|-------|-----:|----------:|----:|----------------------------:|
| panda | 1.08x | 0.75x | 1.03x | 1.04 -> 1.03 (flat) |
| fetch | 1.49x | 0.88x | **1.37x** | 1.34 -> 1.37 |
| baxter| 1.23x | 0.58x | **1.07x** | **0.92 -> 1.07 (loss -> win)** |

**Baxter flips from net-loss to net-win.** Node-caching removes the per-edge bound_fk (33
bounding spheres over a dim-14 chain), which was baxter's setup killer; free 1.09->1.23x, all
0.92->1.07x. Fetch's colliding profile also improves (0.80->0.88x). Panda stays flat (its cost
is leaf-need, not setup). Baxter colliding is still 0.58x -- the 349 per-pair swept tests remain
per-edge overhead (they depend on both endpoints, not node-cacheable); vectorizing them or a
cheaper pair-prune is the next step there.

**Amortization model.** The cache holds one bound_fk per node; an edge is 2 cache hits + masks +
fkcc_swept, 0 FK. Break-even is immediate: node-caching replaces the per-edge bound_fk that the
non-cached swept already paid, and each node is shared by many edges (RRTC connection attempts).
So the amortized limit measured here is the realistic operating point.

**Rigor caveat / production path.** C[k] is a calibrated max sagitta + 50% margin -- a heuristic
bound, not a proof. Production rigor needs an ANALYTIC per-link sagitta bound (max Cartesian
curvature x edge_angular_length^2 / 8), calibrated once at the planner's MAX edge length
(conservative for shorter edges). That makes the endpoints-only swept sphere provably
conservative without any midpoint FK.

---

## (a) vectorized swept tests + (b) provable analytic sagitta (m39)

Two upgrades on the node-cached swept broadphase (m38):

**(b) Provable analytic sagitta -- no calibration.** The endpoints-only swept sphere needs a
sagitta (chord-bulge) remainder. Linear-interp error gives sagitta_k <= (1/8) max||c_k''(t)||;
for affine joint motion c'' = dtheta^T H dtheta, and bounding the Hessian by per-joint reach
yields the PER-EDGE bound  sag_k <= (sum_j r_k[j]|dtheta_j|)(sum_j |dtheta_j|)/8 = M_link*Omega/8,
where r_k[j] is bounding-sphere k's Cartesian displacement per radian of joint j (one FD sweep,
offline). Verified conservative: worst true/bound ratio 1.76 (panda) / 2.32 (fetch) / 2.80
(baxter) -> bound always >= true sagitta, 0 unsafe. (The earlier worst-case-direction form
||r_k||*sqrt(dim)*L^2/8 was 3-5x loose and killed certification; the per-edge form is 1.8-2.8x.)

**(a) Vectorized swept tests.** The per-edge mask setup is 33 bs x 40 obstacles (env) + 349
pairs (baxter). Rewritten SoA + squared distances + branchless (clear iff dist^2 > (RAD+r)^2),
so the obstacle/pair loops autovectorize. The ENV test (not the pairs) was the hog.

Final (node cache + analytic bound + vectorized tests), rigorous (0 unsafe / 9000 edges):

| robot | free | colliding | all | m38 all | m36 all (per-edge) |
|-------|-----:|----------:|----:|--------:|-------------------:|
| panda | 1.08x | 0.83x | 1.05x | 1.03x | 1.04x |
| fetch | 1.36x | **0.98x** | 1.29x | 1.37x | 1.34x |
| baxter| **1.50x** | 0.78x | **1.31x** | 1.07x | 0.92x |

**Baxter: 0.92x (per-edge, m36) -> 1.31x** via node caching (b removes per-edge FK) + vectorized
tests (a removes the setup hog). Colliding regression is now mild (fetch break-even 0.98x,
baxter 0.78x). Rigor is provable (no calibration). fetch dips slightly from m38's calibrated
1.37x because the provable bound is ~2.3x looser than the observed-max; a tighter provable bound
would need the actual Hessian norm (diminishing returns). Panda stays flat (leaf-need bound).

**State of the swept broadphase:** traced natively (bound_fk + fkcc_swept), node-cached,
provably rigorous, vectorized -> fetch +29% / baxter +31% all-edges, free +36-50%, colliding
~break-even. The one structural non-target is panda (coarse spherization -> leaves always
needed). Remaining polish: tighter provable sagitta (Hessian) and lazy leaf FK (fetch/baxter
58% leaf reclaim, orthogonal).

---

## (b') Tighter Hessian sagitta bound (m39) -- wins for fetch, dim^2 cost hurts baxter

Replaced M_link*Omega/8 with the actual Hessian-norm quadratic form:
  sag_k <= (1/8) sum_{i,j} H[k][i][j] |dth_i||dth_j|,  H[k][i][j] = max_config ||d2 c_k/dth_i dth_j||
precomputed per bounding sphere via second finite differences (eps=0.05 to dodge float32
cancellation -- eps=3e-3 under-estimated H ~20x). Conservative with SAFETY=1.3 (covers FD error):
worst true/bound over MOVING spheres (sag>3mm) = 1.3x, 0 unsafe. (The raw ratio is polluted by
near-stationary base spheres where sag~0 and H~0 -> 0/0; they are far from obstacles, never skip.)

| robot | dim | M*Omega bound (free / all) | Hessian bound (free / all) |
|-------|----:|---------------------------:|---------------------------:|
| panda | 7  | 1.08 / 1.05 | 1.10 / 1.07 |
| fetch | 8  | 1.36 / 1.29 | **1.57 / 1.44** |
| baxter| 14 | **1.50 / 1.31** | 1.13 / 0.95 |

**Tighter bound is a win for fetch (1.29->1.44x all) but net-negative for baxter:** the per-edge
quadratic form is O(NBS*dim^2) = 33*196 for baxter, and that setup cost exceeds the certification
gain (all 1.31->0.95x, colliding 0.78->0.52x). fetch (dim 8) is cheap enough (15*64) to profit.
So use the tighter bound for moderate dim, the O(dim) M*Omega bound for high dim. A low-rank
approx of H (top 2-3 eigenvectors -> ad^T H ad ~ sum_r lambda_r (w_r.ad)^2, O(dim*rank)) would
make the tight bound cheap even at high dim -- the clean follow-up.

**Best-of per robot (rigorous, node-cached, vectorized):** panda 1.07x, fetch 1.44x (Hessian),
baxter 1.31x (M*Omega) all-edges; free 1.10 / 1.57 / 1.50x.

---

## (1) Lazy leaf FK -- backfires (m39 fkcc_swept_lazy, m40 FK cost)

Built fkcc_swept_lazy: compute BOUNDING spheres only (bound_fk), run the masked bounding-sphere
broadphase, and compute full leaf FK + narrowphase ONLY on an active bounding-sphere hit.
Rigorous (0 unsafe) but SLOWER than plain swept everywhere:

| robot | swept (all) | swept_lazy (all) |
|-------|------------:|-----------------:|
| panda | 1.07x | 0.73x |
| fetch | 1.52x | 1.16x |
| baxter| 0.98x | 0.78x |

**Why (m40 FK-cost breakdown, per config block):**

| robot | bound_fk | sphere_fk | leaf share | bound/sphere |
|-------|---------:|----------:|-----------:|-------------:|
| panda | 78  | 190 | 59% | 0.41 |
| fetch | 76  | 261 | 71% | 0.29 |
| baxter| 209 | 299 | 30% | 0.70 |

Two distinct causes:
- **baxter: transforms dominate** (dim-14 chain -> bound_fk is 70% of full FK, leaves only 30%),
  so skipping leaves saves little.
- **fetch/panda: leaves ARE most of FK (71%/59%)**, bound_fk is cheap -- but the TWO-PASS lazy
  kernel pays bound_fk THEN full ccfk (double transforms) on the ~42% of rakes that need leaves
  (m37), plus a second broadphase pass; that overhead exceeds the free-rake leaf savings.

**Verdict:** lazy leaf FK needs a SINGLE-PASS kernel (compute transforms once -> bounding spheres
-> branch -> leaves) to avoid recomputing transforms. CppADCodeGen fuses the whole FK into one
SSA block, so a mid-block branch isn't expressible without restructuring the tracer to emit
transforms and leaf-placement as separately-callable stages. That is the real (larger) piece of
work; the two-pass shortcut measured here does not pay. Kept fkcc_swept_lazy as a documented
variant.

---

## Low-rank Hessian sagitta (m39) -- tight bound at O(dim)

The full Hessian quadratic form (O(NBS*dim^2)) was net-negative for baxter. Replace it with a
rank-1 entrywise COVER: b[k][i] = sqrt(max_j H[k][i][j]) => b_i*b_j >= H[k][i][j] (both row-maxes
dominate), so ad^T H ad <= (sum_i b_i ad_i)^2 -- provably conservative, O(dim). Then take the MIN
of the rank-1 and M*Omega bounds per bs (min of two valid uppers is still a valid upper, tighter
than either). Margin 1.3x, 0 unsafe.

| robot | M*Omega (all) | full Hessian O(dim^2) | rank-1/min O(dim) |
|-------|--------------:|----------------------:|------------------:|
| panda | 1.05 | 1.07 | 1.04 |
| fetch | 1.29 | 1.44 | **1.47** (free 1.58) |
| baxter| 1.31 | 0.95 | ~1.1-1.3 (timing-noise) |

**fetch clearly prefers the tight O(dim) bound (1.29 -> 1.47x all).** For baxter the sagitta-bound
choice is within timing noise -- its bottleneck is the 349-pair colliding setup, not the bound.
So the low-rank cover is the right default: tight where it matters (fetch), cheap everywhere.
The full O(dim^2) Hessian is not worth it.

---

## Staged sphere-aware FK (m41) -- roll-our-own FK, faster + enables single-pass lazy

Traced `joint_tf(x)` = the joint placements oMi[j] (R,t; 12 floats each) for the sphere-carrying
joints ONLY -- the shared/expensive part of FK. Spheres are then placed in C++ as
world = R_slot*local + t_slot (place_sphere, ~9 mul + 6 add). Per-sphere (slot, local, radius)
emitted as static data (leaf_place / bsphere_place). Bit-exact vs bound_fk (max_err ~6e-8).

Per-config-block timing (ns):

| robot | bound_fk | staged_bound | sphere_fk | staged_full | staged_lazy (bound+1 link) |
|-------|---------:|-------------:|----------:|------------:|---------------------------:|
| panda | 78  | 83 (0.94x) | 191 | 144 (**1.33x**) | 92 |
| fetch | 75  | 76 (0.99x) | 265 | 191 (**1.39x**) | 96 |
| baxter| 209 | 199 (1.05x)| 301 | 270 (1.11x) | 211 |

Two wins:
1. **Hand-rolled placement beats the traced fused FK by 1.1-1.4x** for all leaves -- it shares the
   chain transforms cleanly and skips CppADCodeGen's intermediate temporaries + Spheres struct
   writes. So "roll our own FK since the geometry is spheres" is a real speedup on its own.
2. **staged_lazy ~92-96ns** (transforms once + bounding + ONE link's leaves) vs fused full FK
   191-265ns. This is the SINGLE-PASS lazy the two-pass kernel (m39) couldn't do: transforms
   computed once, leaves placed only where a bounding sphere hits, NO double transform compute.

**Next (mechanical assembly):** fkcc_swept_staged -- joint_tf -> place bounding spheres ->
masked broadphase -> place only hit links' leaves (from leaf_place + per_link_spheres) ->
narrowphase. Single pass, no recompute. Expected to fix both panda (reclaim leaf FK) and the
lazy backfire, on top of the node-cached + low-rank-sagitta + vectorized swept.

---

## Single-pass staged lazy swept kernel (m39 fkcc_swept_staged) -- the culmination

Built fkcc_swept_staged: joint_tf(x) computes joint transforms ONCE, places bounding spheres
(world = R*local + t), runs the masked broadphase, and places ONLY the hit links' leaves inline
(no transform recompute). Combines the staged FK win + lazy leaf placement + node-cache +
low-rank sagitta + vectorized masks. Rigorous: mismatch=0, unsafe=0.

| robot | fkcc | swept (all) | swept_staged (all) | staged free | staged colliding |
|-------|-----:|------------:|-------------------:|------------:|-----------------:|
| fetch | 1.00 | 1.48x | **1.58x** | **1.71x** | **1.01x** |
| baxter| 1.00 | 1.16x | 1.18x | 1.34x | 0.63x |
| panda | 1.00 | 1.06x | 1.03x | 1.05x | 0.79x |

**Fetch: 1.71x free / 1.58x all / colliding break-even (1.01x)** -- the single-pass staged FK
reclaims fetch's 71% leaf FK that the TWO-PASS lazy kernel (1.16x, m39) wasted on double
transforms. Colliding edges also improve (place only the hit link's leaves), reaching break-even.
Baxter marginal (transform-dominated: leaves only 30% of FK, little to reclaim); panda flat
(leaves needed every rake). Both exactly as m37/m40 predicted.

**Final state of the swept broadphase (all rigorous, traced natively):**
- Kernels: bound_fk, joint_tf + place_sphere, fkcc_swept (unrolled masked), fkcc_swept_staged
  (single-pass staged lazy).
- Techniques: node-level endpoint caching (0 FK/edge), provable low-rank sagitta bound (O(dim),
  margin 1.3x), vectorized swept tests, single-pass staged lazy FK.
- Result over the SHIPPING kernel: **fetch +58% (free +71%), baxter +18-34%, panda ~flat**.
  Fetch is the sweet spot (moderate pairs, leaves = most of FK, tight bounding spheres).
- Structural non-targets, both explained: panda (coarse spherization -> leaves always needed,
  loose bounding spheres), baxter colliding (349-pair setup, transform-dominated FK).

---

## IN-PRACTICE EVALUATION on real MBM environments (m42) — the reality check

Ablations + orthogonal combinations of every method on REAL Motion-Bench-Maker environments for
ur5/panda/fetch/baxter (21/21/21/9 scenes spanning all problem sets; edges sampled RRTC-style
within each robot's RRT range; swept env test via VAMP's own sphere_environment_in_collision so
it handles spheres, capsules and cuboids). Every method verified bit-identical to baseline fkcc:
**0 mismatches, 0 unsafe over ~23,000 edges.** Collision-check ns/edge, vs baseline:

| robot | edges | recur | swept | swept+cache | staged+cache |
|-------|------:|------:|------:|------------:|-------------:|
| ur5    | all  | 1.00x | 0.76x | 0.87x | 0.87x |
| ur5    | free | 1.06x | 0.95x | 0.99x | 0.98x |
| panda  | all  | **1.05x** | 0.92x | 1.03x | 1.02x |
| panda  | free | 1.06x | 0.96x | 1.06x | 1.05x |
| fetch  | all  | 1.03x | 0.80x | 0.89x | 0.94x |
| fetch  | free | 1.04x | 0.99x | 1.06x | **1.09x** |
| baxter | all  | 0.97x | 0.57x | 0.67x | 0.69x |
| baxter | free | 1.00x | 0.78x | 0.88x | 0.90x |

**The swept broadphase does NOT survive contact with real scenes.** It was break-even-to-negative
everywhere (all-edges), badly negative on colliding edges and baxter. The +58% fetch / +71% free
from m36-m41 was a **microbenchmark artifact**: those used a synthetic 40-small-sphere shelf,
~3x denser than MBM's ~12-15 primitives (mostly boxes/cylinders) and all-spheres. That inflated
the baseline collision cost (fetch free baseline 2900ns synthetic -> 1869ns MBM), i.e. inflated
exactly what the swept mask skips. On real SPARSE scenes the per-rake broadphase is already cheap
(few primitives, effective culling), so the O(n_bs + n_pairs) swept setup — 17-33 env tests +
up to 349 pair tests per edge — cannot amortize over n=2-6 rakes. Baxter (33 bs, 349 pairs, n=2)
is worst at 0.57x. Node-caching + staged FK recover some of it but never past ~break-even.

**The one robust win is the leaf-trig recurrence** — a pure FK-side, environment-independent,
exact optimization: **~1.03-1.06x on free/planning-relevant edges across ur5/panda/fetch**
(baxter ~flat: transform-heavy FK, n=2). It's small but it is real in practice and it is the
only method here that holds up on MBM.

**Honest bottom line:** of everything explored, only the exact leaf-trig recurrence delivers a
(single-digit-percent) speedup on real MBM problems. The swept broadphase — the centerpiece of
the later sessions — is a scene-density artifact and does not pay on real MBM scenes as-is; it
would only help in much denser environments (dense pointclouds / many small obstacles) where the
baseline collision cost is high enough to amortize the per-edge setup. The methodological lesson:
the synthetic shelf was not representative, and every headline speedup should have been measured
on MBM from the start.

### Isolating the FK-tracing change on MBM (m42, stagedFULL column)

The staged sphere-aware FK (joint_tf + place_sphere) was bundled inside fkcc_swept_staged (which
also carries the losing swept mask). Isolated it by running fkcc_swept_staged with masks OFF =
a FULL unmasked check driven by the staged FK, directly comparable to baseline fkcc:

| robot | staged-FK full (all / free / colliding) vs base |
|-------|-------------------------------------------------|
| ur5    | 0.96x / 0.94x / 0.95x |
| panda  | 0.96x / 0.96x / 0.97x |
| fetch  | **1.02x** / 1.02x / 1.01x |
| baxter | 0.92x / 0.89x / 0.97x |

**The staged FK-tracing change is break-even-to-slightly-negative in practice** (only fetch a
marginal +2%). The 1.1-1.4x from the isolated m41 microbenchmark did NOT translate: (a) m41
compared against sphere_fk, which writes a full Spheres struct to memory that the fused fkcc
never does -- an unfair baseline; (b) in the real fused kernel the compiler already optimizes
ccfk well when inlined, and the staged lazy structure (bounding pass then leaves-on-hit) adds
branch overhead that eats the placement savings on sparse scenes.

**Verdict on the tracing modifications:** of the two, only the LEAF-TRIG RECURRENCE (fkcc_pretrig)
is a real in-practice win (~1.03-1.06x on free edges, environment-independent, exact). The staged
sphere-aware FK and the swept-mask kernels do not pay on real MBM. So the tracing work produced
exactly one durable, shippable improvement: the recurrence.

### Isolation table on MBM (m42) — CORRECTS the earlier "nothing to skip" diagnosis

Each modification measured alone on real MBM, plus the masking CEILING (masks precomputed/free,
timing only the masked kernel). Rigorous: total_unsafe = 0 for all robots. All-edges vs base:

| modification | ur5 | panda | fetch | baxter |
|--------------|----:|------:|------:|-------:|
| recurrence (FK-side, fkcc_pretrig)        | 1.05x | 1.06x | 1.03x | 0.98x |
| staged FK (FK-side, full unmasked check)  | 0.96x | 0.97x | 1.03x | 0.92x |
| swept masking IDEAL (masks free)          | **1.09x** | **1.13x** | **1.26x** | **1.92x** |
| swept masking REAL (cached setup)         | 0.89x | 1.01x | 0.90x | 0.66x |
| combined (staged FK + masking)            | 0.91x | 1.01x | 0.94x | 0.65x |

**This corrects the earlier claim that sparse scenes "leave nothing to skip."** They do not: the
masking IDEAL shows real headroom on MBM — up to **1.92x on baxter** (its 349 self-pairs are the
most to skip), 1.26x fetch, 1.09-1.13x ur5/panda. The idea is sound. **The killer is the per-edge
mask SETUP cost.** Decomposing baxter (all-edges): IDEAL 619ns vs REAL 1786ns -> the masks() call
costs ~1167ns/edge = 33 swept-sphere env queries (sphere_environment_in_collision) + 349 pair
tests, amortized over only n=2 rakes. Node-caching already removed the endpoint bound_fk from
setup; what remains — the O(n_bs) env-broadphase queries + O(n_pairs) pair tests — is the wall.

So the accurate verdict per modification, in practice on MBM:
- **recurrence**: real win, ~1.03-1.06x free edges, environment-independent, exact. SHIP-ABLE.
- **staged FK**: neutral (0.90-1.03x) in the fused kernel; the m41 1.1-1.4x was an artifact.
- **swept masking**: the IDEA has genuine headroom (1.1-1.9x ideal) but the mask SETUP
  (n_bs env queries + n_pairs tests, /edge) exceeds it on short MBM edges. Not "no headroom" --
  "setup too expensive." Actionable: batch the n_bs swept-sphere env queries into fewer SIMD
  calls, or a coarse whole-robot swept pre-gate, or amortize masks across the RRT tree. Until
  then it does not pay; but the ceiling says a cheaper setup could make it a real win (esp. baxter).

---

## Two FK-op-reduction ideas, prototyped + evaluated on MBM (m43/m44/m45)

### Idea 1: fused sincos -- SUCCESS, the best FK-side win

VAMP's sin() is a Cephes polynomial that already computes BOTH the sin and cos polynomials of
the reduced angle; cos() then calls sin(x+pi/2) -- a SECOND full range reduction. Added a fused
sincos() to the vector interface that shares one reduction + one polynomial pair (cos sign via
(emm2+2)&4, no int xor needed). Validated (m43): fused_sin bit-exact to sin(), fused_cos within
4.1e-6 of cos(), and **1.71x on the trig pair** (8-wide SIMD): separate 7.16ns -> fused 4.19ns.

On real MBM driving the full fkcc collision check (m44), rigorous (0 verdict mismatches / ~23k
edges despite the 4e-6 cos delta):

| robot | sincos (free/all) | recur (free/all) | recur+sincos (free/all) |
|-------|------------------:|-----------------:|------------------------:|
| ur5    | 1.03 / 1.03x | 1.06 / 1.04x | 1.06 / 1.05x |
| panda  | 1.06 / 1.06x | 1.06 / 1.05x | **1.07 / 1.07x** |
| fetch  | 1.05 / 1.05x | 1.05 / 1.05x | 1.06 / 1.07x |
| baxter | **1.07 / 1.06x** | 1.02 / 0.99x | 1.05 / 1.03x |

**Fused sincos is MORE robust than the recurrence:** it helps every rake (not just 1..n-1), so
it helps colliding edges (1.04-1.06x vs recur's 0.94-1.03x) and helps baxter (1.06x, where recur
is flat because n=2 doesn't amortize). recur+sincos combines to the best (1.05-1.07x everywhere).
It is environment-independent, exact (bit-exact sin, 4e-6 cos with 0 practical mismatches), and
simpler than the recurrence. Shippable; native integration = emit a fused-sincos preamble in the
FK trace (a small codegen change, like the pretrig hoist).

### Idea 2: fp16 bounding-sphere FK -- HARDWARE-GATED, not viable here

The CPU is an i9-14900K: avx2 + f16c + fma, NO avx512-fp16 (Raptor Lake fuses AVX-512 off). F16C
converts fp16<->fp32 but does no fp16 arithmetic, so _Float16 math is emulated via up-convert.
Direct measurement (m45), rotate+translate a batch of points (the sphere-placement op):

| points | float | _Float16 | fp16/float |
|--------|------:|---------:|-----------:|
| 75     | 0.374 ns/pt | 0.832 ns/pt | 2.22x SLOWER |
| 512    | 0.329 ns/pt | 0.614 ns/pt | 1.87x SLOWER |

So an fp16 bound_fk is **1.9-2.2x slower** on this machine, not the ~2x faster we wanted. The idea
is sound only on hardware with native fp16 arithmetic -- AVX512-FP16 (Sapphire Rapids / Granite
Rapids) or ARM NEON-fp16 -- where it would ~halve the broadphase transform cost and could unlock
the setup-bound swept masking (1.9x ideal ceiling, esp. baxter). On AVX2 it is a non-starter;
revisit only on fp16-capable hardware.

### Net
Of the two FK-op-reduction ideas, **fused sincos is a real, shippable, environment-independent
~1.03-1.07x** across all four robots on real MBM -- and, combined with the recurrence, the most
robust FK-side win found. fp16 is correct in principle but blocked by this CPU's lack of native
fp16 arithmetic.

### Native fkcc_sincos in the trace (m44 native column) -- shipped

cricket now emits fkcc_sincos: the FK trace's revolute joints get a fused-sincos preamble
(x[j].sincos(ps[j], pc[j]) via the extracted sincos_joints set) feeding the hoisted-trig ccfk
body -- one shared Cephes reduction per joint instead of separate sin+cos. Drop-in for fkcc,
no external ps/pc. On MBM (native column), rigorous (0 mismatches), matches the validator-driven
measurement:

| robot | fkcc_sincos (all / free / colliding) |
|-------|--------------------------------------|
| ur5    | 1.05x / 1.04x / 1.07x |
| panda  | 1.05x / 1.05x / 1.06x |
| fetch  | 1.06x / 1.04x / 1.06x |
| baxter | 1.06x / 1.05x / 1.07x |

So the fused-sincos win is now a native traced kernel: **~1.05x uniformly across all four robots
on real MBM, colliding edges included**, environment-independent, exact sin / ~4e-6 cos with zero
practical mismatches. To make it the default RRTC path, point validate_motion at fkcc_sincos (or
inline the sincos preamble into fkcc). Combined with the recurrence (recur+sincos) it reaches
1.06-1.08x. This is the cleanest, most robust FK-side speedup found -- and the first one wired
end-to-end through the trace and confirmed on MBM.

---

## END-TO-END RRTC on MBM: fkcc_sincos vs baseline (m47) -- the payoff

Standalone RRTC benchmark on real MBM problems (start+goal+env, joint order from cricket, high
validity confirms correct ordering). fkcc_block gated by -DVAMP_FKCC_SINCOS; baseline and sincos
are separate binaries running the SAME problems with the SAME deterministic Halton sampler ->
identical planning (same solved count, same iterations), so the difference is pure wall-clock.
5 timing trials/problem (min), max_iterations = 10,000,000.

| robot | solved | iters p50 | base mean us | sincos mean us | mean speedup | p95 speedup |
|-------|-------:|----------:|-------------:|---------------:|-------------:|------------:|
| ur5    | 138/140 | 161     | 172.0  | 166.1  | 1.04x | 1.08x |
| panda  | 127/140 | 123     | 85.6   | 80.2   | **1.07x** | 1.08x |
| fetch  | 133/140 | 4016    | 3089.1 | 2959.4 | 1.04x | 1.05x |
| baxter | 45/60   | 168563  | 39126.9| 38922.9| 1.005x | 1.007x |

**The fused-sincos FK win survives all the way to end-to-end RRTC planning:** ~1.04-1.07x on the
mean and up to 1.08x on the harder (p95) problems for ur5/panda/fetch -- matching the ~1.05x
collision-check speedup, because RRTC time on those is collision-check-dominated.

**Baxter is the exception (~1.005x, flat).** Its MBM problems are hard (p50 = 168k iterations,
39ms solves): at ~168k tree nodes the planner spends most of its time in nearest-neighbor search
and tree bookkeeping, not collision checking, so the FK speedup is diluted. The FK improvement is
real (m44: baxter fkcc 1.06x) but it is a small slice of baxter's end-to-end cost.

**Net, in practice:** fused sincos -- a native traced-kernel change, environment-independent,
exact-ish, verified bit-safe on 23k edges -- delivers a genuine **~4-7% RRTC planner speedup on
panda/fetch/ur5** MBM problems, and ~1% on baxter (dominated by tree ops). This is the honest
end-to-end payoff of the whole FK-tracing line of work: the recurrence and the fused sincos are
the two real wins, and fused sincos is now confirmed at every level -- trig microbench (1.71x),
FK kernel on MBM (1.05x), and full RRTC planning (1.04-1.07x on collision-bound robots).

### End-to-end RRTC: recurrence standalone + with sincos (m47) -- recurrence does NOT survive

Added the leaf-trig recurrence to validate_vector under -DVAMP_RECUR / -DVAMP_RECUR_SINCOS: seed
sin/cos (or fused sincos) at the first block, advance ps/pc per block by the constant per-block
joint rotation, feed fkcc_pretrig. Setup (per-joint cos/sin advance coefficients) is deferred
past the n==1 early-out so short edges pay nothing extra. All variants produce IDENTICAL plans
(same solved / iterations) -- correct, pure wall-clock A/B. Stable metrics (panda/fetch means,
all-robot p95; ur5/baxter means are noisy -- few, high-variance solves):

| robot | sincos | recur (standalone) | recur+sincos |
|-------|-------:|-------------------:|-------------:|
| panda (mean) | **1.07x** | 1.02x | 1.06x |
| fetch (mean) | **1.05x** | 0.98x | 1.03x |
| panda (p95)  | 1.07x | 1.00x | 1.06x |
| fetch (p95)  | 1.06x | 0.99x | 1.05x |
| ur5 (p95)    | 1.06x | 0.99x | 1.06x |
| baxter (p95) | 1.01x | 0.99x | ~1.0x (noisy) |

**The leaf-trig recurrence is neutral-to-negative in real RRTC (~0.98-1.02x)** -- it does NOT
survive end-to-end, and recur+sincos does NOT beat sincos alone. Why: the recurrence only pays on
LONG edges (its seed amortizes over n rakes), but real RRTC validates mostly SHORT edges (the
local planner connects/extends in small steps -> many n=1,2 blocks). On n==1 edges the recurrence
reduces to its seed (= sincos or sin/cos) with no advance; on n=2 it barely amortizes. The
earlier 1.03-1.06x for the recurrence (m30/m44) was measured on FULL-RANGE synthetic edges (n=4-6)
-- not representative of the RRTC edge-length distribution. recur+sincos is carried entirely by
the sincos SEED (which is why it ~= sincos); the recurrence adds essentially nothing in practice.

**Final verdict on the FK-side wins, end-to-end on MBM:**
- **fused sincos: the one robust win, ~1.05-1.07x RRTC planner speedup** (panda/fetch/ur5;
  ~1.0x baxter where NN/tree ops dominate). Environment-independent, no per-edge setup, helps
  every block regardless of edge length. SHIP THIS.
- **leaf-trig recurrence: neutral in practice** -- a long-edge optimization that RRTC's short
  edges never amortize; subsumed by sincos. Do not ship standalone; recur+sincos = sincos.

---

## Scene specialization (the environment JIT idea) re-evaluated on real MBM (m48)

The design doc's Component 1: bake the scene's obstacles into the collision kernel (const-fold +
type-prune + a compile-time per-sphere broadphase). Measured the HEADROOM on real MBM before
building the JIT machinery.

### Headroom -- real, and better than the swept broadphase
| robot | env-check share of kernel | scene-spec ceiling | compile-time obstacle pruning |
|-------|--------------------------:|-------------------:|------------------------------:|
| ur5    | 53% | 2.14x | 1.16x (86% kept) |
| panda  | 26% | 1.35x | 1.56x (64% kept) |
| fetch  | 52% | 2.10x | 1.64x (61% kept) |
| baxter | 41% | 1.70x | 1.46x (68% kept) |

The environment check is **26-53% of the kernel** on real MBM (I had wrongly assumed sparse scenes
left nothing here). Const-fold + type-prune speeds that whole slice; the novel compile-time
broadphase prunes another 14-39% of obstacle checks per sphere (weakest for ur5, which reaches
across the whole small scene -- exactly the design doc's caveat). So scene specialization's
ceiling is 1.35-2.14x -- genuinely more promising than the swept broadphase (which was setup-bound).

### The crux is the compile cost (design doc Component 3) -- and it kills single-query
Full clang -O3 compile of one robot's fkcc kernel (a scene-specialized kernel compiles the same FK
plus baked obstacle checks): **panda 1.65 s, baxter 2.96 s.** MBM solve times (m47): panda ~85 us,
baxter ~39 ms. Max per-solve saving = env_share x collision-time ~= panda ~19 us, baxter ~14 ms.
Break-even (compile cost / per-solve saving):
- **panda: ~87,000 solves on the SAME scene** to amortize a 1.65 s compile.
- **baxter: ~200 solves** on the same scene.

So at **T2 (full LLVM compile), scene specialization is non-viable for single-query MBM** by 2-5
orders of magnitude -- the compile cost dwarfs the per-solve saving.

### Verdict
Scene specialization has REAL headroom on MBM (unlike the swept broadphase), but only pays in
regimes that amortize the compile:
- **Copy-and-patch specializer (T1, ~us-ms)** -- the design doc's enabler. baxter's ~14 ms/solve
  saving amortizes a ~ms patch even single-query; panda's ~19 us is marginal. This is the large
  unbuilt piece that makes single-query JIT viable.
- **Multi-query / repeated-scene** (workcell, streaming, optimizing planners re-checking the same
  scene thousands of times) -- even T2's ~2 s amortizes.
- **Single-query MBM with full compile: does NOT pay.**

This matches the design doc's own thesis: the speedup source is real and AOT-impossible, but it
needs the fast (copy-and-patch) specializer or a multi-query regime to be worth the compile. The
headroom measurement (env 26-53%, ceiling 1.35-2.14x) is the green light for building T1; the
break-even numbers are the spec for how fast T1 must be (< ~20 us for panda-class, < ~14 ms for
baxter-class, per single-query solve).

---

## Real-time regime: JIT ideas against a dense pointcloud MVT (m49)

Represented MBM scenes as DENSE surface pointclouds (~1.5cm spacing, ~42k pts/scene; baxter 89k)
behind an MVT (Multi-level Voxel Table, the sensor-based-planning structure), vs the sparse
primitives. This is the "where JIT wins" regime the design doc and the swept-broadphase finding
pointed at (baseline collision cost is high).

| robot | pts/scene | base sparse->MVT | env share | scene-spec ceiling | swept IDEAL | swept REAL |
|-------|----------:|-----------------:|----------:|-------------------:|------------:|-----------:|
| ur5    | 42k | 1.2x | 61% | 2.57x | 1.08x | 0.32x |
| panda  | 42k | 1.4x | 48% | 1.94x | 1.32x | 0.45x |
| fetch  | 42k | 2.3x | 79% | 4.79x | 1.57x | 0.42x |
| baxter | 89k | 2.7x | 78% | 4.55x | 1.06x | 0.34x |

**Confirmed:** against a dense MVT the ENVIRONMENT query dominates (48-79% of the kernel, up from
26-53% on sparse primitives), so the env-side headroom is bigger (ceiling 1.9-4.8x). The MVT is
efficient though -- 42k points is only 1.2-2.7x costlier than 12 primitives.

**Refuted (the important part):** the swept broadphase gets WORSE, not better -- **0.32-0.45x**
(vs 0.57-1.03x on sparse). The IDEAL (free masks) still shows headroom (1.06-1.57x), so the SKIP
idea is sound, but the SETUP is now catastrophic: each swept mask does NBS environment queries
with LARGE swept-sphere radii, and a large-radius query on a fine-voxel MVT (voxel edge =
max_query_radius + point_radius ~ 9cm) scans many voxels -- query cost grows super-linearly with
radius. So the swept setup replaces n small per-rake queries with NBS *large* per-edge queries,
which is far more expensive on a voxel structure. This is the precise refinement of the earlier
"swept is setup-bound" finding: it is not "baseline collision cost" that matters but the cost of a
LARGE-radius query vs many small ones -- and voxel structures punish large radii.

**Also:** scene-spec const-folding does NOT apply to pointclouds (baking 42k points as constants
is infeasible; the MVT *is* the specialization). And fused sincos, while still exact and env-
independent, has a smaller RELATIVE payoff here because FK is only 21-52% of the kernel when the
pointcloud query dominates.

**Real-time verdict:** the win in the dense-pointcloud regime is making the MVT QUERY cheaper or
issuing fewer queries with TIGHT bounds -- NOT the swept broadphase (large queries are punished by
voxels) and NOT scene-spec const-fold (infeasible for points). A promising untested direction: a
COARSE-level query for the swept broadphase against the MVT's multi-level hierarchy (query the
coarse voxels matching the swept sphere's size, cheap), which is the one way to make the sound
IDEAL headroom (1.06-1.57x) realizable. fused sincos remains a real but smaller (FK-fraction)
win. This is the honest end of the environment/robot JIT-idea line: bigger headroom in real-time,
but the current swept/scene-spec mechanisms do not capture it -- the MVT query itself is the target.

### Coarse-level swept certification against the MVT (m49) -- rescues, doesn't win

Tested the one lever that could make the swept broadphase pay against a dense pointcloud: certify
each link with a CHEAP coarse occupancy grid (dense bool grid, voxel ~ swept-sphere size) instead
of a large-radius fine-MVT query, falling back to the fine MVT only for non-certified links.
Conservative (occupied coarse voxel bounds all its points; uses radius+point_radius) -- rigorous,
mismatch=0 vs baseline on all robots.

| robot | swept IDEAL | REAL/fine-MVT | COARSE grid |
|-------|------------:|--------------:|------------:|
| ur5    | 1.08x | 0.32x | 0.85x |
| panda  | 1.32x | 0.45x | **0.99x** |
| fetch  | 1.56x | 0.42x | 0.95x |
| baxter | 1.06x | 0.34x | 0.88x |

**The coarse certification rescues the swept broadphase from catastrophic (0.32-0.45x) to
break-even (0.85-0.99x)** -- a 2-3x improvement, confirming the diagnosis (the fine large-radius
query was the killer) and that the coarse-hierarchy fix is correct. A dense bool grid was
essential: an unordered_set version was far slower (0.13-0.54x). Voxel size 0.25-0.6m all land
~0.85-1.00x (plateau).

**But it does NOT clear break-even.** The residual cost is (a) the coarse queries themselves --
NBS large-radius range scans per edge -- and (b) the per-rake fine-MVT narrowphase for the
~10-40% non-certified links. The gap to the IDEAL (1.06-1.57x) is exactly this. And these are
FULL-RANGE synthetic edges (large swept spheres); real RRTC edges are short (small swept spheres,
nothing to amortize), where the swept broadphase gives no benefit regardless.

**Final verdict on the whole environment/robot JIT line, incl. real-time MVT:** the swept
broadphase is not shippable in any measured regime -- setup-bound on sparse scenes, large-query-
bound on dense MVT (fixable to break-even with a coarse grid, but no further), and unamortized on
real short RRTC edges. Scene-spec const-fold has real headroom (1.35-2.14x sparse, up to 4.8x on
dense MVT) but is compile-cost-bound (needs copy-and-patch or multi-query). The one durable,
shippable win from the entire line remains the FK-side fused sincos (~1.05-1.07x, exact, native,
end-to-end confirmed) -- everything scene/environment-specialized needs infrastructure (fast
specializer) or a cheaper MVT query that this codebase does not yet have.

---

## Primitive scene specialization on MBM (m50) -- the compile-time broadphase does NOT pay

Built the novel scene-spec component (Component 1.3): the compile-time per-sphere broadphase --
per robot bounding sphere, keep only obstacles inside its reachable region (a scene-spec-time
prune, AOT-impossible). Measured as per-link PRUNED sub-environments vs the full env, timing the
env-vs-robot check (bounding broadphase -> leaf narrowphase) on real MBM configs. Rigorous, mism=0.

| robot | prune (obstacles kept) | env-check: full -> pruned | speedup |
|-------|-----------------------:|--------------------------:|--------:|
| ur5    | 86% (1.16x) | 329 -> 394 ns | 0.83x |
| panda  | 64% (1.56x) | 165 -> 162 ns | 1.02x |
| fetch  | 61% (1.64x) | 292 -> 251 ns | 1.16x |
| baxter | 68% (1.46x) | 553 -> 660 ns | 0.84x |

**The compile-time broadphase prune does NOT deliver on MBM** -- net-negative for ur5/baxter,
break-even for panda, only 1.16x for fetch. Two reasons:
1. **VAMP's generic env is already a runtime broadphase** -- sphere_environment_in_collision
   iterates a SORTED obstacle list with an early-exit on min_distance, so it already skips far
   obstacles cheaply. The compile-time prune removes obstacles the sorted-cull was already
   skipping for free -> little gain. (This is why the original "reorder/prune 1.46-4x" numbers
   only held vs UN-gated loops.)
2. The env is only 26-53% of the kernel, so even the best env-check win (fetch 1.16x) is ~1.08x
   at the kernel level -- and it is negative where NBS is large (ur5/baxter: 17/33 bounding
   spheres, per-link overhead dominates the small prune).

Combined with the const-fold component being blocked (obstacle fields are SIMD FloatVector, not
constexpr -> can't fold to immediates; and the SIMD distance compute dominates the memory loads
anyway), the primitive scene-spec ceiling (1.35-2.14x, which assumed env->0) is NOT reachable:
the generic env is already efficient.

**Verdict:** primitive scene specialization is not worth the copy-and-patch infrastructure for
MBM -- the compile-time broadphase (its headline lever) is neutralized by VAMP's sorted-cull, and
the residual (const-fold) is SIMD-constexpr-blocked and compute-bound. This confirms the original
<=3-5% finding, now with the real MBM harness and the mechanism pinned down. The end of the
scene/environment-specialization line: real headroom exists only if env->0 (which the sorted-cull
already approximates), so there is nothing left for JIT to bake. The one durable win across the
entire investigation remains the FK-side fused sincos.

---

## Model-noise snap (m51) -- a second FK-side win, exact and free

Idea #1 from the post-scene-spec brainstorm: sparsify the joint transforms. Inspecting the
generated FK (baxter joint_tf) showed the joint axes are essentially coordinate-aligned but
polluted with ~1e-12 numerical dirt (coefficients like `4.89663865010925e-12`) from the
URDF->pinocchio conversion. Left in, CppADCodeGen emits dense 3-term rotation products
(`y[27]*v[0] + y[30]*v[1] + y[18]*v[3]`, where `v[0]~=4.9e-12`) where a sparse 2-term product
suffices. Snapping sub-1e-9 joint-placement entries to exactly 0.0 before the trace lets CppAD
fold `0*x->0` symbolically, and the sparsity cascades through the chain (kills downstream var*var
products too).

**Op-count reduction in `sphere_fk`** (literal-coef muls that were spurious -> total muls dropped):

| robot | spurious lit-coef muls | total muls: base -> snapped |
|-------|-----------------------:|----------------------------:|
| ur5    |   0 (0%)  | 305 -> 305 (already clean) |
| panda  |  28 (6%)  | 545 -> 483 (-11%) |
| fetch  |   0 (0%)  | 454 -> 454 (already clean) |
| baxter | 122 (21%) | 768 -> 578 (-25%) |

**Fused-kernel timing on real MBM edges** (ns/edge, baseline headers vs snapped; verdict
fingerprint identical across builds on all 6720/2880 edges -> exact, mism=0):

| robot | fkcc base -> snap | fkcc_sincos base -> snap | plain-base -> snap+sincos |
|-------|------------------:|-------------------------:|--------------------------:|
| ur5    | 1392 -> 1371 (1.02x) | 1331 -> 1324 | 1392 -> 1324 = **1.05x** |
| panda  | 1923 -> 1875 (1.03x) | 1830 -> 1787 (1.02x) | 1923 -> 1787 = **1.08x** |
| fetch  | 1032 -> 1032 (1.00x) |  985 ->  978 | 1032 ->  978 = **1.05x** |
| baxter | 1176 -> 1148 (1.02x) | 1120 -> 1079 (1.04x) | 1176 -> 1079 = **1.09x** |

The snap adds ~2-4% on top of sincos for the noisy models (panda, baxter), ~0.5% for the
already-clean ur5/fetch. It **stacks with sincos** (orthogonal: sincos fuses the trig range
reduction, the snap sparsifies the transform muls). Combined FK-side wins: 5-9% fused kernel,
all robots.

**Why it is a keeper (same profile as sincos):** exact (fingerprint-identical; the geometry
change is <10nm), environment-independent, every-call, and *free* -- pure codegen-time model
cleanup with zero runtime cost and zero per-scene/per-edge setup. No tradeoff; strictly dominates
the raw model. Now the cricket default (commit cf3c409; CRICKET_NO_SNAP=1 for A/B).

**Caveat (the familiar wall):** the robot with the most FK headroom (baxter, -25% muls) is the
NN/tree-bound one end-to-end, so its RRTC gain is muted; the FK-bound robots (ur5/fetch) were
already clean. Net end-to-end effect is small (kernel 2-4% -> a point or two of RRTC on the
collision-bound easy problems), but it costs nothing and always applies.

---

## Per-primitive collision math (m52) -- already tight, no headroom on MBM

Idea #2: profile the per-obstacle collision kernels (sphere-cuboid / sphere-capsule) for a faster
every-call formulation. Two questions: are real MBM obstacles dispatched to the fast paths, and is
there a further fast-path worth adding?

**Bucket distribution on MBM** (across all scenes): boxes 162/165 z-aligned, capsules 81/81
z-aligned, 0 spheres. The 3 "general" boxes per robot have axis_3_z ~= 0.7067 = cos(45 deg) --
genuinely 45deg-tilted boxes that correctly need the OBB path. **No noise misclassification**
(`general-but-actually-z=0`): unlike the FK trace, the euler->matrix path here is clean -- MBM's
yaw-only boxes (euler ex=ey=0) produce axis_3=(0,0,1) *exactly*, so the `axis_3_z == 1.` gate
routes them to z_aligned correctly. baxter: all 54 boxes z-aligned.

**Per-call cost (ns), yaw-rotated box, near queries:**

| primitive | general OBB | z-aligned (in use) | AABB (hypothetical) |
|-----------|------------:|-------------------:|--------------------:|
| cuboid    | 5.18 | 3.26 (**1.59x** already saved) | 1.95 (1.67x vs z) |
| capsule   | 3.70 | 1.96 (**1.89x** already saved) | -- |

The z-aligned fast path is already dispatched and already banks 1.59-1.89x. The only cheaper
path -- pure AABB (no dot products) -- is **1.67x faster than z-aligned but inapplicable**: MBM
boxes are genuinely yaw-rotated (ez=1.71827, ...), so they need the two xy-plane dots; an AABB
test would be geometrically wrong for them.

**Verdict:** no headroom on MBM. The collision primitives are already tight squared-distance
tests, already correctly fast-pathed (98% z-aligned), with no noise misclassification to snap
away. The one further lever (an AABB `fully_axis_aligned_cuboid` bucket, gated on
axis_1_x==1 && axis_2_y==1) would give ~1.67x per box, but ONLY for axis-aligned-box scenes
(warehouse/table AABBs) -- MBM's yaw-rotated boxes don't qualify. Documented as a conditional
lever for AABB-heavy deployments; not built (YAGNI for the benchmark). The FK-side snap (m51) and
fused sincos remain the two durable, universally-applicable wins.

---

## End-to-end RRTC: the shipped FK-side wins in practice (m47, 10 trials)

Full decomposition of the two durable FK-side wins on real MBM RRTConnect problems (deterministic
Halton, 10 timing trials/problem min, 10M max iters, pinned core). base = un-snapped headers +
base fkcc; +sincos = fused sincos kernel; +snap = model-noise snap (m51); snap+sincos = both
(the shipped default). Solved counts (138/127/133/45) and iteration p50 identical across all four
configs -> pure speedup, no behavior change.

**p50 planner us (min over 10 trials):**

| robot  | base | +sincos | +snap | snap+sincos | combined |
|--------|-----:|--------:|------:|------------:|---------:|
| ur5    |  36.5 |  35.0 (1.04x) |  36.5 (1.00x) |  35.1 | **1.04x** |
| panda  |  32.0 |  30.6 (1.05x) |  31.4 (1.02x) |  30.0 | **1.07x** |
| fetch  | 605.2 | 561.4 (1.08x) | 608.3 (1.00x) | 564.0 | **1.07x** |
| baxter |  8714 |  8692 (1.00x) |  8668 (1.01x) |  8551 | **1.02x** |

(mean planner us combined: ur5 1.06x, panda 1.09x, fetch 1.06x, baxter 1.00x.)

**The decomposition matches the kernel mechanism exactly:**
- **sincos** carries most of it: 1.04-1.08x on the collision-bound robots (ur5/panda/fetch),
  1.00x on baxter.
- **snap** adds ~1-2% *only* on panda -- the one collision-bound robot whose FK carried noise
  (6% spurious muls). ~0 on ur5/fetch (their FK was already clean, 0 spurious), and diluted on
  baxter. This is exactly the m51 kernel prediction propagated end-to-end.
- **baxter** is ~1.00-1.02x despite -25% FK muls: its 168k-iteration solves are NN/tree-bound,
  so collision-kernel wins wash out. The predicted wall, now confirmed at the planner level --
  the FK-bound robots (ur5/fetch) had no FK noise to snap, and the robot with the most FK noise
  (baxter) is NN-bound. The anti-correlation caps the snap's end-to-end reach at ~1-2%.

**Bottom line for the session:** the two FK-side wins deliver ~4-9% end-to-end on collision-bound
MBM problems (sincos the bulk, snap a small exact bonus on FK-noisy robots), and ~0-2% on the
NN-bound hard problems. Both are exact-ish, environment-independent, zero-setup, and always-on.
For further end-to-end headroom on the hard problems, the lever is the NN/tree search, not the
FK+collision kernel.

---

## Sizing the hard-problem bottleneck (m53) -- it is NN + sampler, not collision

perf profile of an isolated baxter RRTC workload (50 problems x 4 reps, ~348M iterations,
-F1999, pinned). Self-time breakdown:

| component                                   | self-time |
|---------------------------------------------|----------:|
| KD-tree NN search (`KDTree::search_one`)    | **43.4%** |
| Halton sampler (`Halton::next`)             | **32.3%** |
| solve body (Robot::distance, insert/grow, bookkeeping) | 21.4% |
| collision (fkcc + sphere_environment_in_collision + validate + steer) | **~2.5%** |

**The headline:** for the NN-bound hard problems, the collision kernel we spent the session
optimizing is only ~2.5% of the time. That is *why* sincos+snap gave baxter ~1.00-1.02x
end-to-end -- there was never more than ~2.5% to win. The 97% is planner machinery.

**Two real levers, sized:**
1. **NN search (43%)** -- already a strong SIMD kd-tree: SoA leaf blocks scanned with vector row
   ops, AABB lower-bound pruning, (1+eps)-approximate queries, leaf-scan checkpoints, prefetch,
   huge pages, compile-time dimension. The 43% is dominated by leaf scans (scan_leaf ~16% + the
   per-lane extraction), i.e. the curse of dimensionality: at 14-D the box bounds prune weakly so
   many leaves get scanned. This is **algorithmic, not codegen** -- the loops are already
   templated + vectorized; the lever is a better ANN / larger nn_epsilon / a different metric, all
   of which trade path quality or determinism. Not a JIT win.
2. **Halton sampler (32%)** -- the surprise. The incremental Halton `next()` is division- and
   floor-heavy: `(d/b).floor()`, a variable-length carry loop `while(x_le_y.any()){ (y/b).floor() }`,
   and a final `n/d`, over a 14-D (2x AVX register) sample every iteration. Two of the three
   division sites divide by the **constant** prime bases `b`, so `/(b)` can become `* (1/b)` with a
   precomputed reciprocal (SIMD divps ~11-14cy vs mulps ~4cy). This is a **numerical-kernel win in
   the same profile as sincos/snap** (environment-independent, every-call, exact if the floor
   tolerance holds) -- and it only helps the high-iteration hard problems, which is exactly where
   the collision wins don't reach. The most promising NEW lever the session has surfaced.

Note this split is baxter-specific (NN/sampler cost scales with iterations x dimension). The easy
robots (panda/ur5/fetch, ~30us / hundreds of iters) are collision-bound, which is why sincos won
there. So the two robot regimes want different optimizations: collision kernel for the easy/
collision-bound problems (done), sampler+NN for the hard/high-iteration ones.

---

## Sampler swap: XORShift vs Halton on all robots (m47) -- 2-3x on the hard problems

Follow-up to the m53 sizing (Halton = 32% of baxter). Swapped Halton for the SIMD xorshift128+
PRNG (same interface, same snap+sincos kernel, 10 trials min, only the sampler differs):

| robot | sampler | solved | iters p50 | p50 us | mean us | p95 us |
|-------|---------|-------:|----------:|-------:|--------:|-------:|
| ur5 (6D)    | halton |  138 |    161 |  35.8 |  163.6 |   1084 |
|             | xorshift | 138 |  231 |  41.8 | **72.4** | **191** |
| panda (7D)  | halton |  127 |    123 |  29.7 |   78.4 |    416 |
|             | xorshift | 127 |  168 |  30.5 |   88.1 |    401 |
| fetch (8D)  | halton |  133 |   4016 | 557.8 |   2953 |  12484 |
|             | xorshift | 132 | 3481 | **395** | **2394** | **5973** |
| baxter (14D)| halton |   45 | 168563 |  8570 |  38911 | 173369 |
|             | xorshift |  45 | 73045 | **3662** | **12519** | **53990** |

**Two effects, and for high-dim the second dominates:**
1. Cheaper per-sample (the 32% -> ~0): baxter per-iter cost 0.231 -> 0.171 us (~1.35x).
2. **Fewer iterations** -- the surprise: baxter needs **2.31x fewer** iterations with xorshift
   (168k -> 73k), fetch 1.15x fewer. High-dimensional Halton has a known correlation pathology
   (large prime bases -> samples correlated over short windows), so at 14-D it explores the
   configuration space *worse* than plain random. The two effects multiply: baxter 2.34x p50,
   3.11x mean, 3.2x p95.

**By regime:** low-dim easy (ur5/panda) -- Halton's low-discrepancy helps the median (fewer iters),
so xorshift is slightly slower at p50, but ur5's hard tail collapses 5.7x (p95); panda ~neutral
(0.89x mean, a small loss). High-dim hard (fetch/baxter) -- xorshift wins everywhere, hugely on
baxter. Solve rate essentially preserved (fetch -1).

**Significance:** on the NN-bound hard problems the sampler swap is worth 2-3x end-to-end -- an
order of magnitude more than the entire FK+collision kernel effort (~1.02x on baxter). It confirms
the m53 sizing (kernel is 2.5% of baxter; the planner machinery is everything) and identifies the
sampler as by far the highest-leverage change for hard/high-dim planning.

**Caveats / not shipped:** one realization each (Halton deterministic; xorshift one fixed key) --
the iteration deltas are far too large to be seed noise, but a rigorous solve-rate claim needs
multiple xorshift seeds. This is a BEHAVIORAL change (different samples/paths), not an exact
drop-in, so it is a VAMP-defaults decision, not a silent commit. The clean takeaway for the JIT
line of work: for hard problems the win is the sampler/NN, not the collision kernel -- and the
sampler win here is algorithmic (which sequence), not codegen.

---

## Native (dependency-free) xorshift sampler (m54) -- works, but a quality gap vs external

Implemented a self-contained SIMD xorshift with no external simdxorshift dependency: one Marsaglia
xorshift32 per 32-bit lane (one stream per sampled dimension), advanced together each next() via
the vamp vector ops. Required adding `xor_` to the AVX integer backend (SIMDVector<__m256i> had
and_/or_/bitneg but no exposed xor_ -- the same gap the sincos work hit). Lanes seeded with
decorrelated nonzero splitmix32 values. Sanity check (m54, 500k samples on baxter): per-dimension
serial autocorrelation < 0.002 and cross-dim correlation 0.0002 -> statistically sound.

**10-trial, 3-way RRTC (same snap+sincos kernel):**

| robot  | metric | Halton | ext xorshift128+ | native xorshift32 |
|--------|--------|-------:|-----------------:|------------------:|
| baxter | solved |     45 |               45 |            **50** |
|        | mean us |  38895 |            12522 |             27511 |
|        | p50 us |   8564 |             3664 |              5370 |
| ur5    | mean us |  162.6 |             72.3 |             166.1 |
| panda  | mean us |   78.4 |             88.1 |             106.2 |
| fetch  | mean us |   2949 |             2394 |              3358 |

**Read:** native xs removes the dependency and beats Halton on the hard problem (baxter 1.41x mean,
1.60x p50, +5 solved), but is neutral-to-slightly-worse than Halton on the easy robots and
consistently slower than the external xorshift128+. Cause: xorshift32 (32-bit state, period 2^32,
fails parts of BigCrush) has weaker equidistribution than xorshift128+ (128-bit state, passes
BigCrush), so it costs iterations -- good median, higher variance (panda p50 is the best of all
three at 27us but its p95 is the worst). As a drop-in replacement it is a partial regression vs the
external lib.

**Fix (not yet built):** keep the identical abstraction but use a higher-quality native generator --
xoshiro128** / xorshift128 (128-bit state per lane), buildable from the same ops (rotate =
(x<<r)|(x>>(32-r)) plus the existing mullo). Expected to match external quality with zero
dependency. The `xor_` backend op added here is the reusable piece.

---

## Multi-seed sampler sweep (m47) -- corrects the single-seed conclusions

The single-realization sampler numbers above were noisy. Ran 8 seeds per PRNG on the two
high-variance robots (Halton is deterministic -> one point), TRIALS=3, median [min-max]:

**baxter:**

| gen           | solved med | iters med       | mean-us med [range]     | vs Halton |
|---------------|-----------:|----------------:|------------------------:|----------:|
| Halton (1 pt) |         45 |         168563  | 39239                   |     --    |
| ext xorshift128+ | 48 [45-49] |   150420 [117k-183k] | 27303 [20854-41472] | 1.44x |
| native xorshift32 | 47 [44-50] | 176990 [156k-203k] | 22327 [17277-32455] | **1.76x** |
| xoshiro128**  | 46 [42-48] |   170832 [109k-203k] | 21906 [18988-30326] | **1.79x** |

**fetch:** all four within noise (ht 2971, xs 2748, xsn 3159, xo 2455 us).

**This overturns two earlier claims:**
1. **The "external xs = 3x on baxter" (73k iters / 12.5ms) was seed luck** -- that draw was *below
   the minimum* of the 8-seed sample (117k). The external-xs median is ~150k iters / 27ms. The
   robust Halton->PRNG win on baxter is **~1.4-1.8x**, not 3x. Per-seed mean spread is ~2x, which
   is exactly why single realizations mislead.
2. **Generator quality barely matters here.** native xorshift32 (22.3ms) and xoshiro128** (21.9ms)
   are indistinguishable and slightly *better* than external xorshift128+ (27.3ms) -- all inside
   seed noise. The previous-turn claim that xorshift32 is a "partial regression" was itself a
   seed-noise artifact. **xoshiro128 buys nothing measurable over the simpler xorshift32.**

**Conclusion:** the robust story is (a) swapping Halton for any cheap PRNG is worth ~1.4-1.8x on
the hard/high-dim robot (baxter) and ~neutral on the rest -- from removing the 32% sampler cost
plus PRNGs dodging high-dim Halton correlation (modestly fewer iters, a few more solves); (b)
among PRNGs the choice is noise, so the **simple dependency-free native xorshift32 is the pick**
(least code, no external lib). xoshiro128** is kept as a higher-quality option but is not needed.
The generic recurring lesson holds once more: single-realization microbenchmarks lie -- the 3x
was a lucky seed.

---

## How much would fully compiling the environment help (multi-query / replanning)? (m55)

Question: in a long-term-planning / replanning-in-a-fixed-scene regime (where the per-scene JIT
compile cost amortizes over many solves), how much does fully compiling the environment buy?

Profiled the two regimes (perf self-time; panda = easy/collision-bound, baxter = hard/NN-bound):

| component                        | panda (123 iters) | baxter (168k iters) |
|----------------------------------|------------------:|--------------------:|
| FK kernel (fkcc self)            |            39.8%  |               1.0%  |
| **env-check (sphere_environment)** |          **33.0%**  |             **1.5%**  |
| Halton sampler                   |             7.0%  |              32.3%  |
| KD-tree NN (search_one)          |             6.5%  |              43.4%  |
| solve body / steer / other       |            ~13%   |               ~22%  |

The env-check is the ONLY part a compiled environment can reduce. Its share of the *solve* is
33% (panda) vs 1.5% (baxter).

**Absolute ceiling** (env-check -> 0, physically impossible since the SIMD distance arithmetic is
irreducible): panda 1/(1-0.33)=**1.49x**, baxter 1/(1-0.015)=**1.015x**.

**Realistic** (what a JIT'd env can actually remove): from m50, the compile-time broadphase prune
is already ~neutral (0.83-1.16x) because VAMP's env is a sorted+culled broadphase -- the sorted
min_distance early-exit already skips far obstacles for free. So the residual lever of a full
compile is const-folding the checked obstacles' fields to immediates + unrolling the loop /
dropping the broadphase branch. m48 found the SIMD distance *compute* dominates the loads, so
const-fold trims only ~10-25% of the checked-obstacle cost. Net env-check speedup ~1.2-1.5x, so:
- panda: 33% x (1 - 1/1.3) ~= 7-8% -> **~1.08x end-to-end** (ceiling 1.49x).
- baxter: 1.5% -> **~1.005x** (ceiling 1.015x).
- fetch (intermediate, 111 spheres, ~4k iters): ~1.04-1.06x.

**Answer:** even with compile cost fully amortized by replanning, fully compiling the environment
buys **~1.05-1.1x on collision-bound easy scenes and ~1.0x on hard scenes**. Two reasons it does
not unlock a big win:
1. **Regime inversion:** the env-check is a large share only for easy, few-iteration, low-dim
   problems (panda 33%) -- exactly the ones that already solve in ~30us and least need speedup.
   The hard problems that *do* need speedup (baxter) are NN(43%)+sampler(32%)-bound; the env-check
   is 1.5%, and compiling the environment cannot touch NN or sampling.
2. **The generic env is already efficient:** the sorted-cull does the broadphase for free (m50),
   so const-fold+unroll is the only residual, and the collision arithmetic itself is irreducible.

**For the multi-query / replanning regime, the real levers are NOT the environment:** (a) tree /
roadmap reuse across replans (algorithmic -- avoid rebuilding the tree each solve), and (b) the
sampler swap (1.4-1.8x on baxter, m47 multi-seed). Compiling the environment is the wrong target:
it is amortizable but capped at ~1.1x, and inverted against where the time actually is.

---

## Compiling the constraints for complex robots -- already done, plus a snap gap now closed

Question: how much would compiling the constraints help for complex robots?

**They are already fully compiled.** cricket traces the constraint error AND its analytic Jacobian
(CppAD `error_func.Jacobian`, shared subexpressions) and emits SIMD-batched, rake-parallel,
per-robot kernels (VAMP `ConstraintSet::evaluate_error_jacobian`), including bimanual/TSR for
dual-arm robots. VAMP's manifold planner (MCFLASK: chart = pivoted-QR of the stacked constraint
Jacobian) calls these compiled kernels directly. So "compile the constraints" is not a future
opportunity -- it is a core, already-realized feature, and the single biggest JIT win in the
system: for a complex robot the analytic compiled Jacobian replaces a finite-difference Jacobian
that would need ~dim+1 (~15 for baxter) FK passes per Newton step, and constrained planning is
projection-dominated (many Newton steps per sample). That is roughly an order of magnitude,
already banked.

**The residual gap (now closed).** The model-noise snap (m51) and fused sincos lived only in the
FK/collision trace path (`trace_sphere_cc_fk`). All 7 constraint trace sites -- plus distance and
FLASK -- cast the raw `info.model` and missed the sparsification. Fix: moved the snap to
`RobotInfo` construction (`snap_placement_noise`), so every trace inherits the sparse model
(cricket 29aed34). Verified: collision kernels byte-identical (sphere_fk unchanged, still exact),
and `joint_tf` -- a pure-FK kernel that previously missed the snap -- drops **381 -> 191 muls
(-50%)** for baxter with the ~1e-12 dirt eliminated.

**Magnitude for constraints:** the constraint error+Jacobian kernels are pure FK/kinematics (no
collision loop to dilute them), like `joint_tf`. So they see ~the full FK-transform reduction --
about **-50% muls for baxter**, roughly 2x the collision kernel's -25% (which was diluted by the
sphere/broadphase work). This lands exactly where projection-heavy constrained planning on complex
robots spends its time. (End-to-end constrained-planning measurement not run here -- no constrained
benchmark set up -- but `joint_tf` is the measured pure-FK proxy and the mechanism is identical.)

**Answer:** compiling the constraints is already the design and is the biggest JIT lever in VAMP
(analytic Jacobian, ~order of magnitude vs finite-diff on complex robots). The only thing left on
the table was the FK-transform snap not reaching the constraint trace -- now fixed, worth ~-50%
FK muls on the constraint kernels for a 14-DOF robot.

---

## sincos on the TSR constraint kernels (m56) -- a ~1.25x kernel win, not yet applied

Question: are the TSR (constraint) kernels using fused sincos, and is there general perf headroom?

The TSR error+Jacobian kernels are a SEPARATE trace path from `fkcc_sincos` (cricket
`emit_error_and_jacobian` -> raw CppADCodeGen), so they use un-fused `sin()`/`cos()`. Generated
the bimanual panda (14-DOF dual-arm, `constraints:true`) and inspected `tsr_bimanual_error`:
- **snap reached it** (0 sub-1e-9 noise terms -> last turn's global-snap fix works on constraints).
- **sincos NOT applied**: 44 `sin` + 43 `cos` calls, 0 fused.
- **~3x redundant trig**: `sin(x[0])` emitted 3x (v[45], v[133], v[162]) -- CppADCodeGen does not
  CSE the trig across the error/Jacobian output blocks, and VAMP's `sin()` isn't marked const so
  the C++ compiler can't fully dedup it either.

Prototyped the same hoist used for `fkcc_sincos` (preamble `x[j].sincos(ps[j], pc[j])` per joint,
then replace every `sin(x[j])`->`ps[j]`, `cos(x[j])`->`pc[j]`): 84 separate sin/cos -> **14 fused
sincos** (dedup + fuse in one transform). Microbench on the real kernel:

| variant                    | ns/call | speedup |
|----------------------------|--------:|--------:|
| orig (84 sin/cos)          | ~660    | --      |
| hoisted (14 fused sincos)  | ~520    | **~1.25x** (1.21-1.30x across runs) |

**Correctness:** the transform is algebraically exact -- 0/90 output diff at well-conditioned
(small) inputs. The only numerical difference is sincos's ~1e-7 cos error; at pathological
far-from-manifold random configs the ill-conditioned TSR Jacobian amplifies it (max-abs 7.9e3 on
~1e4 entries, max-rel blows up only where an output ~= 0), but near the manifold -- where
projection actually operates and where the error must reach ~1e-4 tolerance -- it is negligible.
Same accuracy tradeoff as the shipped `fkcc_sincos` (validated fine for planning).

**Why bigger than collision's 1.05x:** the TSR kernel is pure FK + analytic Jacobian, no collision
loop to dilute the trig -- so the trig fraction (and thus the sincos win) is much larger, like the
snap hit joint_tf (-50%) harder than sphere_fk (-25%).

**Summary of TSR-kernel performance levers:**
1. Already compiled with analytic Jacobian -- the biggest win, realized (~order of magnitude vs
   finite-diff on complex robots).
2. Model-noise snap now reaches them (global snap, cricket 29aed34): ~-50% FK-transform muls for
   a 14-DOF robot.
3. **sincos-hoist: a further ~1.25x on the constraint error+Jacobian kernel (measured, not yet
   built into cricket)** -- would mirror the fkcc_sincos codegen change in
   `emit_error_and_jacobian` + the template. This is the one remaining un-applied FK-side win, and
   it lands on the hot path of projection-heavy constrained planning for complex robots.

---

## Other op reductions for the TSR kernel? (m56 analysis) -- sincos is the only free one

Op composition of `tsr_bimanual_error` (14-DOF bimanual, 6D task error+Jacobian, snap already
applied): 1025 assignments, 3102 muls, 1764 adds, 87 trig.

| lever                          | count | reducible? |
|--------------------------------|------:|------------|
| redundant TRIG RHS             |    56 | YES -> sincos-hoist (the m56 win, ~1.25x) |
| redundant non-trig RHS         |    15 | no -- CppADCodeGen single-assigns arithmetic; -O3 CSEs the rest |
| `-1. * v[k]` negations         |    71 | no -- `-1.0*x` is bit-identical to `-x`, folded to negate/FNMA by -O3 |
| mul-by-float-const             |   375 | no -- FK link lengths + 0.5 factors, irreducible |

**Beyond sincos there is no free op-reduction.** CppADCodeGen already single-assigns every
arithmetic subexpression (only 15 non-trig duplicates, which -O3 CSEs), and the `-1.*` negations
fold away. The one thing it does NOT dedup is the trig (`sin(x[0])` emitted 3x), which is exactly
what the sincos-hoist fixes. The remaining 3102 muls are genuine FK+Jacobian arithmetic.

**One structural lever (not free, needs measurement):** ~35 muls per Jacobian entry (~2900 muls
for a 6x14 Jacobian) is high, because cricket builds the Jacobian by CppAD autodiff of the error
(`error_func.Jacobian`) rather than pinocchio's analytic geometric frame Jacobian. A structured
geometric Jacobian (column = joint axis x lever arm from the FK) is typically ~10-15 muls/entry,
so this *could* be several-fold fewer ops -- but reverse-mode AD is usually efficient, so the gap
may be smaller than the raw count implies; it would need cricket to emit `getFrameJacobian` and a
head-to-head measurement before claiming a win. The other structural option -- specialize to only
the active task DOF (a typical TSR pins 2-4 of 6) -- is plan-time per-TSR, with the usual
amortization question.

**Bottom line:** the FK-side op-reduction story for the TSR kernel is: snap (done, ~-50% FK muls)
+ sincos-hoist (~1.25x, measured, not yet built). After those, the arithmetic is CSE-tight; the
only further headroom is the structural geometric-Jacobian question, which is uncertain and would
need its own measurement.

---

## Geometric-Jacobian path for the TSR kernel (jac_compare) -- measured 2.79x fewer ops

Prototyped the structural lever from the m56 analysis: replace CppAD's autodiff of the constraint
error (`error_func.Jacobian`) with pinocchio's ANALYTIC geometric frame Jacobian composed with the
log-map derivative. Built a standalone head-to-head on the bimanual panda (nq=14) using cricket's
exact toolchain (pinocchio ADCG + CppADCodeGen), counting generated multiplies:

| Jacobian (per end-effector)                    | temps | muls | vs autodiff |
|------------------------------------------------|------:|-----:|------------:|
| raw frame Jacobian d(pose)/dq -- autodiff      |   144 | 1264 | --          |
| raw frame Jacobian d(pose)/dq -- analytic (computeFrameJacobian) | 40 | 174 | **7.26x** |
| full TSR err-Jacobian (6x14) -- autodiff (CURRENT cricket) | 167 | 2150 | -- |
| full TSR err-Jacobian (6x14) -- analytic `Jlog6(Y)*Ad_rTe*J_local` | 102 | 771 | **2.79x** |

**Result:** the geometric path cuts the per-EE TSR Jacobian from 2150 -> 771 muls (**2.79x**), and
fewer temporaries (167 -> 102, less register pressure). The raw kinematic Jacobian alone is 7.26x
cheaper analytically -- the full-kernel ratio is smaller (2.79x) because the log-map (Jlog6) +
adjoint composition is needed by both paths and is ~half the analytic cost. This is by far the
biggest op-reduction lever found for the TSR kernel -- vs the sincos-hoist's 1.25x -- and it
stacks (the analytic frame Jacobian's own FK still benefits from the snap and from sincos).

**Why cricket is currently 2.79x heavier:** it computes the Jacobian by CppAD reverse-mode
autodiff of the whole error function, which does not exploit that d(pose)/dq is the geometric
frame Jacobian (a well-known structured object). The analytic form is the textbook constrained-
kinematics Jacobian (`Jlog6 * Adjoint * frame-Jacobian`), exactly what pinocchio's own constrained
solvers use.

**To ship it** cricket's `emit_error_and_jacobian` / `trace_tsr_error` would compute the Jacobian
analytically (computeFrameJacobian + Jlog6 + adjoint) instead of `error_func.Jacobian`. Caveat:
the prototype uses the full se(3) log6 error; cricket currently uses a decoupled
[translation; so3_log] error, so a drop-in needs either switching to log6 (standard, arguably
better-conditioned) or the decoupled analytic Jacobian (Jlog3 on the rotation rows). Op-count is a
tight proxy for runtime on these pure-arithmetic kernels, so ~2.8x fewer muls ~= ~2.8x on the
Jacobian portion; end-to-end constrained-planning gain scales with the projection/Jacobian share.
Standalone tool: cricket/src/tracing/jac_compare.cc.

---

## Analytic geometric Jacobian shipped in cricket + tested on constrained_example (cd9a41f)

Wired the geometric-Jacobian path into cricket's `trace_tsr_error` (default; CRICKET_AUTODIFF_JAC=1
reverts). It builds d(err)/dq from pinocchio's analytic frame Jacobian + the validated composition
  J_trans = R_A (Jv - skew(R_X t_B) Jw),  J_rot = Jlog3(R_Y) R_B^T R_X^T Jw
instead of CppAD-autodiffing the error.

Correctness path (each step validated before the next):
1. Formula vs central finite differences: max err ~1e-10 (panda + both bipanda EEs).
2. pinocchio's Jlog3 traces (uses if_then_else) but emits **scalar `if`s** (CondExp) that do not
   compile on FloatVector -- exactly why cricket uses branch-free `so3_log_smooth`. Wrote a
   branch-free `so3_Jlog_from_log` (Jlog3 = I + 1/2[w]x + alpha[w]x^2, smooth alpha), validated vs
   pinocchio Jlog3 to ~1e-11 and vs FD to ~1e-10.
3. Generated panda kernel: **0 ifs**, tsr_error **2451 -> 634 muls (3.87x)**, 45 -> 18 trig.
4. Analytic vs autodiff kernel numerically identical at realistic (near-manifold) configs (~1e-8);
   they diverge only at pathological far-from-manifold configs where so3_log's theta~=pi branch cut
   ill-conditions both -- irrelevant to projection.

End-to-end (rebuilt vamp panda module, ran scripts/constrained_example.py):

| mode  | autodiff | analytic | speedup | correct? |
|-------|---------:|---------:|--------:|----------|
| plane | 1.4-1.5 ms | 1.0-1.1 ms | **~1.4x** | solved, all waypoints on manifold |
| line  | 0.2 ms | 0.2 ms | (too short to resolve) | solved, on manifold |

**~1.4x end-to-end on projection-heavy constrained planning** (much larger than the ~1.05x the
collision kernel gives unconstrained, because the Jacobian is a big share of projection). Kernel
3.87x; end-to-end 1.4x since collision/NN/sampling/Cholesky-solve are the rest. Single-EE TSR
done; the relative bimanual kernel (`tsr_bimanual_error`) is still autodiff -- the same treatment
(2 frame Jacobians + relative-pose Jlog composition) is the remaining follow-up. vamp's committed
robot headers are regenerable artifacts; regenerating them with current cricket picks up the win
(the test above rebuilt panda.hh, then restored it to keep vamp's tracked state clean).

---

## Free-flyer (Digit) analytic Jacobian -- correct but tangential-only; a net regression (a5be4d5)

Extended the analytic TSR Jacobian to free-flyer robots (nq != nv, Digit: nq=31 quaternion base,
nv=30). VAMP's constraint Jacobian is rows x nq, so it needs d/dq w.r.t. the RAW quaternion
components (Eigen unit formula), like autodiff. Derivation (all FD-validated in validate_ff.cc):
- joint columns: map 1:1 from the frame Jacobian (tangent col -> config col), exact to ~1e-10.
- base translation cols: world twist [e_k; 0].
- base quaternion cols: world twist [w_c x (t_X - t_base); w_c], w_c = vee(dR/dq_c * R_base^T),
  with a branch-free dR/dq (Eigen unit formula). Frame convention took several FD-guided fixes
  (LWA linear = velocity of the FRAME origin; angular in world; quaternion order x,y,z,w).

**The catch (rigorously characterized):** a RAW quaternion perturbation makes R_base non-orthonormal,
so autodiff's de/dq_quat includes a RADIAL (norm-changing) response that a rigid twist cannot
capture. Confirmed: (Jfd - Janalytic) for the 4 quaternion columns is exactly proportional to q_c
(spread of the ratio ~1e-8), and the TANGENTIAL residual after removing it is ~1e-10. So the
analytic Jacobian is tangentially exact -- identical to autodiff on the unit-quaternion tangent,
differing only in the manifold-normal direction the planner renormalizes away.

**End-to-end (digit_example.py, box pickup -> rack):**

| Jacobian  | kernel muls | solve time | iterations | on-manifold |
|-----------|------------:|-----------:|-----------:|-------------|
| autodiff  |        9232 |    55.6 ms |        353 | yes |
| analytic  | 3646 (2.53x)|   128.6 ms |       1005 | yes |

Both solve and stay on the manifold, but the analytic takes ~3x more RRTC iterations and is a net
**regression** (128 vs 56 ms). Cause: the chart is the null space of the stacked constraint
Jacobian (pivoted QR); the omitted radial columns perturb that basis, so the local planner steers
differently. Matching autodiff exactly would need the radial columns = the response of
so3_log to a quaternion SCALING (non-rotation dR), which is as involved as autodiff for those 4
columns -- no longer a simplification.

**Decision:** analytic is the default for revolute robots (panda: 1.4x e2e, validated); free-flyer
robots default to autodiff (digit: 56 ms, no regression). The validated free-flyer analytic is kept
behind CRICKET_FF_ANALYTIC=1. Net: the analytic geometric Jacobian is a clean win where nq==nv;
for over-parameterized (quaternion) configs it is correct-but-tangential and the chart sensitivity
makes autodiff the right default.

---

## Correction: free-flyer analytic is a net WIN, not a regression (multi-seed) -- cricket 4d31a98

The "digit regresses (1005 vs 353 iters)" result above was a single deterministic Halton draw.
Re-ran with xorshift x 8 seeds on the same digit box-pickup->rack problem, both builds current-cricket:

| config   | iters median [range] | ms median [range] | ms mean |
|----------|---------------------:|------------------:|--------:|
| autodiff |          236 [195-575] |      38.5 [31-92] |    46.4 |
| analytic |          266 [212-371] |    **35.2 [26-48]** | **35.5** |

Averaged over seeds the analytic is NOT a regression: iteration counts are comparable (analytic
+13% median but a *tighter* range -- autodiff has a 575-iter tail), and the analytic is net FASTER
(median ~1.1x, mean ~1.3x) because the 2.5x cheaper kernel more than covers the small iteration
increase. The single Halton 1005-iter draw was a pathological outlier -- above even the analytic's
own xorshift range (212-371). The chart IS mildly perturbed by the omitted radial columns (hence
+13% median iters), but it is not the catastrophe one seed suggested.

**Revised decision:** analytic is now the DEFAULT for free-flyer robots too (cricket 4d31a98). Net
across the session: the analytic geometric TSR Jacobian ships for all robots -- 1.4x e2e on the
revolute panda constrained problems, ~1.1-1.3x on the free-flyer digit whole-body problem, from a
2.5-3.9x cheaper kernel. Same recurring lesson, third time: single-realization comparisons lie;
average over seeds.

---

## The other Digit constraint kernels -- same analytic pattern (a885b2e)

Digit's whole-body stack uses four constraint Jacobian-PRODUCER kernels (all were autodiff). The
analytic geometric-Jacobian pattern (frame/CoM Jacobian + free-flyer tangential quaternion columns)
applies to all of them:

| kernel                | autodiff muls | analytic | status |
|-----------------------|--------------:|---------:|--------|
| tsr_error (feet, 4EE) |          9232 |     3646 | done (2.53x) |
| tsr_bimanual_error    |          3482 |    ~1800 | follow-up (relative-pose: 2 frame Jacs + Jlog) |
| com_jacobian          |          3013 |     2131 | **done (1.41x)** -- pinocchio jacobianCenterOfMass |
| closed_loop_error     |          1228 |    ~700  | follow-up (frame-Jacobian based) |

The **solve_* kernels** (solve_tsr_lm_inner 12896, _outer 18538, com/loop solvers, etc.) are the
larger per-iteration cost but take the Jacobian as INPUT and do a Cholesky/LM solve -- pure linear
algebra, no analytic-Jacobian angle. Their size is set by the projection method (InnerLM 24x24 vs
OuterLM 31x31 vs GradDesc); a cheaper method is the lever there, not codegen.

**CoM done:** d(com - mean(feet))/dq from pinocchio's analytic jacobianCenterOfMass + frame
Jacobians. FD-validated tangentially exact for the free-flyer (radial-spread 7e-10, tangential
residual 1e-10; joints exact 1e-10; base-translation columns vanish because com and feet translate
together). 3013->2131 muls (1.41x -- less than the frame Jacobian's since pinocchio's CRBA-based
CoM Jacobian is already fairly tight). Digit plans on-manifold with analytic tsr+com.

**Bimanual + closed-loop** follow the identical pattern (frame Jacobians of the involved frames +
the relative/loop error's Jlog composition, with the same free-flyer quaternion handling) and are
mechanical to add now that the pattern + free-flyer mapping are validated -- left as follow-ups.
Net: the analytic geometric Jacobian now covers the two largest Digit constraint kernels (tsr 2.53x,
com 1.41x); the remaining two are ~2x each on the same template.

---

## All Digit constraint Jacobians analytic (ada444a) -- bimanual + closed-loop done

Completed the analytic geometric Jacobian for Digit's last two constraint producers.

| producer            | autodiff | analytic | speedup |
|---------------------|---------:|---------:|--------:|
| tsr_error (feet)    |     9232 |     3646 |   2.53x |
| tsr_bimanual_error  |     3482 |     1492 |   2.33x |
| com_jacobian        |     3013 |     2131 |   1.41x |
| closed_loop_error   |     1228 |      918 |   1.34x |
| **total producers** | **16955**| **8187** | **2.07x** |

- **bimanual (relative pose)** e = se3_disp(X_l^-1 X_r C): the relative-pose Jacobian
  de/dv = D_disp(errT) Ad(C^-1) (xi_r - Ad(Z^-1) xi_l) from LOCAL frame Jacobians of the two EEs,
  D_disp(T)=[[R_T,0],[0,Jlog3(R_T)]], Z=X_l^-1 X_r, C=lTr^-1.
- **closed-loop (distance)** e = ||end-start|| - L: de/dq = u^T (Jpos_end - Jpos_start).

Both are invariant to a rigid base transform (relative pose / inter-frame distance don't change
when the whole robot moves), so the free-flyer base columns are exactly zero on the manifold --
FD-validated tangentially exact (validate_bicl.cc: translation cols ~0, quaternion discrepancy
radial-only, tangential residual ~5e-9). This is the *cleanest* free-flyer case: no quaternion
column derivation, just base=0.

All four producers now analytic (0 ifs, default; CRICKET_AUTODIFF_JAC=1 opts out); Digit's full
whole-body stack plans on-manifold. The solve_* kernels (larger, Cholesky/LM) remain autodiff-
independent -- pure linear algebra, no Jacobian to make analytic. Net: the analytic geometric
Jacobian now covers every constraint Jacobian VAMP generates (single/relative TSR, CoM, closed
loop), 1.3-2.5x each, from a shared frame-Jacobian + free-flyer-mapping template.

---

## Solve kernels: exploit Jacobian sparsity (f36679b) -- 2.2x, and it IS a codegen win

Earlier claim was that the solve_* kernels (Cholesky/LM) have "no analytic angle -- pure linear
algebra." That was wrong: they form J J^T densely, but J is structurally SPARSE. On Digit's feet
TSR the Jacobian is only 40% nonzero -- each end-effector's 6 error rows are nonzero only in that
frame's supporting joints (+ the free-flyer base): arm EEs 10/30 tangent cols, feet 14/30. Because
off-diagonal J J^T blocks (different limbs) share only the base columns, forming J J^T sparsely is
~4x cheaper.

`trace_solve_tsr` now computes a per-row nonzero mask from `model.supports[frame.parentJoint]`
(skipping the universe joint, whose idx_q is -1 -- a mask[-1] write was the initial double-free),
and `trace_solve_jacobian` builds structurally-zero J entries as constant 0 so CppADCodeGen folds
0*x out of J J^T and the Cholesky. The input layout is unchanged (VAMP still passes the full dense
J from the producer; the solver just ignores the known-zero slots).

| Digit TSR solver                 | dense | sparse | speedup |
|----------------------------------|------:|-------:|--------:|
| solve_tsr_error_lm_inner (default) | 12896 |  5768 |  2.24x  |
| solve_tsr_error_lm_outer         | 18538 |  8434 |  2.20x  |
| solve_tsr_error_gradient_descent |   744 |   312 |  2.38x  |

**Bit-exact** (the structural zeros are exact zeros in the producer output, so the solve result is
identical): same 348 RRTC iterations, on-manifold, ~1.19x additional end-to-end (34.0 vs 40.4 ms
single Halton) on top of the analytic producers. The relative-bimanual, CoM, and closed-loop
solvers are left dense (small: 902/158/158 muls) -- the relative case needs a two-chain,
base-invariant mask; a clean follow-up.

**Session net on constrained planning:** analytic producers (2.07x on the 4 Jacobian kernels) +
sparse solve (2.24x on the dominant solver), both exact/on-manifold -- the whole per-iteration
constraint cost is now materially cheaper, from codegen that exploits kinematic structure (frame/
CoM Jacobians, free-flyer mapping, and Jacobian sparsity) the generic autodiff+dense-solve missed.

---

## End-to-end evaluation: baseline vs updated constrained planning (8 xorshift seeds)

BASELINE (autodiff constraint Jacobians + dense solve) vs UPDATED (analytic geometric Jacobians +
sparse TSR solve), regenerating panda/bimanual_panda/digit for each config, each constrained
example over 8 xorshift seeds. Median [mean] planner wall-clock:

| example  | robot          | baseline ms | updated ms | speedup (med / mean) |
|----------|----------------|------------:|-----------:|---------------------:|
| line     | panda          |        0.10 |       0.10 | 1.00x (too fast)     |
| plane    | panda          |        2.50 |       1.70 | **1.47x** / 1.23x    |
| bimanual | bimanual_panda |       30.2  |      24.8  | **1.22x** / 1.14x    |
| digit    | digit (14+ff)  |       34.0  |      24.7  | **1.37x** / 1.49x    |

digit iterations: baseline 236 vs updated 246 median (comparable -- the free-flyer tangential
Jacobian's chart perturbation adds ~4%, more than covered by the cheaper kernels).

**Result:** analytic-Jacobian + sparse-solve delivers ~1.2-1.5x end-to-end on the substantial
constrained problems (plane's sphere cage, the bimanual grasp, the digit whole-body box transport),
multi-seed averaged. The win scales with projection-heaviness. Payoff of the full constrained-
planning codegen line: analytic frame/CoM/relative-pose Jacobians (producers 2.07x on Digit) +
Jacobian-sparsity-aware solve (2.24x on the dominant solver), all exact/on-manifold.

---

## Profile: where the digit constrained solve spends its time (m55)

`perf record -F999 --call-graph dwarf` on `digit_example.py` (box_top_shelf_pickup, xorshift
seed 0, 40 reps of the RRTC constrained solve), updated analytic+sparse build compiled `-g`.
1241 samples. Flat self-time, grouped:

| category | self% | what |
|---|---|---|
| **constraint SOLVERS** | **26%** | solve_tsr_error_lm_inner 23.8, solve_tsr_relative_lm_inner 2.3 (sparse LM projection step) |
| **constraint PRODUCERS** | **30%** | tsr_error 13.8, com_jacobian 8.9, tsr_bimanual_error 4.7, closed_loop_error 3.1 (analytic error+Jacobian) |
| constraint machinery | ~10% | HingedTSR/ClosedLoop/CoM wrappers, integrate_step, ConstrainedLocalPlanner |
| generic SIMD Vector math | ~9% | inlined into the producers/solvers above |
| **collision (fkcc + sphere_env)** | **~4%** | the fused FK+collision kernel |
| NN (KDTree scan/nearest) | **<2%** | |
| Extension bookkeeping | ~1% | |

Inclusive (children) view confirms the shape: `project_any` **51.6%** + `project_all` 9.6% ≈
**~60% of total is constraint-manifold projection**; simplify ~1.8%, NN <2%, collision <1% (incl).

**Takeaway.** The digit example is *projection-bound*, not collision- or NN-bound — the opposite of
the unconstrained MBM robots (there fkcc dominates; here it's ~4%). Every remaining win is in the
project loop: producing the stacked Jacobian (30%) and the LM linear solve (26%). Both are the
kernels we already JIT'd; the analytic+sparse work is aimed squarely at the hot 56%. Further wins
would have to come from *fusing* producer→solver (avoid materializing J then re-reading it — form
J·Jᵀ directly), or cutting projection *iterations* (better step/line-search), not from collision/NN.

---

## Fusing producer -> solver (emit J.Jt): investigated, mostly a dead end (m56)

**Q: could the fusion be done generically through the Python interface, or is it a JIT win?**
It is fundamentally a JIT capability -- but the profiling shows the JIT *already captured* most
of it, which is why little is left on the table.

### JJt producer->solver fusion is BLOCKED by the hinge
The projection loop (constraint_set.hh `descend`) is, per iteration:
`squared_error(q)` -> run_kernel (producer: J, e_raw) -> **vamp applies the hinge** -> `step_in_place`
-> solve_step (solver: J.Jt, Cholesky, J^T y). The hinge (`err = (e-lb).min0 + (e-ub).max0`; then
zero every J row whose hinged err == 0) sits *between* producer and solver and is **data-dependent**.
To fuse producer+solver into one tape you must bake that row-gate into the trace -> a CondExp ->
scalar `if(vector)` that won't vectorize on FloatVector (the same wall hit by fkcc_sincos). The only
escapes (pass the gate as an input, or recompute) are circular (the gate needs the error the kernel
produces) or double the FK. So no clean single-kernel fusion.
Separately, the win JJt fusion would supposedly add -- folding J's structural zeros when forming
J.Jt -- is **already done**: `trace_solve_jacobian` sets structurally-zero J entries to `ADCG(0)`
via `row_nonzero`, so CppADCodeGen already folds `0*x` out of J.Jt and the Cholesky (the m54 sparse
solve, 2.24x). Fusion would only remove the L1-resident J round-trip. Marginal.

### Shared-FK producer fusion: real but small, because the JIT already prunes
Every producer independently runs `computeJointJacobians` (tsr/bim/com/cl). Hypothesis: 4x redundant
full-body Jacobian. Measured (fuse_bench.cc, CppADCodeGen temp count = op proxy, digit nq=31 nv=30):

| | temps |
|---|---|
| full dense joint-Jacobian (what a generic pinocchio call materializes every time) | **152** |
| separate producers (JIT, Jacobian-only): tsr 50, bim 61, com 346, cl 42 | **SUM 499** |
| fused (one shared computeJointJacobians) | **389** |
| **fusion ratio** | **1.28x (22% fewer temps)** |

Two findings: (1) each JIT producer is *cheaper than one full dense J* (tsr=50 << 152) because
CppAD's dead-code elimination prunes `data.J` to just the columns the frame uses -- the feared "4x
redundant FK" is already gone. (2) Fusion still saves 22% by sharing the base/torso chain across
frames -- but that's Jacobian-only (an **upper bound**); the per-constraint error composition (log
maps, apply-M) doesn't share and dilutes it toward ~1.1-1.15x on the 30% producer slice => ~3-4%
end-to-end, for a real vamp refactor (ConstraintSet owning one fused kernel, distributing blocks).

### Why this answers the Python-vs-JIT question
The whole fusion story is *symbolic* -- DCE of unused Jacobian columns, CSE across FK -> J entries,
sparsity-folding of J.Jt, one shared FK pass. A generic Python interface can only chain
already-materialized dense arrays: it would call pinocchio 4x (>=4x152 dense-FK temps-equivalent,
no column pruning, no cross-call CSE) plus per-call dispatch on tiny matrices. So the JIT's win over
generic Python is large and **already banked** in the current kernels. The *incremental* win of
fusing them further is small precisely because the JIT is already good at the thing fusion exploits.

**Verdict:** don't build either fusion. The analytic+sparse line already sits on the optimized
frontier of the constraint path; residual producer/solver time is largely irreducible arithmetic.

---

## Projection descent loop: already tuned, no cheap iteration win (m57)

The profile's 60% is *inside* the LM projection loop (constraint_set.hh `descend`), so the lever
would be fewer iterations. Instrumented the loop (VAMP_PROJ_STATS, guarded) on digit transport
(box_top_shelf_pickup, xorshift): **17634 projections, avg 3.16 iters, 99.3% converge, 0.7% hit the
25-cap, 0 stalled/drifted.** Histogram is a sharp peak at 1-3 iters with a thin tail to 25.

Swept the knobs across 3 seeds:

| config | time | rrtc iters | avg proj iters | capped |
|---|---|---|---|---|
| **InnerLM descend_rate=1.0 (current)** | 20-33 ms | 209-246 | **3.16** | 116 |
| InnerLM rate=0.75 | 28-36 ms | 253-312 | 4.56 | 248 |
| InnerLM rate=0.5 | ~267 ms | 2444 | 11.7 | 2351 |
| OuterLM rate=1.0 | 31-52 ms | 155-269 | 3.86 | 44 |

**Conclusions.** `descend_rate=1` (full Gauss-Newton step) is optimal -- lowering it *strictly*
worsens everything (0.5 is catastrophic: 12 iters, 2351 caps). That the step never wants damping
means the additive per-constraint steps (Jacobi sum of 4 independent LM solves: TSR feet err=24,
CoM=2, closed-loop=2, bimanual=6) are **not fighting each other** -- so a joint 34-row stacked solve
would not cut iterations (and its 34^3 Cholesky costs ~2.7x the sum of the separate 24^3+... ones,
so it would be net slower). InnerLM beats OuterLM end-to-end (26 vs 38 ms avg). The loop is on its
frontier; the 60% is inherent -- ~72 well-converged projections per RRTC iteration, each doing
minimal work, is just the cost of keeping every steered waypoint on a 34-row manifold.

**The one structural inefficiency left is codegen, not loop-level.** ~24% of all producer runs
compute a Jacobian used only for the convergence check, never for a step (the final producer of each
converging projection + the 7.6% of projections already on-manifold on entry). Skipping it needs a
cheap error-only kernel for the check -- but a *separate* error-only kernel re-does forwardKinematics,
and the double-FK on the non-converged iters (3.16x) outweighs the one saved Jacobian, so it only
pays if the error path shares FK state with the Jacobian path == a fused dual-output kernel. That,
and the m56 shared-FK producer fusion (~1.28x producer arithmetic upper bound), are the only
remaining levers, each ~5% end-to-end and both codegen. No loop-level or parameter win exists that
maintains success rate.

---

## Fused dual-output (error-only check) kernel: sized (m58)

Follow-up to m57's finding that ~24% of producer runs compute a Jacobian used only for the
convergence check. Measured the key ratio (fuse_bench.cc, digit): **error-only path (forwardKinematics
+ updateFramePlacements + com position + placements, NO computeJointJacobians) = 61 temps vs full
producer 389 => 15.7%.** The Jacobian (computeJointJacobians + composition) is the other 84%.

Two implementable shapes, both restructure `descend` to check convergence with the cheap error path
and compute the Jacobian only when actually stepping:

| version | mechanism | saved/proj | producer time | end-to-end |
|---|---|---|---|---|
| pragmatic | error-only kernel + loop restructure; Jacobian call re-does FK (double-FK) | 135 temps | 8.4% | **~2.5%** |
| max | 2-stage: stage1 emits error + FK-state, stage2 builds Jacobian from that state (no double-FK) | 328 temps | 20.3% | **~6.1%** |

The pragmatic version pays error-only (61) extra on each of the ~3.16 stepping iters (double FK),
which eats most of the (P-E)=328 saved on the one convergence-only iter -> only ~2.5%. The max
version avoids the double FK but needs the generated stage1 to serialize ~30 SE3 joint placements
(~11 KB per rake-8 block) into a state buffer that stage2 reads -- a non-standard tape (pinocchio's
computeJointJacobians re-fed from external oMi state), plus the memory round-trip erodes some of it.

**What it would take.** Easy: the error-only kernel already exists inside `emit_error_and_jacobian`
(it is `error_func.Forward` before the `.Jacobian()` call) -- trace and emit just that. Moderate:
restructure `ConstraintSet::descend` + `HingedTSRConstraint` so the hinge/convergence check runs on
the error-only output and the full producer/solve fire only on stepping iters. Hard (max only):
FK-state I/O across the two kernels.

**Verdict.** ~2.5% for the buildable version, ~6% ceiling for the hard one -- and the restructure
touches exactly the hinge/convergence path m57 showed is delicately tuned (α=1 optimal, lowering it
catastrophic), so it carries real risk to the end-to-end success rate the change is supposed to
preserve. Reward-to-risk does not clear the bar. Constraint path is done; the analytic + sparse line
(m54, 1.2-1.5x end-to-end) already banked the accessible wins.

---

## Robonaut (r2c6) evaluation: baseline vs updated constraint+fkcc line (m59)

R2c6 = NASA Robonaut 2, free-flyer humanoid (nq=36, nv=35, 211 collision spheres), handrail-
climbing in microgravity (r2_handrail_example.py). Constraint stack is a single 3-EEF TSR
(left+right foot grippers pinned to the rail grasp, waist orientation held) -- pure TSR projection,
no com/closed-loop/bimanual to dilute. Not previously compiled into vamp; added to VAMP_ROBOT_MODULES.

### Kernel op-counts (faithful operator count -- NOT temp_variables, which undercounts)
IMPORTANT lesson: `*_code_vars` (CppADCodeGen temp count) is misleading -- it misses the huge inline
expressions in the autodiff Jacobian outputs. Counting arithmetic operators in the generated code is
the runtime-faithful metric and it reproduces the digit summary (tsr 2.56x, solve 2.23x).

| kernel | baseline (autodiff/dense) | updated | speedup |
|---|---|---|---|
| tsr_error (3-EEF producer) | 18378 | 5700 | **3.22x** |
| solve_tsr_error_lm_inner (sparse) | 16176 | 6040 | **2.68x** |
| solve_tsr_error_lm_outer (sparse) | 43404 | 7959 | 5.45x |
| ccfk (collision) | 9017 (nosnap) | 8878 (snap) | 1.016x |

Snap gives r2c6 only ~1.6% (clean URDF, no foldable placement noise) -- robot-specific, as on MBM.
Both producer (3.22x) and sparse InnerLM solve (2.68x) beat digit (2.56x / 2.23x): handrail is pure
3-EEF TSR on separate leg chains, exactly the structure the analytic+sparse work targets.

### End-to-end (r2_handrail, 10 seeds each, matched success)
Full baseline (autodiff + dense solve + no snap) vs updated (analytic + sparse + snap), same header
regenerated both ways, rebuilt + measured:

| | sum solve (40 steps) | sum states | avg proj iters |
|---|---|---|---|
| baseline | **768.5 ms** | 539 | 6-22 (many capped at 50) |
| updated | **81.8 ms** | 526 | 3-4 (0 capped) |
| ratio | **9.4x** | ~equal | ~2-5x fewer iters |

**The 9.4x is real and dominated by CONVERGENCE, not raw op-count.** Final tree sizes are equal
(539 vs 526 states) -- so it is not a search-luck artifact. Instrumented (VAMP_PROJ_STATS): the
baseline autodiff free-flyer Jacobian carries the unit-quaternion *radial* column, a renormalized
no-op that stalls each Gauss-Newton step, so projection takes 2-5x more iterations (avg 6-22 vs 3-4,
with 30-70 caps/seed vs 0). Compounded with ~3x more ops per iteration (analytic producer + sparse
solve) => ~9x. The analytic Jacobian's biggest payoff on a free-flyer is thus better *conditioning*
(drop the radial no-op), not just fewer flops.

**Flag for follow-up:** digit's earlier end-to-end (1.37-1.49x, m54) is also a free-flyer and should
show the same convergence mechanism; the modest number suggests that comparison may not have used the
full autodiff baseline. Worth re-measuring digit under this identical baseline-vs-updated methodology.

---

## Is preconditioning the solve a further lead? Mostly no -- the analytic Jacobian already was it (m60)

The r2c6 finding (autodiff free-flyer projection stalls, 6-22 iters, from the quaternion radial
no-op) raised: does explicit preconditioning of the InnerLM solve help further? Measured condition
numbers of the stacked LWA task Jacobian at 40 random configs (precond.cc, plain pinocchio):

| | kappa(J) | kappa(JJ^T+1e-6 I) [current] | row-scale(1/tol) | col-scale(Jacobi) | Marquardt lam*diag |
|---|---|---|---|---|---|
| r2c6 (18 rows) | 199 | 6.3e4 | 1.2x | 2.0x | 1.0x |
| digit (24 rows) | 165 | 5.6e4 | 1.0x | 1.2x | 1.0x |

**Interpretation.** The autodiff radial column is a near-NULL direction (effective kappa -> inf);
dropping it (the analytic Jacobian) is what took kappa down to ~200 -- i.e. the analytic fix already
*was* the preconditioner, which is why r2c6 saw 9.4x. Residual kappa(J)~200 is moderate; the classic
preconditioners buy little (row 1.2x, Marquardt 1.0x, best = Jacobi col-scale 2.0x on the Gram ==
~1.4x on J). At kappa~200 the residual 3-4 iters is closer to nonlinearity-limited than
conditioning-limited. One real structural point: InnerLM forms the Gram and SQUARES kappa (200 ->
6e4); with float kernels that is ~5 lost digits. A thin-QR solve on J (kappa~200, no squaring) would
avoid it -- the only place a further gain hides -- but QR codegen is materially harder than the
current Cholesky and OuterLM (also squares kappa) already loses to InnerLM end-to-end.

**Verdict.** Weak lead. Estimated ceiling ~5-10% from Jacobi column-scaling (one diagonal scaling,
same Cholesky), which is cheap to test empirically (scale + re-measure instrumented iters) if pursued.
The dominant conditioning win is already banked in the analytic Jacobian.

---

## Digit re-measured under the full autodiff baseline: 1.45x confirmed (m61)

Resolves the m59 flag. Same methodology as r2c6 (regenerate digit.hh as full autodiff+dense+nosnap
baseline vs analytic+sparse+snap updated; build+measure), digit_example box_top_shelf_pickup, xorshift
seeds 0-11, VAMP_PROJ_STATS instrumented:

| | solved | sum_solve (12 seeds) | avg proj iters (base->upd) | caps (base / upd) |
|---|---|---|---|---|
| digit | 12/12 both | baseline 510.1 -> updated 352.8 ms | **3.65 -> 3.25 (~unchanged)** | 1669 / 1849 |
| r2c6 (m59) | 10/10 | 768 -> 82 ms | **6-22 -> 3-4 (2-5x fewer)** | many / ~0 |

**Digit is genuinely 1.45x -- the earlier m54 number was correct, NOT understated.** The difference
from r2c6's 9.4x is now fully explained: digit's projection convergence is *insensitive* to the
autodiff radial pollution (3.65 vs 3.25 iters; baseline and updated cap almost equally), so its win
is nearly pure per-op kernel cost. r2c6's convergence is *highly* sensitive (baseline caps hard).

**Why the sensitivity differs = constraint TIGHTNESS x quaternion-coupling:**
- digit feet PINNED = [1e-3,1e-3,1e-3, 0.1,0.1,0.1]; arms FREE; plus com (position) + closed-loop
  (base-invariant) rows that carry no radial sensitivity and anchor the base.
- r2c6 PIN = [1e-4,1e-4,5e-3, 1e-2,1e-2,1e-2] -- ~10x tighter -- and *all* rows are free-flyer TSR.

Connects to m60: the Gram squares kappa to ~6e4 and the kernels are float (~5 lost digits). Near a
LOOSE tolerance (digit 1e-3) the radial-polluted step still reaches tolerance -> baseline ~= updated
iters. Near a TIGHT tolerance (r2c6 1e-4) the polluted step cannot resolve the last digits ->
baseline caps/iterates many times; the clean analytic step converges in 3-4. So the analytic
free-flyer Jacobian's *convergence* payoff scales with constraint tightness: ~1.5x (per-op only) for
loose/mixed problems, up to ~9x for tight pure-free-flyer-TSR problems.

**Implication for the m60 preconditioning lead:** it is a weak lead for digit-like loose problems but
could matter for r2c6-like TIGHT ones -- though there the analytic Jacobian already captures it. A
thin-QR solve (kappa not kappa^2) would specifically help tight-tolerance free-flyer TSR convergence
in float; still a real codegen lift, now with a clearer target profile if ever pursued.

---

## Thin-QR solve for r2c6: derisked by simulation, gives nothing -- do not build (m62)

Before building a traced QR kernel, simulated the exact Gauss-Newton projection loop offline
(qr_sim.cc: analytic LOCAL frame Jacobian, pinocchio integrate on the free-flyer, tol 1e-4) varying
only the solve method x precision, over 200 random perturb-and-project trials at several off-manifold
scales. In the realistic regime (perturb scale 0.04 -> 3.38 iters, matching real r2c6's 3-4, 200/200
converge):

| method | avg iters |
|---|---|
| Gram double (JJ^T Cholesky) | 3.38 |
| Gram float (current kernel)  | 3.38 |
| QR float (kappa, no squaring) | 3.38 |
| QR double | 3.38 |

All identical; at larger scales QR is if anything slightly worse (0.15: Gram-float 5.64 vs QR-float
6.04). **The thin-QR solve gives zero iteration reduction.**

**Corrects the m60/m61 float-precision speculation.** Gram-float == Gram-double at *every* scale, so
float precision is NOT limiting convergence -- the kappa^2-in-float concern never manifests. Reason:
the solve's relative step error (~kappa^2 * eps ~ 3.6e-3) is relative to the step magnitude, which
-> 0 near convergence, so absolute step error -> 0 and a tight 1e-4 tolerance is reached in the same
iters as an exact solve. So r2c6's baseline convergence blowup (6-22 iters) was NOT Gram-squaring; it
was purely the autodiff radial column (a rank deficiency), which the analytic Jacobian already
removed. With the analytic Jacobian, the Gram-float solve is already iteration-optimal.

**Verdict:** thin-QR (and preconditioning generally) is a dead end on top of the analytic Jacobian.
The residual 3-4 iters is nonlinearity-limited. The conditioning win was entirely the radial-column
removal, fully banked in m59's analytic free-flyer Jacobian. Constraint solve path is closed.

---

## r2c6 handrail: performance vs obstacle count (m63)

Profiled r2_handrail_example (perf -F999 dwarf, driver looping 40 obstacle_seeds/level, symbolized
r2c6 build) at increasing obstacle counts. Feasible range is narrow -- obstacles fill the leg-swing
space and block the rails, so gaits stop completing beyond ~32:

| n_spheres | gaits (of 8) | ms/step | states/step |
|---|---|---|---|
| 8  | 6 | 2.1  | ~13 |
| 16 | 6 | 8.8  | ~16 |
| 32 | 1 | 13.6 | ~22 |
| >=64 | 0 (infeasible) | - | - |

ms/step grows ~4x (8->16) while states/step barely moves -> the growth is per-state cost (more/harder
projections + more collision checks as the search routes around obstacles), not tree size.

**Self-time breakdown (normalized among native planning):**

| category | n=8 | n=16 |
|---|---|---|
| **PROJECTION** (tsr_error + solve_tsr_lm_inner + hinge + integrate + Vector) | **60.9%** | **58.9%** |
| COLLISION (fkcc + sphere_environment) | 21.0% | 23.9% |
| NN (KDTree) | 18.2% | 15.8% |

**r2c6 handrail is PROJECTION-BOUND (~60%) across the entire feasible obstacle range.** The two
collision terms scale differently:
- `sphere_environment_in_collision` (obstacle checks): 2.94 -> 5.84 = **2.0x for 2x obstacles == LINEAR
  in n_spheres**, but stays small (3-6%).
- `fkcc` (robot FK + self-collision, obstacle-independent): 6.27 -> 9.06 = 1.4x (search-driven only).

So the obstacle-dependent cost scales linearly but never dominates -- the problem goes infeasible
before obstacle count gets high enough. Projection stays the bottleneck at every feasible level, so
the analytic constraint kernels (m59) target exactly the dominant cost here.

**Surprise:** NN (KDTree) is ~16-18% -- far above digit's <2% (m55). r2c6's 36-dim free-flyer configs
+ many short gait-step trees make NN a real secondary cost, and a candidate future target for r2c6.

---

## CC tricks on r2c6 (mobile base): cost structure flips, but no new win (m64)

Retried the prior collision-checking tricks against r2c6 (Robonaut 2, free-flyer, 211 spheres), on
the hypothesis that a mobile base behaves differently than a fixed/mixed-base manipulator. Decomposed
the collision kernel (operator counts) and compared the floating base to the SAME robot fixed-base
(r2c6_minimal.json, identical 211 spheres):

| kernel | mobile (floating) | fixed base | base-transform overhead |
|---|---|---|---|
| eefk (FK to EEs)          | 1641 | 101  | +1540 (small kernel; base dominates it) |
| spherefk (FK + 211 spheres) | 7399 | 7069 | **+330 = 4%** |
| ccfk (+ bounding + self)    | 8878 | 8500 | +378 = 4% |

**Two findings:**
1. **The mobile base adds only ~4% to the collision kernel.** CppADCodeGen folds the base transform
   into the FK-chain root, so the quaternion base is applied once at the root, NOT 211 times per
   sphere. => the "transform N obstacles into the base frame instead of 211 robot spheres into world"
   trick has a ~4% ceiling. Dead.
2. **r2c6 collision is FK-PLACEMENT-dominated: spherefk = 7399 = 83% of ccfk.** This *flips* the MBM
   sparse-scene picture (m10: self-collision ~40% for the arm). r2c6 has 211 spheres over 68 links
   (mean 3.1/link) -> placing them is the cost, not self-collision.

The trick that fits an FK-placement-bound robot is the **bounding-sphere-gated FK** (skip a link's
fine spheres if its bounding sphere clears). But: (a) ceiling is modest -- ~3.1 fine spheres/link, so
a cleared link saves ~2.1 placements; (b) it was refuted on MBM (PHASE2 2B, <=1.00x) on a structural
codegen blocker -- the generated FK is flat SSA and a gate can't be inserted to skip a subset without
reordering; (c) the mobile base's one genuine advantage -- rake coherence (a whole limb clears across
all 8 SIMD lanes on a smooth edge) -- would raise the fire rate but does not fix the SSA blocker.

The only trick the mobile base genuinely favors is **per-edge broadphase** of the environment (a
compact robot translating through a sparse field prunes most obstacles per edge, vs a fixed
manipulator whose edge-AABB covers its whole workspace shell). But env-collision is only 3-6% of
native (m63), capped ~10% at the feasibility limit, and VAMP already does per-config radial
early-exit -- so the headroom is a few percent.

**Verdict.** The mobile base changes the collision *cost structure* (FK-placement-bound, not
self-collision-bound) -- the user's intuition is right there -- but it does not unlock a meaningful
CC win: the base transform is ~4%, the fitting trick (bounding gate) is codegen-blocked and low
ceiling, and collision is a planning-minority (21-24%) anyway. r2c6's cost is projection (m59-63),
not CC.

---

## Compiling/patching the environment for TAMP: synthesis + mobile-base reachability (m65)

Revisits the "compile the environment for replanning" question in the TAMP (many-queries-per-scene)
regime, informed by the prior overnight run (OVERNIGHT_FINDINGS.md) + this session's constraint
(m59-63) and mobile-base (m64) findings.

**Prior overnight verdict (still holds):**
1. Const-folding the scene into the kernel = ~1x (dead). VAMP's SIMD collision loop is already
   near-optimal; a full recompile is 0.6-8.8 s -- prohibitive to "patch" per TAMP mode switch.
2. The real lever is broadphase PRUNING via a cheap per-query obstacle PARTITION (42-1588 us, NO
   compiler). TAMP is exactly the amortization regime it needs (single queries don't pay; many
   queries per scene do). "Patching" on a pick/place = re-partition (us-ms), never recompile -- so
   the compile approach is doubly wrong for TAMP and the partition approach is naturally cheap to patch.
3. Robot-dependent: pruning wins for collision-bound LOW-sphere FIXED arms (UR5 2.4-4.3x); masked for
   FK-dominated HIGH-sphere robots (Fetch 111, Baxter 75). The only lever for those is scene-pruned FK,
   which needs BOUNDED per-link reachability.

**Two new reasons it does NOT help the robots we've been studying:**
- **Constraint dominance (m59-63):** r2c6/digit constrained planning is ~60% projection, ~20%
  collision (only 3-6% scene-dependent obstacle checks). Even perfect env-compilation touches <10% of
  the cost. The analytic Jacobian already owns the dominant term.
- **Mobile base kills reachability (m64 + reach.cc).** Scene-pruned FK needs each link's reachable
  AABB small. Measured per-link reachable AABB volume for r2c6:

| base freedom | mean AABB vol | vs fixed |
|---|---|---|
| fully fixed (identity) | 2.52 m^3 | 1.0x |
| + base rotation | 13.17 m^3 | 5.2x |
| + rotation + translation (mobile) | 25.36 m^3 | **10.1x** |

  A localized obstacle (0.6 m off-centroid) leaves **97-99% of links non-prunable for the mobile base
  vs ~45-58% for a fixed base** -- reachability pruning collapses, dominated by the base *rotation*
  (5.2x: a free-flyer can orient the whole robot any way, so every link can reach almost anywhere).

**Verdict by TAMP class:**
- Constrained mobile-base humanoid (r2c6/digit): env compilation is NOT the lever -- projection-bound,
  collision-minority, pruning masked by FK-dominance AND killed by mobile-base reachability.
- Unconstrained fixed-base arm in a workcell (UR5-class): partition-pruning IS the lever, amortizes
  over TAMP queries, 2-4x, no compiler, cheap to patch. The classic TAMP-manipulation win.
- Fixed-base high-sphere (Fetch/Baxter-class): scene-pruned FK is the high-ceiling UNBUILT lever
  (overnight step #1/#3) -- bounded reach makes localized scenes prune whole limbs; worth building
  for a fixed-base manipulation workcell, not for a free-flyer.

---

## Recovering pruning for mobile bases: base-scoped (per-edge) partition (m66)

m65 found reachability pruning collapses for a free-flyer (1-3% prunable) -- but that was a GLOBAL /
world-frame partition. The base rotation smears every link over the whole workspace only if you let
the base range freely. In the base frame the kinematics are base-INVARIANT, so per-link reach is
compact (fixed-base-tight); over one RRTC edge the base barely moves, so a base-scoped partition sees
compact-reach + small-edge-motion. Measured (reach.cc, r2c6, world-frame reach with base motion
bounded to the scope; prune = links whose swept AABB misses a peripheral obstacle):

| partition scope (base motion) | mean reach AABB | pruned |
|---|---|---|
| fixed base (reference) | 2.33 m^3 | 49% |
| **per-EDGE (+-0.15 rad / 0.15 m)** | 3.08 m^3 | **43%** |
| per-QUERY (+-0.5) | 10.2 m^3 | 8% |
| GLOBAL / world-frame (m65) | 28.7 m^3 | 1% |

**Answer to "can we prune mobile bases / check w.r.t. the base": yes.** A per-EDGE base-scoped
partition recovers 43% pruned vs 1% global -- ~40x, essentially the fixed-base rate. Mechanism: each
link's world-swept AABB over an edge = (precomputed base-frame reach, offline, base-invariant) (+)
(the edge's base-pose AABB, cheap per edge). Partition obstacles against those. No compiler; a cheap
per-edge partition, same shape as the overnight broadphase lever but scoped to the base.

**Granularity is the crux and it's mobile-base-specific:** per-edge works (base barely moves),
per-query already fails (8%), global fails (1%). For fixed manipulators any scope prunes; for a
mobile base the partition MUST be per-edge. This upgrades EDGE_BROADPHASE from "nice granularity" to
"the only granularity that works for a free-flyer."

Secondary ("sort w.r.t. base"): ordering each link's obstacle checks by base-relative distance gives
a faster early-out, but VAMP already sorts radially -- minor next to the per-edge scoping.

**Worth-it check:** for the constrained humanoids (r2c6/digit) collision is only ~20% (env-checks
3-6%), so recovered pruning buys a few percent. The real payoff is a COLLISION-BOUND mobile base
(mobile manipulator / AMR navigating clutter) -- there per-edge base-scoped pruning could deliver the
UR5-class 2-4x that world-frame pruning completely fails to get. That is the case to build it for.

---

## Prototype: per-edge base-scoped partition on real r2c6 (m67)

Built edge_partition.py: precompute each collision sphere's reach in the BASE frame (base-invariant,
offline); per edge read the base pose from q[0:7] (NO FK) and sweep the base-frame reach over the
edge's base poses -> per-sphere world swept-AABB. An obstacle outside it can't hit that sphere over
the edge; a sphere with no obstacle in reach has its FK skippable for the edge (r2c6 is FK-dominated).
Tested on a real dumped handrail scene (seed 0, 16 obstacles, 71 configs, 70 edges):

| approach | obstacles pruned | FK spheres skipped/edge | check-pairs kept |
|---|---|---|---|
| GLOBAL / world-frame (m65 baseline) | 0% | 7-21% | -- |
| per-edge, global joint limits | 14% | 17% | 37% |
| **per-edge + query-scoped reach** | **33%** | **79%** | **4.8%** |

**The two levers compound.** Per-edge base-scoping (base barely moves over an edge) and query-scoped
reach (precompute reach over the query's joint envelope, not global limits -- once per query/TAMP mode)
each help; together they skip **79% of FK per edge** and keep <5% of the sphere x obstacle matrix. The
global world-frame partition prunes 0% of obstacles -- exactly the m65 collapse. Base-scoping fixes it.

**Honest caveats:**
- Query-scoped reach used the solution trajectory's joint envelope (+10% margin) -- optimistic; a real
  planner would derive it from start/goal + explored set (looser). Even global-joints per-edge (no
  query knowledge) beats global 14%/17% vs 0%/7%.
- FK-skip is an UPPER bound: skipping a sphere's FK requires its kinematic ANCESTORS not be needed by
  surviving spheres (the FK-DAG survival factor, overnight #1, unmeasured here) and no self-collision
  partner needs it. Real FK saving < 79%.
- Conservative sampled AABBs (+margin); 0-false-negative not verified (needs interval-FK).

**Impact:** for r2c6 (projection-bound, collision ~20%) this is a modest end-to-end few-percent even at
79% FK-skip. The prototype's real value is the PROOF that base-scoped per-edge pruning works for a
free-flyer where world-frame pruning gets 0% -- so a COLLISION-BOUND mobile base (mobile manipulator /
AMR in clutter) is the target: there 79% FK-skip + 95% check-pair prune is the UR5-class win that the
world-frame approach cannot deliver. Next: measure the FK-DAG survival factor to bound the real FK cut,
then wire a per-edge Environment partition into the rrtc collision path for end-to-end numbers.

---

## FK-DAG survival factor for r2c6 per-edge pruning (m68)

m67's "79% FK-skip" was a per-SPHERE-PLACEMENT number; the real FK cut is bounded by the FK-DAG --
a joint's chain transform must run if ANY surviving sphere descends from it. Walked the kinematic
tree (30 joints feed the 211 spheres) and measured the surviving fraction per edge on the real scene:

| reach | sphere-placements skipped | FK-DAG survival | chain-FK saved |
|---|---|---|---|
| global-joints  | 17% | **100%** | **0%** |
| query-scoped   | 79% | **59%**  | **41%** |

**The DAG is the binding constraint, and it flips the verdict on reach quality:**
- GLOBAL reach: surviving spheres spread across all 30 joints -> 100% chain survival -> the trig-heavy
  chain FK (the dominant FK cost, per PHASE2) saves NOTHING, even though 17% of placements skip.
- QUERY-SCOPED reach: surviving spheres concentrate (legs, near the swing-space obstacles) so the arm
  chains (14 of 30 joints, tucked away from obstacles) prune -> 41% chain-FK saved.

Structural reason: r2c6 handrail obstacles sit in the leg-swing space, so LEG chains survive (distal
foot spheres reach obstacles -> whole leg chain runs) and ARM chains prune. The 41% ~ the two arms.

**So the real FK cut is ~41% (chain) not 79% (placements)** -- placements are cheap-and-many, the chain
is the trig cost, so the combined FK saving is chain-weighted, ~40-50% query-scoped, ~0% global.

**Two hard caveats:**
1. **Query-scoped reach is essential** -- global reach gives 0% chain saving. And query-scoped here
   used the solution envelope (optimistic); a real start/goal-derived envelope is looser -> less
   concentration -> lower saving. This is now the #1 sensitivity.
2. **Self-collision keeps the DAG at 100%.** This measured ENV-driven pruning only. Full self-collision
   needs every sphere's position, so it would force 100% FK-DAG survival unless self-collision is ALSO
   scene-pruned per edge. FK-pruning only pays if self-collision is pruned alongside (or is cheap
   enough to compute on a reduced set). For r2c6 self-collision is a kernel minority, but this must be
   handled for the 41% to be real.

**Bottom line:** the per-edge base-scoped partition + query-scoped reach can cut ~40% of r2c6's chain
FK -- real but bounded by the DAG, gated on query-scoped reach quality, and requiring self-collision
to be pruned too. For projection-bound r2c6 that is a few percent end-to-end; the case remains a
COLLISION-BOUND mobile base, where a ~40% FK cut on a FK-dominated collision check is the real prize.

---

## Conditional materialization via per-link bounding-sphere gate (m69)

Answers: can a link's fine spheres be materialized only if its bounding sphere intersects an obstacle
OR another link's bounding sphere (self-collision)? This is a BVH broadphase along the kinematic tree,
and the bounding-vs-bounding self condition directly targets m68's open self-collision caveat.
Measured (bounding_gate.py, real scene, 211 spheres -> 28 per-link bounding spheres, SRDF-filtered
9648 geometry pairs -> 239 self-candidate link-pairs), mean over 71 real configs:

| gate | links materialized | fine spheres kept |
|---|---|---|
| env only (bounding vs obstacle) | 4% | **2%** |
| env + self (vs obstacle OR self-partner bounding) | 63% | **84%** |

- **Env gate alone is huge**: only 2% of fine spheres near an obstacle -> 98% skippable. Obstacles are
  sparse relative to the robot; VAMP already gates fine env-CHECKS this way (PHASE0).
- **The self-bounding gate works**: it prunes **93% of self-collision link-pairs** per config.
- **But self-collision is the binding WALL**: the surviving 7% of pairs still touch 63% of links, so
  **84% of fine spheres must materialize -- only 16% skippable.** A humanoid is self-close-packed
  (arms near torso, legs near each other), so many link bounding spheres overlap even when not
  colliding, and the coarse per-link bound cannot separate them. This is m68's self caveat, quantified:
  self-collision, not obstacles, is what forces materialization.

**What it saves and doesn't:**
- Fine-sphere PLACEMENTS: ~16% skippable (env+self). Small (placements are ~1/3 of spherefk -> ~5% of FK).
- Fine CHECKS: env-checks 98% pruned, self-pairs 93% pruned -- large; the self-check bounding gate is
  the part VAMP may not already do, a real potential cut on the self-collision term.
- CHAIN FK (the trig cost): NOT saved -- the bounding spheres need the link transforms, so the whole
  chain runs. This gate is orthogonal to m68's reachability/subtree gating (which prunes the chain
  BEFORE FK). The two compose: reachability/subtree prunes chain; bounding gate prunes fine placements
  + checks.

**Verdict:** yes, conditional materialization works, and the self-bounding condition is exactly what's
needed to handle self-collision -- but on a self-close-packed humanoid it only frees ~16% of fine
placements (self-collision wall), while delivering large CHECK pruning. To materialize far fewer
spheres you need a FINER hierarchy (2-level bounds / per-sub-link) to separate the self-close-packed
links, or to accept that self-collision materialization is inherent to a compact humanoid.

---

## Which self-pairs force materialization on r2c6 (m70)

m69's 84% materialization wall is caused by a TINY structural set, confirming the intuition that most
pairs are avoidable. Per-pair bounding intersection over the trajectory (239 SRDF self-candidate
link-pairs, 71 configs):

| category | pairs | % |
|---|---|---|
| NEVER close (prunable for the whole motion) | 217 | **91%** |
| rare (<20% of configs) | 6 | 3% |
| ALWAYS close (>=80%, the forcing set) | 16 | **7%** |

**The 16 forcing pairs are geometrically inevitable, not behavioural:**
- within-limb folding: left_leg j1<->j3, j3<->j5, right_leg j1<->j3 (non-adjacent links in the SAME leg)
- torso hub: waist <-> {left/right arm proximal links, leg roots} (the waist is surrounded)
- NONE are arm-vs-arm, leg-vs-leg, or arm-vs-leg -- every cross-limb pair is in the 91% NEVER set.

**Two distinct wins this separates:**
1. **Self-collision CHECK pruning (per-query), the clean win.** 91% of candidate pairs never come close
   over a motion, so a per-query self-pair partition (analogous to the env broadphase: precompute
   per-pair min bounding distance over the query envelope, drop pairs that stay separated) cuts the
   self-check traversal ~10x (239 -> ~22 link-pairs; ~9648 -> ~900 geometry pairs). VAMP currently
   bound-gates each self-pair per config (m10: ~28-48 checks/config) but does NOT prune the
   never-close pairs for the query -- so this is new and it directly attacks the self-collision term.
2. **FK-placement materialization stays ~84%.** The 16 forcing pairs touch 17/28 links, so those links'
   fine spheres must materialize regardless (matches m69's 63% links / 84% spheres). This wall is
   inherent to the coarse per-link bound: the folding/hub links overlap and can't be separated. Only a
   FINER hierarchy (per-sub-link bounds splitting a leg's j1 from j3) could shrink it.

**So the answer: the forcing pairs are ~16 structural (limb-folding + torso-hub) pairs, and 91% of
self-pairs are per-motion prunable.** The materialization wall is not broad interaction -- it is those
16 inevitable pairs. Per-query self-pair pruning is the real, clean self-collision win the question
surfaces (decoupled from the FK-materialization wall, which needs a finer hierarchy).

---

## Prototype: per-query self-pair partition for r2c6 (m71)

Built per_query_partition.py. Self-collision distance is BASE-INVARIANT, so partition in the base
frame over the query's JOINT envelope: sample the envelope (base identity), build each link's
base-frame reach AABB, and prune any SRDF self-candidate pair whose two AABBs (grown by bounding
radii) stay separated -> can't collide over the query. Survivors checked per-config as today.
Verified against the real trajectory:

| envelope | pruned link-pairs | self-checks/config | false-negatives |
|---|---|---|---|
| whole 4-step gait | 50-56% | 1.5-2.1x fewer | 0 |
| **per single step (real query)** | **75-76%** | **~4.0x fewer** | **0** |

- **~75% of self-pairs prunable per query, ~4x fewer self-collision checks, 0 false negatives.** Robust
  to margin (75% at +10% and +25% envelope inflation; worst pruned-pair clearance 27 mm at +25%).
- Whole-gait is lower (50%) because it unions all 4 steps' envelopes; a real planner query is ONE step
  -> 75%. Per-step is the right granularity.
- The ~25% survivors = m70's 16 structural forcing pairs (limb-folding + torso-hub) + pairs that
  genuinely approach during that step.
- **Compiler-free**: cost is K joint-only FK samples once per query (amortized over the query's
  thousands of collision checks). VAMP currently has no per-query self-pair pruning (it bound-gates
  every SRDF pair every config, m10), so this is new and composes with the m66/m67 env partition.

**Caveats (same shape as the env partition):** envelope taken from the solution trajectory
(optimistic; a start/goal-derived envelope is looser but the +margin absorbs it); 0 FN verified for
the solution path only -- a provable bound needs interval-FK, the margin is the empirical stand-in.

**Impact:** for projection-bound r2c6 the self-collision term is a small slice, so ~1-2% end-to-end.
The value is (a) it PROVES per-query self-pair pruning is safe and effective (4x, 0 FN), and (b) it is
the collision-side win for a self-collision-bound robot (m10: self-collision ~40% of a check in sparse
scenes) -- there 4x fewer self-checks is material, and it stacks with the env-side per-edge partition.

---

## Overnight: 3 collision partitions wired into VAMP RRTC, correctness-verified, evaluated (m72)

Task: build each of the 3 partitions in VAMP proper, evaluate inside RRTC end-to-end, verify
correctness. Honest result: **all three integrated/tested and correctness-characterized; none delivers
a meaningful end-to-end speedup on the available robots, for three well-understood structural reasons.**
Work log: experiments/jit/partition/WORKLOG.md. Correctness bar: 0 false negatives (never miss a real
collision).

### Partition 1 — per-query self-pair (m71): INTEGRATED IN VAMP CORE
Changes: environment.hh (+active_self_pairs field, +copy-ctor, +binding def_rw); ccfk_template.hh
(compact self-loop iterates the pruned subset; empty => all pairs == unchanged). Regenerated digit +
r2c6, rebuilt. Partition computed in Python from VAMP's OWN spheres (module.fk) over the query joint
envelope. Wired into r2_handrail plan_step (R2_PRUNE), verified via R2_VERIFY.
- **Correctness:** sampled min-distance is NOT provably conservative. margin 3cm -> 2 FN / 8000
  configs; **margin 15-20cm + 1500 samples -> 0 FN, still ~91-93% pruned.** So correct-ABLE with a
  generous margin (a provable guarantee needs interval-FK, per the overnight caveat). Identity
  (empty active_self_pairs) reproduces baseline exactly (digit 246 iters, r2c6 same).
- **Perf:** with the correct margin (0 FN, no search divergence): baseline 2.01 vs pruned 2.12 ms/step
  -> **~5% SLOWER, no win.** r2c6's check is FK-DOMINATED (1116-value FK >> self-collision minority),
  so pruning 91% of self-pairs saves negligible time and the runtime index-loop costs slightly.
- **Root cause = architectural mismatch:** runtime self-pair pruning only exists on the COMPACT
  kernels (digit, r2c6) -- which are FK-dominated, so it can't help. The robots where self-collision
  is a large fraction (sparse-scene arms, m10 ~40%) use UNROLLED kernels (compile-time pairs) -> no
  runtime pruning. The partition helps exactly where it cannot be applied.

### Partition 2 — per-edge/query env (m67): TESTED END-TO-END (ur5 MBM, 7 scene sets)
Mechanism: reachability-pruned Environment (rebuild with obstacles outside the robot's query
swept-AABB removed). No kernel change.
- **Correctness:** 0 false negatives across all scenes (pruned path re-validated under FULL env);
  solved counts identical baseline vs pruned.
- **Perf:** only 0-18% obstacles pruned (MBM obstacles sit inside the workspace); ~1.0x on the one
  realistic solve (cage 23ms -> 1.03x). Confirms overnight: a global/per-query env prune is ~null --
  VAMP's sorted radial early-exit already skips far obstacles. The 2-4x overnight win was PER-SPHERE
  reachable-AABB pruning on a SYNTHETIC dense-obstacle THROUGHPUT bench, which needs per-link obstacle
  lists in the kernel (a real codegen change) AND a collision-saturated regime MBM planning never hits.

### Partition 3 — FK-DAG materialization gate (m68/m69): 2B-BLOCKED, not integrable
fkcc computes ALL sphere positions up front as flat SSA, then checks. Gating the FK placement needs
SSA reordering (refuted in PHASE2 2B). VAMP ALREADY gates the CHECKS behind per-link/per-pair bounding
spheres (the m69 gate is already shipped). Not attempted -- would require a codegen rewrite.

### Bottom line (honest)
The partitions are correct-verifiable but do not pay end-to-end here, because: (1) the robots that
support runtime pruning (compact humanoids) are FK/projection-bound, so pruning collision *checks*
saves little; (2) the env win needs per-sphere kernel pruning + a dense regime, not global env prune;
(3) materialization is codegen-blocked and VAMP already gates checks. A genuine win would need a
collision-bound mobile manipulator (slim arm, dense clutter) with a COMPACT kernel + per-link obstacle
partition + interval-FK envelopes for provable correctness -- none of which is the current robot set.
The self-pair integration is left in place (backward-compatible, empty=baseline); the env prune is a
harness-level rebuild (no core change). Correctness verified for both (0 FN with proper margins).

---

## Unrolled vs compact vs pruned kernels for large robots (m73) -- corrects m72

User: "try the unrolled kernels for the large robots." Generated r2c6 + digit with
compact_collisions=false: **unrolled r2c6.hh = 604k lines (25x the 24k compact); digit = 303k (16x).**
The unrolled r2c6 single fkcc function took ~5-6 min CPU to compile (gcc register allocator is
superlinear on huge functions). Built it and measured raw collision throughput (validate = FK +
self-collision, no obstacles):

| kernel | ns/call | vs compact |
|---|---|---|
| compact baseline (loops over 1208 cc_self_pairs) | 3131 | 1.00x |
| unrolled (straight-line, 604k lines) | 2612 | **1.20x** |
| **compact + per-query self-pair pruning (87% pruned, 0 FN)** | **1995** | **1.57x** |

**Corrects m72's "the partition doesn't help":** at the KERNEL level it clearly does -- 1.57x, and it
BEATS the unrolled kernel (1.31x faster than unrolled). Self-collision is ~36% of r2c6's FK+self check
(3131-1995 ~= 1136 ns), so pruning 87% of pairs cuts ~1000 ns. Unrolling helps too (1.2x, no loop
overhead) but loses to pruning AND costs 25x code + a ~6 min compile of one giant function -- not
viable for a large robot.

**Why it was masked end-to-end (m72):** r2c6/digit are PROJECTION-bound (~60% projection, collision
~22%, self a fraction of that), so a 1.57x collision-check speedup is ~a few % end-to-end, lost in the
1-2 ms/step noise. End-to-end handrail: unrolled 1.96 vs compact 2.01 ms/step == identical (FK/
projection dominate). So the partition's real value needs a COLLISION-BOUND robot with a compact
kernel -- but it is a genuine 1.57x collision-check win, not a null, and it is the fastest of the
three kernel variants.

**Net:** compact+pruned > unrolled > compact-baseline at the kernel level. The compact representation
+ per-query self-pair pruning is the right design for large robots (smallest code, fastest check,
0 FN); unrolling is a dead end (infeasible size/compile, and still slower than pruned).

---

## Full-suite kernel evaluation: compact / unrolled / compact+pruned (m74)

Built all 6 robots (added fetch+baxter to the module) in all-compact (Build A) and all-unrolled
(Build B). Collision-kernel throughput = validate (FK + self-collision, empty env), ns/call:

| robot | spheres | self-pairs | compact | unrolled | compact+pruned | prune% | fastest |
|---|---|---|---|---|---|---|---|
| ur5    | 40  | 55   | 498  | **439** | 447  | 75% | unrolled |
| panda  | 59  | 21   | 615  | **566** | 625  | 62% | unrolled |
| fetch  | 111 | 48   | 480  | **403** | 431  | 75% | unrolled |
| baxter | 75  | 349  | 932  | 905     | **552** | 92% | **pruned** |
| digit  | 124 | 170  | 1992 | **1688**| 1990 | 8%  | unrolled |
| r2c6   | 211 | 1208 | 3133 | 2597    | **1926**| 92% | **pruned** |

**Three clean facts:**
1. **Unrolled > compact-baseline for ALL 6** (10-20% faster): the compact runtime loops over
   cc_env_links/cc_self_pairs cost vs straight-line code. So the arms shipping unrolled is correct.
2. **compact+pruned beats unrolled only for the many-separable-pair robots**: baxter (552 vs 905 =
   1.64x) and r2c6 (1926 vs 2597 = 1.35x). For few-pair (panda 21, digit 8%-prunable) or small arms,
   pruning removes too little and unrolled's straight-line code wins.
3. **Unrolled is INFEASIBLE for the large robots**: r2c6 unrolled = 604k lines / ~6 min single-function
   compile; digit = 303k / ~3 min. So for large robots the real choice is compact vs compact+pruned,
   and pruned wins (r2c6 1.63x over compact).

**End-to-end (RRTC/handrail):**
- ur5/panda MBM (collision-bound arms): **unrolled fastest** end-to-end -- ur5 20 vs compact 23 ms,
  panda 12 vs 15 ms (~1.15-1.25x). compact+pruned ~neutral (0.94-0.98x).
- r2c6/digit: projection-bound (~60% projection, collision ~22%) -> ALL variants ~identical end-to-end;
  the kernel-level 1.35-1.63x pruned win is fully masked.

**CORRECTNESS (critical):** compact+pruned is **UNSAFE for unconstrained RRTC** -- ur5 MBM produced
**5 false negatives / 104 problems** because the global sampler checks configs far outside the
start-goal envelope, where pruned pairs collide. It is safe only for CONSTRAINED planners that stay
near the manifold (r2c6/digit handrail: 0 FN on paths) -- and even there not provable without
interval-FK. The throughput test's 0 FN was an artifact of testing over the same envelope.

**Verdict:** no partition variant delivers an end-to-end win anywhere in the suite. The robots where
pruning helps the kernel (large many-pair humanoids) are projection-bound (masked); the robots where
collision dominates (small arms) are better served by the unrolled kernel and are unsafe to prune.
The shipped design (small robots unrolled, large robots compact) is already near-optimal; the only
gap is large-robot compact could gain 1.35-1.63x from pruning IF a provably-safe (interval-FK) envelope
existed AND the robot were collision-bound -- neither holds for digit/r2c6.

---

## End-to-end ablation of the 4 kernel optimizations, digit + r2c6 (m75)

Cumulative ablation (each opt added on), end-to-end. Toggles: sincos = VAMP_ABLATE_SINCOS compile
flag (added fkcc_sincos generation to fkcc_gen -- it was JIT-path-only before, missing from offline
headers); snap = CRICKET_NO_SNAP; analytic = CRICKET_AUTODIFF_JAC; sparsity = CRICKET_NO_SPARSE.
Built + measured 5 levels (digit/r2c6-only module for fast builds).

**DIGIT (12 xorshift seeds, box_top_shelf_pickup; total solve ms):**
| level | ms | cumulative | this step |
|---|---|---|---|
| baseline (autodiff, no-snap, dense, no-sincos) | 528.2 | 1.00x | - |
| + sincos | 499.2 | 1.06x | **1.06x** |
| + snap | 506.0 | 1.04x | 0.99x (null) |
| + analytic Jac | 422.9 | 1.25x | **1.20x** |
| + sparsity (full) | 351.3 | **1.50x** | **1.20x** |

**R2C6 (10 handrail seeds; ms/step, normalizes gait-completion variance):**
| level | ms/step | cumulative | this step |
|---|---|---|---|
| baseline | 19.44 | 1.00x | - |
| + sincos | 19.53 | 1.00x | null |
| + snap | 19.30 | 1.01x | null |
| + analytic Jac | 2.28 | 8.52x | **8.46x** |
| + sparsity (full) | 2.16 | **9.01x** | 1.06x |

**Reading:**
- **Analytic Jacobian is THE win** -- r2c6 8.5x (the free-flyer convergence effect, m59/m61: tight
  pure-TSR drops the quaternion radial no-op that stalls Gauss-Newton), digit 1.20x.
- **Sparsity** adds a consistent 1.20x for digit; only ~1.06x end-to-end for r2c6 (the 2.2x sparse-solve
  kernel win is a smaller slice of r2c6's projection).
- **Sincos** helps digit +6% (more collision in the box scene) but is null for r2c6 (few obstacles).
- **Snap** is null end-to-end for both -- it helps collision-bound MBM (4-9%) but the humanoids are
  projection-bound so FK-sparsity gains are masked.
- **Cumulative: digit 1.50x, r2c6 9.0x** -- matching m59/m61. The split is exactly the m61 story: r2c6
  (tight pure-free-flyer TSR) is dominated by the analytic *convergence* win; digit (loose/mixed
  constraints) is a stack of modest per-op wins (sincos + analytic + sparsity, snap null).

fkcc_gen now emits fkcc_sincos for offline headers (was JIT-only). sincos left OFF in production by
default (compile flag); enabling it needs all robots regenerated + VAMP_ABLATE_SINCOS.

---

## Digit at 1e-4 tolerance: the analytic win grows with tightness (m76)

Direct test of m61's claim (analytic Jacobian's convergence payoff scales with constraint tightness):
tightened digit's foot-pin position bound from 1e-3 to 1e-4 (matching r2c6's grasps) via a runtime
env knob; measured analytic vs autodiff producer (snap+sparse held on), 12 seeds:

| digit foot-pin tol | analytic ms (iters) | autodiff ms (iters) | analytic win |
|---|---|---|---|
| 1e-3 (default) | 355 (3549) | 459 (3231) | 1.29x |
| 1e-4 (r2c6-tight) | 303 (2967) | 490 (3306) | **1.62x** |

**Confirmed.** Tightening 10x widens the analytic advantage 1.29x -> 1.62x. Mechanism is exactly m61:
at 1e-4 the analytic step gets FASTER (2967 vs 3549 iters -- a tighter pin gives a cleaner, more
decisive manifold) while the autodiff step gets SLOWER (3306 vs 3231 iters -- the quaternion radial
no-op can't resolve the tighter tolerance, so projections cap/fail and the tree grows). The gap widens
as predicted. Digit at 1e-4 moves toward r2c6's 8.5x but stays partway (1.62x) because digit still
mixes non-radial constraints (CoM position, closed-loop distance) that dilute the free-flyer TSR
effect, whereas r2c6 is PURE free-flyer TSR. So tightness AND constraint-purity together set how big
the analytic convergence win is -- the two knobs m61 identified, now both confirmed by moving one.

================================================================================
m77: CROSS-CONSTRAINT FK FUSION -- built cricket side, measured, verdict = marginal
================================================================================

REQUEST: "Let's do cross constraint fk fusion. Make the API for constraints always
take in the output of FK and joint Jacobians." Idea: every constraint producer
(tsr_error, com_jacobian, closed_loop_error, bimanual) currently recomputes FK
(computeJointJacobians + updateFramePlacements) independently. Compute it ONCE per
config, feed the shared FK state to each producer.

BUILT (cricket/src/tracing/constraints.cc, jit_patch):
 - fk_state_size(model) = 12*(njoints-1) + 6*nv  (oMi[1..] as R(9)+t(3) each, then J 6xnv)
 - setup_fk(): fk_input=false -> computeJointJacobians+updateFramePlacements (original path);
   fk_input=true -> unpack oMi/J from the tape, derive oMf = oMi[parent]*frame.placement.
 - trace_fk_jacobians(): shared kernel  q -> [oMi (12 each); J (6xnv)]
 - trace_tsr_error_fk / trace_closed_loop_error_fk: fk_input variants (thin _impl wrappers;
   PRODUCTION non-fk kernels are BYTE-IDENTICAL to before -- change is purely additive).
 - fkcc_gen emits fk_jacobians_code / tsr_error_fk_code / closed_loop_error_fk_code into the
   output JSON for measurement (template does not reference them -> generated headers unchanged).

CORRECTNESS (de-risked, no runtime harness needed):
 - feas_fk.cc already proved getFrameJacobian is BIT-IDENTICAL (0.00e+00, LWA & LOCAL, digit+r2c6)
   when data.J/oMf/oMi are SET rather than freshly computed.
 - Pack layout (trace_fk_jacobians lines 317-326) and unpack layout (setup_fk lines 278-289)
   are the same ordering: per joint R row-major then t, then J row-major. => tsr_error_fk /
   closed_loop_error_fk are correct BY CONSTRUCTION.

OP-COUNTS (digit, arithmetic ops per generated kernel):
 kernel                    current   fk-input
 tsr_error                    6641       5188   (FK portion of tsr = only 1453 = 22%)
 closed_loop_error            1773        507
 com_jacobian                 4390      (n/a)   keeps its own centerOfMass/jacobianCenterOfMass
 fk_jacobians (shared)           -       1484

DIGIT DESCEND ACCOUNTING (per iteration, InnerLM; avg 3.16 iters/projection):
 producers (current):  tsr 6641 + com 4390 + cl 1773               = 12804
 active solver (Inner): tsr_inner 11577 + com_inner 291 + cl_inner 291 = 12159
 total descend arith  ~= 24963 ops (+ integrate, small)

 fused (shared FK + fk-input tsr & cl; com unchanged):
   fk 1484 + tsr_fk 5188 + cl_fk 507 + com 4390 + solvers 12159 = 23728
   saving = 1235 ops = 4.9% of descend.
 fused + com also sharing FK (optimistic est com_fk ~ 4390-1484):
   saving ~= 2719 ops = ~11% of descend.

VERDICT: MARGINAL. Two reasons the win is small:
 1. FK is only ~22% of a producer's cost -- the analytic-Jacobian COMPOSITION (getFrameJacobian
    + Jlog3 per eef) is 78% and is inherently per-constraint (not shareable).
 2. The LM SOLVERS dominate descend (tsr_lm_inner alone = 11577, a 24x24 J J^T factorization) and
    are completely untouched by FK-fusion. Producers are only ~half of descend arithmetic.
 Plus: only MULTI-producer robots benefit. r2c6 (single TSR) and the manipulators (single TSR)
 get ZERO. Only digit gains, and only ~5-11% of its descend arithmetic => ~1-3% end-to-end
 (producers are ~17-20% of digit runtime per the analytic-Jac 1.20x ablation; solvers untouched).

DECISION: NOT wiring the vamp ConstraintSet API rework (every constraint class + all 6 robots
regenerated/revalidated) for a ~1-3% digit-only gain. cricket side is built + proven; left as a
capability. The real descend bottleneck is the LM solver factorization, not redundant FK.

================================================================================
m78: SOLVER EXPLORATION -- bordered/Woodbury InnerLM: 1.79x ops, 0% wall-clock
================================================================================

REQUEST: after m77 (FK fusion marginal, "LM solver is the real bottleneck"), try the SOLVER.

STRUCTURE: descend is block-Jacobi (constraint_set.hh step_in_place -> each constraint c->step()
independently). Digit's tsr solves its OWN InnerLM: J^T (J J^T + lambda I)^{-1} e with J 24x31
(4 eefs x 6). solve_tsr_error_lm_inner = 11577 ops (5768 mul, dense 24x24 Cholesky w/ 24 sqrt) --
the single largest descend kernel. Column sparsity (per-eef support) is already exploited but
J J^T stays DENSE: all 4 limbs couple through the 6-DOF floating base.

BORDERED/WOODBURY (prototype trace_solve_tsr_bordered): the base coupling is rank<=6. Digit's
limbs are column-disjoint except the shared base, so J J^T + lambda I = A + G G^T with A block-
diagonal (per-eef 6x6, limb columns only) and G = base columns (24x7). Woodbury replaces one dense
24x24 Cholesky with four 6x6 + one 7x7 Schur solve.
  op-count: 11577 -> 6476  (1.79x fewer ops).  sqrt 24 -> 31.
  CORRECTNESS: emitted grad matches dense grad to rel ~1e-9 (Cholesky roundoff), seeds 7/11/29/101.
  Drop-in: same tape layout (J row-major, then err); gated behind CRICKET_BORDERED_SOLVE.

END-TO-END (deployed into digit.hh, rebuilt _core_ext, verified 31-sqrt kernel live):
  RRTC digit box-transport: 32.0 ms/rep baseline -> 32.0 ms/rep bordered (identical, same 319 iters).
  Isolated projection (20k project() calls): 56.41 us baseline vs 57.01 us bordered (bordered a hair
  SLOWER, within noise).  => the 1.79x op reduction is 0% wall-clock.

WHY 0% -- decomposition (project() max_iterations=1, InnerLM vs GradDesc isolates the solver):
  InnerLM 1-iter = 8.61 us,  GradDesc 1-iter = 7.45 us  => full dense solver = 1.16 us = 13% of iter.
  So the LM solver is only ~13% of a projection iteration; a 1.79x cut saves ~6% of the iter in
  theory, and the bordered kernel's EXTRA statements (531 vs 381 lines: more temporaries / worse
  pipelining despite fewer flops) eat that back to ~0.

META (corrects m77): the LM solver is NOT the digit bottleneck. Neither FK (m77) nor solver
arithmetic moves wall-clock. Static op-count is a poor predictor here -- the analytic-Jac win
(1.20x) came from a kernel that also pipelined/vectorized better, not merely fewer ops. The ~57 us
projection is dominated by the PRODUCERS (FK + analytic Jacobian composition) + framework/pybind
overhead, and those are already optimized (analytic Jac). CAUTION: the python planning benchmark is
heavily python-bound (perf: ~90% interpreter over the whole process at 400 reps) -- measure kernel
changes via isolated project() deltas (InnerLM-GradDesc), NOT the RRTC wall-clock.

DECISION: bordered solver REVERTED (proven correct, real 1.79x op win, but 0% wall-clock). Not worth
carrying. If projection wall-clock ever matters, the lever is the PRODUCERS or reducing projection
CALL COUNT / iterations -- not solver arithmetic.

================================================================================
m79: ITERATION COUNT -- the real projection cost is iters, not per-iter arithmetic
================================================================================

REQUEST: after m78 (solver arithmetic = dead end), "look at iteration count then let's drop."

MEASUREMENT (digit box-transport, InnerLM alpha=1, feet+com+loops+bimanual):
 - RRTC projection-cap sweep (real planning):
     cap<=4  -> planning FAILS (projections can't converge, every extension rejected)
     cap=6   -> solves but THRASHES: 13281 RRTC iters, 380 ms
     cap=10  -> 705 iters, 34.5 ms
     cap=25  -> 348 iters, 34.2 ms (default)
   => real projections routinely need >4 iters; the long tail needs 10-25.
 - Per-projection iteration count (python bisection: minimal max_iter to converge, 1500 samples):
     step sigma=0.005 -> mean 9.88, median 6, p90 24, max 30, 93% converge
     step sigma=0.01  -> mean 9.68, median 6, p90 23, max 30, 81% converge
     step sigma=0.02  -> mean 10.9, median 8, p90 24, max 30, 74% converge
     step sigma=0.03  -> mean 13.0, median 9, p90 26, max 30, 61% converge
   Heavy-tailed: median ~6 but p90 ~24. (NB: contradicts the "avg 3.16 iters" I had on file --
   that was wrong / a different problem. Real digit box-transport is mean ~10.)
 - CDF is roughly linear (constant per-iter convergence increment) = LINEAR convergence, the
   signature of block-Jacobi simultaneous full-step, NOT the quadratic convergence a coupled
   Gauss-Newton step gives.

STRUCTURE: descend is block-Jacobi (constraint_set.hh step_in_place -> each constraint c->step()
independently, then sum). The 4 constraints couple heavily through the shared floating base + legs;
applying all full steps simultaneously (each assuming the others don't move) overshoots -> slow
linear convergence -> ~10 iters.

WHY THIS MATTERS (unlike FK/solver arithmetic): iteration count MULTIPLIES the whole per-iter
producer+solver cost, and projection is producer-bound. Cutting iters ~10 -> ~3-4 (quadratic
convergence) would be a real ~2.5x on projection wall-clock -- the first lever found that would
actually move the needle, because it removes WHOLE producer passes, not just flops within one.

LEVER (identified, NOT implemented): replace block-Jacobi with a COUPLED (stacked) Gauss-Newton
step -- build the stacked Jacobian [tsr; com; loops; bimanual] (~29-35 rows x 31) once per iter
(error_jacobian() already assembles it) and take ONE InnerLM step on the whole stack. Quadratic
convergence near the manifold -> far fewer iters. Cost: a bigger solve per iter (35x35 vs the
block solves) + a stacked-descend path (step_in_place is hardcoded Jacobi). Gauss-Seidel
(sequential, each constraint sees prior updates) is a cheaper middle ground (same per-iter cost,
better-than-Jacobi convergence).

DECISION: per user, DROP the projection-perf line here. Instrumentation reverted, module rebuilt
clean. Recorded because iteration-count / coupled-solve is the ONE lever with real wall-clock
upside if projection perf is ever revisited -- the arithmetic levers (FK m77, solver m78) are not.

================================================================================
m80: COUPLED GAUSS-NEWTON PROJECTION -- the iteration lever works: 1.31x end-to-end
================================================================================

REQUEST: "try the gauss newton" (reversing the m79 drop). Replace block-Jacobi descend
(constraint_set.hh step_in_place: each constraint steps independently, linear convergence) with a
COUPLED Gauss-Newton step: assemble the stacked hinged+active-masked error e and Jacobian J of ALL
constraints, take ONE normal-equations step g = (J^T J + lambda I)^{-1} J^T e. Quadratic vs linear.

IMPLEMENTATION (vamp, opt-in `ConstraintSettings.coupled`, default false):
 - settings.hh: `bool coupled`. constraint_set.hh: step_coupled_in_place(). Scalar-extract all rake
   lanes via the existing extract_error_jacobian() virtual (+ replicate squared_error's active-row
   mask), then ONE SIMD Cholesky over FloatVector rows (all lanes at once), n=dim=31 fixed.
 - OuterLM form (J^T J, n x n) chosen: robust when active-row count != n (InnerLM J J^T is rank-
   deficient once masking drops the free rows -> gave ZERO convergence; OuterLM fixed it).

ITERATION COUNT (project() bisection, block-Jacobi vs coupled):
   sigma=0.005: block mean 11.5 (94% conv) -> coupled 2.33 (100%)
   sigma=0.01 : block 12.6 (92%)  -> coupled 2.78 (100%)
   sigma=0.02 : block 13.4 (83%)  -> coupled 3.36 (100%)
   sigma=0.03 : block 16.2 (73%)  -> coupled 3.64 (100%)
   => ~4-5x fewer iterations AND 100% convergence (quadratic, as hypothesized in m79).

WALL-CLOCK:
 - scalar per-lane Eigen prototype: 2.6x SLOWER e2e (8 scalar solves/iter + redundant eval swamp it).
 - SIMD Cholesky (one factorization for all 8 lanes): projection throughput 55.5 -> 48.8 us
   (1.14x + 100% vs 92% conv); END-TO-END RRTC digit box-transport (pick->rack):
       block-Jacobi: 348 iters, 32.7 ms   coupled: 228 iters, 24.9 ms  => 1.31x, path on-manifold.
   Coupled needs FEWER RRTC iters (228 vs 348) because reliable projection -> fewer failed extensions.
 - STILL has a redundant kernel eval (step_coupled calls evaluate_error_jacobian while the descend
   loop's squared_error also runs the kernels -> ~2x producer/iter). Removing it (read the hinged+
   masked cache instead of re-evaluating; needs a per-type SIMD-cache reader) should push toward ~1.6x.

This is the FIRST real wall-clock win in the projection-perf line: op-count arithmetic (FK m77,
solver m78) was a dead end, but the ITERATION lever (m79) pays off -- coupling removes whole
producer passes, which is what actually moves wall-clock.

LIMITATIONS (opt-in experimental): ignores ProjMethod (always OuterLM), does not mask pinned-sampler
columns (fine for halton/no-pins; would be wrong under PinnedRNG), and carries the redundant eval.

--- CORRECTION to m78 ---
Python loads a COPY of _core_ext.abi3.so in site-packages, NOT the ninja build-dir output; `ninja`
does not reinstall. So today's earlier python measurements ran a STALE module. The m78 bordered
"0% end-to-end" was never actually loaded (VAMP_PROJ_STATS also never fired for this reason) -> that
specific end-to-end claim is UNSUPPORTED. Bordered's op-count (1.79x) + correctness still stand, and
it's dropped anyway. m79's baseline iteration numbers used the stale module = valid baseline behavior.
WORKFLOW FIX: after every `ninja _core_ext`, cp the .so to site-packages before measuring.

================================================================================
m81: COUPLED GAUSS-NEWTON productionized -- full suite: digit 2.06x, r2c6 neutral
================================================================================

Removed the redundant kernel eval + productionized the m80 coupled Gauss-Newton projection.

CHANGES (vamp, opt-in ConstraintSettings.coupled):
 - No re-eval: step_coupled reads the hinged+active-masked cache that the descend loop's
   squared_error(q) already populated (new virtual Constraint::stacked_cache(), overridden in
   HingedTSRConstraint [TSR+bimanual], CoMConstraint, ClosedLoopConstraint -- each reads its own
   post-squared_error solve buffer directly; no re-hinge/re-mask, since every type's squared_error
   already applied the correct masking). Assembled + solved in SIMD (FloatVector Cholesky, all lanes).
 - Excludes velocity-only Pfaffian constraints (new Constraint::projects(), false for Pfaffian).
 - Pinned-sampler columns zeroed in the assembled Jacobian (handles PinnedRNG).
 - Single-constraint fallback: if <=1 projecting constraint, step_in_place reverts to block-Jacobi
   (nothing to couple; the n x n coupled solve is strictly costlier than the constraint's own
   generated solver). Makes coupled=true safe to leave on for any problem.
 - `coupled` overrides `method` (always Gauss-Newton / OuterLM). Default path (coupled=false) is
   byte-identical to before (digit still 319/348 iters).

FULL-SUITE EVALUATION (the constrained problems previously evaluated):
 - DIGIT (box-transport, canonical digit_example.py, 10 reps, halton, feet TSR + CoM + closed-loops
   + bimanual = 4 coupled constraints):
       block-Jacobi: 30.9 ms/rep, 319 iters      coupled: 15.0 ms/rep, 261 iters   => 2.06x
   All waypoints on manifold (both). (Double-eval removal took it from 1.31x -> ~2x.)
 - R2C6 (handrail gait, SINGLE TaskSpace constraint): coupled auto-falls-back to block-Jacobi
   (1 projecting constraint) -> identical to baseline (seed 3: 11.5 ms both, gait completes).
   Confirmed the fallback is right: before adding it, coupled-on-single-constraint used OuterLM
   (n x n) vs block's 6x6 InnerLM and ran ~5.8x SLOWER with no robustness gain.

TAKEAWAY: coupling pays off exactly when constraints couple (multi-constraint humanoid stacks:
2.06x on digit), and is a no-op elsewhere. This is the real projection-perf win the whole line was
looking for -- from the ITERATION lever (m79), not per-iteration arithmetic (FK m77 / solver m78,
both dead ends). Convergence is also more robust (100% vs 73-99% on hard projections, m80).

STATUS: opt-in (default false), correct, no-regression, exposed via `--coupled` in digit_example.py
and r2_handrail_example.py. Candidate to enable by default given the single-constraint fallback.

================================================================================
m82: WHEN coupled GN helps -- heterogeneous multi-kernel stacks, not fused multi-eef
================================================================================

Q: does r2c6 benefit from coupled GN with the waist-upright constraint + the handrail?

FINDING: the r2c6 handrail ALREADY includes waist-upright -- it is the 3rd eef of the single
TaskSpaceConstraint (n_eef=3: left tip, right tip, waist_center; WAIST_BOUND = pos free, roll/pitch
<= 0.05 rad, yaw <= 0.5, about WAIST_REFERENCE_RPY). So feet + waist are all TSR eefs sharing ONE
tsr_error kernel, already coupled inside that constraint's InnerLM solve.

r2c6 projection (right foot pinned + waist upright), min-iters + throughput, dim=36:
  sigma=0.05:
    combined (1 TSR, 3 eefs)  block-Jacobi : 4.19 iters, 10.3 us/proj   <- current, optimal
    split (feet-TSR + waist-TSR) block-Jac  : 11.66 iters, 46.4 us/proj  <- splitting loses coupling
    split (feet-TSR + waist-TSR) coupled GN : 4.38 iters, 74.6 us/proj   <- coupled RECOVERS iters...

INTERPRETATION -- the rule for when coupled GN pays off:
 - Coupled GN recovers the cross-constraint coupling that block-Jacobi's independent stepping LOSES.
   Proof: splitting r2c6's feet+waist into two constraints makes block-Jacobi linear (4->12 iters);
   coupled GN restores the 4-iter quadratic count.
 - It only WINS in wall-clock when the constraints CANNOT be fused into one kernel -- i.e. they use
   DIFFERENT generated kernels. DIGIT is exactly this: feet-TSR + CoM + closed-loop + bimanual are
   four distinct kernels (tsr_error / com_jacobian / closed_loop_error / tsr_bimanual_error) that
   must be separate constraint objects -> block-Jacobi steps them independently (linear) -> coupled
   GN gives 2.06x (m81).
 - When the constraints ARE the same kernel (r2c6: feet + waist are all TSR eefs), the right model is
   ONE multi-eef TaskSpaceConstraint, which already couples them in a single solve and is ~7x faster
   than splitting-then-coupling (splitting doubles kernel evals + inflates the coupled solve to
   36x36). Coupled GN correctly FALLS BACK to block-Jacobi here (1 projecting constraint) -> neutral.

BOTTOM LINE: coupled GN is the right tool for HETEROGENEOUS constraint stacks (humanoid whole-body:
TSR + CoM + loops + bimanual = digit, 2.06x). It is neither needed nor beneficial for a manifold
expressible as a single multi-eef TSR (r2c6 handrail) -- fuse those into one constraint instead.
The single-constraint fallback (m81) makes coupled=true safe to leave on either way.
