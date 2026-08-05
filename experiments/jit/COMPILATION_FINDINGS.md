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
