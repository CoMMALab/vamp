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
