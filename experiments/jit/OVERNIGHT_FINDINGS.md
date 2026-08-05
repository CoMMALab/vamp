# Overnight findings — Cost-Aware JIT Scene Specialization

**Run:** autonomous, one night. **Branches:** `jit_patch` in `vamp` and `cricket`.
**Env:** micromamba `jit_patch` (clang/LLVM 22.1.8, pinocchio, cppad, cgal, nanobind).
**Host:** i9-14900K, AVX2 (no AVX-512), rake=8.
**Scope run:** M0 build → M1 baselines → M2 const-fold specialization → **M2.5 broadphase
pruning (added mid-run)** → M3 latency + gate decision. Per-phase detail in
`OVERNIGHT_LOG.md`; raw data in `results/*.csv`.

---

## TL;DR (the decision-gate verdict)

1. **The premise holds but is tight.** VAMP plans in ~40 µs median (Panda MBM); a full-robot
   JIT compile is 5.7–8.8 s. Any per-query specialization must be radically cheaper.
2. **Const-folding the scene into the kernel is a dead end (~1× throughput).** VAMP's generic
   SIMD collision loop is already near-optimal; baking obstacles as immediates removes loads
   that weren't the bottleneck. **Bit-exact** (0 mismatches over millions of checks), but no win.
3. **Broadphase pruning is the real, AOT-impossible lever — and it works, conditionally.**
   Per-sphere reachable-AABB pruning gives **UR5 a correctness-verified 2.4–4.3× speedup**
   (0 false negatives). But it is **strongly robot-dependent**: broad-reach arms (Panda) don't
   beat VAMP's existing radial early-exit, and high-sphere robots (Fetch) are FK-dominated so
   their large pruning factor doesn't cash out.
4. **The winning lever needs NO compiler.** Pruning is a per-query obstacle **partition**
   (42–1588 µs), ~500–1000× cheaper than clang codegen (0.64–1.09 s) — and it *delivers* the
   win that codegen doesn't. **⇒ The design doc's expensive copy-and-patch / LLVM-min-pipeline
   specializer (Component 2) is OFF the critical path.** This is the biggest course-correction
   from the run.

---

## What each result says

### M0/M1 — premise + T0 baseline
- `load_robot`: Panda 8.81 s, UR5 5.74 s; Panda solved 10/10 MBM, 40 µs median → premise intact.
- Generic `Robot::fkcc<8>` throughput: 0.24–3.5 M fkcc/s. In **dense** scenes collision_frac≈1,
  so VAMP's **sorted radial early-exit** already skips most obstacles — the bar to beat is that
  early-exit, not "check everything."

### M2 — const-fold + unroll + type-prune (Component 1, levels 1–2)
- **Correctness: bit-exact** (structurally identical to generic; 0 mismatches).
- **Throughput 0.98–1.26×** (isolated kernel), 0.85–1.16× (robot). Best only at tiny N; →1× as
  per-obstacle FLOPs dominate. **Const-fold is not the lever.**
- Scene-kernel clang compile: 0.64 s (N=10) → 1.09 s (N=1600), mostly fixed header-parse.

### M2.5 — per-sphere reachable-AABB broadphase (Component 1, level 3)
Pruning factor vs VAMP's radial early-exit (`aabb_beats_radial`, >1 ⇒ AABB wins):

| robot | n_spheres | aabb_beats_radial | throughput speedup | note |
|-------|-----------|-------------------|--------------------|------|
| **UR5**   | 40  | 1.5–2.3× | **2.4–4.4×** | clean win |
| Fetch | 111 | 2.8–4.8× | ~1.0–1.16× | pruning masked by FK-dominance |
| Panda | 59  | 0.53–1.05× | 0.99–1.34× | broad reach → AABB≈radial |
| Baxter| 75  | 0.61–1.12× | 1.01–1.03× | broad reach, keeps 77% of obstacles |

- **fn_rate = 0.000000** (sampled AABB + 5 cm margin is empirically conservative over 100k×≤111).
- Per-sphere reachable AABBs are **robot-intrinsic** (joint-box only) → computable **offline**;
  only the obstacle partition is per-scene (O(n_spheres·N)).

### M3 — latency + gate
| specialization | cost | throughput win |
|---|---|---|
| full-robot JIT (clang→ORC) | 5.7–8.8 s | baseline |
| scene const-fold, clang -c | 0.64–1.09 s | ~1× (no) |
| **broadphase partition** | **42–1588 µs** | **UR5 2–4× (yes)** |

Even the µs–ms partition > UR5's ~40–100 µs plan time, so **single easy queries don't pay** —
pruning amortizes over **streaming/workcell** and **long/optimizing runs** (doc §6/E5).

---

### E-FK1 — FK vs collision cost split (`m4_fk_split.cc`, `results/m4_fk_split.csv`)
`fk_fraction = t_fk/t_cc`; `collision_ceiling = t_cc/t_fk` (max speedup if collision → 0):

| robot | FK fraction | collision ceiling | reading |
|-------|-------------|-------------------|---------|
| **UR5**   | 0.02–0.23 | **4.3–45×** | collision-bound → pruning pays (matches 2–4× observed) |
| Panda | 0.28–0.99 | 1.0–3.6× | headroom only in sparse scenes, but AABB pruning weak there |
| **Baxter**| 0.95–1.01 | **~1.0×** | **FK-bound → collision tricks can't help** |
| **Fetch** | 0.96–0.99 | **~1.02×** | **FK-bound → must cut FK** (explains its ~1.0× exactly) |

**This is the map of where the win can come from.** Collision-side specialization
(pruning) helps only where collision is a large fraction of the check (UR5-class:
low sphere count, light FK). High-sphere robots (Fetch 111, Baxter 75) spend ~all
their time in `sphere_fk` → **the frontier for them is scene-conditioned FK reduction.**

---

## How the JIT can attack FK-dominance (the highest-ceiling novel lever)

FK looks "robot-only" (AOT) — but only if you compute *all* spheres. Once the scene
is known at query time, most spheres are irrelevant, and *that* is AOT-impossible to
exploit. Four mechanisms, roughly increasing effort:

1. **Scene-pruned FK (skip spheres/links with no reachable obstacle).** Extend the
   broadphase from collision into FK: if sphere j's reachable AABB contains no
   obstacle in this scene, sphere j can never collide → don't compute its position at
   all (unless a downstream sphere needs its transform). cricket already traces FK as
   a CppAD DAG → emit a scene-pruned trace keeping only nodes feeding surviving
   spheres. Localized scenes (a table on one side, a shelf) leave whole limbs
   obstacle-free → large FK cut. **Ceiling ≈ (surviving-sphere fraction).** This is
   the direct answer for Fetch/Baxter.
2. **Scene-ordered sphere checking (faster early-out).** In collision-heavy scenes
   (Fetch dense → frac≈1, early-exit on sphere 0), *which* sphere you check first
   decides cost. Bake an order that evaluates spheres nearest the (known) obstacles
   first → hit the collision sooner → less FK walked. Free within a link; across
   kinematic branches (Fetch torso+arm, Baxter dual-arm) it can skip whole branches.
3. **Fused pruned-FK + pruned-CC straight-line kernel (doc §5 Option B, now justified).**
   Emit one kernel computing only needed transforms and only surviving (sphere,
   obstacle) pairs, in scene order, with early-out — the "compile the scene in"
   idea applied to the *whole* fkcc, which is the version that pays for high-sphere robots.
4. **Kinematic-branch gating.** For multi-limb robots, a scene touching only one arm
   lets the JIT drop the other arm's entire FK+CC subtree per query.

**Hard limit to quantify:** the kinematic chain forces you to compute all *ancestors*
of any surviving sphere, so FK-pruning is bounded by the DAG, not the sphere count.
Measure the "FK-DAG survival factor" per scene class before building the fused kernel.

---

## Recommended next steps (in priority order)

1. **Measure the FK-DAG survival factor per scene class** (cheap analysis, do first). For
   localized/sparse scenes, what fraction of spheres have a non-empty obstacle set, and what
   fraction of the FK DAG must still be computed for them? This is the ceiling for FK-pruning and
   decides whether mechanism #1 above is worth building for Fetch/Baxter. Extends `m25_pruning.cc`.
2. **Validate pruning on real MBM scenes, not synthetic boxes.** The 2–4× UR5 win is on uniform
   random obstacles; confirm on MBM/Robometrics and report the speedup *distribution* + break-even
   query count (E3/E5). Wire per-sphere pruned Environments into the actual `rrtc` solve path
   (via `RobotOps`/`validate` dispatch) for end-to-end numbers. Localized MBM scenes should also
   reveal the FK-pruning headroom the uniform boxes hid.
3. **Prototype scene-pruned FK for Fetch** (the FK-dominance attack). Start with mechanism #1
   (drop obstacle-free spheres + their leaf FK) on a localized scene; measure FK reduction and
   end-to-end speedup on free + mixed workloads. This is the only path to a win for high-sphere robots.
4. **Drop the copy-and-patch/LLVM-min-pipeline plan for now.** Reallocate to (a) a cheap partition
   (spatial hash / precomputed obstacle-AABB / bitset membership → target ≤50 µs), and (b) the
   cost model deciding *when* to specialize (predict M/plan-time; amortize over Q streaming queries).
5. **Revisit query-scoped (start/goal) reachability with the *real* planner-explored set**, not my
   crude interpolate+jitter (which didn't tighten Panda past radial). Tighter per-query AABBs are
   the way to make broad-reach arms (Panda/Baxter) prunable at all.
6. **Tighten reachability with interval-FK** (doc §4 preferred) to replace sampled AABBs and get a
   *provable* conservative bound (removes the fn-rate risk entirely). Extend cricket's CppAD trace.
7. **Reframe the paper thesis** around "*when* does scene specialization pay, and *by what
   mechanism*": collision pruning under amortization for low-sphere robots, **scene-pruned FK** for
   high-sphere robots — not kernel codegen. The const-fold null, the FK/collision-split map, and the
   robot-dependence are themselves the publishable characterization.

---

## Threats / caveats to keep honest
- **Synthetic scenes.** Uniform/shell obstacle boxes, not real clutter. MBM validation pending (#1).
- **Sampled AABBs are not provably conservative.** 0 false negatives observed *for the sampled
  distribution*; a query outside it could miss a collision. Needs interval-FK (#4) or a proven margin.
- **FK-path FP divergence** inflates the split microbench's block-verdict mismatch (not a kernel
  bug — `sphere_mismatch=0`); the real system reuses one FK.
- **`cricket` was not modified** this run — no reachability pass added yet (deferred to #4). Its
  `jit_patch` branch exists per instruction but is unchanged from `constrained`.
- Single CPU (AVX2). Portability (AVX-512/NEON) untested (E7).

## Reproduce
```
micromamba run -n jit_patch bash                # env
# M1: clang++ -std=c++17 -O3 -march=native -fno-strict-aliasing -I src/impl \
#     -isystem $CONDA_PREFIX/include/eigen3 -I <pdqsort-dir> experiments/jit/m1_baseline.cc
micromamba run -n jit_patch bash experiments/jit/m2/run_m2.sh      # M2 sweep
# M2.5: compile+run experiments/jit/m25_pruning.cc and m25_prune_bench.cc (same flags as M1)
python experiments/jit/m0_smoke.py              # M0 load_robot smoke
```
