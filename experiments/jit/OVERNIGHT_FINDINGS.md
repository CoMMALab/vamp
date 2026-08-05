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
| **UR5**   | 40  | 1.5–2.3× | **2.4–4.3×** | clean win |
| Fetch | 111 | 2.8–4.8× | ~1.0–1.14× | pruning masked by FK-dominance |
| Panda | 59  | 0.53–1.05× | 0.99–1.31× | broad reach → AABB≈radial |
| Baxter| 75  | 0.61–1.12× | (not benched) | broad reach |

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

## Recommended next steps (in priority order)

1. **Validate pruning on real MBM scenes, not synthetic boxes.** The 2–4× UR5 win is on uniform
   random obstacles; confirm on MBM/Robometrics and report the speedup *distribution* + break-even
   query count (E3/E5). Wire per-sphere pruned Environments into the actual `rrtc` solve path
   (via `RobotOps`/`validate` dispatch) for end-to-end numbers.
2. **Drop the copy-and-patch/LLVM-min-pipeline plan for now.** Reallocate to (a) a cheap partition
   (spatial hash / precomputed obstacle-AABB / bitset membership → target ≤50 µs), and (b) the
   cost model deciding *when* to partition (predict M/plan-time; amortize over Q streaming queries).
3. **Attack FK-dominance to unlock Fetch/Baxter.** Their big pruning factor is wasted because
   `sphere_fk` computes all spheres. A reachability pass could also prune *which spheres* need FK
   per query, or fuse pruned-FK+CC. This is where high-sphere robots would finally win.
4. **Tighten reachability with interval-FK** (doc §4 preferred) to replace sampled AABBs and get a
   *provable* conservative bound (removes the fn-rate risk entirely). Extend cricket's CppAD trace.
5. **Reframe the paper thesis** around "*when* does scene specialization pay, and *by what
   mechanism*" — the answer is broadphase pruning under amortization, not kernel codegen. The
   const-fold null and the robot-dependence are themselves publishable characterizations.

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
