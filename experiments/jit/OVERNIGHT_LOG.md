# Overnight JIT-specialization run — log

Branches: `jit_patch` in both vamp and cricket (off `constrained`).
Env: micromamba `jit_patch`.
Host: i9-14900K (AVX2, no AVX-512), system clang 22.
Scope approved: **push through M3** (env+build → T0 baselines → T2 scene-specialized kernel w/ bit-exact correctness → T2 via cricket LLVM min-pipeline). **Halt + report** on any unresolvable blocker.

Goal = the design doc's decision gate (§13 actions 1–3, §10 milestone 1): prove scene specialization helps, with real numbers, before building broadphase / copy-and-patch / cost-model.

---

## Status

- [x] M0  env `jit_patch` created + cricket & vamp built (JIT ON) + load_robot Panda/UR5 smoke
- [x] M1  T0 baselines: generic fkcc throughput sweep + clang load_robot latency
- [x] M2  T2 prototype: scene-specialized kernel (const-fold + type-prune + unroll), bit-exact correctness, speedup + clang latency
- [x] M2.5 broadphase pruning (added): the AOT-impossible lever, vs VAMP's existing early-exit
- [x] M3  scene-specialization compile latency + gate decision on fast specializer

## Timeline

### M0 — env + build + smoke  ✅
- micromamba env `jit_patch`: clang/LLVM 22.1.8, python 3.14, numpy 2.5.1, pinocchio 3.9, cgal, cppad, nanobind. (`results/m0_env_create.log`)
- cricket: `pip install --no-build-isolation .` → exit 0, `import cricket` OK. (`results/m0_cricket_build.log`)
- vamp: `VAMP_BUILD_JIT=ON`, AOT robot modules trimmed to `panda;ur5`, `-j12` (RAM). exit 0. (`results/m0_vamp_build.log`)
- Smoke (`m0_smoke.py` → `results/m0_smoke.json`):
  - **Panda** `load_robot` **8.81 s**, solved **10/10** MBM, median plan **40.1 µs**.
  - **UR5** `load_robot` **5.74 s** (compile OK; no unpacked problems, solve skipped).
- Baseline established: full-robot (FK+CC) JIT compile via cricket→clang→ORC ≈ **5.7–8.8 s** = the "clang" bar. Planning median ~40 µs confirms the doc's microsecond premise.

### M1 — T0 generic-kernel throughput  ✅
`m1_baseline.cc` (header-only, no JIT): generic `Robot::fkcc<8>` over 200k random
config blocks, sweeping N obstacles × extent. (`results/m1_baseline.csv`)
- **T0 throughput** (median): Panda ~1.1–2.8 M fkcc/s (9–22 M configs/s); UR5
  0.24–3.5 M fkcc/s. UR5 falls off steeply with N (318 ns@10 → 4.1 µs@800 obs);
  Panda stays flatter because collision_frac≈1 makes the sorted early-exit fire early.
- **Key caveat for M2:** dense scenes (extent 0.6) → collision_frac≈1.0, so the
  early-exit dominates and the *full* obstacle loop rarely runs. Scene-specialization
  wins most on **collision-free** queries (whole loop executes). M2 therefore adds a
  free-config workload alongside the mixed one. The sparse Panda rows (extent 1.5,
  N≤50 → frac 0.55–0.62) already probe that regime.
- Robot set extended per user request to **UR5(40), Panda(59), Baxter(75, dim14),
  Fetch(111, dim8)** — headers are AOT, no rebuild. High-sphere robots (Baxter,
  Fetch) throughput falls off with N as expected (more per-sphere collision work).
- **Consequence:** Baxter/Fetch hit collision_frac≈1.0 nearly everywhere (many
  spheres ⇒ almost every random config collides), so free configs are too rare to
  sample. M2 therefore uses an **isolated kernel microbench** (random query spheres,
  full-loop controllable) as the clean Component-1 speedup + bit-exact gate, plus a
  robot FK+CC bench where n_spheres scales the win.

### M2 — scene-specialized kernel (const-fold + type-prune + unroll)  ✅
Levels 1–2 of Component 1: obstacles baked as constexpr immediates + precomputed
min_distance early-exit constants, `#pragma unroll(full)`, spheres-only (type-pruned).
Body is structurally identical to the generic spheres loop → **bit-exact by
construction**. Files: `m2/{specialized_kernel.hh,m2_gen.cc,m2_kernel_bench.cc,
m2_robot_bench.cc,m2_kernel_only.cc,run_m2.sh}`. Data: `results/m2_{kernel,robot,spec_latency}.csv`.

**Correctness (the must-have): 0 mismatches** across every isolated-kernel query
(5×10^5 × 16 scenes × 2 dists) and every robot sphere (10^5 blocks × 4 robots × 6
scenes × up-to-111 spheres) — the specialized kernel is bit-identical to generic.

**Throughput — the pivotal (negative) finding:**
- Isolated kernel (clean): **0.98–1.26× generic**. Best (1.26×) only at N=10 where
  loop overhead is relatively large; → 1.0× as N grows and per-obstacle SIMD
  arithmetic dominates. At N=400 "full" it's 0.985× (unroll bloat).
- Robot FK+CC (`speedup_split`, FK held identical): **0.85–1.16×**, same story.
- **Const-folding obstacles is essentially break-even.** VAMP's generic SIMD
  collision loop is already near-optimal; the bottleneck is per-check FLOPs, not
  obstacle-data loads, so baking the *same set* of checks as immediates buys nothing.
  The win must come from checking *fewer* obstacles → **broadphase pruning** (M2.5).

**Compile latency (clang -c, scene granularity):** 0.64 s (N=10) → 0.83 s (N=800)
→ 1.09 s (N=1600). Mostly fixed header-parse overhead; weak N-scaling. Far under the
~5–9 s full-robot compile, but ~100–1000× over a copy-and-patch target — only worth
optimizing *if* throughput justifies specialization (it doesn't yet; see M2.5/M3).

**Correctness caveat (not a bug):** `verdict_mismatch` (fused `fkcc` vs `sphere_fk`
reconstruction) is nonzero for UR5/Panda in free-heavy scenes — pure FK-path FP
divergence at collision boundaries, amplified by the union over N spheres
(1−(1−0.5%)^59 ≈ 26%). `sphere_mismatch=0` proves the *collision kernel* is exact;
the real system reuses one FK so this vanishes. `speedup_vs_fused` is confounded by
this FK-strategy difference and is NOT reported as a specialization win.

### M2.5 — broadphase pruning (the AOT-impossible lever)  ✅
Per robot sphere j, a sampled-and-inflated reachable **AABB** (over the joint box —
robot-intrinsic, offline) prunes obstacles that sphere can never touch. Files:
`m25_pruning.cc` (factor analysis, no codegen), `m25_prune_bench.cc` (throughput +
correctness). Data: `results/m25_{pruning,prune_bench}.csv`.

**Pruning factor vs VAMP's *existing* radial early-exit** (`aabb_beats_radial` =
obstacles left by radial ÷ by AABB; >1 ⇒ AABB wins):
- **Fetch (111 sph): 2.8–4.8×** — tight per-sphere AABBs; the standout.
- **UR5 (40): 1.5–2.3×.**  **Panda/Baxter: 0.53–1.12×** — broad-reach arms, AABB≈radial
  (confirms doc §4 caveat exactly). Start/goal *corridor* AABBs did NOT tighten Panda
  past radial (still <1) at jitter 0.25 rad.

**Throughput (per-sphere pruned Environment + VAMP's own kernel), correctness-verified:**
- **fn_rate = 0.000000** everywhere (100k blocks × ≤111 spheres) — sampled AABBs +5 cm
  margin are empirically conservative (no missed collisions).
- **UR5: 2.4–4.3× speedup** (N=800/0.6: 6411→1477 ns). Real, AOT-impossible.
- **Panda: 0.99–1.31×** (no win, as predicted). **Fetch: 0.98–1.14×** despite keeping
  only 18% of obstacles — Fetch is **FK-dominated** (111-sphere `sphere_fk`) and dense
  scenes early-exit on sphere 0, so there's little collision work left to prune.
- ⇒ **pruning factor does NOT directly predict speedup**: FK cost and the existing
  early-exit gate it. The win is real but conditional (robot geometry, scene density,
  FK/collision balance).

### M3 — specialization latency + fast-specializer gate decision  ✅
Latency picture (host i9-14900K):
| specialization | cost | delivers throughput win? |
|---|---|---|
| full-robot JIT (clang→ORC, M0) | 5.7–8.8 s | n/a (baseline machinery) |
| scene const-fold kernel, clang -c (M2) | 0.64–1.09 s | **no (~1×)** |
| **broadphase partition (M2.5)** | **42–1588 µs** | **yes (UR5 2–4×)** |

**Gate decision (design doc §13 action 4 — "only if T2 shows benefit, invest in the
fast specializer"): DO NOT build the copy-and-patch / LLVM-min-pipeline specializer
now.** The lever that works (pruning) is a **data-structure partition (µs–ms), not
codegen** — it needs no compiler, so the entire Component-2 (fast specializer) effort
is off the critical path. The lever that would need codegen (const-fold) doesn't pay.
The LLVM-min-pipeline latency measurement is therefore deferred as not
decision-relevant — building it would chase a number the results say not to act on.

**Amortization caveat (feeds the cost model, doc §6/E5):** even the µs–ms partition
exceeds UR5's ~40–100 µs median plan time, so single easy queries don't pay. Pruning
amortizes over **streaming/workcell** (scene reused across queries) and **long single
runs** (hard problems, optimizing planners) — exactly the regimes the doc targets.

---

## Final state
All four milestones + the added M2.5 complete; **no blockers hit** (halt-and-report was
never triggered). Synthesis + recommendations in `OVERNIGHT_FINDINGS.md`. Headline: the
winning lever is **broadphase pruning (data partition, no codegen)** with a **UR5 2–4×**
correctness-verified speedup; **const-fold codegen is a dead end (~1×)**, so the
copy-and-patch specializer is off the critical path. `cricket` unchanged (reachability
pass deferred — see its `jit_patch` note).


