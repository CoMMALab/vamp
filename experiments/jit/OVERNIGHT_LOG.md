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
- [ ] M2  T2 prototype: scene-specialized kernel (const-fold + type-prune + unroll), bit-exact correctness, speedup + clang latency
- [ ] M3  T2 via cricket compile() path + LLVM min-pipeline latency

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


