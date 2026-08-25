# Overnight: wire the 3 collision partitions into VAMP RRTC, verify correctness, evaluate

Task (user, going to sleep): build each of the 3 partitions in VAMP proper, evaluate inside RRTC
end-to-end in the relevant experimental env, VERIFY CORRECTNESS is maintained, report in the morning.

The 3 partitions (from this session's prototypes):
1. per-query self-pair partition (m71): prune never-close self-collision pairs for the query.
2. per-edge env partition (m67): base-scoped reachability prune of obstacles per edge.
3. FK-DAG materialization gate (m68/m69): conditionally materialize fine spheres via bounding gate.

Correctness bar: 0 false negatives (never miss a real collision); planning success + paths preserved.

## Progress log

### Architecture found
- fkcc(environment, x): computes ALL sphere positions y[] (FK, up front -> 2B materialization blocker
  confirmed: cannot gate FK placement without SSA reorder), then loops cc_env_links (env, per-link
  bounding-gated) and cc_self_pairs (self, per-link-pair bounding-gated). So VAMP ALREADY has the m69
  per-config bounding gates. The NEW win = prune never-close pairs/obstacles per query so the loops
  iterate fewer (m71/m67).
- cc_env_links/cc_self_pairs are static constexpr arrays. Env obstacles live in the runtime
  Environment (passed per-call). sphere_environment_in_collision already has a sorted radial early-exit.
- Integration points: (a) env obstacle pruning = pass a pruned Environment (no kernel change, safest);
  (b) self-pair pruning = kernel must iterate a runtime subset (needs template/header change);
  (c) materialization = 2B-blocked (FK computed up front).

### Plan (correctness-first, honest scope)
1. Env partition via pruned Environment (no regen) -- test end-to-end on panda/ur5 MBM, verify same
   paths (correctness) + timing. [SAFEST, START HERE]
2. Self-pair partition: edit generated kernel to iterate a runtime-pruned pair list (r2c6/digit),
   verify 0 FN + same planning, measure.
3. Materialization: assess (likely 2B-blocked); report honestly.
4. Combined eval on built robots (ur5,panda,digit,r2c6); build fetch/baxter if time.

### Partition 1 (self-pair) — INTEGRATED + VERIFIED
- environment.hh: added active_self_pairs field (+copy ctor). binding: def_rw. template: compact
  self-loop iterates the pruned subset. Regenerated digit+r2c6, built. Identity (empty)==baseline OK.
- selfpair_partition.py computes active indices from module.fk (VAMP's own spheres) over the query
  joint envelope + margin. Wired into plan_step (R2_PRUNE), verification via R2_VERIFY (re-validate
  path under full self-collision).
- CORRECTNESS: every step "path valid under full self-collision", 0 FN. Pruned **97-98%** of 1208
  self-pairs per query (tight handrail joint envelope).
- CAVEAT: partition overhead ~130ms (Python, 400 samples x fk) >> solve 1-2ms. r2c6 is
  projection-bound (collision ~20%, self a fraction) so it can't pay here regardless. Measuring the
  pure solve-time effect (partition excluded from timer) next.

### Partition 1 (self-pair) — CORRECTNESS + PERF characterized
- CORRECTNESS: sampled min-distance is NOT provably conservative. margin 0.03 -> 2 FN/8000; margin
  0.15+1500 samples -> 0 FN (93% pruned); margin 0.20 -> 91% pruned, 0 FN. So correct-ABLE with a
  15-20cm margin (empirical; provable needs interval-FK). Default set to 0.20/1500.
- PERF (correct margin, 0 FN, no search divergence): baseline 2.01 vs pruned 2.12 ms/step -> ~5%
  SLOWER. r2c6 check is FK-DOMINATED (1116-value FK >> self-collision minority), so pruning self-pairs
  saves ~nothing and the runtime index-loop adds slight overhead. NO win for a compact humanoid.
- ARCHITECTURAL MISMATCH: runtime pruning only works on COMPACT kernels (digit/r2c6 = FK-dominated,
  no benefit). The robots where self-collision is large (sparse-scene arms, m10 ~40%) use UNROLLED
  kernels -> cannot runtime-prune. So the partition helps exactly where it can't be applied.

### Now testing Partition 2 (env) on the COLLISION-BOUND case (ur5) where a win is possible.

### Partition 2 (env) — tested end-to-end on ur5 MBM (7 scene sets, 25 problems each)
- Mechanism: reachability-pruned Environment (rebuild env with obstacles outside the robot's
  query swept-AABB removed). No kernel change.
- CORRECTNESS: 0 false negatives across all scenes (pruned path re-validated under FULL env);
  solved counts identical baseline vs pruned.
- Prune: only 0-18% obstacles removed (MBM obstacles sit inside the workspace). Speedup ~1.0x on the
  one realistic solve (cage 23ms -> 1.03x); the 1.2-1.3x on the sub-2ms scenes is timing noise.
  Confirms overnight: global env prune is ~null; the 2-4x win needs PER-SPHERE pruning in the kernel
  + a dense-obstacle throughput regime (not MBM planning).

### Partition 3 (materialization) — 2B-BLOCKED
- fkcc computes ALL 1116 sphere positions up front (flat SSA), then checks. Gating FK placement
  needs SSA reordering (refuted PHASE2 2B). VAMP already gates the CHECKS behind bounding spheres.
  Not integrable without a codegen rewrite; not attempted.

### Full-suite evaluation (m74) — compact / unrolled / compact+pruned
Kernel throughput (validate = FK+self, empty env), compact + compact+pruned, all 6 robots (0 FN over
the sample envelope): baxter 1.69x, r2c6 1.63x, ur5 1.11x, fetch 1.11x, panda 0.99x, digit 1.00x.
Helps robots with many separable self-pairs (baxter dual-arm, r2c6); null for few-pair (panda) or
interwoven (digit 8% prunable).

CRITICAL CORRECTNESS: end-to-end MBM (ur5/panda RRTC): ~neutral solve (0.94-0.98x, collision is a
minority) AND **ur5 got 5 FALSE NEGATIVES / 104 problems**. The per-query partition is UNSAFE for an
unconstrained global-sampling planner (RRTC) -- the planner checks configs far outside the start-goal
envelope, where pruned pairs collide. The throughput test's 0 FN was an artifact of testing over the
same envelope. Safe only for CONSTRAINED planners that stay near the manifold (r2c6/digit handrail:
0 FN observed on paths) -- and even there not provable without interval-FK.
