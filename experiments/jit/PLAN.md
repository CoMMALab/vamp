# Phase-2 plan — Cost-aware JIT scene specialization

Builds on the overnight findings (`OVERNIGHT_FINDINGS.md`). The data split the problem
into **two levers**, chosen per robot by the FK/collision cost ratio (`m4_fk_split.csv`):

- **Lever A — collision pruning** (collision-bound robots, UR5-class: FK 2–23% of a check).
  Proven **2.4–4.4×**, correctness-clean, needs *no compiler* (µs–ms data partition). Near-ready.
- **Lever B — FK reduction** (FK-bound robots, Fetch/Baxter: FK 95–100% of a check).
  Higher ceiling, the novel bet, needs cricket codegen. **Centerpiece: bounding-sphere-gated FK.**

## Grounding facts (verified in code this session)
- cricket already computes, per link: a **bounding sphere** (`robot_info.hh` `bounding_spheres`,
  `bounding_sphere_index`) and its fine-sphere list (`per_link_spheres`).
- `ccfk_template.hh` (non-`compact_collisions` mode) already gates fine-sphere **checks**:
  test the link bounding sphere `[[unlikely]]`; only on a hit, test that link's fine spheres.
  Confirmed in the generated kernel (`panda.hh:13890` gates `13898/13908`).
- **BUT** the traced FK computes *all* sphere positions (bounding + fine) up front — the gate
  skips *checks*, not *FK*. Since these robots are FK-bound, the up-front fine-sphere FK is
  exactly the wasted work. **The opportunity: compute fine-sphere FK lazily, inside the gate.**

## Key nuance to respect throughout
The kernel is SIMD (rake=8 configs/lane). A gate skips a link's fine spheres only when **all 8
lanes** clear the bounding sphere. So Lever B pays in **sparse / free-heavy / localized** scenes
(links clear across all lanes) and does ~nothing in dense clutter (frac≈1, nothing clears). That
is also exactly where Lever A's full obstacle loop runs — both levers target the same regime, and
both need realistic *localized* scenes to show up (uniform-box scenes hide them).

---

## Phase 0 — decisive cheap measurements (do first; each is a small standalone bench)

Gate the expensive codegen work behind these.

- **P0.1 FK internal split (chain vs per-sphere placement).** Generate a *bounding-sphere-only*
  FK via cricket (`trace_sphere_cc_fk` with only bounding spheres) and time it vs full `sphere_fk`.
  The difference = the *skippable* per-fine-sphere placement cost = the ceiling for Lever B.
  If fine-sphere placement is a small fraction (chain-FK trig dominates), Lever B is capped — know this before building it.
- **P0.2 Bounding-sphere all-lanes-clear rate.** Per link, per config, fraction of the workload
  where all 8 lanes clear the link bounding sphere (⇒ its fine spheres are skippable). Sweep
  robot × scene class, crucially **localized** scenes (obstacles in one region), not just uniform boxes.
  Expected FK saving ≈ Σ_links (clear_rate_link × fine-sphere-FK_link) / total FK.
- **P0.3 FK-DAG survival factor + reachability.** Combine with Lever-A reachability: fraction of
  links whose bounding-sphere *reachable AABB* is obstacle-free for a scene (skip even the BS check,
  statically). Extends `m25_pruning.cc`.
- **Gate:** build Lever B only if P0.1×P0.2 shows a real FK saving for Fetch/Baxter on localized scenes.

## Phase 1 — Lever A: productionize collision pruning + cost model (parallel to Phase 0/2)

The safe, near-ready result. Target: a real end-to-end win on MBM for UR5-class robots.

- **P1.1 Dispatch integration.** Route `fkcc`/`validate` through `RobotOps` (`jit/dynamic_robot.hh`)
  so a per-query pruned collision path can be swapped in on the real `rrtc` solve.
- **P1.2 Cheap partition (≤50 µs).** Replace the current naive per-sphere `Environment` rebuild
  (42–1588 µs) with a precomputed obstacle AABB-grid / bitset membership; per-sphere lists as
  index views, not copied Environments.
- **P1.3 Cost model.** Predict M (collision checks / plan time) from cheap pre-solve features;
  policy `argmin_t (c_t + M̂·p_t)`; amortize over streaming Q. Regret vs offline oracle.
- **P1.4 MBM / Robometrics validation.** Speedup *distribution* + break-even Q (E3/E5), not just means.
  Localized MBM scenes should also reveal Lever-B headroom the uniform boxes hid.

## Phase 2 — Lever B: bounding-sphere-gated FK (the FK-dominance attack)

The novel, high-ceiling work. This is where cricket earns its place.

- **P2.1 Lazy/staged FK codegen.** Restructure `trace_sphere_cc_fk` + `ccfk_template.hh` to emit
  FK *per link, interleaved with the gate*: compute chain transform + bounding sphere, test it,
  and emit the link's fine-sphere placement **inside** the `if`. Needs a DAG-dependency pass over
  the CppAD trace to know which temporaries feed which fine spheres (so only those move inside).
  This is scene-*independent* and already a win on its own (skips fine-sphere FK for clear links).
- **P2.2 Scene-conditioned gating (the JIT part).** Bake the known scene in: statically drop links
  whose bounding-sphere reachable AABB is obstacle-free (skip even the BS check); per surviving
  link, bake only its reachable obstacle sublist (compose with Lever A).
- **P2.3 Fused pruned-FK + pruned-CC.** One straight-line kernel: only needed transforms, only
  surviving (sphere, obstacle) pairs, scene-ordered (spheres nearest obstacles first for faster
  early-out), with the SIMD all-lanes-clear gate.
- **P2.4 Evaluation.** Fetch/Baxter, free + mixed workloads, **localized** scenes; report FK
  reduction and end-to-end speedup. Correctness: conservative (no missed collisions).

## Phase 3 — reachability rigor + generalization

- **P3.1 Interval/affine-FK reachability** in cricket (`codegen.cc` CppAD trace) → *provably*
  conservative per-sphere/per-link AABBs, removing the sampled-AABB false-negative risk entirely.
- **P3.2 Query-scoped reachability** with the *real* planner-explored set (not interpolate+jitter),
  to make broad-reach arms (Panda/Baxter) prunable and tighten the gates further.
- **P3.3 Portability + optimizing planners.** AVX-512/NEON; FCIT*/AORRTC anytime cost (E6/E7).

---

## Sequencing & gates
```
P0 (cheap, ~1 session) ──► gate ──► P2 (cricket codegen, the bet)
                                     │
P1 (productionize A, independent) ───┘── both feed MBM validation (P1.4/P2.4)
P3 rigor once a lever is proven end-to-end
```
Each phase leaves a standalone figure. Correctness (bit-exact for A, conservative for B) is a
gate on every phase, not an afterthought. Drop the copy-and-patch / LLVM-min-pipeline specializer
unless a later result resurrects a codegen-only win.
