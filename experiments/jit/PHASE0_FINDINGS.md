# Phase 0 findings — bounding-sphere-gated FK go/no-go

Cheap, decisive measurements gating the Lever-B codegen work (`PLAN.md`). All on
`jit_patch`, env `jit_patch`. Data: `results/m5_bs_gate.csv`; op-counts below.

## The idea (recap)
The generated `fkcc` already gates fine-sphere **checks** behind a per-link bounding
sphere (`panda.hh:13890`), but computes **all** sphere FK up front. Moving fine-sphere
FK *inside* the gate lets a link whose bounding sphere clears (all 8 SIMD lanes) skip
its fine spheres' FK. For FK-bound robots (Fetch/Baxter, FK = 95–100% of a check) this
is the only available lever.

## P0.1 — how much FK is skippable? (per-sphere placement vs shared chain)
Op-count of generated `sphere_fk`: shared chain (`v[]`, trig+joint transforms, NOT
skippable) vs per-sphere placement (`y[]`, skippable if the link clears).

| robot | chain ops | placement ops | trig | **placement_share** |
|-------|-----------|---------------|------|---------------------|
| UR5   | 65  | 160 | 12 | 0.71 |
| Panda | 95  | 236 | 14 | 0.71 |
| Baxter| 216 | 300 | 28 | 0.58 |
| **Fetch** | 65 | 444 | 14 | **0.87** |

Fetch's FK is 87% per-sphere placement (111 spheres, short chain) — almost all of its
FK is skippable in principle. (Op-count proxy; a bounding-only kernel timing would
sharpen it, but the split is clear.)

## P0.2 — how often does a link clear? (fraction of fine-sphere FK a gate skips)
Per-link enclosing bounding sphere, all-8-SIMD-lanes clear-rate. Two workloads:
`random` (8 independent configs/rake — worst case) and `motion` (8 nearby configs along
a segment — **how VAMP actually fills a rake**). Two scenes: `uniform` and `localized`
(a table/shelf cluster). Link recovery by joint-dependency signature (robust): UR5 7,
Panda 8, Baxter 15, Fetch 9 links — matches kinematics.

**`fine_fk_skip_frac`:**
| robot | uniform+random | uniform+motion | localized+random | **localized+motion** |
|-------|----------------|----------------|------------------|----------------------|
| UR5   | 0.14 | 0.52 | 0.39 | **0.75** |
| Panda | 0.00 | 0.08 | 0.19 | **0.61** |
| Baxter| 0.01 | 0.25 | 0.23 | **0.62** |
| Fetch | 0.03 | 0.22 | 0.06 | **0.27** |

Correlated motion rakes clear **3–10× more often** than random configs — the realistic
workload is far more favorable. (These are still *lower bounds*: the bounding sphere is
a loose enclosing sphere; cricket's tighter/hierarchical one clears more.)

## Combined projection & GATE DECISION — **GO** (targeted)
FK saving ≈ placement_share × skip_frac; throughput gain for FK-bound robots ≈
1/(1 − FK_saving·FK_fraction) with FK_fraction from `m4_fk_split.csv`.

| robot | FK saving (localized+motion) | FK fraction of check | **projected throughput** | verdict |
|-------|------------------------------|----------------------|--------------------------|---------|
| **Baxter** | 0.58×0.62 = **0.36** | ~1.0 | **~1.56×** | FK-bound → build it |
| **Fetch**  | 0.87×0.27 = **0.24** | ~0.98 | **~1.31×** | FK-bound → build it |
| Panda | 0.71×0.61 = 0.43 | 0.3–1.0 | ~1.2–1.4× (sparse) | worth it in clutter |
| UR5   | 0.71×0.75 = 0.53 | 0.02–0.23 | ~1.0× | collision-bound → use Lever A |

**Decision:** build Lever B (bounding-sphere-gated FK), targeted at **FK-bound /
high-sphere robots (Baxter, Fetch)** and clutter regimes, on realistic motion-rake
workloads. It's the *only* lever for those robots (collision pruning can't touch them),
and it composes with Lever A. Projected ~1.3–1.6× — meaningful, not transformative.

The **key remaining uncertainty is not the FK accounting but whether it translates to
throughput** once the SIMD gate branch is in the hot path — that requires the real
gated kernel (Phase 2 P2.1), not more projection.

## Phase-2 implementation spec (concrete, grounded)
The change lives in **cricket** (this is why cricket exists):
1. **`cricket/src/codegen.cc` `trace_sphere_cc_fk`** already supports a bounding-only
   trace (`spheres=false, bounding=true`). The FK trace is one CppAD DAG computing all
   outputs. Add a **per-link staging pass**: partition the emitted straight-line code so
   each link's fine-sphere placement (`y[]` feeding that link's spheres) is a separate
   block, guarded by the link's bounding-sphere check. Needs a dependency walk over the
   trace (which temporaries feed which sphere outputs).
2. **`cricket/resources/templates/ccfk_template.hh`** — the per-link section (currently
   `bounding-check { fine-checks }`) becomes `bounding-check { fine-FK; fine-checks }`,
   i.e. move the link's placement code inside the existing `[[unlikely]]` gate. No new
   branch — reuse the gate that's already there.
3. **Correctness:** the gated kernel must be bit-identical to the flat one on colliding
   configs and conservative on clear ones (a link that clears its bounding sphere cannot
   collide a fine sphere — true iff the bounding sphere encloses all fine spheres, which
   cricket guarantees). Validate over millions of configs.
4. **Measure** on Baxter/Fetch, motion rakes, localized scenes — confirm the ~1.3–1.6×.

Not started this session: this is a core-cricket codegen change affecting every robot
and needs careful correctness validation, so it wants a supervised checkpoint rather
than an unsupervised overnight edit. Ready to execute on go-ahead.

## Caveats
- `fine_fk_skip_frac` uses a loose enclosing bounding sphere + lower-bounds the real gate.
- Motion-rake step size (~0.02 in [0,1] space) is a stand-in for VAMP's resolution step;
  the trend (correlated ≫ random) is robust, the exact fraction will shift with tuning.
- Projection assumes placement cost ∝ op-count and zero added branch cost (the gate
  branch already exists for checks); the real kernel may differ — hence P2.1 measurement.
