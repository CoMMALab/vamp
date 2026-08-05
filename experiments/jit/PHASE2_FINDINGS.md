# Phase 2 findings — FK reduction is refuted (rigorous, bit-exact)

Executed autonomously. Goal: cut FK for FK-bound robots (Fetch/Baxter) via the two
levers from `PHASE2_PLAN.md` (2A static hoist, 2B bounding-sphere-gated FK). **Both
fail to help, and now we know exactly why — proven with a correct, bit-exact prototype.**
This retires the expensive cricket-codegen path before building it. Artifacts:
`phase2/{gen_variants.py, stage.py, m7_lock.cc, m8_stage.cc, m9_envstage.cc}`,
`results/m7_lock.csv`.

## F0 — cricket generation unblocked
`cricket.generate_robot_source` works: `template_path` = the main template *file*;
`subtemplates={"ccfk": ...}`; `data` needs `name` + `resolution`; `active_joints`
(list of joint names) locks the rest. Lets us regenerate kernel variants from Python.

## 2A — static-sphere removal: **~1.5% FK**, not a lever
Generated Fetch base (dim 8) vs torso-locked (`active_joints`=7 arm joints, dim 7).
Measured `t_fk`: **260.0 vs 256.2 ns → 1.5%**. The generated `sphere_fk` shows why:
base spheres are **constants**, torso spheres are **`const + x[0]`** (prismatic torso →
linear, no trig). Static/shallow spheres are already cheap; **the FK cost is the
multi-joint arm** (revolute → trig). The P0.1 op-count over-weighted the cheap
base/torso placements. Static removal helps collision-hoisting, not FK.

## 2B — bounding-sphere-gated fine-sphere FK: **refuted (≤1.00×), bit-exact**
Built a real staged kernel: a source-to-source transform (`stage.py`) that SSA-converts
the FK preamble (the generated code reuses scratch vars — `v[6]` is assigned many
times — so identity-based reordering is invalid without SSA) and moves each gate's
exclusive fine-sphere FK inside its bounding-sphere `if`. A/B harness validates
bit-exactness against the unmodified kernel.

Two hard walls, both measured:

1. **Self-collision coupling blocks it entirely.** `fkcc` runs ~2600 self-collision
   pair checks (48 gates) that reuse the same sphere positions under *different* gates.
   So every fine-sphere position must be computed unconditionally: **0% of FK is movable
   into the env-collision gates** (bit-exact, `movable=0`). The env gate can't skip any FK.

2. **Even the ceiling (self-collision co-gated) doesn't pay.** Stripping self-collision
   to an env-only kernel makes **76% of FK movable** — but staging it is **break-even to
   slightly slower** (bit-exact, mm=0):
   | n_obs | collision_frac | env-base | env-staged | speedup |
   |------:|---------------:|---------:|-----------:|--------:|
   | 300 | 0.50 | 3588.7 ns | 3586.3 ns | **1.001** |
   |  80 | 0.31 |  974.3 ns |  981.1 ns | **0.993** |
   |  40 | 0.24 |  582.8 ns |  589.0 ns | **0.990** |

**Why the projection was wrong** (P0 projected 24–53% FK savings → ~1.3–1.6×):
- The op-count counted *movable ops*, but the expensive part (trig chain) is **hoisted**
  anyway — a link's bounding sphere needs the full chain to that link, so only the cheap
  **leaf placements** (mul-adds) are gated. Skipping cheap ops saves ~nothing.
- On a *serial* arm you must compute the chain to the wrist to place the wrist's bounding
  sphere → the chain is never skipped by per-link gating.
- In Fetch-relevant scenes the arm sits near the obstacle cluster, so few link bounding
  spheres actually clear (collision_frac 0.24–0.50) → little is skipped even when movable.
- Scattering the flat, well-vectorized FK into conditional blocks costs branch + ILP/SIMD
  scheduling, cancelling any small saving.

## Verdict & redirect
- **Do not build the 2B cricket codegen.** Per-link bounding-sphere FK gating cannot
  beat the flat vectorized kernel for FK-bound robots. 2A is ~1.5%. Both retired.
- **The only FK lever left is coarser:** *subtree* bounding volumes that skip a whole
  sub-chain (chain + all descendant FK) when a proximal subtree-bound clears — the sole
  way to skip the trig chain on a serial arm. It's a new construct (subtree radii),
  helps only when a proximal subtree is entirely clear of obstacles, and would still face
  the self-collision coupling + vectorization costs. Low expected value; not recommended
  without a specific high-DoF-in-sparse-scene target.
- **Practical conclusion for the paper:** the win is **Lever A (collision pruning) for
  collision-bound robots (UR5-class), under amortization**. FK-bound robots (Fetch/Baxter)
  are genuinely hard — their cost is an efficiently-vectorized FK that resists
  scene-conditioned reduction. This is a clean, defensible scope boundary, and the
  negative result (FK gating doesn't pay) is itself a contribution.

## Method note (reusable)
`stage.py` is a general SSA-based source-to-source staging pass over cricket-generated
kernels, with `STAGE_HOISTALL` (bit-exact sanity) and `STAGE_NOSELF` (env-only ceiling)
modes. The A/B bit-exact harness pattern (`m8`/`m9`) is how any future kernel-restructuring
idea should be validated before codegen investment.
