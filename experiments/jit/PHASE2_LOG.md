# Phase 2 execution log — FK reduction (static hoist + gated FK)

Branches: `jit_patch` (vamp + cricket). Env: `jit_patch`. Autonomous; user validates gates.
Plan: `PHASE2_PLAN.md`. Order 2A → 2B → 2C.

Key refinement (pre-code): fully-static spheres (Fetch base, 20) are likely already
constant-folded by CppADCodeGen, so their FK is ~free already. 2A's FK win comes from
(a) **locking joints** (torso) → folds the 36 torso-only spheres [use cricket active_joints],
and (b) **hoisting static-sphere collision** out of the per-config loop. Verify both.

## Status
- [x] F0  unblock `cricket.generate_robot_source` (template_path = the .hh file; data needs name+resolution)
- [x] 2A-i  joint-locking FK win: **1.5% only** — base/torso already cheap → 2A is NOT an FK lever
- [~] 2A-ii static-collision hoist — deprioritized (helps collision-bound, not FK-bound robots)
- [x] 2B  staged/gated FK: REFUTED (0% movable w/ self-collision; env-only ceiling 76% movable but <=1.00x, bit-exact)
- [~] 2C  not pursued — FK lever refuted; see PHASE2_FINDINGS.md

## Timeline

### F0 + 2A-i (done)
- `generate_robot_source` works: `template_path` = the main template *file* (not dir);
  `subtemplates={"ccfk": ccfk_template.hh}`; `data` needs `name` + `resolution`.
  (`scripts` scratch: gen_variants.py.)
- Generated Fetch base (dim 8) vs torso-locked (`active_joints`=7 arm joints, dim 7).
  Inspecting generated `sphere_fk`: base spheres are **constants**, torso spheres are
  **`const + x[0]`** (prismatic torso → linear, no trig). So they're already near-free.
- **Measured `t_fk`: base 260.0 ns vs locked 256.2 ns → 1.5% saving.** Confirms:
  static/shallow spheres are cheap; **the FK cost is the multi-joint arm** (revolute →
  trig chains). 2A static-removal is not the FK lever for FK-bound robots — 2B is.
- Correction to the earlier op-count (`PHASE0` P0.1): placement op-count over-weighted
  the cheap base/torso `const/1-add` placements. The *cost*-weighted FK is arm-dominated.

### 2B (in progress) — stage the ccfk so dynamic-link fine-sphere FK moves inside the gate
