# Phase 2 sketch — FK reduction for FK-bound robots

Goal: cut FK for FK-bound / high-sphere robots (Fetch 111 sph, Baxter 75), which
collision-side specialization cannot help (FK = 95–100% of a check, `m4_fk_split.csv`).
Three composable mechanisms, ordered by certainty (do 2A first — simplest, branch-free,
biggest measured headroom). All are **cricket codegen** changes; user validates each gate.

Grounding numbers already in hand:
- Placement is 58–87% of FK (`PHASE0_FINDINGS`, P0.1); Fetch 0.87.
- Static / shallow spheres (`m6_static.csv`): Fetch 20 static (18%) + 36 torso-only →
  up to 50% hoistable; Baxter 9 (12%).
- Dynamic bounding-sphere gate skip-frac on realistic motion rakes (`m5_bs_gate.csv`):
  Fetch 0.27, Baxter 0.62, Panda 0.61.

---

## 2A — Static-sphere hoisting  (do first: branch-free, certain)
A sphere whose position is config-independent (base link; and any link whose only
ancestor joints are *inactive/locked* for this query) has **constant FK** and
**constant scene-collision**.
- **FK:** bake constant positions — delete their per-config placement from the kernel.
- **Collision:** check the static spheres against the scene **once per query**, not per
  config. None collide (typical) → drop from every per-config check. One collides → the
  pose is infeasible regardless of config → reject up front.
- **Query specialization (the JIT part):** locking joints (e.g. Fetch torso) makes their
  downstream spheres static too. cricket already models this — recipe `active_joints` +
  `JointSelection::from_json` (`codegen.cc`), and each sphere carries `parent_joint`
  (`trace_sphere`), so the static set is derivable **structurally** (no sampling; my
  measurement sampled only to quantify).
- **Expected:** Fetch −18% spheres (base) unconditionally, up to −50% (torso locked);
  Baxter −12%. Removes both their FK placement and their per-config collision.
- **Validate:** bit-exact vs flat kernel; the once-per-query static check must not drop a
  real collision (it's the same check, hoisted — prove equivalence).

## 2B — Bounding-sphere-gated dynamic FK  (the conditional win, on remaining spheres)
For the *dynamic* links, stage FK per link and move fine-sphere placement **inside the
existing bounding-sphere gate** (`ccfk_template.hh` already has the `[[unlikely]]` gate
for checks — reuse it, no new branch). A link whose bounding sphere clears all 8 lanes
skips its fine spheres' FK.
- **Expected:** −27% of the *dynamic* fine-sphere FK for Fetch on motion rakes, −62%
  Baxter. Composes on top of 2A (applies to what 2A didn't already remove).
- **Validate:** bit-exact on colliding configs, conservative on clear ones (bounding
  sphere encloses fine spheres — cricket guarantees); **measure throughput** (the branch
  cost is the one thing projection can't settle).

## 2C — Composition + scene-conditioning
- Compose 2A+2B with **Lever A** (per-sphere obstacle pruning): static spheres checked
  once against the pruned scene; dynamic spheres gated + pruned per link.
- **Order** dynamic link checks by proximity to the (known) obstacles → faster early-out
  in collision-heavy scenes.
- Optionally drop static spheres proven obstacle-free by reachability (never checked).

---

## Implementation map (cricket)
| step | file / entity | change |
|------|---------------|--------|
| static partition | `robot_info` (`per_link_spheres`, `parent_joint`, `active_joints`/`JointSelection`) | classify each sphere static/dynamic for the active-joint set (structural) |
| staged FK trace | `codegen.cc` `trace_sphere_cc_fk` | emit static-sphere placements as constants; group dynamic placements per link with dependency info |
| gated template | `resources/templates/ccfk_template.hh` | static block (once) + per-link `bounding-check { fine-FK; fine-checks }` |
| expose for bench | Python `generate_robot_source` + a variant flag | build gated vs flat kernels for A/B timing |

## Evaluation
- **E-FK-1 correctness (gate on all):** gated/hoisted kernel bit-identical to flat over
  1e7+ random configs × scenes; static-once == per-config for static spheres.
- **E-FK-2 FK reduction:** op-level + measured, per robot × active-joint config × scene.
- **E-FK-3 throughput:** flat vs 2A vs 2A+2B vs +LeverA, on Fetch/Baxter, **motion rakes**,
  localized + MBM scenes. Target: Fetch/Baxter ~1.5–2× (2A+2B combined, torso-locked).
- **E-FK-4 cost model:** 2A partition is ~free (structural, per-query if joints lock);
  2B/LeverA carry per-query cost — fold into the when-to-specialize policy.

## Risks / checkpoints (validate with user)
1. **Static-collision equivalence** — hoisting must be provably the same check, once.
2. **Gate branch cost** — measure; only pays when clear-rate is high (motion rakes: it is).
3. **active-joint specialization** — locked-joint kernels change the kernel's effective
   DoF; ensure the dispatch (`RobotOps`) and config mapping stay consistent.
4. **Interaction with self-collision** — Phase 0/2 studied env-collision FK; self-collision
   spheres follow the same gate but check pairs — confirm no regression.

## Suggested order
2A (measure Fetch/Baxter FK cut + correctness) → 2B (add gate, measure throughput) →
2C (compose with Lever A + MBM). Each is a standalone figure and a validation gate.
