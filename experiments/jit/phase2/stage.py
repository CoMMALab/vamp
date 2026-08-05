#!/usr/bin/env python3
"""2B staging transform (prototype, source-to-source, SSA-based).

Rewrites a cricket-generated fused `fkcc` so each per-link bounding-sphere gate
computes its fine spheres' FK *inside* the gate. The generated code REUSES scratch
vars (v[6] assigned many times), so we first convert the FK preamble to SSA (one
def per temp) — only then is dependency-based reordering valid. Correctness is
checked bit-exact by an A/B harness.

Partition (over SSA temps): HOIST anything a bounding sphere needs, anything the
self-collision block needs (fine-sphere positions are shared there under different
gates), or anything >=2 env gates' fines need. Otherwise move into the single gate.

usage: stage.py <in.hh> <out.hh> <OldStruct> <NewStruct>
env STAGE_HOISTALL=1 -> hoist everything (sanity: must be bit-exact vs base).
"""
import os, re, sys
from functools import lru_cache
from collections import Counter
from pathlib import Path

REF = re.compile(r'([vy])\[\s*(\d+)\s*\]')
ASSIGN = re.compile(r'^\s*([vy])\[\s*(\d+)\s*\]\s*=\s*(.*);\s*$')

def main():
    inp, outp, old, new = sys.argv[1:5]
    lines = Path(inp).read_text().splitlines()
    hoist_all = os.environ.get('STAGE_HOISTALL') == '1'

    fk = next(i for i, l in enumerate(lines) if re.match(r'\s*static inline bool fkcc\(', l))
    hdr = fk - 1 if 'template' in lines[fk - 1] else fk
    b = fk
    while '{' not in lines[b]:
        b += 1
    depth = 0
    for e in range(b, len(lines)):
        depth += lines[e].count('{') - lines[e].count('}')
        if depth == 0:
            break
    body = lines[b + 1:e]

    g0 = next(i for i, l in enumerate(body) if 'sphere_environment_in_collision(environment' in l)
    while g0 > 0 and not re.match(r'\s*if \(sphere_environment_in_collision\(environment', body[g0]):
        g0 -= 1
    pre, gates = body[:g0], body[g0:]

    # --- SSA conversion of the preamble ---
    cur = {}          # ('v'|'y', idx) -> ssa id
    rhs = {}          # ssa id -> rewritten rhs string (refs are t{id})
    deps = {}         # ssa id -> set(ssa ids)
    nid = 0
    def ref_to_ssa(m):
        return f't{cur[(m.group(1), int(m.group(2)))]}'
    for l in pre:
        m = ASSIGN.match(l)
        if not m:
            continue
        kind, idx, expr = m.group(1), int(m.group(2)), m.group(3)
        d = set(cur[(k, int(n))] for k, n in REF.findall(expr))   # deps = current versions
        expr_ssa = REF.sub(ref_to_ssa, expr)
        rhs[nid] = expr_ssa
        deps[nid] = d
        cur[(kind, idx)] = nid
        nid += 1
    final = dict(cur)   # positions gates read = final version of each y index

    @lru_cache(maxsize=None)
    def closure(i):
        seen, st = set(), [i]
        while st:
            u = st.pop()
            for x in deps.get(u, ()):
                if x not in seen:
                    seen.add(x); st.append(x)
        return frozenset(seen | {i})

    # rewrite gate region: y[N] -> t{final y N}
    def y_to_ssa(m):
        return f't{final[("y", int(m.group(2)))]}' if m.group(1) == 'y' else m.group(0)
    gates_ssa = [REF.sub(y_to_ssa, l) for l in gates]

    # STAGE_NOSELF: strip the self-collision block -> a valid env-collision-only
    # kernel, to measure the FK-gating throughput ceiling in isolation.
    if os.environ.get('STAGE_NOSELF') == '1':
        _ss = next((i for i, l in enumerate(gates_ssa) if 'sphere_sphere_self_collision' in l), len(gates_ssa))
        gates_ssa = gates_ssa[:_ss] + ['        return true;']

    # parse env gates (column-0 outer if), record bounding/fine ssa temps + open-brace line
    gate_bound, gate_fine, gate_open = [], [], []
    i = 0
    while i < len(gates_ssa):
        l = gates_ssa[i]
        if re.match(r'if \(sphere_environment_in_collision\(environment', l):  # column 0
            args = []
            j = i + 1
            while len(args) < 4 and j < len(gates_ssa):
                args += re.findall(r't(\d+)', gates_ssa[j]); j += 1
            bound = set(int(a) for a in args[:4])
            k = i
            while '{' not in gates_ssa[k]:
                k += 1
            depth, m2 = 0, k
            for m2 in range(k, len(gates_ssa)):
                depth += gates_ssa[m2].count('{') - gates_ssa[m2].count('}')
                if depth == 0:
                    break
            allt = set(int(a) for a in re.findall(r't(\d+)', '\n'.join(gates_ssa[i:m2 + 1])))
            gate_bound.append(bound); gate_fine.append(allt - bound); gate_open.append(k)
            i = m2 + 1
        else:
            i += 1

    # self-collision block: everything from first self call to end -> force hoist
    ss = next((i for i, l in enumerate(gates_ssa) if 'sphere_sphere_self_collision' in l), len(gates_ssa))
    self_t = set(int(a) for a in re.findall(r't(\d+)', '\n'.join(gates_ssa[ss:])))
    if os.environ.get('STAGE_NOSELF') == '1':   # ceiling: pretend self-collision is co-gated
        self_t = set()

    need = set()
    for s in [*[v for bs in gate_bound for v in bs], *self_t]:
        need |= closure(s)
    cnt = Counter()
    for fs in gate_fine:
        c = set()
        for v in fs:
            c |= closure(v)
        for v in c:
            cnt[v] += 1
    fine_cl = []
    for fs in gate_fine:
        c = set()
        for v in fs:
            c |= closure(v)
        fine_cl.append(c)

    def hoisted(i):
        return hoist_all or (i in need) or (cnt[i] >= 2)

    hoist_ids = [i for i in range(nid) if hoisted(i)]
    local = {}
    for gi, c in enumerate(fine_cl):
        local[gi] = [i for i in sorted(c) if not hoisted(i)]
    placed = set(hoist_ids) | {i for g in local.values() for i in g}
    assert placed == set(range(nid)), f"unplaced: {sorted(set(range(nid)) - placed)[:5]}"

    # inject local temps into gates (back to front)
    out_g = list(gates_ssa)
    for gi in range(len(gate_open) - 1, -1, -1):
        inj = [f'        auto t{i} = {rhs[i]};' for i in local[gi]]
        out_g[gate_open[gi] + 1:gate_open[gi] + 1] = inj

    hoist_lines = [f'    auto t{i} = {rhs[i]};' for i in hoist_ids]
    new_body = ['        static_assert(stride >= dimension);', ''] + hoist_lines + [''] + out_g
    new_fn = lines[hdr:b + 1] + new_body + [lines[e]]
    out = lines[:hdr] + new_fn + lines[e + 1:]
    txt = '\n'.join(out) + '\n'
    txt = txt.replace(f'struct {old}', f'struct {new}').replace(f'"{old.lower()}"', f'"{new.lower()}"')
    Path(outp).write_text(txt)
    nl = sum(len(v) for v in local.values())
    print(f"gates={len(gate_bound)} ssa_temps={nid} hoisted={len(hoist_ids)} "
          f"movable={nl} ({100*nl/max(1,nid):.0f}%)")

if __name__ == '__main__':
    main()
