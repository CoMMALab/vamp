"""Per-query self-collision pair partition (m71), computed from VAMP's OWN collision spheres
(module.fk) so it is consistent with the kernel that consumes it. Returns the list of active
cc_self_pairs indices (pairs that could come close over the query's joint envelope + margin);
pairs proven never-close are dropped. Conservative: a pair is kept unless EVERY one of its fine
sphere-pairs stays > margin apart over the sampled, margin-expanded joint envelope.
"""
import json
import numpy as np

_R2C6_JSON = "/tmp/claude-1000/-home-zak-src-vamp-jit/3dcbbe7c-40b1-4721-b12b-3292533b6ada/scratchpad/r2eval/out_analytic_snap.json"


def load_compact_self(json_path=_R2C6_JSON):
    d = json.load(open(json_path))
    entries = np.array(d["compact_self_entries"], dtype=np.int64)   # [bodyA, bodyB, offset, count]
    pa = np.array(d["compact_self_pair_a"], dtype=np.int64)          # fine sphere index a
    pb = np.array(d["compact_self_pair_b"], dtype=np.int64)          # fine sphere index b
    return entries, pa, pb


def compute_active_self_pairs(module, configs, entries, pa, pb, nq,
                              margin=0.20, env_expand=0.5, n_samples=1500, seed=0, base_ff=True):
    """configs: list of representative query configs (start + goals). Returns active indices."""
    rng = np.random.default_rng(seed)
    C = np.asarray(configs, dtype=np.float64)
    jstart = 7 if base_ff else 0                                     # skip free-flyer base dims
    J = C[:, jstart:]
    jlo, jhi = J.min(0), J.max(0)
    mid = 0.5 * (jlo + jhi)
    half = 0.5 * (jhi - jlo) * (1.0 + env_expand) + 1e-3            # expand envelope for safety

    nfp = len(pa)
    min_gap = np.full(nfp, 1e30)
    # include the endpoints themselves plus random envelope samples
    samples = [J[i] for i in range(len(J))]
    samples += [rng.uniform(mid - half, mid + half) for _ in range(n_samples)]
    for js in samples:
        q = np.zeros(nq, dtype=np.float32)
        if base_ff:
            q[6] = 1.0                                              # identity base (self-collision is base-invariant)
            q[7:] = js
        else:
            q[:] = js
        sph = module.fk(q)
        X = np.array([[s.x, s.y, s.z, s.r] for s in sph])
        A, B = X[pa], X[pb]
        d = np.linalg.norm(A[:, :3] - B[:, :3], axis=1) - A[:, 3] - B[:, 3]
        np.minimum(min_gap, d, out=min_gap)

    # entry i is KEPT (active) if any of its fine pairs can come within `margin`
    off, cnt = entries[:, 2], entries[:, 3]
    active = []
    for i in range(len(entries)):
        seg = min_gap[off[i]:off[i] + cnt[i]]
        if seg.size == 0 or seg.min() <= margin:
            active.append(i)
    return active, min_gap
