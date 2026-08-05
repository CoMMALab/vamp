# Per-edge broadphase: for an edge (start A, vector v, |v|<=range), each robot sphere
# sweeps a bounded region. Compute its swept AABB once, prune obstacles to it, then all
# sub-configs check only those. Fully general (any A,v; nothing baked), provably
# conservative. Compare obstacle-CHECKS/edge vs the current per-config radial early-exit.
import cricket, numpy as np, pinocchio as pin
res=cricket.resources_dir()
ROBOTS={"panda":("panda/panda_spherized.urdf",1.25),"ur5":("ur5/ur5_spherized.urdf",1.5),
        "fetch":("fetch/fetch_spherized.urdf",1.0)}
rng=np.random.default_rng(3)
def sphere_positions(m,d,gm,gd,q):
    pin.forwardKinematics(m,d,q); pin.updateGeometryPlacements(m,d,gm,gd)
    return np.array([np.array(go.translation) for go in gd.oMg])
for name,(urdf,rng_len) in ROBOTS.items():
    m=pin.buildModelFromUrdf(str(res/urdf)); d=m.createData()
    gm=pin.buildGeomFromUrdf(m,str(res/urdf),pin.GeometryType.COLLISION); gd=gm.createData()
    rad=np.array([g.geometry.radius for g in gm.geometryObjects]); ns=len(rad)
    lo,hi=m.lowerPositionLimit,m.upperPositionLimit
    base=np.zeros(3)
    # workspace obstacle field
    N=500; obs=np.column_stack([rng.uniform(-0.85,0.85,N),rng.uniform(-0.85,0.85,N),rng.uniform(0.0,1.25,N),np.full(N,0.03)])
    reso=32
    for edge_frac in [1.0,0.5]:
        L=rng_len*edge_frac
        cur_tot=0.0; prop_tot=0.0; bp_keep=0.0; rad_keep=0.0; K=400
        for _ in range(K):
            A=rng.uniform(lo,hi)
            dirn=rng.normal(size=m.nq); dirn/=np.linalg.norm(dirn); v=dirn*L
            n=max(int(np.ceil(L/8*reso)),1)*8  # sub-configs along edge (rake=8)
            ts=np.linspace(0,1,n)
            P=np.array([sphere_positions(m,d,gm,gd,A+t*v) for t in ts])  # (n, ns, 3)
            # swept AABB per sphere (+radius)
            smin=P.min(0)-rad[:,None]; smax=P.max(0)+rad[:,None]  # (ns,3)
            # broadphase kept per sphere
            for s in range(ns):
                ins=np.all((obs[:,:3]>=smin[s]-obs[:,3:4])&(obs[:,:3]<=smax[s]+obs[:,3:4]),axis=1)
                bp_keep+=ins.sum()
            # current: per sub-config radial early-exit -> obstacles with |o|-r_o <= |pos_s|+r_s
            dnorm=np.linalg.norm(obs[:,:3]-base,axis=1)-obs[:,3]
            for t in range(n):
                pn=np.linalg.norm(P[t]-base,axis=1)+rad  # (ns,) radial reach per sphere
                for s in range(ns):
                    rad_keep+=(dnorm<=pn[s]).sum()
            # checks: current = sum over subconfig,sphere of radial_kept
            #         proposed = broadphase(ns*N) + n * sum_s broadphase_kept(s)
            bpk_edge=sum(np.all((obs[:,:3]>=smin[s]-obs[:,3:4])&(obs[:,:3]<=smax[s]+obs[:,3:4]),axis=1).sum() for s in range(ns))
            radk_edge=0
            for t in range(n):
                pn=np.linalg.norm(P[t]-base,axis=1)+rad
                radk_edge+=sum((dnorm<=pn[s]).sum() for s in range(ns))
            cur_tot+=radk_edge; prop_tot+=ns*N + n*bpk_edge
        print(f"{name:5s} edge_L={L:.2f} n_sub~{n:3d}: mean_checks/edge current={cur_tot/K:.0f} proposed={prop_tot/K:.0f} speedup={cur_tot/prop_tot:.2f}x | mean kept/sphere: broadphase={bp_keep/(K*ns):.1f} radial(per-subcfg)={rad_keep/(K*ns*n):.1f} of {N}")
