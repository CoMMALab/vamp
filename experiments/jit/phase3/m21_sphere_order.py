# Per-sphere recurrence order (Hankel rank) along an edge: is advancing each sphere
# position directly (not just the trig) viable? Rank = order of the exact linear
# recurrence the sphere obeys along the edge.
import cricket, numpy as np, pinocchio as pin
res=cricket.resources_dir(); urdf=str(res/"fetch/fetch_spherized.urdf")
m=pin.buildModelFromUrdf(urdf); d=m.createData()
gm=pin.buildGeomFromUrdf(m,urdf,pin.GeometryType.COLLISION); gd=gm.createData()
rng=np.random.default_rng(1)
def analyze(edge_len, tol, N=120):
    start=rng.uniform(m.lowerPositionLimit,m.upperPositionLimit)
    v=rng.normal(size=m.nq); v/=np.linalg.norm(v); v*=edge_len
    P=np.zeros((N,len(gm.geometryObjects),3))
    for i,t in enumerate(np.linspace(0,1,N)):
        pin.forwardKinematics(m,d,start+t*v); pin.updateGeometryPlacements(m,d,gm,gd)
        for s,go in enumerate(gd.oMg): P[i,s]=np.array(go.translation)
    def rank(x):
        xn=x-x.mean()
        if np.abs(xn).max()<1e-9: return 1
        L=N//2; Hk=np.array([[xn[i+j] for j in range(L)] for i in range(L)])
        sv=np.linalg.svd(Hk,compute_uv=False); return int((sv>tol*sv[0]).sum())
    return [max(rank(P[:,s,c]) for c in range(3)) for s in range(len(gm.geometryObjects))]
for L in [0.15,0.5,1.0,1.5]:
    o=analyze(L,1e-6); print(f"edge_len={L}: order max={max(o)} median={int(np.median(o))}")
