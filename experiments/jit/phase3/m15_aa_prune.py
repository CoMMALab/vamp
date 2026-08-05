import cricket, numpy as np, pinocchio as pin
exec(open("/tmp/claude-1000/-home-zak-src-vamp-jit/3dcbbe7c-40b1-4721-b12b-3292533b6ada/scratchpad/aa_lib.py").read())  # AA class + helpers
res=cricket.resources_dir()
ROBOTS={"panda":("panda/panda_spherized.urdf",
        [0.,-0.785,0.,-2.356,0.,1.571,0.785],[2.35,1.,0.,-0.8,0.,2.5,0.785]),
        "ur5":("ur5/ur5_spherized.urdf",
        [-1.0,-1.5,1.0,-1.0,-1.0,0.],[1.5,-0.5,1.5,-2.0,1.0,1.0])}


def load_model(name):
    urdf=str(res/ROBOTS[name][0]); m=pin.buildModelFromUrdf(urdf)
    Rs=[];ts=[];ax=[]
    for j in range(1,m.njoints):
        plc=m.jointPlacements[j];Rs.append(np.array(plc.rotation));ts.append(np.array(plc.translation))
        sn=m.joints[j].shortname()
        ax.append(np.array({'RX':[1,0,0],'RY':[0,1,0],'RZ':[0,0,1]}.get(sn[-2:] if sn.startswith('JointModelR') else '',[0,0,1]),float))
    return m,Rs,ts,ax

def aa_frames(name,lo,hi):
    m,Rs,ts,ax=load_model(name)
    R=[[AA(1.0 if i==j else 0.0) for j in range(3)] for i in range(3)]; t=[AA(0.),AA(0.),AA(0.)]
    frames={0:([[AA(1. if i==j else 0.) for j in range(3)] for i in range(3)],[AA(0.),AA(0.),AA(0.)])}
    for j in range(m.njoints-1):
        Rj=[[AA(Rs[j][i][k]) for k in range(3)] for i in range(3)]; tj=[AA(ts[j][i]) for i in range(3)]
        t=[t[i]+(R[i][0]*tj[0]+R[i][1]*tj[1]+R[i][2]*tj[2]) for i in range(3)]; R=matmul_aa(R,Rj)
        th=AA((lo[j]+hi[j])/2,{('q',j):(hi[j]-lo[j])/2}); R=matmul_aa(R,rodrigues_aa(ax[j],th))
        frames[j+1]=([[R[i][k] for k in range(3)] for i in range(3)],[t[i] for i in range(3)])
    return m,frames

def sphere_aabbs_AA(name,lo,hi):
    m,frames=aa_frames(name,lo,hi)
    gm=pin.buildGeomFromUrdf(m,str(res/ROBOTS[name][0]),pin.GeometryType.COLLISION)
    out=[]
    for g in gm.geometryObjects:
        pj=int(g.parentJoint); off=np.array(g.placement.translation); rad=g.geometry.radius
        Rf,tf=frames[pj]; p=[tf[i]+Rf[i][0]*off[0]+Rf[i][1]*off[1]+Rf[i][2]*off[2] for i in range(3)]
        c=np.array([p[i].c for i in range(3)]); hw=np.array([p[i].rad() for i in range(3)])+rad
        out.append((c,hw,rad)); 
    return out
def sphere_aabbs_sampled(name,lo,hi,N=8000):
    m,Rs,ts,ax=load_model(name); gm=pin.buildGeomFromUrdf(m,str(res/ROBOTS[name][0]),pin.GeometryType.COLLISION)
    d=m.createData(); gd=gm.createData(); rng=np.random.default_rng(0); ns=len(gm.geometryObjects)
    mn=np.full((ns,3),1e30); mx=np.full((ns,3),-1e30); mnorm=np.zeros(ns)
    for _ in range(N):
        q=rng.uniform(lo,hi); pin.forwardKinematics(m,d,q); pin.updateGeometryPlacements(m,d,gm,gd)
        for i,go in enumerate(gd.oMg):
            p=np.array(go.translation); mn[i]=np.minimum(mn[i],p); mx[i]=np.maximum(mx[i],p); mnorm[i]=max(mnorm[i],np.linalg.norm(p))
    rad=np.array([g.geometry.radius for g in gm.geometryObjects])
    return [( (mn[i]+mx[i])/2, (mx[i]-mn[i])/2+rad[i], rad[i]) for i in range(ns)], mnorm, rad

def frac_kept(aabbs, obs):  # obs: (M,4) x,y,z,r ; mean over spheres of fraction of obstacles in inflated AABB
    tot=0.0
    for c,hw,rad in aabbs:
        lo=c-hw; hi=c+hw
        inside=np.all((obs[:,:3]>=lo-obs[:,3:4]) & (obs[:,:3]<=hi+obs[:,3:4]),axis=1)
        tot+=inside.mean()
    return tot/len(aabbs)
def frac_kept_radial(mnorm, rad, obs, base=np.zeros(3)):
    tot=0.0
    for i in range(len(mnorm)):
        Rr=mnorm[i]+rad[i]; d=np.linalg.norm(obs[:,:3]-base,axis=1)-obs[:,3]
        tot+=(d<=Rr).mean()
    return tot/len(mnorm)

rng=np.random.default_rng(7)
for name in ["panda","ur5"]:
    urdf,start,goal=ROBOTS[name]; m=pin.buildModelFromUrdf(str(res/urdf))
    start=np.array(start); goal=np.array(goal)
    # workspace obstacle field
    M=2000; obs=np.column_stack([rng.uniform(-0.85,0.85,M),rng.uniform(-0.85,0.85,M),rng.uniform(0.0,1.25,M),np.full(M,0.03)])
    for margin,label in [(None,"full-box"),(0.3,"corridor+0.3"),(0.1,"corridor+0.1")]:
        if margin is None: lo,hi=m.lowerPositionLimit.copy(),m.upperPositionLimit.copy()
        else:
            lo=np.minimum(start,goal)-margin; hi=np.maximum(start,goal)+margin
            lo=np.maximum(lo,m.lowerPositionLimit); hi=np.minimum(hi,m.upperPositionLimit)
        aa=sphere_aabbs_AA(name,lo,hi); samp,mnorm,rad=sphere_aabbs_sampled(name,lo,hi)
        fa=frac_kept(aa,obs); fs=frac_kept(samp,obs); fr=frac_kept_radial(mnorm,rad,obs)
        print(f"{name:5s} {label:13s}: frac_kept  AA={fa:.3f}  sampled={fs:.3f}  radial={fr:.3f}  | speedup AA~{1/max(fa,1e-3):.1f}x sampled~{1/max(fs,1e-3):.1f}x radial~{1/max(fr,1e-3):.1f}x")
