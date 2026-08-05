import cricket, json, numpy as np, pinocchio as pin
res=cricket.resources_dir()
ROBOTS={"panda":"panda/panda_spherized.urdf","ur5":"ur5/ur5_spherized.urdf","fetch":"fetch/fetch_spherized.urdf"}

# ---------- Affine Arithmetic ----------
_ctr=[0]
def newsym():
    _ctr[0]+=1; return ('e',_ctr[0])
class AA:
    __slots__=('c','n')
    def __init__(self,c=0.0,n=None): self.c=float(c); self.n=n or {}
    def rad(self): return sum(abs(v) for v in self.n.values())
    def interval(self): r=self.rad(); return (self.c-r,self.c+r)
    def __add__(s,o):
        if isinstance(o,AA):
            n=dict(s.n)
            for k,v in o.n.items(): n[k]=n.get(k,0.0)+v
            return AA(s.c+o.c,n)
        return AA(s.c+o,dict(s.n))
    __radd__=__add__
    def __sub__(s,o): return s+(o*-1)
    def __rsub__(s,o): return (s*-1)+o
    def __mul__(s,o):
        if isinstance(o,AA):
            c=s.c*o.c; n={}
            for k,v in s.n.items(): n[k]=n.get(k,0.0)+o.c*v
            for k,v in o.n.items(): n[k]=n.get(k,0.0)+s.c*v
            nl=s.rad()*o.rad()
            if nl>0: n[newsym()]=nl
            return AA(c,n)
        return AA(s.c*o,{k:v*o for k,v in s.n.items()})
    __rmul__=__mul__
    def __neg__(s): return s*-1

def cheb(f,a,b):
    if b-a<1e-12: return (0.0,f(a),0.0)
    xs=np.linspace(a,b,400); al=(f(b)-f(a))/(b-a)
    g=f(xs)-al*xs; r=g.max(); s=g.min()
    return (al,(r+s)/2,(r-s)/2)
def aa_apply(f,x):
    a,b=x.interval(); al,be,de=cheb(f,a,b)
    return (x*al)+AA(be,{newsym():abs(de)})
def aa_sin(x): return aa_apply(np.sin,x)
def aa_cos(x): return aa_apply(np.cos,x)

# ---------- model ----------
def load(name):
    m=pin.buildModelFromUrdf(str(res/ROBOTS[name])); d=m.createData()
    Rs=[];ts=[];ax=[]
    for j in range(1,m.njoints):
        plc=m.jointPlacements[j];Rs.append(np.array(plc.rotation));ts.append(np.array(plc.translation))
        sn=m.joints[j].shortname()
        ax.append({'RX':[1,0,0],'RY':[0,1,0],'RZ':[0,0,1]}.get(sn[-2:] if sn.startswith('JointModelR') else '',[0,0,1]))
    return m,d,Rs,ts,[np.array(a,float) for a in ax]

def rodrigues_aa(axis,th):
    s=aa_sin(th); c=aa_cos(th); x,y,z=axis
    K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]); K2=K@K
    R=[[None]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            e=AA(1.0 if i==j else 0.0)+ s*K[i][j] + (AA(1.0)-c)*K2[i][j]
            R[i][j]=e
    return R
def matmul_aa(A,B): # 3x3 * 3x3 (entries AA or float)
    C=[[AA(0.0) for _ in range(3)] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            acc=AA(0.0)
            for k in range(3): acc=acc+A[i][k]*B[k][j]
            C[i][j]=acc
    return C
def matvec_aa(A,v):
    return [A[i][0]*v[0]+A[i][1]*v[1]+A[i][2]*v[2] for i in range(3)]

def aa_fk(name,lo,hi):
    m,d,Rs,ts,ax=load(name)
    R=[[AA(1.0 if i==j else 0.0) for j in range(3)] for i in range(3)]  # base identity
    t=[AA(0.0),AA(0.0),AA(0.0)]
    origins=[]
    for j in range(m.njoints-1):
        Rj=[[AA(Rs[j][i][k]) for k in range(3)] for i in range(3)]
        tj=[AA(ts[j][i]) for i in range(3)]
        # T = T * fixed(Rj,tj)
        t=[t[i]+ (R[i][0]*tj[0]+R[i][1]*tj[1]+R[i][2]*tj[2]) for i in range(3)]
        R=matmul_aa(R,Rj)
        # T = T * Rz(theta_j)
        th=AA((lo[j]+hi[j])/2,{('q',j):(hi[j]-lo[j])/2})
        Rrot=rodrigues_aa(ax[j],th)
        R=matmul_aa(R,Rrot)
        origins.append([t[0].c,t[1].c,t[2].c, t[0].rad(),t[1].rad(),t[2].rad()])  # center + halfwidths
    return m,origins

def sampled(name,lo,hi,N=20000):
    m,d,Rs,ts,ax=load(name); rng=np.random.default_rng(0)
    mins=np.full((m.njoints-1,3),1e30); maxs=np.full((m.njoints-1,3),-1e30); maxnorm=np.zeros(m.njoints-1)
    for _ in range(N):
        q=rng.uniform(lo,hi); pin.forwardKinematics(m,d,q)
        for j in range(1,m.njoints):
            o=np.array(d.oMi[j].translation); mins[j-1]=np.minimum(mins[j-1],o);maxs[j-1]=np.maximum(maxs[j-1],o);maxnorm[j-1]=max(maxnorm[j-1],np.linalg.norm(o))
    return mins,maxs,maxnorm

def halfvol(hw): return np.prod(2*np.array(hw))  # AABB volume from halfwidths
for name in ["panda","ur5"]:
    m=pin.buildModelFromUrdf(str(res/ROBOTS[name]))
    for label,frac in [("full-box",1.0),("corridor",0.15)]:
        c=(m.lowerPositionLimit+m.upperPositionLimit)/2; half=(m.upperPositionLimit-m.lowerPositionLimit)/2*frac
        # corridor centered at a random-ish config
        if frac<1.0: c=m.lowerPositionLimit+0.5*(m.upperPositionLimit-m.lowerPositionLimit)
        lo=c-half; hi=c+half
        _,aa=aa_fk(name,lo,hi); mins,maxs,mn=sampled(name,lo,hi)
        # compare last link (end) halfwidths
        j=len(aa)-1
        aa_hw=np.array(aa[j][3:]); samp_hw=(maxs[j]-mins[j])/2; rad_hw=np.array([mn[j]]*3)
        print(f"{name:5s} {label:9s} endlink: AA_hw={np.round(aa_hw,3)} sampled_hw={np.round(samp_hw,3)} radial_hw~{np.round(rad_hw,2)}")
        # mean over links: AABB-volume ratio AA/sampled, and radial/sampled
        vr=[];rr=[]
        for j in range(len(aa)):
            shw=(maxs[j]-mins[j])/2; ah=np.array(aa[j][3:])
            if np.prod(shw)>1e-9: vr.append(np.prod(ah)/np.prod(shw)); rr.append((mn[j]**3)/np.prod(shw))
        print(f"      mean AABB-vol  AA/sampled={np.mean(vr):.1f}x   radial/sampled={np.mean(rr):.1f}x  (1=tight, big=loose)")
