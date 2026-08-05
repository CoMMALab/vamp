import cricket, json, numpy as np, pinocchio as pin
res = cricket.resources_dir()
recipe = json.loads((res/"panda.json").read_text()) if (res/"panda.json").exists() else {"urdf":"panda/panda_spherized.urdf"}
urdf = str(res / recipe["urdf"])
m = pin.buildModelFromUrdf(urdf); d = m.createData()
print("nq",m.nq,"njoints",m.njoints)
for j in range(1,m.njoints):
    jm = m.joints[j]
    # axis for revolute: nv==1, motion subspace
    plc = m.jointPlacements[j]
    print(f"joint {j} {m.names[j]} type={jm.shortname()} parent={m.parents[j]}")
    print(f"   placement t={np.round(plc.translation,4)}")
# extract axis via joint's classname / for revolute use the S matrix
q0 = pin.neutral(m); pin.forwardKinematics(m,d,q0)
# validate: my scalar FK vs pinocchio. Extract jointPlacement (R,t) and revolute axis per joint.
axes=[]; Rs=[]; ts=[]
for j in range(1,m.njoints):
    plc=m.jointPlacements[j]; Rs.append(np.array(plc.rotation)); ts.append(np.array(plc.translation))
    jm=m.joints[j]; sn=jm.shortname()
    if 'RX' in sn: ax=np.array([1,0,0.])
    elif 'RY' in sn: ax=np.array([0,1,0.])
    elif 'RZ' in sn: ax=np.array([0,0,1.])
    else:
        # generic revolute: axis in S
        S=np.array(jm.calc_zero_order if False else [0,0,1.]) ; ax=np.array([0,0,1.])
    axes.append(ax)
    print(f"   axis~{ax} shortname={sn}")
def rod(ax,th):
    K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    return np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K
def myfk(q):
    T=np.eye(4); origins=[]
    for j in range(m.njoints-1):
        Tj=np.eye(4); Tj[:3,:3]=Rs[j]; Tj[:3,3]=ts[j]
        T=T@Tj
        Rr=np.eye(4); Rr[:3,:3]=rod(axes[j],q[j]); T=T@Rr
        origins.append(T[:3,3].copy())
    return origins
# compare vs pinocchio for random q
np.random.seed(0); maxerr=0
for _ in range(200):
    q=np.random.uniform(m.lowerPositionLimit,m.upperPositionLimit)
    pin.forwardKinematics(m,d,q); mo=myfk(q)
    for j in range(1,m.njoints):
        err=np.linalg.norm(np.array(d.oMi[j].translation)-mo[j-1]); maxerr=max(maxerr,err)
print("MAX scalar-FK-vs-pinocchio origin error:", maxerr)
