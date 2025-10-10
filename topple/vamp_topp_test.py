import vamp
import numpy as np
from vamp import pybullet_interface as vpb
import pinocchio.visualize
import os
import time
import hppfcl as fcl

pos1 = [0.9941923543408971, -0.051116248331033905, 0.55644523090925, -2.204390838326673, 0.016857619708280643, 2.293574043590873, 0.702844622050357]
pos2 = [-1.4847849572216933, 0.013402851810183476, -0.16617785697236934, -2.1464311136674072, 0.017834601468841655, 2.245508108743958, 0.7687104453444691]
for i in range(14):
    pos1.append(0)
    pos2.append(0)
# set up planner
(
    vamp_module,
    planner_func,
    plan_settings,
    simp_settings,
) = vamp.configure_robot_and_planner_with_kwargs("pandatopp", "rrtctopp")

# robot model, collision model, visual model
path = os.getcwd()
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(
    path + "/fr3_franka_hand.urdf", path + "/fr3/collision", None
)

plane_geom = fcl.Box(0.4, 0.2, 0.5)
plane_name = "front_plane"
plane_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), np.array([0.5, 0, 0.25]))
plane_object = pinocchio.GeometryObject(
    name=plane_name, parent_joint=0, parent_frame=0, placement=plane_placement, collision_geometry=plane_geom
)
plane_object.meshColor = np.array([0, 0, 0, 1])
visual_model.addGeometryObject(plane_object)

plane_geom = fcl.Box(0.4, 0.2, 0.5)
plane_name = "front_plane1"
plane_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), np.array([0.5, 0, 1.1]))
plane_object = pinocchio.GeometryObject(
    name=plane_name, parent_joint=0, parent_frame=0, placement=plane_placement, collision_geometry=plane_geom
)
plane_object.meshColor = np.array([0, 0, 0, 1])
visual_model.addGeometryObject(plane_object)

viz = pinocchio.visualize.MeshcatVisualizer(model, collision_model, visual_model)

try:
    viz.initViewer(zmq_url="tcp://127.0.0.1:6000")
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    exit(0)
viz.loadViewerModel()

rng = vamp_module.halton()

plan_settings.max_iterations = 100000
plan_settings.max_samples = 100000
plan_settings.range = 1
simp_settings.bez = True

# xyz, rpy, lwh
cuboids_data = [
        # table
        #[[0.0, 0.0, -0.08], [0.0, 0.0, 0.0], [1.5, 1.5, 0.1]],
        # roof
        # [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [1.5, 1.5, 0.1]],
        # right wall
        # [[-0.4, 0.0, 0.91], [0.0, 0.0, 0.0], [0.01, 1.0, 1.0]],
        # back
        [[0.5, 0.0, 1.35], [0.0, 0.0, 0.0], [0.2, 0.1, 0.5]],
        # front wall
        [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.2, 0.1, 0.5]],
        # ground plane
        [[0, 0, -0.2], [0.0, 0.0, 0.0], [1.0, 1.0, 0.1]],
    ]

# sim = vpb.PyBulletSimulator(str("../resources/panda/panda.urdf"), vamp_module.joint_names(), True)

env = vamp.Environment()
cuboids = [vamp.Cuboid(*data) for data in cuboids_data]
for cuboid in cuboids:
    env.add_cuboid(cuboid)

# for cuboid in cuboids_data:
#     sim.add_cuboid(cuboid[2], cuboid[0], cuboid[1])

ts = time.perf_counter()
result = planner_func(np.array(pos1), np.array(pos2), env, plan_settings, rng)
tf = time.perf_counter()
print(tf - ts)

if result.solved:
    print("solved")
else:
    print("failed")
    exit()
# print(result.path.numpy())
# simple = vamp_module.simplify(result.path, env, simp_settings, rng)
result = vamp_module.simplify(result.path, env, simp_settings, rng)
traj = vamp_module.compute_traj(result.path, env, simp_settings, rng)
# print(traj.path.numpy(), flush=True)
q_path = traj.path.numpy()[:, 0:7]

viz.display(q_path[0])

input("Ready, press any key: ")
q_curr = None
for i in range(len(q_path)):
    ts = time.perf_counter()
    q_curr = q_path[i]
    viz.display(q_curr)
    tf = time.perf_counter()
    print(tf - ts)
    # time.sleep(0.0001)

