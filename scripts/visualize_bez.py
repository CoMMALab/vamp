import vamp
import numpy as np
from vamp import pybullet_interface as vpb
import matplotlib.pyplot as plt

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

rng = vamp_module.xorshift()

plan_settings.max_iterations = 100000
plan_settings.max_samples = 1000000
plan_settings.range = 4
simp_settings.bez = True
plan_settings.radius = 8
plan_settings.min_radius = 0.5
plan_settings.balance = False

# xyz, rpy, lwh
cuboids_data = [
        # back
        # [[0.0, 0.0, 1.5], [0.0, 0.0, 0.0], [1.6, 0.1, 0.5]],
        # front wall
        # [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.2, 0.1, 0.5]],
        # ground plane
        [[0, 0, -0.15], [0.0, 0.0, 0.0], [1.0, 1.0, 0.1]],
        # [[0.4, -0.7, 0.8], [0, 0, 0], [0.2, 0.1, 0.5]]
    ]
spheres = [
    [0.35, 0, 0.6],
    [0.25, 0.25, 0.25],
    # [0, 0.55, 0.45],
    [-0.35, 0, 0.25],
    [-0.25, -0.35, 0.5],
    # [0, -0.55, 0.25],
    [0.25, -0.35, 0.8],
    [0.35, 0.35, 0.3],
    # [0, 0.55, 0.4],
    [-0.35, 0.35, 0.45],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.45],
    # [0, -0.55, 0.4],
    [0.35, -0.35, 0.45],
]
spheres = np.array(spheres)

env = vamp.Environment()
sim = vpb.PyBulletSimulator(str("../resources/panda/panda.urdf"), vamp_module.joint_names(), True)

cuboids = [vamp.Cuboid(*data) for data in cuboids_data]
for cuboid in cuboids:
    env.add_cuboid(cuboid)
for cuboid in cuboids_data:
    sim.add_cuboid(cuboid[2], cuboid[0], cuboid[1])

spheres = [vamp.Sphere(sphere, 0.05) for sphere in spheres]
for sphere in spheres:
    env.add_sphere(sphere)
    sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z])

result = planner_func(np.array(pos1), np.array(pos2), env, plan_settings, rng)

if result.solved:
    print("solved")
else:
    print("failed")
    exit()

result = vamp_module.simplify(result.path, env, simp_settings, rng)
traj = vamp_module.compute_traj(result.path, env, simp_settings, rng)

path = traj.path.numpy()
q_path = traj.path.numpy()[:, 0:7]
for i in range(7):
    plt.plot(q_path[:, i])
    plt.legend([f"joint {j}" for j in range(7)])
plt.show()

# sim.animate(result.path.numpy())

anchors = np.ones((6, 7))
bez = vamp.Bezier(anchors)
print(bez.combs)
