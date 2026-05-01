import vamp
import numpy as np
from vamp import pybullet_interface as vpb
import matplotlib.pyplot as plt

pos1 = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
pos2 = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]
for i in range(14):
    pos1.append(0)
    pos2.append(0)
# set up planner
(
    vamp_module,
    planner_func,
    plan_settings,
    simp_settings,
) = vamp.configure_robot_and_planner_with_kwargs("pandatopp", "topple")

rng = vamp_module.xorshift()

plan_settings.max_iterations = 10000000
plan_settings.rrtc.max_iterations = 100000
plan_settings.max_samples = 10000000
plan_settings.simplify.bez = True
plan_settings.use_phs = False
plan_settings.optimize = False
plan_settings.simplify_intermediate = False
plan_settings.max_runs = 1
plan_settings.cost_bound_resample = False
plan_settings.bez_range = 0.1
plan_settings.k_nearest = 4
plan_settings.alpha = 0.1
plan_settings.dynamic_extension = True

env = vamp.Environment()
sim = vpb.PyBulletSimulator(str("../resources/panda/panda.urdf"), vamp_module.joint_names(), True)

result = planner_func(np.array(pos1), np.array(pos2), env, plan_settings, rng)
if result.solved:
    print("solved")
else:
    print("failed")
    exit()

traj = vamp_module.compute_bez_traj(result, env, simp_settings, rng)
path = traj.path.numpy()

sim.animate(traj.path.numpy()[np.arange(0, len(traj.path.numpy()), 10)])

# bez = result.beziers[1]
# sub_bez = bez.subdivide(0.1)

# traj1 = np.array(bez.generate_trajectory())
# traj2 = np.array(sub_bez.generate_trajectory())
# ax = plt.figure().add_subplot(projection='3d')
# ax.plot(traj1[:, 0], traj1[:, 1], traj1[:, 2], label="bez")
# ax.plot(traj2[:, 0], traj2[:, 1], traj2[:, 2], label="sub_bez")
# ax.scatter(bez.anchors[:, 0], bez.anchors[:, 1], bez.anchors[:, 2], label="bez anchors")
# ax.scatter(sub_bez.anchors[:, 0], sub_bez.anchors[:, 1], sub_bez.anchors[:, 2], label="sub_bez anchors")
# plt.legend()
# plt.show()
# print(bez.combs)
