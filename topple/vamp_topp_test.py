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

# plane_geom = fcl.Box(0.4, 0.2, 0.5)
# plane_name = "front_plane"
# plane_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), np.array([0.5, 0, 0.25]))
# plane_object = pinocchio.GeometryObject(
#     name=plane_name, parent_joint=0, parent_frame=0, placement=plane_placement, collision_geometry=plane_geom
# )
# plane_object.meshColor = np.array([0, 0, 0, 1])
# visual_model.addGeometryObject(plane_object)

# plane_geom = fcl.Box(0.4, 0.2, 0.5)
# plane_name = "front_plane1"
# plane_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), np.array([0.5, 0, 1.05]))
# plane_object = pinocchio.GeometryObject(
#     name=plane_name, parent_joint=0, parent_frame=0, placement=plane_placement, collision_geometry=plane_geom
# )
# plane_object.meshColor = np.array([0, 0, 0, 1])
# visual_model.addGeometryObject(plane_object)



rng = vamp_module.halton()

plan_settings.max_iterations = 100000
plan_settings.max_samples = 1000000
plan_settings.range = 1
simp_settings.bez = True
plan_settings.radius = 2
plan_settings.min_radius = 0.5

# xyz, rpy, lwh
cuboids_data = [
        # back
        # [[0.0, 0.0, 1.3], [0.0, 0.0, 0.0], [1.6, 0.1, 0.5]],
        # front wall
        # [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.2, 0.1, 0.5]],
        # ground plane
        [[0, 0, -0.15], [0.0, 0.0, 0.0], [1.0, 1.0, 0.1]],
        # [[0.4, -0.6, 0.8], [0, 0, 0], [0.2, 0.1, 0.5]]
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

for i in range(len(cuboids_data)):
    cuboid = cuboids_data[i]
    plane_geom = fcl.Box(cuboid[2][0] * 2, cuboid[2][1] * 2, 2 * cuboid[2][2])
    plane_name = f"front_plane{i}"
    plane_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), np.array([cuboid[0][0], cuboid[0][1], cuboid[0][2]]))
    plane_object = pinocchio.GeometryObject(
        name=plane_name, parent_joint=0, parent_frame=0, placement=plane_placement, collision_geometry=plane_geom
    )
    plane_object.meshColor = np.array([0, 0, 0, 1])
    visual_model.addGeometryObject(plane_object)

for i in range(len(spheres)):
    sphere = spheres[i]
    sphere_geom = fcl.Sphere(0.05)
    sphere_name = f"sphere{i}"
    sphere_placement = pinocchio.SE3(pinocchio.utils.rotate('x', 0), sphere)
    sphere_object = pinocchio.GeometryObject(
        name=sphere_name, parent_joint=0, parent_frame=0, placement=sphere_placement, collision_geometry=sphere_geom
    )
    sphere_object.meshColor = np.array([1, 1, 1, 1])
    visual_model.addGeometryObject(sphere_object)

# viz = pinocchio.visualize.MeshcatVisualizer(model, collision_model, visual_model)

# try:
#     viz.initViewer(zmq_url="tcp://127.0.0.1:6002")
# except ImportError as err:
#     print(
#         "Error while initializing the viewer. It seems you should install Python meshcat"
#     )
#     print(err)
#     exit(0)
# viz.loadViewerModel()


env = vamp.Environment()
sim = vpb.PyBulletSimulator(str("../resources/panda/panda.urdf"), vamp_module.joint_names(), True)

# cuboids = [vamp.Cuboid(*data) for data in cuboids_data]
# for cuboid in cuboids:
#     env.add_cuboid(cuboid)

spheres = [vamp.Sphere(sphere, 0.05) for sphere in spheres]
for sphere in spheres:
    env.add_sphere(sphere)
    sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z])



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

path = traj.path.numpy()
q_path = traj.path.numpy()[:, 0:7]
# for p in path:
#     if not vamp_module.validate(p, env, True):
#         print("Invalid state in plan!") # :(
#         break

# viz.display(q_path[0])
sim.animate(path[np.arange(0, len(path), 5)])

# input("Ready, press any key: ")
# q_curr = pos1[0:7]
# for i in range(0, len(q_path), 1):
#     ts = time.perf_counter()
#     # q_curr += qd_path[i] * 0.001
#     q_curr = q_path[i]
#     print(path[i])
#     # if not vamp_module.validate(path[i], env, False):
#     #     print(f"Invalid state {i} in plan!") # :(
#     viz.display(q_curr)
#     tf = time.perf_counter()
#     # print(tf - ts)
#     # time.sleep(0.0001)

def exec_vels(qd_path):
    robot = Robot("172.16.0.2")

    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

        # Start joint position control with external control loop
        active_control = robot.start_joint_position_control(ControllerMode.CartesianImpedance)

        initial_position = [0.0] * 7
        time_elapsed = 0.0
        motion_finished = False

        # External control loop
        while not motion_finished:
            # Read robot state and duration
            robot_state, duration = active_control.readOnce()

            # Update time
            time_elapsed += duration.to_sec()

            # On first iteration, capture initial position
            if time_elapsed <= duration.to_sec():
                initial_position = robot_state.q_d if hasattr(robot_state, "q_d") else robot_state.q

            # Calculate delta angle using the same formula as in C++ example
            delta_angle = np.pi / 8.0 * (1 - np.cos(np.pi / 2.5 * time_elapsed))

            # Update joint positions
            new_positions = [
                initial_position[0],
                initial_position[1],
                initial_position[2],
                initial_position[3] + delta_angle,
                initial_position[4] + delta_angle,
                initial_position[5],
                initial_position[6] + delta_angle,
            ]

            # Set joint positions
            joint_positions = JointPositions(new_positions)

            # Set motion_finished flag to True on the last update
            if time_elapsed >= 5.0:
                joint_positions.motion_finished = True
                motion_finished = True
                print("Finished motion, shutting down example")

            # Send command to robot
            active_control.writeOnce(joint_positions)
    except:
        print("Error")

