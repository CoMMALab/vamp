import numpy as np
import os
from pathlib import Path
import json
import vamp
from fire import Fire
import itertools
import meshcat_viz as viz
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import mjcf_parser


start = [
    0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000,
    0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
    -0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
    -0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
    0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000
]

box_top_shelf_pregrasp = [
    0.0063, -0.0209, 0.0088, -0.0000, -0.0181, -0.0059, 
    0.3864, -0.0001, 0.2961, 0.3205, -0.0071, -0.2946, 0.0024, 0.0088, 
    0.1748, -0.1121, 0.3611, 1.0023, 
    -0.3695, -0.0039, -0.3003, -0.3223, 0.0047, 0.2928, -0.1172, 0.0056, 
    -0.1745, 0.0752, -0.0661, -1.0229
]


# box_top_shelf_pickup = [
#     0.0103, -0.0199, 0.0059, -0.0033, -0.0143, -0.0060, 
    # 0.3841, 0.0005, 0.2989, 0.3228, -0.0038, -0.3013, 0.0096, 0.0102, 
    # -0.2059, 0.1104, -0.0340, -0.3423, 
    # -0.3721, -0.0033, -0.2989, -0.3189, 0.0087, 0.2840, -0.1172, 0.0068, 
    # -0.0953, -0.0804, -0.0383, 0.2745
# ]

box_top_shelf_pickup = [
    0.0063, -0.0209, 0.0088, -0.0000, -0.0181, -0.0059, 
    0.3864, -0.0001, 0.2961, 0.3205, -0.0071, -0.2946, 0.0024, 0.0088, 
    0.1748, -0.1121, 0.3611, 1.0023, 
    -0.3695, -0.0039, -0.3003, -0.3223, 0.0047, 0.2928, -0.1172, 0.0056, 
    -0.1745, 0.0752, -0.0661, -1.0229
]

# rack_2 = [
# 0.0155, -0.0108, -0.4696, -0.0162, -0.0259, -0.0076, 
# 0.4025, 0.0069, -0.2216, -0.8470, -0.0189, 0.8939, -0.5340, 0.0435, 
# -0.0078, 0.3387, 0.0149, -0.1738, 
# -0.3596, -0.0049, 0.2166, 0.8447, 0.0196, -0.9010, 0.4214, 0.0378, 
# 0.0557, -0.2407, -0.1004, 0.1267
# ]

# rack_2 = [
#     0.0150, -0.0110, -0.4671, -0.0149, -0.0278, -0.0076, 
#     0.4035, 0.0064, -0.2223, -0.8493, -0.0209, 0.8991, -0.5371, 0.0431, 
#     0.2043, 0.3651, -0.0362, 0.8329, 
#     -0.3588, -0.0055, 0.2165, 0.8424, 0.0178, -0.8959, 0.4204, 0.0372, 
#     0.2043, 0.3651, -0.0362, 0.8329, 
#     -0.1535, -0.0866, -0.4511, -1.0565
# ]

rack_2 = [
0.0148, -0.0110, -0.4670, -0.0149, -0.0269, -0.0076, 
0.4035, 0.0063, -0.2221, -0.8494, -0.0209, 0.8992, -0.5376, 0.0431, 
0.1760, 0.3800, 0.1001, 0.9285, 
-0.3587, -0.0056, 0.2158, 0.8424, 0.0176, -0.8957, 0.4207, 0.0372, 
-0.1361, -0.3690, -0.0670, -0.9361    
]

rack_3 = [
0.0124, -0.0160, -0.1359, -0.0078, -0.0150, -0.0074, 0.3866, 0.0035, 0.0986, -0.0945, -0.0076, 0.1193, -0.1893, 0.0176, 0.0068, 0.1739, 0.0643, -0.0354, -0.3707, -0.0048, -0.1002, 0.0951, 0.0115, -0.1313, 0.0793, 0.0139, 0.0814, -0.1567, -0.0306, 0.0591
]

# rack_2_release = [
# 0.01738501, -0.01096749, -0.47006118, -0.01586935, -0.02395158, -0.00807717  ,
#   0.4024402 ,  0.00743171, -0.22230545, -0.8472173 , -0.01932546,  0.8947249 ,
#  -0.5300461 ,  0.04379871,  0.09859389,  0.35082862,  -0.08830395,  0.13806033,
#  -0.35991108, -0.00484328,  0.21751796,  0.8449603 ,  0.02007664, -0.9019995 ,
#   0.41773874,  0.03763357,  0.11142297, -0.31057477, 0.08879709, -0.08477921,
# ]
rack_2_release = [
    # 0.0150, -0.0110, -0.4671, -0.0149, -0.0278, -0.0076, 
    # 0.4035, 0.0064, -0.2223, -0.8493, -0.0209, 0.8991, -0.5371, 0.0431, 
    # 0.2043, 0.3651, -0.90362, 0.8329, 
    # -0.3588, -0.0055, 0.2165, 0.8424, 0.0178, -0.8959, 0.4204, 0.0372, 
    # -0.1535, -0.0866, 0.610511, -1.0565

0.0148, -0.0110, -0.4670, -0.0149, -0.0269, -0.0076, 
0.4035, 0.0063, -0.2221, -0.8494, -0.0209, 0.8992, -0.5376, 0.0431, 
0.1760, 0.3800, -0.4001, 0.9285, 
-0.3587, -0.0056, 0.2158, 0.8424, 0.0176, -0.8957, 0.4207, 0.0372, 
-0.1361, -0.3690, 0.40670, -0.9361    


]


# rack_3 = [
#     0.00927,-0.01219,-0.47283,-0.01411,-0.02955,-0.00937,0.40005,0.01408,-0.23717,-0.84704,-0.02429,0.90190,-0.48186,0.04543,-0.17059,-0.36363,0.06204,1.09005,-0.36164,-0.00267,0.22962,0.84538,0.02448,-0.90900,0.40052,0.03680,-0.29646,0.37895,-0.15478,-1.16821
# ]
'''

start = [
    0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000,
    0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
    -0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
    -0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
    0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000
]

box_top_shelf_pregrasp = [
    1.11889839e-02, -1.99530125e-02,  5.81884384e-03, -3.29416106e-03, -1.42211439e-02, -5.99248195e-03,
    3.84082675e-01,  5.58406231e-04, 2.98647016e-01,  3.22717577e-01, -3.98048153e-03, -3.00990015e-01, 1.07899308e-02,  1.02880923e-02,
    3.03654131e-02,  1.43816188e-01, -0.139, -5.22131145e-01,
     -3.72209966e-01, -3.30008916e-03, -2.98724055e-01, -3.18776041e-01,  8.88650957e-03,  2.83709198e-01, -1.18316561e-01,  6.75600534e-03,
    4.73398268e-02, -1.23632416e-01,  0.139,  4.99689251e-01
]


box_top_shelf_pickup = [
    0.0108, -0.0207, 0.0077, -0.0008, -0.0142, -0.0059, 0.3858, 0.0002, 0.2982, 0.3205, -0.0065, -0.2954, 0.0072, 0.0094, 0.0304, 0.1438, 0.2460, -0.5221, -0.3705, -0.0037, -0.2994, -0.3210, 0.0064, 0.2893, -0.1210, 0.0058, 0.0473, -0.1236, -0.2170, 0.4997
]

rack_2 = [
0.0160, -0.0108, -0.4697, -0.0161, -0.0261, -0.0077,
0.4025, 0.0070, -0.2220, -0.8470, -0.0190, 0.8941, -0.5331, 0.0436, 
0.1756, 0.3280, 0.1925, -0.1410,
 -0.3597, -0.0049, 0.2170, 0.8447, 0.0197, -0.9012, 0.4206, 0.0378,
0.0701, -0.2473, -0.3055, 0.1341
]

rack_2_release = [
0.0160, -0.0108, -0.4697, -0.0161, -0.0261, -0.0077,
0.4025, 0.0070, -0.2220, -0.8470, -0.0190, 0.8941, -0.5331, 0.0436, 
0.1756, 0.3280, 0.001925, -0.1410,
 -0.3597, -0.0049, 0.2170, 0.8447, 0.0197, -0.9012, 0.4206, 0.0378,
0.0701, -0.2473, -0.003055, 0.1341
]


rack_3 = [
    0.0117, -0.0164, -0.1340, -0.0062, -0.0155, -0.0077, 0.3877, 0.0032, 0.0981, -0.0966, -0.0096, 0.1239, -0.1928, 0.0169, 0.2535, 0.1767, 0.2234, -0.0627, -0.3695, -0.0052, -0.1006, 0.0930, 0.0095, -0.1265, 0.0777, 0.0133, 0.1542, -0.0989, -0.2631, 0.1183
]

rack_3_release = [
    0.0120, -0.0160, -0.1358, -0.0078, -0.0154, -0.0077,
    0.3866, 0.0035, 0.0985, -0.0945, -0.0076, 0.1192, -0.1897,
    0.0175, 0.2535, 0.0767, 0.2234,
    -0.0627, -0.3706, -0.0048, -0.1001, 0.0950, 0.0114, -0.1311, 0.0797, 0.0139,
    0.1542, -0.0989, -0.0631, 0.1183
]
'''
attach_spheres = [
    [-0.05, 0.0, -0.04],
    [-0.05, -0.02, -0.07],
    [-0.05, -0.04, -0.11],
    [-0.05, -0.06, -0.13],
    [-0.05, 0.05, -0.04],
    [-0.05, 0.03, -0.07],
    [-0.05, 0.01, -0.11],
    [-0.05, -0.01, -0.13],
    [-0.02, 0.0, -0.04],
    [-0.02, -0.02, -0.07],
    [-0.02, -0.04, -0.11],
    [-0.02, -0.06, -0.13],
    [-0.02, 0.05, -0.04],
    [-0.02, 0.03, -0.07],
    [-0.02, 0.01, -0.11],
    [-0.02, -0.01, -0.13],
    # [0.01, 0.0, -0.04],
    # [0.01, -0.02, -0.07],
    # [0.01, -0.04, -0.11],
    # [0.01, -0.06, -0.13],
    # [0.01, 0.05, -0.04],
    # [0.01, 0.03, -0.07],
    # [0.01, 0.01, -0.11],
    # [0.01, -0.01, -0.13],
    # [0.04, 0.0, -0.04],
    # [0.04, -0.02, -0.07],
    # [0.04, -0.04, -0.11],
    # [0.04, -0.06, -0.13],
    # [0.04, 0.05, -0.04],
    # [0.04, 0.03, -0.07],
    # [0.04, 0.01, -0.11],
    # [0.04, -0.01, -0.13],
    # [-0.07, 0.0, -0.04],
    # [-0.07, -0.02, -0.07],
    # [-0.07, -0.04, -0.11],
    # [-0.07, -0.06, -0.13],
    # [-0.07, 0.05, -0.04],
    # [-0.07, 0.03, -0.07],
    # [-0.07, 0.01, -0.11],
    # [-0.07, -0.01, -0.13],
]

def convert_trajectory_eef_to_controller_format(trajectory, eef_waypoints):
    # create a 8 + 7 + 7 + 7 = 29 dim waypoint
    # the first 8 values are left and right arms joint positions
    # next 7 is the floating base pose (x, y, z, qw, qx, qy, qz)
    # next 7 is left foot pose (x, y, z, qw, qx, qy, qz)
    # next 7 is right foot pose (x, y, z, qw, qx, qy, qz)
    # eef_waypoints is a n x 4 x 4 x 4 array where the first 4 is number of eefs, n is number of waypoints, and the last 4x4 is the transform of the eef in world frame

    print(eef_waypoints.shape)

    output = np.zeros((len(trajectory), 29))
    # arm joints are 14:18 and, 26:29
    output[:, :8] = trajectory[:, [14, 15, 16, 17, 26, 27, 28, 29]]
    # floating base is 0:6. Need to convert euler to quat for orientation

    def to_quat(euler):
        r = R.from_euler('xyz', euler, degrees=False)
        return r.as_quat() # returns in (x, y, z, w) format

    def to_quat_from_rotmatrix(rotmat):
        r = R.from_matrix(rotmat)
        return r.as_quat() # returns in (x, y, z, w) format

    output[:, 8:11] = trajectory[:, 0:3]
    output[:, 11:15] = np.apply_along_axis(to_quat, 1, trajectory[:, 3:6])
    # left foot is eef_waypoints[:, 2] as a 4x4 matrix
    # need to convert it to (x, y, z, qw, qx, qy, qz) format
    output[:, 15:18] = eef_waypoints[:, 2, :3, 3] # position
    output[:, 18:22] = np.array([to_quat_from_rotmatrix(eef_waypoints[i, 2, :3, :3]) for i in range(len(eef_waypoints))]) # orientation
    # right foot is eef_waypoints[:, 3] as a 4x4 matrix
    output[:, 22:25] = eef_waypoints[:, 3, :3, 3] # position
    output[:, 25:29] = np.array([to_quat_from_rotmatrix(eef_waypoints[i, 3, :3, :3]) for i in range(len(eef_waypoints))]) # orientation

    return output




def get_eef_of_waypoints(waypoints):

    (vamp_module, _, _, _) = vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc")
    eef_poses = []
    for waypoint in waypoints:
        eef_pose = [pose.tolist() for pose in vamp_module.eefk(waypoint)]
        eef_poses.append(eef_pose)
    return np.array(eef_poses)

def run_planner(
    start,
    goal,
    combination,
    constraint = 0,
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,
    **kwargs,
):



    polygon_points = [
        0.005, -0.01, 0.005, 0.03, -0.05, 0.03, -0.05, -0.01
    ]

    bimanual_limit_lower_bound = [
        -0.005, -0.005, -0.005, -0.1, -0.1, -10.1
    ]
    bimanual_limit_upper_bound = [
        0.005, 0.005, 0.005, 0.1, 0.1, 10.1
    ]

    tsr_lower_bound = [
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
        -0.001, -0.001, -0.001, -0.03, -0.03, -0.03,
        -0.001, -0.001, -0.001, -0.03, -0.03, -0.03
    ]

    tsr_upper_bound = [
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
        0.001, 0.001, 0.001, 0.03, 0.03, 0.03,
        0.001, 0.001, 0.001, 0.03, 0.03, 0.03
    ]


    eef_transforms = [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [0.59, 0.38, 0.4, 0.59 , -0.02070, 0.06015, -0.94832], [0.61, -0.36, 0.35, -0.61 , -0.02228, -0.11609, -0.94832]]
    eef_transforms_ref_frame_w_world = [[1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0]]

    # bimanual_transform = [0.05, 0.95, 0.00000, 0.3, 0.06308, -0.00, -0.17395]
    bimanual_transform = [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.28];




    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc", **kwargs)
    )

    print(vamp_module.joint_names())



    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        bimanual_transform,
        bimanual_limit_lower_bound,
        bimanual_limit_upper_bound
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        polygon_points,
    )


    feet_tsr_constraint = vamp_module.TaskSpaceConstraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    )

    closed_link_constraint = vamp_module.ClosedLinkConstraint()


    transport_constraints = vamp_module.Composable_F_C_CL_B(feet_tsr_constraint, com_constraint, closed_link_constraint, bimanual_constraint)
    non_transport_constraints = vamp_module.Composable_F_C_CL(feet_tsr_constraint, com_constraint, closed_link_constraint)




    e = vamp.Environment()
    env_geoms = mjcf_parser.parse_mjcf('resources/environments/cuboids/wooden_shelf.xml')
    for geom in env_geoms:
        if geom.type == mjcf_parser.GeomType.BOX:
            if 'transport' in geom.geom_name and constraint!= 0:
                continue
            e.add_cuboid(vamp.Cuboid([geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z], [0, 0, 0], [geom.size.x, geom.size.y, geom.size.z]))

    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, 0])
    attachment = vamp.Attachment(tf)

    attachment.add_spheres(
        [
            vamp.Sphere(sphere, 0.02) for sphere in attach_spheres
        ]
    )
    if constraint != 0:
        e.attach(attachment, 0)

    sampler = vamp_module.halton()

    plan_settings.range = combination[0]
    plan_settings.dynamic_domain = combination[1]

    plan_settings.max_iterations = 100000
    plan_settings.descend_rate = 1.0

    plan_settings.radius = 10.0
    plan_settings.num_projection_iterations = combination[3]
    plan_settings.insert_all_to_tree = False
    plan_settings.std_dev_scaling_factor = combination[2]

    if constraint == 0:
        task_constraint = non_transport_constraints
    else:
        task_constraint = transport_constraints

    print(task_constraint.distanceToConstraint(np.array(start)))
    print(task_constraint.distanceToConstraint(np.array(goal)))

    c1 = task_constraint.projectConfiguration(np.array(start), 0, 10.0, 0.75, 50, True)
    c2 = task_constraint.projectConfiguration(np.array(goal), 0, 10.0, 0.75, 50, True)

    # print c1 with commas
    print("c1: ", ", ".join([f"{x:.4f}" for x in c1]))
    print("c2: ", ", ".join([f"{x:.4f}" for x in c2]))


    result = planner_func(start, goal, e, plan_settings, task_constraint, sampler)
    # if constraint == 0:
    #     simple = result
    # else:
    simple = vamp_module.simplify_with_constraints(result.path, e, task_constraint, simp_settings, sampler)

    return result, simple, constraint


if __name__ == '__main__':
    ranges = [1.0]
    # dyndoms = [False, True]
    std_dev_float = [0.3]
    num_projection_iterations = [20]
    ranges = [0.75]
    dyndoms = [False]

    all_combinations = list(itertools.product(ranges, dyndoms, std_dev_float, num_projection_iterations))

    planning_times = {
        combination : [] for combination in all_combinations
    }
    # for _ in range(1):
    #     for combination in all_combinations:
    #         print("Running combination ", combination)
    #         # result, simple = run_planner(rack_2, rack_3, combination, constraint = 2)
    #         # result, simple = run_planner(box_top_shelf_pickup, rack_3, combination, constraint = 2)

    #         # result1, simple1 = run_planner(start, box_top_shelf_pregrasp, combination, constraint = 0)
    #         # simple1.path.interpolate_to_resolution(vamp.digit.resolution())
    #         # traj1 = simple1.path.numpy()

    #         # result2, simple2 = run_planner(box_top_shelf_pregrasp, box_top_shelf_pickup, combination, constraint = 0)
    #         # simple2.path.interpolate_to_resolution(vamp.digit.resolution())
    #         # traj2 = simple2.path.numpy()
    #         result3, simple3, _ = run_planner(box_top_shelf_pickup, rack_2, combination, constraint = 2)
    #         simple3.path.interpolate_to_resolution(vamp.digit.resolution())
    #         traj3 = simple3.path.numpy()
    #         result4, simple4, _ = run_planner(rack_2, rack_3, combination, constraint = 2)
    #         simple4.path.interpolate_to_resolution(vamp.digit.resolution())
    #         traj4 = simple4.path.numpy()
    #         result5, simple5, _ = run_planner(rack_3, box_top_shelf_pickup, combination, constraint = 2)
    #         simple5.path.interpolate_to_resolution(vamp.digit.resolution())
    #         traj5 = simple5.path.numpy()
    #         # done



    #         planning_times[combination].append(result3.nanoseconds/1e6 + result4.nanoseconds/1e6 + result5.nanoseconds/1e6)
    # print("Execution completed")
    # print("Planning times for each combination:")

    # combination_means = []
    # for combination, times in planning_times.items():
    #     print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")
    #     combination_means.append((combination, np.mean(times)))


    # # find the combination with the best mean
    # best_combination = sorted(combination_means, key=lambda x: x[1])[0]
    # print(sorted(combination_means, key=lambda x: x[1]))
    best_combination = [(0.2, False, 0.2, 25)]


    viz.init_viz()
    viz.clear_all_waypoints()


    env_geoms = mjcf_parser.parse_mjcf('resources/environments/cuboids/wooden_shelf.xml')
    env_cuboids = [[geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z, 0, 0, 0, geom.size.x * 2.0, geom.size.y * 2.0, geom.size.z*2.0] for geom in env_geoms if geom.type == mjcf_parser.GeomType.BOX]
    viz.add_cuboids(env_cuboids, colors=(90, 60, 0))

    attach_sphere_w_r = [[x, y, z, 0.02] for x, y, z in attach_spheres]
    viz.set_attach_object_to_robot(attach_sphere_w_r)
    for geom in env_geoms:
        if geom.type == mjcf_parser.GeomType.BOX:
            viz.add_cuboids([
                [geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z, 0, 0, 0, geom.size.x * 2.0, geom.size.y * 2.0, geom.size.z*2.0]
            ], colors=(150, 75, 0))


    results_and_simples_and_constraints = []
    # results_and_simples_and_constraints.append(run_planner(rack_2_release, rack_2_release, best_combination[0], constraint = 0))



    # results_and_simples_and_constraints.append(run_planner(start, box_top_shelf_pregrasp, best_combination[0], constraint = 0))
    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pregrasp, box_top_shelf_pickup, best_combination[0], constraint = 0))
    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pickup, rack_2, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(rack_2, rack_3, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(rack_3, box_top_shelf_pickup, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pickup, box_top_shelf_pregrasp, best_combination[0], constraint = 0))

    results_and_simples_and_constraints.append(run_planner(start, rack_2_release, best_combination[0], constraint = 0))
    # results_and_simples_and_constraints.append(run_planner(rack_2_release, rack_2_release, best_combination[0], constraint = 0))
    # results_and_simples_and_constraints.append(run_planner(rack_2, rack_2, best_combination[0], constraint = 0))
    results_and_simples_and_constraints.append(run_planner(rack_2, box_top_shelf_pickup, best_combination[0], constraint = 2))
    results_and_simples_and_constraints.append(run_planner(box_top_shelf_pickup, rack_2, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(rack_2, rack_2_release,  best_combination[0], constraint = 0))
    results_and_simples_and_constraints.append(run_planner(rack_2_release, start, best_combination[0], constraint = 0))

    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pregrasp, box_top_shelf_pickup, best_combination[0], constraint = 0))
    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pickup, rack_2, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(rack_2, rack_3, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(rack_3, box_top_shelf_pickup, best_combination[0], constraint = 2))
    # results_and_simples_and_constraints.append(run_planner(box_top_shelf_pickup, box_top_shelf_pregrasp, best_combination[0], constraint = 0))


    # simple.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj1 = simple.path.numpy()



    # best_combination = [(0.5, False, 0.3, 10)]
    # result1, simple1 = run_planner(start, box_top_shelf_pregrasp, best_combination[0], constraint = 0)
    # simple1.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj1 = simple1.path.numpy()

    # result2, simple2 = run_planner(box_top_shelf_pregrasp, box_top_shelf_pickup, best_combination[0], constraint = 0)
    # simple2.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj2 = simple2.path.numpy()

    # result3, simple3 = run_planner(box_top_shelf_pickup, rack_2, best_combination[0], constraint = 2)
    # simple3.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj3 = simple3.path.numpy()

    # # result4, simple4 = run_planner(rack_2, rack_2_release, best_combination[0], constraint = 0)
    # # simple4.path.interpolate_to_resolution(vamp.digit.resolution())
    # # traj4 = simple4.path.numpy()


    # # result5, simple5 = run_planner(rack_2_release, rack_2, best_combination[0], constraint = 0)
    # # simple5.path.interpolate_to_resolution(vamp.digit.resolution())
    # # traj5 = simple5.path.numpy()


    # result4, simple4 = run_planner(rack_2, rack_3, best_combination[0], constraint = 2)
    # simple4.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj4 = simple4.path.numpy()

    # # result7, simple7 = run_planner(rack_3, rack_3_release, best_combination[0], constraint = 0)
    # # simple7.path.interpolate_to_resolution(vamp.digit.resolution())
    # # traj7 = simple7.path.numpy()

    # # result7, simple7 = run_planner(rack_3, rack_3_release, best_combination[0], constraint = 0)
    # # simple7.path.interpolate_to_resolution(vamp.digit.resolution())
    # # traj7 = simple7.path.numpy()



    # result5, simple5 = run_planner(rack_3, box_top_shelf_pickup, best_combination[0], constraint = 2)
    # simple5.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj5 = simple5.path.numpy()

    # result6, simple6 = run_planner(box_top_shelf_pickup, box_top_shelf_pregrasp, best_combination[0], constraint = 0)
    # simple6.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj6 = simple6.path.numpy()


    # now combine the trajectories

    trajs = []
    attachment_masks_list = []


    for indx, (result, simple, constraint) in enumerate(results_and_simples_and_constraints):
        if indx == 1:
            # interpolate 10 points between rack_2_release and rack_2
            numpy_path = np.linspace(rack_2_release, rack_2, num=10)
            trajs.append(numpy_path)
            attachment_masks_list.append(np.zeros(len(numpy_path)))
        elif indx == 3:
            # interpolate 10 points between rack_2 and rack_2_release
            numpy_path = np.linspace(rack_2, rack_2_release, num=10)
            trajs.append(numpy_path)
            attachment_masks_list.append(np.zeros(len(numpy_path)))

        simple.path.interpolate_to_resolution(vamp.digit.resolution())
        trajs.append(simple.path.numpy())
        attachment_masks_list.append(np.ones(len(simple.path.numpy())) * (constraint > 0))

    final_traj = np.concatenate(trajs, axis=0)
    attachment_masks = np.concatenate(attachment_masks_list, axis=0)

    # final_traj = np.concatenate((traj1, traj2, traj3, traj4, traj5, traj6), axis=0)
    # attachment_masks = np.concatenate((np.zeros(len(traj1)), np.zeros(len(traj2)), np.ones(len(traj3)), np.ones(len(traj4)), np.ones(len(traj5)), np.ones(len(traj6))), axis=0)

    # print(final_traj.shape)
    # stop

    # final_traj = traj1


    full_waypoints = np.array(final_traj)
    waypoints_no_floating = full_waypoints[:, 6:]
    final_waypoints = np.zeros((len(waypoints_no_floating), 30))
    final_waypoints[:, :6] = waypoints_no_floating[:, :6]
    final_waypoints[:, 9:21] = waypoints_no_floating[:, 6:18]
    final_waypoints[:, 24:30] = waypoints_no_floating[:, 18:24]
    np.savetxt('shelf_top_to_down.txt', final_waypoints, fmt='%.4f', delimiter = ',')

    print("Final time : ", sum([result.nanoseconds/1e6 for result, _, _ in results_and_simples_and_constraints]))

    # print("Final time: ", result1.nanoseconds/1e6 + result2.nanoseconds/1e6 + result3.nanoseconds/1e6 + result4.nanoseconds/1e6 + result5.nanoseconds/1e6)
    # print(result1.nanoseconds/1e6, result2.nanoseconds/1e6, result3.nanoseconds/1e6, result4.nanoseconds/1e6, result5.nanoseconds/1e6, best_combination[0])


    eef_poses = get_eef_of_waypoints(final_traj)
    controller_task_space_traj = convert_trajectory_eef_to_controller_format(final_traj, eef_poses)
    np.savetxt('shelf_top_to_down_controller_format.txt', controller_task_space_traj, fmt='%.4f', delimiter = ',')


    if len(final_traj) > 0:
        # simple.path.interpolate_to_resolution(vamp.digit.resolution())

        # traj = simple.path.numpy()
        eef_poses = get_eef_of_waypoints(final_traj)
        print(eef_poses[:, 0, 2, 3])

        # find norm between positions of both hands for each waypoint
        hand_distances = np.linalg.norm(eef_poses[:, 0, :3, 3] - eef_poses[:, 1, :3, 3], axis=1)
        # print("min hand distances:", np.min(hand_distances[attachment_masks==1]))
        transport_hand_distances = hand_distances[attachment_masks==1]
        # plot the transport_hand_distances
        plt.figure()
        plt.plot(eef_poses[:, 0, 2, 3])
        plt.title("Distance between hands during transport")
        plt.xlabel("Waypoint index")
        plt.ylabel("Distance (m)")
        plt.savefig("hand_distances.png")


        # for viz flatten the first two dimensions so it's just a list of eef poses
        viz.render_eefs(eef_poses.reshape(-1, 4, 4))
        viz.animate(final_traj, np.arange(0, len(final_traj), dtype=np.float64) / 10, attachment_spheres=attach_sphere_w_r, attachment_masks=attachment_masks, loop=True)


        while True:
            pass
