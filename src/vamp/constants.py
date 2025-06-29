DEFAULT_ITERATIONS = 1000000

ROBOT_JOINTS = {
    "ur5": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        ],
    "panda": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        ],
    "fetch": [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
        ],
    "baxter": [
        "right_s0",
        "right_s1",
        "right_e0",
        "right_e1",
        "right_w0",
        "right_w1",
        "right_w2",
        "left_s0",
        "left_s1",
        "left_e0",
        "left_e1",
        "left_w0",
        "left_w1",
        "left_w2",
        ],
    "motoman": [
        "torso_joint_b1",
        "arm_left_joint_1_s",
        "arm_left_joint_2_l",
        "arm_left_joint_3_e",
        "arm_left_joint_4_u",
        "arm_left_joint_5_r",
        "arm_left_joint_6_b",
        "arm_left_joint_7_t",
        "arm_right_joint_1_s",
        "arm_right_joint_2_l",
        "arm_right_joint_3_e",
        "arm_right_joint_4_u",
        "arm_right_joint_5_r",
        "arm_right_joint_6_b",
        "arm_right_joint_7_t",
        "torso_joint_b2"
        ]
    }

ROBOT_RRT_RANGES = {
    "sphere": 1,
    "ur5": 1.5,
    "panda": 1.0,
    "fetch": 1.0,
    "baxter": 0.5,
    "motoman": 0.5,
    }

ROBOT_RADII_RANGES = {
    "baxter": (0.012, 0.08),
    "fetch": (0.012, 0.055),
    "panda": (0.012, 0.06),
    "sphere": (0.2, 0.2),
    "ur5": (0.015, 0.08),
    "motoman": (0.034, 0.344),
    }

ROBOT_FIRST_JOINT_LOCATIONS = {
    "fetch": [0.0, 0.0, 0.4],
    "ur5": [0.0, 0.0, 0.91],
    "panda": [0.0, 0.0, 0.0],
    }

ROBOT_MAX_RADII = {
    "ur5": 1.2,
    "fetch": 1.5,
    "panda": 1.19,
    }

POINT_RADIUS = 0.0025
