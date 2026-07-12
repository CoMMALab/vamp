DEFAULT_ITERATIONS = 1000000

ROBOT_RRT_RANGES = {
    "sphere": 1,
    "ur5": 1.5,
    "ur5_flask": 1.5,
    "panda": 1.0,
    "panda_flask": 1.5,
    "fetch": 1.0,
    "fetch_flask": 1.0,
    "baxter": 0.5,
    "digit": 0.75,
    }

# Per-robot RRTC dynamic-domain radius overrides. z-space distance in flask robots easily
# blows past the C++ default (4.0), so dynamic-domain rejects most samples once the tree
# has grown -- iteration count and wall time balloon. Bump the radius per robot. On Panda
# MBM this gives 4x planning speedup (3.74 -> 0.91 ms) at ~2% cost regression; on fetch it
# lifts solve rate from 39% to 71% at fixed budget.
ROBOT_RRTC_DD_RADIUS = {
    "panda_flask": 8.0,
    "fetch_flask": 8.0,
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
