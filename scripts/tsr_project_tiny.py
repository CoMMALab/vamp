import numpy as np
import vamp

# Goal pose (qw, qx, qy, qz, x, y, z) of the cage problem's goal configuration, as an exact TSR.
POSE = [0.0271051731, 0.705981255, -0.0272457302, 0.707187057, 0.717607677, -0.155613914, 0.283716708]
tsr = vamp.panda.TaskSpaceConstraint([1, 0, 0, 0, 0, 0, 0], POSE, [0.0] * 6, [0.0] * 6)

# Node 4 of the trace (the blue config).
q = np.array([0.152289048, -0.706145108, -0.666280568, -2.48238468, -0.949631512, 1.02204382, -0.607726097], dtype=np.float32)

settings = vamp.ConstraintSettings()
error = lambda c: vamp.panda.squared_error(np.asarray(c, dtype=np.float32), [tsr], settings)

orange = vamp.panda.step(q, [tsr], settings)  # one raw LM step (damping 1e-6, no length limit)
print("blue error  :", error(q))
print("orange      :", orange.tolist())
print("orange error:", error(orange), "| step length", np.linalg.norm(orange - q))

# ---- one step with eight dampings (one per SIMD lane in the planner), step capped at RANGE ----
RANGE = 1.0
w, x, y, z = POSE[:4]
R_goal = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def error_vector(c):  # (position error in the goal frame, rotation vector): what the TSR error is made of
    T = np.asarray(vamp.panda.eefk(np.asarray(c, dtype=np.float32)), dtype=np.float64)
    R = R_goal.T @ T[:3, :3]
    half = np.sqrt(max(0.0, 1 + np.trace(R))) / 2
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (4 * half)
    return np.concatenate([R_goal.T @ (T[:3, 3] - POSE[4:]), 2 * np.arctan2(np.linalg.norm(v), half) * v / np.linalg.norm(v)])


e0 = error_vector(q)
J = np.stack([(error_vector(q + 1e-5 * np.eye(7)[k]) - e0) / 1e-5 for k in range(7)], axis=1)
lo, hi = np.asarray(vamp.panda.lower_bounds()), np.asarray(vamp.panda.upper_bounds())

candidates = []
for damping in 10.0 ** np.arange(-6, 2):
    step = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), e0)
    step *= min(1.0, RANGE / np.linalg.norm(step))
    candidates.append(np.clip(q - step, lo, hi))
    print(f"damping {damping:7.0e}: step length {np.linalg.norm(step):.3f}, error {error(candidates[-1]):.4f}")

best = candidates[int(np.argmin([error(c) for c in candidates]))]
print("best of 8   :", best.tolist(), "| error", error(best))
