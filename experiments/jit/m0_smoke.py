"""M0 gate: JIT load_robot works for Panda + UR5, and end-to-end planning runs.

Also records the clang `load_robot` compile latency -- the "clang" specialization
cost bar that T1/T2 must beat (design doc §13 action 2, E2 baseline).
"""
import pickle
import time
import json
from pathlib import Path

import cricket
import vamp

ROBOTS = {
    "panda": dict(urdf="panda_spherized.urdf", srdf="panda.srdf",
                  end_effector="panda_grasptarget", name="Panda"),
    "ur5": dict(urdf="ur5_spherized.urdf", srdf="ur5.srdf",
                end_effector="robotiq_85_base_link", name="UR5"),
}

res = cricket.resources_dir()
out = {}

for robot, cfg in ROBOTS.items():
    base = res / robot
    urdf, srdf = base / cfg["urdf"], base / cfg["srdf"]
    assert urdf.exists(), f"missing {urdf}"
    print(f"[{robot}] load_robot from {urdf.name} ...", flush=True)
    t0 = time.perf_counter()
    jit_robot = vamp.jit.load_robot(
        urdf=str(urdf), srdf=str(srdf), end_effector=cfg["end_effector"],
        planners=["rrtc"], rake=8, resolution=32, name=cfg["name"],
    )
    t1 = time.perf_counter()
    load_s = t1 - t0
    out[robot] = {"load_robot_s": load_s}
    print(f"[{robot}] load_robot took {load_s:.3f} s", flush=True)

    # Solve a handful of MBM problems to confirm end-to-end.
    ppath = base / "problems.pkl"
    if not ppath.exists():
        print(f"[{robot}] no problems.pkl, skipping solve", flush=True)
        continue
    with open(ppath, "rb") as f:
        problems = pickle.load(f)

    settings = vamp.RRTCSettings()
    if robot in vamp.ROBOT_RRT_RANGES:
        settings.range = vamp.ROBOT_RRT_RANGES[robot]
    settings.max_iterations = 1_000_000
    settings.max_samples = 1_000_000

    solved = tried = 0
    times_us = []
    for name, pset in problems["problems"].items():
        for data in pset:
            if not data.get("valid", False):
                continue
            env = vamp.problem_dict_to_vamp(data)
            start = list(data["start"])
            goals = list(data["goals"])
            sampler = jit_robot.halton()
            r = jit_robot.rrtc(start, goals, env, settings, sampler)
            tried += 1
            if r.solved:
                solved += 1
                times_us.append(r.nanoseconds / 1000.0)
            if tried >= 10:
                break
        if tried >= 10:
            break
    med = sorted(times_us)[len(times_us)//2] if times_us else None
    out[robot].update(solved=solved, tried=tried,
                      median_plan_us=med)
    print(f"[{robot}] solved {solved}/{tried}, median plan {med} us", flush=True)

Path(__file__).parent.joinpath("results", "m0_smoke.json").write_text(json.dumps(out, indent=2))
print("\nSUMMARY:", json.dumps(out, indent=2))
