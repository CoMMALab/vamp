"""Viser viewer for fr3_marker maze results: renders the FR3 arm and the maze cuboids,
and lets you scrub through any solved problem's shortcut trajectory with a joint slider.

Loads <trajectory_dir>/problem_<n>.txt trajectory files -- one waypoint per line,
comma-separated joint values -- the format both scripts/fr3_marker_maze_example.py and
scripts/cpp/fr3_maze_solver_benchmark.cc's write_ambient_path write, one problem at a
time. Pick which problem to view with the "Problem" slider; "Current Waypoint" scrubs
through that problem's trajectory, and "Autoplay" steps through it automatically.

Usage:
    python scripts/fr3_marker_maze_viser.py
    python scripts/fr3_marker_maze_viser.py --trajectory_dir /tmp/quick_check
"""

import json
import re
import time
from pathlib import Path

import numpy as np
from viser import transforms as tf
from viser_utils import setup_viser_with_robot
from fire import Fire

RESOURCES = Path(__file__).parents[1] / "resources"
MAZE_JSON = RESOURCES / "environments" / "maze_cuboids.json"
DEFAULT_TRAJECTORY_DIR = RESOURCES / "fr3_marker" / "maze_solver_benchmark_trajectories_python"
FR3_MARKER_DIR = RESOURCES / "fr3_marker"
FR3_MARKER_URDF = "fr3_expo_spherized.urdf"
TRAJECTORY_FILENAME_RE = re.compile(r"problem_(\d+)\.txt")


def add_maze_cuboids(server) -> None:
    """Render the maze walls as boxes, with the same offsets
    scripts/fr3_marker_maze_example.py's load_maze_environment applies when building the
    collision environment, so the rendered walls line up with what the planner saw."""
    with open(MAZE_JSON) as f:
        cuboids = json.load(f)

    for i, c in enumerate(cuboids):
        position = (c["x"] + 0.285 * 2, c["y"], c["z"] + 0.05)
        dimensions = (c["dx"] + 0.01, c["dy"] + 0.01, c["dz"] + 0.01)
        wxyz = tf.SO3.from_rpy_radians(
            c.get("roll", 0.0), c.get("pitch", 0.0), c.get("yaw", 0.0)).wxyz
        server.scene.add_box(
            f"/maze/cuboid_{i}",
            color=(120, 120, 130),
            dimensions=dimensions,
            wxyz=wxyz,
            position=position,
            opacity=0.9,
        )


def load_trajectory(path: Path) -> np.ndarray:
    """One waypoint per line, comma-separated joint values -- matches
    fr3_maze_solver_benchmark.cc's write_ambient_path and
    fr3_marker_maze_example.py's write_ambient_path."""
    with open(path) as f:
        rows = [line.split(",") for line in f if line.strip()]
    return np.asarray(rows, dtype=np.float32)


def find_trajectory_files(trajectory_dir: Path) -> list:
    """problem_<n>.txt files in trajectory_dir, sorted by their numeric index."""
    matches = []
    for path in trajectory_dir.glob("problem_*.txt"):
        m = TRAJECTORY_FILENAME_RE.fullmatch(path.name)
        if m:
            matches.append((int(m.group(1)), path))
    matches.sort(key=lambda pair: pair[0])
    return matches


def main(trajectory_dir: str = str(DEFAULT_TRAJECTORY_DIR)):
    trajectory_dir = Path(trajectory_dir)
    trajectory_files = find_trajectory_files(trajectory_dir)
    if not trajectory_files:
        raise ValueError(f"No problem_<n>.txt trajectory files found in {trajectory_dir}")
    print(f"Loaded {len(trajectory_files)} solved problems from {trajectory_dir}")

    server, robot = setup_viser_with_robot(FR3_MARKER_DIR, FR3_MARKER_URDF)
    add_maze_cuboids(server)

    trajectory = load_trajectory(trajectory_files[0][1])

    def show(q):
        robot.update_cfg(np.asarray(q, dtype=np.float32))

    show(trajectory[0])

    problem_slider = server.gui.add_slider(
        "Problem", min=0, max=len(trajectory_files) - 1, step=1, initial_value=0)
    waypoint_slider = server.gui.add_slider(
        "Current Waypoint", min=0, max=len(trajectory) - 1, step=1, initial_value=0)
    autoplay = server.gui.add_checkbox("Autoplay", initial_value=True)
    rate = server.gui.add_slider(
        "Playback Rate (Hz)", min=1.0, max=60.0, step=1.0, initial_value=20.0)

    @problem_slider.on_update
    def _(event):
        nonlocal trajectory
        problem_index, path = trajectory_files[int(event.target.value)]
        trajectory = load_trajectory(path)
        waypoint_slider.max = len(trajectory) - 1
        waypoint_slider.value = 0
        show(trajectory[0])
        print(f"Problem {problem_index}: {len(trajectory)} waypoints")

    @waypoint_slider.on_update
    def _(event):
        show(trajectory[int(event.target.value)])

    print("visualization at http://localhost:8080; ctrl-c to exit")
    while True:
        if autoplay.value:
            waypoint_slider.value = (waypoint_slider.value + 1) % len(trajectory)
        time.sleep(1.0 / rate.value)


if __name__ == "__main__":
    Fire(main)
