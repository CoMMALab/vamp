"""Replay a TaskSpaceRRTC trace (written by scripts/cpp/task_space_rrtc_cage.cc) in viser.

    python scripts/task_space_trace_viz.py --trace=task_space_trace --port=8080

A slider steps through every event the planner logged, in order:

  Root           the start node
  Sample         a random configuration drawn for exploration
  Extend         a node added by the exploration step (from the nearest node)
  ExtendFailed   the exploration step was blocked (nearest node -> sample)
  WalkStart      a goal-directed walk begins at a node
  WalkLm         the unclamped LM endpoint proposed from the walk's current node
  WalkProposal   the clamped step tried from the current node, with its outcome
  Solved         a node satisfies the task-space region

Two robots are drawn: A (blue) is the node the event starts from and B (orange) is the
configuration the event produced or tried. The tree is drawn in end-effector space (a segment
from each node's end-effector position to its parent's); samples, the path from the current
node to the root, and the goal frame can be toggled.
"""

import json
import time
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from fire import Fire
from viser import transforms as vtf
from viser.extras import ViserUrdf

import vamp

EVENTS = ["Root", "Sample", "Extend", "ExtendFailed", "WalkStart", "WalkLm", "WalkProposal", "Solved"]
OUTCOMES = ["", "Accepted", "ZeroStep", "NoImprovement", "OutOfBounds", "Collision", "TreeFull"]
ROOT, SAMPLE, EXTEND, EXTEND_FAILED, WALK_START, WALK_LM, WALK_PROPOSAL, SOLVED = range(8)
ACCEPTED = 1

MAGIC = b"TSRTRACE"
EXTEND_COLOR = np.array([70, 110, 220], dtype=np.uint8)
WALK_COLOR = np.array([60, 190, 90], dtype=np.uint8)
OUTCOME_COLORS = {
    "Accepted": (60, 190, 90),
    "ZeroStep": (150, 150, 150),
    "NoImprovement": (230, 190, 40),
    "OutOfBounds": (200, 90, 200),
    "Collision": (220, 60, 60),
    "TreeFull": (150, 150, 150),
    "": (230, 130, 40),
}


def format_config(cfg):
    """A configuration as a Python list literal, ready to paste into a script."""
    return "" if cfg is None else "[" + ", ".join(f"{float(x):.9g}" for x in cfg) + "]"


def load_trace(prefix):
    """Returns (problem dict, structured array of trace entries)."""
    prefix = str(prefix)
    problem = json.loads(Path(prefix + ".json").read_text())
    raw = Path(prefix + ".bin").read_bytes()
    if raw[:8] != MAGIC:
        raise ValueError(f"{prefix}.bin is not a TaskSpaceRRTC trace")

    version, dimension = np.frombuffer(raw, dtype="<u4", count=2, offset=8)
    count = int(np.frombuffer(raw, dtype="<u8", count=1, offset=16)[0])
    if version != 1:
        raise ValueError(f"unsupported trace version {version}")

    entry = np.dtype(
        [
            ("event", "u1"),
            ("outcome", "u1"),
            ("iteration", "<u4"),
            ("node", "<i4"),
            ("parent", "<i4"),
            ("error", "<f4"),
            ("reference_error", "<f4"),
            ("configuration", "<f4", (int(dimension), )),
        ]
    )
    entries = np.frombuffer(raw, dtype=entry, count=count, offset=24)
    return problem, entries


class Trace:
    """The planner's tree rebuilt from the trace, plus per-event lookups."""

    def __init__(self, problem, entries, eefk):
        self.problem = problem
        self.entries = entries

        creating = np.flatnonzero(
            (entries["event"] == ROOT) | (entries["event"] == EXTEND) |
            ((entries["event"] == WALK_PROPOSAL) & (entries["outcome"] == ACCEPTED))
        )
        n_nodes = len(creating)
        self.node_entry = creating  # entry index at which node k was created
        self.node_config = entries["configuration"][creating]
        self.node_parent = entries["parent"][creating].copy()
        self.node_parent[0] = -1
        self.node_walk = entries["event"][creating] == WALK_PROPOSAL
        assert (entries["node"][creating] == np.arange(n_nodes)).all(), "nodes are not sequential"

        self.node_ee = np.array([eefk(c)[:3, 3] for c in self.node_config], dtype=np.float32)

        sample_entries = np.flatnonzero(entries["event"] == SAMPLE)
        self.sample_entry = sample_entries
        self.sample_ee = np.array([eefk(c)[:3, 3] for c in entries["configuration"][sample_entries]], dtype=np.float32)

        # Segment k-1 joins node k to its parent.
        child = np.arange(1, n_nodes)
        self.segments = np.stack([self.node_ee[child], self.node_ee[self.node_parent[1:]]], axis=1)
        self.segment_colors = np.where(self.node_walk[1:, None], WALK_COLOR, EXTEND_COLOR)
        self.segment_colors = np.repeat(self.segment_colors[:, None, :], 2, axis=1)

    def __len__(self):
        return len(self.entries)

    def n_nodes_at(self, i):
        """Nodes that exist once entry i has been applied."""
        return int(np.searchsorted(self.node_entry, i, side="right"))

    def n_samples_at(self, i):
        return int(np.searchsorted(self.sample_entry, i, side="right"))

    def path_to_root(self, node):
        chain = [int(node)]
        while self.node_parent[chain[-1]] >= 0:
            chain.append(int(self.node_parent[chain[-1]]))

        return chain

    def focus(self, i):
        """(config A, config B, description) for entry i; either config may be None."""
        e = self.entries[i]
        event = int(e["event"])
        cfg = e["configuration"]
        parent_cfg = self.node_config[e["parent"]] if e["parent"] >= 0 else None
        if event == ROOT:
            return cfg, None, "start node"
        if event == SAMPLE:
            nxt = self.entries[i + 1] if i + 1 < len(self.entries) else None
            near = None
            if nxt is not None and int(nxt["event"]) in (EXTEND, EXTEND_FAILED) and nxt["parent"] >= 0:
                near = self.node_config[nxt["parent"]]
            return near, cfg, "random sample (A = nearest node)"
        if event == EXTEND:
            return parent_cfg, cfg, "exploration step added node"
        if event == EXTEND_FAILED:
            return parent_cfg, cfg, "exploration step blocked (B = the sample)"
        if event == WALK_START:
            return self.node_config[e["node"]], None, "goal-directed walk starts"
        if event == WALK_LM:
            return parent_cfg, cfg, "unclamped LM endpoint (B)"
        if event == WALK_PROPOSAL:
            return parent_cfg, cfg, "clamped step: " + OUTCOMES[int(e["outcome"])]
        return self.node_config[e["node"]], None, "task-space region reached"

    def describe(self, i):
        e = self.entries[i]
        return {
            "event": f"{i}/{len(self) - 1}  {EVENTS[int(e['event'])]}",
            "iteration": str(int(e["iteration"])),
            "node / parent": f"{int(e['node'])} / {int(e['parent'])}",
            "error (sq)": "-" if np.isnan(e["error"]) else f"{float(e['error']):.5f}",
            "error at step start": "-" if np.isnan(e["reference_error"]) else f"{float(e['reference_error']):.5f}",
            "outcome": OUTCOMES[int(e["outcome"])],
        }


class TraceViewer:

    def __init__(self, trace, server, urdf_path, eefk):
        self.trace = trace
        self.server = server
        self.eefk = eefk
        self.shown_nodes = -1
        self.shown_samples = -1
        self.shown_a = self.shown_b = None

        urdf = yourdfpy.URDF.load(str(urdf_path))
        kwargs = dict(load_meshes=True, load_collision_meshes=False)
        self.robot_a = ViserUrdf(server, urdf, root_node_name="/robot_a", mesh_color_override=(0.25, 0.5, 1.0), **kwargs)
        self.robot_b = ViserUrdf(server, urdf, root_node_name="/robot_b", mesh_color_override=(1.0, 0.55, 0.15), **kwargs)
        # The configuration whose end-effector pose is the goal (the planner never sees it): faded green.
        self.robot_goal = ViserUrdf(
            server, urdf, root_node_name="/robot_goal", mesh_color_override=(0.2, 0.8, 0.3, 0.25), **kwargs
        )
        self.robot_goal.update_cfg(np.asarray(trace.problem["goal_configuration"], dtype=np.float64))

        problem = trace.problem
        for k, box in enumerate(problem["boxes"]):
            rho, theta, phi = (float(angle) for angle in box["euler_xyz"])
            server.scene.add_box(
                f"/obstacles/{k}",
                color=(120, 120, 120),
                dimensions=tuple(2 * np.asarray(box["half_extents"])),
                position=tuple(box["center"]),
                wxyz=vtf.SO3.from_rpy_radians(rho, theta, phi).wxyz,
                opacity=0.35,
            )

        pose = problem["goal_pose_wxyz_xyz"]
        server.scene.add_frame("/goal_pose", wxyz=tuple(pose[:4]), position=tuple(pose[4:]), axes_length=0.12, axes_radius=0.006)

        self._build_gui()
        self.show(0)

    def _build_gui(self):
        gui = self.server.gui
        n = len(self.trace)
        self.slider = gui.add_slider("Event", min=0, max=n - 1, step=1, initial_value=0)
        self.fields = {k: gui.add_text(k, initial_value="", disabled=True) for k in self.trace.describe(0)}
        self.note = gui.add_text("What", initial_value="", disabled=True)
        # Select the text to copy it (viser has no server-side clipboard); edits are ignored.
        self.config_a = gui.add_text("Config A (blue)", initial_value="", multiline=True)
        self.config_b = gui.add_text("Config B (orange)", initial_value="", multiline=True)
        print_a, print_b = gui.add_button("Print config A to terminal"), gui.add_button("Print config B to terminal")
        self.show_tree = gui.add_checkbox("Tree (end-effector space)", True)
        self.show_samples = gui.add_checkbox("Samples (end-effector space)", False)
        self.show_path = gui.add_checkbox("Path from current node to root", True)
        self.show_goal = gui.add_checkbox("Goal configuration (faded green)", True)
        self.playing = gui.add_checkbox("Play", False)
        self.speed = gui.add_slider("Events per second", min=1, max=200, step=1, initial_value=20)

        prev, nxt = gui.add_button("Previous"), gui.add_button("Next")
        nxt_walk = gui.add_button("Next walk start")
        nxt_fail = gui.add_button("Next rejected step")

        @print_a.on_click
        def _(_):
            print(f"event {int(self.slider.value)} config A: {format_config(self.shown_a)}")

        @print_b.on_click
        def _(_):
            print(f"event {int(self.slider.value)} config B: {format_config(self.shown_b)}")

        @self.slider.on_update
        def _(_):
            self.show(int(self.slider.value))

        @prev.on_click
        def _(_):
            self.slider.value = max(0, int(self.slider.value) - 1)

        @nxt.on_click
        def _(_):
            self.slider.value = min(n - 1, int(self.slider.value) + 1)

        def jump(mask):
            idx = np.flatnonzero(mask)
            idx = idx[idx > int(self.slider.value)]
            if len(idx):
                self.slider.value = int(idx[0])

        @nxt_walk.on_click
        def _(_):
            jump(self.trace.entries["event"] == WALK_START)

        @nxt_fail.on_click
        def _(_):
            e = self.trace.entries
            jump((e["event"] == WALK_PROPOSAL) & (e["outcome"] != ACCEPTED))

        @self.show_goal.on_update
        def _(_):
            self.robot_goal.show_visual = bool(self.show_goal.value)

        for handle in (self.show_tree, self.show_samples, self.show_path):

            @handle.on_update
            def _(_):
                self.shown_nodes = self.shown_samples = -1
                self.show(int(self.slider.value))

    def show(self, i):
        t = self.trace
        a, b, note = t.focus(i)
        self.shown_a, self.shown_b = a, b
        self.config_a.value = format_config(a)
        self.config_b.value = format_config(b)
        for robot, cfg in ((self.robot_a, a), (self.robot_b, b)):
            robot.show_visual = cfg is not None
            if cfg is not None:
                robot.update_cfg(np.asarray(cfg, dtype=np.float64))

        for key, text in t.describe(i).items():
            self.fields[key].value = text
        self.note.value = note

        scene = self.server.scene
        n_nodes = t.n_nodes_at(i)
        if n_nodes != self.shown_nodes:
            self.shown_nodes = n_nodes
            if self.show_tree.value and n_nodes > 1:
                scene.add_line_segments("/tree", points=t.segments[:n_nodes - 1], colors=t.segment_colors[:n_nodes - 1], thickness=0.002)
            else:
                scene.add_line_segments("/tree", points=np.zeros((1, 2, 3)), colors=(0, 0, 0), thickness=0.001, visible=False)

        n_samples = t.n_samples_at(i)
        if n_samples != self.shown_samples:
            self.shown_samples = n_samples
            if self.show_samples.value and n_samples > 0:
                scene.add_point_cloud("/samples", points=t.sample_ee[:n_samples], colors=(160, 160, 160), point_size=0.006)
            else:
                scene.add_point_cloud("/samples", points=np.zeros((1, 3)), colors=(0, 0, 0), point_size=0.001, visible=False)

        # The step under inspection: end effector at A -> end effector at B.
        if a is not None and b is not None:
            color = OUTCOME_COLORS[OUTCOMES[int(t.entries[i]["outcome"])]]
            ends = np.array([[self.eefk(a)[:3, 3], self.eefk(b)[:3, 3]]], dtype=np.float32)
            scene.add_line_segments("/step", points=ends, colors=color, thickness=0.01)
        else:
            scene.add_line_segments("/step", points=np.zeros((1, 2, 3)), colors=(0, 0, 0), thickness=0.001, visible=False)

        node = int(t.entries[i]["node"])
        if node < 0:
            node = int(t.entries[i]["parent"])
        if self.show_path.value and node >= 0:
            chain = t.path_to_root(node)
            if len(chain) > 1:
                pts = t.node_ee[chain]
                scene.add_line_segments("/path", points=np.stack([pts[:-1], pts[1:]], axis=1), colors=(250, 220, 60), thickness=0.006)
            else:
                scene.add_line_segments("/path", points=np.zeros((1, 2, 3)), colors=(0, 0, 0), thickness=0.001, visible=False)
        else:
            scene.add_line_segments("/path", points=np.zeros((1, 2, 3)), colors=(0, 0, 0), thickness=0.001, visible=False)

    def run(self):
        last = time.time()
        while True:
            time.sleep(0.01)
            if self.playing.value and time.time() - last >= 1.0 / self.speed.value:
                last = time.time()
                nxt = int(self.slider.value) + 1
                if nxt < len(self.trace):
                    self.slider.value = nxt
                else:
                    self.playing.value = False


def main(trace: str = "task_space_trace", port: int = 8080):
    problem, entries = load_trace(trace)
    robot = problem["robot"]
    module = getattr(vamp, robot)
    eefk = module.eefk

    print(f"{len(entries)} trace entries; building tree and end-effector positions...")
    parsed = Trace(problem, entries, eefk)
    print(f"{len(parsed.node_config)} nodes, {len(parsed.sample_entry)} samples; result: {problem['result']}")

    robot_dir = Path(__file__).parents[1] / "resources" / robot
    urdf = robot_dir / f"{robot}_spherized.urdf"
    if not urdf.exists():
        urdf = robot_dir / f"{robot}.urdf"

    server = viser.ViserServer(port=port)
    viewer = TraceViewer(parsed, server, urdf, eefk)
    print(f"open http://localhost:{port}")
    viewer.run()


if __name__ == "__main__":
    Fire(main)
