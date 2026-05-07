#!/usr/bin/env python3
"""
Visualize incremental bidirectional tree growth in 3D using end-effector positions.

Usage:
    python visualize_tree_growth.py <tree_growth.json> [robot_type] [output_gif] [max_iteration] [play_mode] [autoplay_interval_ms] [autoplay_end_hold_frames]

Examples:
    python visualize_tree_growth.py tree_growth_data.json
    python visualize_tree_growth.py tree_growth_data.json panda growth.gif
    python visualize_tree_growth.py tree_growth_data.json panda growth.gif 80
    python visualize_tree_growth.py tree_growth_data.json panda None 80 autoplay
    python visualize_tree_growth.py tree_growth_data.json panda None 80 autoplay 400
    python visualize_tree_growth.py tree_growth_data.json panda None 80 autoplay 400 10
"""

import json
import sys
import matplotlib
matplotlib.use('TkAgg') # Set the backend FIRST
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

try:
    import vamp
except ImportError:
    print("Error: vamp module not found. Make sure it's installed.")
    sys.exit(1)


def load_tree_growth(filepath):
    """Load tree growth data from JSON file."""
    with open(filepath, 'r') as file_handle:
        return json.load(file_handle)


def get_robot_module(robot_type):
    """Get the robot submodule from vamp."""
    robot_map = {
        'panda': vamp.panda,
    }

    if robot_type not in robot_map:
        raise ValueError(f"Unknown robot type: {robot_type}. Available: {list(robot_map.keys())}")

    return robot_map[robot_type]


def config_to_eef_xyz(robot_module, config):
    """Convert a configuration to end-effector xyz position."""
    if config is None:
        return None

    try:
        config_np = np.asarray(config, dtype=np.float32)
        transforms = robot_module.eefk(config_np)
        pose = transforms[0] if isinstance(transforms, list) else transforms
        return np.asarray(pose[:3, 3], dtype=np.float32)
    except Exception:
        try:
            config_np = np.asarray(config[:7], dtype=np.float32)
            transforms = robot_module.eefk(config_np)
            pose = transforms[0] if isinstance(transforms, list) else transforms
            return np.asarray(pose[:3, 3], dtype=np.float32)
        except Exception as exc:
            print(f"Warning: failed EEFK for one configuration: {exc}")
            return None


def preprocess(tree_growth_data, robot_module, max_iteration=None):
    """Map tree nodes into EEF xyz and precompute parent edges for fast animation."""
    nodes = tree_growth_data.get('nodes', [])
    if len(nodes) == 0:
        raise ValueError('No nodes found in tree growth data')

    max_iter_in_data = max(int(node.get('iteration', 0)) for node in nodes)
    if max_iteration is None:
        iteration_limit = max_iter_in_data
    else:
        iteration_limit = min(int(max_iteration), max_iter_in_data)

    filtered_nodes = [node for node in nodes if int(node.get('iteration', 0)) <= iteration_limit]

    xyz_list = []
    planner_to_plot = {}
    node_iterations = []
    creation_order = []
    node_loop_types = []
    node_sides = []

    print(f"Computing EEF positions for {len(filtered_nodes)} nodes...")
    for idx, node in enumerate(filtered_nodes):
        if idx % max(1, len(filtered_nodes) // 20) == 0:
            print(f"  {idx}/{len(filtered_nodes)}")

        xyz = config_to_eef_xyz(robot_module, node.get('config', []))
        if xyz is None:
            continue

        planner_index = int(node.get('planner_index', idx))
        planner_to_plot[planner_index] = len(xyz_list)
        xyz_list.append(xyz)
        node_iterations.append(int(node.get('iteration', 0)))
        creation_order.append(idx)
        node_loop_types.append(str(node.get('loop_type', 'outer_extend')))
        node_sides.append(str(node.get('tree_side', 'Start')))

    if len(xyz_list) == 0:
        raise ValueError('No valid nodes after EEFK conversion')

    xyz = np.asarray(xyz_list, dtype=np.float32)
    node_iterations = np.asarray(node_iterations, dtype=np.int32)
    creation_order = np.asarray(creation_order, dtype=np.int32)
    node_loop_types = np.asarray(node_loop_types)
    node_sides = np.asarray(node_sides)

    edges = []
    for idx, node in enumerate(filtered_nodes):
        planner_index = int(node.get('planner_index', idx))
        parent_index = int(node.get('parent_index', planner_index))
        if planner_index == parent_index:
            continue
        if planner_index in planner_to_plot and parent_index in planner_to_plot:
            edges.append((planner_to_plot[parent_index], planner_to_plot[planner_index]))

    unique_iterations = sorted(set(node_iterations.tolist()))
    print(f"Successfully converted {len(xyz)} nodes across {len(unique_iterations)} iterations")

    failed_extensions = tree_growth_data.get('failed_extensions', [])
    failed_xyz_list = []
    failed_iterations = []
    failed_loop_types = []
    failed_sides = []

    if failed_extensions:
        print(f"Computing EEF positions for {len(failed_extensions)} failed extensions...")
        for failed in failed_extensions:
            iteration = int(failed.get('iteration', 0))
            if iteration > iteration_limit:
                continue

            xyz_failed = config_to_eef_xyz(robot_module, failed.get('config', []))
            if xyz_failed is None:
                continue

            failed_xyz_list.append(xyz_failed)
            failed_iterations.append(iteration)
            failed_loop_types.append(str(failed.get('loop_type', 'outer_extend')))
            failed_sides.append(str(failed.get('tree_side', 'Start')))

    if failed_xyz_list:
        failed_xyz = np.asarray(failed_xyz_list, dtype=np.float32)
        failed_iterations = np.asarray(failed_iterations, dtype=np.int32)
        failed_loop_types = np.asarray(failed_loop_types)
        failed_sides = np.asarray(failed_sides)
    else:
        failed_xyz = np.empty((0, 3), dtype=np.float32)
        failed_iterations = np.empty((0,), dtype=np.int32)
        failed_loop_types = np.empty((0,), dtype=np.str_)
        failed_sides = np.empty((0,), dtype=np.str_)

    return (
        xyz,
        node_iterations,
        creation_order,
        node_loop_types,
        node_sides,
        edges,
        unique_iterations,
        failed_xyz,
        failed_iterations,
        failed_loop_types,
        failed_sides,
    )


def set_equal_axes(ax):
    """Set a fixed workspace view."""
    ax.set_xlim(-2.0, 2.0)
    ax.set_ylim(-2.0, 2.0)
    ax.set_zlim(-2.0, 2.0)
    ax.view_init(elev=90.0, azim=-90.0)


def tree_side_of(value):
    side = str(value)
    return side if side in ('Start', 'Goal') else 'Start'


def node_style_for(loop_type, tree_side):
    tree_side = tree_side_of(tree_side)
    palette = {
        'Start': {
            'seed': {'color': '#f8fafc', 'edge': '#0f172a'},
            'outer_extend': {'color': '#f97316', 'edge': '#7c2d12'},
            'inner_connect': {'color': '#0891b2', 'edge': '#083344'},
            'goal_connect': {'color': '#22c55e', 'edge': '#14532d'},
        },
        'Goal': {
            'seed': {'color': '#fde68a', 'edge': '#92400e'},
            'outer_extend': {'color': '#8b5cf6', 'edge': '#4c1d95'},
            'inner_connect': {'color': '#0ea5e9', 'edge': '#1e3a8a'},
            'goal_connect': {'color': '#16a34a', 'edge': '#14532d'},
        },
    }
    return palette[tree_side].get(loop_type, palette[tree_side]['outer_extend'])


def failed_style_for(loop_type, tree_side):
    tree_side = tree_side_of(tree_side)
    if loop_type == 'inner_connect':
        return '#be185d' if tree_side == 'Start' else '#db2777'
    return '#dc2626' if tree_side == 'Start' else '#ef4444'


def draw_obstacles(ax, obstacles):
    if not obstacles:
        return

    u = np.linspace(0.0, 2.0 * np.pi, 20)
    v = np.linspace(0.0, np.pi, 12)
    cu = np.cos(u)
    su = np.sin(u)
    sv = np.sin(v)
    cv = np.cos(v)

    for obstacle in obstacles:
        center = np.asarray(obstacle.get('center', [0.0, 0.0, 0.0]), dtype=np.float32)
        radius = float(obstacle.get('radius', 0.0))
        if radius <= 0.0:
            continue

        x = center[0] + radius * np.outer(cu, sv)
        y = center[1] + radius * np.outer(su, sv)
        z = center[2] + radius * np.outer(np.ones_like(cu), cv)

        ax.plot_surface(
            x,
            y,
            z,
            color='crimson',
            alpha=0.18,
            linewidth=0,
            shade=False,
            zorder=0,
        )


def _euler_xyz_to_rotation(euler_xyz):
    rho, theta, phi = euler_xyz
    cr, sr = np.cos(rho), np.sin(rho)
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
        dtype=np.float32,
    )
    ry = np.array(
        [[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]],
        dtype=np.float32,
    )
    rz = np.array(
        [[cp, -sp, 0.0], [sp, cp, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    return rz @ ry @ rx


def draw_cuboids(ax, cuboids):
    if not cuboids:
        return

    for cuboid in cuboids:
        center = np.asarray(cuboid.get('center', [0.0, 0.0, 0.0]), dtype=np.float32)
        half_extents = np.asarray(cuboid.get('half_extents', [0.0, 0.0, 0.0]), dtype=np.float32)
        euler_xyz = np.asarray(cuboid.get('euler_xyz', [0.0, 0.0, 0.0]), dtype=np.float32)

        if center.shape[0] != 3 or half_extents.shape[0] != 3 or euler_xyz.shape[0] != 3:
            continue
        if np.any(half_extents <= 0.0):
            continue

        rotation = _euler_xyz_to_rotation(euler_xyz)
        local = np.array(
            [
                [-1, -1, -1],
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, 1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
            ],
            dtype=np.float32,
        )
        corners = (local * half_extents) @ rotation.T + center

        faces = [
            [corners[0], corners[1], corners[2], corners[3]],
            [corners[4], corners[5], corners[6], corners[7]],
            [corners[0], corners[1], corners[5], corners[4]],
            [corners[2], corners[3], corners[7], corners[6]],
            [corners[1], corners[2], corners[6], corners[5]],
            [corners[4], corners[7], corners[3], corners[0]],
        ]

        collection = Poly3DCollection(
            faces,
            facecolors=(0.10, 0.45, 0.85, 0.16),
            edgecolors=(0.04, 0.25, 0.52, 0.70),
            linewidths=0.8,
        )
        ax.add_collection3d(collection)


def draw_iteration_frame(
    ax,
    xyz,
    node_iterations,
    node_loop_types,
    node_sides,
    edges,
    iter_now,
    obstacles,
    cuboids,
    start_marker_xyz,
    goal_marker_xyz,
    failed_xyz,
    failed_iterations,
    failed_loop_types,
    failed_sides,
    planner_name,
    phase_label=None,
):
    ax.cla()
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    title = f'{planner_name} Incremental Tree Growth - Iteration {iter_now}'
    if phase_label:
        title = f'{title} | {phase_label}'
    ax.set_title(title)
    set_equal_axes(ax)
    draw_obstacles(ax, obstacles)
    draw_cuboids(ax, cuboids)

    if start_marker_xyz is not None:
        ax.scatter(
            [start_marker_xyz[0]],
            [start_marker_xyz[1]],
            [start_marker_xyz[2]],
            c='#22c55e',
            s=220,
            marker='*',
            edgecolors='black',
            linewidths=0.8,
            depthshade=False,
            label='Start',
        )

    if goal_marker_xyz is not None:
        ax.scatter(
            [goal_marker_xyz[0]],
            [goal_marker_xyz[1]],
            [goal_marker_xyz[2]],
            c='#e11d48',
            s=240,
            marker='*',
            edgecolors='black',
            linewidths=0.8,
            depthshade=False,
            label='Goal',
        )

    visible = node_iterations <= iter_now
    visible_idx = np.where(visible)[0]
    if visible_idx.size == 0:
        return

    for loop_name, marker, size in [
        ('seed', 's', 34),
        ('outer_extend', '^', 30),
        ('inner_connect', 'o', 26),
        ('goal_connect', 'D', 38),
    ]:
        loop_idx = visible_idx[node_loop_types[visible_idx] == loop_name]
        if loop_idx.size == 0:
            continue
        for tree_side in ('Start', 'Goal'):
            side_idx = loop_idx[node_sides[loop_idx] == tree_side]
            if side_idx.size == 0:
                continue
            style = node_style_for(loop_name, tree_side)
            ax.scatter(
                xyz[side_idx, 0],
                xyz[side_idx, 1],
                xyz[side_idx, 2],
                c=style['color'],
                marker=marker,
                s=size,
                alpha=0.78 if tree_side == 'Start' else 0.88,
                edgecolors=style['edge'],
                linewidths=0.55 if tree_side == 'Start' else 0.75,
                depthshade=True,
            )

    visible_set = set(visible_idx.tolist())
    for parent_plot, child_plot in edges:
        if parent_plot in visible_set and child_plot in visible_set:
            child_loop = str(node_loop_types[child_plot])
            child_side = tree_side_of(node_sides[child_plot])
            style = node_style_for(child_loop, child_side)
            if child_loop == 'inner_connect':
                edge_width = 1.15
            elif child_loop == 'outer_extend':
                edge_width = 1.05
            elif child_loop == 'goal_connect':
                edge_width = 1.70
            else:
                edge_width = 0.90
            ax.plot(
                [xyz[parent_plot, 0], xyz[child_plot, 0]],
                [xyz[parent_plot, 1], xyz[child_plot, 1]],
                [xyz[parent_plot, 2], xyz[child_plot, 2]],
                color=style['color'],
                linewidth=edge_width,
            )

    current_idx = np.where(node_iterations == iter_now)[0]
    if current_idx.size > 0:
        for loop_name, marker, size in [
            ('outer_extend', '^', 84),
            ('inner_connect', 'o', 74),
            ('seed', 's', 68),
            ('goal_connect', 'D', 92),
        ]:
            loop_idx = current_idx[node_loop_types[current_idx] == loop_name]
            if loop_idx.size == 0:
                continue
            for tree_side in ('Start', 'Goal'):
                side_idx = loop_idx[node_sides[loop_idx] == tree_side]
                if side_idx.size == 0:
                    continue
                style = node_style_for(loop_name, tree_side)
                ax.scatter(
                    xyz[side_idx, 0],
                    xyz[side_idx, 1],
                    xyz[side_idx, 2],
                    c=style['color'],
                    marker=marker,
                    s=size,
                    alpha=0.99,
                    edgecolors=style['edge'],
                    linewidths=0.80 if tree_side == 'Start' else 0.95,
                    depthshade=True,
                )

    failed_idx = np.where(failed_iterations == iter_now)[0]
    if failed_idx.size > 0:
        for loop_name, size in [('outer_extend', 76), ('inner_connect', 84)]:
            loop_idx = failed_idx[failed_loop_types[failed_idx] == loop_name]
            if loop_idx.size == 0:
                continue
            for tree_side in ('Start', 'Goal'):
                side_idx = loop_idx[failed_sides[loop_idx] == tree_side]
                if side_idx.size == 0:
                    continue
                ax.scatter(
                    failed_xyz[side_idx, 0],
                    failed_xyz[side_idx, 1],
                    failed_xyz[side_idx, 2],
                    c=failed_style_for(loop_name, tree_side),
                    s=size,
                    alpha=0.95,
                    marker='x',
                    linewidths=2.3 if loop_name == 'outer_extend' else 2.5,
                    depthshade=False,
                )

    import matplotlib.lines as mlines

    legend_items = [
        mlines.Line2D([], [], color='#f97316', marker='^', linestyle='None', markersize=8, label='Start tree outer extend'),
        mlines.Line2D([], [], color='#8b5cf6', marker='^', linestyle='None', markersize=8, label='Goal tree outer extend'),
        mlines.Line2D([], [], color='#0891b2', marker='o', linestyle='None', markersize=7, label='Start tree inner connect'),
        mlines.Line2D([], [], color='#0ea5e9', marker='o', linestyle='None', markersize=7, label='Goal tree inner connect'),
        mlines.Line2D([], [], color='#22c55e', marker='D', linestyle='None', markersize=8, label='Start tree connect'),
        mlines.Line2D([], [], color='#16a34a', marker='D', linestyle='None', markersize=8, label='Goal tree connect'),
        mlines.Line2D([], [], color='#dc2626', marker='x', linestyle='None', markersize=8, label='Start fail'),
        mlines.Line2D([], [], color='#ef4444', marker='x', linestyle='None', markersize=8, label='Goal fail'),
        mlines.Line2D([], [], color=(0.04, 0.25, 0.52, 0.90), linestyle='-', linewidth=2.0, label='Cuboid obstacle'),
    ]
    if start_marker_xyz is not None:
        legend_items.append(mlines.Line2D([], [], color='#22c55e', marker='*', linestyle='None', markersize=11, label='Start marker'))
    if goal_marker_xyz is not None:
        legend_items.append(mlines.Line2D([], [], color='#e11d48', marker='*', linestyle='None', markersize=11, label='Goal marker'))

    ax.legend(handles=legend_items, loc='upper right', fontsize=8, framealpha=0.80)


def build_frame_states(unique_iterations, auto_play_end_hold_frames):
    """Create frame states for forward, end-pause, reverse, and start-pause."""
    if len(unique_iterations) == 0:
        return []

    states = []
    for iteration in unique_iterations:
        states.append({'iter': int(iteration), 'phase': 'forward'})

    states.append({'iter': int(unique_iterations[-1]), 'phase': 'end_pause'})

    if len(unique_iterations) > 1:
        for iteration in unique_iterations[-2::-1]:
            states.append({'iter': int(iteration), 'phase': 'reverse'})

    states.append({'iter': int(unique_iterations[0]), 'phase': 'start_pause'})

    if auto_play_end_hold_frames > 1:
        extra_end_holds = max(0, int(auto_play_end_hold_frames) - 1)
        insert_at = len(unique_iterations)
        for _ in range(extra_end_holds):
            states.insert(insert_at, {'iter': int(unique_iterations[-1]), 'phase': 'end_pause'})

    return states


def animate_incremental_growth(
    xyz,
    node_iterations,
    creation_order,
    node_loop_types,
    node_sides,
    edges,
    unique_iterations,
    obstacles,
    cuboids,
    start_marker_xyz,
    goal_marker_xyz,
    failed_xyz,
    failed_iterations,
    failed_loop_types,
    failed_sides,
    planner_name='Tree',
    save_animation=None,
    auto_play=False,
    auto_play_interval_ms=400,
    auto_play_end_hold_frames=8,
):
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    set_equal_axes(ax)

    state = {'iter_pos': 0, 'phase': 'forward'}
    auto_anim = None

    def current_iter_and_label():
        iter_now = int(unique_iterations[state['iter_pos']])
        phase = state['phase']
        if phase == 'end_pause':
            return iter_now, 'RUN COMPLETE - click Next to reverse'
        if phase == 'start_pause':
            return iter_now, 'REVERSE COMPLETE - click Next to roll forward'
        if phase == 'reverse':
            return iter_now, 'Reversing'
        return iter_now, None

    def render_current():
        iter_now, phase_label = current_iter_and_label()
        draw_iteration_frame(
            ax,
            xyz,
            node_iterations,
            node_loop_types,
            node_sides,
            edges,
            iter_now,
            obstacles,
            cuboids,
            start_marker_xyz,
            goal_marker_xyz,
            failed_xyz,
            failed_iterations,
            failed_loop_types,
            failed_sides,
            planner_name,
            phase_label,
        )
        fig.canvas.draw_idle()

    button_ax_next = fig.add_axes([0.80, 0.02, 0.12, 0.06])
    button_ax_reset = fig.add_axes([0.66, 0.02, 0.12, 0.06])
    button_next = Button(button_ax_next, 'Next')
    button_reset = Button(button_ax_reset, 'Reset')

    def step_iteration_click():
        """One click == one planner iteration (extend + connect)."""
        if len(unique_iterations) <= 1:
            return

        if state['phase'] == 'forward':
            if state['iter_pos'] < len(unique_iterations) - 1:
                state['iter_pos'] += 1
            else:
                state['phase'] = 'end_pause'
        elif state['phase'] == 'end_pause':
            state['phase'] = 'reverse'
            if state['iter_pos'] > 0:
                state['iter_pos'] -= 1
        elif state['phase'] == 'reverse':
            if state['iter_pos'] > 0:
                state['iter_pos'] -= 1
            else:
                state['phase'] = 'start_pause'
        else:
            state['phase'] = 'forward'
            if state['iter_pos'] < len(unique_iterations) - 1:
                state['iter_pos'] += 1

    if auto_play:
        def auto_step(_frame_idx):
            if state['phase'] in ('end_pause', 'start_pause'):
                return
            step_iteration_click()
            render_current()

        auto_anim = FuncAnimation(
            fig,
            auto_step,
            interval=max(1, int(auto_play_interval_ms)),
            repeat=True,
            cache_frame_data=False,
        )

    def on_next(_event):
        step_iteration_click()
        render_current()

    def on_reset(_event):
        state['iter_pos'] = 0
        state['phase'] = 'forward'
        render_current()

    button_next.on_clicked(on_next)
    button_reset.on_clicked(on_reset)

    render_current()

    if save_animation:
        print(f"Saving looped animation to {save_animation} ...")
        save_frame_states = build_frame_states(unique_iterations, auto_play_end_hold_frames)

        def draw_frame(frame_idx):
            frame = save_frame_states[frame_idx]
            iter_now = frame['iter']
            phase = frame['phase']
            phase_label = None
            if phase == 'end_pause':
                phase_label = 'RUN COMPLETE - click Next to reverse'
            elif phase == 'start_pause':
                phase_label = 'REVERSE COMPLETE - click Next to roll forward'
            elif phase == 'reverse':
                phase_label = 'Reversing'
            draw_iteration_frame(
                ax,
                xyz,
                node_iterations,
                node_loop_types,
                node_sides,
                edges,
                iter_now,
                obstacles,
                cuboids,
                start_marker_xyz,
                goal_marker_xyz,
                failed_xyz,
                failed_iterations,
                failed_loop_types,
                failed_sides,
                planner_name,
                phase_label,
            )

        anim = FuncAnimation(
            fig,
            draw_frame,
            frames=len(save_frame_states),
            interval=max(1, int(auto_play_interval_ms)),
            repeat=True,
            repeat_delay=0,
        )
        writer_fps = max(1, int(round(1000.0 / max(1, int(auto_play_interval_ms)))))
        writer = PillowWriter(fps=writer_fps)
        anim.save(save_animation, writer=writer)
        print(f"Saved: {save_animation}")

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    tree_growth_file = sys.argv[1]
    robot_type = sys.argv[2] if len(sys.argv) > 2 else 'panda'
    save_animation = sys.argv[3] if len(sys.argv) > 3 else None
    if isinstance(save_animation, str) and save_animation.lower() in ('none', 'null', ''):
        save_animation = None
    max_iteration = int(sys.argv[4]) if len(sys.argv) > 4 else None
    play_mode = sys.argv[5].strip().lower() if len(sys.argv) > 5 else 'manual'
    auto_play = play_mode in ('auto', 'autoplay', 'play')
    auto_play_interval_ms = int(sys.argv[6]) if len(sys.argv) > 6 else 400
    auto_play_end_hold_frames = int(sys.argv[7]) if len(sys.argv) > 7 else 8

    print(f"Loading tree growth data from {tree_growth_file}...")
    tree_growth_data = load_tree_growth(tree_growth_file)

    print(f"Using robot: {robot_type}")
    robot_module = get_robot_module(robot_type)

    (
        xyz,
        node_iterations,
        creation_order,
        node_loop_types,
        node_sides,
        edges,
        unique_iterations,
        failed_xyz,
        failed_iterations,
        failed_loop_types,
        failed_sides,
    ) = preprocess(tree_growth_data, robot_module, max_iteration=max_iteration)

    if failed_xyz.shape[0] > 0:
        print(f"Loaded {failed_xyz.shape[0]} failed extension points")

    obstacles = tree_growth_data.get('obstacles', [])
    if obstacles:
        print(f"Loaded {len(obstacles)} obstacle spheres")

    cuboids = tree_growth_data.get('cuboids', [])
    if cuboids:
        print(f"Loaded {len(cuboids)} obstacle cuboids")

    start_config = tree_growth_data.get('start_config', None)
    goal_config = tree_growth_data.get('goal_config', None)
    start_marker_xyz = config_to_eef_xyz(robot_module, start_config)
    goal_marker_xyz = config_to_eef_xyz(robot_module, goal_config)

    if start_marker_xyz is not None:
        print(f"Loaded start marker at ({start_marker_xyz[0]:.3f}, {start_marker_xyz[1]:.3f}, {start_marker_xyz[2]:.3f})")
    if goal_marker_xyz is not None:
        print(f"Loaded goal marker at ({goal_marker_xyz[0]:.3f}, {goal_marker_xyz[1]:.3f}, {goal_marker_xyz[2]:.3f})")

    planner_name = str(tree_growth_data.get('planner', 'Tree'))

    animate_incremental_growth(
        xyz,
        node_iterations,
        creation_order,
        node_loop_types,
        node_sides,
        edges,
        unique_iterations,
        obstacles,
        cuboids,
        start_marker_xyz,
        goal_marker_xyz,
        failed_xyz,
        failed_iterations,
        failed_loop_types,
        failed_sides,
        planner_name=planner_name,
        save_animation=save_animation,
        auto_play=auto_play,
        auto_play_interval_ms=auto_play_interval_ms,
        auto_play_end_hold_frames=auto_play_end_hold_frames,
    )
