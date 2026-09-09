"""Compare rby1_task_space_planner.cc and rby1_mcvamp_planner.cc result JSONs on the three
metrics they both report in a common shape: success rate, RRTC iterations, and planning time.

Both binaries write a JSON array of per-problem result objects (one "solved" bool, plus
"iterations"/"planning_time_ms" when RRTC actually ran -- an entry can be "solved": false with
neither key present, e.g. a malformed problem or a start/goal that failed to
project/IK-resolve or was in collision before RRTC ever started). This script only reads
those three common fields, so it works even though the two planners' JSONs otherwise differ
(task-space's "cost"/"shortcut_cost_before"/"shortcut_cost_after" have no mcvamp equivalent,
and vice versa for "config_distance").

Usage:
    python3 scripts/compare_rby1_planners.py \
        --task_space_json resources/ruby/problem_set_skipped_intermediate_res.json \
        --mcvamp_json resources/ruby/rrtc_calls_res_mvplanner.json \
        --output_dir plots/rby1_planner_comparison

Requires: numpy, pandas, matplotlib (pip install numpy pandas matplotlib).
"""

import json
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from fire import Fire

# Seaborn colorblind palette, same two colors scripts/plot_bimanual_results.py uses first --
# kept fixed per planner (not reassigned by argument order) so repeated runs of this script
# stay visually consistent.
_PLANNER_COLORS = {
    "task_space": "#66c2a5",
    "mcvamp": "#fc8d62",
}
_PLANNER_DISPLAY_NAMES = {
    "task_space": "Task-space (IK)",
    "mcvamp": "MCVAMP (projection)",
}


def load_run(path: str, planner: str) -> pd.DataFrame:
    with open(path) as f:
        data = json.load(f)

    # Drop "trajectory" (task-space) -- the only field either planner writes that's
    # per-waypoint rather than per-problem, and irrelevant to these three metrics.
    rows = [{k: v for k, v in entry.items() if k != "trajectory"} for entry in data]
    df = pd.DataFrame(rows)

    df["planner"] = planner
    df["solved"] = df.get("solved", pd.Series(dtype=bool)).fillna(False).astype(bool)
    # Presence of "iterations" is what distinguishes "RRTC actually ran on this problem" from
    # a problem skipped before RRTC (malformed input, failed projection/IK, or a colliding
    # start/goal) -- both planners only set "iterations"/"planning_time_ms" once RRTC runs.
    df["attempted"] = df["iterations"].notna() if "iterations" in df.columns else False
    if "iterations" not in df.columns:
        df["iterations"] = np.nan
    if "planning_time_ms" not in df.columns:
        df["planning_time_ms"] = np.nan

    return df


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for planner in ("task_space", "mcvamp"):
        sub = df[df["planner"] == planner]
        if sub.empty:
            continue

        attempted = sub[sub["attempted"]]
        solved = sub[sub["solved"]]
        rows.append(
            {
                "planner": _PLANNER_DISPLAY_NAMES[planner],
                "n_problems": len(sub),
                "n_attempted": len(attempted),
                "n_solved": len(solved),
                "success_rate_of_total_%": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
                "success_rate_of_attempted_%": (
                    100.0 * len(solved) / len(attempted) if len(attempted) else float("nan")
                ),
                "mean_iterations": attempted["iterations"].mean(),
                "median_iterations": attempted["iterations"].median(),
                "mean_planning_time_ms": attempted["planning_time_ms"].mean(),
                "median_planning_time_ms": attempted["planning_time_ms"].median(),
            }
        )

    return pd.DataFrame(rows).set_index("planner")


def plot_comparison(df: pd.DataFrame, summary: pd.DataFrame, out_dir: Path) -> Path:
    planners = [p for p in ("task_space", "mcvamp") if p in df["planner"].unique()]
    display_names = [_PLANNER_DISPLAY_NAMES[p] for p in planners]
    colors = [_PLANNER_COLORS[p] for p in planners]

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))

    # --- Success rate (of attempted problems) ---
    ax = axes[0]
    rates = summary.loc[display_names, "success_rate_of_attempted_%"]
    bars = ax.bar(display_names, rates, color=colors)
    for bar, planner in zip(bars, planners):
        sub = df[df["planner"] == planner]
        n_solved = int(sub["solved"].sum())
        n_attempted = int(sub["attempted"].sum())
        ax.annotate(
            f"{n_solved}/{n_attempted}",
            (bar.get_x() + bar.get_width() / 2, bar.get_height()),
            ha="center",
            va="bottom",
        )
    ax.set_ylabel("Success rate (%, of attempted problems)")
    ax.set_ylim(0, 105)
    ax.set_title("Success rate")

    # --- Iterations (solved problems only) ---
    ax = axes[1]
    data = [
        df[(df["planner"] == p) & df["solved"]]["iterations"].dropna().to_numpy() for p in planners
    ]
    bplot = ax.boxplot(data, tick_labels=display_names, patch_artist=True, showfliers=True)
    for patch, color in zip(bplot["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    ax.set_ylabel("RRTC iterations")
    ax.set_title("Iterations (solved problems)")

    # --- Planning time (solved problems only) ---
    ax = axes[2]
    data = [
        df[(df["planner"] == p) & df["solved"]]["planning_time_ms"].dropna().to_numpy()
        for p in planners
    ]
    bplot = ax.boxplot(data, tick_labels=display_names, patch_artist=True, showfliers=True)
    for patch, color in zip(bplot["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    ax.set_ylabel("Planning time (ms)")
    ax.set_yscale("log")
    ax.set_title("Planning time (solved problems)")

    for ax in axes:
        plt.setp(ax.get_xticklabels(), rotation=15, ha="right")

    fig.tight_layout()

    path = out_dir / "rby1_planner_comparison.png"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


def plot_table(summary: pd.DataFrame, out_dir: Path) -> Path:
    display_columns = [
        ("n_problems", "N"),
        ("n_attempted", "Attempted"),
        ("n_solved", "Solved"),
        ("success_rate_of_total_%", "Success % (of N)"),
        ("success_rate_of_attempted_%", "Success % (of attempted)"),
        ("mean_iterations", "Mean iters"),
        ("median_iterations", "Median iters"),
        ("mean_planning_time_ms", "Mean time (ms)"),
        ("median_planning_time_ms", "Median time (ms)"),
    ]

    def fmt(planner: str, col: str) -> str:
        v = summary.loc[planner, col]
        if pd.isna(v):
            return "--"
        if col in ("n_problems", "n_attempted", "n_solved"):
            return f"{int(v)}"
        return f"{v:.1f}" if "%" in col or "time" in col else f"{v:.2f}"

    cell_text = [[fmt(planner, col) for col, _ in display_columns] for planner in summary.index]

    fig, ax = plt.subplots(figsize=(1.4 * len(display_columns), 0.9 + 0.6 * len(summary)))
    ax.axis("off")
    table = ax.table(
        cellText=cell_text,
        rowLabels=list(summary.index),
        colLabels=[label for _, label in display_columns],
        cellLoc="center",
        rowLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.8)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor("#e6e6e6")
            cell.set_text_props(weight="bold")
        elif col == -1:
            cell.set_text_props(weight="bold")
            planner_key = "task_space" if summary.index[row - 1] == _PLANNER_DISPLAY_NAMES["task_space"] else "mcvamp"
            cell.set_facecolor(_PLANNER_COLORS[planner_key])
            cell.set_alpha(0.35)

    fig.tight_layout()

    path = out_dir / "rby1_planner_comparison_table.png"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


def main(
    task_space_json: str,
    mcvamp_json: str,
    output_dir: str = "plots/rby1_planner_comparison",
    show: bool = False,
):
    """Compare rby1_task_space_planner.cc and rby1_mcvamp_planner.cc results.

    task_space_json: result JSON written by vamp_rby1_task_space_planner.
    mcvamp_json: result JSON written by vamp_rby1_mcvamp_planner.
    output_dir: directory the plot/table images are written to (created if missing).
    show: also call plt.show() after saving, for interactive use.
    """
    df = pd.concat(
        [
            load_run(task_space_json, "task_space"),
            load_run(mcvamp_json, "mcvamp"),
        ],
        ignore_index=True,
    )

    summary = summarize(df)

    print("\n=== Summary (task-space IK planner vs. mcvamp projection planner) ===")
    try:
        print(summary.to_markdown(floatfmt=".2f"))
    except ImportError:
        print(summary.to_string(float_format=lambda v: f"{v:.2f}"))

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    plot_path = plot_comparison(df, summary, out_dir)
    table_path = plot_table(summary, out_dir)

    print("\n=== Saved ===")
    print(f"{plot_path} (+ .svg)")
    print(f"{table_path}")

    if show:
        plt.show()


if __name__ == "__main__":
    Fire(main)
