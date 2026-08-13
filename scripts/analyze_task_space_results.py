"""Compare success rates, planning times, and path quality across one or more
rby1_task_space_planner.cc result JSON files (e.g. different environment/local-planner
variants run over the same problem set, like resources/ruby/problem_set_*_res*.json).

Usage:
    python3 scripts/analyze_task_space_results.py \
        --paths '["resources/ruby/problem_set_skipped_intermediate_res.json", \
                   "resources/ruby/problem_set_skipped_intermediate_res_without_early_eef.json"]' \
        --labels '["with_early_eef", "without_early_eef"]' \
        --output_dir plots/task_space

Requires: numpy, pandas, matplotlib (pip install numpy pandas matplotlib).
"""

import json
from pathlib import Path
from typing import List, Optional, Union

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from fire import Fire

# problem_result["error"] strings set in rby1_task_space_planner.cc, mapped to short
# failure-category labels. A result with solved == False and no "error" key means RRTC
# itself ran (start/goal passed every pre-check) but exhausted its budget without a path.
_ERROR_SUBSTRING_TO_CATEGORY = [
    ("wrong number of elements", "malformed_problem"),
    ("did not resolve through IK", "ik_failed"),
    ("colliding ambient configuration", "start_or_goal_collision"),
    ("fails resolve_and_check", "initial_conditions_failed"),
]


def _failure_category(row: pd.Series) -> str:
    if row["solved"]:
        return "solved"
    error = row.get("error")
    if isinstance(error, str) and error:
        for substring, category in _ERROR_SUBSTRING_TO_CATEGORY:
            if substring in error:
                return category
        return "other_precheck_failure"
    return "no_path_found_in_budget"


def load_run(path: str, label: Optional[str] = None) -> pd.DataFrame:
    with open(path) as f:
        data = json.load(f)

    rows = [{k: v for k, v in entry.items() if k != "trajectory"} for entry in data]
    df = pd.DataFrame(rows)

    df["run"] = label if label is not None else Path(path).stem
    df["solved"] = df["solved"].fillna(False).astype(bool)
    df["planning_time_ms"] = df.get("nanoseconds", pd.Series(dtype=float)) / 1e6
    df["shortcut_cost_reduction_pct"] = 100.0 * (
        1.0 - df.get("shortcut_cost_after", np.nan) / df.get("shortcut_cost_before", np.nan)
    )
    df["failure_category"] = df.apply(_failure_category, axis=1)
    return df


def load_all(paths: List[str], labels: Optional[List[str]]) -> pd.DataFrame:
    if labels is not None and len(labels) != len(paths):
        raise ValueError(f"Got {len(paths)} paths but {len(labels)} labels; they must match 1:1.")

    frames = [
        load_run(path, labels[i] if labels is not None else None) for i, path in enumerate(paths)
    ]
    return pd.concat(frames, ignore_index=True)


def _run_order(df: pd.DataFrame) -> List[str]:
    # Preserve first-seen order (i.e. the order paths/labels were passed in) rather than
    # letting groupby sort alphabetically, so plots line up with how the user specified runs.
    return list(dict.fromkeys(df["run"]))


def _run_colors(runs: List[str]) -> dict:
    cmap = plt.get_cmap("tab10")
    return {run: cmap(i % 10) for i, run in enumerate(runs)}


def print_summary(df: pd.DataFrame) -> None:
    runs = _run_order(df)
    rows = []
    for run in runs:
        sub = df[df["run"] == run]
        solved = sub[sub["solved"]]
        rows.append(
            {
                "run": run,
                "n_problems": len(sub),
                "n_solved": len(solved),
                "success_rate_%": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
                "median_time_ms": solved["planning_time_ms"].median(),
                "p90_time_ms": solved["planning_time_ms"].quantile(0.9),
                "mean_iterations": solved["iterations"].mean(),
                "mean_cost_after_shortcut": solved["shortcut_cost_after"].mean(),
                "mean_shortcut_reduction_%": solved["shortcut_cost_reduction_pct"].mean(),
            }
        )
    summary = pd.DataFrame(rows).set_index("run")

    print("\n=== Summary ===")
    try:
        print(summary.to_markdown(floatfmt=".2f"))
    except ImportError:
        print(summary.to_string(float_format=lambda v: f"{v:.2f}"))

    print("\n=== Failure breakdown (unsolved problems only) ===")
    unsolved = df[~df["solved"]]
    if unsolved.empty:
        print("(none -- every run solved every problem)")
    else:
        breakdown = (
            unsolved.groupby(["run", "failure_category"]).size().unstack(fill_value=0).reindex(runs)
        )
        print(breakdown.to_string())


def _cdf(values: pd.Series):
    values = np.sort(values.dropna().to_numpy(dtype=float))
    if len(values) == 0:
        return values, values
    return values, np.arange(1, len(values) + 1) / len(values)


def plot_success_rate(df: pd.DataFrame, out_dir: Path, colors: dict) -> Path:
    runs = _run_order(df)
    rates = [100.0 * df[df["run"] == r]["solved"].mean() for r in runs]
    counts = [(df[df["run"] == r]["solved"].sum(), len(df[df["run"] == r])) for r in runs]

    fig, ax = plt.subplots(figsize=(max(4, 1.2 * len(runs)), 4.5))
    bars = ax.bar(runs, rates, color=[colors[r] for r in runs])
    for bar, (n_solved, n_total) in zip(bars, counts):
        ax.annotate(
            f"{n_solved}/{n_total}",
            (bar.get_x() + bar.get_width() / 2, bar.get_height()),
            ha="center",
            va="bottom",
        )

    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Planning success rate by run")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()

    path = out_dir / "success_rate.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_time_cdf(df: pd.DataFrame, out_dir: Path, colors: dict, log_x: bool = True) -> Path:
    runs = _run_order(df)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    for run in runs:
        solved = df[(df["run"] == run) & df["solved"]]
        x, y = _cdf(solved["planning_time_ms"])
        if len(x):
            ax.step(x, y, where="post", label=run, color=colors[run])

    if log_x:
        ax.set_xscale("log")
    ax.set_xlabel("Planning time (ms, log scale)" if log_x else "Planning time (ms)")
    ax.set_ylabel("Fraction of solved problems <= x")
    ax.set_title("CDF of planning time (solved problems only)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    path = out_dir / "planning_time_cdf.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_time_box(df: pd.DataFrame, out_dir: Path, colors: dict) -> Path:
    runs = _run_order(df)
    data = [df[(df["run"] == r) & df["solved"]]["planning_time_ms"].dropna().to_numpy() for r in runs]

    fig, ax = plt.subplots(figsize=(max(4, 1.2 * len(runs)), 4.5))
    bplot = ax.boxplot(data, tick_labels=runs, patch_artist=True, showfliers=True)
    for patch, run in zip(bplot["boxes"], runs):
        patch.set_facecolor(colors[run])
        patch.set_alpha(0.6)

    ax.set_ylabel("Planning time (ms)")
    ax.set_yscale("log")
    ax.set_title("Planning time distribution (solved problems only)")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()

    path = out_dir / "planning_time_box.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_cost_cdf(df: pd.DataFrame, out_dir: Path, colors: dict) -> Path:
    runs = _run_order(df)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    any_data = False
    for run in runs:
        solved = df[(df["run"] == run) & df["solved"]]
        x, y = _cdf(solved["shortcut_cost_after"])
        if len(x):
            any_data = True
            ax.step(x, y, where="post", label=run, color=colors[run])

    if not any_data:
        plt.close(fig)
        return None

    ax.set_xlabel("Final (shortcut) path cost")
    ax.set_ylabel("Fraction of solved problems <= x")
    ax.set_title("CDF of solution path cost (solved problems only)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    path = out_dir / "path_cost_cdf.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_shortcut_improvement(df: pd.DataFrame, out_dir: Path, colors: dict) -> Path:
    runs = _run_order(df)
    means = [df[(df["run"] == r) & df["solved"]]["shortcut_cost_reduction_pct"].mean() for r in runs]
    if all(np.isnan(m) for m in means):
        return None

    fig, ax = plt.subplots(figsize=(max(4, 1.2 * len(runs)), 4.5))
    ax.bar(runs, means, color=[colors[r] for r in runs])
    ax.set_ylabel("Mean cost reduction from shortcutting (%)")
    ax.set_title("Shortcut effectiveness by run")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()

    path = out_dir / "shortcut_improvement.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_failure_breakdown(df: pd.DataFrame, out_dir: Path) -> Path:
    unsolved = df[~df["solved"]]
    if unsolved.empty:
        return None

    runs = _run_order(df)
    counts = unsolved.groupby(["run", "failure_category"]).size().unstack(fill_value=0).reindex(runs)

    ax = counts.plot(kind="bar", stacked=True, figsize=(max(4, 1.2 * len(runs)), 4.5), colormap="Set2")
    ax.set_ylabel("Number of unsolved problems")
    ax.set_title("Failure breakdown by run")
    ax.legend(title="Failure category", bbox_to_anchor=(1.02, 1), loc="upper left")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig = ax.get_figure()
    fig.tight_layout()

    path = out_dir / "failure_breakdown.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_per_problem_heatmap(df: pd.DataFrame, out_dir: Path) -> Path:
    if "index" not in df.columns:
        return None

    runs = _run_order(df)
    pivot = df.pivot_table(index="index", columns="run", values="solved", aggfunc="first").reindex(
        columns=runs
    )

    fig, ax = plt.subplots(figsize=(max(4, 1.2 * len(runs)), max(4, 0.25 * len(pivot))))
    ax.imshow(pivot.to_numpy(dtype=float), cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(runs)))
    ax.set_xticklabels(runs, rotation=20, ha="right")
    ax.set_yticks(range(len(pivot)))
    ax.set_yticklabels(pivot.index)
    ax.set_ylabel("Problem index")
    ax.set_title("Solved (green) / unsolved (red) by problem")
    fig.tight_layout()

    path = out_dir / "per_problem_heatmap.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main(
    paths: Union[str, List[str]],
    labels: Optional[List[str]] = None,
    output_dir: str = "plots/task_space_analysis",
    log_time_axis: bool = True,
    show: bool = False,
):
    """Analyze and plot one or more rby1_task_space_planner.cc result JSON files.

    paths: a single result JSON path, or a list of them (one per run/variant to compare).
    labels: optional list of run names, same length as paths (defaults to each file's stem).
    output_dir: directory plots are written to (created if missing).
    log_time_axis: use a log-scaled x-axis for the planning-time CDF/box plots.
    show: also call plt.show() after saving, for interactive use.
    """
    if isinstance(paths, str):
        paths = [paths]

    df = load_all(paths, labels)
    print_summary(df)

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    colors = _run_colors(_run_order(df))

    saved = [
        plot_success_rate(df, out_dir, colors),
        plot_time_cdf(df, out_dir, colors, log_x=log_time_axis),
        plot_time_box(df, out_dir, colors),
        plot_cost_cdf(df, out_dir, colors),
        plot_shortcut_improvement(df, out_dir, colors),
        plot_failure_breakdown(df, out_dir),
        plot_per_problem_heatmap(df, out_dir),
    ]

    print("\n=== Plots written ===")
    for p in saved:
        if p is not None:
            print(p)

    if show:
        plt.show()


if __name__ == "__main__":
    Fire(main)
