"""Compare planning time, shortcutting time, and shortcut path distance (config-space and
eef-space) across one or more bimanual_iiwa_*_shelf.cc result CSVs (e.g. the projection,
leader-follower, and parameterized baselines run over the same shelf problem).

Each input CSV is the per-trial results file written by
scripts/cpp/bimanual_iiwa_projection_shelf.cc / bimanual_iiwa_leader_follower_shelf.cc /
bimanual_iiwa_parameterized_shelf.cc (results/bimanual_iiwa_*_shelf.csv by default), with
one row per trial: method,trial,pair,solved,planning_time_ms,iterations,shortcut_time_ms,
config_distance,eef_distance. shortcut_time_ms/config_distance/eef_distance are only
populated for solved trials, and config_distance/eef_distance are the SHORTCUT path's
length, not the raw RRTC path's.

Usage:
    python3 scripts/plot_bimanual_results.py \
        --paths '["results/bimanual_iiwa_projection_shelf.csv", \
                   "results/bimanual_iiwa_leader_follower_shelf.csv", \
                   "results/bimanual_iiwa_parameterized_shelf.csv"]' \
        --output_dir plots/bimanual_results

`paths`/`labels` aren't limited to three -- pass as many CSVs as you want to compare, with
an optional manual label per CSV (same length as `paths`) overriding whatever `method`
value that CSV's own rows carry, e.g. for 5 runs:
    python3 scripts/plot_bimanual_results.py \
        --paths '["results/a.csv","results/b.csv","results/c.csv","results/d.csv", \
                   "results/e.csv"]' \
        --labels '["Baseline A","Baseline B","Baseline C","Ablation D","Ablation E"]' \
        --output_dir plots/bimanual_results

Requires: pandas, matplotlib, seaborn (pip install pandas matplotlib seaborn).
"""

from pathlib import Path
from typing import List, Optional

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import seaborn as sns
from fire import Fire

# CSV method value -> display label/order for plots. Any method present in a CSV but not
# listed here still plots, just appended after these in whatever order pandas encounters
# it (so a new baseline doesn't silently disappear from the figures).
_METHOD_DISPLAY_ORDER = [
    ("projection", "TSR / Projection"),
    ("parameterized", "Parameterized"),
    ("leader_follower", "Leader-Follower"),
]

_METRICS = [
    ("planning_time_ms", "Planning time (ms)"),
    ("shortcut_time_ms", "Shortcut time (ms)"),
    ("config_distance", "Config-space distance (rad)"),
    ("eef_distance", "EEF-space distance"),
]

COLORS = ['#66c2a5','#fc8d62','#8da0cb','#e78ac3']

def _set_style() -> None:
    sns.set_theme(
        style="whitegrid",
        context="paper",
        font_scale=1.2,
        rc={
            "axes.edgecolor": "0.3",
            "axes.linewidth": 0.9,
            "grid.color": "0.88",
            "grid.linewidth": 0.7,
            "axes.titleweight": "bold",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        },
    )


def load_run(path: str, label: Optional[str] = None) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["solved"] = df["solved"].astype(bool)
    if label is not None:
        df["method"] = label
    return df


def load_all(paths: List[str], labels: Optional[List[str]]) -> pd.DataFrame:
    if labels is not None and len(labels) != len(paths):
        raise ValueError(f"Got {len(paths)} paths but {len(labels)} labels; they must match 1:1.")

    frames = [load_run(path, labels[i] if labels is not None else None) for i, path in enumerate(paths)]
    return pd.concat(frames, ignore_index=True)


def _method_order(df: pd.DataFrame) -> List[str]:
    present = list(dict.fromkeys(df["method"]))
    ordered = [m for m, _ in _METHOD_DISPLAY_ORDER if m in present]
    ordered += [m for m in present if m not in ordered]
    return ordered


def _display_name(method: str) -> str:
    for key, label in _METHOD_DISPLAY_ORDER:
        if key == method:
            return label
    return method


def _method_palette(methods: List[str]) -> dict:
    # n_colors=len(methods) asks seaborn for exactly that many distinct pastel colors
    # (it'll space them around the pastel hue wheel rather than just repeating a fixed
    # 10-color list), so this stays distinguishable well past 3 methods.
    # colors = sns.color_palette("bright", n_colors=len(methods))
    colors = COLORS[:len(methods)]
    return dict(zip(methods, colors))


def print_summary(df: pd.DataFrame) -> None:
    methods = _method_order(df)
    rows = []
    for method in methods:
        sub = df[df["method"] == method]
        solved = sub[sub["solved"]]
        rows.append(
            {
                "method": _display_name(method),
                "n_trials": len(sub),
                "n_solved": len(solved),
                "success_rate_%": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
                "median_planning_ms": solved["planning_time_ms"].median(),
                "median_shortcut_ms": solved["shortcut_time_ms"].median(),
                "median_config_distance": solved["config_distance"].median(),
                "median_eef_distance": solved["eef_distance"].median(),
            }
        )

    summary = pd.DataFrame(rows).set_index("method")
    with pd.option_context("display.float_format", "{:.4f}".format):
        print(summary)


def plot_small_multiples(df: pd.DataFrame, output_dir: Path) -> Path:
    """1x4 grid of box plots (one panel per metric, one box per method) with a jittered
    strip of the underlying trials overlaid -- the primary results figure: every method's
    full distribution, and the raw data behind it, on every metric at a glance."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = {name: _method_palette(methods)[m] for m, name in zip(methods, display_names)}
    solved = df[df["solved"]].copy()
    solved["Method"] = solved["method"].map(_display_name)

    # Panel width scales with method count so boxes/labels don't get crushed once
    # there's more than a handful of methods being compared.
    panel_width = max(4.4, 0.85 * len(methods) + 2.2)
    fig, axes = plt.subplots(1, len(_METRICS), figsize=(panel_width * len(_METRICS), 5.0), sharex=False)
    for ax, (column, title) in zip(axes, _METRICS):
        sns.boxplot(
            data=solved,
            x="Method",
            y=column,
            order=display_names,
            hue="Method",
            hue_order=display_names,
            palette=palette,
            legend=False,
            width=0.55,
            linewidth=1.1,
            fliersize=0,
            boxprops={"alpha": 0.85, "edgecolor": "0.25"},
            whiskerprops={"color": "0.25"},
            capprops={"color": "0.25"},
            medianprops={"color": "0.2", "linewidth": 1.6},
            ax=ax,
        )
        sns.stripplot(
            data=solved,
            x="Method",
            y=column,
            order=display_names,
            color="0.25",
            alpha=0.35,
            size=3.2,
            jitter=0.22,
            ax=ax,
        )

        ax.set_title(title, pad=10)
        ax.set_ylabel(title)
        ax.set_xlabel("")
        ax.yaxis.set_major_locator(mticker.MaxNLocator(nbins=5))
        plt.setp(ax.get_xticklabels(), rotation=22, ha="right")
        sns.despine(ax=ax, left=False, bottom=False)

    fig.suptitle("Planning cost and shortcut path length by method", y=1.03, fontsize=15, fontweight="bold")
    fig.tight_layout()
    path = output_dir / "small_multiples.pdf"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


def plot_stacked_time(df: pd.DataFrame, output_dir: Path) -> Path:
    """Median planning time + median shortcut time, stacked per method -- the total
    end-to-end wall-clock cost a user actually waits for, with each bar's total
    annotated."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = _method_palette(methods)
    solved = df[df["solved"]]

    planning = [solved.loc[solved["method"] == m, "planning_time_ms"].median() for m in methods]
    shortcut = [solved.loc[solved["method"] == m, "shortcut_time_ms"].median() for m in methods]
    totals = [p + s for p, s in zip(planning, shortcut)]

    fig, ax = plt.subplots(figsize=(1.7 * len(methods) + 2.2, 5.2))
    x = range(len(methods))
    bar_width = 0.55

    ax.bar(
        x,
        planning,
        width=bar_width,
        color=[palette[m] for m in methods],
        edgecolor="0.3",
        linewidth=1.0,
        label="Planning",
        zorder=3,
    )
    ax.bar(
        x,
        shortcut,
        width=bar_width,
        bottom=planning,
        color=[palette[m] for m in methods],
        edgecolor="0.3",
        linewidth=1.0,
        hatch="//",
        alpha=0.55,
        label="Shortcut",
        zorder=3,
    )

    for xi, total in zip(x, totals):
        ax.annotate(
            f"{total:.2f} ms",
            xy=(xi, total),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=10,
            fontweight="bold",
            color="0.2",
        )

    ax.set_xticks(list(x))
    ax.set_xticklabels(display_names, rotation=22, ha="right")
    ax.set_ylabel("Median time (ms)")
    ax.set_title("Median planning + shortcut time", pad=12, fontweight="bold")
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    ax.set_ylim(0, max(totals) * 1.18 if totals else 1)
    sns.despine(ax=ax)
    ax.grid(axis="x", visible=False)

    fig.tight_layout()
    path = output_dir / "stacked_time.pdf"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


def plot_distance_scatter(df: pd.DataFrame, output_dir: Path) -> Path:
    """Config-space vs. eef-space shortcut path distance, one point per solved trial,
    colored by method -- shows whether any method trades one distance off against the
    other rather than just comparing method-level averages."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = {name: _method_palette(methods)[m] for m, name in zip(methods, display_names)}
    solved = df[df["solved"]].copy()
    solved["Method"] = solved["method"].map(_display_name)

    fig, ax = plt.subplots(figsize=(7, 5.8))
    sns.scatterplot(
        data=solved,
        x="config_distance",
        y="eef_distance",
        hue="Method",
        hue_order=display_names,
        palette=palette,
        alpha=0.75,
        s=55,
        edgecolor="white",
        linewidth=0.6,
        ax=ax,
    )

    ax.set_xlabel("Config-space distance (rad)")
    ax.set_ylabel("EEF-space distance")
    ax.set_title("Shortcut path length: config-space vs. eef-space", pad=12, fontweight="bold")
    ax.legend(frameon=False, title=None, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    sns.despine(ax=ax)

    fig.tight_layout()
    path = output_dir / "distance_scatter.pdf"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


_TABLE_COLUMNS = [
    # (key, header, higher_is_better, fmt)
    ("success_rate", "Success (\\%)", True, "{:.1f}"),
    ("iterations", "Iterations", False, "{:.0f}"),
    ("planning_time", "Planning (ms)", False, "{:.2f}"),
    ("shortcut_time", "Shortcut (ms)", False, "{:.2f}"),
    ("total_time", "Total (ms)", False, "{:.2f}"),
    ("failure_time", "Failure (ms)", False, "{:.2f}"),
    ("config_distance", "Config dist. (rad)", False, "{:.2f}"),
    ("eef_distance", "EEF dist.", False, "{:.2f}"),
]


def _table_stats(df: pd.DataFrame) -> "pd.DataFrame":
    """Per-method median stats feeding the LaTeX table: success rate, iteration count
    and planning/shortcut/total time on solved trials, time-to-failure on failed trials
    (how long a method burns before giving up), and shortcut path length (config- and
    eef-space)."""
    methods = _method_order(df)
    rows = []
    for method in methods:
        sub = df[df["method"] == method]
        solved = sub[sub["solved"]]
        failed = sub[~sub["solved"]]
        planning = solved["planning_time_ms"].median()
        shortcut = solved["shortcut_time_ms"].median()
        rows.append(
            {
                "method": _display_name(method),
                "success_rate": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
                "iterations": solved["iterations"].median(),
                "planning_time": planning,
                "shortcut_time": shortcut,
                "total_time": planning + shortcut,
                "failure_time": failed["planning_time_ms"].median() if len(failed) else float("nan"),
                "config_distance": solved["config_distance"].median(),
                "eef_distance": solved["eef_distance"].median(),
            }
        )
    return pd.DataFrame(rows).set_index("method")


def generate_latex_table(df: pd.DataFrame, output_dir: Path) -> Path:
    """Median success rate / planning / shortcut / total / failure time and shortcut path
    length per method, as a ready-to-paste booktabs LaTeX table with the best value in
    each row bolded (highest for success rate, lowest everywhere else).

    Transposed relative to `_table_stats` (metrics as rows, methods as columns): with
    only a handful of methods this stays narrow enough to fit a single IEEE column,
    whereas one column per metric runs 8 columns wide and bleeds across the page."""
    stats = _table_stats(df)  # index: method, columns: metric key
    methods = list(stats.index)

    lines = []
    lines.append("% Auto-generated by plot_bimanual_results.py -- paste into your LaTeX source.")
    lines.append("\\begin{table}[!t]")
    lines.append("\\centering")
    lines.append("\\caption{Median planning cost and shortcut path length per method.}")
    lines.append("\\label{tab:bimanual_results}")
    col_spec = "l" + "c" * len(methods)
    lines.append(f"\\begin{{tabular}}{{{col_spec}}}")
    lines.append("\\toprule")
    header = "Metric & " + " & ".join(methods) + " \\\\"
    lines.append(header)
    lines.append("\\midrule")

    for key, display_name, higher_is_better, fmt in _TABLE_COLUMNS:
        row = stats[key]
        valid = row.dropna()
        best = (valid.max() if higher_is_better else valid.min()) if not valid.empty else None

        cells = [display_name]
        for method in methods:
            value = row[method]
            if pd.isna(value):
                cells.append("--")
                continue
            text = fmt.format(value)
            if best is not None and value == best:
                text = f"\\textbf{{{text}}}"
            cells.append(text)
        lines.append(" & ".join(cells) + " \\\\")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}")

    table_tex = "\n".join(lines) + "\n"
    path = output_dir / "results_table.tex"
    path.write_text(table_tex)
    return path, table_tex


def main(
    paths: List[str],
    labels: Optional[List[str]] = None,
    output_dir: str = "plots/bimanual_results",
    show: bool = False,
) -> None:
    """paths: one CSV per method (or a single CSV already containing multiple methods'
    rows). labels: optional override for each CSV's `method` column, same length as
    paths -- leave unset to use whatever `method` value the C++ benchmark wrote.
    show: also call plt.show() after saving, for interactive use."""
    _set_style()
    df = load_all(paths, labels)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    print_summary(df)

    small_multiples_path = plot_small_multiples(df, output_path)
    stacked_time_path = plot_stacked_time(df, output_path)
    distance_scatter_path = plot_distance_scatter(df, output_path)
    table_path, table_tex = generate_latex_table(df, output_path)

    print(f"\nSaved: {small_multiples_path} (+ .svg)")
    print(f"Saved: {stacked_time_path} (+ .svg)")
    print(f"Saved: {distance_scatter_path} (+ .svg)")
    print(f"Saved: {table_path}\n")
    print(table_tex)

    if show:
        plt.show()


if __name__ == "__main__":
    Fire(main)
