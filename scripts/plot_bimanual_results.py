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

import numpy as np
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

# plot_small_multiples groups: metrics that belong together share one figure, side by side,
# rather than each getting its own standalone plot -- planning and shortcut are both "time
# to do a planning stage", config- and eef-space are both "shortcut path length", so pairing
# them makes a more direct visual comparison than two separate images. total_time_ms (see
# load_run) gets its own standalone figure instead (see _metric_groups_for) rather than
# joining this pair, so it reads as its own distribution rather than a third variation on
# "planning/shortcut time" -- it's each trial's own planning_time_ms + shortcut_time_ms,
# the honest way to see "distribution of planning+shortcutting time together" without
# visually stacking the two stages' separate distributions on top of each other (tried and
# reverted -- see plot_stacked_time's docstring for why that doesn't read cleanly).
_METRIC_GROUPS = [
    [("planning_time_ms", "Planning time (ms)"), ("shortcut_time_ms", "Shortcut time (ms)")],
    [("config_distance", "Config-space distance (rad)"), ("eef_distance", "EEF-space distance")],
]

# Metrics whose plot_small_multiples panel uses a log y-axis instead of linear -- these
# time metrics routinely span 2-3+ orders of magnitude across methods
# (e.g. a --use_smm --use_psi resolve vs. a full branch/psi search), which flattens a linear
# axis into an unreadable pile near zero with a few tall outliers. Distances don't have that
# spread, so they stay linear. plot_stacked_time's bars are NOT put on a log scale even though
# they show the same three stage metrics: stacking requires a linear axis to sum correctly
# (segment heights on a log axis don't add up to the visual total), so that plot stays linear
# by design.
_LOG_SCALE_METRICS = {"resolve_time_ms", "planning_time_ms", "shortcut_time_ms", "total_time_ms"}


def _metric_groups_for(df: pd.DataFrame) -> List[List[tuple]]:
    """_METRIC_GROUPS, with a standalone resolve_time_ms group prepended when that column
    is present (the maze problem generator's start/goal-branch resolve cost -- see
    iiwa_maze_solver_benchmark.cc) -- it has no natural partner metric, so it gets its own
    single-panel figure rather than being forced into one of the pairs above. Same for
    total_time_ms (each trial's own planning_time_ms + shortcut_time_ms -- see load_run):
    it's a derived metric in its own right, not just a third view of the planning/shortcut
    pair, so it gets its own figure too. This also means the script still runs unchanged on
    CSVs that don't have either column at all (e.g. bimanual shelf results)."""
    groups = []
    if "resolve_time_ms" in df.columns:
        groups.append([("resolve_time_ms", "Resolve time (ms)")])
    groups.extend(_METRIC_GROUPS)
    if "total_time_ms" in df.columns:
        groups.append([("total_time_ms", "Total time (ms)")])
    return groups

COLORS = ['#66c2a5','#fc8d62','#8da0cb','#e78ac3', "#a6d854","#ffd92f","#e5c494","#b3b3b3"]  # from seaborn colorblind palette

# COLORS = ["#fae515", '#d55df8']
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
    # Each solved trial's own planning_time_ms + shortcut_time_ms -- the actual per-trial
    # combined cost, not a sum of two methods' independent medians (see _METRIC_GROUPS).
    # NaN automatically on unsolved rows (shortcut_time_ms is only populated when solved).
    df["total_time_ms"] = df["planning_time_ms"] + df["shortcut_time_ms"]
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
    has_resolve = "resolve_time_ms" in df.columns
    rows = []
    for method in methods:
        sub = df[df["method"] == method]
        solved = sub[sub["solved"]]
        row = {
            "method": _display_name(method),
            "n_trials": len(sub),
            "n_solved": len(solved),
            "success_rate_%": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
        }
        if has_resolve:
            row["median_resolve_ms"] = solved["resolve_time_ms"].median()
        row.update(
            {
                "median_planning_ms": solved["planning_time_ms"].median(),
                "median_shortcut_ms": solved["shortcut_time_ms"].median(),
                "median_config_distance": solved["config_distance"].median(),
                "median_eef_distance": solved["eef_distance"].median(),
            }
        )
        rows.append(row)

    summary = pd.DataFrame(rows).set_index("method")
    with pd.option_context("display.float_format", "{:.4f}".format):
        print(summary)


def plot_small_multiples(df: pd.DataFrame, output_dir: Path, log_scale: bool = True) -> List[Path]:
    """One box plot per metric (one box per method), each its own standalone figure -- the
    primary results figures: every method's full distribution, per metric. Related metrics
    (see _METRIC_GROUPS) share one figure side by side rather than each getting a separate
    image; resolve_time_ms (no natural partner) gets its own standalone figure instead.

    log_scale (default True): use a log y-axis for metrics in _LOG_SCALE_METRICS. Pass False
    to force every panel to a plain linear axis instead."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = {name: _method_palette(methods)[m] for m, name in zip(methods, display_names)}
    solved = df[df["solved"]].copy()
    solved["Method"] = solved["method"].map(_display_name)

    groups = _metric_groups_for(df)
    # Panel width scales with method count so boxes/labels don't get crushed once there's
    # more than a handful of methods being compared.
    panel_width = max(4.4, 0.85 * len(methods) + 2.2)

    paths = []
    for group in groups:
        fig, axes = plt.subplots(1, len(group), figsize=(panel_width * len(group), 5.0), sharex=False)
        if len(group) == 1:
            axes = [axes]
        for ax, (column, title) in zip(axes, group):
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
            ax.set_title(title, pad=12, fontweight="bold")
            ax.set_ylabel(title)
            ax.set_xlabel("")
            if log_scale and column in _LOG_SCALE_METRICS:
                ax.set_yscale("log")
                ax.yaxis.set_major_locator(mticker.LogLocator(base=10.0))
                ax.yaxis.set_minor_locator(mticker.LogLocator(base=10.0, subs=tuple(range(2, 10))))
                ax.yaxis.set_minor_formatter(mticker.NullFormatter())
            else:
                ax.yaxis.set_major_locator(mticker.MaxNLocator(nbins=5))
            plt.setp(ax.get_xticklabels(), rotation=22, ha="right")
            sns.despine(ax=ax, left=False, bottom=False)

        fig.tight_layout()
        filename = "_and_".join(column for column, _ in group)
        path = output_dir / f"{filename}.pdf"
        fig.savefig(path, dpi=200, bbox_inches="tight")
        fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
        plt.close(fig)
        paths.append(path)

    return paths


def plot_success_rate(df: pd.DataFrame, output_dir: Path) -> Path:
    """Fraction of all trials each method actually solved -- the cost metrics elsewhere
    are computed only over solved trials, so this is the one plot that shows how often a
    method fails to produce a path at all."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = _method_palette(methods)
    rates = [100.0 * df.loc[df["method"] == m, "solved"].mean() for m in methods]

    fig, ax = plt.subplots(figsize=(1.6 * len(methods) + 2.0, 4.6))
    x = range(len(methods))
    ax.bar(
        x,
        rates,
        width=0.55,
        color=[palette[m] for m in methods],
        edgecolor="0.3",
        linewidth=1.0,
        zorder=3,
    )
    for xi, rate in zip(x, rates):
        ax.annotate(
            f"{rate:.1f}%",
            xy=(xi, rate),
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
    ax.set_ylabel("Success rate (%)")
    ax.set_title("Success rate by method", pad=12, fontweight="bold")
    ax.set_ylim(0, 105)
    sns.despine(ax=ax)
    ax.grid(axis="x", visible=False)

    fig.tight_layout()
    path = output_dir / "success_rate.pdf"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


_STACKED_TIME_STAGES = [
    # (csv column, legend label, hatch)
    ("resolve_time_ms", "Resolve", ".."),
    ("planning_time_ms", "Planning", None),
    ("shortcut_time_ms", "Shortcut", "//"),
]


def plot_stacked_time(df: pd.DataFrame, output_dir: Path) -> Path:
    """Median time per stage (resolve/planning/shortcut), stacked per method -- whichever
    of those columns a given method's CSV actually has. A method missing a stage (e.g. no
    resolve_time_ms, or no shortcut_time_ms because it doesn't shortcut) just contributes
    0 to that segment instead of NaN-ing out its whole bar.

    Deliberately plain median-height segments, not a full distribution per stage: stacking
    only reads cleanly when each segment is a single point value, since a segment with its
    own spread (tried here as both a violin and a box plot) balloons and visually overlaps
    the segment(s) above/below it once one stage's variance is comparable to or larger than
    another's whole height. Per-stage distributions are already covered separately by
    plot_small_multiples (see planning_time_ms/shortcut_time_ms's paired figure and
    resolve_time_ms's standalone one)."""
    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = _method_palette(methods)
    solved = df[df["solved"]]

    stages = [(col, label, hatch) for col, label, hatch in _STACKED_TIME_STAGES if col in df.columns]

    per_stage = {}
    for col, _, _ in stages:
        per_stage[col] = [
            solved.loc[solved["method"] == m, col].median() if col in solved.columns else float("nan")
            for m in methods
        ]
        # A method that never reports a stage (e.g. no shortcutting) should contribute 0
        # to the stack, not NaN out the rest of its bar.
        per_stage[col] = [0.0 if pd.isna(v) else v for v in per_stage[col]]

    totals = [sum(per_stage[col][i] for col, _, _ in stages) for i in range(len(methods))]

    fig, ax = plt.subplots(figsize=(1.7 * len(methods) + 2.2, 5.2))
    x = range(len(methods))
    bar_width = 0.55

    running_bottom = [0.0] * len(methods)
    for col, label, hatch in stages:
        values = per_stage[col]
        bar_kwargs = dict(
            width=bar_width,
            bottom=list(running_bottom),
            color=[palette[m] for m in methods],
            edgecolor="0.3",
            linewidth=1.0,
            label=label,
            zorder=3,
        )
        if hatch is not None:
            bar_kwargs.update(hatch=hatch, alpha=0.55)
        ax.bar(x, values, **bar_kwargs)
        running_bottom = [b + v for b, v in zip(running_bottom, values)]

    for xi, total in zip(x, totals):
        if pd.isna(total) or total == 0:
            continue
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
    ax.set_title("Planning time", pad=12, fontweight="bold")
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    max_total = pd.Series(totals, dtype=float).max(skipna=True)
    ax.set_ylim(0, max_total * 1.18 if pd.notna(max_total) and max_total > 0 else 1)
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


def plot_ecdf(
    df: pd.DataFrame, output_dir: Path, column: str, xlabel: str, filename: str, log_scale: bool = True
) -> Optional[Path]:
    """Empirical CDF of `column` over solved trials, one step curve per method -- lets you
    directly read off e.g. "what fraction of trials finished within X ms", and compare tail
    behavior across methods, without the binning artifacts a histogram/KDE would introduce.
    This is the companion to plot_small_multiples's box plot for the same metric: the box
    plot gives a compact 5-number summary, this gives the full shape. Log-scale x-axis for
    metrics in _LOG_SCALE_METRICS (when log_scale is True, the default), matching
    plot_small_multiples's y-axis treatment of them; pass log_scale=False to force a plain
    linear x-axis instead. Returns None (and draws nothing) if `column` isn't present or has
    no solved-trial data at all, so callers can skip it gracefully on CSVs that don't carry
    it."""
    if column not in df.columns:
        return None

    methods = _method_order(df)
    display_names = [_display_name(m) for m in methods]
    palette = {name: _method_palette(methods)[m] for m, name in zip(methods, display_names)}
    solved = df[df["solved"]]

    series_by_method = {
        name: solved.loc[solved["method"] == m, column].dropna().sort_values().to_numpy()
        for m, name in zip(methods, display_names)
    }
    if not any(values.size > 0 for values in series_by_method.values()):
        return None

    fig, ax = plt.subplots(figsize=(7.2, 5.4))

    # Subtle reference line at the 50th percentile -- gives the median markers below
    # somewhere to visually "land" and reads as a deliberate reference, not just a line plot
    # floating in empty space.
    ax.axhline(0.5, color="0.82", linestyle="--", linewidth=1.1, zorder=1)

    for i, (name, values) in enumerate(series_by_method.items()):
        if values.size == 0:
            continue
        color = palette[name]
        fractions = np.arange(1, values.size + 1) / values.size

        # Filled step area under the curve (not just a bare line) -- fills out the plot and
        # makes each method's region easy to tell apart at a glance, same color already used
        # for it everywhere else.
        ax.fill_between(values, fractions, step="post", color=color, alpha=0.18, zorder=2)
        ax.step(values, fractions, where="post", label=name, color=color, linewidth=2.2, zorder=3)

        # Marker + label at the median (the 50th-percentile point on this curve) -- ties the
        # ECDF back to the same median already reported in the table/box plots, and gives
        # each curve a concrete anchor point instead of just an outline. Labels stack
        # vertically by index (small offset, no connector line) so two methods with close
        # medians (common on a log x-axis) don't collide into unreadable overlapping text.
        median = float(np.median(values))
        ax.plot(
            median,
            0.5,
            marker="o",
            markersize=7.5,
            color=color,
            markeredgecolor="white",
            markeredgewidth=1.3,
            zorder=4,
        )
        ax.annotate(
            f"{median:.2g} ms",
            xy=(median, 0.5),
            xytext=(8, -4 + 12 * i),
            textcoords="offset points",
            fontsize=9,
            fontweight="bold",
            color=color,
            va="center",
        )

    if log_scale and column in _LOG_SCALE_METRICS:
        ax.set_xscale("log")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Fraction of solved trials $\\leq x$")
    ax.set_ylim(0, 1.02)
    ax.set_title(f"{xlabel}: empirical CDF", pad=12, fontweight="bold")
    ax.legend(frameon=False, loc="lower right")
    sns.despine(ax=ax)

    fig.tight_layout()
    path = output_dir / f"{filename}.pdf"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    fig.savefig(path.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    return path


_TABLE_COLUMNS = [
    # (key, header, higher_is_better, fmt)
    ("success_rate", "Success (\\%)", True, "{:.1f}"),
    ("iterations", "Iterations", False, "{:.0f}"),
    ("resolve_time", "Resolve (ms)", False, "{:.2f}"),
    ("planning_time", "Planning (ms)", False, "{:.2f}"),
    ("shortcut_time", "Shortcut (ms)", False, "{:.2f}"),
    ("total_time", "Total (ms)", False, "{:.2f}"),
    ("failure_time", "Failure (ms)", False, "{:.2f}"),
    ("config_distance", "Config dist. (rad)", False, "{:.2f}"),
    ("eef_distance", "EEF dist.", False, "{:.2f}"),
]


def _table_columns_for(df: pd.DataFrame) -> List[tuple]:
    """_TABLE_COLUMNS, dropping the resolve_time row on CSVs that don't carry a
    resolve_time_ms column (e.g. bimanual shelf results)."""
    if "resolve_time_ms" in df.columns:
        return _TABLE_COLUMNS
    return [c for c in _TABLE_COLUMNS if c[0] != "resolve_time"]


def _stat_triplet(key: str, series: pd.Series) -> dict:
    """{key: median, key_mean: mean, key_std: std} for a per-trial series -- median is what
    _table_columns_for's bolding compares on (unchanged from before), mean/std are shown
    alongside it in the same cell (see generate_latex_table)."""
    return {
        key: series.median(),
        f"{key}_mean": series.mean(),
        f"{key}_std": series.std(),
    }


def _table_stats(df: pd.DataFrame) -> "pd.DataFrame":
    """Per-method stats feeding the LaTeX table: median (still what "best value" bolding
    compares on) plus mean +/- std, for success rate (median/mean/std don't apply -- it's
    already one aggregate rate, not a per-trial distribution), iteration count, resolve time
    (when present), planning/shortcut/total time on solved trials, time-to-failure on failed
    trials (how long a method burns before giving up), and shortcut path length (config- and
    eef-space). total_time uses total_time_ms (see load_run -- each trial's own
    planning_time_ms + shortcut_time_ms) rather than the sum of the two stages' independent
    medians, so its median/mean/std are all drawn from the same distribution instead of the
    median being a sum of two other medians while mean/std would be trial-paired."""
    methods = _method_order(df)
    has_resolve = "resolve_time_ms" in df.columns
    rows = []
    for method in methods:
        sub = df[df["method"] == method]
        solved = sub[sub["solved"]]
        failed = sub[~sub["solved"]]

        row = {
            "method": _display_name(method),
            "success_rate": 100.0 * len(solved) / len(sub) if len(sub) else float("nan"),
        }
        row.update(_stat_triplet("iterations", solved["iterations"]))
        row.update(_stat_triplet("planning_time", solved["planning_time_ms"]))
        row.update(_stat_triplet("shortcut_time", solved["shortcut_time_ms"]))
        row.update(_stat_triplet("total_time", solved["total_time_ms"]))
        row.update(_stat_triplet("failure_time", failed["planning_time_ms"]))
        row.update(_stat_triplet("config_distance", solved["config_distance"]))
        row.update(_stat_triplet("eef_distance", solved["eef_distance"]))
        if has_resolve:
            row.update(_stat_triplet("resolve_time", solved["resolve_time_ms"]))
        rows.append(row)
    return pd.DataFrame(rows).set_index("method")


def generate_latex_table(df: pd.DataFrame, output_dir: Path) -> Path:
    """Median (with mean +/- std alongside it in the same cell, where the metric is a
    per-trial distribution -- see _stat_triplet) success rate / iterations / resolve /
    planning / shortcut / total / failure time and shortcut path length per method, as a
    ready-to-paste booktabs LaTeX table with the best value in each row bolded (highest for
    success rate, lowest everywhere else -- bolding compares medians only, same as before).

    Transposed relative to `_table_stats` (metrics as rows, methods as columns): with
    only a handful of methods this stays narrow enough to fit a single IEEE column,
    whereas one column per metric runs 8+ columns wide and bleeds across the page."""
    stats = _table_stats(df)  # index: method, columns: metric key (+ "<key>_mean"/"<key>_std")
    methods = list(stats.index)
    table_columns = _table_columns_for(df)

    lines = []
    lines.append("% Auto-generated by plot_bimanual_results.py -- paste into your LaTeX source.")
    lines.append("\\begin{table}[!t]")
    lines.append("\\centering")
    lines.append("\\caption{Median (mean $\\pm$ std) planning cost and shortcut path length per method.}")
    lines.append("\\label{tab:bimanual_results}")
    col_spec = "l" + "c" * len(methods)
    lines.append(f"\\begin{{tabular}}{{{col_spec}}}")
    lines.append("\\toprule")
    header = "Metric & " + " & ".join(methods) + " \\\\"
    lines.append(header)
    lines.append("\\midrule")

    for key, display_name, higher_is_better, fmt in table_columns:
        row = stats[key]
        valid = row.dropna()
        best = (valid.max() if higher_is_better else valid.min()) if not valid.empty else None

        mean_key, std_key = f"{key}_mean", f"{key}_std"
        has_mean_std = mean_key in stats.columns and std_key in stats.columns

        cells = [display_name]
        for method in methods:
            value = row[method]
            if pd.isna(value):
                cells.append("--")
                continue
            text = fmt.format(value)
            if has_mean_std:
                mean_value = stats.loc[method, mean_key]
                std_value = stats.loc[method, std_key]
                if pd.notna(mean_value) and pd.notna(std_value):
                    text += f" ({fmt.format(mean_value)} $\\pm$ {fmt.format(std_value)})"
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
    log_scale: bool = True,
) -> None:
    """paths: one CSV per method (or a single CSV already containing multiple methods'
    rows). labels: optional override for each CSV's `method` column, same length as
    paths -- leave unset to use whatever `method` value the C++ benchmark wrote.
    show: also call plt.show() after saving, for interactive use. log_scale: use a log
    axis for time metrics that span large ranges (see _LOG_SCALE_METRICS) in
    plot_small_multiples and the total-time ECDF -- default True; pass --log_scale=False
    for plain linear axes everywhere instead."""
    _set_style()
    df = load_all(paths, labels)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    print_summary(df)

    small_multiples_paths = plot_small_multiples(df, output_path, log_scale=log_scale)
    stacked_time_path = plot_stacked_time(df, output_path)
    total_time_ecdf_path = plot_ecdf(
        df, output_path, "total_time_ms", "Total time (ms)", "total_time_ecdf", log_scale=log_scale
    )
    distance_scatter_path = plot_distance_scatter(df, output_path)
    success_rate_path = plot_success_rate(df, output_path)
    table_path, table_tex = generate_latex_table(df, output_path)

    print()
    for path in small_multiples_paths:
        print(f"Saved: {path} (+ .svg)")
    print(f"Saved: {stacked_time_path} (+ .svg)")
    if total_time_ecdf_path is not None:
        print(f"Saved: {total_time_ecdf_path} (+ .svg)")
    print(f"Saved: {distance_scatter_path} (+ .svg)")
    print(f"Saved: {success_rate_path} (+ .svg)")
    print(f"Saved: {table_path}\n")
    print(table_tex)

    if show:
        plt.show()


if __name__ == "__main__":
    Fire(main)
