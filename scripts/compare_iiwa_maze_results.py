"""Compare the C++ (scripts/cpp/iiwa_maze_solver_benchmark.cc) and python
(scripts/iiwa_marker_maze_example.py) task-space maze benchmark results.

Both write the same result schema (list of per-problem dicts keyed by problem_index,
with path/shortcut_path costs and timing) to their own json file, so this loads both,
matches entries by problem_index, and reports where they agree/disagree. Exact numeric
equality isn't expected -- the two runs consume their Halton sequences differently (the
python side pins psi from the problem file directly, same as the C++ benchmark loop, but
JIT/optimization differences between the two binaries can still nudge iteration counts)
-- so this focuses on: which problems solve in one but not the other, and how far apart
the solved set's costs/timings are in aggregate.

Usage:
    python scripts/compare_iiwa_maze_results.py
    python scripts/compare_iiwa_maze_results.py --cpp_results path/to/cpp.json \\
        --python_results path/to/python.json --cost_tolerance 0.2
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd
from fire import Fire

RESOURCES = Path(__file__).parents[1] / "resources" / "iiwa_marker"
DEFAULT_CPP_RESULTS = RESOURCES / "maze_solver_benchmark_paths.json"
DEFAULT_PYTHON_RESULTS = RESOURCES / "maze_solver_benchmark_paths_python.json"

# Fields compared per solved-in-both problem: (json key, human label).
COMPARED_FIELDS = [
    ("iterations", "iterations"),
    ("nanoseconds", "time (ms)"),
    ("path_ambient_distance", "raw path ambient distance"),
    ("path_se3_distance", "raw path SE3 distance"),
    ("shortcut_path_ambient_distance", "shortcut path ambient distance"),
    ("shortcut_path_se3_distance", "shortcut path SE3 distance"),
]


def load_by_index(path: str) -> dict:
    with open(path) as f:
        entries = json.load(f)
    return {e["problem_index"]: e for e in entries}


def main(
    cpp_results: str = str(DEFAULT_CPP_RESULTS),
    python_results: str = str(DEFAULT_PYTHON_RESULTS),
    cost_tolerance: float = 0.25,  # Flag a problem if a cost field's relative diff exceeds this.
):
    cpp = load_by_index(cpp_results)
    py = load_by_index(python_results)

    cpp_indices = set(cpp.keys())
    py_indices = set(py.keys())
    both = sorted(cpp_indices & py_indices)
    cpp_only = sorted(cpp_indices - py_indices)
    py_only = sorted(py_indices - cpp_indices)

    print(f"C++ results:    {cpp_results} ({len(cpp)} solved problems)")
    print(f"python results: {python_results} ({len(py)} solved problems)")
    print(f"solved in both: {len(both)}")
    print(f"solved in C++ only:    {len(cpp_only)}{' -> ' + str(cpp_only) if cpp_only else ''}")
    print(f"solved in python only: {len(py_only)}{' -> ' + str(py_only) if py_only else ''}")

    if not both:
        print("\nNo problems solved in both runs -- nothing further to compare.")
        return

    rows = []
    for idx in both:
        c, p = cpp[idx], py[idx]
        row = {"problem_index": idx}
        for key, _ in COMPARED_FIELDS:
            row[f"cpp_{key}"] = c[key]
            row[f"py_{key}"] = p[key]
        row["cpp_path_size"] = len(c["path"])
        row["py_path_size"] = len(p["path"])
        row["cpp_shortcut_path_size"] = len(c["shortcut_path"])
        row["py_shortcut_path_size"] = len(p["shortcut_path"])
        rows.append(row)
    df = pd.DataFrame(rows).set_index("problem_index")

    print(f"\n--- summary over {len(both)} commonly-solved problems ---")
    summary_rows = []
    flagged = {}
    for key, label in COMPARED_FIELDS:
        cpp_vals = df[f"cpp_{key}"].to_numpy(dtype=np.float64)
        py_vals = df[f"py_{key}"].to_numpy(dtype=np.float64)
        # Relative diff against the larger of the two, so a solved-with-zero-cost edge
        # case doesn't divide by zero.
        denom = np.maximum(np.abs(cpp_vals), np.abs(py_vals))
        denom = np.where(denom == 0.0, 1.0, denom)
        rel_diff = np.abs(cpp_vals - py_vals) / denom

        scale = 1e6 if key == "nanoseconds" else 1.0  # ns -> ms for display
        summary_rows.append({
            "field": label,
            "cpp_mean": cpp_vals.mean() / scale,
            "py_mean": py_vals.mean() / scale,
            "mean_rel_diff": rel_diff.mean(),
            "max_rel_diff": rel_diff.max(),
        })

        over = df.index[rel_diff > cost_tolerance]
        if len(over):
            flagged[label] = list(over)

    summary = pd.DataFrame(summary_rows).set_index("field")
    with pd.option_context("display.float_format", "{:.4f}".format):
        print(summary)

    path_size_diff = (df["cpp_path_size"] - df["py_path_size"]).abs()
    shortcut_size_diff = (df["cpp_shortcut_path_size"] - df["py_shortcut_path_size"]).abs()
    print(f"\nraw path size:      cpp mean {df['cpp_path_size'].mean():.1f}, "
          f"py mean {df['py_path_size'].mean():.1f}, mean |diff| {path_size_diff.mean():.2f}")
    print(f"shortcut path size: cpp mean {df['cpp_shortcut_path_size'].mean():.1f}, "
          f"py mean {df['py_shortcut_path_size'].mean():.1f}, mean |diff| {shortcut_size_diff.mean():.2f}")

    if flagged:
        print(f"\nproblems exceeding {cost_tolerance:.0%} relative difference on at least one field:")
        for label, indices in flagged.items():
            print(f"  {label}: {indices}")
    else:
        print(f"\nno problem exceeded {cost_tolerance:.0%} relative difference on any compared field.")


if __name__ == "__main__":
    Fire(main)
