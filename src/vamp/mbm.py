"""Shared helpers for the MBM evaluation scripts: dataset loading and summary tables."""

import pickle
from pathlib import Path
from typing import Dict, List, Tuple, Union

import pandas as pd
from tabulate import tabulate

PERCENTILES = [0.25, 0.5, 0.75, 0.95]
RESOURCES_DIR = Path(__file__).resolve().parents[2] / "resources"


def load_problems(robot: str, dataset: str,
                  problem: Union[str, List[str]]) -> Tuple[Dict, List[str]]:
    """Load a pickled MBM dataset for a robot and resolve the requested problem names."""
    with open(RESOURCES_DIR / robot / dataset, 'rb') as f:
        problems = pickle.load(f)['problems']

    names = list(problems.keys())
    if isinstance(problem, str):
        problem = [problem]

    if not problem:
        problem = names
    else:
        for name in problem:
            if name not in names:
                raise RuntimeError(f"Problem `{name}` not available! Available problems: {names}")

    return problems, problem


def to_milliseconds(df: pd.DataFrame, fields: List[str]):
    for field in fields:
        df[field] = df[field].dt.total_seconds() * 1e3


def describe(df: pd.DataFrame, metrics: Dict[str, str]) -> pd.DataFrame:
    """Percentile summary of the metric columns, relabeled with display names."""
    stats = df[list(metrics)].describe(percentiles = PERCENTILES)
    stats = stats.drop(index = ["count", "min", "max"]).T
    stats.index = [metrics[m] for m in stats.index]
    return stats


def print_metric_tables(df: pd.DataFrame, problems: List[str], metrics: Dict[str, str]):
    """Per-problem and all-environment summary tables of the paper metrics."""
    for name in problems:
        sub = df[df['problem'] == name]
        if sub.empty:
            continue

        print(f"\n{name} ({len(sub)} trials)")
        print(tabulate(describe(sub, metrics), headers = 'keys', tablefmt = 'github', floatfmt = '.2f'))

    print(f"\nAll environments ({len(df)} trials)")
    print(tabulate(describe(df, metrics), headers = 'keys', tablefmt = 'github', floatfmt = '.2f'))


def print_stats_table(df: pd.DataFrame, columns: Dict[str, str]):
    """Full describe() table (mean/std/percentiles/min/max) with display-name headers."""
    stats = df[list(columns)].describe(percentiles = PERCENTILES)
    stats.drop(index = ["count"], inplace = True)
    print()
    print(tabulate(stats, headers = list(columns.values()), tablefmt = 'github'))
