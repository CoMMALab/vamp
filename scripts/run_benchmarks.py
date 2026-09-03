#!/usr/bin/env python3
"""Run one or more vamp C++ benchmarks and stash a versioned copy of their output.

Each invocation gets one run-id derived from the current git state (HEAD sha,
plus a "-dirty-<sha>" suffix if the working tree differs from HEAD -- see
below) and a UTC timestamp. All selected benchmarks from this invocation land
under results/<run-id>/<target>/, alongside a top-level metadata.json
recording exactly what was run and against which git state.

Dirty-tree versioning: rather than hashing `git diff` output by hand, this
snapshots the full working tree (tracked changes *and* untracked files, minus
anything gitignored) into a real commit object via low-level plumbing
(`add`/`write-tree`/`commit-tree` against a scratch index file, never the
real one) and keeps it reachable under refs/benchmark-snapshots/<sha>. It
never touches the real working tree or index -- unlike `git stash push` --
so there's no risk of a pop leaving things stashed away on a crash. The
commit is fully recoverable later with `git show <sha>` / `git diff <sha>` /
`git checkout <sha> -- .`, unlike a hand-hashed diff file, and two runs over
the same uncommitted changes collapse to the same tag for free.

Usage:
    scripts/run_benchmarks.py                      # run every known target
    scripts/run_benchmarks.py iiwa_maze bimanual_iiwa
    scripts/run_benchmarks.py --list
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

VAMP_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BUILD_DIR = VAMP_ROOT / "build"
DEFAULT_RESULTS_DIR = VAMP_ROOT / "results"


# Registry of runnable benchmarks. `binary` is the executable name under the
# build dir. `args` builds the argv list given this target's own output
# directory (already created); omit / return [] for binaries that only print
# stats to stdout. `extra_inputs` are resource files worth recording the
# mtime/size of in metadata.json, since they can silently change a run's
# result independent of the git diff of tracked source files.
TARGETS = {
    "iiwa_maze": {
        "binary": "vamp_iiwa_maze_solver_benchmark",
        "args": lambda run_dir: [str(run_dir / "maze_solver_benchmark_paths.json")],
        "extra_inputs": [
            "resources/environments/real_maze.json",
            "resources/iiwa_marker/maze_problems_checked_ik.json",
        ],
    },
    "iiwa_maze_tsr": {
        "binary": "vamp_iiwa_maze_tsr_benchmark",
        "args": lambda run_dir: [str(run_dir / "maze_solver_mcvamp_benchmark_paths.json")],
        "extra_inputs": [
            "resources/environments/real_maze.json",
            "resources/iiwa_marker/maze_problems_checked_ik.json",
        ],
    },
    # "iiwa_branch_selector": {
    #     "binary": "vamp_iiwa_branch_selector",
    #     "args": lambda run_dir: [],
    #     "extra_inputs": ["resources/environments/real_maze.json"],
    # },
    # "bimanual_iiwa": {
    #     "binary": "vamp_bimanual_iiwa_leader_follower_shelf",
    #     "args": lambda run_dir: [],
    #     "extra_inputs": [],
    # },
    # "rby1_task_space": {
    #     "binary": "vamp_rby1_task_space_planner",
    #     "args": lambda run_dir: [
    #         "resources/ruby/problem_set_skipped_intermediate.json",
    #         str(run_dir / "rby1_task_space_results.json"),
    #     ],
    #     "extra_inputs": ["resources/ruby/problem_set_skipped_intermediate.json"],
    # },
    # "rby1_branch_selector": {
    #     "binary": "vamp_rby1_gcp_branch_selector",
    #     "args": lambda run_dir: [],
    #     "extra_inputs": [],
    # },
}


def run_git(args: list[str]) -> str:
    return subprocess.run(
        ["git", *args], cwd=VAMP_ROOT, check=True, capture_output=True, text=True
    ).stdout.strip()


def is_git_repo() -> bool:
    return (
        subprocess.run(
            ["git", "rev-parse", "--is-inside-work-tree"],
            cwd=VAMP_ROOT, capture_output=True, text=True,
        ).returncode
        == 0
    )


def snapshot_dirty_tree(head_sha: str) -> str | None:
    """Commit the full working tree (tracked changes + untracked, non-ignored
    files) as a real git object, entirely via a scratch index file so the
    real working tree and index are never touched. Returns the commit sha, or
    None if the tree exactly matches HEAD. The commit is kept reachable under
    refs/benchmark-snapshots/<sha> so it survives gc."""
    git_dir = Path(run_git(["rev-parse", "--git-dir"]))
    if not git_dir.is_absolute():
        git_dir = VAMP_ROOT / git_dir

    with tempfile.TemporaryDirectory() as tmp:
        scratch_index = Path(tmp) / "index"
        real_index = git_dir / "index"
        if real_index.exists():
            shutil.copy(real_index, scratch_index)

        env = {"GIT_INDEX_FILE": str(scratch_index)}
        subprocess.run(
            ["git", "add", "-A"], cwd=VAMP_ROOT, check=True,
            capture_output=True, text=True, env={**os.environ, **env},
        )
        tree_sha = subprocess.run(
            ["git", "write-tree"], cwd=VAMP_ROOT, check=True,
            capture_output=True, text=True, env={**os.environ, **env},
        ).stdout.strip()

    if tree_sha == run_git(["rev-parse", f"{head_sha}^{{tree}}"]):
        return None

    commit_sha = run_git(
        ["commit-tree", tree_sha, "-p", head_sha, "-m", "benchmark run snapshot (tracked + untracked changes)"]
    )
    run_git(["update-ref", f"refs/benchmark-snapshots/{commit_sha}", commit_sha])
    return commit_sha


def git_state() -> dict:
    """Capture HEAD sha and, if the tree is dirty, snapshot it so it's
    recoverable later without ever touching the working tree/index or
    requiring a real commit on the current branch. If this tree isn't a git
    repo at all (e.g. a container image built from a source copy with no
    .git), degrades to a timestamp-only run-id instead of failing outright --
    there's simply nothing to version against."""
    if not is_git_repo():
        return {
            "head_sha": None,
            "head_short_sha": None,
            "dirty_sha": None,
            "dirty_short_sha": None,
            "dirty_status": None,
            "note": "not a git repository -- run-id has no git provenance",
        }

    head_sha = run_git(["rev-parse", "HEAD"])
    head_short = run_git(["rev-parse", "--short", "HEAD"])

    dirty_sha = snapshot_dirty_tree(head_sha)
    dirty_short = run_git(["rev-parse", "--short", dirty_sha]) if dirty_sha else None
    porcelain = run_git(["status", "--porcelain"]) or None

    return {
        "head_sha": head_sha,
        "head_short_sha": head_short,
        "dirty_sha": dirty_sha,
        "dirty_short_sha": dirty_short,
        "dirty_status": porcelain if porcelain else None,
    }


def make_run_id(state: dict, timestamp: str) -> str:
    tag = state["head_short_sha"] or "nogit"
    if state["dirty_short_sha"]:
        tag += f"-dirty-{state['dirty_short_sha']}"
    return f"{tag}_{timestamp}"


def input_fingerprint(paths: list[str]) -> dict:
    out = {}
    for rel in paths:
        p = VAMP_ROOT / rel
        if p.exists():
            st = p.stat()
            out[rel] = {"size": st.st_size, "mtime": st.st_mtime}
        else:
            out[rel] = None
    return out


def build_target(name: str, spec: dict, build_dir: Path, target_dir: Path) -> str | None:
    """Build (or rebuild, if stale) this target's binary via cmake so the
    binary in build_dir always matches the current source before we run it --
    otherwise a stale binary silently ignores source changes (e.g. the argv
    output-path override) with no error. Returns an error string on failure,
    else None."""
    cmd = ["cmake", "--build", str(build_dir), "--target", spec["binary"]]
    print(f"[{name}] building: {' '.join(cmd)}")
    build_log_path = target_dir / "build.log"
    proc = subprocess.run(cmd, cwd=VAMP_ROOT, capture_output=True, text=True)
    build_log_path.write_text(proc.stdout + proc.stderr)
    if proc.returncode != 0:
        print(f"[{name}] BUILD FAILED (exit {proc.returncode}) -- see {build_log_path}")
        return f"cmake --build failed (exit {proc.returncode}); see {build_log_path}"
    return None


def run_target(name: str, spec: dict, build_dir: Path, run_dir: Path, skip_build: bool) -> dict:
    binary = build_dir / spec["binary"]
    target_dir = run_dir / name
    target_dir.mkdir(parents=True, exist_ok=True)

    if not skip_build:
        build_error = build_target(name, spec, build_dir, target_dir)
        if build_error:
            return {"status": "failed", "reason": build_error}

    if not binary.exists():
        print(f"[{name}] SKIPPED -- binary not found: {binary}")
        return {"status": "skipped", "reason": f"binary not found: {binary}"}

    args = spec["args"](target_dir)
    cmd = [str(binary), *args]
    print(f"[{name}] running: {' '.join(cmd)}")

    summary_path = target_dir / "summary.txt"
    start = time.time()
    with open(summary_path, "w") as summary_file:
        proc = subprocess.run(cmd, cwd=VAMP_ROOT, stdout=summary_file, stderr=subprocess.STDOUT)
    duration = time.time() - start

    status = "ok" if proc.returncode == 0 else "failed"
    print(f"[{name}] {status} in {duration:.1f}s (exit {proc.returncode}) -> {target_dir}")

    return {
        "status": status,
        "returncode": proc.returncode,
        "duration_seconds": duration,
        "command": cmd,
        "output_dir": str(target_dir.relative_to(run_dir.parent)),
        "input_fingerprint": input_fingerprint(spec.get("extra_inputs", [])),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("targets", nargs="*", help="Target names to run (default: all)")
    parser.add_argument("--list", action="store_true", help="List available targets and exit")
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR)
    parser.add_argument(
        "--skip-build", action="store_true",
        help="Don't cmake --build before running; use whatever binary is already in --build-dir as-is",
    )
    args = parser.parse_args()

    if args.list:
        for name, spec in TARGETS.items():
            print(f"{name:24s} ({spec['binary']})")
        return 0

    selected = args.targets or list(TARGETS.keys())
    unknown = [t for t in selected if t not in TARGETS]
    if unknown:
        print(f"Unknown target(s): {', '.join(unknown)}", file=sys.stderr)
        print(f"Available: {', '.join(TARGETS.keys())}", file=sys.stderr)
        return 1

    state = git_state()
    timestamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id = make_run_id(state, timestamp)
    run_dir = args.results_dir / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"Run id: {run_id}")
    if state.get("note"):
        print(f"  ({state['note']})")
    elif state["dirty_sha"]:
        print(f"  (working tree is dirty; snapshot committed as {state['dirty_sha']}"
              f" -- recover with `git show {state['dirty_sha']}` or `git checkout {state['dirty_sha']} -- .`)")

    results = {}
    for name in selected:
        results[name] = run_target(name, TARGETS[name], args.build_dir, run_dir, args.skip_build)

    metadata = {
        "run_id": run_id,
        "timestamp_utc": timestamp,
        "git": state,
        "build_dir": str(args.build_dir),
        "targets": results,
    }
    metadata_path = run_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2))
    print(f"\nWrote run metadata to {metadata_path}")

    return 0 if all(r["status"] != "failed" for r in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
