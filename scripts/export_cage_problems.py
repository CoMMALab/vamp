"""Export the Panda MBM cage problems as CSV rows (start[7], goal[7]) for cricket's ik_seed_experiment.

Usage: python scripts/export_cage_problems.py [problems.pkl [output.csv]]
"""
import pickle
import sys

problems_path = sys.argv[1] if len(sys.argv) > 1 else "resources/panda/problems.pkl"
output_path = sys.argv[2] if len(sys.argv) > 2 else "cage_problems.csv"

with open(problems_path, "rb") as f:
    problems = pickle.load(f)["problems"]["cage"]

with open(output_path, "w") as f:
    f.write("# start[7], goal[7] for each valid cage problem\n")
    for problem in problems:
        if problem["valid"]:
            f.write(",".join(repr(float(x)) for x in [*problem["start"], *problem["goals"][0]]) + "\n")

print(f"wrote {sum(p['valid'] for p in problems)} problems to {output_path}")
