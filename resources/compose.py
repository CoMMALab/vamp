"""Compose a mobile manipulator URDF from a base URDF and one or more arm URDFs in resources/.

    python compose.py                      # compose every */compose.toml under resources/
    python compose.py ridgeback_dual_panda # compose resources/ridgeback_dual_panda/compose.toml
    python compose.py path/to/compose.toml
    python compose.py --check              # fail if any generated URDF is out of date

Each composite is described by a compose.toml spec (see resources/README.md, "Composite robots").
For both variants, "" and "_spherized", the base's <base><variant>.urdf and each arm's
<model><variant>.urdf are merged into <name><variant>.urdf next to the spec. Arm links and joints
are renamed with a per-arm prefix, attached to a base link with a fixed joint, and mesh paths are
rewritten relative to the output, so nothing is copied.
"""

from __future__ import annotations

import argparse
import os
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

RESOURCES = Path(__file__).resolve().parent
SPEC_NAME = "compose.toml"
VARIANTS = ("", "_spherized")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "specs",
        nargs="*",
        help=f"Composite names (resources/<name>/{SPEC_NAME}) or spec paths; default: all specs",
    )
    parser.add_argument("--check", action="store_true", help="Report out-of-date outputs instead of writing them")
    return parser.parse_args()


def find_specs(names: list[str]) -> list[Path]:
    if not names:
        return sorted(RESOURCES.glob(f"**/{SPEC_NAME}"))
    specs = []
    for name in names:
        path = Path(name)
        spec = path if path.suffix == ".toml" else RESOURCES / name / SPEC_NAME
        if not spec.is_file():
            raise SystemExit(f"No spec found at {spec}")
        specs.append(spec.resolve())
    return specs


def model_urdf(model: str, variant: str) -> Path:
    """resources/<model>/<last path component of model><variant>.urdf, e.g. panda -> panda/panda.urdf."""
    path = RESOURCES / model / f"{Path(model).name}{variant}.urdf"
    if not path.is_file():
        raise SystemExit(f"Missing {path.relative_to(RESOURCES)} (every model needs both URDF variants)")
    return path


def fmt(values) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def rebase_meshes(robot: ET.Element, source: Path, output_dir: Path) -> None:
    """Rewrite mesh paths from relative-to-source (or package://) to relative-to-output."""
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename").removeprefix("package://")
        if not os.path.isabs(filename):
            filename = os.path.relpath(source.parent / filename, output_dir)
        mesh.set("filename", filename)


def apply_overrides(robot: ET.Element, overrides: dict, model: str) -> None:
    """Apply [overrides.<model>] joint_limits and joint_origins, keyed by the model's own joint names."""
    joints = {joint.get("name"): joint for joint in robot.findall("joint")}
    for kind in ("joint_limits", "joint_origins"):
        for name in overrides.get(kind, {}):
            if name not in joints:
                raise SystemExit(f"[overrides.{model}.{kind}] names unknown joint {name!r}")
    for name, (lower, upper) in overrides.get("joint_limits", {}).items():
        limit = joints[name].find("limit")
        if limit is None:
            raise SystemExit(f"Joint {name!r} has no <limit> to override")
        limit.set("lower", fmt([lower]))
        limit.set("upper", fmt([upper]))
    for name, values in overrides.get("joint_origins", {}).items():
        origin = joints[name].find("origin")
        if origin is None:
            origin = ET.SubElement(joints[name], "origin")
        for key in ("xyz", "rpy"):
            if key in values:
                origin.set(key, fmt(values[key]))


def rename(robot: ET.Element, prefix: str, strip_prefix: str) -> None:
    """Prefix every link, joint, and material name (after removing strip_prefix) and their references."""

    def new(name: str) -> str:
        return prefix + name.removeprefix(strip_prefix)

    for el in robot.iter():
        if el.tag in ("link", "joint", "material") and "name" in el.attrib:
            el.set("name", new(el.get("name")))
        elif el.tag in ("parent", "child"):
            el.set("link", new(el.get("link")))
        elif el.tag == "mimic":
            el.set("joint", new(el.get("joint")))


def root_link(robot: ET.Element, source: Path) -> str:
    children = {joint.find("child").get("link") for joint in robot.findall("joint")}
    roots = [link.get("name") for link in robot.findall("link") if link.get("name") not in children]
    if len(roots) != 1:
        raise SystemExit(f"{source.relative_to(RESOURCES)} must have exactly one root link, found {roots}")
    return roots[0]


def fixed_joint(robot: ET.Element, name: str, parent: str, child: str, xyz, rpy) -> None:
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)


def validate(robot: ET.Element, spec_path: Path) -> None:
    for tag in ("link", "joint"):
        names = [el.get("name") for el in robot.findall(tag)]
        if duplicates := sorted({n for n in names if names.count(n) > 1}):
            raise SystemExit(f"{spec_path}: duplicate {tag} names {duplicates}; use distinct arm prefixes")
    links = {link.get("name") for link in robot.findall("link")}
    for joint in robot.findall("joint"):
        for end in ("parent", "child"):
            if joint.find(end).get("link") not in links:
                raise SystemExit(f"{spec_path}: joint {joint.get('name')!r} {end} link does not exist")


def compose(spec_path: Path, spec: dict, variant: str) -> str:
    output_dir = spec_path.parent
    base_file = model_urdf(spec["base"], variant)
    robot = ET.parse(base_file).getroot()
    robot.set("name", spec["name"])
    rebase_meshes(robot, base_file, output_dir)
    apply_overrides(robot, spec.get("overrides", {}).get(spec["base"], {}), spec["base"])
    base_links = {link.get("name") for link in robot.findall("link")}

    sources = [base_file]
    for arm in spec.get("arms", []):
        arm_file = model_urdf(arm["model"], variant)
        sources.append(arm_file)
        if arm["parent"] not in base_links:
            raise SystemExit(f"{spec_path}: arm parent link {arm['parent']!r} is not in {spec['base']}")
        arm_robot = ET.parse(arm_file).getroot()
        rebase_meshes(arm_robot, arm_file, output_dir)
        apply_overrides(arm_robot, spec.get("overrides", {}).get(arm["model"], {}), arm["model"])
        rename(arm_robot, arm["prefix"], arm.get("strip_prefix", ""))
        mount_joint = arm.get("joint", f"{arm['prefix']}mount_joint")
        fixed_joint(
            robot, mount_joint, arm["parent"], root_link(arm_robot, arm_file), arm.get("xyz", [0, 0, 0]), arm.get("rpy", [0, 0, 0])
        )
        robot.extend(arm_robot)

    for frame in spec.get("frames", []):
        ET.SubElement(robot, "link", name=frame["name"])
        fixed_joint(
            robot,
            frame.get("joint", f"{frame['name']}_joint"),
            frame["parent"],
            frame["name"],
            frame.get("xyz", [0, 0, 0]),
            frame.get("rpy", [0, 0, 0]),
        )

    validate(robot, spec_path)
    inputs = ", ".join(dict.fromkeys(os.path.relpath(source, output_dir) for source in sources))
    robot.insert(0, ET.Comment(f" Generated by resources/compose.py from {spec_path.name} ({inputs}). Do not edit. "))
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    return ET.tostring(robot, encoding="unicode", xml_declaration=True) + "\n"


def main() -> None:
    args = parse_args()
    stale = []
    for spec_path in find_specs(args.specs):
        spec = tomllib.loads(spec_path.read_text())
        for variant in VARIANTS:
            output = spec_path.parent / f"{spec['name']}{variant}.urdf"
            text = compose(spec_path, spec, variant)
            label = output.relative_to(RESOURCES) if output.is_relative_to(RESOURCES) else output
            if args.check:
                if not output.is_file() or output.read_text() != text:
                    stale.append(label)
            else:
                output.write_text(text)
                print(f"Wrote {label}")
    if stale:
        print("Out of date (rerun compose.py):", *stale, sep="\n  ")
        sys.exit(1)
    if args.check:
        print("All composites are up to date.")


if __name__ == "__main__":
    main()
