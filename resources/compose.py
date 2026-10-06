"""Compose a mobile manipulator URDF and SRDF from a base and one or more arms in resources/.

    python compose.py                      # compose every */compose.toml under resources/
    python compose.py ridgeback_dual_panda # compose resources/ridgeback_dual_panda/compose.toml
    python compose.py path/to/compose.toml
    python compose.py --check              # fail if any generated file is out of date

Each composite is described by a compose.toml spec (see resources/README.md, "Composite robots").
For both variants, "" and "_spherized", the base's <base><variant>.urdf and each arm's
<model><variant>.urdf are merged into <name><variant>.urdf next to the spec. Arm links and joints
are renamed with a per-arm prefix, attached to a base link with a fixed joint, and mesh paths are
rewritten relative to the output, so nothing is copied. The models' <model>.srdf files are merged
the same way into <name>.srdf, plus the arm-to-base collision pairs the spec disables.
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


def model_srdf(model: str) -> Path:
    path = RESOURCES / model / f"{Path(model).name}.srdf"
    if not path.is_file():
        raise SystemExit(f"Missing {path.relative_to(RESOURCES)} (every model needs an SRDF)")
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


def compose_urdf(spec_path: Path, spec: dict, variant: str) -> tuple[ET.Element, list[Path]]:
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
    return robot, sources


# SRDF element -> attributes holding link, joint, or group names, all renamed with an arm's prefix.
SRDF_NAME_ATTRIBUTES = {
    "group": ("name",),
    "link": ("name",),
    "joint": ("name",),
    "passive_joint": ("name",),
    "chain": ("base_link", "tip_link"),
    "group_state": ("group",),
    "end_effector": ("name", "group", "parent_group", "parent_link"),
    "disable_collisions": ("link1", "link2"),
}
SRDF_LINK_ATTRIBUTES = {
    "link": ("name",),
    "chain": ("base_link", "tip_link"),
    "end_effector": ("parent_link",),
    "disable_collisions": ("link1", "link2"),
    "virtual_joint": ("child_link",),
}


def collision_links(urdf: Path) -> list[str]:
    return [link.get("name") for link in ET.parse(urdf).getroot().findall("link") if link.find("collision") is not None]


def compose_srdf(spec_path: Path, spec: dict, urdf: ET.Element) -> tuple[ET.Element, list[Path]]:
    srdf = ET.Element("robot", name=spec["name"])
    base_file = model_srdf(spec["base"])
    srdf.extend(ET.parse(base_file).getroot())
    base_collision_links = collision_links(model_urdf(spec["base"], ""))

    sources = [base_file]
    for arm in spec.get("arms", []):
        arm_file = model_srdf(arm["model"])
        sources.append(arm_file)
        prefix, strip_prefix = arm["prefix"], arm.get("strip_prefix", "")
        arm_srdf = ET.parse(arm_file).getroot()
        for el in arm_srdf.iter():
            for attribute in SRDF_NAME_ATTRIBUTES.get(el.tag, ()):
                if attribute in el.attrib:
                    el.set(attribute, prefix + el.get(attribute).removeprefix(strip_prefix))
        # A mounted arm is not a floating robot, so its virtual joint does not apply.
        srdf.extend(el for el in arm_srdf if el.tag != "virtual_joint")
        for link in arm.get("disable_with_base", []):
            for base_link in base_collision_links:
                ET.SubElement(
                    srdf,
                    "disable_collisions",
                    link1=prefix + link.removeprefix(strip_prefix),
                    link2=base_link,
                    reason="Mount",
                )
    for pair in spec.get("disable_collisions", []):
        ET.SubElement(
            srdf, "disable_collisions", link1=pair["link1"], link2=pair["link2"], reason=pair.get("reason", "Never")
        )

    # Drop repeated pairs, then check every name refers to the composed URDF.
    seen = set()
    for el in list(srdf.findall("disable_collisions")):
        key = frozenset((el.get("link1"), el.get("link2")))
        if key in seen:
            srdf.remove(el)
        seen.add(key)
    links = {link.get("name") for link in urdf.findall("link")}
    joints = {joint.get("name") for joint in urdf.findall("joint")}
    for el in srdf.iter():
        for attribute in SRDF_LINK_ATTRIBUTES.get(el.tag, ()):
            if attribute in el.attrib and el.get(attribute) not in links:
                raise SystemExit(f"{spec_path}: SRDF <{el.tag} {attribute}={el.get(attribute)!r}> is not a URDF link")
        if el.tag in ("joint", "passive_joint") and el.get("name") not in joints:
            raise SystemExit(f"{spec_path}: SRDF <{el.tag} name={el.get('name')!r}> is not a URDF joint")
    return srdf, sources


def serialize(root: ET.Element, spec_path: Path, sources: list[Path]) -> str:
    inputs = ", ".join(dict.fromkeys(os.path.relpath(source, spec_path.parent) for source in sources))
    root.insert(0, ET.Comment(f" Generated by resources/compose.py from {spec_path.name} ({inputs}). Do not edit. "))
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def main() -> None:
    args = parse_args()
    stale = []
    for spec_path in find_specs(args.specs):
        spec = tomllib.loads(spec_path.read_text())
        outputs = {}
        for variant in VARIANTS:
            urdf, sources = compose_urdf(spec_path, spec, variant)
            if variant == "":
                srdf, srdf_sources = compose_srdf(spec_path, spec, urdf)
                outputs[f"{spec['name']}.srdf"] = serialize(srdf, spec_path, srdf_sources)
            outputs[f"{spec['name']}{variant}.urdf"] = serialize(urdf, spec_path, sources)
        for filename, text in sorted(outputs.items()):
            output = spec_path.parent / filename
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
