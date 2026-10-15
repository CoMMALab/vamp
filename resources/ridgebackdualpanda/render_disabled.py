"""Render the spherized collision geometry, highlighting the links compose.toml excludes from checks.

    python render_disabled.py [--output ridgeback_dual_panda_disabled_pairs.png]

Each arm link listed in disable_with_base gets its own color (shared by every arm), and the base
links those are paired with share one color; extra [[disable_collisions]] links get another. All
other collision spheres are grey. Two views of the robot at robosuite's home pose.
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

import compose  # noqa: E402
import render  # noqa: E402

BASE_RGBA = (0.2, 0.45, 0.95, 0.2)
EXTRA_RGBA = (0.1, 0.8, 0.8, 0.9)
OTHER_RGBA = (0.7, 0.7, 0.7, 0.18)
ARM_RGBAS = [(1.0, 0.55, 0.0, 0.95), (0.55, 0.9, 0.1, 0.95), (0.9, 0.2, 0.75, 0.95), (1.0, 0.9, 0.1, 0.95)]
VIEWS = [(135, -20), (90, -5)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spec", type=Path, default=HERE / compose.SPEC_NAME, help="compose.toml to read")
    parser.add_argument("--output", type=Path, help="PNG to write (default: <name>_disabled_pairs.png next to the spec)")
    parser.add_argument("--width", type=int, default=800, help="Width of each panel in pixels (default: 800)")
    parser.add_argument("--height", type=int, default=600, help="Height in pixels (default: 600)")
    return parser.parse_args()


def link_colors(spec: dict) -> tuple[dict[str, tuple], list[tuple[str, tuple]]]:
    """Map highlighted link names to colors, and build the legend."""
    colors, legend = {}, []
    base_links = compose.collision_links(compose.model_urdf(spec["base"], "_spherized"))
    arm_links: dict[str, tuple] = {}
    for arm in spec.get("arms", []):
        strip_prefix = arm.get("strip_prefix", "")
        for link in arm.get("disable_with_base", []):
            local = link.removeprefix(strip_prefix)
            if local not in arm_links:
                arm_links[local] = ARM_RGBAS[len(arm_links) % len(ARM_RGBAS)]
            colors[arm["prefix"] + local] = arm_links[local]
    for local, rgba in arm_links.items():
        legend.append((f"arm *_{local} (disable_with_base)", rgba))
    if arm_links:
        for link in base_links:
            colors[link] = BASE_RGBA
        legend.append((f"base links paired with them: {', '.join(base_links)}", BASE_RGBA))
    extra = {name for pair in spec.get("disable_collisions", []) for name in (pair["link1"], pair["link2"])}
    for link in extra:
        colors.setdefault(link, EXTRA_RGBA)
    if extra:
        legend.append(("[[disable_collisions]] links", EXTRA_RGBA))
    legend.append(("all other collision spheres (checked)", OTHER_RGBA))
    return colors, legend


def main() -> None:
    args = parse_args()
    spec_path = args.spec.expanduser().resolve()
    spec = tomllib.loads(spec_path.read_text())
    urdf = spec_path.parent / f"{spec['name']}_spherized.urdf"
    if not urdf.is_file():
        raise SystemExit(f"{urdf} not found; run compose.py first")
    colors, legend = link_colors(spec)

    model = render.load_urdf(urdf)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    missing = set(colors)
    for geom in range(model.ngeom):
        if model.geom_group[geom] != 2:
            continue
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom])
        model.geom_rgba[geom] = colors.get(body, OTHER_RGBA)
        missing.discard(body)
    if missing:
        print(f"Warning: highlighted links without collision geometry: {sorted(missing)}")

    data = mujoco.MjData(model)
    data.qpos[:] = render.home_qpos(model)
    mujoco.mj_forward(model, data)
    option = mujoco.MjvOption()
    option.geomgroup[:3] = [1, 0, 1]  # floor and collision spheres, no visual meshes

    panels = []
    renderer = mujoco.Renderer(model, args.height, args.width)
    try:
        for azimuth, elevation in VIEWS:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [0.15, 0, 0.55]
            camera.distance = 2.8
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera, option)
            panels.append(renderer.render())
    finally:
        renderer.close()

    image = Image.fromarray(np.concatenate(panels, axis=1))
    draw = ImageDraw.Draw(image)
    draw.text((12, 10), f"{urdf.name}: links excluded from self-collision checks ({spec_path.name})", fill=(255, 255, 255))
    y = args.height - 18 * len(legend) - 12
    for label, rgba in legend:
        draw.rectangle((12, y, 26, y + 12), fill=tuple(int(255 * c) for c in rgba[:3]), outline=(255, 255, 255))
        draw.text((34, y), label, fill=(255, 255, 255))
        y += 18

    output = (args.output or spec_path.parent / f"{spec['name']}_disabled_pairs.png").expanduser().resolve()
    image.save(output)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
