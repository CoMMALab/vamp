"""Render ridgeback_dual_panda.urdf and ridgeback_dual_panda_spherized.urdf side by side.

    python render.py [--output ridgeback_dual_panda.png]

Left: the mesh URDF's visual geometry. Right: the spherized URDF's collision spheres over its
visual geometry. Both are posed at robosuite's RidgebackDualPanda home configuration. Rendering is
offscreen; without a display, EGL is used.
"""

from __future__ import annotations

import argparse
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

if "DISPLAY" not in os.environ:
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
PANELS = {
    "ridgeback_dual_panda.urdf": "visual meshes",
    "ridgeback_dual_panda_spherized.urdf": "collision spheres",
}
# robosuite's RidgebackDualPanda.init_qpos for each arm.
PANDA_HOME = [0, np.pi / 16, 0, -5 * np.pi / 6, 0, np.pi - 0.2, np.pi / 4]
SPHERE_RGBA = (0.9, 0.15, 0.15, 0.45)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=HERE / "ridgeback_dual_panda.png", help="PNG to write")
    parser.add_argument("--width", type=int, default=800, help="Width of each panel in pixels (default: 800)")
    parser.add_argument("--height", type=int, default=600, help="Height in pixels (default: 600)")
    parser.add_argument("--azimuth", type=float, default=135, help="Camera azimuth in degrees (default: 135)")
    parser.add_argument("--elevation", type=float, default=-20, help="Camera elevation in degrees (default: -20)")
    return parser.parse_args()


def load_urdf(path: Path) -> mujoco.MjModel:
    """Load a URDF into MuJoCo with visuals kept, plus a light and a floor."""
    root = ET.fromstring(path.read_text())
    for visual in root.iter("visual"):
        visual.attrib.pop("name", None)  # duplicate names only produce importer warnings
    ET.SubElement(
        ET.SubElement(root, "mujoco"),
        "compiler",
        meshdir=str(path.parent),
        strippath="false",
        fusestatic="false",
        discardvisual="false",
        boundmass="1e-6",  # the base's intermediate joint links are massless
        boundinertia="1e-9",
    )
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    spec.worldbody.add_light(pos=[1, -1, 4], dir=[-0.25, 0.25, -1], diffuse=[0.8, 0.8, 0.8])
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[3, 3, 0.05],
        pos=[0, 0, -0.0262],  # robosuite spawns robot0_base 0.0262 m above the floor
        rgba=[0.35, 0.4, 0.45, 1],
        contype=0,
        conaffinity=0,
    )
    model = spec.compile()

    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for geom in range(model.ngeom):
        if geom == floor:
            continue
        if model.geom_contype[geom] or model.geom_conaffinity[geom]:
            model.geom_group[geom] = 2
            model.geom_rgba[geom] = SPHERE_RGBA
        else:
            model.geom_group[geom] = 1
    return model


def home_qpos(model: mujoco.MjModel) -> np.ndarray:
    qpos = np.zeros(model.nq)
    for jnt in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt)
        match = re.search(r"(?:right|left)_joint(\d)$", name)
        value = PANDA_HOME[int(match.group(1)) - 1] if match else 0.0
        if model.jnt_limited[jnt]:
            value = np.clip(value, *model.jnt_range[jnt])
        qpos[model.jnt_qposadr[jnt]] = value
    return qpos


def render(path: Path, show_spheres: bool, args: argparse.Namespace) -> np.ndarray:
    model = load_urdf(path)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    data.qpos[:] = home_qpos(model)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.15, 0, 0.55]
    camera.distance = 2.8
    camera.azimuth = args.azimuth
    camera.elevation = args.elevation
    option = mujoco.MjvOption()
    option.geomgroup[:3] = [1, 1, int(show_spheres)]

    renderer = mujoco.Renderer(model, args.height, args.width)
    try:
        renderer.update_scene(data, camera, option)
        return renderer.render()
    finally:
        renderer.close()


def main() -> None:
    args = parse_args()
    panels = []
    for filename, label in PANELS.items():
        path = HERE / filename
        if not path.is_file():
            raise SystemExit(f"{path} not found; run compose.py first")
        panel = Image.fromarray(render(path, "spherized" in filename, args))
        ImageDraw.Draw(panel).text((12, 10), f"{filename}  ({label})", fill=(255, 255, 255))
        panels.append(np.asarray(panel))

    output = args.output.expanduser().resolve()
    Image.fromarray(np.concatenate(panels, axis=1)).save(output)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
