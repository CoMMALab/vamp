import xmltodict
from pathlib import Path
import json
from typing import Any

def _urdf_clean_filename(filename: str) -> str:
    if filename.startswith('package://'):
        filename = filename[len('package://'):]

    return filename

def load_urdf(urdf_path: Path):
    with open(urdf_path, 'r') as f:
        xml = xmltodict.parse(f.read())
        xml['robot']['@path'] = urdf_path
        return xml

def save_urdf(urdf: dict[str, Any], filename: Path):
    with open(filename, 'w') as f:
        f.write(xmltodict.unparse(urdf, pretty = True))


def load_spheres(path: Path):
    with open(path, 'r') as f:
        return json.load(f)

def set_urdf_spheres(urdf, spheres):
    total_spheres = 0
    for link in urdf['robot']['link']:
        name = link['@name']
        if 'collision' not in link:
            continue

        collisions = link['collision']
        print(collisions)

        if not isinstance(collisions, list):
            collisions = [collisions]

        spherizations = []
        for i, collision in enumerate(collisions):

            geometry = collision['geometry']

            if 'box' in geometry or 'cylinder' in geometry or 'sphere' in geometry:
                key = f"{name}::primitive{i}"
                if key in spheres:
                    spherizations.append(spheres[key])

            elif 'mesh' in geometry:
                mesh = geometry['mesh']
                filename = _urdf_clean_filename(mesh['@filename'])
                key = f"{name}::{filename}"
                print(key)

                if name in spheres:
                    spherizations.append(spheres[name])

        collision = []
        for spherization in spherizations:
            print(spherization)
            for sphere_center, sphere_radius in zip(spherization["centers"], spherization["radii"]):
                total_spheres += 1
                collision.append(
                    {
                        'geometry': {
                            'sphere': {
                                '@radius': sphere_radius
                                }
                            },
                        'origin': {
                            '@xyz': ' '.join(map(str, sphere_center)), '@rpy': '0 0 0'
                            }
                        }
                    )

        link['collision'] = collision
    print(f"spheres: {total_spheres}")


if __name__ == "__main__":
    urdf = load_urdf('dualpanda_exported_dup.urdf')
    spheres = load_spheres('spheres.json')
    set_urdf_spheres(urdf, spheres)
    save_urdf(urdf, 'dualpanda_exported_spherized.urdf')
