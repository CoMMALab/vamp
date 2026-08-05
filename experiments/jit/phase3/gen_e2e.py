# Generate PandaE / FetchE headers exercising the cricket-native fkcc_pretrig
# (leaf-trig hoisted to ps/pc inputs) for the m30 end-to-end edge-validation bench.
# Writes panda_e.hh / fetch_e.hh next to this script.
#   micromamba run -n jit_patch python gen_e2e.py
import cricket
from pathlib import Path
res = cricket.resources_dir()
out = Path(__file__).parent


def gen(name, rdir, urdf, srdf, eef, fn):
    base = res / rdir
    o = cricket.GenOptions(
        urdf=str(base / urdf), srdf=str(base / srdf), end_effector=eef,
        template_path=str(res / "templates/fk_template.hh"),
        subtemplates={"ccfk": str(res / "templates/ccfk_template.hh")},
        data={"name": name, "resolution": 32})
    r = cricket.generate_robot_source(o)
    (out / fn).write_text(r.source)
    print(f"{name}: fkcc_pretrig={'fkcc_pretrig' in r.source} "
          f"sphere_fk_pretrig={'sphere_fk_pretrig' in r.source} "
          f"ns={r.n_spheres} dim={r.dimension} -> {fn}")


gen("PandaE", "panda", "panda_spherized.urdf", "panda.srdf", "panda_grasptarget", "panda_e.hh")
gen("FetchE", "fetch", "fetch_spherized.urdf", "fetch.srdf", "gripper_link", "fetch_e.hh")
