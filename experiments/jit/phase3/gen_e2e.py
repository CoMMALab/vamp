# Generate unrolled swept-enabled headers (fkcc + fkcc_swept + bound_fk) for panda/fetch/baxter,
# plus swept_aux.hh (per-allowed-pair bounding-sphere indices) and link_groups.hh (leaf groups).
# Non-compact -> unrolled kernels. Writes next to this script.
#   micromamba run -n jit_patch python gen_e2e.py
import cricket
from pathlib import Path
res = cricket.resources_dir()
out = Path(__file__).parent
aux = ["// per-allowed-pair bounding-sphere indices (bs1,bs2), unrolled loop order",
       "#pragma once", "#include <vector>", "#include <array>", ""]
groups = ["// leaf-sphere indices per collision link", "#pragma once", "#include <vector>", ""]

ROBOTS = [
    ("PandaE", "panda", "panda_spherized.urdf", "panda.srdf", "panda_grasptarget", "panda"),
    ("FetchE", "fetch", "fetch_spherized.urdf", "fetch.srdf", "gripper_link", "fetch"),
    ("BaxterE", "baxter", "baxter_spherized.urdf", "baxter.srdf", "right_gripper", "baxter"),
]
for name, rdir, urdf, srdf, ee, tag in ROBOTS:
    o = cricket.GenOptions(
        urdf=str(res / rdir / urdf), srdf=str(res / rdir / srdf), end_effector=ee,
        template_path=str(res / "templates/fk_template.hh"),
        subtemplates={"ccfk": str(res / "templates/ccfk_template.hh")},
        data={"name": name, "resolution": 32})
    r = cricket.generate_robot_source(o)
    (out / f"{tag}_e.hh").write_text(r.source)
    d = r.data
    pb = [(d["bounding_sphere_index"][p[0]], d["bounding_sphere_index"][p[1]]) for p in d["allowed_link_pairs"]]
    aux.append(f"static const std::vector<std::array<int,2>> {tag}_pair_bs = {{" +
               ",".join("{%d,%d}" % (a, b) for a, b in pb) + "};")
    lg = [g for g in d["per_link_spheres"] if g]
    groups.append(f"static const std::vector<std::vector<int>> {tag}_links = {{" +
                  ",".join("{" + ",".join(map(str, g)) + "}" for g in lg) + "};")
    print(f"{name}: fkcc_swept={'fkcc_swept' in r.source} bound_fk={'bound_fk' in r.source} "
          f"n_bs={d['n_bounding_spheres']} n_pairs={len(pb)} n_spheres={r.n_spheres} dim={r.dimension}")

(out / "swept_aux.hh").write_text("\n".join(aux) + "\n")
(out / "link_groups.hh").write_text("\n".join(groups) + "\n")
print("-> swept_aux.hh, link_groups.hh")
