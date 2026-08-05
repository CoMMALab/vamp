# Generate swept-enabled headers + all aux data for ur5/panda/fetch/baxter for the MBM eval.
# Emits {robot}_e.hh (fkcc, fkcc_pretrig, fkcc_swept, fkcc_swept_staged, bound_fk, joint_tf),
# swept_aux.hh (per-pair bs indices), staged_place.hh (per-sphere placement), and prints the
# revolute-joint index range each robot's fkcc_pretrig uses (for the leaf-trig recurrence).
import re
import cricket
from pathlib import Path
res = cricket.resources_dir()
out = Path(__file__).parent

aux = ["// per-allowed-pair bounding-sphere indices (bs1,bs2), unrolled loop order",
       "#pragma once", "#include <vector>", "#include <array>", ""]
place = ["// per-sphere placement rows for the staged FK", "#pragma once", "#include <vector>", "",
         "struct Place { unsigned slot; float lx, ly, lz, r; };", ""]

ROBOTS = [
    ("Ur5", "ur5", "ur5_spherized.urdf", "ur5.srdf", "robotiq_85_base_link", "ur5"),
    ("PandaE", "panda", "panda_spherized.urdf", "panda.srdf", "panda_grasptarget", "panda"),
    ("FetchE", "fetch", "fetch_spherized.urdf", "fetch.srdf", "gripper_link", "fetch"),
    ("BaxterE", "baxter", "baxter_spherized.urdf", "baxter.srdf", "right_gripper", "baxter"),
]
info = {}
for cls, rdir, urdf, srdf, ee, tag in ROBOTS:
    o = cricket.GenOptions(urdf=str(res / rdir / urdf), srdf=str(res / rdir / srdf), end_effector=ee,
        template_path=str(res / "templates/fk_template.hh"),
        subtemplates={"ccfk": str(res / "templates/ccfk_template.hh")},
        data={"name": cls, "resolution": 32})
    r = cricket.generate_robot_source(o)
    (out / f"{tag}_e.hh").write_text(r.source)
    d = r.data
    # revolute joint index range used by fkcc_pretrig (ps[N] references)
    idxs = sorted(set(int(m) for m in re.findall(r"ps\[(\d+)\]", r.source)))
    jlo, jhi = (idxs[0], idxs[-1] + 1) if idxs else (0, 0)
    # per-pair bounding-sphere indices, unrolled order
    pb = [(d["bounding_sphere_index"][p[0]], d["bounding_sphere_index"][p[1]]) for p in d["allowed_link_pairs"]]
    aux.append(f"static const std::vector<std::array<int,2>> {tag}_pair_bs = {{" +
               ",".join("{%d,%d}" % (a, b) for a, b in pb) + "};")
    # placement rows
    def prows(key):
        return ",".join("{%d,%.9g,%.9g,%.9g,%.9g}" % (int(x[0]), x[1], x[2], x[3], x[4]) for x in d[key])
    place.append(f"static const std::vector<Place> {tag}_leaf = {{{prows('leaf_place')}}};")
    place.append(f"static const std::vector<Place> {tag}_bsph = {{{prows('bsphere_place')}}};")
    info[tag] = dict(cls=cls, dim=r.dimension, nbs=d["n_bounding_spheres"], npairs=len(pb),
                     nsph=r.n_spheres, njtf=d["n_joint_tf"], jlo=jlo, jhi=jhi)
    print(f"{tag}: class={cls} dim={r.dimension} n_bs={d['n_bounding_spheres']} pairs={len(pb)} "
          f"n_spheres={r.n_spheres} n_joint_tf={d['n_joint_tf']} revolute=[{jlo},{jhi}) "
          f"pretrig={'fkcc_pretrig' in r.source} staged={'fkcc_swept_staged' in r.source}")

(out / "swept_aux.hh").write_text("\n".join(aux) + "\n")
(out / "staged_place.hh").write_text("\n".join(place) + "\n")
print("-> {ur5,panda,fetch,baxter}_e.hh, swept_aux.hh, staged_place.hh")
