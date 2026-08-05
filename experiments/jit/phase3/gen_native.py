import cricket
from pathlib import Path
res=cricket.resources_dir()
base=res/"panda"
o=cricket.GenOptions(urdf=str(base/"panda_spherized.urdf"),srdf=str(base/"panda.srdf"),
    end_effector="panda_grasptarget",template_path=str(res/"templates/fk_template.hh"),
    subtemplates={"ccfk":str(res/"templates/ccfk_template.hh")},data={"name":"PandaNative","resolution":32})
r=cricket.generate_robot_source(o)
src=r.source
out=Path("/tmp/claude-1000/-home-zak-src-vamp-jit/3dcbbe7c-40b1-4721-b12b-3292533b6ada/scratchpad/panda_native.hh")
out.write_text(src)
print("sphere_fk_pretrig emitted:", "sphere_fk_pretrig" in src)
print("n_spheres", r.n_spheres, "dim", r.dimension)
