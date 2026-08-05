import cricket, json
from pathlib import Path
SC = "/tmp/claude-1000/-home-zak-src-vamp-jit/3dcbbe7c-40b1-4721-b12b-3292533b6ada/scratchpad"
res = cricket.resources_dir()
recipe = json.loads((res/"fetch.json").read_text())
urdf = str(res/recipe["urdf"]); srdf = str(res/recipe["srdf"])
tmpl = str(res/"templates/fk_template.hh"); sub = {"ccfk": str(res/"templates/ccfk_template.hh")}
arm = ["shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"]
def gen(name, data, out):
    data = {"resolution": recipe.get("resolution",32), **data}
    o = cricket.GenOptions(urdf=urdf, srdf=srdf, end_effector=recipe["end_effector"],
                           template_path=tmpl, subtemplates=sub, data=data)
    r = cricket.generate_robot_source(o)
    Path(out).write_text(r.source)
    print(f"{name}: n_spheres={r.n_spheres} dim={r.dimension} -> {out}")
gen("base", {"name":"FetchBase"}, f"{SC}/fetch_base.hh")
gen("locked", {"name":"FetchLocked","active_joints":arm}, f"{SC}/fetch_locked.hh")
