import pickle
from pathlib import Path
import numpy as np
from fire import Fire
import vamp
from vamp import pybullet_interface as vpb
from vamp import pointcloud as vpc
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import time


def topple(
    robot: str = "pandatopp",                  # Robot to plan for
    planner: str = "aorrtctopp",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: str = "box",                     # Problem name
    index: int = 53,                        # Problem index
    sampler_name: str = "xorshift",          # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    display_object_names: bool = False,    # Display object names over geometry
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 20000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = False,              # Cull pointcloud around robot by maximum distance
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / "panda"
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )
    # plan_settings.max_iterations = 10000000
    # plan_settings.max_samples = 1000000
    # plan_settings.range = 12
    # plan_settings.radius = 16
    # plan_settings.min_radius = 8
    # plan_settings.dynamic_domain = False
    # simp_settings.bez = True

    plan_settings.max_iterations = 10000000
    plan_settings.max_samples = 1000000
    plan_settings.rrtc.max_iterations = 10000000
    plan_settings.rrtc.max_samples = 1000000
    plan_settings.rrtc.range = 12
    plan_settings.simplify.bez = True
    plan_settings.rrtc.radius = 16
    plan_settings.rrtc.min_radius = 8
    plan_settings.rrtc.dynamic_domain = False
    plan_settings.use_phs = True
    plan_settings.optimize = True
    plan_settings.simplify_intermediate = True
    simp_settings.bez = True

    if not problem:
        problem = list(data['problems'].keys())[0]

    if problem not in data['problems']:
        raise RuntimeError(
            f"""No problem with name {problem}!
Existing problems: {list(data['problems'].keys())}"""
            )

    problems = data['problems'][problem]
    try:
        problem_data = next(problem for problem in problems if problem['index'] == index)
    except StopIteration:
        raise RuntimeError(f"No problem in {problem} with index {index}!")

    if pointcloud:
        r_min, r_max = vamp_module.min_max_radii()
        (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
            "panda",
            r_min,
            r_max,
            problem_data,
            samples_per_object,
            filter_radius,
            filter_cull,
            )

        print(
            f"""
Original Pointcloud size: {len(original_pc)}
Filtered Pointcloud size: {len(filtered_pc)}

        Filtering Time: {filter_time * 1e-6:5.3f}ms
CAPT Construction Time: {build_time * 1e-6:5.3f}ms
            """
            )

    else:
        env = vamp.problem_dict_to_vamp(problem_data)

    start = problem_data['start']
    goals = problem_data['goals']
    valid = problem_data['valid']

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    for i in range(14):
        start.append(0)
    for goal in goals:
        for j in range(14):
            goal.append(0)

    if valid:
        result = planner_func(start, goals, env, plan_settings, sampler)
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
        simplify = vamp_module.simplify(result.path, env, simp_settings, sampler)

        stats = vamp.results_to_dict(result, simplify)
        print(
            f"""
Planning Time: {stats['planning_time'].microseconds:8d}μs
Simplify Time: {stats['simplification_time'].microseconds:8d}μs
   Total Time: {stats['total_time'].microseconds:8d}μs

Planning Iters: {stats['planning_iterations']}
n Graph States: {stats['planning_graph_size']}

Path Length:
   Initial: {stats['initial_path_cost']:5.3f}
Simplified: {stats['simplified_path_cost']:5.3f}"""
            )

        plan = simplify.path
        plan = vamp_module.compute_traj(plan, env, simp_settings, sampler).path.numpy()
        

    if valid and not solved:
        print("Failed to solve problem! Displaying start and goals.")
        print(start)
        for goal in goals:
            print(goal)

        if problem_data['valid']:
            print(
                f"""
Planning Time: {int(result.nanoseconds / 1000):8d}μs
Planning Iters: {result.iterations}
n Graph States: {result.size}
"""
                )

    if not solved:
        plan = vamp_module.Path()
        plan.append(start)
        for goal in goals:
            plan.append(goal)

    sim = vpb.PyBulletSimulator(str(robot_dir / f"panda_spherized.urdf"), vamp_module.joint_names(), True)
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    if pointcloud:
        sim.draw_pointcloud(filtered_pc)

    # if not valid:
    #     for state in [start, *goals]:
    # for state in plan:
    #     # if not vamp_module.validate(state, env):
    #     # state = state[0:7]
    #     # print(f"Displaying colliding spheres for first invalid state: {state}")
    #     debug = vamp_module.debug(state, env)
    #     # invalid = set([x[0] for x in filter(lambda x: x[1], enumerate(debug[0]))])

    #     # for (a, b) in debug[1]:
    #     #     invalid.add(a)
    #     #     invalid.add(b)

    #     # spheres = vamp_module.fk(state)
    #     # for i in range(len(spheres)):
    #     #     sphere = spheres[i]
    #     #     if i in invalid:
    #     #         sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 0., 0., 1.])
    #     #     else:
    #     #         sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 1., 1., 1.])
    #     for i, colliding in enumerate(debug[0]):  
    #         if colliding:  
    #             spheres = vamp_module.fk(state)  
    #             s = spheres[i]  
    #             for i in range(len(spheres)):
    #                 sphere = spheres[i]
    #                 sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 0., 0., 1.])


    # sim.animate(simplify.path)
    print(plan.shape)
    sim.animate(plan[np.arange(0, len(plan), 15)])


def toppra(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: str = "table_under_pick",                     # Problem name
    index: int = 45,                        # Problem index
    sampler_name: str = "xorshift",          # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    display_object_names: bool = False,    # Display object names over geometry
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 20000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = False,              # Cull pointcloud around robot by maximum distance
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / "panda"
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )
    plan_settings.max_iterations = 10000000
    plan_settings.max_samples = 1000000
    plan_settings.range = 2
    simp_settings.bez = False
    plan_settings.radius = 4
    plan_settings.min_radius = 0.5
    plan_settings.dynamic_domain = True
    plan_settings.alpha = 0.00001

    max_vel = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
    max_acc = np.ones(7) * 10

    pc_vel = constraint.JointVelocityConstraint(max_vel)
    pc_acc = constraint.JointAccelerationConstraint(max_acc)


    if not problem:
        problem = list(data['problems'].keys())[0]

    if problem not in data['problems']:
        raise RuntimeError(
            f"""No problem with name {problem}!
Existing problems: {list(data['problems'].keys())}"""
            )

    problems = data['problems'][problem]
    try:
        problem_data = next(problem for problem in problems if problem['index'] == index)
    except StopIteration:
        raise RuntimeError(f"No problem in {problem} with index {index}!")

    if pointcloud:
        r_min, r_max = vamp_module.min_max_radii()
        (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
            "panda",
            r_min,
            r_max,
            problem_data,
            samples_per_object,
            filter_radius,
            filter_cull,
            )

        print(
            f"""
Original Pointcloud size: {len(original_pc)}
Filtered Pointcloud size: {len(filtered_pc)}

        Filtering Time: {filter_time * 1e-6:5.3f}ms
CAPT Construction Time: {build_time * 1e-6:5.3f}ms
            """
            )

    else:
        env = vamp.problem_dict_to_vamp(problem_data)

    start = problem_data['start']
    goals = problem_data['goals']
    valid = problem_data['valid']

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    if valid:
        result = planner_func(start, goals, env, plan_settings, sampler)
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
        simplify = vamp_module.simplify(result.path, env, simp_settings, sampler)

        stats = vamp.results_to_dict(result, simplify)
        print(
            f"""
Planning Time: {stats['planning_time'].microseconds:8d}μs
Simplify Time: {stats['simplification_time'].microseconds:8d}μs
   Total Time: {stats['total_time'].microseconds:8d}μs

Planning Iters: {stats['planning_iterations']}
n Graph States: {stats['planning_graph_size']}

Path Length:
   Initial: {stats['initial_path_cost']:5.3f}
Simplified: {stats['simplified_path_cost']:5.3f}"""
            )

        plan = simplify.path
        path = ta.SplineInterpolator(np.linspace(0, 1, len(plan.numpy())), plan.numpy())
        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        traj = instance.compute_trajectory()
        plan = []
        for t in range(0, int(traj.duration * 1000)):
            plan.append(traj(t / 1000))
        plan = np.array(plan)

    if valid and not solved:
        print("Failed to solve problem! Displaying start and goals.")
        print(start)
        for goal in goals:
            print(goal)

        if problem_data['valid']:
            print(
                f"""
Planning Time: {int(result.nanoseconds / 1000):8d}μs
Planning Iters: {result.iterations}
n Graph States: {result.size}
"""
                )

    if not solved:
        plan = vamp_module.Path()
        plan.append(start)
        for goal in goals:
            plan.append(goal)

    sim = vpb.PyBulletSimulator(str(robot_dir / f"panda_spherized.urdf"), vamp_module.joint_names(), True)
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    if pointcloud:
        sim.draw_pointcloud(filtered_pc)


    # sim.animate(simplify.path)
    print(plan.shape)
    sim.animate(plan[np.arange(0, len(plan), 15)])

if __name__ == "__main__":
    Fire(topple)
    # Fire(toppra)
