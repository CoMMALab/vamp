"""Tests for the batched validation bindings.

Run with `pytest tests/test_batch_validation.py` or `python tests/test_batch_validation.py`.
"""

import numpy as np
import vamp

PANDA_START = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
PANDA_GOAL = np.array([2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785], dtype=np.float32)
PANDA_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float32)
PANDA_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)


def sphere_cage():
    env = vamp.Environment()
    for center in [[0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25], [-0.55, 0, 0.25],
                   [0.35, -0.35, 0.8], [0, 0.55, 0.8], [-0.35, 0.35, 0.8]]:
        env.add_sphere(vamp.Sphere(center, 0.15))
    return env


def random_motions(dimension, n=2000, seed=0):
    rng = np.random.default_rng(seed)
    a = rng.uniform(-np.pi, np.pi, (n, dimension)).astype(np.float32)
    b = (a + rng.normal(0, 0.3, (n, dimension))).astype(np.float32)
    return a, b


def test_validate_motion_batch_matches_scalar():
    env = sphere_cage()
    for name in vamp.robots:
        robot = getattr(vamp, name)
        a, b = random_motions(robot.dimension())

        for check_bounds in (True, False):
            batch = robot.validate_motion_batch(a, b, env, check_bounds=check_bounds)
            scalar = np.array([robot.validate_motion(x, y, env, check_bounds) for x, y in zip(a, b)])
            assert (batch == scalar).all(), f"{name}, check_bounds={check_bounds}"

        # Defaults agree with validate_motion's defaults.
        batch = robot.validate_motion_batch(a, b, env)
        scalar = np.array([robot.validate_motion(x, y, env) for x, y in zip(a, b)])
        assert (batch == scalar).all(), f"{name}, default arguments"


def test_validate_motion_batch_strided_input():
    env = sphere_cage()
    robot = vamp.panda
    a, b = random_motions(robot.dimension())
    wide = np.zeros((len(a), 2 * robot.dimension()), dtype=np.float32)
    wide[:, ::2] = a
    assert (robot.validate_motion_batch(wide[:, ::2], b, env) == robot.validate_motion_batch(a, b, env)).all()


def test_validate_motion_batch_mismatched_rows():
    a, b = random_motions(vamp.panda.dimension(), n=10)
    try:
        vamp.panda.validate_motion_batch(a, b[:5], sphere_cage())
    except RuntimeError:
        return
    raise AssertionError("expected RuntimeError for mismatched row counts")


def test_rrtc_and_aorrtc_batch_solve():
    env = sphere_cage()
    starts = np.repeat(PANDA_START[None], 8, axis=0)
    goals = np.repeat(PANDA_GOAL[None], 8, axis=0)

    results = vamp.panda.rrtc_batch(starts, goals, env, vamp.RRTCSettings())
    assert len(results) == 8 and all(r.solved for r in results)

    settings = vamp.AORRTCSettings()
    settings.max_iterations = 2000
    results = vamp.panda.aorrtc_batch(starts[:2], goals[:2], env, settings)
    assert len(results) == 2 and all(r.solved for r in results)


def valid_panda_layers(env, batch, layers, dreams, seed=0):
    rng = np.random.default_rng(seed)
    points = []
    while len(points) < batch * layers * dreams:
        q = rng.uniform(PANDA_LOW, PANDA_HIGH).astype(np.float32)
        if vamp.panda.validate(q, env):
            points.append(q)
    return np.array(points, dtype=np.float32).reshape(batch, layers, dreams, 7)


def connectivity_arrays(batch, layers, dreams, goals):
    return (np.zeros((batch, 1, dreams), dtype=bool),
            np.zeros((batch, layers - 1, dreams, dreams), dtype=bool),
            np.zeros((batch, dreams, goals), dtype=bool))


def test_incremental_validation_matches_full_recompute():
    env = sphere_cage()
    settings = vamp.RRTCSettings()
    B, L, D = 2, 3, 6
    layers = valid_panda_layers(env, B, L, D)
    goals = PANDA_GOAL[None].copy()

    full = connectivity_arrays(B, L, D, 1)
    vamp.panda.incremental_batch_validate_rrtc(PANDA_START, layers, goals, *full, env, settings, 0, D)

    incremental = connectivity_arrays(B, L, D, 1)
    vamp.panda.incremental_batch_validate_rrtc(PANDA_START, layers, goals, *incremental, env, settings, 0, 3)
    vamp.panda.incremental_batch_validate_rrtc(PANDA_START, layers, goals, *incremental, env, settings, 3, D)

    for f, i in zip(full, incremental):
        assert (f == i).all()


def test_incremental_validation_rejects_bad_shapes():
    env = sphere_cage()
    settings = vamp.RRTCSettings()
    B, L, D = 1, 2, 4
    layers = valid_panda_layers(env, B, L, D)
    goals = PANDA_GOAL[None].copy()

    start_v, layer_v, goal_v = connectivity_arrays(B, L, D, 1)
    bad_calls = [
        (start_v, layer_v[:, :, :2], goal_v, 0, D),  # layer_validate too small
        (start_v, layer_v, goal_v, 0, D + 1),  # end_point_range past num_dreams
        (start_v, layer_v, goal_v, 3, 2),  # start > end
    ]
    for sv, lv, gv, lo, hi in bad_calls:
        try:
            vamp.panda.incremental_batch_validate_rrtc(PANDA_START, layers, goals, sv, lv, gv, env, settings, lo, hi)
        except RuntimeError:
            continue
        raise AssertionError(f"expected RuntimeError for range ({lo}, {hi}) / shapes {lv.shape}")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"{len(tests)} tests passed")
