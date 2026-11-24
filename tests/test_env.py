"""crane.env: the Gym-style batched env over the batched engine: determinism, batch invariance, the declared
spaces and reward, the obs/action recorder, and the fast backends against the numpy reference."""

import importlib.util
import json

import numpy as np
import pytest

import crane.env as ce
from crane.data.rows import read_rows


def policy(env, obs):
    """A numpy policy: head for the goal (obs goal.dx, goal.dy) at a speed the distance sets."""
    i = env.observation_space.names.index('goal.dx')
    dx, dy = obs[:, i], obs[:, i + 1]
    return np.stack([0.8 * dx, 0.5 * dy, np.arctan2(dy, dx)], axis=-1)


def rollout(env, seed, steps):
    obs = env.reset(seed=seed)
    trace = [obs]
    for _ in range(steps):
        obs, reward, term, trunc, info = env.step(policy(env, obs))
        trace += [obs, reward[:, None], term[:, None], trunc[:, None]]
    return np.concatenate(trace, axis=1), info


def test_reset_and_step_repeat_and_an_env_is_the_same_alone_and_in_a_batch(tmp_path):
    env = ce.make('boat', tier='T1', n=3, seed=20, obs=['pose', 'vel', 'goal', 'perception'], record=tmp_path / 'rec')
    assert env.observation_space.names[:3] == ['x', 'y', 'yaw'] and env.observation_space.shape == (13,)
    a, info = rollout(env, 20, 120)
    assert env.action_space.names == ['surge', 'sway', 'yaw'] and env.action_space.units[0] == 'm/s'
    b, _ = rollout(env, 20, 120)
    assert np.array_equal(a, b)                                       # a reset with the same seed repeats
    assert info['goal_index'].min() >= 2 and (info['seed'] == [20, 21, 22]).all()
    for i in range(3):                                                # env i at n=3 is env seed+i alone
        one, _ = rollout(ce.make('boat', n=1, obs=['pose', 'vel', 'goal', 'perception']), 20 + i, 120)
        assert np.array_equal(one[0], a[i])
    manifest = env.close()
    rows = sorted(read_rows(tmp_path / 'rec' / 'obs'), key=lambda r: (r['episode'], r['env']))
    assert manifest['channels'] == env.observation_space.names and manifest['actions'] == ['surge', 'sway', 'yaw']
    assert len(rows) == 6 and rows[0]['obs'].shape == (120, 13) and rows[0]['action'].shape == (120, 3)
    assert rows[0]['return'] == pytest.approx(float(rows[0]['reward'].sum()), rel=1e-5)
    assert rows[0]['return'] == pytest.approx(rows[3]['return'], rel=1e-6)    # episode 0 and 1, same seed


def test_the_declared_spaces_reward_and_ends(tmp_path):
    env = ce.make('boat', n=2, seed=3, action='pwm', set={'time_limit_s': 2.0}, reward={'step': -0.1})
    obs = env.reset()
    assert env.action_space.names == ['pwm0', 'pwm1', 'pwm2', 'pwm3'] and env.action_space.high[0] == 1900
    total, ends = np.zeros(2), []
    for _ in range(25):
        obs, reward, term, trunc, info = env.step(np.full((2, 4), 1500.0))   # neutral: the boat stays put
        total += reward
        ends.append(trunc.copy())
    assert np.allclose(obs[:, :2], 0.0) and np.array_equal(ends[19], [True, True]) and not ends[18].any()
    assert np.allclose(total, -0.1 * 20)                               # after its end an env gives 0
    with pytest.raises(ce.EnvError) as caught:
        ce.make('boat', tier='T0', n=1, action='pwm').reset()
    assert 'fix: make(..., action=' in str(caught.value) and '\n' not in str(caught.value)
    with pytest.raises(ce.EnvError, match='no gym: section'):
        ce.make('toy')
    cfg = json.loads(json.dumps(env.weights))
    assert cfg['contact'] < 0 and cfg['step'] == -0.1


@pytest.mark.parametrize('backend', ['numba', 'jax'])
def test_the_fast_backends_step_as_the_numpy_reference(backend):
    if importlib.util.find_spec(backend) is None:
        pytest.skip(f"{backend} is not installed; pip install 'crane[fast]' (numba) or jax")
    ref, _ = rollout(ce.make('boat', n=2, seed=5), 5, 40)
    try:
        got, _ = rollout(ce.make('boat', n=2, seed=5, backend=backend, dtype='float64'), 5, 40)
    except Exception as error:  # noqa: BLE001  a GPU another process has filled is the host's state, not a result
        if 'RESOURCE_EXHAUSTED' in str(error) or 'OUT_OF_MEMORY' in str(error):
            pytest.skip(f'{backend}: the GPU is out of memory ({type(error).__name__}); fix: JAX_PLATFORMS=cpu')
        raise
    assert np.abs(got - ref).max() <= (0.0 if backend == 'numba' else 1e-9)
