# Gym-style env (`crane.env`)

```python
import numpy as np, crane.env
env = crane.env.make('boat', tier='T1', n=64, backend='numpy', seed=0)    # 64 envs, scenario seeds 0..63
gx = env.observation_space.names.index('goal.dx')
obs = env.reset(seed=0)                                                   # [64, obs_dim]
for step in range(3000):
    dx, dy = obs[:, gx], obs[:, gx + 1]                                   # the next goal, body frame
    action = np.stack([0.8 * dx, 0.5 * dy, np.arctan2(dy, dx)], axis=-1)  # surge, sway, yaw setpoints
    obs, reward, terminated, truncated, info = env.step(action)           # each [64]
    if (terminated | truncated).all():
        obs = env.reset()                                                 # the next 64 seeds
```

`crane.env.make(domain, tier='T1', n=E, backend='numpy'|'numba'|'jax', card=None, seed=0, **options)` returns an
`Env` of E envs over the batched engine ([Engine.md](Engine.md)): the domain tier's plant card, limits, perception
and scenarios, the ones `crane run` and `crane study` use. There is no CLI noun for it: `crane env` is the
course/world generator, and stepping a policy is Python work, so the env is the Python API only.

## What the domain declares

`config/domains/<D>.yaml` `gym:` holds every specific (the boat's is below); the engine's controller module for the
domain gives each env's goals (the boat: the course route, then the approach and the berth).

```yaml
gym:
  action: velocity                  # the default action space
  actions:
    velocity: {channels: [surge, sway, yaw], unit: [m/s, m/s, rad/s]}   # bounds: the tier's limits.vmin/vmax
    pwm: {channels: [pwm0, pwm1, pwm2, pwm3], low: 1100, high: 1900, unit: us}   # the T1 plant's thrusters
  obs: [pose, vel, goal]            # channel groups; perception adds the camera's targets
  reward: {progress: 1.0, goal: 1.0, contact: -5.0, step: 0.0, success: 10.0}
```

| obs group | channels |
|---|---|
| `pose` | `x y yaw`: the map-frame pose (m, rad) |
| `vel` | `u v r`: the body-frame velocity |
| `goal` | `goal.dx goal.dy` (body frame), `goal.dist`, `goal.done` (goals reached / goals) |
| `perception` | `seen.n` (targets the camera sees), `seen.dx seen.dy` (the nearest one, body frame; 0 with none) |

- **Actions** are clipped to `env.action_space` (`names`, `low`, `high`, `units`) and held for one decision period,
  1/`hz` s (default the tier's `tick_hz`). `velocity` goes through the plant's velocity loop (as the tracker's
  target does); `pwm` drives a plant that takes raw inputs (the T1 boat), else `make` says which actions it takes.
- **Reward** per step: `progress` x the metres gained toward the current goal, plus `goal` per goal reached,
  `contact` per new contact, `step`, and `success` when the last goal is reached. `make(..., reward={...})` overrides
  weights.
- **Ends:** `terminated` when the last goal is reached; `truncated` at the scenario's time limit. An ended env holds
  still and returns reward 0 until `reset`.

## Determinism and batches

Env i runs scenario seed `seed + i`, reads only its own rows and draws no randomness: a reset with the same seed
repeats the episode bit for bit, and env i is the same at n=1 and in a batch. `reset()` without a seed takes the
next n seeds. numpy is the float64 reference; numba steps the plant through the vehicle's numba twin in parallel
(the same bits); jax jits the plant (float32 by default, `dtype='float64'` within 1e-9 of numpy). Obs and reward
are computed on the host.

## Recording

`make(..., record=DIR)` keeps every episode and `env.close()` writes `DIR/obs` (crane.data.rows, npz): one row per
(env, episode) with `obs` T x C, `action` T x A, `reward` T, plus `seed`, `env`, `episode`, `steps`, `return`,
`success`; its `manifest.json` names the obs channels, the action channels and kind, `hz` and the reward weights.
`crane data` reads it like any dataset.

## Options

| option | what |
|---|---|
| `action` | a `gym.actions` name |
| `obs` | channel groups, e.g. `['pose', 'vel', 'goal', 'perception']` |
| `reward` | weights over the domain's |
| `card` | a plant card ref or path (bound to the domain's card keys) |
| `set` | `{dotted key: value}` tier config overrides, e.g. `{'time_limit_s': 60}` |
| `dtype` | `float64` (numpy, numba) or `float32` (jax's default) |
| `hz` | decisions per second |
| `record` | a dir for the obs/action dataset |
