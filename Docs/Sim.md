# crane sim

```bash
crane sim up --headless --seed 3 && crane sim check --wait 120
astro on sim                     # then use it like the boat
crane sim down
```

`crane sim` runs a domain's T2 world (the Unity player plus its ROS side) so it is used exactly like the robot.
`astro sim up|down|status|check` calls it through the sim profile's `astro.sim`; `Tools/RoboBoat/sim_up` is its
old name and passes every argument through (bare `sim_up` means `up`). Exit codes: 0 ok, 1 the world failed or
is not ready, 2 usage or config.

## up

Start the world and return once it is ready.

```bash
crane sim up                                   # window on a terminal with a display, else headless
crane sim up --headless --seed 3 --current 0   # still water, for characterization
crane sim up --card characterized --env course:roboboat_2026 --profile interactive-high
```

1. The ROS side starts in its container (`ros.container`, today `sim_up_ros`) on the domain's dev image, host
   network and IPC, with the endpoint on port `port_base + ROS_DOMAIN_ID`. up waits up to `ros.endpoint_s`
   (60 s) for it.
2. The player starts through `heavy unity` in its own process group: in a window (`--window`), or under
   `xvfb-run` (`--headless`). up waits up to `ready_s` (300 s, `SIM_UP_READY_S`) for the domain's ready marker.

What you'll see: `crane sim: ROS side starting (domain 101, endpoint port 10101, …)`, `crane sim: player queued
on heavy unity (headless), waiting up to 300 s for the course`, then the status block (`--json`: the progress
lines on stderr, the status as JSON). A failure is one line naming the cause and the fix, for example
`crane sim: headless needs xvfb-run: install xvfb or use --window`. Run as `sim_up`, the lines say `sim_up:`.

| Option | Meaning |
|---|---|
| `--seed N` | the world seed (default 1) |
| `--env KIND:NAME`, `--course FAMILY` | the world, default `course:roboboat_2026` |
| `--card F`, `--profile-json F` | a plant profile JSON, or a name in `card.dir` |
| `--profile P` | the player's runtime profile, one of `tiers.T2.profiles` (default `interactive-low`) |
| `--current SPEED,TOWARD_DEG`, `--current 0` | the water current in m/s toward deg; 0 is still water |
| `--display window\|headless`, `--window`, `--headless` | default: a window when stdin is a terminal and DISPLAY or WAYLAND_DISPLAY is set |
| `--player PATH` | else `CRANE_PLAYER`, `Builds/CRANE-Worker`, `~/.cache/crane-phase1/player` (the first with `required_flag`) |
| `--robot R`, `--set K=V`, `--domain D`, `-- ARGS` | the robot, a pinned world parameter, the domain (default `config/sim.yaml`), extra player args |

## status

Show what runs. Run it any time; it never changes anything.

```bash
crane sim status            # --json for one JSON document
```

What you'll see:

```
sim_up status
  mode:      headless
  vessel:    sim   ROS_DOMAIN_ID 101   endpoint 127.0.0.1:10101
  player:    running, pgid 41234 (/home/me/crane_ml/Builds/CRANE-Worker/CRANE.x86_64)
  ros:       container sim_up_ros running (device.launch.py device:=sim)
  course:    ready (seed 3, truth /home/me/.cache/sim_up/course_truth.json)
  current:   0.15 m/s toward 270 deg (the scene default)
  logs:      ~/.cache/sim_up/player.log  ~/.cache/sim_up/player-stdout.log  ~/.cache/sim_up/ros.log
  next:      astro on sim
             …
```

## check

Ask whether the running world is ready to use, right after up or when something looks wrong.

```bash
crane sim check --wait 120      # poll every 2 s, up to 120 s; --json for one JSON document
crane sim check axis RESULTS    # an offline Tools/SimCheck check on recorded results
```

What you'll see: one `ok`/`FAIL` row each for the player, the course (the ready marker), the plant applied, the
ROS container, the ready nodes in `ros2 node list` and player exceptions, then `sim check: ready` (exit 0) or
`sim check: not ready, failed: …` (exit 1).

## logs

Read the last lines of the logs.

```bash
crane sim logs                 # player.log, player-stdout.log and ros.log, 40 lines each
crane sim logs ros -n 100
```

## down

Stop the world: the player's process group (TERM, then KILL after 10 s) and the domain's container, nothing
else. What you'll see: `crane sim: down` (`--json`: one JSON document).

```bash
crane sim down
```

## Where things come from

- **Domain config:** `config/domains/<domain>.yaml` `tiers.T2` names the scene, the runtime profile, the player
  flag bundle, `required_flag`, the course truth flag, the default current, the ready and plant markers, the ROS
  launch, container, image, mounts and ready nodes, and the `next:` lines. `env`, `robot` and `card` map the
  selectors to player flags. `crane/player.py` `resolve()` builds the player argv from them.
- **Host tooling** (`tiers.T2.host`, for boat astro's):
  - `ROS_DOMAIN_ID` counts only beside an env in `trust_domain_id` (boat: `ASTRO_STATE` or
    `MHSEALS_HARDWARE_CONFIG`, as astro exports it).
  - Otherwise the id comes from the vessel profile's `network.env` under `profiles` (boat: `ASTRO_PROFILES`, else
    the `ASTRO_PROFILES=` line of `~/.config/astro/profiles.env`, else `$ASTRO_DOCK/profiles`).
  - `dock` is the checkout the ROS container mounts; `heavy` the guard when crane's `bin/heavy` is missing; `up`
    the host's own command, named in fixes.
- **Environment:** `SIM_UP_VESSEL`, `SIM_UP_RUN` (the state dir, `~/.cache/sim_up`), `SIM_UP_READY_S`,
  `CRANE_PLAYER`, `HEAVY` (default crane's `bin/heavy`; `none` runs bare) and the image env named in
  `ros.image.env` (boat: `ASTRO_IMAGE`).
