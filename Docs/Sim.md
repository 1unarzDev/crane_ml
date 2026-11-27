# crane sim

```bash
crane sim list                          # the sims you can start by name (* is this machine's default)
crane sim up roboboat-high              # the RoboBoat course in the Unity world, full quality
crane sim use roboboat-high --display window    # once per machine: a bare `crane sim up` starts it
crane sim up && crane sim check --wait 120
astro on sim                            # with astro: use it like the boat; without: eval "$(crane sim env)"
crane sim down
```

`crane sim` runs a domain's world (the Unity player, or the T1 twin) plus the robot software beside it, so it is used
exactly like the robot. `astro sim up|down|status|check` calls it through the sim profile's `astro.sim`;
`Tools/RoboBoat/sim_up` is its old name and passes every argument through (bare `sim_up` means `up`). Exit codes: 0
ok, 1 the world failed or is not ready, 2 usage or config.

## Choosing a sim

Four words choose everything:

| Word | What it is | Where it is defined |
|---|---|---|
| **sim** | a named recipe: domain, course, fidelity, and any other up setting | `config/sims/NAME.yaml`; your own in `~/.config/crane/sims/` (yours win by name) |
| **fidelity** | `twin` (T1, no Unity), `low`, `high`, `eval` (T2 with the player's interactive-low, interactive-high, evaluation-high) | the domain's `fidelity:` ladder |
| **stack** | the software beside the world: `astro` (the robot team's container), `bridge` (crane's ROS-TCP endpoint, bring your own nodes), `none` (the player alone) | the domain's `stacks:`, `config/stacks/bridge.yaml` |
| **instance** | which parallel world: 0, 1 and up, or `auto` | `--instance`, `CRANE_SIM_INSTANCE` |

- **Which sim.** The SIM word, else `CRANE_SIM`, else this machine's default, else the domain's defaults (today's
  `roboboat`). `--domain` without a SIM word skips the machine's default.
- **The layers**, lowest first: the domain's defaults, the sim file, this machine's `~/.config/crane/crane.yaml`
  `sim:` block (`default`, `display`, `player`, `stack`; written by `crane sim use` and by `crane setup`), then
  the flags. `crane sim show [SIM] [flags]` prints each value with where it came from.
- **Expert overrides.** `--tier` and `--profile` beat the fidelity's. `--course NAME` (or `--env course:NAME`) picks
  the course version: `crane sim up --list env` lists them.
- **Stack auto.** astro when its checkout (`ASTRO_DOCK`, else `~/astro_dock`) is here, else bridge. The bridge
  image is `docker build -t crane-bridge:jazzy docker/bridge`, with the team's ROS-TCP endpoint fork at astro's
  commit.

A sim file:

```yaml
about: RoboBoat 2026 course in the Unity world, full quality (needs a GPU)
domain: boat
course: roboboat_2026
fidelity: high
```

### ROS domains

You never set one. The sim's ROS_DOMAIN_ID is astro's sim profile's (101), or `config/sim.yaml` `ros_domain_base`
(101) without astro, plus the instance number. A domain that a robot vessel profile names (blastoise's 42) is always
refused, so a shell on the boat's vessel (`astro crane …` on the operator laptop) cannot put the sim on the boat's
network. To connect, run `astro on sim` (the astro stack) or `eval "$(crane sim env)"` (any tool).

### Running twice, people and scripts

`up` writes `recipe.json` (the recipe, its identity and its owner) into the instance's state dir.

- **The same sim already up, same owner:** it is reused at once (`--fresh` restarts it).
- **A different sim up:** refused, naming `--fresh`, `--instance auto` and `down`.
- **A person's sim and a script's up:** a terminal is a person; an agent, test or pipeline is a script
  (`CRANE_SIM_OWNER` overrides). A script's T1 world goes to the next free instance. A T2 world is refused, because
  the one Unity player runs one T2 world at a time.
- **A script's down:** it never stops a person's sim without `--force`.

### Repeated trials

`crane sim reset` puts the boat back at the course or pool start without restarting anything. It calls the world's
`/crane/reset_to_start` (the T2 player's or the T1 node's), which also re-seats the EKFs. Stop any goal first: the
navigation keeps running. On the bridge stack, the T1 twin publishes truth odometry (there is no EKF there).

### Watching

`crane sim watch` brings a headless T2 player back in a window. Only the player restarts, so the ROS side and the
course stay up and the boat goes back to the start. `crane sim restart player --display headless|window` does the
same either way. A T1 world has no 3-D view: follow `/crane/odom` after `eval "$(crane sim env)"`.

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
| `--dry-run` | print the resolved player argv and the ROS side's `docker run`; nothing starts |

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

## restart

Restart one part of the running world and keep the rest.

```bash
crane sim restart nav        # new nav2_limits in the sim state take effect; nothing else stops
crane sim restart ros        # the ROS container again with the same docker run; the player stays and reconnects
crane sim restart player     # the player again with the same argv; ROS stays
```

- **nav** runs the domain's `tiers.T2.ros.restart.nav` recipe inside the ROS container: TERM the Nav2 nodes
  (matched by exact `__node:=NAME`), wait up to `stop_s` (20 s), KILL if needed, start
  `ros2 launch mhseals_nav device.launch.py device:=sim only:=navigation` (the same device profile and overrides
  as `up`, so the same navigation args), and wait up to `ready_s` (90 s) for `bt_navigator` to be `active`. It never
  starts a second set while the first is alive. Goals in flight are lost.
- **ros** waits up to 60 s for the endpoint and 120 s for the ready nodes. **player** waits up to `ready_s` of the
  course (300 s).
- The state dir keeps the argv of `up` (`player.argv`, `ros.docker`); status shows `restarted:`. Exit 0 ok, 1
  failed, 2 usage or nothing running, 75 timed out waiting. It ends with `next: crane sim check`.

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
- **Stacks:** `stacks.NAME.T2` and `stacks.NAME.T1` lay over `tiers.T2` and `tiers.T1.sim` (`crane/sim/stacks.py`).
  boat's `stacks.astro` holds the ROS side below (container `sim_up_ros`, the mhseals launch, mounts, restart
  recipe) and the host tooling; `config/stacks/bridge.yaml` holds crane's (container `crane_sim_ros`, one command:
  the endpoint, or `wait` beside the T1 node). The running world's stack is in the state dir's `stack`.
- **Host tooling** (`stacks.astro.T2.host`, for boat astro's):
  - An exported `ROS_DOMAIN_ID` counts only when `vessel_env` names the sim vessel (boat: `ASTRO_VESSEL=sim`, as
    `astro sim` exports it). A shell on a robot's vessel exports the robot's domain, so it is ignored.
  - Otherwise the id comes from the vessel profile's `network.env` under `profiles` (boat: `ASTRO_PROFILES`, else
    the `ASTRO_PROFILES=` line of `~/.config/astro/profiles.env`, else `$ASTRO_DOCK/profiles`). With no profile set
    on the machine it is `config/sim.yaml` `ros_domain_base` (101).
  - A domain that any other vessel profile names (blastoise: 42) is refused, instance offsets included.
  - The envs in `ros.state_env` (boat: `ASTRO_STATE`, `MHSEALS_HARDWARE_CONFIG`) go into the ROS container
    with a leading `$HOME` mapped to `ros.home`, so its Nav2 reads the sim state's `nav2_limits`.
  - `dock` is the checkout the ROS container mounts; `heavy` the guard when crane's `bin/heavy` is missing; `up`
    the host's own command, named in fixes.
- **Environment:** `SIM_UP_VESSEL`, `SIM_UP_RUN` (the state dir, `~/.cache/sim_up`), `SIM_UP_READY_S`,
  `CRANE_PLAYER`, `HEAVY` (default crane's `bin/heavy`; `none` runs bare) and the image env named in
  `ros.image.env` (boat: `ASTRO_IMAGE`).
