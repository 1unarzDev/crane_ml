# Tools/MissionSim: fast mission-level simulator for two-drone find, verify and report

Evidence label: **IMPLEMENTED** (synthetic, mission level, placeholder observation model). Nothing here is
LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED. Pure Python + numpy, seeded, 1 s ticks, no Unity.

| Module | Content |
|---|---|
| `missionsim/world.py` | Scenario family (`Scenario`), seeded 10x10 grid of 20 m cells: canopy density field, thickets/blockages, clearings (landmarks), lost-person prior (trail line + clearings), person with per-view occlusion latents, decoys, RF truth (log-distance + foliage integral + correlated shadowing, uplink asymmetry), link FORECAST from the prior map (no shadowing, nominal attenuation; sigma 6 dB). Incomplete/stale prior map. `PublicInfo` is all a planner may know before launch. Split guard: `Tools/DroneData/splits.py` (dev 0-1999, val 70000-79999, HELD-OUT 90000-99999 refused). |
| `missionsim/obs_model.py` | Swappable observation model. Placeholder: per-frame NavPilot RGB model (`navpilot.sensors.detection_p`) aggregated per pass; correlated repeat views (per-view latent, ForestSearch convention); glimpses on bad views; background false alerts and persistent decoys; confidence + location noise. `load(prefer_vision=True)` uses `Tools/Vision/obs_model.py` (`ObsModel`) when it exists. Stand visibility is calibrated by sampling sight lines in NavPilot stands (`cache/vis_table.json`). |
| `missionsim/agent.py` | Knowledge only: cell posterior with correlated views, candidates with posterior AND fused location covariance (alignment error is common-mode within a drone, independent across drones), labels `CONFIRMED_GOOD_LOC` / `CONFIDENT_POOR_LOC` / `UNCONFIRMED`, link experience. `AgentView` is the only planner input. |
| `missionsim/comms.py` | Per-direction Gilbert-Elliott links driven by the RF truth margin, rate limit, delay jitter (out-of-order), hop-by-hop custody with acks (store-and-forward), base relay, partner data-mule relay, priority queues (DET/ACK > HB/LEASE > COV > MAP > IMG; FIFO ablation), forced asymmetric outages. |
| `missionsim/policies.py` | static, frontier, pod (react), coordinator, lease (default plan + time-limited leases + health beacon with "last heard you"), rendezvous (B3, da Silva-style, reimplemented), pod_maintain, pod_deadzone (deliberate dead-zone entry: onboard assignment, planned reconnect point, lease, revisions; VOI evidence decisions), pod_full (full sharing), ORACLE (labelled), and literature baselines B1 (Lin & Goodrich PoD-greedy), B2 racer_lite, B4 find-then-mule (Hayat), B4+B5 (+ Schedl re-check), B5, B6 gossip. Each adaptation is labelled in its docstring. |
| `missionsim/sim.py` | Mission loop: truth execution (true graph, energy, odometry drift + heading-bias, alignment error, landmark re-alignment, near-miss proxy), observations, comms, base fusion, coordinator, metrics, failure causes, JSONL decision logs. |
| `missionsim/stats.py` | Paired differences with cluster bootstrap over layouts. |
| `experiments.py` | `run` (sharded), `report`, `logs`. |

Budgets matched across policies: same battery (75 Wh), speeds {2,4,6} m/s capped by canopy, detector, radio and
message set (HB 1 Hz, COV 0.2 Hz, event DET; FULL sharing adds bulk MAP/IMG on the same bandwidth).

PRIMARY metric: the base holds a fused candidate with posterior >= 0.95 whose TRUE location error is <= E (10 m) at
some t <= D. Failed missions count as failures; every mission gets a failure-cause label.

Decision logs (one JSON object per decision epoch per drone): `t`, `t_ns`, `drone`, `policy`, `pose`,
`pose_sigma_m`, `cell`, `energy_wh`, `link` (heard base / partner ages, forecast P(link)), `belief` (top cells,
candidates with pi / est / sigma / label / delivered, current excursion plan), `candidate_actions` (alternatives with
scores and score terms), `chosen`, `goal`, `reason`, and `truth_debug.pose_err_m` (for the replay only; planners
never see it).

```
F=/home/lunarz/.cache/research/fs-scripts/fsrun; PY=/home/lunarz/.cache/research/fs-venv/bin/python
cd Tools/MissionSim
$F $PY -m unittest discover -s tests -p 'test_*.py'
for i in 0 1 2; do $F ./run_all.sh $i 3 val & done; wait
$F $PY experiments.py report --exp E1,E3,ABL,P7 --split val
$F $PY experiments.py logs --seed 7
```
