# Public land data candidates

Audit date: 2026-10-11. These are candidate pilot sources, not CRANE platform
validation. CRANE hardware and its synchronized robot logs are not yet available.

| Candidate | Download and rights audit | Timing, commands, independent truth | Suitability |
|---|---|---|---|
| LILocBench Dingo dynamics | Successfully acquired in this work; see [pilot evidence](CRANE-Fidelity-PilotEvidence.md) for pinned source, file hashes, and data rights | Root pilot inspected commands and separate ground-truth poses; unstamped command timing remains a limit | Highest current priority for testing offline ingestion and temporal evidence; different robot, so no TurtleBot plant-fit claim |
| TurtleBot3 Waffle Pi / MOCAP teaching dataset | First-party download script is public; HTTP and HTTPS bag endpoint both failed to connect during this audit. Repository is GPL-3.0; separate external bag redistribution terms were not confirmed | Sensor/odom/TF topics and independent 60 Hz MOCAP; published topic listing omits `cmd_vel`. Fixed bag applies a constant MOCAP time offset and replaces record time with header time | Best platform match and independent truth; useful sensor/odometry pilot if download is restored, unsuitable for actuator calibration without command data |
| TurtleBot5G, Zenodo 14995354 | Record public but files restricted; unauthenticated API returned no files. Metadata declares CC-BY-4.0 | ROS 2 bag includes `/cmd_vel_stamped` and odometry/delay streams. Published topic list does not show independent MOCAP/RTK truth | Good timing pilot once access is authorized; cannot independently validate plant trajectories from odometry alone |

The [TurtleBot3 owner README](https://github.com/guilhermelawless/turtlebot3_datasets/blob/master/README.md)
documents Waffle Pi hardware, sensor topics, MOCAP markers, and frames. Its
[timestamp repair script](https://github.com/guilhermelawless/turtlebot3_datasets/blob/master/scripts/fix_stamps.py)
records a 3961.461462163-second constant MOCAP correction; this is correction
evidence, not a measured bound on residual synchronization error. The
[download script](https://github.com/guilhermelawless/turtlebot3_datasets/blob/master/scripts/download_dataset.sh)
points to `dantecloud.isr.tecnico.ulisboa.pt`; both ports failed from this workspace
on the audit date. No robot bag was downloaded or redistributed from this source.
The [repository license](https://github.com/guilhermelawless/turtlebot3_datasets/blob/master/LICENSE)
does not by itself resolve distinct terms for externally hosted logs.

The [TurtleBot5G record](https://zenodo.org/records/14995354) exposes topic counts
and explicitly marks its files restricted. The
[record API](https://zenodo.org/api/records/14995354) reports `access_right:
restricted`, license `cc-by-4.0`, and an empty unauthenticated file list. Its
camera calibration is documented as missing or incorrect. No access request,
login, author email, or raw-file download was made; resolving restricted access
requires the user's account or an authorized request.

Public sources can exercise parsers, timestamp analysis, frame conversion, and
residual reporting. They do not replace CRANE's measured mass/actuator profile,
repeat runs over matched routes/days/materials, acquisition-to-actuation clock
provenance, or paired physical closed-loop navigation experiments. A fixed
timestamp repair or a SLAM trajectory is insufficient independent evidence for
those claims.
