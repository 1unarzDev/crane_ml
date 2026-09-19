# Reference-environment source audit

This audit records the primary-source basis for treating AWSIM and Flightmare as design
references, and for adapting Unity's Nav2/SLAM example without copying Robotics Warehouse art.
It was performed on 2026-09-19. Repository licenses indicate available permissions; this is not
legal advice. The corresponding implemented environments and validation status are documented in
[ReferenceEnvironments.md](ReferenceEnvironments.md).

## Reuse decision

| Source | Audited revision | CRANE decision |
|---|---|---|
| AWSIM | [`46a68528`](https://github.com/autowarefoundation/AWSIM/tree/46a68528b330fbcb487f9d2f3e15126990edfe0b) (2026-09-02) | Architecture reference; consider individual Apache-2.0 source files. Do not import Shinjuku or other CC BY-NC assets by default. |
| Flightmare | [`d4218aed`](https://github.com/uzh-rpg/flightmare/tree/d4218aedac18cbe9364a0a0df10ab992c4b65e4f) (2023-05-15) and Unity renderer [`203351ff`](https://github.com/uzh-rpg/flightmare_unity/tree/203351ffca0bf10ea64193456b3505abe9bfef85) (2021-04-14) | Rendering/physics separation reference; individually identified MIT code may be reused with notice. Treat environment art as uncleared third-party content. |
| Unity Nav2/SLAM example | [`a0c9846e`](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/tree/a0c9846eaa04f4800d3521639990e74c3b287ceb) (2021-10-01) | Reuse Apache-2.0 integration/configuration concepts. Build the warehouse with CRANE-owned primitives rather than copying Warehouse assets. |
| Robotics Warehouse package | [`9fa61061`](https://github.com/Unity-Technologies/Robotics-Warehouse/tree/9fa61061d31b30686573bd2e0213f049738a8949), pinned by the example | Executable upstream dependency only unless Unity clarifies terms; no asset copying or redistribution. |

## AWSIM

AWSIM moved from TIER IV to the Autoware Foundation on 2026-05-07. At the audited revision it
targets Unity 6000.0.61f1 with HDRP/URP 17.0.4, uses native ROS 2 communication, and exposes
Autoware vehicle and sensor interfaces. Its documented dependency direction is Scene → UI →
Usecase → Entity → Common. A scene explicitly controls initialization and update order instead
of leaving that order to unrelated Unity lifecycle callbacks
([architecture](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/docs/DeveloperGuide/Architecture/index.md),
[directory structure](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/docs/DeveloperGuide/Directory/index.md)).
CRANE can reuse that ownership pattern: one environment descriptor should create canonical
objects, then explicitly derive physics and presentation rather than allowing a visual prefab to
become implicit collision truth.

AWSIM's license is format-specific, not a blanket open-source grant. AWSIM-specific `.cs`,
`.compute`, and `.xml` extensions are Apache-2.0, while listed `.fbx`, `.pcd`, `.osm`, `.png`,
`.anim`, `.unitypackage`, and `.x86_64` assets are CC BY-NC. The license also prohibits public
hosting of `/docs` outside the official site
([license](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/LICENSE)).
The principal Shinjuku scene is separately downloaded as a Unity package, imported into the
ignored `Assets/Awsim/Externals` directory, and licensed CC BY-NC 4.0 with an explicit
noncommercial restriction
([setup](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/docs/DeveloperGuide/SetupUnityProject/index.md),
[downloads](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/docs/Downloads/index.md)).
The source checkout therefore does not contain a permissively redistributable Shinjuku scene.

AWSIM is not graphics-free headless. Its own experimental guide says Unity's ordinary headless
options either still require X11 or disable GPU support, while AWSIM requires a GPU for sensor
simulation. The supported workaround is `xvfb-run ./AWSIM.x86_64`, on Ubuntu only
([headless guide](https://github.com/autowarefoundation/AWSIM/blob/46a68528b330fbcb487f9d2f3e15126990edfe0b/docs/DeveloperGuide/Experimental/RunningHeadless/index.md)).
That is virtual-display GPU execution, not evidence that sensors remain valid under Unity
`-nographics`. It does not improve CRANE's CPU-only path.

**Decision:** use the explicit layer/scene-lifecycle design and, where useful, individually
verified Apache-2.0 code patterns. Do not copy AWSIM documentation or CC BY-NC environment assets
into CRANE by default. Importing Shinjuku would add licensing, scale, and validation work without
materially strengthening the current navigation-explanation study.

## Flightmare

Flightmare explicitly separates a Unity rendering server from its C++ physics/dynamics client
([README](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/README.md),
[server/client documentation](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/docs/source/first_steps/server_and_client.rst)).
The core repository contains `flightlib`, `flightrender`, `flightrl`, and ROS 1 `flightros`; it is
not a ROS 2 Jazzy integration. Its documentation names INDUSTRIAL, WAREHOUSE, GARAGE, and
NATUREFOREST and supports point-cloud export, but the separately inspected Unity repository
contains only the `Industrial` environment directory and uses Unity 2020.1.10f1
([standalone/environment guide](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/docs/source/building_flightmare_binary/standalone.rst),
[point-cloud guide](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/docs/source/first_steps/pointcloud.rst)).
The four names must not be presented as four environment packages cleared for redistribution.

Both repositories include MIT license files. However, the Unity renderer's own README says the
bundled Industrial demo was created by Dmitrii Kutsenko and obtained from the Unity Asset Store
package “RPG/FPS Game Assets for PC/Mobile (Industrial Set v2.0)”
([Unity renderer README](https://github.com/uzh-rpg/flightmare_unity/blob/203351ffca0bf10ea64193456b3505abe9bfef85/README.md)).
The core documentation also describes adding Asset Store scenes
([acknowledgements](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/docs/source/getting_started/readme.rst)).
A repository-level MIT file does not establish a right to relicense third-party Asset Store art.

Flightmare's renderer/client boundary is useful for canonical separation: physics state can be
authoritative while rendering is a consumer, and exported point clouds can be a derived artifact.
It is not proof that a visual mesh is valid collision geometry. In fact, Flightmare's FAQ reports
terrain-tree capsule colliders and a startup replacement with mesh-collider prefabs, demonstrating
that visual and collision representations can diverge
([FAQ](https://github.com/uzh-rpg/flightmare/blob/d4218aedac18cbe9364a0a0df10ab992c4b65e4f/docs/source/building_flightmare_binary/faq.rst)).

Server execution is not a qualified headless path. The same FAQ says the current standalone build
is for local desktops; server execution is technically possible but has shader problems. Because
the Unity process is the rendering server, graphics-free execution would also defeat its central
role. CRANE should not inherit this old Unity 2020/ROS 1 bridge for batch execution.

**Decision:** borrow the authority boundary—physics/canonical state first, rendering second—and
reuse only individually reviewed MIT code with its notice. Environment art is design-reference
only until each underlying asset's terms and provenance are resolved.

## Unity Robotics Nav2/SLAM example and Warehouse assets

The Unity example targets ROS 2 Galactic, Unity 2020.3.11f1, TurtleBot3 Waffle Pi, Nav2, and SLAM
Toolbox—not CRANE's Unity 6/ROS 2 Jazzy stack
([README](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/README.md),
[Dockerfile](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/ros2_docker/Dockerfile)).
Unity owns simulated time, TF, and a documented perfect, instantaneous, noiseless 2-D lidar. The
TurtleBot is URDF-imported, after which its base/caster collision and laser are manually adjusted.
`SimpleWarehouseScene` is generated using Robotics Warehouse; the ROS side is largely standard
Nav2 and SLAM Toolbox
([implementation explanation](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/readmes/explanation.md)).
The documented reusable seam is the configured robot prefab, which may be placed in another
mostly flat scene whose objects the lidar can detect
([custom-scene guide](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/readmes/custom_viz.md)).

The example itself is Apache-2.0 and its third-party notice identifies TurtleBot3 as Apache-2.0
and FreeCam as MIT
([notices](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/Third%20Party%20Notices.md)).
Its package lock pins Robotics Warehouse at `9fa61061d31b30686573bd2e0213f049738a8949`
([lockfile](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/a0c9846eaa04f4800d3521639990e74c3b287ceb/Nav2SLAMExampleProject/Packages/packages-lock.json)).
At that revision, Robotics Warehouse has no LICENSE or NOTICE file, and its
[`package.json`](https://github.com/Unity-Technologies/Robotics-Warehouse/blob/9fa61061d31b30686573bd2e0213f049738a8949/com.unity.robotics.warehouse/package.json)
has no license field. Apache-2.0 on the example therefore must not be extended to the dependency's
meshes and textures.

No repository documentation inspected here qualifies this example as headless or validates its
camera/lidar outputs under `-nographics`. CRANE's own headless fixture, not upstream silence, is
the evidence for the current primitive warehouse adaptation.

**Decision:** retain the Apache-2.0 integration ideas, robot dimensions/configuration, and source
attribution. Do not copy Warehouse meshes or textures. CRANE's procedural primitives with
separate canonical colliders and non-colliding presentation objects are the smallest auditable
adaptation.

## Contract for CRANE adaptations

1. Canonical descriptors own stable object identity, dimensions, transforms, provenance, and
   source hashes.
2. Physics/collision and navigation occupancy are explicit derived products. A renderer never
   becomes an authoritative collider by accident.
3. Presentation is replaceable and non-authoritative. Asset license and content hashes remain in
   the manifest even when the presentation layer is not distributed.
4. Simulator/evaluator truth is physically separated from robot-visible observations. Geometry
   proves that an object existed, not that a sensor observed it or a controller consumed it.
5. Headless support is a tested sensor-and-physics property, not the fact that a binary accepts a
   command-line flag. GPU virtual-display execution, CPU `-nographics`, and local desktop
   standalone runs are reported as distinct modes.

