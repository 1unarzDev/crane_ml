# Third-party notices

## LILocBench public-data pilot

The optional CRANE-Fidelity ingestion pilot downloads `dynamics_0` bag and reference poses from
the [University of Bonn LILocBench publisher](https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/).
The data are licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
Raw data and derived exports stay in the ignored local results directory; they are not bundled
with CRANE assets. The importer changes representation through a fixed axis/yaw conversion and
extracts stream statistics. Those data remain external-platform pilot evidence.

Attribution: N. Trekel, T. Guadagnino, T. L\"abe, L. Wiesmann, P. Aguiar, J. Behley, and C. Stachniss,
*Benchmark for Evaluating Long-Term Localization in Indoor Environments Under Substantial Static
and Dynamic Scene Changes*, IROS 2025. Download URLs and hashes are recorded in
`Tools/Campus/public_pilot_sources.json`.

## PX4 Gazebo models

The `PX4 Walls Validation` reference geometry is derived from the primitive poses and dimensions
in `PX4/PX4-gazebo-models`, `worlds/walls.sdf`, commit
`bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9`.

BSD 3-Clause License

Copyright (c) 2022, PX4 Autopilot for Drones
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
   and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written
   permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Clearpath simulator

The optional offline Clearpath pipeline import derives its source geometry, textures, and SDF
metadata from `clearpathrobotics/clearpath_simulator` 2.9.4. These generated assets are not stored
in Git.

BSD 3-Clause License

Copyright (c) 2023, clearpathrobotics

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
   and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written
   permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Rover Robotics robot descriptions

`Assets/Robot Descriptions/rover/` (URDFs and meshes) is copied from `roverrobotics_description` in
[roverrobotics/roverrobotics_ros2](https://github.com/roverrobotics/roverrobotics_ros2), commit
`8f74c12a92b4237e38ba5f4bd0ce297a9cdb9955`, licensed under the Apache License 2.0. A copy of the
license is in `Assets/Robot Descriptions/rover/LICENSE-APACHE-2.0.txt`.
Copyright Rover Robotics and Jack Rivera (per upstream `package.xml`; the upstream LICENSE
copyright template is unfilled and the repository ships no NOTICE file).
Modifications: include and mesh paths were rewritten for the Unity project. `max_65`, `max_100*` and
`miti_65` are not present at upstream HEAD and probably came from an earlier upstream revision. Their
exact source commit is unverified. The URDF friction values (μ=5, 200) are upstream values, not
physical calibration.
