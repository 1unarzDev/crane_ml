# Third-party notices

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

## Generic underwater vehicle profile

`domains/sub/profiles/generic_rov8.json` holds numeric facts, each cited in the file with its location, from:
M. von Benzon et al., "An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot", J. Mar.
Sci. Eng. 10(12):1898, 2022, doi:10.3390/jmse10121898 (CC BY 4.0); C. Wu, "6-DoF Modelling and Control of a
Remotely Operated Vehicle", Flinders University, 2018; Blue Robotics' published T200 and Basic ESC
specifications; and the motor factors of ArduPilot's `AP_Motors6DOF.cpp` (SUB_FRAME_VECTORED_6DOF, GPL-3.0).
No text, data files or code from these sources are included. The 6-DOF reference model
(`engine/kernels/marine6_ref.py`) and profile loader (`marine6_profile.py`) are this repository's own code
(GPL-3.0), moved from the sub platform branch.
