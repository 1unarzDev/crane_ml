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

The aerial example card `domains/drones/profiles/x500.json` takes its numbers (mass, inertia, motor and
moment constants, motor time constants, rotor drag, rolling moment, rotor poses) from `models/x500/model.sdf` and
`models/x500_base/model.sdf` of the same repository at commit `e5997f455e33ec071c7c677fc59a65781de496a4`, under
the BSD 3-Clause License above (repository LICENSE: Copyright (c) 2022, PX4 Autopilot for Drones;
`models/x500/LICENSE`: Copyright (c) 2022, Rudis Laboratories). No mesh or SDF file is copied.

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

## Multirotor force laws and references (drones domain)

`domains/drones/model.py` and `engine/kernels/aerial.py` implement, in their own code, force laws described by:

- **gz-sim** `src/systems/multicopter_motor_model/MulticopterMotorModel.cc` (gazebosim/gz-sim, commit
  `3984434a2e9de3e7e771dfa59616b802d017e404`), Apache License 2.0, Copyright Open Source Robotics Foundation: thrust
  kf w^2, drag torque -direction kf w^2 km, rotor drag -|w| lambda1 v_perp, rolling moment -|w| mu1 v_perp. The
  laws were read; no gz-sim code is copied or linked.
- P. Martin and E. Salaun, "The true role of accelerometer feedback in quadrotor control", IEEE ICRA 2010 (the
  rotor drag form).
- I. C. Cheeseman and W. E. Bennett, "The effect of the ground on a helicopter rotor in forward flight", ARC R&M
  3021, 1955 (the ground-effect ratio).
- T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)", IEEE CDC 2010
  (the attitude error of `engine/kernels/aerial_control.py`).

## Bitcraze crazyflie-simulation

The aerial example card `domains/drones/profiles/crazyflie21.json` takes its inertia, moment constant and rotor
poses from `simulator_files/gazebo/crazyflie/model.sdf` of `bitcraze/crazyflie-simulation`, commit
`7e93752dbc803af2488c1db46bb79b3da55f5d8c`; its thrust and battery numbers are derived from Bitcraze's published
PWM-to-thrust table and the Crazyflie 2.1 datasheet, and J. Foerster, "System Identification of the Crazyflie 2.0
Nano Quadrocopter", BSc thesis, ETH Zurich, 2015, cited as facts. No file is copied.

MIT License

Copyright (c) 2022 Bitcraze

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
