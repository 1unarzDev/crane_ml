using System;
using System.IO;
using Sim.Physics.Aerial;
using UnityEngine;

namespace Crane.Theater.Aerial {
    /// <summary>The real AOS 7 V5 plant (Crane.Aerial.Physics: 1 kHz RK4 sub-steps, motor lag, battery, cascaded controller)
    /// tracking a reference trajectory. Deterministic for a seed (no turbulence unless wind is set). Rotor speeds come from the
    /// plant state, ArduPilot Quad-X order, which is what DroneVisualRig.SetRotorSpeeds expects. Advance steps must be whole
    /// milliseconds (the plant rounds dt to its 1 ms sub-step), so bake at 5 ms.</summary>
    public sealed class CranePlantAdapter : IAerialPlantFollower {
        CraneAerialPlantCore plant; CraneMultirotorController ctl;
        public string Name { get; private set; } = "aerial-v2 plant";
        double time;

        public static CranePlantAdapter TryCreate() {
            try { var a = new CranePlantAdapter(); a.Load(); return a; }
            catch (Exception e) { Debug.LogWarning("CranePlantAdapter unavailable, using kinematic stub: " + e.Message); return null; }
        }

        void Load() {
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles"));
            var prof = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve("aos7_v5_f90_6s", dir), true, true);
            var spec = prof.Vehicle;
            spec.GroundHeight = -1000.0;   // forest terrain is not flat; the demo flight never touches it
            plant = new CraneAerialPlantCore(spec, 1000.0, 500.0);
            ctl = new CraneMultirotorController(spec, plant.ControlDt);
            plant.CommandSource = ctl;
            Name = "aerial-v2 plant (" + prof.Id + " sha256 " + prof.Sha256.Substring(0, 8) + ")";
        }

        static double[] Enu(Vector3 u) => new double[] { u.x, u.z, u.y };

        public void Reset(Vector3 pos, Quaternion rot, int seed) {
            var f = rot * Vector3.forward; var u = rot * Vector3.up;
            var s = CraneAerialPlantCore.HoverState(plant.Spec, Enu(pos));
            s.Q = CraneAerialFrames.QuatFromUnityAxes(new double[] { f.x, f.y, f.z }, new double[] { u.x, u.y, u.z });
            plant.Reset(s, (ulong)seed);
            ctl.ResetIntegrators();
            ctl.HoldPosition(s.P, CraneAerialFrames.YawEnu(s.Q));
            time = 0;
        }

        public TrajSample Step(double dt, TrajSample r) {
            var f = r.rot * Vector3.forward;
            double yaw = Math.PI / 2 - Mathf.Atan2(f.x, f.z);   // Unity yaw (cw from +z) to ENU yaw (ccw from east)
            ctl.HoldPosition(Enu(r.pos), yaw);
            var v = Enu(r.vel); Array.Copy(v, ctl.VelocitySetpoint, 3);
            plant.Advance(dt); time += dt;
            var st = plant.State;
            CraneAerialFrames.UnityLookAxes(st.Q, out double[] fw, out double[] up);
            var w = new float[4];
            for (int i = 0; i < 4; i++) w[i] = (float)Math.Abs(st.RotorSpeed[i]);
            var pu = CraneAerialFrames.EnuToUnity(st.P); var vu = CraneAerialFrames.EnuToUnity(st.V);
            return new TrajSample {
                t = r.t, pos = new Vector3((float)pu[0], (float)pu[1], (float)pu[2]), vel = new Vector3((float)vu[0], (float)vu[1], (float)vu[2]),
                rot = Quaternion.LookRotation(new Vector3((float)fw[0], (float)fw[1], (float)fw[2]), new Vector3((float)up[0], (float)up[1], (float)up[2])),
                rotorRadS = w
            };
        }
    }
}
