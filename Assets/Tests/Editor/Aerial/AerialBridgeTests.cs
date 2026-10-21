using System;
using NUnit.Framework;
using UnityEngine;
using Sim.Physics.Aerial;
using Sim.Sensors.Nav;

namespace Sim.Tests.Aerial {
    /// <summary>
    /// Sign/frame contract of ArduPilotJsonFrames against the ArduPilot SITL JSON readme
    /// (libraries/SITL/examples/JSON/readme.md @9c412ba): position/velocity NED, gyro and
    /// accel_body in the FRD body frame, attitude = aerospace roll/pitch/yaw, double timestamp.
    /// The readme states frames, not Euler sign conventions; the roll/pitch/yaw convention is the
    /// aerospace ZYX one implied by NED/FRD, and is cross-checked below by geometry through
    /// UnityEngine rotations rather than by formula.
    /// </summary>
    [Category("AerialBridge")]
    public class AerialBridgeTests {
        const double Tol = 1e-9;
        static readonly double Deg = Math.PI / 180.0;

        static QuatU Q(Quaternion q) => new(q.x, q.y, q.z, q.w);
        static Vec3d V(Vector3 v) => new(v.x, v.y, v.z);
        static void AssertVec(Vec3d v, double x, double y, double z, double tol = Tol, string msg = "") {
            Assert.AreEqual(x, v.X, tol, msg + " x");
            Assert.AreEqual(y, v.Y, tol, msg + " y");
            Assert.AreEqual(z, v.Z, tol, msg + " z");
        }

        // ---- A1 counterexamples ------------------------------------------------------------
        [Test] public void FiveMetersAltitudeIsDownMinusFive() {
            Vec3d ned = ArduPilotJsonFrames.UnityToNed(new Vec3d(1.0, 5.0, 2.0));
            AssertVec(ned, 2.0, 1.0, -5.0, Tol, "N=Unity z, E=Unity x, D=-Unity y");
        }

        [Test] public void ClimbIsNegativeDownVelocity() {
            AssertVec(ArduPilotJsonFrames.UnityToNed(new Vec3d(0, 1.0, 0)), 0, 0, -1.0);
        }

        [Test] public void RollTenDegreesAboutUnityZIsMinusTenDegrees() {
            // Unity +z rotation tips the up axis toward -x (left): right wing UP = negative aerospace roll.
            Vec3d rpy = ArduPilotJsonFrames.AttitudeEuler(Q(Quaternion.Euler(0, 0, 10)));
            AssertVec(rpy, -10 * Deg, 0, 0, 1e-6);
            Assert.AreEqual(-0.17453292519943295, rpy.X, 1e-6);
        }

        [Test] public void PitchTenDegreesAboutUnityXIsMinusTenDegrees() {
            // Unity +x rotation tips the forward axis down: nose down = negative pitch.
            AssertVec(ArduPilotJsonFrames.AttitudeEuler(Q(Quaternion.Euler(10, 0, 0))), 0, -10 * Deg, 0, 1e-6);
        }

        [Test] public void YawNinetyDegreesFacesEast() {
            AssertVec(ArduPilotJsonFrames.AttitudeEuler(Q(Quaternion.Euler(0, 90, 0))), 0, 0, Math.PI / 2, 1e-6);
        }

        [Test] public void NorthRateAtNinetyDegreeHeadingGivesZeroBodyRoll() {
            // World rate about Unity +z (north) at heading east: that is the body's pitch axis.
            // The legacy bridge sent w.z as roll (0.5); the FRD gyro is roll 0, pitch +0.5.
            Vec3d g = ArduPilotJsonFrames.GyroFrd(Q(Quaternion.Euler(0, 90, 0)), new Vec3d(0, 0, 0.5));
            AssertVec(g, 0.0, 0.5, 0.0, 1e-6, "gyro FRD");
        }

        [Test] public void YawRateAboutUnityUpIsPositiveBodyZ() {
            AssertVec(ArduPilotJsonFrames.GyroFrd(Q(Quaternion.identity), new Vec3d(0, 0.3, 0)), 0, 0, 0.3, 1e-6);
            // Same world rate on a pitched body still projects onto the body axes (R^T w).
            var q = Quaternion.Euler(0, 90, 0);
            AssertVec(ArduPilotJsonFrames.GyroFrd(Q(q), new Vec3d(0, 0.3, 0)), 0, 0, 0.3, 1e-6);
        }

        [Test] public void GyroAndAttitudeAgreeOnRotationSense() {
            // Rotate a level, north-facing body by a small world rotation; d(rpy)/dt must equal gyro.
            Vector3 w = new(0.1f, 0.2f, -0.3f);
            double dt = 1e-6;
            Quaternion q2 = Quaternion.AngleAxis((float)(w.magnitude * dt / Deg), w.normalized);
            Vec3d rpy = ArduPilotJsonFrames.AttitudeEuler(Q(q2));
            Vec3d gyro = ArduPilotJsonFrames.GyroFrd(Q(Quaternion.identity), V(w));
            Assert.AreEqual(gyro.X, rpy.X / dt, 5e-3, "roll rate");
            Assert.AreEqual(gyro.Y, rpy.Y / dt, 5e-3, "pitch rate");
            Assert.AreEqual(gyro.Z, rpy.Z / dt, 5e-3, "yaw rate");
        }

        [Test] public void EulerMatchesBodyAxesBuiltFromUnityRotation() {
            // Independent of formulas: map the real UnityEngine body axes to NED and compare with
            // the aerospace ZYX direction cosines of (roll, pitch, yaw).
            foreach (var e in new[] { new Vector3(10, 30, 20), new Vector3(-25, 200, 15), new Vector3(5, -70, -40) }) {
                Quaternion q = Quaternion.Euler(e);
                Vec3d rpy = ArduPilotJsonFrames.AttitudeEuler(Q(q));
                Vec3d fwd = ArduPilotJsonFrames.UnityToNed(V(q * Vector3.forward));
                Vec3d right = ArduPilotJsonFrames.UnityToNed(V(q * Vector3.right));
                Vec3d down = ArduPilotJsonFrames.UnityToNed(V(q * Vector3.down));
                double sr = Math.Sin(rpy.X), cr = Math.Cos(rpy.X), sp = Math.Sin(rpy.Y), cp = Math.Cos(rpy.Y);
                double sy = Math.Sin(rpy.Z), cy = Math.Cos(rpy.Z);
                AssertVec(fwd, cy * cp, sy * cp, -sp, 1e-5, "forward " + e);
                AssertVec(right, cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr, 1e-5, "right " + e);
                AssertVec(down, cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr, 1e-5, "down " + e);
            }
        }

        [Test] public void EulerAnglesAreWrappedToPlusMinusPi() {
            // Unity eulerAngles are in [0, 360); the legacy bridge sent them raw. Heading 200 deg
            // must be -160 deg; Unity z 190 deg is aerospace roll -190 deg, i.e. +170 deg.
            Vec3d y = ArduPilotJsonFrames.AttitudeEuler(Q(Quaternion.Euler(0, 200, 0)));
            Assert.AreEqual(-160 * Deg, y.Z, 1e-6);
            Vec3d r = ArduPilotJsonFrames.AttitudeEuler(Q(Quaternion.Euler(0, 0, 190)));
            Assert.AreEqual(170 * Deg, r.X, 1e-6);
        }

        [Test] public void AllAttitudeOutputsStayWithinRange() {
            var rng = new System.Random(7);
            for (int i = 0; i < 2000; i++) {
                Quaternion q = Quaternion.Euler((float)(rng.NextDouble() * 360), (float)(rng.NextDouble() * 360),
                    (float)(rng.NextDouble() * 360));
                Vec3d e = ArduPilotJsonFrames.AttitudeEuler(Q(q));
                Assert.LessOrEqual(Math.Abs(e.X), Math.PI + 1e-6);
                Assert.LessOrEqual(Math.Abs(e.Y), Math.PI / 2 + 1e-6);
                Assert.LessOrEqual(Math.Abs(e.Z), Math.PI + 1e-6);
            }
        }

        // ---- accelerometer: specific force, not a constant ---------------------------------
        [Test] public void LevelHoverReadsMinusGOnZ() {
            Vec3d a = ArduPilotJsonFrames.AccelBodyFrd(Q(Quaternion.identity), new Vec3d(0, 0, 0), new Vec3d(0, -9.81, 0));
            AssertVec(a, 0, 0, -9.81);
        }

        [Test] public void FreeFallReadsZero() {
            AssertVec(ArduPilotJsonFrames.AccelBodyFrd(Q(Quaternion.Euler(12, 34, 56)),
                new Vec3d(0, -9.81, 0), new Vec3d(0, -9.81, 0)), 0, 0, 0);
        }

        [Test] public void ForwardAccelerationAppearsOnBodyX() {
            AssertVec(ArduPilotJsonFrames.AccelBodyFrd(Q(Quaternion.identity),
                new Vec3d(0, 0, 2.0), new Vec3d(0, -9.81, 0)), 2.0, 0, -9.81);
        }

        [Test] public void AttitudeAtRestMatchesAerospaceSpecificForce() {
            // At rest the accelerometer reads (g sin th, -g sin ph cos th, -g cos ph cos th) in FRD.
            const double g = 9.81;
            foreach (var e in new[] { new Vector3(10, 30, 20), new Vector3(-25, 200, 15), new Vector3(0, 0, 10) }) {
                Quaternion q = Quaternion.Euler(e);
                Vec3d rpy = ArduPilotJsonFrames.AttitudeEuler(Q(q));
                Vec3d f = ArduPilotJsonFrames.AccelBodyFrd(Q(q), new Vec3d(0, 0, 0), new Vec3d(0, -g, 0));
                AssertVec(f, g * Math.Sin(rpy.Y), -g * Math.Sin(rpy.X) * Math.Cos(rpy.Y),
                    -g * Math.Cos(rpy.X) * Math.Cos(rpy.Y), 1e-5, "rest " + e);
            }
        }

        [Test] public void AccelerationComesFromVelocityDifference() {
            Vec3d a = ArduPilotJsonFrames.AccelerationFromVelocities(new Vec3d(0, 0, 1), new Vec3d(0, 0, 1.04), 0.02);
            AssertVec(a, 0, 0, 2.0, 1e-9);
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                ArduPilotJsonFrames.AccelerationFromVelocities(new Vec3d(), new Vec3d(), 0.0));
        }

        // ---- timestamp ---------------------------------------------------------------------
        [Test] public void TimestampIsDoubleAndResolvesOneTickLateInTheRun() {
            double dt = 2822399.0 / 141120000.0;
            long tick = 50_000_000;                       // ~11.5 days of 50 Hz physics
            double d = ArduPilotJsonFrames.Timestamp(tick + 1, dt) - ArduPilotJsonFrames.Timestamp(tick, dt);
            Assert.AreEqual(dt, d, 1e-7);
            float f = (float)ArduPilotJsonFrames.Timestamp(tick, dt), f1 = (float)ArduPilotJsonFrames.Timestamp(tick + 1, dt);
            Assert.Greater(Math.Abs((f1 - f) - dt), 0.01, "float32 timestamps cannot resolve one tick here");
        }

        [Test] public void PacketKeepsDoublePrecisionThroughJsonUtility() {
            var p = new SITLCommsJsonOutputPacketV2 { timestamp = 1234567.890123 };
            p.position = new[] { 1.0000000001, 2.0, -5.0 };
            var back = JsonUtility.FromJson<SITLCommsJsonOutputPacketV2>(JsonUtility.ToJson(p));
            Assert.AreEqual(p.timestamp, back.timestamp, 1e-9);
            Assert.AreEqual(p.position[0], back.position[0], 1e-12);
            Assert.AreEqual(3, back.imu.gyro.Length);
            StringAssert.Contains("\"accel_body\"", JsonUtility.ToJson(p));
        }

        // ---- servo order / spin ------------------------------------------------------------
        [Test] public void QuadXChannelTableMatchesArduCopterMotorMatrix() {
            // AP_MotorsMatrix.cpp @9c412ba MOTOR_FRAME_TYPE_X: {angle (cw from front), yaw factor} per output.
            double[] angle = { 45, -135, -45, 135 };
            int[] yawCcw = { 1, 1, -1, -1 };
            for (int ch = 0; ch < 4; ch++) {
                double fwd = Math.Cos(angle[ch] * Deg), right = Math.Sin(angle[ch] * Deg);
                string name = (fwd > 0 ? "front_" : "rear_") + (right > 0 ? "right" : "left");
                Assert.AreEqual(name, ArduPilotJsonFrames.QuadXChannelNames[ch], "channel " + ch);
                Assert.AreEqual(yawCcw[ch], ArduPilotJsonFrames.QuadXSpin[ch], "spin " + ch);
            }
        }

        [Test] public void CraneLegacyRotorOrderMapsAndSpinMismatchIsDetected() {
            // CRANE order FL, FR, RR, RL at +-0.23 m; legacy yawSigns {1,-1,1,-1} = spin FL +1, FR -1, RR +1, RL -1.
            var pos = new[] { new[] { .23, .23, 0 }, new[] { .23, -.23, 0 }, new[] { -.23, -.23, 0 }, new[] { -.23, .23, 0 } };
            int[] map = ArduPilotJsonFrames.ChannelToRotorByGeometry(pos, new[] { 1, -1, 1, -1 }, out bool ok);
            CollectionAssert.AreEqual(new[] { 1, 3, 0, 2 }, map);
            CollectionAssert.AreEqual(ArduPilotJsonFrames.QuadXChannelToCraneRotor, map);
            Assert.IsFalse(ok, "legacy spin pattern is the mirror image of ArduCopter");
            map = ArduPilotJsonFrames.ChannelToRotorByGeometry(pos, new[] { -1, 1, -1, 1 }, out ok);
            Assert.IsTrue(ok);
        }

        [Test] public void X500ProfileOrderIsAlreadyArduCopterOrder() {
            var pos = new[] { new[] { .174, -.174, .06 }, new[] { -.174, .174, .06 }, new[] { .174, .174, .06 }, new[] { -.174, -.174, .06 } };
            int[] map = ArduPilotJsonFrames.ChannelToRotorByGeometry(pos, new[] { 1, 1, -1, -1 }, out bool ok);
            CollectionAssert.AreEqual(new[] { 0, 1, 2, 3 }, map);
            Assert.IsTrue(ok);
            Assert.Throws<ArgumentException>(() =>
                ArduPilotJsonFrames.ChannelToRotorByGeometry(new[] { pos[0], pos[0], pos[2], pos[3] }, new[] { 1, 1, -1, -1 }, out _));
        }

        [Test] public void PwmNormalization() {
            Assert.AreEqual(0.0, ArduPilotJsonFrames.NormalizePwm(900, 1000, 2000));
            Assert.AreEqual(0.68, ArduPilotJsonFrames.NormalizePwm(1680, 1000, 2000), 1e-12);
            Assert.AreEqual(1.0, ArduPilotJsonFrames.NormalizePwm(2500, 1000, 2000));
            Assert.Throws<ArgumentException>(() => ArduPilotJsonFrames.NormalizePwm(1500, 2000, 1000));
        }

        // ---- flags -------------------------------------------------------------------------
        [Test] public void LegacyBridgeRefusesWithoutOverride() {
            string[] none = { "CRANE.x86_64", "--crane-profile", "train-cpu" };
            Assert.AreEqual(CraneAerialOptions.BridgeMode.LegacyRefused, CraneAerialOptions.ResolveBridge(none));
            StringAssert.Contains("--crane-allow-legacy-sitl-bridge", CraneAerialOptions.LegacyRefusalMessage);
            StringAssert.Contains("ardupilot-json-v2", CraneAerialOptions.LegacyRefusalMessage);
            Assert.AreEqual(CraneAerialOptions.BridgeMode.LegacyAllowed,
                CraneAerialOptions.ResolveBridge(new[] { "--crane-allow-legacy-sitl-bridge" }));
        }

        [Test] public void V2BridgeFlagParsesBothSpellings() {
            Assert.AreEqual(CraneAerialOptions.BridgeMode.ArduPilotJsonV2,
                CraneAerialOptions.ResolveBridge(new[] { "--crane-sitl-bridge=ardupilot-json-v2" }));
            Assert.AreEqual(CraneAerialOptions.BridgeMode.ArduPilotJsonV2,
                CraneAerialOptions.ResolveBridge(new[] { "--crane-sitl-bridge", "ardupilot-json-v2" }));
            Assert.Throws<ArgumentException>(() =>
                CraneAerialOptions.ResolveBridge(new[] { "--crane-sitl-bridge=bogus" }));
            Assert.AreEqual("x500_px4_gz",
                CraneAerialOptions.ReadAerialProfile(new[] { "--crane-aerial-profile=x500_px4_gz" }));
            Assert.IsNull(CraneAerialOptions.ReadAerialProfile(new[] { "--crane-aerial-profile" }));
        }
    }
}
