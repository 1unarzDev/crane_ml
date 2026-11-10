// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Every rate, latency, noise and gain of the stack in one record. Values default to the NavPilot priors
    /// (Tools/NavPilot/navpilot/sensors.py, planner.py, stacks/core.py) unless noted; all are [U] (vendor sheets and first
    /// principles, nothing calibrated on this airframe). Canonical() is hashed into the run manifest.
    /// Kinematic quantities only (m, s, m/s, m/s^2, rad); force/torque gains live in the flight controller.
    /// </summary>
    public sealed class AutonomyConfig {
        // ---- scheduler
        public long TickNs = 5_000_000;                 // plant log tick (CRANE fixed dt)
        // ---- stereo depth (OAK-D S2 stereo pair, 80 deg HFOV, 7.5 cm baseline)
        public double DepthRateHz = 15.0, DepthLatencyS = 0.045, DepthRangeMax = 15.0, DepthRangeMin = 0.2;
        public int DepthWidth = 80, DepthHeight = 50;   // decimated ray grid (each ray carries its pixel footprint)
        public int DepthNativeWidth = 640;              // native width for pixel-width dropout and disparity noise
        public double DepthHfovDeg = 80.0, DepthVfovDeg = 55.0;
        public double DispSigmaPx = 0.15, BaselineM = 0.075, ExposureS = 0.008;
        public double ThinW50Px = 3.0, ThinWScalePx = 0.8, InvalidP = 0.03, FreeEvidenceM = 8.0;
        public double DepthMountForward = 0.10, DepthMountUp = 0.02, DepthPitchDeg = 0.0;
        // ---- LiDAR (Unitree L2-like: 360 x 96 deg used, 10 Hz sweeps, non-repetitive pattern)
        public double LidarRateHz = 10.0, LidarLatencyS = 0.065, LidarRangeMax = 20.0, LidarRangeMin = 0.05;
        public double LidarPointsPerS = 64000.0, LidarElevMinDeg = -48.0, LidarElevMaxDeg = 48.0;
        public double LidarRangeSigma = 0.02, LidarDropP = 0.02, LidarMountUp = 0.08;
        public bool UseDepth = true, UseLidar = true;
        // ---- estimator (VIO/LIO-like drift on truth)
        public double EstRateHz = 100.0, EstLatencyS = 0.010;
        public double LocalDriftSigma = 0.03;           // [m / sqrt(m)] random walk per sqrt(distance), x/y (NavPilot)
        public double LocalDriftSigmaZ = 0.01, YawDriftSigma = 0.0005;   // [m/sqrt(m)], [rad/sqrt(m)]
        public double DriftStepM = 0.25;
        public double VelNoiseSigma = 0.02;             // [m/s] white
        public double AlignOffsetX = 0.0, AlignOffsetY = 0.0, AlignRampPerM = 0.0;   // global-alignment drift (prior vs local)
        // ---- map
        public double MapRes = 0.2;
        public float LHit = 0.85f, LMiss = -0.1f, LMin = -1.0f, LMax = 3.5f, LOcc = 0.4f;   // NavPilot MAP (conservative miss)
        public double CarveBackoffM = 0.15;
        public double MapWindowXY = 11.0, MapWindowZDown = 2.5, MapWindowZUp = 2.5;   // planning window half-sizes
        // ---- prior map
        public string PriorUse = "route";               // none | route | belief
        public double PriorDropFraction = 0.0, PriorShiftX = 0.0, PriorShiftY = 0.0;
        // ---- planner (MAP_RHP port)
        public string Planner = "map_rhp";              // map_rhp | reactive
        public double PlanRateHz = 20.0;
        public double BodyRadius = 0.0 /* 0 = from the profile rotors */, InflateExtra = 0.15, Margin = 0.5;
        public double Horizon = 14.0, ReactiveHorizon = 8.0;
        public int Headings = 48; public double[] ClimbDeg = { -12, 0, 12 };
        public double[] SpeedFractions = { 1.0, 0.75, 0.5, 0.3, 0.15 };
        public double WTurn = 0.35, WFree = 0.6, WAlt = 0.4, SEval = 4.0;
        public double TrackTau = 0.33;                  // [s] closed-loop velocity time constant used for primitive rollout
        public double TauPrior = 0.12; public int AgeWindow = 200; public double AgeQuantile = 0.99;
        public double CruiseAgl = 2.2, AltMinAgl = 1.0, AltMaxAgl = 4.5;
        public double VLimit = 2.5;
        public double CtgRes = 0.5, CtgPeriodS = 0.5, CtgBandDown = 0.8, CtgBandUp = 0.8;
        // ---- compute-time model (lognormal + Pareto stall; NavPilot LATENCY_TAILS "nominal")
        public double PlanMedianS = 0.025, PlanSigma = 0.25, PlanPStall = 0.003, PlanStallMinS = 0.04, PlanStallAlpha = 2.5;
        public double MapInsertMedianS = 0.010;
        // ---- recursive feasibility and path-following brake (2026-11-10, after the harness attribution of >= 5 m/s collisions)
        // DEFAULT OFF until the paired harness sweep (Tools/CraneHarness/specs/recfeas_sweep*.json) confirms them: the queued
        // harness batch relies on bit-identity with the M5 rows. Enable all three together.
        public bool RecursiveFeasibility = false;        // feasible iff d_free >= v (tau + plan period) + D_turn(v, dheading) + margin, lateral band added
        public bool TurnBrakeTable = false;             // D_turn(v, dheading) and lateral excursion simulated on the real plant
        public bool BrakeAlongPath = false;             // fallback brake follows the last validated path instead of braking straight
        public double BrakeDecel = 7.0;                 // [m/s^2] commanded speed ramp of the path-following brake (controller MaxAccelXY 8)
        public double PathTrackGain = 1.5, PathTrackMaxLat = 1.0;   // [1/s], [m/s] cross-track correction while braking along the path
        public double SafePathMaxAgeS = 2.0;            // last validated path usable for this long
        // ---- arbiter / safety
        public double WatchdogS = 0.15;                 // no fresh planner command -> brake (NavPilot WD_S)
        public double EstimateStaleS = 0.10;            // estimator input age above this -> brake
        public double NearMissRecoverM = 0.15;          // true clearance below this -> recovery mode (slow + replan)
        public double RecoverySpeedFrac = 0.4, RecoveryHoldS = 1.0;
        public double StuckS = 12.0, StuckProgressM = 1.0;
        // ---- mission
        public double GoalRadius = 1.5;
        public double TimeLimitS = 0;                   // 0 = route length / 0.5 m/s + 30 s
        public double HardRadiusM = 0.01;               // capsules thinner than this are soft twigs (contact counted, not terminal)
        public ulong RunSeed = 1;

        public AutonomyConfig Clone() => (AutonomyConfig)MemberwiseClone();

        public string Canonical() {
            var ci = CultureInfo.InvariantCulture; var sb = new StringBuilder();
            foreach (var f in typeof(AutonomyConfig).GetFields(BindingFlags.Public | BindingFlags.Instance)) {
                object v = f.GetValue(this);
                string s = v is double d ? d.ToString("R", ci) : v is double[] a ? string.Join(",", Array.ConvertAll(a, x => x.ToString("R", ci)))
                    : Convert.ToString(v, ci);
                sb.Append(f.Name).Append('=').Append(s).Append(';');
            }
            return sb.ToString();
        }
    }

    /// <summary>SplitMix64 stream + Box-Muller (deterministic, independent of UnityEngine.Random and System.Random).</summary>
    public sealed class Rng {
        private ulong s; private double spare; private bool has;
        public Rng(ulong seed, ulong stream) { s = seed * 0x9E3779B97F4A7C15UL ^ (stream + 0x632BE59BD9B4E019UL); Next(); Next(); }
        public ulong Next() { ulong z = (s += 0x9E3779B97F4A7C15UL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL; return z ^ (z >> 31); }
        public double Uniform() => (Next() >> 11) * (1.0 / 9007199254740992.0);
        public double Normal() {
            if (has) { has = false; return spare; }
            double u, v, r;
            do { u = 2 * Uniform() - 1; v = 2 * Uniform() - 1; r = u * u + v * v; } while (r >= 1 || r == 0);
            double f = Math.Sqrt(-2 * Math.Log(r) / r); spare = v * f; has = true; return u * f;
        }
    }

    /// <summary>Per-cycle compute time: lognormal body + Bernoulli-gated Pareto stall (NavPilot compute_time).</summary>
    public sealed class ComputeTimeModel {
        public double Median, Sigma, PStall, StallMin, Alpha;
        private readonly Rng rng;
        public ComputeTimeModel(Rng rng, double median, double sigma, double pStall, double stallMin, double alpha) {
            this.rng = rng; Median = median; Sigma = sigma; PStall = pStall; StallMin = stallMin; Alpha = alpha;
        }
        public double Sample() {
            double t = Median * Math.Exp(Sigma * rng.Normal());
            double u2 = rng.Uniform(), u3 = rng.Uniform();
            if (u2 < PStall) t += StallMin * Math.Pow(Math.Max(u3, 1e-12), -1.0 / Alpha);
            return Math.Min(t, 3.0);
        }
    }
}
