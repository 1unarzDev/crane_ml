using System;
using Sim.Utils.Performance;
using UnityEngine;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>
    /// Pressure + DVL adapter. Inert unless the CraneUnderwaterBody on the same object is active (i.e. the profile flag is set).
    /// Samples every N physics ticks, stamps with the physics tick (tick * fixedDeltaTime), seeds noise per episode and clears
    /// everything on reset. Ground truth (seabed depth, true current, pose) stays private; only Observation structs are public.
    /// </summary>
    [DisallowMultipleComponent, RequireComponent(typeof(CraneUnderwaterBody))]
    public sealed class CraneUnderwaterSensors : MonoBehaviour, ICraneEpisodeResettable {
        [SerializeField] int pressurePeriodTicks = 5;
        [SerializeField] int dvlPeriodTicks = 10;
        [SerializeField] float seabedDepthMeters = 10f;      // hidden environment truth
        [SerializeField] Vector3 waterCurrentNed;            // hidden environment truth (used by water-track only)
        [SerializeField] bool waterTrack;

        CraneUnderwaterBody body;
        readonly CranePressureSensor pressure = new CranePressureSensor();
        readonly CraneDvl dvl = new CraneDvl();
        CraneRng rngPressure = new CraneRng(0), rngDvl = new CraneRng(0);
        long episodeSeed;
        long tick, pSeq, dSeq;
        double dt;
        CranePressureObservation latestPressure;
        CraneDvlObservation latestDvl;

        public CranePressureObservation LatestPressure => latestPressure;
        public CraneDvlObservation LatestDvl => latestDvl;
        public long TickCount => tick;
        public int ResetPriority => -45;

        public CranePressureSensor PressureModel => pressure;
        public CraneDvl DvlModel => dvl;

        void Awake() { body = GetComponent<CraneUnderwaterBody>(); Reseed(0); }

        public void Configure(long seed, double seabedDepth, Vector3 currentNed, bool useWaterTrack) {
            seabedDepthMeters = (float)seabedDepth; waterCurrentNed = currentNed; waterTrack = useWaterTrack;
            Reseed(seed);
        }

        void Reseed(long seed) {
            episodeSeed = seed;
            rngPressure.Reseed(CraneRng.Mix(seed, 1));
            rngDvl.Reseed(CraneRng.Mix(seed, 2));
        }

        void FixedUpdate() => Tick(Time.fixedDeltaTime);

        /// <summary>Advance one physics tick: sample from the Rigidbody-measured state when a period elapses.</summary>
        public void Tick(double fixedDt) {
            if (body == null) body = GetComponent<CraneUnderwaterBody>();
            if (!body.Active) return;
            dt = fixedDt;
            if (tick % pressurePeriodTicks == 0) {
                var s = body.MeasureState();
                double p = pressure.Measure(s.Pos, s.Q, rngPressure);
                latestPressure = new CranePressureObservation(tick, tick * dt, ++pSeq, p, pressure.DepthFromPressure(p));
            }
            if (tick % dvlPeriodTicks == 0) {
                var s = body.MeasureState();
                var m = dvl.Measure(s.Pos, s.Q, s.Nu, seabedDepthMeters,
                    new[] { (double)waterCurrentNed.x, waterCurrentNed.y, waterCurrentNed.z }, waterTrack, rngDvl);
                double alt = m.Lock ? dvl.Altitude(m, s.Q) : double.NaN;
                int mask = 0; for (int i = 0; i < m.Valid.Length; i++) if (m.Valid[i]) mask |= 1 << i;
                latestDvl = new CraneDvlObservation(tick, tick * dt, ++dSeq, m.Lock, waterTrack, m.VelocitySensor, alt, mask);
            }
            tick++;
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context, CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            tick = 0; pSeq = 0; dSeq = 0;
            latestPressure = default; latestDvl = default;
            Reseed(context.Seed);
        }
    }
}
