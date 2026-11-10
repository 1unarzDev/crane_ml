using System;

namespace Sim.Physics.Aerial {
    /// <summary>Anything that turns plant state into per-rotor speed commands [rad/s] at control ticks.
    /// The built-in cascaded controller implements it; a PX4 SITL lockstep bridge would too.</summary>
    public interface IAerialRotorCommandSource {
        /// <param name="state">truth state at the tick (read-only by contract)</param>
        /// <param name="time">tick time = tickIndex * controlDt [s]</param>
        double[] RotorCommands(StateV2 state, double time);
    }

    /// <summary>
    /// Unity-independent sub-stepped multirotor plant. Advance(dt) integrates rotors and rigid body
    /// with CraneRotorModelV2.Step at SubstepHz (default 1 kHz) regardless of Unity's fixed step;
    /// the command source is polled every ControlDt (ZOH), wind = mean + Dryden gust per sub-step,
    /// the IMU accumulates every sub-step and emits at its own rate. Deterministic given seed.
    /// </summary>
    public sealed class CraneAerialPlantCore {
        public readonly VehicleSpecV2 Spec;
        public StateV2 State { get; private set; }
        public readonly double SubstepDt;
        public readonly int ControlEvery;
        public double ControlDt => SubstepDt * ControlEvery;
        public bool Contact = true;
        public double[] MeanWindWorld = new double[3];
        public CraneDrydenTurbulence Turbulence;           // null = no gusts
        public IAerialRotorCommandSource CommandSource;    // null = hold RotorCommand
        public double[] RotorCommand;
        public CraneAerialImu Imu;
        public double[] LastWindWorld { get; private set; } = new double[3];
        public long SubstepIndex { get; private set; }
        private long controlTick;
        private int imuEvery = 1;

        public CraneAerialPlantCore(VehicleSpecV2 spec, double substepHz = 1000.0, double controlHz = 500.0) {
            if (!(substepHz >= 1000.0)) throw new ArgumentOutOfRangeException(nameof(substepHz), "plant must run at >= 1 kHz");
            Spec = spec;
            SubstepDt = 1.0 / substepHz;
            ControlEvery = Math.Max(1, (int)Math.Round(substepHz / controlHz));
            State = new StateV2(spec.Rotors.Length);
            RotorCommand = new double[spec.Rotors.Length];
        }

        public void AttachImu(CraneAerialImu imu) {
            Imu = imu;
            double ratio = 1.0 / (SubstepDt * imu.RateHz);
            imuEvery = (int)Math.Round(ratio);
            if (imuEvery < 1 || Math.Abs(ratio - imuEvery) > 1e-6)
                throw new ArgumentException("IMU rate must divide the sub-step rate.");
        }

        /// <summary>Episode reset: state, tick counters, turbulence and IMU noise all restart from seed.</summary>
        public void Reset(StateV2 initial, ulong seed) {
            State = initial.Copy();
            SubstepIndex = 0; controlTick = 0;
            Array.Clear(RotorCommand, 0, RotorCommand.Length);
            Array.Copy(initial.RotorSpeed, RotorCommand, RotorCommand.Length);
            Turbulence?.Reset(seed);
            Imu?.Reset(seed ^ 0xA5A5A5A5UL);
            LastWindWorld = new double[3];
        }

        public static StateV2 HoverState(VehicleSpecV2 spec, double[] pEnu, double yaw = 0.0) {
            var s = new StateV2(spec.Rotors.Length);
            Array.Copy(pEnu, s.P, 3);
            s.Q = new[] { Math.Cos(yaw / 2), 0, 0, Math.Sin(yaw / 2) };
            double w = CraneRotorModelV2.HoverRotorSpeed(spec);
            for (int i = 0; i < s.RotorSpeed.Length; i++) s.RotorSpeed[i] = w;
            return s;
        }

        /// <summary>Advance by dt (typically Time.fixedDeltaTime) in ceil(dt / SubstepDt) equal sub-steps.</summary>
        public void Advance(double dt) {
            int n = Math.Max(1, (int)Math.Round(dt / SubstepDt));
            double h = Math.Abs(n * SubstepDt - dt) < 1e-12 ? SubstepDt : dt / n;
            for (int k = 0; k < n; k++) Substep(h);
        }

        private void Substep(double h) {
            if (CommandSource != null && SubstepIndex % ControlEvery == 0) {
                double[] cmd = CommandSource.RotorCommands(State, controlTick * ControlDt);
                Array.Copy(cmd, RotorCommand, RotorCommand.Length);
                controlTick++;
            }
            double[] wind = MeanWindWorld;
            if (Turbulence != null) {
                double[] g = Turbulence.Step(h, State.P[2] - Spec.GroundHeight, MeanWindWorld, State.V);
                wind = new[] { MeanWindWorld[0] + g[0], MeanWindWorld[1] + g[1], MeanWindWorld[2] + g[2] };
            }
            LastWindWorld = wind;
            StateV2 prev = State;
            State = CraneRotorModelV2.Step(Spec, prev, RotorCommand, h, wind, Contact);
            SubstepIndex++;
            Imu?.Accumulate(prev, State, h, SubstepIndex % imuEvery == 0);
        }
    }
}
