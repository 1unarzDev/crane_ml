using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    // Pose2 (planar x, y, yaw) is the docking core's Crane.Fleet.Pose2 (Docking/Geometry.cs); one type for both.

    /// <summary>Holonomic go-to: P control on the position error in the body frame and on the yaw error, speed-limited.</summary>
    public static class GoToController {
        public static Twist Command(Pose2 pose, double x, double y, double yaw, double kp, double kyaw, double vmax, double wmax, out double dist, out double yawErr) {
            double ex = x - pose.X, ey = y - pose.Y;
            yawErr = Angles.Wrap(yaw - pose.Yaw);
            dist = Math.Sqrt(ex * ex + ey * ey);
            double c = Math.Cos(pose.Yaw), sn = Math.Sin(pose.Yaw);
            double bx = c * ex + sn * ey, by = -sn * ex + c * ey;
            double vx = kp * bx, vy = kp * by, sp = Math.Sqrt(vx * vx + vy * vy);
            if (sp > vmax) { vx *= vmax / sp; vy *= vmax / sp; }
            double wz = Math.Max(-wmax, Math.Min(wmax, kyaw * yawErr));
            return new Twist(vx, vy, wz);
        }
    }

    public static class Angles {
        public static double Wrap(double a) {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            return a;
        }
    }

    /// <summary>A step boundary the runner logs (with truth snapshots it takes itself).</summary>
    public sealed class StepEvent {
        public string Rover, Kind, Step;   // Kind: step_start | step_end
        public int Index;
        public bool TimedOut;
        public Twist Commanded;            // for open-loop steps
        public double Duration;
    }

    /// <summary>
    /// The seeded qualification script for one rover (not autonomy): wait for its departure time, follow its
    /// route holding heading with a P controller, then open-loop strafe left/right and a rotate-in-place.
    /// Deterministic for a seed: the seed jitters the departure time; noise lives in the plant.
    /// The controller reads the pose it is given (the runner passes evaluator truth and says so in the manifest).
    /// </summary>
    public sealed class QualificationScript {
        enum Kind { Wait, GoTo, Twist, Hold }

        sealed class Step {
            public Kind K;
            public string Name;
            public double X, Y, Yaw, Until, Duration;
            public Twist Cmd;
        }

        readonly string rover;
        readonly List<Step> steps = new List<Step>();
        readonly double kp, kyaw, vmax, wmax, posTol, yawTol, timeout;
        int index = -1;
        double stepStart;
        public bool Done => index >= steps.Count;
        public string CurrentStep => index >= 0 && index < steps.Count ? steps[index].Name : (Done ? "done" : "none");

        public QualificationScript(FleetSpec spec, string rover, MecanumGeometry g, Pose2 start, FleetRng rng) {
            this.rover = rover;
            JsonNode q = spec.O["qualification"], c = q["controller"], r = q["rovers." + rover];
            kp = c["kp_pos"].Double; kyaw = c["kp_yaw"].Double;
            vmax = c["speed_fraction"].Double * g.MaxSpeedXy; wmax = c["yaw_fraction"].Double * g.MaxYawRate;
            posTol = c["pos_tolerance"].Double; yawTol = c["yaw_tolerance"].Double; timeout = c["step_timeout_s"].Double;
            double depart = r["depart_s"].Double + rng.Uniform(0, q["depart_jitter_s"].Double);
            double heading = r["start"].Doubles[2];
            steps.Add(new Step { K = Kind.Wait, Name = "wait_depart", Until = depart });
            int i = 0;
            foreach (var wp in r["route"].Items) {
                double[] p = wp.Doubles;
                steps.Add(new Step { K = Kind.GoTo, Name = $"goto_{i++}", X = p[0], Y = p[1], Yaw = heading });
            }
            double sv = q["strafe_test.speed"].Double, sd = q["strafe_test.duration_s"].Double;
            double rw = q["rotate_test.yaw_rate"].Double, rd = q["rotate_test.duration_s"].Double;
            steps.Add(new Step { K = Kind.Hold, Name = "settle_0", Duration = 1.0 });
            steps.Add(new Step { K = Kind.Twist, Name = "strafe_left", Cmd = new Twist(0, sv, 0), Duration = sd });
            steps.Add(new Step { K = Kind.Hold, Name = "settle_1", Duration = 1.0 });
            steps.Add(new Step { K = Kind.Twist, Name = "strafe_right", Cmd = new Twist(0, -sv, 0), Duration = sd });
            steps.Add(new Step { K = Kind.Hold, Name = "settle_2", Duration = 1.0 });
            steps.Add(new Step { K = Kind.Twist, Name = "rotate_ccw", Cmd = new Twist(0, 0, rw), Duration = rd });
            steps.Add(new Step { K = Kind.Hold, Name = "settle_3", Duration = 1.0 });
        }

        /// <summary>The command at time t given the controller's pose; <paramref name="events"/> receives step boundaries.</summary>
        public Twist Command(double t, Pose2 pose, List<StepEvent> events) {
            if (index < 0) Advance(t, events, false);
            while (!Done) {
                Step s = steps[index];
                double elapsed = t - stepStart;
                switch (s.K) {
                    case Kind.Wait:
                        if (t >= s.Until) { Advance(t, events, false); continue; }
                        return new Twist();
                    case Kind.Hold:
                        if (elapsed >= s.Duration) { Advance(t, events, false); continue; }
                        return new Twist();
                    case Kind.Twist:
                        if (elapsed >= s.Duration) { Advance(t, events, false); continue; }
                        return s.Cmd;
                    case Kind.GoTo: {
                        double ex = s.X - pose.X, ey = s.Y - pose.Y, eyaw = Angles.Wrap(s.Yaw - pose.Yaw);
                        double dist = Math.Sqrt(ex * ex + ey * ey);
                        bool timedOut = elapsed > timeout;
                        if ((dist < posTol && Math.Abs(eyaw) < yawTol) || timedOut) { Advance(t, events, timedOut); continue; }
                        double c = Math.Cos(pose.Yaw), sn = Math.Sin(pose.Yaw);
                        double bx = c * ex + sn * ey, by = -sn * ex + c * ey;  // error in the body frame
                        double vx = kp * bx, vy = kp * by, sp = Math.Sqrt(vx * vx + vy * vy);
                        if (sp > vmax) { vx *= vmax / sp; vy *= vmax / sp; }
                        double wz = Math.Max(-wmax, Math.Min(wmax, kyaw * eyaw));
                        return new Twist(vx, vy, wz);
                    }
                }
            }
            return new Twist();
        }

        void Advance(double t, List<StepEvent> events, bool timedOut) {
            if (index >= 0 && index < steps.Count) {
                Step s = steps[index];
                events?.Add(new StepEvent { Rover = rover, Kind = "step_end", Step = s.Name, Index = index, TimedOut = timedOut,
                    Commanded = s.Cmd, Duration = t - stepStart });
            }
            index++;
            stepStart = t;
            if (index < steps.Count) {
                Step s = steps[index];
                events?.Add(new StepEvent { Rover = rover, Kind = "step_start", Step = s.Name, Index = index, Commanded = s.Cmd, Duration = s.Duration });
            }
        }
    }

    public static class RunIds {
        /// <summary>fleet/warehouse_a/v0/&lt;variant&gt;/&lt;seed&gt;</summary>
        public static string Make(string scenarioId, string variant, long seed) => $"{scenarioId}/{variant}/{seed}";
        public static string DirectoryName(string runId) => runId.Replace('/', '_');
    }
}
