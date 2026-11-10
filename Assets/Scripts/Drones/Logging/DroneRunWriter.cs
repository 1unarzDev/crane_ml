using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Crane.Drones.Logging {
    /// <summary>
    /// Writes one run directory (docs: Docs/DroneVisuals/LOG_FORMAT.md). Every stream is keyed by the
    /// integer-nanosecond sim clock; the writer rejects non-monotonic timestamps. Files are flushed at
    /// Close; manifest.json is written at start and rewritten at close with the "end" record.
    /// </summary>
    public sealed class DroneRunWriter : IDisposable {
        public readonly string Dir;
        public readonly DroneRunManifest Manifest;
        public readonly DroneSimClock Clock;
        private readonly StreamWriter state, imu, events;
        private readonly Dictionary<string, StreamWriter> cameraIndex = new();
        private readonly Dictionary<string, StreamWriter> lidarIndex = new();
        private readonly Dictionary<string, long> lastTime = new();
        private readonly Dictionary<string, long> counts = new();
        private bool closed;

        public DroneRunWriter(string dir, DroneRunManifest manifest) {
            Dir = Path.GetFullPath(dir);
            Manifest = manifest;
            Clock = new DroneSimClock(manifest.FixedDtNs, manifest.Substeps);
            if (File.Exists(Path.Combine(Dir, "manifest.json")))
                throw new IOException("run directory already holds a run: " + Dir);
            Directory.CreateDirectory(Dir);
            WriteManifest(null);
            state = Open(DroneLogFormat.StateFile);
            var h = new StringBuilder("tick,sub,t_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az");
            for (int i = 0; i < manifest.RotorCount; i++) h.Append(",rpm").Append(i);
            for (int i = 0; i < manifest.RotorCount; i++) h.Append(",cmd").Append(i);
            h.Append(",batt_v,batt_a,batt_soc");
            state.WriteLine(h.ToString());
            imu = Open(DroneLogFormat.ImuFile);
            imu.WriteLine("t_ns,gx,gy,gz,ax,ay,az,noise_flags");
            events = Open(DroneLogFormat.EventsFile);
        }

        private StreamWriter Open(string rel) {
            string p = Path.Combine(Dir, rel);
            Directory.CreateDirectory(Path.GetDirectoryName(p));
            return new StreamWriter(new FileStream(p, FileMode.Create, FileAccess.Write), new UTF8Encoding(false)) { NewLine = "\n" };
        }

        private static string N(double v) => double.IsNaN(v) ? "nan" : v.ToString("R", CultureInfo.InvariantCulture);

        private void CheckMono(string stream, long t, bool strict) {
            if (lastTime.TryGetValue(stream, out long prev) && (strict ? t <= prev : t < prev))
                throw new InvalidDataException($"non-monotonic timestamp in {stream}: {t} after {prev}");
            lastTime[stream] = t;
            counts[stream] = counts.TryGetValue(stream, out long c) ? c + 1 : 1;
        }

        public void WriteState(long tick, int sub, DroneStateSample s) {
            long t = Clock.TimeNs(tick, sub);
            CheckMono("state", t, true);
            var sb = new StringBuilder(256);
            sb.Append(tick).Append(',').Append(sub).Append(',').Append(t);
            foreach (double v in s.Pos) sb.Append(',').Append(N(v));
            foreach (double v in s.Quat) sb.Append(',').Append(N(v));
            foreach (double v in s.Vel) sb.Append(',').Append(N(v));
            foreach (double v in s.AngRateBody) sb.Append(',').Append(N(v));
            foreach (double v in s.AccBody) sb.Append(',').Append(N(v));
            foreach (double v in s.RotorRpm) sb.Append(',').Append(N(v));
            foreach (double v in s.MotorCmd) sb.Append(',').Append(N(v));
            sb.Append(',').Append(N(s.BattVolt)).Append(',').Append(N(s.BattAmp)).Append(',').Append(N(s.BattSoc));
            state.WriteLine(sb.ToString());
        }

        public void WriteImu(long tNs, DroneImuSample s) {
            CheckMono("imu", tNs, true);
            imu.WriteLine($"{tNs},{N(s.Gx)},{N(s.Gy)},{N(s.Gz)},{N(s.Ax)},{N(s.Ay)},{N(s.Az)},{s.NoiseFlags}");
        }

        public void WriteCameraFrame(DroneCameraFrame f) {
            if (f.CompletionNs < f.AcquisitionNs) throw new InvalidDataException("completion before acquisition");
            CheckMono("camera/" + f.Camera, f.AcquisitionNs, true);
            if (!cameraIndex.TryGetValue(f.Camera, out var idx)) {
                idx = Open("camera/" + f.Camera + ".csv");
                idx.WriteLine("seq,acq_ns,complete_ns,width,height,kind,file");
                cameraIndex[f.Camera] = idx;
            }
            long seq = counts["camera/" + f.Camera] - 1;
            string rel = $"camera/{f.Camera}/{seq:D6}.png";
            byte[] png = f.Kind == DroneCameraKind.Depth
                ? DronePng.EncodeGray16(f.Depth16, f.Width, f.Height)
                : DronePng.EncodeRgb8(f.Rgb8, f.Width, f.Height);
            string path = Path.Combine(Dir, rel);
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            File.WriteAllBytes(path, png);
            idx.WriteLine($"{seq},{f.AcquisitionNs},{f.CompletionNs},{f.Width},{f.Height},{f.Kind},{rel}");
        }

        /// <summary>Scan file: little-endian records of float x,y,z,intensity + int32 t_offset_ns (20 bytes per point).</summary>
        public void WriteLidarScan(DroneLidarScan s) {
            CheckMono("lidar/" + s.Sensor, s.AcquisitionNs, true);
            int prev = 0;
            for (int i = 0; i < s.Count; i++) {
                if (s.TimeOffsetNs[i] < prev) throw new InvalidDataException("lidar per-point time not monotonic");
                prev = s.TimeOffsetNs[i];
            }
            if (!lidarIndex.TryGetValue(s.Sensor, out var idx)) {
                idx = Open("lidar/" + s.Sensor + ".csv");
                idx.WriteLine("seq,acq_ns,points,t_first_ns,t_last_ns,file");
                lidarIndex[s.Sensor] = idx;
            }
            long seq = counts["lidar/" + s.Sensor] - 1;
            string rel = $"lidar/{s.Sensor}/{seq:D6}.bin";
            string path = Path.Combine(Dir, rel);
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var bw = new BinaryWriter(File.Create(path))) {
                for (int i = 0; i < s.Count; i++) {
                    bw.Write(s.X[i]); bw.Write(s.Y[i]); bw.Write(s.Z[i]); bw.Write(s.Intensity[i]); bw.Write(s.TimeOffsetNs[i]);
                }
            }
            long first = s.Count > 0 ? s.AcquisitionNs + s.TimeOffsetNs[0] : s.AcquisitionNs;
            long last = s.Count > 0 ? s.AcquisitionNs + s.TimeOffsetNs[s.Count - 1] : s.AcquisitionNs;
            idx.WriteLine($"{seq},{s.AcquisitionNs},{s.Count},{first},{last},{rel}");
        }

        /// <summary>kind: collision | near_miss | detection | marker. Fields are written verbatim after t_ns/kind.</summary>
        public void WriteEvent(long tNs, string kind, Dictionary<string, object> fields = null) {
            CheckMono("events", tNs, false);
            var d = new Dictionary<string, object> { ["t_ns"] = tNs, ["kind"] = kind };
            if (fields != null) foreach (var kv in fields) d[kv.Key] = kv.Value;
            events.WriteLine(DroneJson.Write(d));
        }

        public void WriteObstacles(IEnumerable<Dictionary<string, object>> obstacles) =>
            File.WriteAllText(Path.Combine(Dir, DroneLogFormat.ObstaclesFile),
                DroneJson.Write(new Dictionary<string, object> { ["frame"] = "world_enu", ["obstacles"] = obstacles }, true));

        private void WriteManifest(Dictionary<string, object> end) {
            var m = Manifest;
            var d = new Dictionary<string, object> {
                ["schema"] = DroneLogFormat.Schema,
                ["run_id"] = m.RunId,
                ["crane_git_sha"] = m.CraneGitSha,
                ["scene"] = m.Scene,
                ["scene_seed"] = m.SceneSeed,
                ["split"] = m.Split,
                ["forest_manifest_sha256"] = m.ForestManifestSha256,
                ["physics_profile"] = m.PhysicsProfile,
                ["physics_params_sha256"] = m.PhysicsParamsSha256,
                ["controller"] = m.Controller,
                ["trajectory_spec"] = m.TrajectorySpec,
                ["disturbance"] = new Dictionary<string, object> { ["wind"] = m.Disturbance_Wind, ["lighting"] = m.Disturbance_Lighting },
                ["unity_version"] = m.UnityVersion,
                ["wall_clock_start_utc"] = m.WallClockStartUtc,
                ["clock"] = new Dictionary<string, object> {
                    ["unit"] = "ns", ["fixed_dt_ns"] = m.FixedDtNs, ["substeps"] = m.Substeps,
                    ["substep_dt_ns"] = m.FixedDtNs / m.Substeps, ["substep_hz"] = 1e9 * m.Substeps / m.FixedDtNs,
                    ["state_every_substeps"] = m.StateEverySubsteps,
                    ["state_hz"] = 1e9 * m.Substeps / (m.FixedDtNs * m.StateEverySubsteps),
                },
                ["frames"] = new Dictionary<string, object> { ["world"] = "ENU", ["body"] = "FLU", ["quat"] = "wxyz world_from_body" },
                ["rotor_count"] = m.RotorCount,
                ["imu"] = new Dictionary<string, object> { ["rate_hz"] = m.ImuRateHz, ["noise"] = m.ImuNoise },
                ["cameras"] = m.Cameras,
                ["lidars"] = m.Lidars,
                ["extra"] = m.Extra,
            };
            if (end != null) d["end"] = end;
            string tmp = Path.Combine(Dir, "manifest.json.tmp");
            File.WriteAllText(tmp, DroneJson.Write(d, true));
            string dst = Path.Combine(Dir, "manifest.json");
            if (File.Exists(dst)) File.Delete(dst);
            File.Move(tmp, dst);
        }

        public void Close(long endTick) {
            if (closed) return;
            closed = true;
            state.Dispose(); imu.Dispose(); events.Dispose();
            foreach (var w in cameraIndex.Values) w.Dispose();
            foreach (var w in lidarIndex.Values) w.Dispose();
            var c = new Dictionary<string, object>();
            foreach (var kv in counts) c[kv.Key] = kv.Value;
            WriteManifest(new Dictionary<string, object> { ["end_tick"] = endTick, ["end_t_ns"] = Clock.TimeNs(endTick), ["counts"] = c });
        }

        public void Dispose() => Close(0);
    }
}
