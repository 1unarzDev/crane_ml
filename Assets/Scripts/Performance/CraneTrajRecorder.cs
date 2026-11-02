using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Sim.Physics.Core;
using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Performance {
    /// <summary>
    /// The lightweight trajectory recorder: <c>--crane-traj PATH</c> writes the robot's pose after
    /// physics at <c>--crane-traj-hz</c> (default 20) as crane-traj/1 (<see cref="CraneTraj"/>),
    /// with the run's events (<see cref="CraneTrajEvents"/>, task outcomes). It is independent of
    /// the full-state <c>--crane-record</c> and works under any launcher (rig, cluster, local).
    ///
    /// The robot is the root body of the scene's <see cref="RobotDescription"/> (else the largest
    /// root articulation); its pose is world ENU (x = Unity z, y = -Unity x, z = Unity y, the
    /// /crane/odom frame of the course truth) and body FLU angles (yaw CCW from east). Columns
    /// per agent (<c>--crane-traj-agent</c>, default <c>robot</c>): x, y, z, yaw, pitch, roll, speed.
    /// The file is rewritten atomically every block (30 s) and at quit, so a killed player keeps
    /// what it had. The header records what the player knows (seed, course flags, build, water);
    /// a launcher stamps outcome, score, study and params afterwards (<c>theater/meta.py</c>).
    /// </summary>
    [DefaultExecutionOrder(32000)]
    public sealed class CraneTrajRecorder : MonoBehaviour {
        private static readonly string[] s_Axes = { "x", "y", "z", "yaw", "pitch", "roll", "speed" };

        private string outputPath;
        private string agent = "robot";
        private double requestedHz = 20;
        private int stepsPerSample = 1;
        private double hz;
        private long fixedSteps;
        private long samplesSinceWrite;
        private int samplesPerBlock;
        private Transform body;
        private Rigidbody rigid;
        private ArticulationBody articulation;
        private Quaternion bodyFromFlu = Quaternion.identity;
        private string robotModel = "unknown";
        private double waterT0 = double.NaN;
        private float waterMultiplier = float.NaN;
        private bool started;
        private readonly List<CraneTraj.Column> columns = new();
        private readonly List<string> events = new();
        private readonly object eventsLock = new();
        private readonly List<(string kind, Dictionary<string, object> fields)> pending = new();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string path = CraneReplayFormat.ReadArgument(Environment.GetCommandLineArgs(), "--crane-traj");
            if (string.IsNullOrWhiteSpace(path)) return;
            var host = new GameObject("CRANE Trajectory Recorder");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneTrajRecorder>();
        }

        private void Awake() {
            string[] args = Environment.GetCommandLineArgs();
            outputPath = Path.GetFullPath(CraneReplayFormat.ReadArgument(args, "--crane-traj"));
            if (double.TryParse(CraneReplayFormat.ReadArgument(args, "--crane-traj-hz"), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out double value) && value > 0) requestedHz = value;
            agent = CraneReplayFormat.ReadArgument(args, "--crane-traj-agent") ?? agent;
            foreach (string axis in s_Axes) {
                string name = $"{agent}.{axis}";
                columns.Add(new CraneTraj.Column { Name = name, Scale = CraneTraj.ScaleOf(name) });
            }
            CraneTrajEvents.Subscribe(OnEvent);
            CraneTaskOutcome.Subscribe(OnTaskOutcome);
            StartCoroutine(SampleAfterPhysics());
        }

        private void OnDestroy() {
            CraneTrajEvents.Unsubscribe(OnEvent);
            CraneTaskOutcome.Unsubscribe(OnTaskOutcome);
            WriteNow();
        }

        private void OnApplicationQuit() => WriteNow();

        private IEnumerator SampleAfterPhysics() {
            while (true) {
                yield return new WaitForFixedUpdate();
                if (!started && !TryStart()) continue;
                if (body == null && !FindRobot()) continue;
                if (fixedSteps++ % stepsPerSample != 0) continue;
                Sample();
                if (++samplesSinceWrite >= samplesPerBlock) WriteNow();
            }
        }

        private bool TryStart() {
            if (!FindRobot()) return false;
            stepsPerSample = Math.Max(1, (int)Math.Round(1.0 / (requestedHz * Time.fixedDeltaTime)));
            hz = 1.0 / (stepsPerSample * Time.fixedDeltaTime);
            samplesPerBlock = Math.Max(1, (int)Math.Round(CraneTraj.BlockSeconds * hz));
            WaterSurface water = FindAnyObjectByType<WaterSurface>(FindObjectsInactive.Exclude);
            if (water != null) {
                waterT0 = water.simulationTime;
                waterMultiplier = water.timeMultiplier;
            }
            started = true;
            Debug.Log($"CRANE_TRAJ_START path={outputPath} hz={hz:R} robot={robotModel} body={body.name}");
            return true;
        }

        /// <summary>The robot: a described one's root body, else the largest root articulation.</summary>
        private bool FindRobot() {
            Transform root = null;
            RobotDescription description = FindObjectsByType<RobotDescription>(FindObjectsInactive.Exclude)
                .FirstOrDefault();
            if (description != null) {
                ArticulationBody a = description.GetComponentsInChildren<ArticulationBody>().FirstOrDefault(x => x.isRoot);
                Rigidbody r = description.GetComponentInChildren<Rigidbody>();
                root = a != null ? a.transform : r != null ? r.transform : description.transform;
                robotModel = string.IsNullOrEmpty(description.urdf) ? description.name
                    : Path.GetFileNameWithoutExtension(description.urdf);
            }
            else {
                ArticulationBody a = FindObjectsByType<ArticulationBody>(FindObjectsInactive.Exclude)
                    .Where(x => x.isRoot).OrderByDescending(x => x.GetComponentsInChildren<ArticulationBody>().Length)
                    .FirstOrDefault();
                if (a != null) {
                    root = a.transform;
                    robotModel = a.transform.root.name;
                }
            }
            if (root == null) return false;
            body = root;
            articulation = root.GetComponent<ArticulationBody>();
            rigid = articulation == null ? root.GetComponent<Rigidbody>() : null;
            bodyFromFlu = RobotDescription.BodyFromFlu(root);
            return true;
        }

        private static Vector3 Enu(Vector3 u) => new(u.z, -u.x, u.y);

        private void Sample() {
            Quaternion rotation = body.rotation * bodyFromFlu;
            Vector3 p = Enu(body.position);
            Vector3 f = Enu(rotation * Vector3.forward), l = Enu(rotation * Vector3.left), u = Enu(rotation * Vector3.up);
            double yaw = Math.Atan2(f.y, f.x);
            double pitch = -Math.Asin(Mathf.Clamp(f.z, -1f, 1f));
            double roll = Math.Atan2(l.z, u.z);
            Vector3 v = articulation != null ? articulation.linearVelocity : rigid != null ? rigid.linearVelocity : Vector3.zero;
            double speed = Math.Sqrt(v.x * v.x + v.z * v.z);
            double[] values = { p.x, p.y, p.z, yaw, pitch, roll, speed };
            for (int i = 0; i < values.Length; i++) {
                double x = values[i];
                if (double.IsNaN(x) || double.IsInfinity(x)) {
                    List<double> col = columns[i].Values;
                    x = col.Count > 0 ? col[col.Count - 1] : 0;
                    AddEvent("note", new Dictionary<string, object> { ["text"] = $"non-finite {columns[i].Name}" });
                }
                columns[i].Values.Add(x);
            }
            FlushPending();
        }

        private double Now => columns[0].Values.Count > 0 ? (columns[0].Values.Count - 1) / hz : 0;

        private void OnEvent(CraneTrajEvent e) {
            lock (eventsLock) pending.Add((e.Kind, new Dictionary<string, object>(e.Fields)));
        }

        private void OnTaskOutcome(CraneTaskOutcomeEvent o) {
            var fields = new Dictionary<string, object> {
                ["source"] = o.Source, ["reward"] = o.Reward, ["cumulative"] = o.CumulativeReward,
                ["terminated"] = o.Terminated, ["truncated"] = o.Truncated, ["reason"] = o.Reason
            };
            lock (eventsLock) pending.Add(("task", fields));
        }

        private void FlushPending() {
            lock (eventsLock) {
                foreach (var (kind, fields) in pending) AddEvent(kind, fields);
                pending.Clear();
            }
        }

        private void AddEvent(string kind, Dictionary<string, object> fields) {
            var b = new StringBuilder("{");
            b.Append("\"kind\":").Append(CraneTraj.Str(kind)).Append(",\"t\":").Append(CraneTraj.Num(Math.Round(Now, 4)));
            foreach (var kv in fields.OrderBy(kv => kv.Key, StringComparer.Ordinal))
                b.Append(',').Append(CraneTraj.Str(kv.Key)).Append(':').Append(Value(kv.Value));
            events.Add(b.Append('}').ToString());
        }

        private static string Value(object v) => v switch {
            null => "null",
            bool x => x ? "true" : "false",
            string s => CraneTraj.Str(s),
            double d => CraneTraj.Num(d),
            float f => CraneTraj.Num(f),
            int i => i.ToString(CultureInfo.InvariantCulture),
            long l => l.ToString(CultureInfo.InvariantCulture),
            IFormattable n => CraneTraj.Num(Convert.ToDouble(n, CultureInfo.InvariantCulture)),
            _ => CraneTraj.Str(v.ToString())
        };

        private string HeaderJson() {
            string[] args = Environment.GetCommandLineArgs();
            string Arg(string key) => CraneReplayFormat.ReadArgument(args, key);
            string manifest = CraneReplayFormat.BuildManifestJson();
            var b = new StringBuilder("{");
            b.Append("\"build\":{\"recorder\":\"unity\",\"unity_version\":").Append(CraneTraj.Str(Application.unityVersion))
                .Append(",\"build_guid\":").Append(CraneTraj.Str(Application.buildGUID ?? ""))
                .Append(",\"build_hash\":").Append(CraneTraj.Str(CraneReplayFormat.HashText(manifest).ToLowerInvariant()))
                .Append(",\"source_commit\":").Append(CraneTraj.Str(Arg("--crane-source-commit"))).Append('}');
            b.Append(",\"course\":{\"family\":").Append(CraneTraj.Str(Arg("--crane-course-family")))
                .Append(",\"seed\":").Append(CraneTraj.Str(Arg("--crane-course-seed") ?? Arg("--crane-seed")))
                .Append(",\"mode\":").Append(CraneTraj.Str(Arg("--crane-course-mode"))).Append('}');
            b.Append(",\"frame\":{\"type\":\"enu\",\"origin\":\"world\"}");
            b.Append(",\"hz\":").Append(CraneTraj.Num(hz)).Append(",\"t0_s\":0");
            b.Append(",\"outcome\":\"unknown\",\"pose_dims\":\"6dof\"");
            b.Append(",\"robot\":{\"model\":").Append(CraneTraj.Str(robotModel)).Append(",\"agents\":[")
                .Append(CraneTraj.Str(agent)).Append("]}");
            b.Append(",\"schema\":").Append(CraneTraj.Str(CraneTraj.Schema));
            b.Append(",\"scene\":").Append(CraneTraj.Str(Arg("--crane-scene")));
            b.Append(",\"seed\":").Append(CraneTraj.Str(Arg("--crane-seed")));
            b.Append(",\"ticks\":").Append(columns[0].Values.Count.ToString(CultureInfo.InvariantCulture));
            b.Append(",\"water\":{\"mode\":").Append(CraneTraj.Str(Arg("--crane-water-mode") ?? "free"))
                .Append(",\"sim_t0_s\":").Append(CraneTraj.Num(waterT0))
                .Append(",\"time_multiplier\":").Append(CraneTraj.Num(waterMultiplier)).Append('}');
            return b.Append('}').ToString();
        }

        private void WriteNow() {
            if (!started || columns[0].Values.Count == 0) return;
            samplesSinceWrite = 0;
            try {
                FlushPending();
                byte[] bytes = CraneTraj.Encode(HeaderJson(), hz, columns, events);
                string directory = Path.GetDirectoryName(outputPath);
                if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
                string temporary = outputPath + ".tmp";
                File.WriteAllBytes(temporary, bytes);
                if (File.Exists(outputPath)) File.Delete(outputPath);
                File.Move(temporary, outputPath);
            }
            catch (Exception error) {
                Debug.LogError($"CRANE_TRAJ_WRITE_FAILED path={outputPath} {error.Message}");
            }
        }
    }
}
