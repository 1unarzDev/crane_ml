using System;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;
using Sim.Actuators.Motors;
using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Lockstep link to an external controller (<c>--crane-lockstep HOST:PORT</c>): the player
    /// is the plant, the controller runs elsewhere (a study's Python pipeline). After
    /// <c>--crane-lockstep-after</c> simulated seconds (default 6) the player connects and the
    /// link is line-based text, one exchange per physics tick:
    /// <code>
    /// player: hello start_x start_y start_yaw fixed_dt          (world-ROS of /crane/odom)
    /// ctrl:   scenario cur_speed cur_toward_deg wind_speed wind_toward_deg   (course frame)
    /// player: s tick t x y yaw vx vy r    (t: seconds since the episode reset; world-ROS)
    /// ctrl:   reset | p FL FR RR RL (ESC pulses, us) | end
    /// </code>
    /// <c>reset</c> puts the boat back on the course start (reset_to_start) and is answered by a
    /// fresh state; pulses drive the four thrusters for this tick; <c>end</c> (or a lost link)
    /// neutralises the thrusters and asks the benchmark runner to finish. The tick blocks while it
    /// waits, so the controller's time does not leak into the plant (headless runs only).
    /// </summary>
    [DefaultExecutionOrder(-500)]
    public sealed class CraneLockstepControl : MonoBehaviour {
        internal const string Flag = "--crane-lockstep";
        private OmniXController controller;
        private Thruster[] thrusters;   // FL, FR, RR, RL
        private string host;
        private int port;
        private float after;
        private TcpClient client;
        private StreamReader reader;
        private StreamWriter writer;
        private bool done;
        private long tick;
        private double episodeStart;
        private float startX, startY, startYaw;
        private int seed;

        internal void Initialize(OmniXController target, string endpoint, float afterSeconds, int windSeed) {
            controller = target;
            thrusters = new[] { target.frontLeft, target.frontRight, target.rearRight, target.rearLeft };
            int colon = endpoint.LastIndexOf(':');
            host = colon > 0 ? endpoint.Substring(0, colon) : "127.0.0.1";
            port = int.Parse(colon >= 0 ? endpoint.Substring(colon + 1) : endpoint, CultureInfo.InvariantCulture);
            after = afterSeconds;
            seed = windSeed;
            Debug.Log($"CRANE_LOCKSTEP_READY endpoint={host}:{port} after={after:R}");
        }

        private void FixedUpdate() {
            if (done || controller == null || Time.timeSinceLevelLoad < after) return;
            try {
                if (client == null) Connect();
                Exchange();
            }
            catch (Exception error) {
                Debug.LogError($"CRANE_LOCKSTEP_LOST tick={tick} {error.GetType().Name}: {error.Message}");
                Finish();
            }
        }

        private void Connect() {
            client = new TcpClient { NoDelay = true, ReceiveTimeout = 300000, SendTimeout = 60000 };
            IAsyncResult pending = client.BeginConnect(host, port, null, null);
            if (!pending.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(60)))
                throw new IOException($"no controller at {host}:{port} within 60 s");
            client.EndConnect(pending);
            NetworkStream stream = client.GetStream();
            reader = new StreamReader(stream, Encoding.ASCII);
            writer = new StreamWriter(stream, new UTF8Encoding(false)) { NewLine = "\n", AutoFlush = true };
            RoboBoatStartReset reset = FindAnyObjectByType<RoboBoatStartReset>();
            if (reset == null) throw new InvalidOperationException("no course start (reset_to_start) in the scene");
            startX = reset.StartPosition.x;
            startY = reset.StartPosition.y;
            startYaw = reset.StartYaw;
            Send($"hello {F(startX)} {F(startY)} {F(startYaw)} {F(Time.fixedDeltaTime)}");
            string[] scenario = Read();
            if (scenario[0] != "scenario" || scenario.Length != 5)
                throw new InvalidDataException($"expected scenario, got {string.Join(" ", scenario)}");
            float yawDeg = startYaw * Mathf.Rad2Deg;
            RoboBoatCourse.ApplyCurrent(P(scenario[1]), P(scenario[2]) + yawDeg);
            if (P(scenario[3]) > 0f) RoboBoatWind.Attach(controller, P(scenario[3]), P(scenario[4]) + yawDeg, seed);
            ResetEpisode();
            Debug.Log($"CRANE_LOCKSTEP_CONNECTED start=({startX:R},{startY:R},{startYaw:R}) " +
                      $"dt={Time.fixedDeltaTime:R}");
        }

        private void ResetEpisode() {
            RoboBoatStartReset reset = FindAnyObjectByType<RoboBoatStartReset>();
            reset.Handle();
            Neutral();
            episodeStart = Time.fixedTimeAsDouble;
        }

        private void Exchange() {
            while (true) {
                Transform body = RoboBoatStartReset.FindBoatBody();
                ArticulationBody root = body.GetComponent<ArticulationBody>();
                Vector2 p = RoboBoatCourse.UnityToRos(body.position);
                float yaw = RoboBoatCourse.RosYawOfUnityDirection(
                    RoboBoatRosFrame.Rotation(body.rotation) * Vector3.forward);
                Vector3 lin = root != null ? root.linearVelocity : Vector3.zero;
                Vector3 ang = root != null ? root.angularVelocity : Vector3.zero;
                Vector2 v = RoboBoatCourse.UnityToRos(lin);
                // ROS yaw rate is counter-clockwise; Unity +y turns clockwise.
                Send($"s {tick} {F((float)(Time.fixedTimeAsDouble - episodeStart))} {F(p.x)} {F(p.y)} {F(yaw)} " +
                     $"{F(v.x)} {F(v.y)} {F(-ang.y)}");
                string[] command = Read();
                switch (command[0]) {
                    case "p" when command.Length == 5:
                        for (int i = 0; i < 4; i++) thrusters[i].SetPulse(P(command[i + 1]));
                        tick++;
                        return;
                    case "reset":
                        ResetEpisode();
                        Debug.Log($"CRANE_LOCKSTEP_RESET tick={tick}");
                        continue;   // answer with the fresh state at once
                    case "end":
                        Finish();
                        return;
                    default:
                        throw new InvalidDataException($"unknown command {string.Join(" ", command)}");
                }
            }
        }

        private void Neutral() {
            foreach (Thruster thruster in thrusters) thruster.SetPulse(ThrusterModel.Neutral);
        }

        private void Finish() {
            done = true;
            if (thrusters != null) Neutral();
            try { client?.Close(); } catch (Exception) { }
            Debug.Log($"CRANE_LOCKSTEP_DONE ticks={tick}");
            CraneRuntimeMetrics.RequestFinish();
        }

        private void OnDestroy() {
            try { client?.Close(); } catch (Exception) { }
        }

        private void Send(string line) => writer.WriteLine(line);

        private string[] Read() {
            string line = reader.ReadLine() ?? throw new EndOfStreamException("controller closed the link");
            return line.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        }

        private static string F(float value) => value.ToString("R", CultureInfo.InvariantCulture);

        private static float P(string text) => float.Parse(text, NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    internal static class CraneLockstepControlBootstrap {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Initialize() {
            string[] args = Environment.GetCommandLineArgs();
            int flag = Array.IndexOf(args, CraneLockstepControl.Flag);
            if (flag < 0 || flag + 1 >= args.Length) return;
            string endpoint = args[flag + 1];
            float after = ReadFloat(args, "--crane-lockstep-after", 6f);
            int seed = (int)ReadFloat(args, "--crane-seed", 0f);
            SceneManager.sceneLoaded += (scene, _) => {
                OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                    FindObjectsInactive.Exclude);
                if (controller == null) {
                    Debug.LogWarning($"CRANE_LOCKSTEP_UNAVAILABLE scene={scene.name} reason=no-enabled-omni-x-controller");
                    return;
                }
                if (controller.GetComponent<CraneLockstepControl>() != null) return;
                controller.gameObject.AddComponent<CraneLockstepControl>().Initialize(controller, endpoint, after, seed);
            };
        }

        private static float ReadFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length &&
                   float.TryParse(args[index + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out float value)
                ? value : fallback;
        }
    }
}
