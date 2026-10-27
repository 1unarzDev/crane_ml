using System.Collections;
using System.Globalization;
using RosMessageTypes.Std;
using Sim.Physics.Core;
using UnityEngine;

namespace Sim.Controllers {
    /// <summary>
    /// Player smoke of /crane/reset_to_start (--crane-roboboat-reset-probe; sim only, added by
    /// <see cref="RoboBoatStartReset"/>). Resets through the same queued path as the service:
    /// after drifting, while driven fast and turning far from the start, twice in one tick, and
    /// three times in a row. For 10 s after each, the boat must stay finite, slow and on the
    /// course truth start (2 m and 15 deg, as the testing harness checks; the position bound grows
    /// with the current's drift), the reply must name the start, and the plant must report no
    /// out-of-envelope or non-finite event. Logs CRANE_ROBOBOAT_RESET_PROBE lines and quits with
    /// 0 on a pass, 3 on a failure.
    /// </summary>
    public sealed class RoboBoatStartResetProbe : MonoBehaviour {
        private const float Watch = 10f, PoseTolerance = 2f, YawToleranceDeg = 15f;
        private const float SpeedMargin = 0.5f, RateLimit = 0.5f;

        private RoboBoatStartReset reset;
        private ArticulationBody root;
        private bool pass = true;

        private IEnumerator Start() {
            reset = GetComponent<RoboBoatStartReset>();
            yield return new WaitForSeconds(2f);
            root = reset.Body != null ? reset.Body.GetComponent<ArticulationBody>() : null;
            if (root == null) {
                Debug.LogError("CRANE_ROBOBOAT_RESET_PROBE no articulated boat body");
                Application.Quit(3);
                yield break;
            }
            Debug.Log($"CRANE_ROBOBOAT_RESET_PROBE start current={RoboBoatCourse.CurrentNow().speed:F3}");

            yield return new WaitForSeconds(5f);
            yield return ResetAndWatch("drift", 1);

            yield return Drive(5f, 4f, 0f);
            Coroutine turning = StartCoroutine(Drive(3f, 3f, 0.8f));
            yield return new WaitForSeconds(1.5f);
            StopCoroutine(turning);  // in the same frame as the request: it arrives moving and turning
            yield return ResetAndWatch("moving_turning", 1);

            yield return Drive(2f, 2f, -0.5f);
            yield return ResetAndWatch("twice_in_one_tick", 2);

            for (int i = 1; i <= 3; i++) {
                yield return new WaitForSeconds(3f);
                yield return ResetAndWatch($"repeat_{i}", 1);
            }

            Debug.Log($"CRANE_ROBOBOAT_RESET_PROBE_DONE pass={(pass ? "true" : "false")}");
            Application.Quit(pass ? 0 : 3);
        }

        /// <summary>Holds the boat at <paramref name="surge"/> along its bow [m/s] and
        /// <paramref name="yawRate"/> [rad/s, ROS counter-clockwise] for <paramref name="seconds"/>.</summary>
        private IEnumerator Drive(float seconds, float surge, float yawRate) {
            for (float t = 0f; t < seconds; t += Time.fixedDeltaTime) {
                Vector3 bow = RoboBoatRosFrame.Rotation(root.transform.rotation) * Vector3.forward;
                root.linearVelocity = new Vector3(bow.x, 0f, bow.z).normalized * surge;
                root.angularVelocity = new Vector3(0f, -yawRate, 0f);
                yield return new WaitForFixedUpdate();
            }
        }

        private IEnumerator ResetAndWatch(string scenario, int requests) {
            var replies = new System.Threading.Tasks.Task<TriggerResponse>[requests];
            for (int i = 0; i < requests; i++) replies[i] = reset.Request();
            while (!replies[requests - 1].IsCompleted) yield return null;
            int events = PlantEvents.Count("OUT_OF_ENVELOPE") + PlantEvents.Count("NONFINITE_STATE") +
                         PlantEvents.Count("NONFINITE_WRENCH");

            bool ok = true;
            string replyText = "";
            float replyError = 0f;
            foreach (var reply in replies) {
                TriggerResponse response = reply.Result;
                replyText = response.message;
                ok &= response.success && ReplyPose(response.message, out Vector2 at, out float yaw) &&
                      (replyError = Mathf.Max(replyError, Vector2.Distance(at, reset.StartPosition))) <= PoseTolerance &&
                      YawErrorDeg(yaw) <= YawToleranceDeg;
            }

            float current = RoboBoatCourse.CurrentNow().speed;
            float maxSpeed = 0f, maxRate = 0f, maxExcess = 0f, maxError = 0f, maxYaw = 0f;
            for (float t = 0f; t < Watch; t += Time.fixedDeltaTime) {
                yield return new WaitForFixedUpdate();
                Vector3 v = root.linearVelocity, w = root.angularVelocity;
                Vector3 p = root.transform.position;
                if (!(IsFinite(v) && IsFinite(w) && IsFinite(p))) {
                    ok = false;
                    maxSpeed = float.NaN;
                    break;
                }
                Vector2 at = RoboBoatCourse.UnityToRos(p);
                float yaw = RoboBoatCourse.RosYawOfUnityDirection(
                    RoboBoatRosFrame.Rotation(root.transform.rotation) * Vector3.forward);
                float error = Vector2.Distance(at, reset.StartPosition);
                maxSpeed = Mathf.Max(maxSpeed, new Vector2(v.x, v.z).magnitude);
                maxRate = Mathf.Max(maxRate, w.magnitude);
                maxError = Mathf.Max(maxError, error);
                maxExcess = Mathf.Max(maxExcess, error - current * t);
                maxYaw = Mathf.Max(maxYaw, YawErrorDeg(yaw));
            }
            int newEvents = PlantEvents.Count("OUT_OF_ENVELOPE") + PlantEvents.Count("NONFINITE_STATE") +
                            PlantEvents.Count("NONFINITE_WRENCH") - events;
            ok &= maxSpeed <= current + SpeedMargin && maxRate <= RateLimit && maxExcess <= PoseTolerance &&
                  maxYaw <= YawToleranceDeg && newEvents == 0;
            pass &= ok;
            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "CRANE_ROBOBOAT_RESET_PROBE scenario={0} pass={1} requests={2} reply=\"{3}\" reply_err={4:F3} " +
                "max_speed={5:F3} max_rate={6:F3} max_err={7:F3} max_err_beyond_drift={8:F3} max_yaw_err_deg={9:F2} " +
                "plant_events={10} start=({11:F3},{12:F3},{13:F3})",
                scenario, ok ? "true" : "false", requests, replyText, replyError, maxSpeed, maxRate, maxError,
                maxExcess, maxYaw, newEvents, reset.StartPosition.x, reset.StartPosition.y, reset.StartYaw));
        }

        private float YawErrorDeg(float yaw) => Mathf.Abs(Mathf.DeltaAngle(yaw * Mathf.Rad2Deg, reset.StartYaw * Mathf.Rad2Deg));

        private static bool IsFinite(Vector3 v) => float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);

        /// <summary>Parses "x=.. y=.. yaw=.." (<see cref="RoboBoatStartReset.DescribeRosPose"/>).</summary>
        internal static bool ReplyPose(string message, out Vector2 at, out float yaw) {
            at = Vector2.zero;
            yaw = 0f;
            string[] parts = message.Split(' ');
            if (parts.Length != 3) return false;
            bool ok = float.TryParse(parts[0].Substring(2), NumberStyles.Float, CultureInfo.InvariantCulture, out at.x);
            ok &= float.TryParse(parts[1].Substring(2), NumberStyles.Float, CultureInfo.InvariantCulture, out at.y);
            ok &= float.TryParse(parts[2].Substring(4), NumberStyles.Float, CultureInfo.InvariantCulture, out yaw);
            return ok;
        }
    }
}
