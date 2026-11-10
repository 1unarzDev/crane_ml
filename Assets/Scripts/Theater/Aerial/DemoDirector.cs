using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater.Aerial {
    public struct CamKey { public Vector3 pos; public Quaternion rot; public float fov, focus; public Vector3 look; }

    /// <summary>One output frame, fully determined by its index.</summary>
    public sealed class DirectedFrame {
        public int frame, shot;
        public float shotU;                 // 0..1 through the shot
        public double flightT;
        public TrajSample drone;
        public CamKey cam;
        public float aperture;              // 0 = no depth of field
        public float fade;                  // 0 clear .. 1 black
        public string inset = "";
        public bool detection;
        public ShotSpec spec;
        public TextSpec title, lowerThird;
        public float titleAlpha, lowerAlpha;
        public string clock = "";
    }

    /// <summary>Evaluates a demo spec against a flight: smooth camera paths (centripetal Catmull-Rom, critically damped
    /// follow), look-at damping, orbit and chase rigs, transitions and text timing. All shots are baked once, so a
    /// frame is a pure function of its index.</summary>
    public sealed class DemoDirector {
        public readonly DemoSpec Spec;
        public readonly ITrajectory Flight;
        public readonly Vector3 Person;
        public const float PersonHeight = 1.75f, PersonWidth = 0.55f;
        readonly List<CamKey[]> baked = new List<CamKey[]>();
        readonly List<double> shotStart = new List<double>();
        readonly List<int> firstFrame = new List<int>();
        readonly List<float> shotAperture = new List<float>();
        public int FrameCount { get; }
        public string Notes { get; } = "";

        public DemoDirector(DemoSpec spec, ITrajectory flight, Vector3 person, ForestMarkers markers) {
            Spec = spec; Flight = flight; Person = person; Markers = markers;
            double prevEnd = 0; int frame = 0;
            foreach (var s in spec.shots) {
                double t0 = prevEnd;
                if (!string.IsNullOrEmpty(s.t0Mark)) {
                    if (!flight.Marks.TryGetValue(s.t0Mark, out var m)) throw new ArgumentException($"shot {s.id}: unknown flight mark '{s.t0Mark}'");
                    t0 = m + s.t0Offset;
                } else t0 = prevEnd + s.t0Offset;
                t0 = Math.Max(0, Math.Min(t0, flight.Duration));
                int n = Mathf.RoundToInt(s.duration * spec.fps);
                shotStart.Add(t0); firstFrame.Add(frame);
                baked.Add(BakeShot(s, t0, n));
                shotAperture.Add(s.aperture);
                frame += n;
                prevEnd = t0 + s.duration * s.timeScale;
                if (prevEnd > flight.Duration + 1e-3) Notes += $"shot {s.id} runs {prevEnd - flight.Duration:0.0}s past the flight (holds the last pose). ";
            }
            FrameCount = frame;
        }

        // ------------------------------------------------------------------ anchoring

        static float YawOf(Quaternion q) { var f = q * Vector3.forward; f.y = 0; return f.sqrMagnitude < 1e-6f ? 0 : Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg; }

        Vector3 ToWorld(Vector3 local, PathSpace space, TrajSample anchor, ForestMarkers m) {
            switch (space) {
                case PathSpace.World: return local;
                case PathSpace.Spawn: return m == null ? local : m.spawnPos + m.spawnRot * local;
                default: return anchor.pos + Quaternion.Euler(0, YawOf(anchor.rot), 0) * local;
            }
        }

        public ForestMarkers Markers { get; }

        // ------------------------------------------------------------------ smoothing

        /// <summary>Critically damped follower (semi-implicit, 240 Hz substeps so the result does not depend on the frame rate).</summary>
        struct Damper {
            public Vector3 x, v;
            public void Reset(Vector3 p) { x = p; v = Vector3.zero; }
            public Vector3 Step(Vector3 target, float tau, float dt) {
                if (tau < 1e-3f) { x = target; v = Vector3.zero; return x; }
                float w = 2f / tau;
                int sub = Mathf.Max(1, Mathf.CeilToInt(dt * 240f)); float h = dt / sub;
                for (int i = 0; i < sub; i++) { Vector3 a = w * w * (target - x) - 2f * w * v; v += a * h; x += v * h; }
                return x;
            }
        }

        static float Ease(float u) { float s = u * u * (3 - 2 * u); return Mathf.Lerp(u, s, 0.65f); }

        Vector3 Subject(string look, Vector3 dronePos) {
            Vector3 chest = Person + Vector3.up * (PersonHeight * 0.6f);
            switch (look) { case "person": return chest; case "mid": return Vector3.Lerp(dronePos, chest, 0.4f); default: return dronePos; }
        }

        CamKey[] BakeShot(ShotSpec s, double t0, int n) {
            var keys = new CamKey[n + 1];
            float dt = 1f / Spec.fps;
            var anchor = Flight.Sample(t0);
            SplinePath spline = null;
            if (s.path.Count >= 2) {
                var pts = new List<Vector3>();
                foreach (var p in s.path) pts.Add(ToWorld(p, s.space, anchor, Markers));
                spline = new SplinePath(pts);
            }
            var camD = new Damper(); var lookD = new Damper();
            float orbitBase = 0;
            {
                Vector3 d = Person - anchor.pos; d.y = 0;
                orbitBase = (d.sqrMagnitude > 1e-4f ? Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg : 0f) + 180f;
            }
            for (int k = 0; k <= n; k++) {
                float u = n == 0 ? 0 : k / (float)n;
                var drone = Flight.Sample(t0 + k * dt * s.timeScale);
                Vector3 desired;
                if (spline != null) desired = spline.AtNorm(Ease(u));
                else if (s.type == "orbit") {
                    float a = (orbitBase + s.startDeg + s.sweepDeg * Ease(u)) * Mathf.Deg2Rad;
                    desired = drone.pos + new Vector3(Mathf.Sin(a), 0, Mathf.Cos(a)) * s.radius + Vector3.up * s.height;
                } else {   // chase rig in the drone's yaw frame
                    desired = drone.pos + Quaternion.Euler(0, YawOf(drone.rot), 0) * new Vector3(s.side, s.up, -s.back);
                }
                Vector3 want = Subject(s.look, drone.pos);
                if (k == 0) { camD.Reset(desired); lookD.Reset(want); }
                Vector3 cam = camD.Step(desired, s.damping, dt);
                Vector3 look = lookD.Step(want, s.damping * 0.6f, dt);
                if (s.avoidTrees && k > 0) {
                    Vector3 toCam = cam - drone.pos;
                    if (toCam.sqrMagnitude > 0.25f && Physics.Raycast(drone.pos, toCam.normalized, out var hit, toCam.magnitude, ~0, QueryTriggerInteraction.Ignore))
                        cam = hit.point - toCam.normalized * 0.4f;
                }
                Vector3 fwd = look - cam; if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
                keys[k] = new CamKey {
                    pos = cam, rot = Quaternion.LookRotation(fwd, Vector3.up), look = look,
                    fov = Mathf.Lerp(s.fov, s.fovEnd > 0 ? s.fovEnd : s.fov, Ease(u)),
                    focus = Mathf.Max(0.3f, Vector3.Distance(cam, Subject(s.look == "person" ? "person" : "drone", drone.pos)) + s.focusBias)
                };
            }
            return keys;
        }

        // ------------------------------------------------------------------ per-frame

        public int ShotAt(int frame) {
            int i = firstFrame.Count - 1;
            while (i > 0 && firstFrame[i] > frame) i--;
            return i;
        }

        public int FirstFrame(int shot) => firstFrame[shot];
        public int ShotFrames(int shot) => Mathf.RoundToInt(Spec.shots[shot].duration * Spec.fps);

        static float Pulse(float t, float at, float dur, float ramp) =>
            Mathf.Clamp01((t - at) / ramp) * Mathf.Clamp01((at + dur - t) / ramp);

        public DirectedFrame Evaluate(int frame) {
            frame = Mathf.Clamp(frame, 0, FrameCount - 1);
            int si = ShotAt(frame); var s = Spec.shots[si];
            int local = frame - firstFrame[si], n = ShotFrames(si);
            float lt = local / Spec.fps;
            var keys = baked[si];
            var f = new DirectedFrame {
                frame = frame, shot = si, spec = s, shotU = n > 0 ? local / (float)n : 0, flightT = shotStart[si] + lt * s.timeScale,
                cam = keys[Mathf.Min(local, keys.Length - 1)], aperture = s.aperture, inset = s.inset, detection = s.detection
            };
            f.drone = Flight.Sample(f.flightT);
            // transitions: dip fades the previous shot's last half and this shot's first half; fade ramps in from black
            float fade = 0;
            var tr = s.transition;
            if (tr.type != "cut" && tr.duration > 0) {
                float half = tr.type == "dip" ? tr.duration * 0.5f : tr.duration;
                if (lt < half) fade = Mathf.Max(fade, 1f - lt / half);
            }
            if (si + 1 < Spec.shots.Count) {
                var nt = Spec.shots[si + 1].transition;
                if (nt.type == "dip" && nt.duration > 0) {
                    float half = nt.duration * 0.5f, rem = s.duration - lt;
                    if (rem < half) fade = Mathf.Max(fade, 1f - rem / half);
                }
            } else { float rem = s.duration - lt; if (rem < 1.2f) fade = Mathf.Max(fade, 1f - rem / 1.2f); }
            f.fade = fade;
            f.title = s.title; f.lowerThird = s.lowerThird;
            if (s.title != null) f.titleAlpha = Pulse(lt, s.title.at, s.title.duration, 0.5f);
            if (s.lowerThird != null) f.lowerAlpha = Pulse(lt, s.lowerThird.at, s.lowerThird.duration, 0.5f);
            f.clock = $"{(int)(frame / Spec.fps) / 60:00}:{(int)(frame / Spec.fps) % 60:00}:{frame % Mathf.RoundToInt(Spec.fps):00}";
            return f;
        }

        /// <summary>The person's 2D box in a camera's target texture pixels (origin top-left); false when behind the camera or off-frame.</summary>
        public bool PersonBox(Camera cam, out Rect box, out bool occluded) {
            box = default; occluded = false;
            Vector3 c = Person + Vector3.up * PersonHeight * 0.5f;
            float hx = PersonWidth * 0.5f, hy = PersonHeight * 0.5f;
            float minX = float.MaxValue, minY = float.MaxValue, maxX = float.MinValue, maxY = float.MinValue;
            for (int i = 0; i < 8; i++) {
                Vector3 p = c + new Vector3((i & 1) == 0 ? -hx : hx, (i & 2) == 0 ? -hy : hy, (i & 4) == 0 ? -hx : hx);
                Vector3 sp = cam.WorldToScreenPoint(p);
                if (sp.z <= 0.1f) return false;
                minX = Mathf.Min(minX, sp.x); maxX = Mathf.Max(maxX, sp.x); minY = Mathf.Min(minY, sp.y); maxY = Mathf.Max(maxY, sp.y);
            }
            float W = cam.pixelWidth, H = cam.pixelHeight;
            if (maxX < 0 || minX > W || maxY < 0 || minY > H) return false;
            box = Rect.MinMaxRect(Mathf.Max(0, minX), H - Mathf.Min(H, maxY), Mathf.Min(W, maxX), H - Mathf.Max(0, minY));
            occluded = Physics.Linecast(cam.transform.position, c, out _, ~0, QueryTriggerInteraction.Ignore);
            return box.width > 3 && box.height > 3;
        }

        /// <summary>Flight-time stills: for each shot its still fractions, as (shot, frame) pairs.</summary>
        public IEnumerable<(int shot, int frame)> StillFrames() {
            for (int i = 0; i < Spec.shots.Count; i++)
                foreach (var fr in Spec.shots[i].stills)
                    yield return (i, firstFrame[i] + Mathf.Clamp(Mathf.RoundToInt(fr * (ShotFrames(i) - 1)), 0, ShotFrames(i) - 1));
        }
    }

    /// <summary>Builds the flight from a spec: scripted over/under-canopy sortie through the scene's corridor markers
    /// to the person, a recorded CSV, optionally routed through the physics follower.</summary>
    public static class FlightBuilder {
        public static ITrajectory Build(DemoSpec spec, ForestMarkers m, Vector3 person, int seed, string baseDir, out string note) {
            var f = spec.flight; note = "";
            ITrajectory reference;
            if (f.kind == "trajectory") {
                string path = System.IO.Path.IsPathRooted(f.file) ? f.file : System.IO.Path.Combine(baseDir, f.file);
                reference = TrajectoryTable.FromCsv(System.IO.File.ReadAllText(path), f.file);
            } else reference = Scripted(spec, m, person, out note);
            if (f.throughPlant || f.kind == "plant") {
                var plant = AerialPlants.Factory();
                reference = TrajectoryTable.BakeThroughPlant(reference, plant, seed);
                note += " plant=" + plant.Name;
            }
            return reference;
        }

        static float Ground(Vector3 p, float fallback) =>
            Physics.Raycast(new Vector3(p.x, 400f, p.z), Vector3.down, out var h, 800f, ~0, QueryTriggerInteraction.Ignore) ? h.point.y : fallback;

        static ITrajectory Scripted(DemoSpec spec, ForestMarkers m, Vector3 person, out string note) {
            var f = spec.flight;
            var wps = new List<TrajectoryBaker.Waypoint>();
            note = "";
            if (f.points.Count >= 2) {
                foreach (var p in f.points) {
                    Vector3 w = f.space == PathSpace.World ? p.pos : m.spawnPos + m.spawnRot * p.pos;
                    wps.Add(new TrajectoryBaker.Waypoint { pos = w, t = p.t, mark = p.mark });
                }
                note = "flight: spec waypoints";
                return TrajectoryBaker.Bake(wps, person + Vector3.up, 1.0 / 120, "scripted (spec waypoints)");
            }
            var corridor = new List<Vector3>(m.corridor);
            if (corridor.Count == 0) {   // straight corridor from the spawn to 6 m before the person
                Vector3 a = m.spawnPos, b = person; Vector3 d = b - a; d.y = 0; d = d.normalized;
                for (int i = 0; i < 4; i++) corridor.Add(Vector3.Lerp(a, b - d * 6f, i / 3f));
                note = "flight: stub corridor (no CorridorWaypoint markers in scene)";
            } else note = $"flight: {corridor.Count} scene corridor waypoints";
            float g0 = Ground(corridor[0], m.spawnPos.y);
            Vector3 dir = corridor.Count > 1 ? corridor[1] - corridor[0] : person - corridor[0]; dir.y = 0; dir = dir.normalized;
            Vector3 c0 = new Vector3(corridor[0].x, g0 + f.cruiseHeight, corridor[0].z);
            float topY = g0 + f.topAltitude;
            wps.Add(new TrajectoryBaker.Waypoint { pos = new Vector3(c0.x, topY, c0.z) - dir * 48f, t = 0, mark = "top" });
            wps.Add(new TrajectoryBaker.Waypoint { pos = new Vector3(c0.x, topY - 1f, c0.z) - dir * 18f, t = 8 });
            wps.Add(new TrajectoryBaker.Waypoint { pos = new Vector3(c0.x, topY - 2f, c0.z) - dir * 9f, t = 12, mark = "descent" });
            wps.Add(new TrajectoryBaker.Waypoint { pos = Vector3.Lerp(new Vector3(c0.x, topY, c0.z) - dir * 4f, c0, 0.55f), t = 17 });
            wps.Add(new TrajectoryBaker.Waypoint { pos = c0, t = 22, mark = "corridor" });
            // along the corridor at the cruise speed (never slower than 4 s per leg end to end budget of 24 s)
            float len = 0; for (int i = 1; i < corridor.Count; i++) len += Vector3.Distance(corridor[i - 1], corridor[i]);
            float dur = Mathf.Max(24f, len / 9f), acc = 0;
            for (int i = 1; i < corridor.Count; i++) {
                acc += Vector3.Distance(corridor[i - 1], corridor[i]);
                var c = corridor[i]; c.y = Ground(c, m.spawnPos.y) + f.cruiseHeight;
                wps.Add(new TrajectoryBaker.Waypoint { pos = c, t = 22 + dur * (len > 0 ? acc / len : 1f) });
            }
            // hover near the person: stand-off on the approach side, at hover height, facing the person
            Vector3 last = wps[wps.Count - 1].pos; Vector3 toP = person - last; toP.y = 0; Vector3 pd = toP.sqrMagnitude > 1e-4f ? toP.normalized : dir;
            Vector3 hover = person - pd * f.standoff; hover.y = Ground(person, m.spawnPos.y) + f.hoverHeight;
            float tHover = (float)wps[wps.Count - 1].t + 3.5f;
            wps.Add(new TrajectoryBaker.Waypoint { pos = hover, t = tHover, mark = "hover" });
            wps.Add(new TrajectoryBaker.Waypoint { pos = hover + new Vector3(0.05f, 0.1f, 0.03f), t = tHover + f.hoverSeconds * 0.5f });
            wps.Add(new TrajectoryBaker.Waypoint { pos = hover + new Vector3(-0.04f, 0.0f, 0.06f), t = tHover + f.hoverSeconds });
            return TrajectoryBaker.Bake(wps, person + Vector3.up, 1.0 / 120, "scripted sortie");
        }
    }
}
