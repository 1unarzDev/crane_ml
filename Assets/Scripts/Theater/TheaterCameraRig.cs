using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater {
    /// <summary>The theater look's camera rig. It rewrites each act's camera path once, after the plain director has
    /// computed it, so a frame's camera is still a pure function of (manifest, script, frame index):
    /// <list type="bullet">
    /// <item><c>follow</c>/<c>chase</c> with a hero: a damped follow whose aim leads the hero by its smoothed velocity
    /// (<c>lead_s</c>, capped at <c>lead_max</c> metres) from slightly off its quarter (<c>side</c>);</item>
    /// <item><c>hero_orbit</c>: a slow orbit (<c>radius</c>, <c>height</c>, <c>angle</c>, <c>deg_per_s</c>) around the
    /// smoothed hero;</item>
    /// <item><c>tactical</c>: a near-vertical, long-lens top-down view of the course (<c>pitch</c> 84, <c>fov</c> 28,
    /// <c>margin</c>, <c>push</c>);</item>
    /// <item>every path: zero-phase Gaussian smoothing of position, aim and up (<c>smooth_s</c>), slow push-ins on the
    /// heroes' events (<c>push_in</c>, a field-of-view fraction), and an eased dolly from where the previous act's
    /// camera ended (<c>transition_s</c>, default 1.2; <c>"cut": true</c> keeps a hard cut).</item>
    /// </list></summary>
    public static class TheaterCameraRig {
        public static void Apply(TheaterState st) {
            ActPlan prev = null;
            foreach (var p in st.Plans) {
                var a = p.act;
                string shot = a.Shot;
                if (shot == "hero_orbit" && p.hero >= 0) HeroOrbit(st, p);
                else if (shot == "tactical") Tactical(st, p);
                else if ((shot == "follow" || shot == "chase") && p.hero >= 0) Follow(st, p);
                bool close = TheaterLook.CloseShot(shot);
                Smooth(p, a.Cam("smooth_s", close ? 0.22f : 0.12f) * st.S.Fps);
                PushIns(st, p);
                if (prev != null && !TJson.Bool(a.camera, "cut", false) && !a.camera.ContainsKey("blend_in_s")) Transition(st, prev, p);
                prev = p;
            }
        }

        // ------------------------------------------------------------------ shots

        static Vector3[] HeroTrack(TheaterState st, ActPlan p, float omega) {
            int n = p.camPos.Length;
            var target = new Vector3[n];
            var r = st.M.Runs[p.hero];
            for (int f = 0; f < n; f++) {
                float t = st.ActRunTime(p, f / st.S.Fps, out _);
                target[f] = st.UnityPos(st.M.Pack.At(p.hero, Mathf.Clamp(t, 0, r.duration)));
            }
            Damp(target, omega, 1f / st.S.Fps);
            return target;
        }

        static void Follow(TheaterState st, ActPlan p) {
            var a = p.act;
            int n = p.camPos.Length;
            float dt = 1f / st.S.Fps;
            var target = HeroTrack(st, p, a.Cam("stiffness", 1.8f));
            // heading from the smoothed velocity (the hull's yaw where the boat is nearly still)
            var heading = new Vector3[n];
            var speed = new float[n];
            var r = st.M.Runs[p.hero];
            for (int f = 0; f < n; f++) {
                Vector3 v = (target[Mathf.Min(n - 1, f + 1)] - target[Mathf.Max(0, f - 1)]) / (dt * (Mathf.Min(n - 1, f + 1) - Mathf.Max(0, f - 1)));
                v.y = 0;
                speed[f] = v.magnitude;
                float t = st.ActRunTime(p, f * dt, out _);
                float yaw = st.World.UnityYaw(st.M.Pack.At(p.hero, Mathf.Clamp(t, 0, r.duration)).yaw) * Mathf.Deg2Rad;
                var hullDir = new Vector3(Mathf.Sin(yaw), 0, Mathf.Cos(yaw));
                heading[f] = speed[f] > 0.05f ? Vector3.Lerp(hullDir, v / speed[f], Mathf.Clamp01(speed[f] / 0.4f)) : hullDir;
            }
            Damp(heading, a.Cam("heading_stiffness", 0.9f), dt);
            float back = a.Cam("back", 9f), height = a.Cam("height", 5.5f), side = a.Cam("side", 1.8f);
            float leadS = a.Cam("lead_s", 1.2f), leadMax = a.Cam("lead_max", 6f);
            for (int f = 0; f < n; f++) {
                var hd = heading[f]; hd.y = 0;
                hd = hd.sqrMagnitude > 1e-6f ? hd.normalized : Vector3.forward;
                var right = Vector3.Cross(Vector3.up, hd);
                float lead = Mathf.Min(leadMax, 1.0f + speed[f] * leadS);   // where the hero will be lead_s video seconds on
                var look = target[f] + hd * lead;
                var pos = target[f] - hd * back + Vector3.up * height + right * side;
                Set(st, p, f, pos, look, Vector3.up, a.Cam("fov", 48f));
            }
        }

        static void HeroOrbit(TheaterState st, ActPlan p) {
            var a = p.act;
            var target = HeroTrack(st, p, a.Cam("stiffness", 1.2f));
            float radius = a.Cam("radius", 11f), height = a.Cam("height", 4.5f);
            Vector3 north = st.North, east = st.East;
            for (int f = 0; f < target.Length; f++) {
                float tau = f / st.S.Fps;
                float ang = (a.Cam("angle", 200f) + a.Cam("deg_per_s", 7f) * tau) * Mathf.Deg2Rad;
                var pos = target[f] + (east * Mathf.Cos(ang) + north * Mathf.Sin(ang)) * radius + Vector3.up * height;
                Set(st, p, f, pos, target[f] + Vector3.up * 0.4f, Vector3.up, a.Cam("fov", 42f));
            }
        }

        static void Tactical(TheaterState st, ActPlan p) {
            var a = p.act;
            Bounds B = st.World.Bounds;
            Vector3 C = B.center, north = st.North, east = st.East;
            float fov = a.Cam("fov", 28f);
            float aspect = st.S.Width / (float)st.S.Height;
            float vt = Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad), ht = vt * aspect;
            float spanE = Mathf.Abs(Vector3.Dot(B.size, east)), spanN = Mathf.Abs(Vector3.Dot(B.size, north));
            float pitch = a.Cam("pitch", 84f) * Mathf.Deg2Rad;
            var fwd = (north * Mathf.Cos(pitch) - Vector3.up * Mathf.Sin(pitch)).normalized;
            float d0 = Mathf.Max(spanE * 0.5f / ht, spanN * 0.5f / vt) * a.Cam("margin", 1.1f);
            int n = p.camPos.Length;
            for (int f = 0; f < n; f++) {
                float u = n > 1 ? f / (float)(n - 1) : 0f;
                float d = d0 * (1f - a.Cam("push", 0.05f) * Ease(u));
                Set(st, p, f, C - fwd * d, C, north, fov);
            }
        }

        static void Set(TheaterState st, ActPlan p, int f, Vector3 pos, Vector3 look, Vector3 up, float fov) {
            float minY = st.World.SurfaceY(pos) + 1.0f;
            if (pos.y < minY) pos.y = minY;
            var fwd = look - pos;
            if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
            fwd.Normalize();
            if (Mathf.Abs(Vector3.Dot(fwd, up)) > 0.999f) up = st.North;
            p.camPos[f] = pos;
            p.camRot[f] = Quaternion.LookRotation(fwd, up);
            p.camFov[f] = fov;
        }

        // ------------------------------------------------------------------ path filters

        /// <summary>Zero-phase Gaussian (sigma in frames) of position, aim, up and field of view over the act.</summary>
        static void Smooth(ActPlan p, float sigma) {
            int n = p.camPos.Length;
            if (n < 3 || sigma < 0.5f) return;
            int k = Mathf.CeilToInt(sigma * 3f);
            var w = new float[k + 1];
            for (int i = 0; i <= k; i++) w[i] = Mathf.Exp(-0.5f * i * i / (sigma * sigma));
            var pos = new Vector3[n]; var fwd = new Vector3[n]; var up = new Vector3[n]; var fov = new float[n];
            for (int f = 0; f < n; f++) {
                Vector3 sp = Vector3.zero, sf = Vector3.zero, su = Vector3.zero; float sv = 0, sw = 0;
                for (int j = -k; j <= k; j++) {
                    int g = f + j;
                    if (g < 0 || g >= n) continue;
                    float ww = w[Mathf.Abs(j)];
                    sp += p.camPos[g] * ww; sf += p.camRot[g] * Vector3.forward * ww; su += p.camRot[g] * Vector3.up * ww;
                    sv += p.camFov[g] * ww; sw += ww;
                }
                pos[f] = sp / sw; fwd[f] = sf; up[f] = su; fov[f] = sv / sw;
            }
            for (int f = 0; f < n; f++) {
                p.camPos[f] = pos[f];
                if (fwd[f].sqrMagnitude > 1e-8f && up[f].sqrMagnitude > 1e-8f) p.camRot[f] = Quaternion.LookRotation(fwd[f].normalized, up[f].normalized);
                p.camFov[f] = fov[f];
            }
        }

        /// <summary>Slow push-ins (a narrower field of view) around the heroes' events in tracking shots.</summary>
        static void PushIns(TheaterState st, ActPlan p) {
            var a = p.act;
            string shot = a.Shot;
            if (a.type == "evolution" || !(TheaterLook.CloseShot(shot) || shot == "pack")) return;
            float amount = a.Cam("push_in", 0.1f);
            if (amount <= 0) return;
            var times = new List<float>();
            foreach (int run in new[] { p.hero, p.previous }) {
                if (run < 0) continue;
                foreach (var e in st.M.Runs[run].events) times.Add((e.t - a.t0) / Mathf.Max(1e-3f, a.speed));
            }
            if (times.Count == 0) return;
            int n = p.camFov.Length;
            for (int f = 0; f < n; f++) {
                float tau = f / st.S.Fps, b = 0;
                foreach (float te in times) {
                    float d = tau - te;
                    float w = d < 0 ? Ease(Mathf.Clamp01(1f + d / 0.9f)) : Ease(Mathf.Clamp01(1f - d / 2.2f));
                    b = Mathf.Max(b, w);
                }
                p.camFov[f] *= 1f - amount * b;
            }
        }

        /// <summary>An eased dolly from the previous act's last camera onto this act's path, lifted at mid-flight.</summary>
        static void Transition(TheaterState st, ActPlan prev, ActPlan p) {
            int n = Mathf.Min(Mathf.RoundToInt(p.act.Cam("transition_s", 1.2f) * st.S.Fps), Mathf.RoundToInt(p.camPos.Length * 0.4f));
            if (n <= 1 || prev.camPos.Length == 0) return;
            Vector3 p0 = prev.camPos[prev.camPos.Length - 1];
            Quaternion r0 = prev.camRot[prev.camRot.Length - 1];
            float f0 = prev.camFov[prev.camFov.Length - 1];
            float lift = Mathf.Min(25f, Vector3.Distance(p0, p.camPos[Mathf.Min(n, p.camPos.Length - 1)]) * 0.12f);
            for (int f = 0; f < n; f++) {
                float w = Ease(f / (float)n);
                p.camPos[f] = Vector3.Lerp(p0, p.camPos[f], w) + Vector3.up * Mathf.Sin(Mathf.PI * w) * lift;
                p.camRot[f] = Quaternion.Slerp(r0, p.camRot[f], w);
                p.camFov[f] = Mathf.Lerp(f0, p.camFov[f], w);
            }
        }

        /// <summary>Smootherstep.</summary>
        public static float Ease(float u) { u = Mathf.Clamp01(u); return u * u * u * (u * (u * 6 - 15) + 10); }

        /// <summary>Critically damped follow of xs (in place), forward from xs[0].</summary>
        static void Damp(Vector3[] xs, float omega, float dt) {
            if (xs.Length == 0) return;
            Vector3 x = xs[0], v = Vector3.zero;
            for (int i = 0; i < xs.Length; i++) {
                float k = omega * dt, e = 1f / (1f + k + 0.48f * k * k + 0.235f * k * k * k);
                Vector3 change = x - xs[i];
                Vector3 temp = (v + omega * change) * dt;
                v = (v - omega * temp) * e;
                x = xs[i] + (change + temp) * e;
                xs[i] = x;
            }
        }
    }
}
