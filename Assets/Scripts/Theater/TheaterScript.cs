using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater {
    /// <summary>One act of a theater script (JSON; see <see cref="TheaterScript.Parse"/>).</summary>
    public sealed class TheaterAct {
        public string type = "fleet";          // fleet | evolution | heat | compare | spotlight
        public float duration = 5f;            // video seconds
        public float speed = 4f;               // run seconds per video second
        public float t0;                       // run time at the act's start
        public int iterFrom = int.MinValue, iterTo = int.MaxValue;
        public HashSet<int> iterList;          // only these iterations (evolution acts in a short cut)
        public string outcome = "any";         // any | finished | failed
        public int topK;                       // per iteration, by score (0: all)
        public int sample;                     // at most this many runs, seeded (0: all)
        public int seed = 1;
        public float stagger;                  // seeded per-run start offsets in 0..stagger (fleet acts only)
        public float perIteration = 1f;        // evolution: video seconds per iteration
        public string colour = "progress";     // progress | iteration | score | outcome
        public string ramp = "cividis";
        public string trails = "comet";        // none | comet | top
        public int trailK = 12;
        public float tail = 2.5f;              // comet tail, run seconds
        public bool heat, markers = true;
        public string hero = "auto";           // auto | none | best | <run id>
        public string previous = "auto";       // auto | none | previous | <run id>
        public float ghostScale = 1.6f;
        public float ghostAlpha = 0.9f;
        public string caption = "";
        // hero appearance (defaults = the fleet look every earlier render had)
        public float heroTint = 0.55f;         // opacity of the translucent gold/silver shell over each hero; 0 = no shell (the boat's own materials)
        public bool heroRing = true;           // the ring marker under each hero
        public bool heroTrail = true;          // the hero's (and previous's) path ribbon
        public float trailWidth = 0.34f;       // hero ribbon width (m)
        public float previousTrailWidth = 0.28f;
        public Color? trailColour;             // hero ribbon colour (sRGB, alpha = opacity); null = gold
        public Dictionary<string, object> camera = new Dictionary<string, object> { { "shot", "orbit" } };
        public string Shot => TJson.Str(camera, "shot", "orbit");
        public float Cam(string key, float fallback) => (float)TJson.Num(camera, key, fallback);
    }

    public sealed class TheaterScript {
        public float Fps = 60f;
        public int Width = 1920, Height = 1080;
        public string Title = "";
        public string Hud = "fleet";           // fleet (the study HUD) | minimal (title + caption line) | none
        public bool HudSet;                    // the script named a HUD (else the manifest's "hud", else fleet)
        public string Backend = "";            // which backend produced the run (shown by the minimal HUD)
        public bool CameraRig;                 // the theater look's rig (TheaterCameraRig) rewrites the camera paths
        public object LookDoc;                 // the script's "look": a preset name or {"name", "time_of_day", ...}; null = not set
        public readonly List<TheaterAct> Acts = new List<TheaterAct>();
        public float Duration { get { float d = 0; foreach (var a in Acts) d += a.duration; return d; } }
        public int FrameCount => Mathf.RoundToInt(Duration * Fps);

        /// <summary><c>{"fps", "size": [w, h], "title", "acts": [{"type", "duration_s", "speed", "t0", "iterations": [a, b],
        /// "iteration_list", "outcome", "top_k", "sample", "seed", "stagger_s", "per_iteration_s", "colour", "ramp", "trails", "trail_k",
        /// "tail_s", "heat", "markers", "hero", "previous", "ghost_scale", "ghost_alpha", "caption", "hero_tint", "hero_ring", "hero_trail",
        /// "trail_width", "previous_trail_width", "trail_colour": [r, g, b, a], "camera": {"shot", ...}}], "hud", "backend"}</c></summary>
        public static TheaterScript Parse(object doc) {
            var s = new TheaterScript {
                Fps = (float)TJson.Num(doc, "fps", 60), Title = TJson.Str(doc, "title", ""),
                Hud = NormHud(TJson.Str(doc, "hud", "fleet")), HudSet = TJson.Has(doc, "hud"), Backend = TJson.Str(doc, "backend", "")
            };
            s.LookDoc = doc is Dictionary<string, object> dd && dd.TryGetValue("look", out var lk) ? lk : null;
            var size = TJson.Arr(doc, "size");
            if (size != null && size.Count == 2) { s.Width = (int)(double)size[0]; s.Height = (int)(double)size[1]; }
            foreach (var o in TJson.Arr(doc, "acts") ?? new List<object>()) {
                var a = new TheaterAct {
                    type = TJson.Str(o, "type", "fleet"), duration = (float)TJson.Num(o, "duration_s", 5),
                    speed = (float)TJson.Num(o, "speed", 4), t0 = (float)TJson.Num(o, "t0", 0),
                    outcome = TJson.Str(o, "outcome", "any"), topK = (int)TJson.Num(o, "top_k", 0),
                    sample = (int)TJson.Num(o, "sample", 0), seed = (int)TJson.Num(o, "seed", 1),
                    stagger = (float)TJson.Num(o, "stagger_s", 0), perIteration = (float)TJson.Num(o, "per_iteration_s", 1),
                    colour = TJson.Str(o, "colour", "progress"), ramp = TJson.Str(o, "ramp", "cividis"),
                    trails = TJson.Str(o, "trails", "comet"), trailK = (int)TJson.Num(o, "trail_k", 12),
                    tail = (float)TJson.Num(o, "tail_s", 2.5), heat = TJson.Bool(o, "heat", false),
                    markers = TJson.Bool(o, "markers", true), hero = TJson.Str(o, "hero", "auto"),
                    previous = TJson.Str(o, "previous", "auto"), ghostScale = (float)TJson.Num(o, "ghost_scale", 1.6),
                    ghostAlpha = (float)TJson.Num(o, "ghost_alpha", 0.9), caption = TJson.Str(o, "caption", "")
                };
                a.heroTint = Mathf.Clamp01((float)TJson.Num(o, "hero_tint", 0.55));
                a.heroRing = TJson.Bool(o, "hero_ring", true);
                a.heroTrail = TJson.Bool(o, "hero_trail", true);
                a.trailWidth = Mathf.Max(0f, (float)TJson.Num(o, "trail_width", 0.34));
                a.previousTrailWidth = Mathf.Max(0f, (float)TJson.Num(o, "previous_trail_width", 0.28));
                var tc = TJson.Arr(o, "trail_colour");
                if (tc != null && tc.Count >= 3)
                    a.trailColour = new Color((float)(double)tc[0], (float)(double)tc[1], (float)(double)tc[2], tc.Count > 3 ? (float)(double)tc[3] : 1f);
                var it = TJson.Arr(o, "iterations");
                if (it != null && it.Count == 2) { a.iterFrom = (int)(double)it[0]; a.iterTo = (int)(double)it[1]; }
                var list = TJson.Arr(o, "iteration_list");
                if (list != null) { a.iterList = new HashSet<int>(); foreach (var x in list) a.iterList.Add((int)(double)x); }
                var cam = TJson.Obj(o, "camera");
                if (cam != null) a.camera = cam;
                if (a.type != "fleet") a.stagger = 0f;   // staggered starts only where nothing is compared
                s.Acts.Add(a);
            }
            if (s.Acts.Count == 0) throw new ArgumentException("theater script has no acts");
            return s;
        }

        /// <summary>"fleet" | "minimal" | "none"; anything else is the fleet HUD.</summary>
        public static string NormHud(string v) => v == "minimal" || v == "none" ? v : "fleet";

        /// <summary>The standard cut for a study: establishing fleet coloured by iteration, the evolution with
        /// its counter, the failure heat, best vs previous best, and a chase of the best run.</summary>
        public static TheaterScript Default(TheaterManifest m, float seconds) {
            int iters = m.MaxIteration - m.MinIteration + 1;
            float longest = 0; foreach (var r in m.Runs) longest = Mathf.Max(longest, r.duration);
            var acts = new List<object>();
            float fleet = seconds * 0.3f, evo = seconds * 0.25f, heat = seconds * 0.15f, cmp = seconds * 0.3f;
            acts.Add(Act("fleet", fleet, longest * 0.6f / fleet, "iteration", "viridis", "comet", false,
                $"{m.Runs.Count} runs, {iters} iterations", Cam("pack", ("height", 26), ("back", 22))));
            acts.Add(Act("evolution", evo, longest * 0.5f / Mathf.Max(0.5f, evo / iters), "progress", "cividis", "comet", false,
                "evolution: each iteration's fleet", Cam("top"), ("per_iteration_s", (double)(evo / iters))));
            acts.Add(Act("heat", heat, longest / heat, "outcome", "cividis", "none", true, "where runs went, and where they failed", Cam("top")));
            acts.Add(Act("compare", cmp, Mathf.Max(1.5f, longest * 0.35f / cmp), "progress", "cividis", "top", false,
                "best (gold) vs previous best (silver)", Cam("follow", ("height", 7), ("back", 9))));
            var doc = new Dictionary<string, object> { { "fps", 60.0 }, { "size", new List<object> { 1920.0, 1080.0 } },
                { "title", m.Study }, { "acts", acts } };
            return Parse(doc);
        }

        static Dictionary<string, object> Cam(string shot, params (string, double)[] kv) {
            var d = new Dictionary<string, object> { { "shot", shot } };
            foreach (var (k, v) in kv) d[k] = v;
            return d;
        }

        static Dictionary<string, object> Act(string type, float dur, float speed, string colour, string ramp, string trails, bool heat,
            string caption, Dictionary<string, object> cam, params (string, double)[] extra) {
            var d = new Dictionary<string, object> {
                { "type", type }, { "duration_s", (double)dur }, { "speed", (double)speed }, { "colour", colour }, { "ramp", ramp },
                { "trails", trails }, { "heat", heat }, { "caption", caption }, { "camera", cam }
            };
            foreach (var (k, v) in extra) d[k] = v;
            return d;
        }
    }
}
