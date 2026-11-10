using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater.Aerial {
    /// <summary>Where a shot's camera path or flight points are expressed.</summary>
    public enum PathSpace { DroneStart, Spawn, World }

    public sealed class TransitionSpec { public string type = "cut"; public float duration = 0.5f; }   // cut | dip (to black) | fade (from black)
    public sealed class TextSpec { public string text = "", sub = ""; public float at = 0.5f, duration = 3.5f; }

    public sealed class ShotSpec {
        public string id = "", type = "chase";   // establishing | descent | chase | fpv_split | orbit | pullout
        public float duration = 8f;
        public string t0Mark = "";               // flight mark the shot starts at ("" = continue from previous shot)
        public float t0Offset;                   // s after the mark
        public float timeScale = 1f;             // flight seconds per video second
        public PathSpace space = PathSpace.DroneStart;
        public List<Vector3> path = new List<Vector3>();   // camera control points (Catmull-Rom); empty for chase/orbit
        public string look = "drone";            // drone | person | mid
        public float fov = 50f, fovEnd = -1f;
        public float damping = 0.35f;            // s, critically damped time constant of camera and look point
        public float back = 4f, up = 1.4f, side;  // chase offsets, metres (drone yaw frame)
        public float radius = 5f, height = 1.6f, startDeg, sweepDeg = 300f;   // orbit
        public float aperture = 0f, focusBias;   // f-number (0 = DOF off), metres added to the subject distance
        public bool detection;
        public string inset = "";                // "" | split | pip (onboard feed beside the third-person view)
        public bool avoidTrees = true;           // pull the camera in when a collider sits between it and the drone
        public TransitionSpec transition = new TransitionSpec();
        public TextSpec title, lowerThird;       // title card / lower third
        public List<float> stills = new List<float> { 0.5f };   // fractions of the shot
    }

    public sealed class FlightSpec {
        public string kind = "scripted";         // scripted | trajectory | plant
        public string file = "";                 // trajectory CSV (kind trajectory)
        public bool throughPlant;                // route a scripted/recorded reference through the physics follower
        public float topAltitude = 30f;          // m above the spawn point over the canopy
        public float cruiseHeight = 2.2f, hoverHeight = 1.7f, standoff = 3.2f, cruise = 4.5f, hoverSeconds = 22f;
        public PathSpace space = PathSpace.Spawn;
        public List<TrajPoint> points = new List<TrajPoint>();   // scripted waypoints when the scene has no corridor markers
    }

    public struct TrajPoint { public Vector3 pos; public float t; public string mark; }

    /// <summary>A polished-demo sequence (crane-aerial-demo/1). Positions are Unity metres (+x right, +y up, +z forward).</summary>
    public sealed class DemoSpec {
        public const string Schema = "crane-aerial-demo/1";
        public string title = "", subtitle = "";
        public float fps = 30f;
        public int width = 1920, height = 1080;
        public int seed = 1;
        public string scene = "Assets/Scenes/Drone Forest.unity";
        public string prefab = "Assets/Drones/Models/AOS7Quad.prefab";
        public string lightPreset = "GoldenHour";
        public float canopyTop = 18f;
        public Vector3 personOffset = new Vector3(0, 0, 40);   // from spawn when the scene has no Target marker
        public FlightSpec flight = new FlightSpec();
        public readonly List<ShotSpec> shots = new List<ShotSpec>();

        public float Duration { get { float d = 0; foreach (var s in shots) d += s.duration; return d; } }
        public int FrameCount => Mathf.RoundToInt(Duration * fps);

        static Vector3 V3(object o, Vector3 fb) {
            if (o is List<object> l && l.Count == 3) return new Vector3((float)(double)l[0], (float)(double)l[1], (float)(double)l[2]);
            return fb;
        }
        static Vector3 V3(object doc, string key, Vector3 fb) => doc is Dictionary<string, object> d && d.TryGetValue(key, out var o) ? V3(o, fb) : fb;
        static PathSpace Space(string s, PathSpace fb) {
            switch ((s ?? "").ToLowerInvariant()) { case "drone_start": return PathSpace.DroneStart; case "spawn": return PathSpace.Spawn; case "world": return PathSpace.World; default: return fb; }
        }
        static TextSpec Text(object doc, string key) {
            var o = TJson.Obj(doc, key); if (o == null) return null;
            return new TextSpec { text = TJson.Str(o, "text", ""), sub = TJson.Str(o, "sub", ""), at = (float)TJson.Num(o, "at_s", 0.5), duration = (float)TJson.Num(o, "duration_s", 3.5) };
        }

        public static DemoSpec Parse(string json) {
            var doc = TJson.Parse(json);
            if (TJson.Str(doc, "schema") != Schema) throw new ArgumentException("not a " + Schema + " document");
            var s = new DemoSpec {
                title = TJson.Str(doc, "title", ""), subtitle = TJson.Str(doc, "subtitle", ""), fps = (float)TJson.Num(doc, "fps", 30),
                seed = (int)TJson.Num(doc, "seed", 1), scene = TJson.Str(doc, "scene", "Assets/Scenes/Drone Forest.unity"),
                prefab = TJson.Str(doc, "prefab", "Assets/Drones/Models/AOS7Quad.prefab"), lightPreset = TJson.Str(doc, "light_preset", "GoldenHour"),
                canopyTop = (float)TJson.Num(doc, "canopy_top_m", 18), personOffset = V3(doc, "person_offset", new Vector3(0, 0, 40))
            };
            var size = TJson.Arr(doc, "size");
            if (size != null && size.Count == 2) { s.width = (int)(double)size[0]; s.height = (int)(double)size[1]; }
            var fl = TJson.Obj(doc, "flight");
            if (fl != null) {
                var f = s.flight;
                f.kind = TJson.Str(fl, "kind", "scripted"); f.file = TJson.Str(fl, "file", ""); f.throughPlant = TJson.Bool(fl, "through_plant", false);
                f.topAltitude = (float)TJson.Num(fl, "top_altitude_m", 30); f.cruiseHeight = (float)TJson.Num(fl, "cruise_height_m", 2.2);
                f.hoverHeight = (float)TJson.Num(fl, "hover_height_m", 1.7); f.standoff = (float)TJson.Num(fl, "standoff_m", 3.2);
                f.cruise = (float)TJson.Num(fl, "cruise_ms", 4.5); f.hoverSeconds = (float)TJson.Num(fl, "hover_s", 22);
                f.space = Space(TJson.Str(fl, "space"), PathSpace.Spawn);
                foreach (var o in TJson.Arr(fl, "points") ?? new List<object>())
                    f.points.Add(new TrajPoint { pos = V3(TJson.Arr(o, "p") != null ? (object)TJson.Arr(o, "p") : null, Vector3.zero), t = (float)TJson.Num(o, "t", 0), mark = TJson.Str(o, "mark", "") });
            }
            foreach (var o in TJson.Arr(doc, "shots") ?? new List<object>()) {
                var h = new ShotSpec {
                    id = TJson.Str(o, "id", "shot" + s.shots.Count), type = TJson.Str(o, "type", "chase"), duration = (float)TJson.Num(o, "duration_s", 8),
                    t0Mark = TJson.Str(o, "t0_mark", ""), t0Offset = (float)TJson.Num(o, "t0_offset_s", 0), timeScale = (float)TJson.Num(o, "time_scale", 1),
                    space = Space(TJson.Str(o, "space"), PathSpace.DroneStart), look = TJson.Str(o, "look", "drone"), fov = (float)TJson.Num(o, "fov", 50),
                    fovEnd = (float)TJson.Num(o, "fov_end", -1), damping = (float)TJson.Num(o, "damping_s", 0.35), back = (float)TJson.Num(o, "back_m", 4),
                    up = (float)TJson.Num(o, "up_m", 1.4), side = (float)TJson.Num(o, "side_m", 0), radius = (float)TJson.Num(o, "radius_m", 5),
                    height = (float)TJson.Num(o, "height_m", 1.6), startDeg = (float)TJson.Num(o, "start_deg", 0), sweepDeg = (float)TJson.Num(o, "sweep_deg", 300),
                    aperture = (float)TJson.Num(o, "aperture_f", 0), focusBias = (float)TJson.Num(o, "focus_bias_m", 0), detection = TJson.Bool(o, "detection", false),
                    inset = TJson.Str(o, "inset", ""), title = Text(o, "title"), lowerThird = Text(o, "lower_third")
                };
                string ty = h.type;
                h.avoidTrees = TJson.Bool(o, "avoid_trees", ty == "chase" || ty == "fpv_split" || ty == "orbit");
                var tr = TJson.Obj(o, "transition");
                if (tr != null) h.transition = new TransitionSpec { type = TJson.Str(tr, "type", "cut"), duration = (float)TJson.Num(tr, "duration_s", 0.5) };
                foreach (var p in TJson.Arr(o, "path") ?? new List<object>()) h.path.Add(V3(p, Vector3.zero));
                var st = TJson.Arr(o, "stills");
                if (st != null) { h.stills.Clear(); foreach (var x in st) h.stills.Add((float)(double)x); }
                if (h.duration <= 0) throw new ArgumentException("shot " + h.id + ": duration_s must be positive");
                s.shots.Add(h);
            }
            if (s.shots.Count == 0) throw new ArgumentException("demo spec has no shots");
            return s;
        }
    }
}
