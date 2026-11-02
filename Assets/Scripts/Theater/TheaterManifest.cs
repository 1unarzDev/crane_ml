using System;
using System.Collections.Generic;
using System.IO;

namespace Crane.Theater {
    public sealed class TheaterEvent {
        public float t;
        public string kind, reason;
        public float x, y;
    }

    public sealed class TheaterRun {
        public int index;            // pack order
        public string id, outcome, failureReason;
        public int iteration;
        public float score, progress, duration;
        public bool valid;
        public Dictionary<string, object> parameters;
        public List<TheaterEvent> events = new List<TheaterEvent>();
        public bool Failed => outcome == "failed" || outcome == "timeout" || outcome == "invalid";
    }

    /// <summary><c>crane-theater/1</c> (theater/pack.py): the pack, the world and robot plug-ins and their inputs,
    /// and every run's metadata in pack order.</summary>
    public sealed class TheaterManifest {
        public string Dir, Study, PackFile;
        public Dictionary<string, object> World, Robot, Water, Script, Raw;
        public readonly List<TheaterRun> Runs = new List<TheaterRun>();
        public readonly SortedDictionary<int, string> Insights = new SortedDictionary<int, string>();
        public TrackPack Pack;
        public int MinIteration, MaxIteration;

        public static TheaterManifest Load(string path) {
            var m = Parse(File.ReadAllText(path), Path.GetDirectoryName(Path.GetFullPath(path)));
            m.Pack = TrackPack.Load(Path.Combine(m.Dir, m.PackFile));
            if (m.Pack.RunCount != m.Runs.Count)
                throw new InvalidDataException($"manifest has {m.Runs.Count} runs, pack {m.Pack.RunCount}");
            for (int i = 0; i < m.Runs.Count; i++) m.Runs[i].duration = m.Pack.Duration(i);
            return m;
        }

        public static TheaterManifest Parse(string json, string dir) {
            var doc = TJson.Parse(json) as Dictionary<string, object>;
            if (TJson.Str(doc, "schema") != "crane-theater/1") throw new InvalidDataException("not a crane-theater/1 manifest");
            var m = new TheaterManifest {
                Dir = dir, Raw = doc, Study = TJson.Str(doc, "study", ""), PackFile = TJson.Str(doc, "pack", "tracks.pack"),
                World = TJson.Obj(doc, "world"), Robot = TJson.Obj(doc, "robot"), Water = TJson.Obj(doc, "water"),
                Script = TJson.Obj(doc, "script")
            };
            int i = 0;
            m.MinIteration = int.MaxValue; m.MaxIteration = int.MinValue;
            foreach (var o in TJson.Arr(doc, "runs") ?? new List<object>()) {
                var r = new TheaterRun {
                    index = i++, id = TJson.Str(o, "id", $"run{i}"), outcome = TJson.Str(o, "outcome", "finished"),
                    failureReason = TJson.Str(o, "failure_reason"), iteration = (int)TJson.Num(o, "iteration"),
                    score = (float)TJson.Num(o, "score"), progress = (float)TJson.Num(o, "progress"),
                    valid = TJson.Bool(o, "valid", true), parameters = TJson.Obj(o, "params") ?? new Dictionary<string, object>()
                };
                foreach (var e in TJson.Arr(o, "events") ?? new List<object>())
                    r.events.Add(new TheaterEvent {
                        t = (float)TJson.Num(e, "t"), kind = TJson.Str(e, "kind", "note"), reason = TJson.Str(e, "reason"),
                        x = (float)TJson.Num(e, "x", double.NaN), y = (float)TJson.Num(e, "y", double.NaN)
                    });
                m.Runs.Add(r);
                m.MinIteration = Math.Min(m.MinIteration, r.iteration);
                m.MaxIteration = Math.Max(m.MaxIteration, r.iteration);
            }
            if (m.Runs.Count == 0) { m.MinIteration = 0; m.MaxIteration = 0; }
            foreach (var o in TJson.Arr(doc, "iterations") ?? new List<object>())
                m.Insights[(int)TJson.Num(o, "iteration")] = TJson.Str(o, "insight", "");
            return m;
        }

        /// <summary>The record holders in iteration order (a run that beat every earlier run's score).</summary>
        public List<TheaterRun> Records() {
            var order = new List<TheaterRun>(Runs);
            order.Sort((a, b) => a.iteration != b.iteration ? a.iteration.CompareTo(b.iteration) : a.index.CompareTo(b.index));
            var rec = new List<TheaterRun>();
            float best = float.NegativeInfinity;
            foreach (var r in order)
                if (r.valid && r.score > best) { best = r.score; rec.Add(r); }
            return rec;
        }

        /// <summary>Best score per iteration (NaN where an iteration has no valid run).</summary>
        public float[] BestPerIteration() {
            var best = new float[MaxIteration - MinIteration + 1];
            for (int i = 0; i < best.Length; i++) best[i] = float.NaN;
            foreach (var r in Runs) {
                if (!r.valid) continue;
                int k = r.iteration - MinIteration;
                if (float.IsNaN(best[k]) || r.score > best[k]) best[k] = r.score;
            }
            return best;
        }
    }
}
