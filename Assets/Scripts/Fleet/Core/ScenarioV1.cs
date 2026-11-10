using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    /// <summary>
    /// The scenes-v1 extension block (<c>x_v1</c>) of a compiled scenario (Tools/Fleet/scenes/crane_compile.py): the v1 fields the
    /// v0 loaders ignore. The v0 part of the same file (box obstacles, zones, the doorway stub) is built by FleetWorld unchanged;
    /// this class only reads what is extra: door records, door thresholds (traversable sills), floor patches with their profile
    /// floor, landmark tags, the v1 zones, units and faults. A scenario without <c>x_v1</c> gives an empty, Present = false block.
    /// UNTESTED IN UNITY at the time of writing (scenes v1 handoff, Docs/Fleet/SCENES_v1.md).
    /// </summary>
    public sealed class ScenarioV1 {
        public sealed class Door { public string Id, Wall; public double Centre, Width, Height, ThresholdHeight, ThresholdDepth; }
        /// <summary>An axis-aligned box in world coordinates (centre, size); thresholds are traversable (not planning obstacles).</summary>
        public sealed class Box { public string Id, Kind; public double Cx, Cy, Cz, Sx, Sy, Sz; public bool Traversable; }
        public sealed class Patch { public string Id, FrictionClass, ProfileFloor; public double X0, Y0, X1, Y1; }
        public sealed class Tag { public int Id; public string Name, On; public double X, Y, Yaw, Z, Size; }
        public sealed class Zone { public string Id, Kind; public double X0, Y0, X1, Y1; public double? RequiredYaw; public int Capacity = 1; }

        public bool Present;
        public string SourceSha256, Family, Split;
        public readonly List<Door> Doors = new List<Door>();
        public readonly List<Box> Thresholds = new List<Box>();
        public readonly List<Patch> FloorPatches = new List<Patch>();
        public readonly List<Tag> Tags = new List<Tag>();
        public readonly List<Zone> Zones = new List<Zone>();

        public static ScenarioV1 From(JsonNode scenario) {
            var r = new ScenarioV1();
            JsonNode x = scenario.TryGet("x_v1");
            if (x == null) return r;
            r.Present = true;
            r.SourceSha256 = x.Has("source_sha256") ? x["source_sha256"].String : null;
            if (x.Has("family")) { r.Family = x["family.id"].String; r.Split = x["family.split"].String; }
            if (x.Has("doors"))
                foreach (var d in x["doors"].Items)
                    r.Doors.Add(new Door {
                        Id = d["id"].String, Wall = d["wall"].String, Centre = d["centre"].Double, Width = d["width"].Double,
                        Height = d["height"].Double, ThresholdHeight = d.Has("threshold_height") ? d["threshold_height"].Double : 0,
                        ThresholdDepth = d.Has("threshold_depth") ? d["threshold_depth"].Double : 0
                    });
            if (x.Has("thresholds"))
                foreach (var t in x["thresholds"].Items)
                    r.Thresholds.Add(new Box {
                        Id = t["id"].String, Kind = t["kind"].String, Cx = t["cx"].Double, Cy = t["cy"].Double, Cz = t["cz"].Double,
                        Sx = t["sx"].Double, Sy = t["sy"].Double, Sz = t["sz"].Double, Traversable = true
                    });
            if (x.Has("floor_patches"))
                foreach (var p in x["floor_patches"].Items) {
                    double[] rc = p["rect"].Doubles;
                    r.FloorPatches.Add(new Patch {
                        Id = p["id"].String, FrictionClass = p["friction_class"].String,
                        ProfileFloor = p.Has("profile_floor") && p["profile_floor"].Raw is string s ? s : null,
                        X0 = rc[0], Y0 = rc[1], X1 = rc[2], Y1 = rc[3]
                    });
                }
            if (x.Has("tags"))
                foreach (var t in x["tags"].Items) {
                    double[] pose = t["pose"].Doubles;
                    r.Tags.Add(new Tag {
                        Id = (int)t["id"].Double, Name = t["name"].String, On = t.Has("on") ? t["on"].String : "wall",
                        X = pose[0], Y = pose[1], Yaw = pose[2], Z = t.Has("z") ? t["z"].Double : 0.3, Size = t["size"].Double
                    });
                }
            if (x.Has("zones"))
                foreach (var z in x["zones"].Items) {
                    double[] rc = z["rect"].Doubles;
                    r.Zones.Add(new Zone {
                        Id = z["id"].String, Kind = z["kind"].String, X0 = rc[0], Y0 = rc[1], X1 = rc[2], Y1 = rc[3],
                        RequiredYaw = z.Has("required_yaw") ? z["required_yaw"].Double : (double?)null,
                        Capacity = z.Has("capacity") ? (int)z["capacity"].Double : 1
                    });
                }
            return r;
        }

        /// <summary>The profile floor under (x, y): the last patch that contains the point, else <paramref name="arenaFloor"/>.
        /// A patch whose friction class is not in the profile (ProfileFloor null, e.g. "polished") falls back to the arena floor.</summary>
        public string FloorAt(double x, double y, string arenaFloor) {
            string f = arenaFloor;
            foreach (var p in FloorPatches)
                if (x >= p.X0 && x <= p.X1 && y >= p.Y0 && y <= p.Y1 && p.ProfileFloor != null) f = p.ProfileFloor;
            return f;
        }

        /// <summary>Landmark tags (door jambs, bays, slots) for FleetConfig.Landmarks: name -> (x, y, yaw).</summary>
        public IEnumerable<Tag> Landmarks() {
            foreach (var t in Tags) if (t.On == "door" || t.On == "bay" || t.On == "shelf" || t.On == "wall") yield return t;
        }
    }

    public static class ScenarioV1Spec {
        /// <summary>The scenes-v1 extension of this spec's scenario (empty when the scenario is plain v0).</summary>
        public static ScenarioV1 V1(this FleetSpec spec) => ScenarioV1.From(spec.S);

        /// <summary>Floor properties at a world point: the v1 floor patch under it, else the arena floor.</summary>
        public static FloorProperties FloorAt(this FleetSpec spec, double x, double y) {
            string arena = spec.S["arena.floor"].String;
            return spec.Floor(spec.V1().FloorAt(x, y, arena));
        }
    }
}
