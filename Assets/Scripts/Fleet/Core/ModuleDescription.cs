using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    /// <summary>
    /// <see cref="RoverDescription"/> for one module of a capplan module graph (capplan.module_graph/v0, whose "types" embed the
    /// library.py type specs; Tools/Fleet/spec/module_types_v0.json is the exported registry). Geometry, mass, wheels, decks, faces and
    /// sensor kinds come from the type; what the type does not state comes from the profile (ground clearance, wheel width, encoder
    /// resolution, body limits) or the overlay (colours, sensor model names). The plant is the per-wheel <see cref="TractivePlant"/>.
    /// Inertia and centre of mass are derived (uniform box between the clearance and the type height) and labelled so.
    /// </summary>
    public static class ModuleDescription {
        public static readonly string[] SupportedHealth = { "ok", "dead_free", "dead_locked", "weak" };

        public static WheelKind Kind(string k) => k switch {
            "wheel_mecanum" => WheelKind.Mecanum, "wheel_conventional" => WheelKind.Conventional, "wheel_omni" => WheelKind.Omni,
            "wheel_caster" => WheelKind.Caster, _ => throw new FormatException("wheel kind " + k)
        };

        public static WheelHealth Health(string h) => h switch {
            "ok" => WheelHealth.Ok, "dead_free" => WheelHealth.DeadFree, "dead" => WheelHealth.DeadFree, "free" => WheelHealth.DeadFree,
            "dead_locked" => WheelHealth.DeadLocked, "locked" => WheelHealth.DeadLocked, "weak" => WheelHealth.Weak,
            _ => throw new NotSupportedException("wheel health " + h + " (intermittent is not modelled in CRANE)")
        };

        public static RoverDescription Resolve(FleetSpec spec, string id, string typeName, JsonNode type, int instanceIndex) {
            JsonNode p = spec.P, o = spec.O;
            JsonNode ch = p["chassis"], wh = p["chassis.wheel"];
            var d = new RoverDescription { Id = id, Role = typeName, Namespace = "/fleet/" + id, InstanceIndex = instanceIndex };
            d.ModuleType = typeName;
            d.TypeHash = type.Has("type_hash") ? type["type_hash"].String : "";
            double[] size = type["size"].Doubles;
            double height = type["height"].Double, mass = type["mass"].Double;
            var wheels = type["wheels"].Items;
            string origin = o.TryGet("frame_convention.origin")?.String ?? "floor";
            double clearance = wheels.Count > 0 ? ch["ground_clearance"].Double : 0;   // a wheel-less module rests on its underside
            double g = origin == "body_underside" ? clearance : origin == "floor" ? 0 : throw new FormatException("frame_convention.origin " + origin);
            d.OriginHeight = g;
            d.Mass = mass;
            double bh = height - clearance;
            // uniform box about its centroid, FLU (Ixx, Iyy, Izz)
            d.InertiaDiag = new[] { mass / 12 * (size[1] * size[1] + bh * bh), mass / 12 * (size[0] * size[0] + bh * bh), mass / 12 * (size[0] * size[0] + size[1] * size[1]) };
            d.ComOffset = new[] { 0, 0, clearance + bh / 2 - g };
            d.Provenance["mass"] = "capplan type " + typeName + " (" + (type.Has("provenance") ? type["provenance"].String : "assumed") + ")";
            d.Provenance["inertia"] = "derived(uniform box from capplan type size/height)";
            d.Provenance["com"] = "derived(uniform box centroid)";

            double baseColorStep = o["visual.instance_shade_step"].Double * instanceIndex;
            double[] c0 = o.TryGet("visual.role_colors." + typeName)?.Doubles ?? o["visual.module_color"].Doubles;
            d.Color = new[] { Clamp01(c0[0] - baseColorStep), Clamp01(c0[1] - baseColorStep * 0.3), Clamp01(c0[2] + baseColorStep) };

            // wheels: axle at wheel radius above the floor; names <kind initial><index> unless a 4-wheel mecanum set (fl fr rl rr)
            double width = wh["width"].Double;
            bool mecanum4 = wheels.Count == 4;
            foreach (var w in wheels) mecanum4 &= w["kind"].String == "wheel_mecanum";
            string[] mecNames = { "fl", "fr", "rl", "rr" };
            int iw = 0;
            foreach (var w in wheels) {
                var kind = Kind(w["kind"].String);
                double r = w["radius"].Double;
                string name = mecanum4 ? mecNames[iw] : (kind == WheelKind.Caster ? "caster" : kind.ToString().ToLowerInvariant()) + iw;
                d.Wheels.Add(new WheelFrame { Name = name, Axle = new FramePose(w["x"].Double, w["y"].Double, r - g, w["yaw"].Double), Radius = r, Width = width });
                d.WheelUnits.Add(new WheelUnit { Name = name, Kind = kind, X = w["x"].Double, Y = w["y"].Double, Yaw = w["yaw"].Double, Hand = (int)w["hand"].Double,
                    Radius = r, MaxSpeed = w["phi_max"].Double, MaxTorque = w["tau_max"].Double });
                iw++;
            }
            d.WheelContact = o["physics.wheel_contact"].String;
            d.PlantModel = "tractive";
            d.CountsPerRev = (int)wh["encoder_counts_per_rev"].Double;
            d.MaxSpeedXy = ch["max_speed_xy"].Double; d.MaxYawRate = ch["max_yaw_rate"].Double;
            if (mecanum4) {
                var u = d.WheelUnits;
                d.Geometry = new MecanumGeometry(u[0].Radius, u[0].X, u[0].Y, u[0].MaxSpeed, u[0].MaxTorque, d.CountsPerRev, d.MaxSpeedXy, d.MaxYawRate);
            }

            // faces (type face z is above the floor)
            foreach (var kv in type["faces"].Members) {
                var f = kv.Value;
                d.Faces.Add(new DockingFace { Name = kv.Key, Pose = new FramePose(f["x"].Double, f["y"].Double, f["z"].Double - g, f["yaw"].Double), Active = f["couplable"].Bool });
            }

            // sensors: kinds from the type, models from the overlay (module_sensor_models), mounted on the top surface (elevated: same, the type
            // height includes the mast)
            double top = height - g;
            int si = 0;
            foreach (var s in type["sensors"].Items) {
                string kind = s["kind"].String;
                var model = o.TryGet("module_sensor_models." + kind);
                if (model == null) { si++; continue; }   // wheel_encoder etc. are not mounted sensors
                double x = s.Has("x") ? s["x"].Double : 0, y = s.Has("y") ? s["y"].Double : 0, yaw = s.Has("yaw") ? s["yaw"].Double : 0;
                string sid = kind + "_" + si;
                if (kind == "stereo") {
                    double half = p["sensors.stereo_baseline"].Double / 2;
                    d.Sensors.Add(new SensorFrame { Id = sid + "_left", Sensor = model.String, Pose = new FramePose(x, y + half, top, yaw) });
                    d.Sensors.Add(new SensorFrame { Id = sid + "_right", Sensor = model.String, Pose = new FramePose(x, y - half, top, yaw) });
                }
                else d.Sensors.Add(new SensorFrame { Id = sid, Sensor = model.String, Pose = new FramePose(x, y, kind == "imu" ? bh / 2 + clearance - g : top, yaw) });
                si++;
            }
            d.SensorFrameSource = "capplan_type";

            // decks and tools as role properties (the cargo/lifter missions read these names)
            foreach (var dk in type["decks"].Items) {
                d.Payload["deck_height"] = dk["z"].Double; d.Payload["deck_capacity"] = dk["capacity"].Double;
                d.Payload["deck_size_lw_0"] = dk["lx"].Double; d.Payload["deck_size_lw_1"] = dk["ly"].Double;
            }
            foreach (var t in type["tools"].Items)
                foreach (var kv in t.Members) if (kv.Value.Unwrapped is double v) d.Payload[t["kind"].String + "_" + kv.Key] = v;

            // collision and render primitives
            double bodyZ = clearance + bh / 2 - g;
            var lwh = new[] { size[0], size[1], bh };
            d.CollisionParts.Add(new PrimitivePart { Name = "chassis", Kind = PrimitiveKind.Box, Centre = new[] { 0, 0, bodyZ }, Size = lwh });
            d.CollisionSource = "capplan_type";
            d.RenderPrimitives.Add(new PrimitivePart { Name = "chassis", Kind = PrimitiveKind.Box, Centre = new[] { 0, 0, bodyZ }, Size = lwh, Color = d.Color });
            double[] wheelColor = o["visual.wheel_color"].Doubles;
            foreach (var u in d.WheelUnits) {
                var wf = d.Wheels.Find(x => x.Name == u.Name);
                if (u.Kind == WheelKind.Caster)
                    d.RenderPrimitives.Add(new PrimitivePart { Name = "wheel_" + u.Name, Kind = PrimitiveKind.Sphere, Centre = new[] { wf.Axle.X, wf.Axle.Y, wf.Axle.Z },
                        Size = new[] { 2 * u.Radius, 2 * u.Radius, 2 * u.Radius }, Color = wheelColor });
                else
                    d.RenderPrimitives.Add(new PrimitivePart { Name = "wheel_" + u.Name, Kind = PrimitiveKind.Cylinder, Axis = 'y', Yaw = u.Yaw,
                        Centre = new[] { wf.Axle.X, wf.Axle.Y, wf.Axle.Z }, Size = new[] { 2 * u.Radius, width, 2 * u.Radius }, Color = wheelColor });
            }
            foreach (var dk in type["decks"].Items) {
                double t = o["visual.deck_thickness"].Double, dz = dk["z"].Double - g;
                var part = new PrimitivePart { Name = "deck", Kind = PrimitiveKind.Box, Centre = new[] { dk["x"].Double, dk["y"].Double, dz - t / 2 },
                    Size = new[] { dk["lx"].Double, dk["ly"].Double, t }, Color = Lighten(d.Color, 0.35) };
                d.RenderPrimitives.Add(part);
            }
            double[] faceColor = o["visual.face_marker_color"].Doubles;
            double spacing = p["docking.hook_lateral_spacing"].Double;
            foreach (var f in d.Faces)
                d.RenderPrimitives.Add(new PrimitivePart { Name = "face_" + f.Name, Kind = PrimitiveKind.Box, Centre = new[] { f.Pose.X, f.Pose.Y, f.Pose.Z },
                    Size = new[] { 0.004, spacing, 0.02 }, Color = f.Active ? faceColor : Lighten(faceColor, 0.5), Yaw = f.Pose.Yaw });
            d.MeshManifestPresent = false;
            d.MeshManifestPath = "";
            return d;
        }

        /// <summary>Initial per-wheel health of a capplan module (act_health / act_gain), in wheel order.</summary>
        public static (WheelHealth[] health, double[] gain) InitialHealth(JsonNode module, int wheels) {
            var h = new WheelHealth[wheels]; var g = new double[wheels];
            for (int i = 0; i < wheels; i++) { h[i] = WheelHealth.Ok; g[i] = 1; }
            if (module.Has("act_health")) { var a = module["act_health"].Items; for (int i = 0; i < Math.Min(wheels, a.Count); i++) h[i] = Health(a[i].String); }
            if (module.Has("act_gain")) { var a = module["act_gain"].Doubles; for (int i = 0; i < Math.Min(wheels, a.Length); i++) g[i] = a[i]; }
            return (h, g);
        }

        static double Clamp01(double v) => v < 0 ? 0 : v > 1 ? 1 : v;
        static double[] Lighten(double[] c, double k) => new[] { c[0] + (1 - c[0]) * k, c[1] + (1 - c[1]) * k, c[2] + (1 - c[2]) * k };
    }
}
