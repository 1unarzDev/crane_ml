using System;
using System.Collections.Generic;

namespace Sim.Course {
    /// <summary>A point in the course frame (origin at the boat start, x along the start
    /// heading, y to port, metres).</summary>
    public readonly struct P2 {
        public readonly double X, Y;
        public P2(double x, double y) { X = x; Y = y; }
        public static P2 operator +(P2 a, P2 b) => new P2(a.X + b.X, a.Y + b.Y);
        public static P2 operator -(P2 a, P2 b) => new P2(a.X - b.X, a.Y - b.Y);
        public static P2 operator *(P2 a, double k) => new P2(a.X * k, a.Y * k);
        public double Length => Math.Sqrt(X * X + Y * Y);
        public static P2 Dir(double yaw) => new P2(Math.Cos(yaw), Math.Sin(yaw));
        public P2 Left => new P2(-Y, X);
        public static double Dist(P2 a, P2 b) => (a - b).Length;
        public static double Dot(P2 a, P2 b) => a.X * b.X + a.Y * b.Y;
    }

    /// <summary>A task's anchor: a position and heading in the course frame. Tasks lay their
    /// objects out in anchor-local coordinates, so moving the anchor moves them rigidly.</summary>
    public readonly struct Pose2 {
        public readonly double X, Y, Yaw;
        public Pose2(double x, double y, double yaw) { X = x; Y = y; Yaw = yaw; }
        public P2 Position => new P2(X, Y);
        public P2 At(double forward, double left) {
            double c = Math.Cos(Yaw), s = Math.Sin(Yaw);
            return new P2(X + c * forward - s * left, Y + s * forward + c * left);
        }
        public double YawOf(double localYaw) => CourseFrame.Wrap(Yaw + localYaw);
    }

    public static class CourseFrame {
        public static double Wrap(double a) {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }

        /// <summary>Course frame to world (/crane/odom): rotate by the start yaw, then translate.</summary>
        public static P2 ToWorld(Pose2 start, P2 p) => start.At(p.X, p.Y);

        public static double Mm(double v) => Math.Round(v, 3, MidpointRounding.AwayFromZero);
        public static double Rad(double v) => Math.Round(v, 4, MidpointRounding.AwayFromZero);
    }

    /// <summary>One placed object. Circles have <see cref="Radius"/>; structures (pier spines
    /// and tines) are boxes of <see cref="Length"/> × <see cref="Width"/> about their yaw.</summary>
    public sealed class PlacedObject {
        public string Id, Role, Kind, StateId;
        public P2 At;
        public double Yaw, Z;
        public double Length, Width;  // box structures only
        public bool IsBox => Length > 0;
    }

    public sealed class GateSpec {
        public string Id, Port, Starboard;
        public int Order;
        public double NormalYaw;  // the travel direction through the gate
    }

    public sealed class BaySpec {
        public string Id, Pier, SignId, IndicatorId, IndicatorColor, Vessel;
        public int Dock, Slip, Number;
        public P2 At;
        public double Yaw, Width, Depth;  // yaw: the direction the slip opens toward
        public bool Occupied;
    }

    /// <summary>A state-carrying object's program, recorded in truth (§5).</summary>
    public sealed class StateSpec {
        public string Id, Kind, Initial;
        public JObj Program;
    }

    /// <summary>What one task produced, in the course frame.</summary>
    public sealed class TaskLayout {
        public readonly List<PlacedObject> Objects = new List<PlacedObject>();
        public readonly List<GateSpec> Gates = new List<GateSpec>();
        public readonly List<BaySpec> Bays = new List<BaySpec>();
        public readonly List<StateSpec> States = new List<StateSpec>();
        public readonly JObj Anchors = new JObj();       // named points {x, y[, r]}
        public readonly JObj Semantics = new JObj();
        public List<P2> Footprint = new List<P2>();      // convex, course frame

        public PlacedObject Add(string id, string role, string kind, P2 at, double yaw = 0) {
            var o = new PlacedObject { Id = id, Role = role, Kind = kind, At = at, Yaw = yaw };
            Objects.Add(o);
            return o;
        }

        public void Anchor(string name, P2 p, double radius = 0) {
            var o = new JObj().Set("x", p.X).Set("y", p.Y);
            if (radius > 0) o.Set("r", radius);
            Anchors[name] = o;
        }
    }

    /// <summary>What earlier tasks left, for a task's placement rule.</summary>
    public sealed class PlacementContext {
        public double Side;                       // +1 port, -1 starboard (side S)
        public CourseBoundary Boundary;
        public IOccupancy Occupancy;
        public IReadOnlyDictionary<string, TaskLayout> Placed;
        public IReadOnlyDictionary<string, string> TaskIdsByType;
        public ParamValues Params;
        public CourseRng Rng;

        public TaskLayout ByType(string type) =>
            TaskIdsByType.TryGetValue(type, out string id) && Placed.TryGetValue(id, out var l) ? l : null;
    }

    public sealed class TaskContext {
        public string TaskId;
        public CourseRng Rng;
        public Pose2 Anchor;
        public double Side;
        public ParamValues Params;
        public IOccupancy Occupancy;
        public CourseObjectCatalog Catalog;
        public VesselProfile Vessel;
        public PlacementContext Placement;
    }

    /// <summary>The course area (D): a rectangle in the course frame; its rear edge, behind
    /// the start, is the shore.</summary>
    public sealed class CourseBoundary {
        public double Back = 10, Length = 130, HalfWidth = 65;
        public double ShoreX => -Back;
        public List<P2> Polygon => new List<P2> {
            new P2(-Back, -HalfWidth), new P2(Length - Back, -HalfWidth),
            new P2(Length - Back, HalfWidth), new P2(-Back, HalfWidth) };
        public List<P2> Shore => new List<P2> { new P2(-Back, -HalfWidth), new P2(-Back, HalfWidth) };
        public bool Contains(P2 p, double margin) =>
            p.X >= -Back + margin && p.X <= Length - Back - margin &&
            Math.Abs(p.Y) <= HalfWidth - margin;
    }
}
