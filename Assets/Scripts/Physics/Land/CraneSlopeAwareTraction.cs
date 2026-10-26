using System;
using UnityEngine;

namespace Sim.Physics.LandTerrain
{
    // Thin Unity adapter over SlopeTractionModel. Opt-in: does nothing unless the process was started
    // with --crane-terrain-traction=slope-v1 (or a test sets ForceEnabledForTests before Awake).
    // It is not wired into CraneCampusRobotModel (decision D12): the owner adds the component to a
    // robot whose colliders have zero friction, then calls SetWheelSurfaceSpeeds each step.
    // Frame mapping: core x forward = Unity +z, core y left = Unity -x, core z up = Unity +y.
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class CraneSlopeAwareTraction : MonoBehaviour
    {
        public const string FlagValue = "--crane-terrain-traction=slope-v1";
        public static bool ForceEnabledForTests;

        [Serializable]
        public struct SupportPoint
        {
            public Vector2 forwardLeftMeters;   // contact point in the body plane (x forward, y left)
            public bool driven;
        }

        public SupportPoint[] supports =
        {
            new SupportPoint { forwardLeftMeters = new Vector2(0f, 0.115f), driven = true },
            new SupportPoint { forwardLeftMeters = new Vector2(0f, -0.115f), driven = true },
            new SupportPoint { forwardLeftMeters = new Vector2(0.12f, 0f), driven = false },
            new SupportPoint { forwardLeftMeters = new Vector2(-0.12f, 0f), driven = false },
        };
        public Vector3 cgForwardLeftHeight = new Vector3(0f, 0f, 0.05f);   // prior; measure before tip-over claims
        public float wheelRadius = 0.033f;
        public TractionParamsSerializable parameters = new TractionParamsSerializable();

        [Serializable]
        public sealed class TractionParamsSerializable
        {
            public double mu = 0.7, longitudinalStiffness = 12, lateralStiffness = 8, rollingResistance = 0.02,
                casterFriction = 0.02, wheelTorqueLimit = 1e9, velocityEpsilon = 0.05, loadFilterSeconds = 0;
        }

        public static bool FlagRequested()
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == FlagValue) return true;
                if (args[i] == "--crane-terrain-traction" && i + 1 < args.Length && args[i + 1] == "slope-v1") return true;
            }
            return false;
        }

        public bool Active { get; private set; }
        public VehicleForces LastForces { get; private set; }

        Rigidbody body;
        SlopeTractionModel model;
        double[] surfaceSpeeds;
        Vector3 previousVelocityBody;
        bool havePrevious;

        void Awake()
        {
            Active = ForceEnabledForTests || FlagRequested();
            if (!Active) { enabled = false; return; }
            body = GetComponent<Rigidbody>();
            var geo = new VehicleGeometry
            {
                mass = body.mass,
                wheelRadius = wheelRadius,
                cg = new V3(cgForwardLeftHeight.x, cgForwardLeftHeight.y, cgForwardLeftHeight.z),
                supports = new Support[supports.Length],
            };
            for (int i = 0; i < supports.Length; i++)
                geo.supports[i] = new Support { x = supports[i].forwardLeftMeters.x, y = supports[i].forwardLeftMeters.y, driven = supports[i].driven };
            var p = new TractionParams
            {
                mu = parameters.mu, longitudinalStiffness = parameters.longitudinalStiffness,
                lateralStiffness = parameters.lateralStiffness, rollingResistance = parameters.rollingResistance,
                casterFriction = parameters.casterFriction, wheelTorqueLimit = parameters.wheelTorqueLimit,
                velocityEpsilon = parameters.velocityEpsilon, loadFilterSeconds = parameters.loadFilterSeconds,
                gravity = UnityEngine.Physics.gravity.magnitude,
            };
            model = new SlopeTractionModel(geo, p);
            surfaceSpeeds = new double[supports.Length];
        }

        // Commanded wheel surface speed (omega r, m/s) for left/right driven wheels. Zero = braked hold.
        public void SetWheelSurfaceSpeeds(double left, double right)
        {
            if (surfaceSpeeds == null) return;
            for (int i = 0; i < supports.Length; i++)
                if (supports[i].driven) surfaceSpeeds[i] = supports[i].forwardLeftMeters.y >= 0 ? left : right;
        }

        // Episode reset: clears load filter, acceleration history and the last output.
        public void ResetState()
        {
            model?.Reset();
            if (surfaceSpeeds != null) Array.Clear(surfaceSpeeds, 0, surfaceSpeeds.Length);
            havePrevious = false;
            LastForces = default;
        }

        void OnDisable() { havePrevious = false; }

        static V3 ToCore(Vector3 u) { return new V3(u.z, -u.x, u.y); }

        void FixedUpdate()
        {
            if (!Active || model == null) return;
            float dt = Time.fixedDeltaTime;
            Vector3 vb = transform.InverseTransformDirection(body.linearVelocity);
            Vector3 wb = transform.InverseTransformDirection(body.angularVelocity);
            V3 vc = ToCore(vb);
            double ax = 0, ay = 0;
            if (havePrevious)
            {
                V3 pc = ToCore(previousVelocityBody);
                ax = (vc.x - pc.x) / dt; ay = (vc.y - pc.y) / dt;
            }
            previousVelocityBody = vb; havePrevious = true;
            V3 g = ToCore(transform.InverseTransformDirection(UnityEngine.Physics.gravity));
            var motion = new BodyMotion { vx = vc.x, vy = vc.y, wz = -wb.y, ax = ax, ay = ay };
            var f = model.Step(g, motion, surfaceSpeeds, dt);
            LastForces = f;
            for (int i = 0; i < supports.Length; i++)
            {
                var s = supports[i].forwardLeftMeters;
                Vector3 localPoint = new Vector3(-s.y, 0f, s.x);
                Vector3 localForce = new Vector3(-(float)f.fy[i], 0f, (float)f.fx[i]);
                body.AddForceAtPosition(transform.TransformDirection(localForce), transform.TransformPoint(localPoint), ForceMode.Force);
            }
        }
    }
}
