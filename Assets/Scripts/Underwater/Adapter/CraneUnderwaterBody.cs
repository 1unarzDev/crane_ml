using System;
using Sim.Utils.Performance;
using UnityEngine;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>
    /// Thin adapter: applies the 6-DOF Fossen wrench to ONE Rigidbody every FixedUpdate.
    /// Inert unless launched with --crane-underwater-profile=&lt;name&gt; (or "--crane-underwater-profile name"),
    /// or configured programmatically via Configure(). When inert it disables itself and touches nothing.
    ///
    /// Method: PhysX integrates the rigid body (incl. C_RB gyroscopic terms). The added-mass-aware acceleration
    /// nudot = (M_RB+M_A)^-1 (tau - C_A nu_r - D nu_r - g + M_A nudot_c - C_RB nu) is solved from the measured state and the
    /// force F = M_RB nudot + C_RB nu is applied about CO (force at CO + pure couple), so PhysX reproduces nudot.
    /// PhysX uses its own integrator and step ordering, so this is NOT bit-identical to the RK4 oracle (core parity is
    /// tested separately; Rigidbody-level agreement is NOT_RUN).
    /// CO is the transform origin; the Rigidbody centre of mass is set to the profile CG.
    /// </summary>
    [DisallowMultipleComponent, RequireComponent(typeof(Rigidbody))]
    public sealed class CraneUnderwaterBody : MonoBehaviour, ICraneEpisodeResettable {
        public const string ProfileArgument = "--crane-underwater-profile";

        [SerializeField] float seaLevelUnityY;
        [SerializeField] Vector3 currentNed;      // inertial-frame current, m/s, NED

        Rigidbody rb;
        CraneUnderwaterProfile profile;
        CraneFossenSimulator sim;
        CraneThrusterBank bank;
        double[] commandN;
        double[] lastWrenchFrd = new double[6];   // total wrench applied this step (about CO, FRD)
        double[] lastNu = new double[6];
        long stepCount;
        Vector3 initialPosition; Quaternion initialRotation; bool captured;

        public bool Active => profile != null && enabled;
        public CraneUnderwaterProfile Profile => profile;
        public CraneThrusterBank Bank => bank;
        public long StepCount => stepCount;
        public double[] LastWrenchFrd => lastWrenchFrd;
        public double[] LastNu => lastNu;
        public int ResetPriority => -50;

        public static string ReadProfileName(string[] args) {
            for (int i = 0; i < args.Length; i++) {
                string a = args[i];
                if (a.StartsWith(ProfileArgument + "=", StringComparison.Ordinal)) return a.Substring(ProfileArgument.Length + 1);
                if (a == ProfileArgument && i + 1 < args.Length) return args[i + 1];
            }
            return null;
        }

        void Awake() => InitializeFromCommandLine(Environment.GetCommandLineArgs());

        /// <summary>Opt-in gate used by Awake: inert (component disabled, Rigidbody untouched) unless the flag is present.</summary>
        public void InitializeFromCommandLine(string[] args) {
            rb = GetComponent<Rigidbody>();
            string name = ReadProfileName(args);
            if (string.IsNullOrEmpty(name)) { enabled = false; return; }
            Configure(CraneUnderwaterProfile.LoadFromResources(name));
        }

        /// <summary>Activate with a loaded profile (used by Awake and by tests).</summary>
        public void Configure(CraneUnderwaterProfile p) {
            rb = rb != null ? rb : GetComponent<Rigidbody>();
            profile = p;
            sim = new CraneFossenSimulator(p.Params, new[] { (double)currentNed.x, currentNed.y, currentNed.z }, p.Thrusters);
            bank = new CraneThrusterBank(p.Thrusters);
            commandN = new double[p.Thrusters.Count];
            var par = p.Params;
            rb.useGravity = false;
            rb.linearDamping = 0f; rb.angularDamping = 0f;
            rb.mass = (float)par.Mass;
            // FRD (x fwd, y stbd, z down) -> Unity local (x right, y up, z fwd)
            var cg = CraneUnderwaterFrames.NedToUnity(par.RG);
            rb.centerOfMass = new Vector3((float)cg[0], (float)cg[1], (float)cg[2]);
            rb.inertiaTensor = new Vector3((float)par.InertiaCg[1, 1], (float)par.InertiaCg[2, 2], (float)par.InertiaCg[0, 0]);
            rb.inertiaTensorRotation = Quaternion.identity;
            CaptureEpisodeInitialState();
            enabled = true;
        }

        /// <summary>Per-thruster force commands in N (profile order). Clamped/deadbanded by the bank.</summary>
        public void SetThrusterCommands(float[] newtons) {
            if (commandN == null) return;
            for (int i = 0; i < commandN.Length && i < newtons.Length; i++) commandN[i] = newtons[i];
        }

        public void SetCurrentNed(Vector3 c) {
            currentNed = c;
            if (sim != null) sim.CurrentN = new[] { (double)c.x, c.y, c.z };
        }

        void FixedUpdate() => StepFixed(Time.fixedDeltaTime);

        public void StepFixed(float dt) {
            if (sim == null) return;
            var rot = rb.rotation;
            var qNed = CraneUnderwaterFrames.UnityQuatToNedFrd(rot.x, rot.y, rot.z, rot.w);
            // measured body-frame velocity at CO
            var inv = Quaternion.Inverse(rot);
            Vector3 vG = inv * rb.linearVelocity, w = inv * rb.angularVelocity;
            var vGf = CraneUnderwaterFrames.UnityToNed(vG.x, vG.y, vG.z);
            var wf = CraneUnderwaterFrames.UnityAxialToNed(w.x, w.y, w.z);
            var vCo = Add(vGf, Cross(Scale(wf, -1.0), profile.Params.RG));   // v_CO = v_G + w x (r_CO - r_G) = v_G - w x r_g
            // (FRD body: CG offset is r_g from CO, so CO - CG = -r_g)
            var nu = new[] { vCo[0], vCo[1], vCo[2], wf[0], wf[1], wf[2] };
            var pos = rb.position;
            var state = new CraneState { Pos = new double[] { pos.z, pos.x, -(pos.y - seaLevelUnityY) }, Q = qNed, Nu = nu, Thrust = new double[0] };

            var applied = bank.Step(dt, commandN);
            // nudot including thruster wrench, without PhysX-owned C_RB: use full model then add C_RB nu back.
            var nudot = sim.Acceleration(state, applied);
            var wrench = Add(Mul(sim.MRb, nudot), Mul(CraneFossen.CoriolisFromMass(sim.MRb, nu), nu));

            var fU = CraneUnderwaterFrames.NedToUnity(Slice(wrench, 0, 3));
            var tU = CraneUnderwaterFrames.NedAxialToUnity(Slice(wrench, 3, 3));
            var fW = rot * new Vector3((float)fU[0], (float)fU[1], (float)fU[2]);
            var tW = rot * new Vector3((float)tU[0], (float)tU[1], (float)tU[2]);
            if (!(float.IsFinite(fW.x) && float.IsFinite(fW.y) && float.IsFinite(fW.z) && float.IsFinite(tW.x) && float.IsFinite(tW.y) && float.IsFinite(tW.z))) {
                Debug.LogError("[CraneUnderwaterBody] non-finite wrench; skipping step");
                return;
            }
            rb.AddForceAtPosition(fW, rb.position, ForceMode.Force);
            rb.AddTorque(tW, ForceMode.Force);
            lastWrenchFrd = wrench; lastNu = nu;
            stepCount++;
        }

        public void CaptureEpisodeInitialState() {
            if (rb == null) rb = GetComponent<Rigidbody>();
            initialPosition = transform.position; initialRotation = transform.rotation; captured = true;
        }

        public void ResetEpisode(in CraneEpisodeResetContext context, CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            if (rb == null) rb = GetComponent<Rigidbody>();
            bank?.Reset();
            if (commandN != null) Array.Clear(commandN, 0, commandN.Length);
            lastWrenchFrd = new double[6]; lastNu = new double[6];
            stepCount = 0;
            if (captured) {
                transform.SetPositionAndRotation(initialPosition, initialRotation);
                rb.position = initialPosition; rb.rotation = initialRotation;
            }
            rb.linearVelocity = Vector3.zero; rb.angularVelocity = Vector3.zero;
        }
    }
}
