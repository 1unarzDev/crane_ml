using System;
using UnityEngine;
using Sim.Utils.Performance;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils;

namespace Sim.Physics.Water.Dynamics {
    public class FossenDynamics : MonoBehaviour {
        [Header("Features")]
        [SerializeField] private bool Damping = true;
        [SerializeField] private bool AddedMass = true;
        [SerializeField] private bool Coriolis = true;

        [Header("Added mass force derivative terms proportional to acceleration (kg)")]
        [SerializeField, Tooltip("Surge")] private float XdotU = 6.0f;
        [SerializeField, Tooltip("Sway")] private float YdotV = 8.0f;
        [SerializeField, Tooltip("Heave")] private float ZdotW = 2.0f;

        [Header("Added mass moment derivative terms proportional to acceleration (kgm^2)")]
        [SerializeField, Tooltip("Roll")] private float KdotP = 0.15f;
        [SerializeField, Tooltip("Pitch")] private float MdotQ = 0.25f;
        [SerializeField, Tooltip("Yaw")] private float NdotR = 0.35f;

        [Header("Linear damping movement coefficients (Ns/m)")]
        [SerializeField, Tooltip("Surge")] private float Xu = 30.0f;
        [SerializeField, Tooltip("Sway")] private float Yv = 40.0f;
        [SerializeField, Tooltip("Heave")] private float Zw = 150.0f;

        [Header("Linear damping rotational coefficients (Nms/rad)")]
        [SerializeField, Tooltip("Roll")] private float Kp = 8.0f;
        [SerializeField, Tooltip("Pitch")] private float Mq = 12.0f;
        [SerializeField, Tooltip("Yaw")] private float Nr = 20.0f;

        [Header("Quadratic damping linear coefficients (Ns^2/m^2)")]
        [SerializeField, Tooltip("Surge")] private float Xuu = 25.0f;
        [SerializeField, Tooltip("Sway")] private float Yvv = 35.0f;
        [SerializeField, Tooltip("Heave")] private float Zww = 40.0f;

        [Header("Quadratic damping rotational coefficients (Nms^2/rad^2)")]
        [SerializeField, Tooltip("Roll")] private float Kpp = 3.0f;
        [SerializeField, Tooltip("Pitch")] private float Mqq = 6.0f;
        [SerializeField, Tooltip("Yaw")] private float Nrr = 30.0f;

        private const int nStates = 6;
        private float[,] Cor = new float[nStates, nStates];
        private float[,] Ma = new float[nStates, nStates];
        private float[,] D = new float[nStates, nStates];
        private float[] state = new float[nStates];
        private float[] statePrev = new float[nStates];
        private float[] stateDot = new float[nStates];

        private IPhysicsBody body;

        void OnValidate() {
            if (GetComponent<Rigidbody>() == null && GetComponent<ArticulationBody>() == null)
                Debug.LogWarning($"{name} should have either a Rigidbody or an ArticulationBody attached.");
        }

        void Awake() {
            var rb = GetComponent<Rigidbody>();
            var ab = GetComponent<ArticulationBody>();

            if (rb != null) body = new RigidbodyAdapter(rb);
            else if (ab != null) body = new ArticulationBodyAdapter(ab);
            else throw new MissingComponentException($"{name} requires a Rigidbody or ArticulationBody!");
        }

        private void Start() {
            Ma[0, 0] = XdotU;
            Ma[1, 1] = YdotV;
            Ma[2, 2] = ZdotW;
            Ma[3, 3] = KdotP;
            Ma[4, 4] = MdotQ;
            Ma[5, 5] = NdotR;
            state = GetState();
        }

        private void FixedUpdate() {
            using var marker = CraneProfiler.VehicleDynamics.Auto();
            using var componentMarker = CraneProfiler.VehicleDynamicsFossen.Auto();
            state = GetState();
            stateDot = GetStateDot(state, statePrev, Time.deltaTime);
            Cor = CalculateCoriolisMatrix(state);
            D = CalculateDampingMatrix(state);

            (Vector3 Fd, Vector3 Td) = CalculateDampingForceTorque(state);
            (Vector3 Fc, Vector3 Tc) = CalculateCoriolisForceTorque(Cor, state);
            (Vector3 Fma, Vector3 Tma) = CalculateAddedMassForceTorque(Ma, stateDot);

            float DampingFactor = Damping ? 1.0f : 0.0f;
            float CoriolisFactor = Coriolis ? 1.0f : 0.0f;
            float AddedMassFactor = AddedMass ? 1.0f : 0.0f;

            Vector3 F = DampingFactor * Fd + CoriolisFactor * Fc + AddedMassFactor * Fma;
            Vector3 T = DampingFactor * Td + CoriolisFactor * Tc + AddedMassFactor * Tma;

            // Convert from right-handed, z-up to left-handed, y-up (Unity's coordinate system)
            F = new Vector3(-F.y, F.z, F.x); // Switch y and z, negate new y
            T = new Vector3(T.y, -T.z, -T.x); // Switch y and z, negate new z

            body.AddRelativeForce(F);
            body.AddTorque(T);
            Array.Copy(state, statePrev, nStates);
        }

        private float[] GetState() {
            float[] eta = state;
            Vector3 worldVelocity = body.linearVelocity;
            Vector3<FLU> localVelocity = transform.InverseTransformDirection(worldVelocity).To<FLU>();
            Vector3<FLU> localAngularVelocity = -body.angularVelocity.To<FLU>();

            eta[0] = localVelocity.x; //forward velocity
            eta[1] = localVelocity.y;
            eta[2] = localVelocity.z;

            eta[3] = localAngularVelocity.x;
            eta[4] = localAngularVelocity.y;
            eta[5] = localAngularVelocity.z;
            return eta;
        }

        private float[] GetStateDot(float[] currentState, float[] previousState, float dt) {
            float[] retStateDot = new float[6];
            for (var i = 0; i < nStates; i++) {
                retStateDot[i] = (currentState[i] - previousState[i]) / dt;
            }
            return retStateDot;
        }
        private float[,] CalculateCoriolisMatrix(float[] eta) {
            float[,] C = new float[6, 6];
            C[0, 5] = YdotV * eta[1];
            C[1, 5] = XdotU * eta[0];
            C[5, 0] = YdotV * eta[1];
            C[5, 1] = YdotV * eta[0];
            return C;
        }

        private float[,] CalculateDampingMatrix(float[] eta) {
            D[0, 0] = Xu + Xuu * Mathf.Abs(eta[0]);
            D[1, 1] = Yv + Yvv * Mathf.Abs(eta[1]);
            D[2, 2] = Zw + Zww * Mathf.Abs(eta[2]);
            D[3, 3] = Kp + Kpp * Mathf.Abs(eta[3]);
            D[4, 4] = Mq + Mqq * Mathf.Abs(eta[4]);
            D[5, 5] = Nr + Nrr * Mathf.Abs(eta[5]);
            return D;
        }
        private (Vector3, Vector3) CalculateDampingForceTorque(float[] eta) {
            Vector3 Fd = new(); // Damping force
            Vector3 Td = new(); // Damping torque

            // Calculating linear damping force
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Fd[i] += D[i, j] * eta[j];
                }
            }

            // Calculating angular damping torque
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Td[i] += D[i + 3, j + 3] * eta[j + 3];
                }
            }

            return (-Fd, -Td);
        }


        private (Vector3, Vector3) CalculateCoriolisForceTorque(float[,] C, float[] eta) {
            Vector3 Fc = new();
            Vector3 Tc = new();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 6; j++) {
                    if (j < 3)
                        Fc[i] += C[i, j] * eta[j];
                    else
                        Tc[i] += C[i, j] * eta[j];
                }
            }

            return (Fc, Tc);
        }

        private (Vector3, Vector3) CalculateAddedMassForceTorque(float[,] AddedMassMatrix, float[] etaDot) {
            Vector3 F = new();
            Vector3 T = new();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 6; j++) {
                    if (j < 3)
                        F[i] += AddedMassMatrix[i, j] * etaDot[j];
                    else
                        T[i] += AddedMassMatrix[i, j] * etaDot[j];
                }
            }

            return (F, T);
        }
    }
}
