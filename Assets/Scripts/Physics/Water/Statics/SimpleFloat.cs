using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using Unity.Mathematics;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Sim.Physics.Misc {
    [ExecuteInEditMode] // Optional, may turn off for battery saving
    public class SimpleFloat : MonoBehaviour {
        [Header("Water Settings")]
        public WaterSurface targetSurface;
        public bool includeDeformers = true;
        public bool useRotation = false; 
        public float verticalOffset = 0.08f;
    
        [Header("Follow Settings")]
        public bool followWaterCurrent = false;
        public float currentSpeedMultiplier = 1f;
    
        [Header("Boat Dimensions")]
        public float length = 1f;
        public float width = 1f;
        public bool showGizmos = false;
    
        [Header("Motion Settings")]
        public float positionLerpSpeed = 5f; // Smooth speed for vertical motion
        public float rotationLerpSpeed = 2f; // Smooth speed for rotation
    
        [Header("Collision Settings")]
        public bool useRigidbodyForCollision = false;
        public LayerMask obstacleLayers;
    
        private bool disableCurrentFlow = false;
        private Rigidbody rb;
        private WaterSearchParameters searchParams = new();
        private WaterSearchResult searchResult = new();
        private Vector3 smoothedPosition;
        private Quaternion smoothedRotation;
    
        void Awake() {
            rb = GetComponent<Rigidbody>();
            if (rb == null && useRigidbodyForCollision)
                rb = gameObject.AddComponent<Rigidbody>();
    
            if (rb != null) {
                rb.useGravity = false;
                rb.isKinematic = !useRigidbodyForCollision;
            }
    
            smoothedPosition = transform.position;
            smoothedRotation = transform.rotation;
        }
    
        void LateUpdate() {
            if (targetSurface == null) return;

            // For moving objects in editor
            smoothedPosition.x = transform.position.x;
            smoothedPosition.z = transform.position.z;
            
            // Sampling points around the boat
            if (useRotation) {
                Vector3 localBow = transform.forward * (length / 2f);
                Vector3 localStern = -transform.forward * (length / 2f);
                Vector3 localLeft = -transform.right * (width / 2f);
                Vector3 localRight = transform.right * (width / 2f);

                Vector3 worldBow = transform.position + localBow;
                Vector3 worldStern = transform.position + localStern;
                Vector3 worldLeft = transform.position + localLeft;
                Vector3 worldRight = transform.position + localRight;

                float hBow = GetWaterHeight(worldBow);
                float hStern = GetWaterHeight(worldStern);
                float hLeft = GetWaterHeight(worldLeft);
                float hRight = GetWaterHeight(worldRight);

                Vector3 adjustedBow = new(worldBow.x, hBow, worldBow.z);
                Vector3 adjustedStern = new(worldStern.x, hStern, worldStern.z);
                Vector3 adjustedLeft = new(worldLeft.x, hLeft, worldLeft.z);
                Vector3 adjustedRight = new(worldRight.x, hRight, worldRight.z);

                // Directions and water normal
                Vector3 forwardDir = (adjustedBow - adjustedStern).normalized;
                Vector3 rightDir = (adjustedRight - adjustedLeft).normalized;
                Vector3 waterNormal = Vector3.Cross(forwardDir, rightDir).normalized;

                float avgHeight = (hBow + hStern + hLeft + hRight) / 4f;
                smoothedPosition.y = Mathf.Lerp(smoothedPosition.y, avgHeight + verticalOffset, positionLerpSpeed * Time.deltaTime);
                smoothedRotation = Quaternion.Slerp(smoothedRotation, Quaternion.FromToRotation(transform.up, waterNormal) * transform.rotation, rotationLerpSpeed * Time.deltaTime);
            }
            else {
                float height = GetWaterHeight(transform.position);
                smoothedPosition.y = Mathf.Lerp(smoothedPosition.y, height + verticalOffset, positionLerpSpeed * Time.deltaTime);
            }
    
            // Apply water current
            if (followWaterCurrent && !disableCurrentFlow) {
                Vector3 currentDir = GetWaterCurrentDirection(transform.position);
                smoothedPosition += currentSpeedMultiplier * Time.deltaTime * currentDir;
            }
           
            transform.position = smoothedPosition;
            transform.rotation = smoothedRotation;
        }
    
        private static bool searchFailureLogged;

        // Search from the surface's rest level, never from the object's own height: a failed
        // search used to return the object's height, so the target (height + verticalOffset)
        // chased itself and every floater drifted by verticalOffset * positionLerpSpeed per second.
        private float GetWaterHeight(Vector3 worldPos) {
            worldPos.y = targetSurface.transform.position.y;
            searchParams.startPositionWS = (float3)worldPos;
            searchParams.targetPositionWS = (float3)worldPos;
            searchParams.includeDeformation = includeDeformers;
            searchParams.maxIterations = 8;
            searchParams.error = 0.01f;
            searchParams.excludeSimulation = false;

            if (targetSurface.ProjectPointOnWaterSurface(searchParams, out searchResult))
                return searchResult.projectedPositionWS.y;

            if (!searchFailureLogged && Application.isPlaying) {
                searchFailureLogged = true;
                Debug.LogWarning($"CRANE_SIMPLE_FLOAT_SEARCH_FAILED object={name}; " +
                                 "floaters hold the surface's rest level");
            }
            return worldPos.y;
        }
    
        private Vector3 GetWaterCurrentDirection(Vector3 worldPos) {
            searchParams.startPositionWS = (float3)worldPos;
            searchParams.targetPositionWS = (float3)worldPos;
            searchParams.includeDeformation = includeDeformers;
            searchParams.maxIterations = 4;
            searchParams.error = 0.01f;
            searchParams.excludeSimulation = false;
    
            if (targetSurface.ProjectPointOnWaterSurface(searchParams, out searchResult))
                return ((Vector3)searchResult.currentDirectionWS).normalized;
    
            return Vector3.zero;
        }
    
        private void OnCollisionEnter(Collision collision) {
            if (!useRigidbodyForCollision) return;
    
            if (((1 << collision.gameObject.layer) & obstacleLayers) != 0) {
                disableCurrentFlow = true;
                followWaterCurrent = false;
            }
        }
    
        private void OnCollisionExit(Collision collision) {
            if (!useRigidbodyForCollision) return;
    
            if (((1 << collision.gameObject.layer) & obstacleLayers) != 0)
                disableCurrentFlow = false;
        }
    
        private void OnDrawGizmos() { // Called by Unity to draw debug visuals (Gizmos) in the Scene view
            if (!showGizmos) return;       
            Gizmos.color = Color.cyan;
    
            // Define local positions for the boat’s key points (relative to its center)
            Vector3 localBow = transform.forward * (length / 2f); 
            Vector3 localStern = -transform.forward * (length / 2f);
            Vector3 localLeft = -transform.right * (width / 2f);
            Vector3 localRight = transform.right * (width / 2f);
    
            // Convert local positions to world space (actual positions in the scene)
            Vector3 bowPoint = transform.position + localBow;  
            Vector3 sternPoint = transform.position + localStern;
            Vector3 leftPoint = transform.position + localLeft;  
            Vector3 rightPoint = transform.position + localRight;
    
            // Draw small spheres at each of the measurement points for visual reference
            Gizmos.DrawSphere(bowPoint, 0.05f);  
            Gizmos.DrawSphere(sternPoint, 0.05f);
            Gizmos.DrawSphere(leftPoint, 0.05f); 
            Gizmos.DrawSphere(rightPoint, 0.05f);
    
            // Draw lines connecting the measurement points to visualize boat shape
            Gizmos.DrawLine(bowPoint, sternPoint);
            Gizmos.DrawLine(leftPoint, rightPoint);
        }
    }
    
    #if UNITY_EDITOR
    
    [CustomEditor(typeof(SimpleFloat))]
    public class SimpleFloatEditor : Editor {
        public override void OnInspectorGUI() {
            DrawDefaultInspector();
    
            SimpleFloat obj = (SimpleFloat)target;
    
            if (obj.useRigidbodyForCollision) {
                Collider[] colliders = obj.GetComponents<Collider>(); // Get all colliders
    
                bool hasValidCollider = colliders.Any(c =>
                    c is BoxCollider || // Valid box collider
                    c is SphereCollider || // Valid sphere collider
                    c is CapsuleCollider || // Valid capsule collider
                    (c is MeshCollider mc && mc.convex) // Valid convex mesh collider
                );
    
                if (!hasValidCollider) {
                    EditorGUILayout.HelpBox(
                        "This object requires a Basic Collider (Box,Sphere etc.) to use Rigidbody for collisions. Please add a Collider manually.",
                        MessageType.Warning
                    );
                }
            }
        }
    }
    #endif
}