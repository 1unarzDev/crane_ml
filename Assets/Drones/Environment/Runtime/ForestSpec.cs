using System;
using UnityEngine;

namespace Crane.Drones.Environment
{
    /// <summary>Parameters of one procedurally generated forest. Same spec + same seed => identical forest.</summary>
    [CreateAssetMenu(menuName = "CRANE/Drone Forest Spec")]
    public class ForestSpec : ScriptableObject
    {
        [Header("Identity")]
        public int seed = 1337;
        public string lightingPreset = "GoldenHour"; // GoldenHour | Overcast | Dusk

        [Header("Stand structure")]
        [Tooltip("Square side length in metres (terrain is centred on the origin).")]
        public float areaSize = 140f;
        [Tooltip("Overstory stems >= 8 cm DBH per hectare.")]
        public float stemDensityPerHa = 450f;
        [Tooltip("Understory saplings (2-5 m) per hectare.")]
        public float saplingDensityPerHa = 380f;
        public float dbhMeanCm = 26f;
        [Tooltip("Sigma of ln(DBH).")]
        public float dbhSigma = 0.45f;
        [Tooltip("Size dependent spacing: dmin = spacingBase + spacingK * (r_i + r_j).")]
        public float spacingK = 7f;
        public float spacingBase = 1.0f;
        [Range(0, 1)] public float clustering = 0.5f;
        [Range(0, 1)] public float coniferFraction = 0.45f;
        [Range(0, 0.4f)] public float deadFraction = 0.07f;
        [Range(0, 1)] public float canopyOpenness = 0.25f;
        public int clearingCount = 3;
        public Vector2 clearingRadiusRange = new Vector2(9f, 16f);

        [Header("Terrain")]
        [Range(0, 30)] public float slopeDeg = 6f;
        [Range(0, 3)] public float terrainRoughness = 1f;

        [Header("Flight band (1-4 m)")]
        [Tooltip("Height of the lowest live limb (m).")]
        public float branchHeightMin = 1.2f;
        [Tooltip("Multiplier on thin dead-twig whorls per metre of trunk in the 0.8-5 m band.")]
        [Range(0, 3)] public float lowBranchDensity = 1.0f;
        public float lowTwigMaxLength = 1.7f;
        public float corridorClearRadius = 1.3f;
        public float trailWidth = 2.4f;

        [Header("Ground layers")]
        [Range(0, 3)] public float undergrowthDensity = 1f;
        [Range(0, 3)] public float fernDensity = 1f;
        public int logCount = 14;
        public int rockCount = 26;

        [Header("Search targets")]
        [Range(0, 3)] public int targetCount = 3;
        [Tooltip("0 = fully exposed, 1 = heavily covered by litter, sticks, ferns and logs.")]
        [Range(0, 1)] public float targetOcclusion = 0.6f;
        public Vector2 targetDistanceFromSpawn = new Vector2(22f, 60f);

        public string ToJson() { return JsonUtility.ToJson(this, true); }
        public static ForestSpec FromJson(string json)
        {
            var s = CreateInstance<ForestSpec>();
            JsonUtility.FromJsonOverwrite(json, s);
            return s;
        }
    }
}
