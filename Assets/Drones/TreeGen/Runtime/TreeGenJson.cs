// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using UnityEngine;

namespace Crane.Drones.TreeGenImport
{
    // JsonUtility-friendly mirrors of the TreeGen schema-1 files (Tools/TreeGen/CONTRACT.md). Unknown keys are ignored.
    [Serializable] public class TgV3 { public float x, y, z; public Vector3 V => new Vector3(x, y, z); }

    [Serializable]
    public class TgCapsule
    {
        public TgV3 a, b, center, axis; public float radius, height; public int node_a, node_b; public bool dead;
    }

    [Serializable] public class TgFoliage { public TgV3 center; public float radius; }

    [Serializable]
    public class TgColliders
    {
        public string schema; public int schema_version; public int tree_id; public string species, skeleton_hash;
        public TgCapsule[] capsules = new TgCapsule[0];
        public TgCapsule[] soft_obstacles = new TgCapsule[0];
        public TgFoliage[] foliage_volumes = new TgFoliage[0];
    }

    [Serializable] public class TgBounds { public float[] min = new float[3], max = new float[3]; }

    [Serializable]
    public class TgMetrics { public float height_m, dbh_m, crown_base_m, crown_ratio, crown_radius_m; public int nodes, dead_nodes, max_order; }

    [Serializable] public class TgTris { public int bark_tris, leaf_tris; }
    [Serializable] public class TgTriCounts { public TgTris lod0, lod1, lod2; }

    [Serializable]
    public class TgTreeEntry
    {
        public int tree_id; public string species, variant, dir, skeleton_hash, params_hash;
        public float[] position_unity = new float[3]; public float yaw_deg;
        public TgBounds bounds_unity_local; public TgMetrics metrics;
        public int n_capsules, n_soft_obstacles, start_season;
    }

    [Serializable]
    public class TgManifest
    {
        public string schema; public int schema_version; public string generator, generator_version; public int forest_seed;
        public TgTreeEntry[] trees = new TgTreeEntry[0];
    }

    public static class TgJson
    {
        /// <summary>Python's json writes NaN for undefined DBH; JsonUtility cannot parse it. Map NaN to -1.</summary>
        public static string Sanitize(string s) { return s.Replace("NaN", "-1").Replace("Infinity", "1e30"); }
        public static T Parse<T>(string text) { return JsonUtility.FromJson<T>(Sanitize(text)); }
        public static Vector3 V(float[] a) { return a != null && a.Length >= 3 ? new Vector3(a[0], a[1], a[2]) : Vector3.zero; }
    }
}
