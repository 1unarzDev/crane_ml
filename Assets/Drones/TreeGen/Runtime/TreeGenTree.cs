// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using UnityEngine;

namespace Crane.Drones.TreeGenImport
{
    /// <summary>One TreeGen segment (capsule or soft obstacle) in the tree-local Unity frame (Tools/TreeGen/CONTRACT.md section 6).</summary>
    [Serializable]
    public struct TreeGenSegment
    {
        public Vector3 a, b; public float radius; public bool dead;
    }

    /// <summary>Metadata on the root of every imported TreeGen prefab. Ground truth stays with the prefab:
    /// hard capsules are CapsuleColliders under "Colliders" (Environment layer); soft obstacles (thin twigs,
    /// diameter 4 mm up to the hard threshold) are listed here for triggers, penalties or LiDAR labels.</summary>
    public class TreeGenTree : MonoBehaviour
    {
        public int treeId;
        public string species;
        public string variant;
        public string skeletonHash;
        public string source;            // library or stand directory the tree came from
        public float heightM;
        public float dbhM;               // -1 if not defined (logs, trees under 1.3 m)
        public Vector3 boundsMin, boundsMax; // bounds_unity_local from the manifest
        public Vector3 standPosition;    // stand-local root position (stand imports), zero for library trees
        public int[] barkTris = new int[3], leafTris = new int[3];
        public TreeGenSegment[] softObstacles = new TreeGenSegment[0];
        public int capsuleCount;
    }
}
