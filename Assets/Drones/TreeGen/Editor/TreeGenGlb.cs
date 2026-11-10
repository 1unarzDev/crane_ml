// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEditor.AssetImporters;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Drones.TreeGenImport.Editor
{
    /// <summary>Minimal reader for TreeGen's own glTF 2.0 binary files (Tools/TreeGen/treegen/glb.py): one scene,
    /// meshes "bark" and "leaves", one triangle primitive each, float32 POSITION/NORMAL/TEXCOORD_0, uint32 indices.
    /// It is not a general glTF importer. Frame: files store (-x_u, y_u, z_u); this reader negates X (like Unity's
    /// OBJ importer and glTFast), flips the winding to compensate for the mirror, and flips V (glTF UV origin is
    /// top-left), so the mesh lands in TreeGen's "unity" frame and matches bounds_unity_local.</summary>
    public static class TreeGenGlb
    {
        [Serializable] class GAcc { public int bufferView = -1; public int byteOffset; public int componentType; public int count; public string type; }
        [Serializable] class GView { public int buffer; public int byteOffset; public int byteLength; }
        [Serializable] class GAttr { public int POSITION = -1; public int NORMAL = -1; public int TEXCOORD_0 = -1; }
        [Serializable] class GPrim { public GAttr attributes; public int indices = -1; public int material = -1; public int mode = 4; }
        [Serializable] class GMesh { public string name; public GPrim[] primitives; }
        [Serializable] class GRoot { public GAcc[] accessors; public GView[] bufferViews; public GMesh[] meshes; }

        public class Part { public string name; public Vector3[] pos, nrm; public Vector2[] uv; public int[] tri; }

        public static List<Part> Read(string path)
        {
            byte[] d = File.ReadAllBytes(path);
            if (d.Length < 28 || BitConverter.ToUInt32(d, 0) != 0x46546C67u || BitConverter.ToUInt32(d, 4) != 2u)
                throw new InvalidDataException("not a glTF 2.0 binary: " + path);
            int jl = BitConverter.ToInt32(d, 12);
            if (BitConverter.ToUInt32(d, 16) != 0x4E4F534Au) throw new InvalidDataException("first glb chunk is not JSON: " + path);
            var g = JsonUtility.FromJson<GRoot>(Encoding.UTF8.GetString(d, 20, jl));
            int binHdr = 20 + jl; int bl = BitConverter.ToInt32(d, binHdr);
            if (BitConverter.ToUInt32(d, binHdr + 4) != 0x004E4942u) throw new InvalidDataException("second glb chunk is not BIN: " + path);
            int bin = binHdr + 8;
            var parts = new List<Part>();
            if (g.meshes == null) return parts;
            foreach (var m in g.meshes)
            {
                if (m.primitives == null || m.primitives.Length == 0) continue;
                var pr = m.primitives[0];
                if (pr.mode != 4) throw new InvalidDataException("only triangle primitives are supported");
                float[] P = Floats(d, bin, bl, g, pr.attributes.POSITION, "VEC3");
                float[] N = pr.attributes.NORMAL >= 0 ? Floats(d, bin, bl, g, pr.attributes.NORMAL, "VEC3") : null;
                float[] T = pr.attributes.TEXCOORD_0 >= 0 ? Floats(d, bin, bl, g, pr.attributes.TEXCOORD_0, "VEC2") : null;
                uint[] I = Indices(d, bin, bl, g, pr.indices);
                int nv = P.Length / 3; var part = new Part { name = m.name, pos = new Vector3[nv], nrm = new Vector3[nv], uv = new Vector2[nv], tri = new int[I.Length] };
                for (int i = 0; i < nv; i++)
                {
                    part.pos[i] = new Vector3(-P[3 * i], P[3 * i + 1], P[3 * i + 2]);
                    part.nrm[i] = N != null ? new Vector3(-N[3 * i], N[3 * i + 1], N[3 * i + 2]) : Vector3.up;
                    part.uv[i] = T != null ? new Vector2(T[2 * i], 1f - T[2 * i + 1]) : Vector2.zero;
                }
                for (int t = 0; t + 2 < I.Length; t += 3) { part.tri[t] = (int)I[t]; part.tri[t + 1] = (int)I[t + 2]; part.tri[t + 2] = (int)I[t + 1]; }
                parts.Add(part);
            }
            return parts;
        }

        static int Comps(string type) { return type == "SCALAR" ? 1 : type == "VEC2" ? 2 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : throw new InvalidDataException("accessor type " + type); }

        static float[] Floats(byte[] d, int bin, int bl, GRoot g, int ai, string want)
        {
            var a = g.accessors[ai]; if (a.componentType != 5126 || a.type != want) throw new InvalidDataException("expected float32 " + want);
            var v = g.bufferViews[a.bufferView]; int n = a.count * Comps(a.type); int off = v.byteOffset + a.byteOffset;
            if (off + n * 4 > bl) throw new InvalidDataException("accessor out of range");
            var r = new float[n]; Buffer.BlockCopy(d, bin + off, r, 0, n * 4); return r;
        }

        static uint[] Indices(byte[] d, int bin, int bl, GRoot g, int ai)
        {
            var a = g.accessors[ai]; var v = g.bufferViews[a.bufferView]; int off = v.byteOffset + a.byteOffset; var r = new uint[a.count];
            if (a.componentType == 5125) { if (off + a.count * 4 > bl) throw new InvalidDataException("index accessor out of range"); Buffer.BlockCopy(d, bin + off, r, 0, a.count * 4); }
            else if (a.componentType == 5123) { for (int i = 0; i < a.count; i++) r[i] = BitConverter.ToUInt16(d, bin + off + 2 * i); }
            else throw new InvalidDataException("index component type " + a.componentType);
            return r;
        }

        /// <summary>One Mesh: submesh 0 = bark, submesh 1 = leaves (only when present).</summary>
        public static Mesh BuildMesh(List<Part> parts, string name)
        {
            Part bark = parts.Find(p => p.name == "bark"), leaves = parts.Find(p => p.name == "leaves");
            var order = new List<Part>(); if (bark != null) order.Add(bark); if (leaves != null) order.Add(leaves);
            var V = new List<Vector3>(); var N = new List<Vector3>(); var U = new List<Vector2>(); var subs = new List<int[]>();
            foreach (var p in order)
            {
                int o = V.Count; V.AddRange(p.pos); N.AddRange(p.nrm); U.AddRange(p.uv);
                var t = new int[p.tri.Length]; for (int i = 0; i < t.Length; i++) t[i] = p.tri[i] + o; subs.Add(t);
            }
            var m = new Mesh { name = name, indexFormat = V.Count > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            m.SetVertices(V); m.SetNormals(N); m.SetUVs(0, U);
            m.subMeshCount = Mathf.Max(1, subs.Count);
            for (int i = 0; i < subs.Count; i++) m.SetTriangles(subs[i], i, false);
            m.RecalculateBounds();
            if (V.Count > 0) m.RecalculateTangents();
            return m;
        }

        public static Mesh Load(string path) { return BuildMesh(Read(path), Path.GetFileNameWithoutExtension(path)); }
    }
}
