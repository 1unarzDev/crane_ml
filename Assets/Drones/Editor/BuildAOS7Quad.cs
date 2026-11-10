using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;
using Crane.Drones;

namespace Crane.Drones.EditorTools
{
    /// <summary>
    /// Deterministic builder for Assets/Drones/Models/AOS7Quad.prefab. Run:
    ///   unity-drones LOG -executeMethod Crane.Drones.EditorTools.BuildAOS7Quad.Build -quit
    /// Units meters. Body frame in Unity axes: +X right, +Y up, +Z forward (FLU: x=fwd=+Z, y=left=-X, z=up=+Y).
    /// Dimensions: AOS 7 V5 wheelbase 313 mm = sqrt(254^2+184^2) (iFlight spec: 254 x 184 x 36 mm frame, 8 mm arms,
    /// 2 mm top / 2.5 mm bottom plates, 19x19 motor pattern, 20x20/30x30 stack).
    /// </summary>
    public static class BuildAOS7Quad
    {
        const float MM = 0.001f;
        public const string PrefabPath = "Assets/Drones/Models/AOS7Quad.prefab";
        const string MeshDir = "Assets/Drones/Models/Meshes";

        // Layout constants (m)
        const float MotorX = 0.092f, MotorZ = 0.127f;          // 184 x 254 mm motor rectangle -> 313.6 mm diagonal
        const float ArmY0 = -0.020f, ArmY1 = -0.012f;           // 8 mm arms
        const float BotY0 = -0.0225f, BotY1 = -0.0200f;          // 2.5 mm bottom plate
        const float TopY0 = 0.0115f, TopY1 = 0.0135f;            // 2 mm top plate
        const float PropPlaneY = 0.0125f;                        // rotor hub / prop plane height
        const float StackY = 0.0165f;                            // top of PCB carriers above top plate

        static Dictionary<string, Material> M;

        // ArduPilot Quad-X: M1 FR CCW, M2 RL CCW, M3 FL CW, M4 RR CW.
        static readonly string[] RotorNames = { "FR", "RL", "FL", "RR" };
        static readonly int[] RotorSx = { 1, -1, -1, 1 };
        static readonly int[] RotorSz = { 1, -1, 1, -1 };
        static readonly RotorSpin[] RotorDir = { RotorSpin.CCW, RotorSpin.CCW, RotorSpin.CW, RotorSpin.CW };

        [MenuItem("CRANE/Drones/Build AOS7 Quad")]
        public static void Build()
        {
            Directory.CreateDirectory(MeshDir);
            DroneTextures.GenerateAll();
            AssetDatabase.Refresh();
            M = DroneMaterials.BuildAll();

            var root = new GameObject("AOS7Quad");
            var rig = root.AddComponent<DroneVisualRig>();
            var vis = Child(root, "Visual");

            Part(vis, "Frame", FrameMesh());
            Part(vis, "ESC_HolybroTekko32_65A", EscMesh());
            Part(vis, "CubeOrange_MiniCarrier", CubeMesh());
            Part(vis, "JetsonOrinNano_Carrier", JetsonMesh());
            Part(vis, "Microhard_pMDDL900", RadioMesh());
            Part(vis, "FrSky_RC_Receiver", RxMesh());
            Part(vis, "Antennas", AntennaMesh());
            Part(vis, "UnitreeL2_Lidar", LidarMesh());
            Part(vis, "OAK-D_S2", OakMesh());
            Part(vis, "Battery_6S_LiPo", BatteryMesh());
            Part(vis, "Wiring", WiringMesh());

            // Motors (static stator/flange) and rotors (spinning bell + prop)
            var motors = Child(vis, "Motors_TMotorF90");
            var statorMesh = Save2(MotorBaseMesh(), "MotorBase");
            var rotorCw = Save2(RotorMesh(false), "RotorCW");
            var rotorCcw = Save2(RotorMesh(true), "RotorCCW");
            var bladeCw = Save2(BladeOnlyMesh(false), "PropBladesCW");
            var bladeCcw = Save2(BladeOnlyMesh(true), "PropBladesCCW");
            var disc = Save2(BlurDiscMesh(), "PropBlurDisc");
            var rotorsT = Child(vis, "Rotors");
            var list = new List<DroneVisualRig.Rotor>();
            for (int i = 0; i < 4; i++)
            {
                float x = RotorSx[i] * MotorX, z = RotorSz[i] * MotorZ;
                var st = Child(motors, "MotorBase_M" + (i + 1) + "_" + RotorNames[i]);
                st.transform.localPosition = new Vector3(x, 0, z);
                Attach(st, statorMesh, "MotorBase");

                bool ccw = RotorDir[i] == RotorSpin.CCW;
                var hub = Child(rotorsT, "Rotor_M" + (i + 1) + "_" + RotorNames[i] + "_" + (ccw ? "CCW" : "CW"));
                hub.transform.localPosition = new Vector3(x, PropPlaneY, z);
                // the rotor mesh includes bell + hardware + (solid) blades; hide solid blades via separate renderer
                var bell = Child(hub, "BellAndHub"); Attach(bell, ccw ? rotorCcw : rotorCw, "Rotor");
                var bo = Child(hub, "Blades_Solid"); AttachMat(bo, ccw ? bladeCcw : bladeCw, "Prop_Plastic");
                var bf = Child(hub, "Blades_Fade"); AttachMat(bf, ccw ? bladeCcw : bladeCw, "Prop_BladeFade");
                var bd = Child(hub, "BlurDisc"); AttachMat(bd, disc, "Prop_BlurDisc");
                bd.transform.localPosition = new Vector3(0, 0.0016f, 0);
                var r = new DroneVisualRig.Rotor
                {
                    name = hub.name, motorIndex = i + 1, hub = hub.transform, spin = RotorDir[i],
                    bladesOpaque = bo.GetComponent<MeshRenderer>(), bladesFade = bf.GetComponent<MeshRenderer>(), blurDisc = bd.GetComponent<MeshRenderer>()
                };
                bf.GetComponent<MeshRenderer>().enabled = false; bd.GetComponent<MeshRenderer>().enabled = false;
                list.Add(r);
            }
            rig.rotors = list.ToArray();

            // Marker / mount frames (Unity axes, +Z = sensor forward). Names carry FLU coordinates in mm.
            var marks = Child(root, "Frames");
            rig.comMarker = Marker(marks, "CoM", new Vector3(0, -0.0105f, -0.004f));
            rig.cameraMount = Marker(marks, "Mount_Camera_OAKD_RGB", new Vector3(0, -0.00175f, 0.1195f));
            rig.lidarMount = Marker(marks, "Mount_Lidar_UnitreeL2", new Vector3(0, 0.0525f + 0.0375f, 0.0f));
            rig.imuMount = Marker(marks, "Mount_IMU_CubeOrange", new Vector3(0, StackY + 0.0008f + 0.011f, -0.040f));
            Marker(marks, "Mount_RC_Rx", new Vector3(0.024f, StackY + 0.004f, -0.074f));
            Marker(marks, "Mount_Battery", new Vector3(0, -0.0430f, -0.005f));
            AddRotorAxisMarkers(marks);

            // LEDs: 4 arm nav lights (front white, rear red) + FC, Jetson and ESC indicators
            var ledRoot = Child(vis, "LEDs"); var leds = new List<Renderer>();
            for (int i = 0; i < 4; i++)
            {
                float x = RotorSx[i] * (MotorX - 0.018f), z = RotorSz[i] * (MotorZ - 0.035f);
                leds.Add(Led(ledRoot, "LED_Arm_M" + (i + 1), new Vector3(x, ArmY0 - 0.0012f, z), RotorSz[i] > 0 ? "LED_White" : "LED_Red", true));
            }
            leds.Add(Led(ledRoot, "LED_FC_Status", new Vector3(0.0125f, StackY + 0.0008f + 0.0235f, -0.052f), "LED_Blue", false));
            leds.Add(Led(ledRoot, "LED_Jetson_Power", new Vector3(-0.022f, StackY + 0.0021f, 0.018f), "LED_Green", false));
            leds.Add(Led(ledRoot, "LED_ESC_Status", new Vector3(0.0185f, -0.0079f, 0.019f), "LED_Green", false));
            rig.statusLeds = leds.ToArray();
            rig.ResetVisuals();

            PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            UnityEngine.Object.DestroyImmediate(root);
            AssetDatabase.SaveAssets(); AssetDatabase.Refresh();
            Debug.Log("[BuildAOS7Quad] wrote " + PrefabPath);
        }

        // ------------------------------------------------------------ scene helpers
        static GameObject Child(GameObject p, string n) { var g = new GameObject(n); g.transform.SetParent(p.transform, false); return g; }
        static Transform Marker(GameObject p, string n, Vector3 pos) { var g = Child(p, n); g.transform.localPosition = pos; return g.transform; }

        static void AddRotorAxisMarkers(GameObject marks)
        {
            for (int i = 0; i < 4; i++)
                Marker(marks, "RotorAxis_M" + (i + 1) + "_" + RotorNames[i], new Vector3(RotorSx[i] * MotorX, PropPlaneY, RotorSz[i] * MotorZ));
        }

        static Mesh Save(DroneMeshBuilder b, string name)
        {
            var m = b.ToMesh(name); string path = MeshDir + "/" + name + ".asset";
            var ex = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if (ex == null) { AssetDatabase.CreateAsset(m, path); return m; }
            EditorUtility.CopySerialized(m, ex); UnityEngine.Object.DestroyImmediate(m); return ex;
        }

        // Records mesh -> materials order for Attach()
        static readonly Dictionary<Mesh, List<string>> _mats = new Dictionary<Mesh, List<string>>();
        static Mesh Save2(DroneMeshBuilder b, string name) { var m = Save(b, name); _mats[m] = new List<string>(b.Order); return m; }

        static void Part(GameObject p, string name, DroneMeshBuilder b)
        {
            var m = Save2(b, name.Replace("-", "_")); var g = Child(p, name); Attach(g, m, name);
        }

        static void Attach(GameObject g, Mesh m, string label)
        {
            if (!_mats.ContainsKey(m)) throw new Exception("mesh not registered " + label);
            g.AddComponent<MeshFilter>().sharedMesh = m;
            var mr = g.AddComponent<MeshRenderer>(); var arr = new Material[_mats[m].Count];
            for (int i = 0; i < arr.Length; i++) arr[i] = M[_mats[m][i]];
            mr.sharedMaterials = arr;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
        }

        static void AttachMat(GameObject g, Mesh m, string mat)
        {
            g.AddComponent<MeshFilter>().sharedMesh = m;
            var mr = g.AddComponent<MeshRenderer>(); mr.sharedMaterial = M[mat];
            if (mat != "Prop_Plastic") mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        static Renderer Led(GameObject p, string name, Vector3 pos, string mat, bool down)
        {
            var b = new DroneMeshBuilder(); b.Mat(mat);
            b.Sphere(Vector3.zero, 0.0023f, 12, 8, 0.7f);
            var m = Save2(b, name); var g = Child(p, name); g.transform.localPosition = pos; Attach(g, m, name);
            return g.GetComponent<MeshRenderer>();
        }

        // ------------------------------------------------------------ primitives
        static void Screw(DroneMeshBuilder b, Vector3 pos, bool up, float r = 0.0027f)
        {
            
            b.Mat("Steel");
            b.Push(pos, up ? Quaternion.identity : Quaternion.Euler(180, 0, 0));
            b.Cyl(r, 0, 0.0016f, 14, 0.0004f);
            b.Mat("Plastic_Black"); b.Cyl(r * 0.45f, 0.0013f, 0.00165f, 6, 0, true);
            b.Pop();
        }

        static void Standoff(DroneMeshBuilder b, float x, float z, float y0, float y1, float r = 0.0025f)
        {
            b.Mat("Aluminum_Brushed");
            b.Push(new Vector3(x, 0, z), Quaternion.identity); b.Cyl(r, y0, y1, 6, 0.0003f, true); b.Pop();
        }

        static Vector2 P2(Vector3 v) { return new Vector2(v.x, v.z); }

        // ------------------------------------------------------------ frame
        static DroneMeshBuilder FrameMesh()
        {
            var b = new DroneMeshBuilder();
            b.Mat("Carbon");
            b.Prism(DroneMeshBuilder.RectPoly(0, -0.001f, 0.068f, 0.182f, 0.020f), BotY0, BotY1, 0.0004f);
            b.Prism(DroneMeshBuilder.RectPoly(0, -0.0055f, 0.068f, 0.183f, 0.018f), TopY0, TopY1, 0.0004f);
            // arms: tapered with rounded motor pads
            for (int i = 0; i < 4; i++)
            {
                float sx = RotorSx[i], sz = RotorSz[i];
                b.Prism(ArmPoly(new Vector2(sx * 0.016f, sz * 0.040f), new Vector2(sx * MotorX, sz * MotorZ), 0.030f, 0.0168f), ArmY0, ArmY1, 0.0008f);
            }
            // lidar mounting plate (carbon, 2 mm)
            b.Prism(DroneMeshBuilder.RectPoly(0, 0, 0.082f, 0.078f, 0.012f), 0.0500f, 0.0520f, 0.0004f);
            // camera shelf and cage
            b.Mat("Aluminum_Anodized");
            b.Prism(DroneMeshBuilder.RectPoly(0, 0.1025f, 0.106f, 0.036f, 0.004f), -0.0185f, -0.0165f, 0.0004f);
            b.Prism(DroneMeshBuilder.RectPoly(-0.0525f, 0.1025f, 0.003f, 0.036f, 0), -0.0185f, 0.0105f, 0.0004f);
            b.Prism(DroneMeshBuilder.RectPoly(0.0525f, 0.1025f, 0.003f, 0.036f, 0), -0.0185f, 0.0105f, 0.0004f);
            b.Prism(DroneMeshBuilder.RectPoly(0, 0.0875f, 0.106f, 0.004f, 0), -0.0185f, 0.0105f, 0.0004f);   // rear crossbar
            // standoffs between bottom plate and top plate
            float[] zs = { -0.083f, 0.0f, 0.078f };
            foreach (float z in zs) foreach (float sx in new[] { -1f, 1f }) Standoff(b, sx * 0.027f, z, BotY1, TopY0);
            foreach (float sx in new[] { -1f, 1f }) foreach (float sz in new[] { -1f, 1f }) Standoff(b, sx * 0.01525f, sz * 0.01525f, BotY1, TopY0, 0.0022f);
            // lidar standoffs from the top plate to the lidar plate
            foreach (float sx in new[] { -1f, 1f }) foreach (float sz in new[] { -1f, 1f })
                { b.Mat("Aluminum_Anodized"); b.Push(new Vector3(sx * 0.0335f, 0, sz * 0.032f), Quaternion.identity); b.Cyl(0.003f, TopY1, 0.0500f, 8, 0.0004f, true); b.Pop(); }
            // screws
            foreach (float z in zs) foreach (float sx in new[] { -1f, 1f }) { Screw(b, new Vector3(sx * 0.027f, TopY1, z), true); Screw(b, new Vector3(sx * 0.027f, BotY0, z), false); }
            foreach (float sx in new[] { -1f, 1f }) foreach (float sz in new[] { -1f, 1f })
                { Screw(b, new Vector3(sx * 0.0335f, 0.0520f, sz * 0.032f), true); Screw(b, new Vector3(sx * 0.01525f, TopY1, sz * 0.01525f), true); Screw(b, new Vector3(sx * 0.01525f, BotY0, sz * 0.01525f), false); }
            // arm clamping screws
            for (int i = 0; i < 4; i++)
            {
                float sx = RotorSx[i], sz = RotorSz[i];
                Vector2 s = new Vector2(sx * 0.016f, sz * 0.040f), e = new Vector2(sx * MotorX, sz * MotorZ); Vector2 d = (e - s).normalized;
                for (int k = 0; k < 3; k++) { Vector2 p = s + d * (0.025f + k * 0.0235f); Screw(b, new Vector3(p.x, ArmY1, p.y), true, 0.0022f); }
            }
            return b;
        }

        static List<Vector2> ArmPoly(Vector2 s, Vector2 e, float w0, float rE)
        {
            Vector2 t = (e - s).normalized, p = new Vector2(-t.y, t.x); var l = new List<Vector2>();
            l.Add(s + p * (w0 * 0.5f)); l.Add(e + p * rE);
            for (int k = 1; k < 10; k++) { float a = Mathf.PI * k / 10f; l.Add(e + (p * Mathf.Cos(a) + t * Mathf.Sin(a)) * rE); }
            l.Add(e - p * rE); l.Add(s - p * (w0 * 0.5f)); return l;
        }

        // ------------------------------------------------------------ motors
        static DroneMeshBuilder MotorBaseMesh()
        {
            var b = new DroneMeshBuilder();
            float y0 = ArmY1;                      // sits on the arm pad
            b.Mat("Anodized_Red"); b.Cyl(0.0165f, y0, y0 + 0.0022f, 40, 0.0006f);
            b.Mat("Aluminum_Anodized"); b.Cyl(0.0108f, y0 + 0.0022f, y0 + 0.0042f, 32, 0.0004f);
            // 19x19 mounting screws on the flange
            for (int i = 0; i < 4; i++)
            {
                float a = (45 + 90 * i) * Mathf.Deg2Rad; float rr = 0.0095f * Mathf.Sqrt(2f);
                Screw(b, new Vector3(rr * Mathf.Cos(a), y0 + 0.0022f, rr * Mathf.Sin(a)), true, 0.0018f);
            }
            // visible copper windings through the gap between flange and bell
            b.Mat("Copper"); b.Cyl(0.0115f, y0 + 0.0042f, y0 + 0.0062f, 24, 0.0002f);
            return b;
        }

        // Rotor coordinates are hub-local (y=0 at prop plane). Bell spans y -0.0215..0; nut to +0.0102.
        static DroneMeshBuilder RotorMesh(bool ccw)
        {
            var b = new DroneMeshBuilder();
            if (!ccw) { }
            b.Mat("Aluminum_Anodized");
            var prof = new List<Vector2>
            {
                new Vector2(0, -0.0215f), new Vector2(0.0140f, -0.0215f), new Vector2(0.0153f, -0.0202f), new Vector2(0.0167f, -0.0188f),
                new Vector2(0.0167f, -0.0030f), new Vector2(0.0160f, -0.0014f), new Vector2(0.0125f, -0.0010f), new Vector2(0.0100f, 0f), new Vector2(0, 0)
            };
            b.Lathe(prof, 48, false);
            // cooling fins around the lower bell (sub-mm detail)
            b.Mat("Aluminum_Anodized");
            for (int k = 0; k < 12; k++)
            {
                b.Push(Vector3.zero, Quaternion.Euler(0, k * 30f + 15f, 0));
                b.Box(new Vector3(0.0170f, -0.0125f, 0), new Vector3(0.0022f, 0.0102f, 0.0012f), 0.0002f);
                b.Pop();
            }
            // groove rings
            b.Mat("Plastic_Black"); b.Cyl(0.01685f, -0.0165f, -0.0158f, 40, 0);
            // top screws and shaft nut
            for (int k = 0; k < 4; k++)
            {
                float a = (45 + 90 * k) * Mathf.Deg2Rad;
                Screw(b, new Vector3(0.0070f * Mathf.Cos(a), 0.0030f, 0.0070f * Mathf.Sin(a)), true, 0.0017f);
            }
            b.Mat("Prop_Plastic");
            b.Cyl(0.0125f, 0f, 0.0034f, 36, 0.0006f);                 // prop hub
            b.Mat("Anodized_Red"); b.Cyl(0.0058f, 0.0034f, 0.0098f, 6, 0.0004f, true);   // locknut
            b.Mat("Steel"); b.Cyl(0.0022f, 0.0098f, 0.0112f, 12, 0.0004f);
            return b;
        }

        static DroneMeshBuilder BladeOnlyMesh(bool ccw)
        {
            var b = new DroneMeshBuilder(); b.Mat("Prop_Plastic");
            if (ccw) b.PushMirrorX();
            for (int k = 0; k < 3; k++)
            {
                b.Push(Vector3.zero, Quaternion.Euler(0, k * 120f, 0));
                AddBlade(b); b.Pop();
            }
            if (ccw) b.Pop();
            return b;
        }

        // 7 inch (177.8 mm) diameter, 3 blades, twisted airfoil-like blade
        static void AddBlade(DroneMeshBuilder b)
        {
            const int NI = 14; float r0 = 0.0105f, R = 0.0889f;
            float[] cs = { 0f, 0.04f, 0.18f, 0.5f, 0.82f, 1f, 0.82f, 0.5f, 0.18f, 0.04f };
            float[] hs = { 0f, 0.55f, 0.95f, 0.8f, 0.32f, 0f, -0.14f, -0.26f, -0.2f, -0.08f };
            int nj = cs.Length; var P = new Vector3[NI, nj];
            for (int i = 0; i < NI; i++)
            {
                float u = (float)i / (NI - 1), r = Mathf.Lerp(r0, R, u);
                float chord = (0.0125f + 0.0075f * Mathf.Sin(Mathf.PI * Mathf.Clamp01(u * 0.85f))) * (1f - 0.42f * u * u * u);
                float thick = 0.0030f * (1f - 0.75f * u) + 0.0005f;
                float th = Mathf.Lerp(38f, 13f, Mathf.Pow(u, 0.8f)) * Mathf.Deg2Rad;
                Vector3 d = new Vector3(0, -Mathf.Sin(th), Mathf.Cos(th)), n = new Vector3(0, Mathf.Cos(th), Mathf.Sin(th));
                float rake = 0.0010f * u * u + 0.0006f;
                for (int j = 0; j < nj; j++)
                    P[i, j] = new Vector3(r, 0.0016f + rake, 0) + d * ((cs[j] - 0.3f) * chord) + n * (hs[j] * thick);
            }
            b.Grid(P, true);
        }

        static DroneMeshBuilder BlurDiscMesh()
        {
            var b = new DroneMeshBuilder(); b.Mat("Prop_BlurDisc");
            const int A = 72; float ri = 0.0125f, ro = 0.0889f;
            for (int side = 0; side < 2; side++)
            {
                Vector3 n = side == 0 ? Vector3.up : Vector3.down;
                for (int s = 0; s < A; s++)
                {
                    float a0 = 2 * Mathf.PI * s / A, a1 = 2 * Mathf.PI * (s + 1) / A;
                    int i0 = b.AddVert(new Vector3(ri * Mathf.Cos(a0), 0, ri * Mathf.Sin(a0)), n, new Vector2(0, 0.5f));
                    int i1 = b.AddVert(new Vector3(ro * Mathf.Cos(a0), 0, ro * Mathf.Sin(a0)), n, new Vector2(1, 0.5f));
                    int i2 = b.AddVert(new Vector3(ro * Mathf.Cos(a1), 0, ro * Mathf.Sin(a1)), n, new Vector2(1, 0.5f));
                    int i3 = b.AddVert(new Vector3(ri * Mathf.Cos(a1), 0, ri * Mathf.Sin(a1)), n, new Vector2(0, 0.5f));
                    b.Face(i0, i1, i2); b.Face(i0, i2, i3);
                }
            }
            return b;
        }

        // ------------------------------------------------------------ electronics
        static DroneMeshBuilder EscMesh()
        {
            var b = new DroneMeshBuilder(); float y = -0.0095f;
            b.Mat("Aluminum_Brushed"); b.Prism(DroneMeshBuilder.RectPoly(0, 0, 0.040f, 0.040f, 0.003f), y - 0.0017f, y - 0.0008f, 0.0002f);
            b.Mat("PCB_Black"); b.Prism(DroneMeshBuilder.RectPoly(0, 0, 0.043f, 0.044f, 0.004f), y - 0.0008f, y + 0.0008f, 0.0002f);
            b.Mat("Plastic_Black");
            for (int r = 0; r < 2; r++) for (int c = 0; c < 4; c++)
                    b.Box(new Vector3(-0.0105f + c * 0.007f, y + 0.0013f, 0.007f - r * 0.016f), new Vector3(0.0055f, 0.0010f, 0.0065f), 0.0002f);
            b.Mat("Gold");
            foreach (float sx in new[] { -1f, 1f }) foreach (float sz in new[] { -1f, 1f }) b.Box(new Vector3(sx * 0.0165f, y + 0.0011f, sz * 0.0185f), new Vector3(0.0045f, 0.0006f, 0.0045f), 0f);
            b.Push(new Vector3(0, y + 0.0008f, -0.0105f), Quaternion.Euler(0, 0, 90));
            b.Cyl(0.0055f, -0.0070f, 0.0070f, 24, 0.0005f); b.Pop();      // 1000 uF capacitor lying across the rear
            b.Mat("Aluminum_Brushed"); b.Push(new Vector3(0, y + 0.0008f, -0.0105f), Quaternion.Euler(0, 0, 90)); b.Cyl(0.0042f, 0.0070f, 0.0073f, 18, 0); b.Pop();
            return b;
        }

        static DroneMeshBuilder CubeMesh()
        {
            var b = new DroneMeshBuilder(); float cz = -0.040f, y0 = StackY;
            b.Mat("PCB_Black"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, 0.050f, 0.046f, 0.004f), y0, y0 + 0.0016f, 0.0002f);
            b.Mat("Plastic_White");
            for (int k = 0; k < 5; k++) b.Box(new Vector3(-0.0215f, y0 + 0.0016f + 0.0025f, cz - 0.016f + k * 0.008f), new Vector3(0.0045f, 0.0050f, 0.0065f), 0.0003f);
            for (int k = 0; k < 3; k++) b.Box(new Vector3(0.0215f, y0 + 0.0016f + 0.0025f, cz - 0.010f + k * 0.010f), new Vector3(0.0045f, 0.0050f, 0.0085f), 0.0003f);
            float ys = y0 + 0.0016f;
            b.Mat("Plastic_Black"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, 0.03825f, 0.03825f, 0.002f), ys, ys + 0.0223f, 0.0010f);
            b.Mat("Plastic_DarkGray"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, 0.0300f, 0.0300f, 0.001f), ys + 0.0223f, ys + 0.0229f, 0.0001f);
            b.Mat("Plastic_Orange"); b.Prism(DroneMeshBuilder.RectPoly(0, cz + 0.011f, 0.0240f, 0.0036f, 0), ys + 0.0229f, ys + 0.0233f, 0f);
            b.Mat("Gold"); b.Box(new Vector3(0, ys - 0.0004f, cz), new Vector3(0.030f, 0.0008f, 0.004f), 0f);
            return b;
        }

        static DroneMeshBuilder JetsonMesh()
        {
            var b = new DroneMeshBuilder(); float cz = 0.043f, y0 = StackY;
            b.Mat("PCB_Green"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, 0.056f, 0.080f, 0.004f), y0, y0 + 0.0016f, 0.0002f);
            // module on SO-DIMM style connector
            b.Mat("Plastic_Black"); b.Box(new Vector3(0, y0 + 0.0016f + 0.0015f, cz + 0.002f), new Vector3(0.046f, 0.0030f, 0.0050f), 0.0003f);
            b.Mat("PCB_Green"); b.Prism(DroneMeshBuilder.RectPoly(0, cz + 0.002f, 0.045f, 0.0696f, 0.002f), y0 + 0.0050f, y0 + 0.0066f, 0.0002f);
            // heatsink block with fins
            b.Mat("Aluminum_Anodized"); b.Prism(DroneMeshBuilder.RectPoly(0, cz + 0.002f, 0.044f, 0.060f, 0.002f), y0 + 0.0066f, y0 + 0.0106f, 0.0004f);
            for (int k = 0; k < 11; k++) b.Box(new Vector3(-0.020f + k * 0.004f, y0 + 0.0106f + 0.0055f, cz + 0.002f), new Vector3(0.0016f, 0.0110f, 0.056f), 0.0002f);
            // small fan on top (shroud + blades)
            b.Mat("Plastic_Black"); b.Prism(DroneMeshBuilder.RectPoly(0, cz + 0.002f, 0.040f, 0.040f, 0.003f), y0 + 0.0216f, y0 + 0.0236f, 0.0003f);
            b.Push(new Vector3(0, 0, cz + 0.002f), Quaternion.identity);
            b.Mat("Plastic_DarkGray"); b.Cyl(0.0170f, y0 + 0.0236f, y0 + 0.0240f, 32, 0.0001f);
            b.Mat("Plastic_Black"); b.Cyl(0.0075f, y0 + 0.0240f, y0 + 0.0252f, 20, 0.0008f);
            for (int k = 0; k < 7; k++) { b.Push(Vector3.zero, Quaternion.Euler(0, k * 51.4f, 0)); b.Box(new Vector3(0.0115f, y0 + 0.0246f, 0), new Vector3(0.0100f, 0.0006f, 0.0045f), 0.0001f); b.Pop(); }
            b.Pop();
            // ports on the left edge: 2x USB-A stack and RJ45
            b.Mat("Aluminum_Brushed");
            b.Box(new Vector3(-0.0245f, y0 + 0.0016f + 0.0075f, cz - 0.026f), new Vector3(0.0140f, 0.0150f, 0.0130f), 0.0003f);
            b.Box(new Vector3(-0.0230f, y0 + 0.0016f + 0.0070f, cz - 0.004f), new Vector3(0.0170f, 0.0140f, 0.0160f), 0.0003f);
            b.Mat("Plastic_Black");
            b.Box(new Vector3(-0.0320f, y0 + 0.0016f + 0.0075f, cz - 0.026f), new Vector3(0.0008f, 0.0105f, 0.0090f), 0f);
            b.Box(new Vector3(-0.0318f, y0 + 0.0016f + 0.0070f, cz - 0.004f), new Vector3(0.0008f, 0.0090f, 0.0120f), 0f);
            b.Mat("Gold"); for (int k = 0; k < 20; k++) b.Box(new Vector3(0.0235f, y0 + 0.0016f + 0.0013f, cz + 0.030f - k * 0.0013f), new Vector3(0.0050f, 0.0026f, 0.0008f), 0f);
            return b;
        }

        static DroneMeshBuilder RadioMesh()
        {
            var b = new DroneMeshBuilder(); float cz = -0.081f, y0 = TopY1 + 0.0030f;
            b.Mat("Aluminum_Brushed");
            foreach (float sx in new[] { -1f, 1f }) foreach (float sz in new[] { -1f, 1f }) { b.Push(new Vector3(sx * 0.012f, 0, cz + sz * 0.011f), Quaternion.identity); b.Cyl(0.0022f, TopY1, y0, 8, 0, true); b.Pop(); }
            b.Mat("PCB_Green"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, 0.032f, 0.030f, 0.002f), y0, y0 + 0.0016f, 0.0002f);
            b.Mat("Aluminum_Brushed"); b.Prism(DroneMeshBuilder.RectPoly(0f, cz + 0.002f, 0.024f, 0.020f, 0.001f), y0 + 0.0016f, y0 + 0.0046f, 0.0003f);
            b.Mat("Steel"); b.Prism(DroneMeshBuilder.RectPoly(-0.0055f, cz - 0.0095f, 0.010f, 0.007f, 0), y0 + 0.0016f, y0 + 0.0050f, 0.0003f);
            b.Push(new Vector3(0.011f, y0 + 0.0016f, cz - 0.013f), Quaternion.identity); b.Cyl(0.0015f, 0, 0.0020f, 10, 0); b.Pop();
            b.Push(new Vector3(-0.011f, y0 + 0.0016f, cz - 0.013f), Quaternion.identity); b.Cyl(0.0015f, 0, 0.0020f, 10, 0); b.Pop();
            return b;
        }

        static DroneMeshBuilder RxMesh()
        {
            var b = new DroneMeshBuilder(); float y0 = TopY1;
            b.Mat("Plastic_Black"); b.Box(new Vector3(0.024f, y0 + 0.0028f, -0.074f), new Vector3(0.0165f, 0.0050f, 0.0115f), 0.0006f);
            b.Mat("Plastic_Orange"); b.Box(new Vector3(0.024f, y0 + 0.0056f, -0.074f), new Vector3(0.0120f, 0.0004f, 0.0030f), 0f);
            b.Mat("PCB_Black"); b.Box(new Vector3(0.024f, y0 + 0.0003f, -0.074f), new Vector3(0.0190f, 0.0006f, 0.0125f), 0f);
            return b;
        }

        static DroneMeshBuilder AntennaMesh()
        {
            var b = new DroneMeshBuilder();
            // Microhard pMDDL900 900 MHz whips: SMA bulkhead on the rear bracket plus 95 mm sleeve dipoles, splayed back and out.
            b.Mat("Aluminum_Anodized");
            b.Prism(DroneMeshBuilder.RectPoly(0, -0.1005f, 0.056f, 0.006f, 0.0008f), TopY0 - 0.0005f, TopY1 + 0.0100f, 0.0005f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Vector3 b0 = new Vector3(sx * 0.018f, TopY1 + 0.0065f, -0.1040f);
                Quaternion q = Quaternion.Euler(-62f, 0, 0) * Quaternion.Euler(0, 0, -sx * 12f);
                b.Mat("Gold"); b.Push(b0, Quaternion.Euler(-90f, 0, 0)); b.Cyl(0.0032f, 0, 0.0090f, 6, 0.0003f, true); b.Pop();
                b.Mat("Steel"); b.Push(b0 + new Vector3(0, 0, -0.008f), Quaternion.Euler(-90f, 0, 0)); b.Cyl(0.0040f, 0, 0.0040f, 14, 0.0004f); b.Pop();
                b.Mat("Plastic_Black"); b.Push(b0 + new Vector3(0, 0, -0.012f), q);
                var prof = new List<Vector2> { new Vector2(0, 0), new Vector2(0.0048f, 0), new Vector2(0.0050f, 0.002f), new Vector2(0.0050f, 0.090f), new Vector2(0.0044f, 0.0940f), new Vector2(0.0025f, 0.0965f), new Vector2(0, 0.0975f) };
                b.Lathe(prof, 20, false); b.Pop();
                b.Mat("Wire_Black"); b.Push(b0 + new Vector3(0, 0, -0.012f), q); b.Cyl(0.0053f, 0.0020f, 0.0140f, 20, 0.0003f); b.Pop();   // base boot
            }
            // FrSky receiver T-antenna tips (rigid 28 mm white sleeves) hang below the rear arms
            foreach (float sx in new[] { -1f, 1f })
            {
                b.Mat("Plastic_White");
                Vector3 s = new Vector3(sx * 0.030f, ArmY0 - 0.0015f, -0.112f);
                b.CylBetween(s, s + new Vector3(sx * 0.010f, -0.004f, -0.026f), 0.0014f, 8, 0.0003f);
            }
            return b;
        }

        static DroneMeshBuilder LidarMesh()
        {
            var b = new DroneMeshBuilder(); float y0 = 0.0520f;
            b.Mat("Plastic_White"); b.Prism(DroneMeshBuilder.RoundRectPoly(0, 0, 0.075f, 0.075f, 0.010f, 6), y0, y0 + 0.0225f, 0.0015f);
            b.Mat("Plastic_Black"); b.Prism(DroneMeshBuilder.RoundRectPoly(0, 0, 0.072f, 0.072f, 0.0095f, 6), y0 + 0.0225f, y0 + 0.0238f, 0.0004f);
            b.Mat("Plastic_DarkGray");
            b.Cyl(0.0295f, y0 + 0.0238f, y0 + 0.0262f, 48, 0.0005f);
            // smoked dome hemisphere with scan optics inside
            b.Mat("Glass_Smoked");
            float r = 0.0285f, yb = y0 + 0.0262f; var pr = new List<Vector2> { new Vector2(0.0285f, yb) };
            for (int i = 1; i <= 10; i++) { float a = Mathf.PI * 0.5f * i / 10f; pr.Add(new Vector2(r * Mathf.Cos(a), yb + 0.0300f * Mathf.Sin(a))); }
            b.Lathe(pr, 48, false);
            b.Mat("Plastic_Black"); b.Cyl(0.0085f, yb, yb + 0.0260f, 24, 0.0006f);
            b.Mat("Aluminum_Anodized"); b.Push(new Vector3(0f, yb + 0.0150f, 0f), Quaternion.Euler(0, 0, 40)); b.Box(new Vector3(0, 0, 0), new Vector3(0.0220f, 0.0012f, 0.0200f), 0.0002f); b.Pop();
            b.Mat("LED_Blue"); // tiny status window on base front
            b.Box(new Vector3(0, y0 + 0.0110f, 0.0372f), new Vector3(0.0120f, 0.0012f, 0.0008f), 0f);
            b.Mat("Wire_Black"); b.Box(new Vector3(0, y0 + 0.0050f, -0.0385f), new Vector3(0.0100f, 0.0050f, 0.0060f), 0.0006f);
            return b;
        }

        static DroneMeshBuilder OakMesh()
        {
            var b = new DroneMeshBuilder(); float cy = -0.00175f, zc = 0.1080f; // body z 0.0965..0.1195
            b.Mat("Aluminum_Anodized"); b.Prism(DroneMeshBuilder.RectPoly(0, zc, 0.097f, 0.0229f, 0.003f), cy - 0.01475f, cy + 0.01475f, 0.0008f);
            float zf = zc + 0.0229f * 0.5f;
            b.Mat("Glass_Lens"); b.Box(new Vector3(0, cy, zf + 0.0003f), new Vector3(0.0935f, 0.0262f, 0.0006f), 0.0002f);
            // three lens barrels: left mono, RGB center, right mono (75 mm baseline)
            foreach (float x in new[] { -0.0375f, 0f, 0.0375f })
            {
                b.Mat("Aluminum_Brushed");
                b.Push(new Vector3(x, cy + (x == 0f ? 0f : 0f), zf + 0.0006f), Quaternion.Euler(90, 0, 0));
                b.Cyl(x == 0f ? 0.0075f : 0.0062f, 0, 0.0016f, 28, 0.0003f);   // ring
                b.Pop();
                b.Mat("Glass_Lens"); b.Push(new Vector3(x, cy, zf + 0.0006f - 0.0000f), Quaternion.Euler(90, 0, 0));
                b.Cyl(x == 0f ? 0.0058f : 0.0046f, 0.0008f, 0.0019f, 28, 0.0002f); b.Pop();
            }
            b.Mat("Plastic_Black"); b.Box(new Vector3(0, cy + 0.0060f, zf + 0.0010f), new Vector3(0.0035f, 0.0030f, 0.0004f), 0f);   // ambient sensor
            b.Mat("Aluminum_Brushed"); b.Box(new Vector3(0f, cy, zc - 0.0229f * 0.5f - 0.0004f), new Vector3(0.0120f, 0.0070f, 0.0008f), 0.0002f);   // USB-C
            return b;
        }

        static DroneMeshBuilder BatteryMesh()
        {
            var b = new DroneMeshBuilder(); float top = -0.0240f, h = 0.038f, w = 0.036f, l = 0.108f, cz = -0.005f;
            b.Mat("LiPo_Wrap"); b.Prism(DroneMeshBuilder.RectPoly(0, cz, w, l, 0.005f), top - h, top, 0.0025f);
            b.Mat("LiPo_Label"); b.Box(new Vector3(0, top - 0.0004f + 0.0000f, cz + 0f), new Vector3(0.0325f, 0.0008f, 0.080f), 0.0001f);
            // straps (two Velcro loops around the pack and bottom plate)
            b.Mat("Rubber");
            foreach (float z in new[] { 0.026f, -0.036f })
            {
                b.Box(new Vector3(0, top + 0.0007f, z), new Vector3(w + 0.0045f, 0.0030f, 0.0160f), 0.0006f);
                b.Box(new Vector3(0, top - h - 0.0007f, z), new Vector3(w + 0.0045f, 0.0030f, 0.0160f), 0.0006f);
                b.Box(new Vector3(-w * 0.5f - 0.0009f, top - h * 0.5f, z), new Vector3(0.0030f, h + 0.002f, 0.0160f), 0.0006f);
                b.Box(new Vector3(w * 0.5f + 0.0009f, top - h * 0.5f, z), new Vector3(0.0030f, h + 0.002f, 0.0160f), 0.0006f);
            }
            // XT60 on the rear, balance JST-XH on the side
            b.Mat("Connector_Yellow"); b.Box(new Vector3(0, top - 0.011f, cz - l * 0.5f - 0.0090f), new Vector3(0.0158f, 0.0083f, 0.0180f), 0.0008f);
            b.Mat("Plastic_White"); b.Box(new Vector3(w * 0.5f + 0.0040f, top - 0.026f, cz - l * 0.5f + 0.008f), new Vector3(0.0065f, 0.0038f, 0.0175f), 0.0004f);
            return b;
        }

        static DroneMeshBuilder WiringMesh()
        {
            var b = new DroneMeshBuilder();
            // motor leads: motor flange -> underside of arm -> body -> ESC edge
            b.Mat("Wire_Black");
            for (int i = 0; i < 4; i++)
            {
                float sx = RotorSx[i], sz = RotorSz[i];
                Vector2 s = new Vector2(sx * 0.016f, sz * 0.040f), e = new Vector2(sx * MotorX, sz * MotorZ); Vector2 d = (e - s).normalized, p = new Vector2(-d.y, d.x);
                Vector2 q0 = e - d * 0.012f, q1 = e - d * 0.030f, q2 = s + d * 0.012f, q3 = new Vector2(sx * 0.022f, sz * 0.020f);
                var pts = new List<Vector3> {
                    new Vector3(q0.x, ArmY1 + 0.0035f, q0.y), new Vector3(q0.x, ArmY0 - 0.0024f, q0.y),
                    new Vector3(q1.x, ArmY0 - 0.0024f, q1.y), new Vector3(q2.x, ArmY0 - 0.0024f, q2.y),
                    new Vector3(q3.x, -0.0150f, q3.y), new Vector3(sx * 0.0195f, -0.0100f, sz * 0.0185f) };
                b.Tube(DroneMeshBuilder.CatmullRom(pts, 6), 0.0026f, 10);
            }
            // battery leads XT60 -> ESC (12 AWG: ~2.1 mm conductor, 3.2 mm jacket)
            float zx = -0.005f - 0.054f - 0.0090f;
            foreach (float sx in new[] { -1f, 1f })
            {
                b.Mat(sx > 0 ? "Wire_Red" : "Wire_Black");
                var pts = new List<Vector3> {
                    new Vector3(sx * 0.0035f, -0.0350f, zx), new Vector3(sx * 0.0045f, -0.0300f, zx + 0.012f), new Vector3(sx * 0.009f, -0.0215f, -0.052f),
                    new Vector3(sx * 0.0145f, -0.0150f, -0.034f), new Vector3(sx * 0.0125f, -0.0100f, -0.0215f) };
                b.Tube(DroneMeshBuilder.CatmullRom(pts, 6), 0.0017f, 10);
            }
            // balance lead and FC signal harness (ribbon-ish)
            b.Mat("Wire_Black");
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0.0215f, -0.0500f, -0.0475f), new Vector3(0.0260f, -0.0300f, -0.0420f), new Vector3(0.0235f, -0.0100f, -0.0300f), new Vector3(0.0195f, 0.0060f, -0.0160f) }, 6), 0.0016f, 8);
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0.0120f, -0.0080f, 0.0215f), new Vector3(0.0190f, 0.0030f, 0.0290f), new Vector3(0.0225f, 0.0155f, 0.0220f), new Vector3(0.0050f, 0.0185f, 0.0060f), new Vector3(-0.0030f, 0.0200f, -0.0100f) }, 8), 0.0020f, 8);
            // OAK-D USB-C cable up to Jetson
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0, -0.0018f, 0.0950f), new Vector3(-0.004f, 0.0100f, 0.0800f), new Vector3(-0.012f, 0.0180f, 0.0600f), new Vector3(-0.026f, 0.0200f, 0.0420f) }, 8), 0.0022f, 8);
            // microhard u.FL pigtails and RX coax
            b.Mat("Plastic_DarkGray");
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0.011f, 0.0185f, -0.0940f), new Vector3(0.0150f, 0.0170f, -0.0985f), new Vector3(0.0180f, 0.0185f, -0.1020f) }, 6), 0.0009f, 8);
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0.0285f, 0.0160f, -0.0775f), new Vector3(0.0300f, 0.0020f, -0.0900f), new Vector3(0.0310f, -0.0150f, -0.1000f), new Vector3(0.0300f, ArmY0 - 0.0015f, -0.1120f) }, 8), 0.0009f, 8);
            b.Tube(DroneMeshBuilder.CatmullRom(new List<Vector3> {
                new Vector3(0.0200f, 0.0160f, -0.0775f), new Vector3(-0.0150f, 0.0150f, -0.0980f), new Vector3(-0.0290f, 0.0010f, -0.1050f), new Vector3(-0.0300f, ArmY0 - 0.0015f, -0.1120f) }, 8), 0.0009f, 8);
            return b;
        }

        // ------------------------------------------------------------ preview
        [MenuItem("CRANE/Drones/Export Preview OBJ")]
        public static void ExportPreviewObj()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null) { Debug.LogError("prefab missing; run Build first"); return; }
            string outDir = "Docs/DroneVisuals/model"; Directory.CreateDirectory(outDir);
            var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            var rig = go.GetComponent<DroneVisualRig>();
            WriteObj(go, outDir, "aos7quad");
            rig.SetRotorSpeeds(new float[] { 400f, 400f, 400f, 400f });   // hover-like: blur disc + ghost blades
            WriteObj(go, outDir, "aos7quad_blur");
            rig.SetRotorSpeeds(new float[] { 150f, 150f, 150f, 150f });   // crossfade mid-point
            WriteObj(go, outDir, "aos7quad_fade");
            rig.SetRotorSpeeds(new float[] { 0, 0, 0, 0 });
            var sb = new StringBuilder();
            sb.AppendLine("rotors (body FLU, m):");
            foreach (var r in rig.rotors) sb.AppendLine("  " + r.name + " flu=" + rig.BodyFlu(r.hub).ToString("F4"));
            sb.AppendLine("diag M1-M2 (mm): " + (Vector3.Distance(rig.rotors[0].hub.position, rig.rotors[1].hub.position) * 1000f).ToString("F1"));
            sb.AppendLine("diag M3-M4 (mm): " + (Vector3.Distance(rig.rotors[2].hub.position, rig.rotors[3].hub.position) * 1000f).ToString("F1"));
            var bounds = new Bounds(go.transform.position, Vector3.zero); bool first = true;
            foreach (var r in go.GetComponentsInChildren<Renderer>()) { if (!r.enabled) continue; if (first) { bounds = r.bounds; first = false; } else bounds.Encapsulate(r.bounds); }
            sb.AppendLine("bounds size (m) " + bounds.size.ToString("F4") + " min " + bounds.min.ToString("F4") + " max " + bounds.max.ToString("F4"));
            sb.AppendLine("cam flu=" + rig.BodyFlu(rig.cameraMount).ToString("F4") + " lidar flu=" + rig.BodyFlu(rig.lidarMount).ToString("F4") + " imu flu=" + rig.BodyFlu(rig.imuMount).ToString("F4") + " com flu=" + rig.BodyFlu(rig.comMarker).ToString("F4"));
            int tris = 0, verts = 0; foreach (var mf in go.GetComponentsInChildren<MeshFilter>()) { tris += mf.sharedMesh.triangles.Length / 3; verts += mf.sharedMesh.vertexCount; }
            sb.AppendLine("tris(all renderers incl. hidden prop variants)=" + tris + " verts=" + verts);
            File.WriteAllText(outDir + "/dimensions.txt", sb.ToString());
            UnityEngine.Object.DestroyImmediate(go);
            Debug.Log("[BuildAOS7Quad] preview obj + dimensions written\n" + sb);
        }

        static void WriteObj(GameObject go, string outDir, string baseName)
        {
            var obj = new StringBuilder(); var mats = new Dictionary<string, Color>(); var sbMat = new StringBuilder();
            int voff = 1; obj.AppendLine("mtllib " + baseName + ".mtl");
            foreach (var mr in go.GetComponentsInChildren<MeshRenderer>(false))
            {
                var mf = mr.GetComponent<MeshFilter>(); if (mf == null || mf.sharedMesh == null || !mr.enabled) continue;
                var mesh = mf.sharedMesh; var l2w = mr.transform.localToWorldMatrix;
                var vs = mesh.vertices; var ns = mesh.normals;
                foreach (var v in vs) { var w = l2w.MultiplyPoint3x4(v); obj.AppendLine("v " + w.x.ToString("F5") + " " + w.y.ToString("F5") + " " + w.z.ToString("F5")); }
                foreach (var n in ns) { var w = l2w.MultiplyVector(n).normalized; obj.AppendLine("vn " + w.x.ToString("F4") + " " + w.y.ToString("F4") + " " + w.z.ToString("F4")); }
                for (int s = 0; s < mesh.subMeshCount; s++)
                {
                    var mat = mr.sharedMaterials[Mathf.Min(s, mr.sharedMaterials.Length - 1)];
                    if (!mats.ContainsKey(mat.name)) { Color c = mat.GetColor("_BaseColor"); if (mat.HasProperty("_EmissiveColorLDR") && mat.GetFloat("_UseEmissiveIntensity") > 0.5f) c = mat.GetColor("_EmissiveColorLDR"); mats[mat.name] = c; }
                    obj.AppendLine("usemtl " + mat.name);
                    var tri = mesh.GetTriangles(s);
                    for (int t = 0; t < tri.Length; t += 3)
                    {
                        int a = tri[t] + voff, b = tri[t + 2] + voff, c2 = tri[t + 1] + voff;   // Unity CW -> OBJ CCW
                        obj.AppendLine("f " + a + "//" + a + " " + b + "//" + b + " " + c2 + "//" + c2);
                    }
                }
                voff += vs.Length;
            }
            foreach (var kv in mats) sbMat.AppendLine("newmtl " + kv.Key + "\nKd " + kv.Value.r.ToString("F4") + " " + kv.Value.g.ToString("F4") + " " + kv.Value.b.ToString("F4") + "\n");
            File.WriteAllText(outDir + "/" + baseName + ".obj", obj.ToString()); File.WriteAllText(outDir + "/" + baseName + ".mtl", sbMat.ToString());
        }
    }
}
