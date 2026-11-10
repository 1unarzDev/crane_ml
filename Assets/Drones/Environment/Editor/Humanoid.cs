using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>Anatomically proportioned adult figure (1.75 m) lofted from elliptical limb sections, with clothing layers as
    /// separate submeshes: 0 skin, 1 jacket, 2 trousers, 3 boots, 4 hood/beanie. Local frame: +X forward/head, +Y up, +Z left.</summary>
    public static class Humanoid
    {
        public enum Pose { SupineBentKnee, SittingAgainstTree, ProneArmExtended }
        static Vector3 V(float x, float y, float z) { return new Vector3(x, y, z); }

        // radii of limb sections (m)
        static List<Vector3> Spline(params Vector3[] k) // Catmull-Rom through key joints, ~9 pts per segment pair
        {
            var o = new List<Vector3>();
            for (int i = 0; i < k.Length - 1; i++)
            {
                Vector3 p0 = k[Mathf.Max(0, i - 1)], p1 = k[i], p2 = k[i + 1], p3 = k[Mathf.Min(k.Length - 1, i + 2)];
                for (int s = 0; s < 6; s++)
                {
                    float t = s / 6f, t2 = t * t, t3 = t2 * t;
                    o.Add(0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3));
                }
            }
            o.Add(k[k.Length - 1]); return o;
        }

        static List<float> Radii(int n, params float[] key) // piecewise-linear over normalised length
        {
            var r = new List<float>();
            for (int i = 0; i < n; i++)
            {
                float t = (float)i / (n - 1) * (key.Length - 1); int a = Mathf.Min(key.Length - 2, (int)t);
                r.Add(Mathf.Lerp(key[a], key[a + 1], t - a));
            }
            return r;
        }

        static void Limb(MB mb, int sub, float uTile, float rScale, float ex, float ey, Vector3? refA, params object[] kr)
        {
            // kr = joint positions (Vector3...) followed by radius keys (float...)
            var joints = new List<Vector3>(); var rk = new List<float>();
            foreach (var o in kr) { if (o is Vector3) joints.Add((Vector3)o); else rk.Add((float)o); }
            var pts = Spline(joints.ToArray()); var rad = Radii(pts.Count, rk.ToArray());
            for (int i = 0; i < rad.Count; i++) rad[i] *= rScale;
            // rounded ends: taper the first/last rings
            Geo.Tube(mb, sub, pts, rad, 12, uTile, 0.6f, ex, ey, refA, true, true);
        }

        public static MB Build(Pose pose, System.Random rng)
        {
            var mb = new MB(5);
            float k = Geo.R(rng, 0.96f, 1.05f);
            Vector3 zL = V(0, 0, 1);
            Vector3 P, C, N, Hd, shL, shR, hipL, hipR, kneeL, kneeR, ankL, ankR, toeL, toeR, elbL, elbR, wrL, wrR, handL, handR;
            switch (pose)
            {
                case Pose.SupineBentKnee:
                    P = V(0, .15f, 0); C = V(.50f, .17f, 0); N = V(.64f, .15f, 0); Hd = V(.78f, .115f, .03f);
                    shL = C + V(0, 0, .19f); shR = C - V(0, 0, .19f); hipL = P + V(0, 0, .09f); hipR = P - V(0, 0, .09f);
                    kneeL = V(-.30f, .40f, .13f); ankL = V(-.55f, .11f, .16f); toeL = V(-.62f, .30f, .16f);
                    kneeR = V(-.46f, .12f, -.10f); ankR = V(-.90f, .10f, -.14f); toeR = V(-.92f, .32f, -.14f);
                    elbL = V(.36f, .10f, .34f); wrL = V(.12f, .09f, .30f); handL = V(.0f, .08f, .30f);
                    elbR = V(.46f, .20f, -.26f); wrR = V(.40f, .27f, -.06f); handR = V(.36f, .24f, .04f);
                    break;
                case Pose.SittingAgainstTree:
                    P = V(0, .14f, 0); C = V(-.10f, .66f, 0); N = V(-.08f, .80f, 0); Hd = V(.03f, .91f, 0);
                    shL = C + V(0, 0, .19f); shR = C - V(0, 0, .19f); hipL = P + V(0, 0, .09f); hipR = P - V(0, 0, .09f);
                    kneeL = V(.36f, .24f, .13f); ankL = V(.78f, .09f, .17f); toeL = V(.98f, .10f, .17f);
                    kneeR = V(.30f, .30f, -.12f); ankR = V(.72f, .09f, -.20f); toeR = V(.92f, .11f, -.22f);
                    elbL = V(-.02f, .40f, .28f); wrL = V(.18f, .26f, .20f); handL = V(.30f, .24f, .17f);
                    elbR = V(-.04f, .42f, -.27f); wrR = V(.12f, .32f, -.12f); handR = V(.20f, .30f, -.04f);
                    break;
                default: // prone
                    P = V(0, .13f, 0); C = V(.48f, .16f, 0); N = V(.66f, .17f, 0); Hd = V(.80f, .13f, .04f);
                    shL = C + V(0, 0, .19f); shR = C - V(0, 0, .19f); hipL = P + V(0, 0, .09f); hipR = P - V(0, 0, .09f);
                    kneeL = V(-.42f, .11f, .14f); ankL = V(-.86f, .09f, .16f); toeL = V(-.99f, .07f, .16f);
                    kneeR = V(-.40f, .12f, -.10f); ankR = V(-.78f, .20f, -.22f); toeR = V(-.88f, .24f, -.24f);
                    elbL = V(.70f, .09f, .28f); wrL = V(.98f, .07f, .24f); handL = V(1.12f, .06f, .22f);
                    elbR = V(.32f, .09f, -.30f); wrR = V(.12f, .08f, -.26f); handR = V(.02f, .07f, -.24f);
                    break;
            }
            Vector3 mid = (P + C) * 0.5f + V(0, 0.02f, 0);
            // torso: pelvis -> waist -> chest -> neck base (jacket); ellipse wider in Z
            Limb(mb, 1, 3, k, 1.45f, 0.95f, zL, P - (C - P).normalized * .05f, P, mid, C, N, .13f, .15f, .15f, .17f, .10f);
            // neck + head
            Limb(mb, 0, 1, k, 1f, 1f, null, N, (N + Hd) * 0.5f, .05f, .052f);
            Vector3 hd = (Hd - N).normalized;
            Limb(mb, 0, 1, k, 0.88f, 1f, zL, Hd - hd * .115f, Hd - hd * .06f, Hd + hd * .04f, Hd + hd * .12f, .03f, .085f, .096f, .05f);
            // hood / beanie shell over upper head
            Limb(mb, 4, 2, k, 0.92f, 1.0f, zL, Hd - hd * .085f, Hd, Hd + hd * .09f, Hd + hd * .125f, .055f, .106f, .108f, .04f);
            // legs: trousers, boots
            foreach (var leg in new[] { new[] { hipL, kneeL, ankL, toeL }, new[] { hipR, kneeR, ankR, toeR } })
            {
                Limb(mb, 2, 2, k, 1f, 1f, null, leg[0] - (leg[1] - leg[0]).normalized * .03f, leg[0], (leg[0] + leg[1]) * .5f, leg[1], .085f, .088f, .075f, .056f);
                Limb(mb, 2, 2, k, 1f, 1f, null, leg[1], (leg[1] + leg[2]) * .5f, leg[2], .056f, .052f, .045f);
                Limb(mb, 3, 1, k, 1f, 1f, null, leg[2] + (leg[2] - leg[1]).normalized * -.02f, leg[2], leg[3], .056f, .062f, .045f);
            }
            // arms: jacket sleeves, hands skin
            foreach (var arm in new[] { new[] { shL, elbL, wrL, handL }, new[] { shR, elbR, wrR, handR } })
            {
                Limb(mb, 1, 2, k, 1f, 1f, null, arm[0] - (arm[1] - arm[0]).normalized * .02f, arm[0], (arm[0] + arm[1]) * .5f, arm[1], .058f, .058f, .052f, .045f);
                Limb(mb, 1, 2, k, 1f, 1f, null, arm[1], (arm[1] + arm[2]) * .5f, arm[2], .045f, .042f, .036f);
                Limb(mb, 0, 1, k, 1f, 0.6f, null, arm[2], (arm[2] + arm[3]) * .5f, arm[3], .036f, .04f, .018f);
            }
            return mb;
        }
    }
}
