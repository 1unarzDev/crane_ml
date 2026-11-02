using UnityEngine;

namespace Crane.Theater {
    /// <summary>Perceptually uniform ramps (sRGB stops, 9 each, linearly interpolated).</summary>
    public static class Colormaps {
        static readonly Color32[] Cividis = {
            new Color32(0, 34, 78, 255), new Color32(32, 52, 108, 255), new Color32(68, 73, 107, 255), new Color32(97, 96, 111, 255),
            new Color32(124, 123, 120, 255), new Color32(154, 148, 117, 255), new Color32(188, 175, 106, 255),
            new Color32(222, 204, 82, 255), new Color32(254, 232, 56, 255)
        };
        static readonly Color32[] Viridis = {
            new Color32(68, 1, 84, 255), new Color32(71, 45, 123, 255), new Color32(59, 82, 139, 255), new Color32(44, 114, 142, 255),
            new Color32(33, 145, 140, 255), new Color32(40, 174, 128, 255), new Color32(94, 201, 98, 255),
            new Color32(173, 220, 48, 255), new Color32(253, 231, 37, 255)
        };
        static readonly Color32[] Plasma = {
            new Color32(13, 8, 135, 255), new Color32(84, 2, 163, 255), new Color32(139, 10, 165, 255), new Color32(185, 50, 137, 255),
            new Color32(219, 92, 104, 255), new Color32(244, 136, 73, 255), new Color32(254, 188, 43, 255),
            new Color32(240, 249, 33, 255), new Color32(240, 249, 33, 255)
        };

        public static Color Eval(string name, float u) {
            var stops = name == "viridis" ? Viridis : name == "plasma" ? Plasma : Cividis;
            u = Mathf.Clamp01(float.IsNaN(u) ? 0 : u) * (stops.Length - 1);
            int i = Mathf.Min(stops.Length - 2, (int)u);
            return Color.Lerp(stops[i], stops[i + 1], u - i);
        }

        public static readonly Color Gold = new Color32(255, 196, 40, 255);
        public static readonly Color Silver = new Color32(200, 214, 230, 255);
        public static readonly Color Failure = new Color32(255, 70, 60, 255);
        public static readonly Color Gate = new Color32(90, 220, 255, 255);
    }
}
