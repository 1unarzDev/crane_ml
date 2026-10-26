using System.Collections.Generic;
using UnityEngine;

namespace Sim.Course {
    /// <summary>A dock banner's face (H p.55: 24 in, black numeral on white): Number draws the
    /// numeral, Shape the 2025 coloured shapes for replay. Stands in for stream A's numeral atlas.</summary>
    public sealed class SignFace : StatefulCourseObject {
        public enum Style { Number, Shape }
        [SerializeField] Style style = Style.Number;
        static readonly Dictionary<string, Texture2D> Cache = new Dictionary<string, Texture2D>();

        public override string Kind => "sign_face";
        public Style FaceStyle { get => style; set => style = value; }

        // 5x7 glyphs, rows top to bottom.
        static readonly Dictionary<char, string[]> Glyphs = new Dictionary<char, string[]> {
            ['1'] = new[] { "..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###." },
            ['2'] = new[] { ".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####" },
            ['3'] = new[] { ".###.", "#...#", "....#", "..##.", "....#", "#...#", ".###." },
        };

        /// <summary>Is texel (u, v) in [0, 1] (v up) inked for this face?</summary>
        public static bool Inked(Style style, string value, float u, float v) {
            if (style == Style.Shape) {
                string shape = value.Contains("_") ? value.Substring(value.IndexOf('_') + 1) : value;
                return CourseVisuals.InShape(shape, u, v);
            }
            if (string.IsNullOrEmpty(value) || !Glyphs.TryGetValue(value[0], out var rows)) return false;
            float gx = (u - 0.2f) / 0.6f * 5f, gy = (0.85f - v) / 0.7f * 7f;
            int col = Mathf.FloorToInt(gx), row = Mathf.FloorToInt(gy);
            return col >= 0 && col < 5 && row >= 0 && row < 7 && rows[row][col] == '#';
        }

        protected override void Render(string value) {
            Renderer r = Slot();
            if (r == null) return;
            string key = $"{style}:{value}";
            if (!Cache.TryGetValue(key, out var texture)) {
                const int size = 64;
                Color ink = style == Style.Shape ? CourseVisuals.BannerColor(value) : Color.black;
                texture = new Texture2D(size, size, TextureFormat.RGBA32, false);
                for (int y = 0; y < size; y++)
                    for (int x = 0; x < size; x++)
                        texture.SetPixel(x, y, Inked(style, value, (x + 0.5f) / size, (y + 0.5f) / size) ? ink : Color.white);
                texture.Apply();
                Cache[key] = texture;
            }
            var block = new MaterialPropertyBlock();
            r.GetPropertyBlock(block);
            block.SetTexture("_BaseColorMap", texture);
            block.SetTexture("_MainTex", texture);
            block.SetColor("_BaseColor", Color.white);
            r.SetPropertyBlock(block);
        }
    }
}
