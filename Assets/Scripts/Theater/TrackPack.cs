using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Crane.Theater {
    /// <summary>One pose sample in the world's frame (ENU: x east, y north, z up; angles in degrees, yaw CCW
    /// from +x), as the theater's plug-ins receive it.</summary>
    public struct Pose6 {
        public float x, y, z, yaw, pitch, roll, speed, progress;
    }

    /// <summary>Reader of <c>crane-theater-pack/1</c> (theater/pack.py): every run resampled to one rate and
    /// quantized to int16, run-major. Resident as int16 (16 bytes a sample, about 100 MB for 500 ten-minute
    /// runs at 10 Hz); a pose at any time is decoded on demand (Catmull-Rom on x and y, linear elsewhere).</summary>
    public sealed class TrackPack {
        public const uint Magic = 0x4B505443; // "CTPK" little-endian
        public float Hz { get; private set; }
        public int RunCount => first.Length;
        public string[] Channels { get; private set; }
        int[] first, count;
        short[] data;
        int nch;
        float[] scale, offset;
        int cx = -1, cy = -1, cz = -1, cyaw = -1, cpitch = -1, croll = -1, cspeed = -1, cprog = -1;

        public static TrackPack Load(string path) => Parse(File.ReadAllBytes(path));

        public static TrackPack Parse(byte[] bytes) {
            if (bytes.Length < 20 || BitConverter.ToUInt32(bytes, 0) != Magic) throw new InvalidDataException("not a crane theater pack");
            uint crc = BitConverter.ToUInt32(bytes, bytes.Length - 4);
            if (crc != Crc32(bytes, 0, bytes.Length - 4)) throw new InvalidDataException("theater pack crc mismatch");
            uint version = BitConverter.ToUInt32(bytes, 4);
            if (version != 1) throw new InvalidDataException($"theater pack version {version}");
            int hlen = (int)BitConverter.ToUInt32(bytes, 8);
            var header = TJson.Parse(Encoding.UTF8.GetString(bytes, 12, hlen));
            var p = new TrackPack { Hz = (float)TJson.Num(header, "hz", 10) };
            var chans = TJson.Arr(header, "channels");
            p.nch = chans.Count;
            p.Channels = new string[p.nch];
            p.scale = new float[p.nch];
            p.offset = new float[p.nch];
            for (int i = 0; i < p.nch; i++) {
                p.Channels[i] = TJson.Str(chans[i], "name");
                p.scale[i] = (float)TJson.Num(chans[i], "scale", 1);
                p.offset[i] = (float)TJson.Num(chans[i], "offset", 0);
                switch (p.Channels[i]) {
                    case "x": p.cx = i; break;
                    case "y": p.cy = i; break;
                    case "z": p.cz = i; break;
                    case "yaw": p.cyaw = i; break;
                    case "pitch": p.cpitch = i; break;
                    case "roll": p.croll = i; break;
                    case "speed": p.cspeed = i; break;
                    case "progress": p.cprog = i; break;
                }
            }
            int o = 12 + hlen;
            int n = (int)BitConverter.ToUInt32(bytes, o); o += 4;
            p.first = new int[n]; p.count = new int[n];
            long total = 0;
            for (int i = 0; i < n; i++) {
                p.first[i] = (int)BitConverter.ToUInt32(bytes, o);
                p.count[i] = (int)BitConverter.ToUInt32(bytes, o + 4);
                o += 8;
                total = Math.Max(total, (long)p.first[i] + p.count[i]);
            }
            long need = total * p.nch;
            if (o + need * 2 > bytes.Length - 4) throw new InvalidDataException("theater pack truncated");
            p.data = new short[need];
            Buffer.BlockCopy(bytes, o, p.data, 0, (int)(need * 2));
            return p;
        }

        public int Samples(int run) => count[run];
        public float Duration(int run) => count[run] > 0 ? (count[run] - 1) / Hz : 0f;

        float Ch(int run, int k, int c) => c < 0 ? 0f : data[(long)(first[run] + k) * nch + c] * scale[c] + offset[c];

        public Pose6 Sample(int run, int k) {
            k = Math.Max(0, Math.Min(count[run] - 1, k));
            return new Pose6 {
                x = Ch(run, k, cx), y = Ch(run, k, cy), z = Ch(run, k, cz), yaw = Ch(run, k, cyaw),
                pitch = Ch(run, k, cpitch), roll = Ch(run, k, croll), speed = Ch(run, k, cspeed), progress = Ch(run, k, cprog)
            };
        }

        /// <summary>The pose at run time t (s from the run's start), clamped to the run.</summary>
        public Pose6 At(int run, double t) {
            int n = count[run];
            if (n == 0) return default;
            double f = t * Hz;
            if (f <= 0) return Sample(run, 0);
            if (f >= n - 1) return Sample(run, n - 1);
            int k = (int)Math.Floor(f);
            float u = (float)(f - k);
            Pose6 a = Sample(run, k), b = Sample(run, k + 1);
            Pose6 a0 = Sample(run, k - 1), b1 = Sample(run, k + 2);
            return new Pose6 {
                x = CatmullRom(a0.x, a.x, b.x, b1.x, u), y = CatmullRom(a0.y, a.y, b.y, b1.y, u),
                z = a.z + (b.z - a.z) * u, yaw = LerpAngle(a.yaw, b.yaw, u), pitch = a.pitch + (b.pitch - a.pitch) * u,
                roll = a.roll + (b.roll - a.roll) * u, speed = a.speed + (b.speed - a.speed) * u,
                progress = a.progress + (b.progress - a.progress) * u
            };
        }

        static float CatmullRom(float p0, float p1, float p2, float p3, float u) {
            float u2 = u * u, u3 = u2 * u;
            return 0.5f * (2f * p1 + (-p0 + p2) * u + (2f * p0 - 5f * p1 + 4f * p2 - p3) * u2 + (-p0 + 3f * p1 - 3f * p2 + p3) * u3);
        }

        public static float LerpAngle(float a, float b, float u) {
            float d = (b - a) % 360f;
            if (d > 180f) d -= 360f;
            if (d < -180f) d += 360f;
            return a + d * u;
        }

        static uint[] crcTable;
        public static uint Crc32(byte[] b, int start, int len) {
            if (crcTable == null) {
                var t = new uint[256];
                for (uint i = 0; i < 256; i++) {
                    uint c = i;
                    for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                    t[i] = c;
                }
                crcTable = t;
            }
            uint crc = 0xFFFFFFFFu;
            for (int i = start; i < start + len; i++) crc = crcTable[(crc ^ b[i]) & 0xFF] ^ (crc >> 8);
            return crc ^ 0xFFFFFFFFu;
        }
    }
}
