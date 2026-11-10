using System;
using System.IO;
using System.IO.Compression;

namespace Crane.Drones.Logging {
    /// <summary>Dependency-free PNG encoder: 8-bit RGB and 16-bit grayscale (depth, millimetres).</summary>
    public static class DronePng {
        private static uint[] s_Crc;

        public static byte[] EncodeRgb8(byte[] rgb, int width, int height) {
            if (rgb.Length != width * height * 3) throw new ArgumentException("rgb size mismatch");
            return Encode(rgb, width, height, 8, 2, 3);
        }

        /// <summary>16-bit grayscale, big-endian samples per PNG spec.</summary>
        public static byte[] EncodeGray16(ushort[] samples, int width, int height) {
            if (samples.Length != width * height) throw new ArgumentException("gray16 size mismatch");
            var raw = new byte[samples.Length * 2];
            for (int i = 0; i < samples.Length; i++) {
                raw[2 * i] = (byte)(samples[i] >> 8);
                raw[2 * i + 1] = (byte)(samples[i] & 0xFF);
            }
            return Encode(raw, width, height, 16, 0, 2);
        }

        private static byte[] Encode(byte[] pixels, int w, int h, int depth, int colorType, int bytesPerPixel) {
            int stride = w * bytesPerPixel;
            var scan = new byte[(stride + 1) * h];
            for (int y = 0; y < h; y++) {
                scan[y * (stride + 1)] = 0; // filter: none
                Buffer.BlockCopy(pixels, y * stride, scan, y * (stride + 1) + 1, stride);
            }
            var z = new MemoryStream();
            z.WriteByte(0x78); z.WriteByte(0x01);
            using (var d = new DeflateStream(z, System.IO.Compression.CompressionLevel.Fastest, true)) d.Write(scan, 0, scan.Length);
            uint a = 1, b = 0;
            foreach (byte x in scan) { a = (a + x) % 65521; b = (b + a) % 65521; }
            uint adler = (b << 16) | a;
            z.WriteByte((byte)(adler >> 24)); z.WriteByte((byte)(adler >> 16));
            z.WriteByte((byte)(adler >> 8)); z.WriteByte((byte)adler);

            var o = new MemoryStream();
            o.Write(new byte[] { 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A }, 0, 8);
            var ihdr = new byte[13];
            BE(ihdr, 0, (uint)w); BE(ihdr, 4, (uint)h);
            ihdr[8] = (byte)depth; ihdr[9] = (byte)colorType;
            Chunk(o, "IHDR", ihdr);
            Chunk(o, "IDAT", z.ToArray());
            Chunk(o, "IEND", new byte[0]);
            return o.ToArray();
        }

        private static void BE(byte[] b, int i, uint v) {
            b[i] = (byte)(v >> 24); b[i + 1] = (byte)(v >> 16); b[i + 2] = (byte)(v >> 8); b[i + 3] = (byte)v;
        }

        private static void Chunk(Stream s, string type, byte[] data) {
            var len = new byte[4]; BE(len, 0, (uint)data.Length); s.Write(len, 0, 4);
            var body = new byte[4 + data.Length];
            for (int i = 0; i < 4; i++) body[i] = (byte)type[i];
            Buffer.BlockCopy(data, 0, body, 4, data.Length);
            s.Write(body, 0, body.Length);
            var crc = new byte[4]; BE(crc, 0, Crc32(body)); s.Write(crc, 0, 4);
        }

        private static uint Crc32(byte[] data) {
            if (s_Crc == null) {
                var t = new uint[256];
                for (uint n = 0; n < 256; n++) {
                    uint c = n;
                    for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                    t[n] = c;
                }
                s_Crc = t;
            }
            uint crc = 0xFFFFFFFFu;
            foreach (byte x in data) crc = s_Crc[(crc ^ x) & 0xFF] ^ (crc >> 8);
            return crc ^ 0xFFFFFFFFu;
        }
    }
}
