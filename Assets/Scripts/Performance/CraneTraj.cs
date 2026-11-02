using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Text;

namespace Sim.Performance {
    /// <summary>
    /// crane-traj/1 (<c>.ctraj</c>): a run's fixed-rate columns and events. The layout is defined by
    /// <c>theater/traj.py</c> (the Python reader and writer); this is its C# twin, kept in step by
    /// the golden file <c>theater/tests/golden/small.ctraj</c> (CraneTrajTests). Little-endian:
    /// "CTRJ" u16 version u16 flags, u32 header_len + JSON, u16 n_columns + (u8 name_len, name,
    /// f64 scale, f64 offset), u32 ticks u32 block_ticks u32 n_blocks, n_blocks x (u32 first, u32 n,
    /// u32 len), the blocks (raw deflate of zigzag varint deltas, column after column), u32 + raw
    /// deflate of the events (JSON lines), u32 crc32 of everything before it.
    /// </summary>
    public static class CraneTraj {
        public const string Schema = "crane-traj/1";
        public const ushort Version = 1;
        public const double BlockSeconds = 30.0;
        public const double Metre = 0.01;
        public static readonly double Radian = 0.01 * Math.PI / 180.0;

        public sealed class Column {
            public string Name;
            public double Scale;
            public double Offset;
            public List<double> Values = new();
        }

        public sealed class Data {
            public string HeaderJson;
            public double Hz;
            public int Ticks;
            public int BlockTicks;
            public List<Column> Columns = new();
            public List<string> EventLines = new();

            public Column this[string name] {
                get {
                    foreach (Column c in Columns) if (c.Name == name) return c;
                    return null;
                }
            }
        }

        /// <summary>The default quantum of a column by its last dotted part.</summary>
        public static double ScaleOf(string name) {
            string axis = name.Substring(name.LastIndexOf('.') + 1);
            switch (axis) {
                case "x": case "y": case "z": return Metre;
                case "yaw": case "pitch": case "roll": return Radian;
                default: return 0.001;
            }
        }

        public static long Quantize(double v, double scale, double offset) {
            if (double.IsNaN(v) || double.IsInfinity(v)) throw new ArgumentException($"non-finite value {v}");
            return (long)Math.Floor((v - offset) / scale + 0.5);
        }

        /// <summary>The file's bytes. <paramref name="headerJson"/> is a JSON object (its keys are
        /// kept; the Python side normalizes key order when it amends the header).</summary>
        public static byte[] Encode(string headerJson, double hz, IReadOnlyList<Column> columns,
            IReadOnlyList<string> eventLines, double blockSeconds = BlockSeconds) {
            if (!(hz > 0)) throw new ArgumentException("hz must be positive");
            int ticks = columns.Count > 0 ? columns[0].Values.Count : 0;
            foreach (Column c in columns)
                if (c.Values.Count != ticks) throw new ArgumentException($"column {c.Name} has {c.Values.Count} of {ticks} ticks");
            using var output = new MemoryStream();
            using var w = new BinaryWriter(output, Encoding.UTF8, true);
            w.Write(Encoding.ASCII.GetBytes("CTRJ"));
            w.Write(Version);
            w.Write((ushort)0);
            byte[] header = Encoding.UTF8.GetBytes(headerJson);
            w.Write((uint)header.Length);
            w.Write(header);
            w.Write((ushort)columns.Count);
            foreach (Column c in columns) {
                byte[] name = Encoding.UTF8.GetBytes(c.Name);
                if (name.Length == 0 || name.Length > 255) throw new ArgumentException($"bad column name {c.Name}");
                w.Write((byte)name.Length);
                w.Write(name);
                w.Write(c.Scale);
                w.Write(c.Offset);
            }
            int blockTicks = Math.Max(1, (int)Math.Round(blockSeconds * hz, MidpointRounding.AwayFromZero));
            var blocks = new List<(int first, int n, byte[] data)>();
            var raw = new MemoryStream();
            for (int first = 0; first < ticks; first += blockTicks) {
                int n = Math.Min(blockTicks, ticks - first);
                raw.SetLength(0);
                foreach (Column c in columns) {
                    long previous = 0;
                    for (int i = first; i < first + n; i++) {
                        long q = Quantize(c.Values[i], c.Scale, c.Offset);
                        WriteVarint(raw, ZigZag(q - previous));
                        previous = q;
                    }
                }
                blocks.Add((first, n, Deflate(raw.ToArray())));
            }
            w.Write((uint)ticks);
            w.Write((uint)blockTicks);
            w.Write((uint)blocks.Count);
            foreach (var b in blocks) {
                w.Write((uint)b.first);
                w.Write((uint)b.n);
                w.Write((uint)b.data.Length);
            }
            foreach (var b in blocks) w.Write(b.data);
            var events = new StringBuilder();
            foreach (string line in eventLines) events.Append(line).Append('\n');
            byte[] eventBytes = Deflate(Encoding.UTF8.GetBytes(events.ToString()));
            w.Write((uint)eventBytes.Length);
            w.Write(eventBytes);
            w.Flush();
            byte[] body = output.ToArray();
            uint crc = Crc32(body, body.Length);
            var result = new byte[body.Length + 4];
            Buffer.BlockCopy(body, 0, result, 0, body.Length);
            BitConverter.GetBytes(crc).CopyTo(result, body.Length);
            if (!BitConverter.IsLittleEndian) throw new PlatformNotSupportedException("crane-traj is little-endian");
            return result;
        }

        public static Data Decode(byte[] bytes) {
            if (bytes.Length < 16 || Encoding.ASCII.GetString(bytes, 0, 4) != "CTRJ")
                throw new InvalidDataException("not a crane-traj file");
            uint crc = BitConverter.ToUInt32(bytes, bytes.Length - 4);
            if (Crc32(bytes, bytes.Length - 4) != crc) throw new InvalidDataException("crc mismatch");
            using var r = new BinaryReader(new MemoryStream(bytes, 0, bytes.Length - 4));
            r.ReadBytes(4);
            ushort version = r.ReadUInt16();
            if (version != Version) throw new InvalidDataException($"crane-traj version {version} is not supported");
            r.ReadUInt16();
            var data = new Data { HeaderJson = Encoding.UTF8.GetString(r.ReadBytes((int)r.ReadUInt32())) };
            data.Hz = ReadNumber(data.HeaderJson, "hz");
            int ncols = r.ReadUInt16();
            for (int i = 0; i < ncols; i++) {
                string name = Encoding.UTF8.GetString(r.ReadBytes(r.ReadByte()));
                data.Columns.Add(new Column { Name = name, Scale = r.ReadDouble(), Offset = r.ReadDouble() });
            }
            data.Ticks = (int)r.ReadUInt32();
            data.BlockTicks = (int)r.ReadUInt32();
            int nblocks = (int)r.ReadUInt32();
            var index = new (int first, int n, int len)[nblocks];
            for (int b = 0; b < nblocks; b++) index[b] = ((int)r.ReadUInt32(), (int)r.ReadUInt32(), (int)r.ReadUInt32());
            foreach (var entry in index) {
                byte[] block = Inflate(r.ReadBytes(entry.len));
                int pos = 0;
                foreach (Column c in data.Columns) {
                    long previous = 0;
                    for (int i = 0; i < entry.n; i++) {
                        previous += UnZigZag(ReadVarint(block, ref pos));
                        c.Values.Add(previous * c.Scale + c.Offset);
                    }
                }
            }
            string events = Encoding.UTF8.GetString(Inflate(r.ReadBytes((int)r.ReadUInt32())));
            foreach (string line in events.Split('\n')) if (line.Trim().Length > 0) data.EventLines.Add(line);
            return data;
        }

        /// <summary>A top-level number from a flat-enough JSON header (enough for "hz").</summary>
        public static double ReadNumber(string json, string key) {
            string token = $"\"{key}\":";
            int at = json.IndexOf(token, StringComparison.Ordinal);
            if (at < 0) {
                token = $"\"{key}\": ";
                at = json.IndexOf(token, StringComparison.Ordinal);
            }
            if (at < 0) return double.NaN;
            int start = at + token.Length, end = start;
            while (end < json.Length && "0123456789+-.eE ".IndexOf(json[end]) >= 0) end++;
            return double.Parse(json.Substring(start, end - start).Trim(), CultureInfo.InvariantCulture);
        }

        public static ulong ZigZag(long n) => (ulong)((n << 1) ^ (n >> 63));
        public static long UnZigZag(ulong z) => (long)(z >> 1) ^ -(long)(z & 1);

        private static void WriteVarint(Stream s, ulong z) {
            while (z >= 0x80) {
                s.WriteByte((byte)((z & 0x7F) | 0x80));
                z >>= 7;
            }
            s.WriteByte((byte)z);
        }

        private static ulong ReadVarint(byte[] b, ref int pos) {
            ulong z = 0;
            int shift = 0;
            while (true) {
                if (pos >= b.Length) throw new InvalidDataException("block shorter than its index says");
                byte x = b[pos++];
                z |= (ulong)(x & 0x7F) << shift;
                shift += 7;
                if (x < 0x80) return z;
            }
        }

        private static byte[] Deflate(byte[] data) {
            using var output = new MemoryStream();
            using (var d = new DeflateStream(output, CompressionLevel.Optimal, true)) d.Write(data, 0, data.Length);
            return output.ToArray();
        }

        private static byte[] Inflate(byte[] data) {
            using var input = new DeflateStream(new MemoryStream(data), CompressionMode.Decompress);
            using var output = new MemoryStream();
            input.CopyTo(output);
            return output.ToArray();
        }

        private static uint[] s_CrcTable;

        public static uint Crc32(byte[] data, int length) {
            if (s_CrcTable == null) {
                var table = new uint[256];
                for (uint n = 0; n < 256; n++) {
                    uint c = n;
                    for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                    table[n] = c;
                }
                s_CrcTable = table;
            }
            uint crc = 0xFFFFFFFFu;
            for (int i = 0; i < length; i++) crc = s_CrcTable[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
            return crc ^ 0xFFFFFFFFu;
        }

        /// <summary>A JSON string literal.</summary>
        public static string Str(string s) {
            if (s == null) return "null";
            var b = new StringBuilder("\"");
            foreach (char ch in s) {
                switch (ch) {
                    case '"': b.Append("\\\""); break;
                    case '\\': b.Append("\\\\"); break;
                    case '\n': b.Append("\\n"); break;
                    case '\r': b.Append("\\r"); break;
                    case '\t': b.Append("\\t"); break;
                    default:
                        if (ch < 0x20) b.Append("\\u").Append(((int)ch).ToString("x4"));
                        else b.Append(ch);
                        break;
                }
            }
            return b.Append('"').ToString();
        }

        /// <summary>A JSON number (null when not finite).</summary>
        public static string Num(double v) =>
            double.IsNaN(v) || double.IsInfinity(v) ? "null" : v.ToString("R", CultureInfo.InvariantCulture);
    }
}
