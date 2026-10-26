using System.Collections.Generic;
using System.Text;

namespace Sim.Course {
    /// <summary>SplitMix64, our own so every runtime gives the same stream. <see cref="Fork"/>
    /// derives a substream from (seed, key) with a fixed FNV-1a-64, never GetHashCode, so a
    /// task's stream depends only on the course seed and its id, not on draw order.</summary>
    public sealed class CourseRng {
        public readonly ulong Seed;
        ulong state;

        public CourseRng(ulong seed) { Seed = seed; state = seed; }

        public ulong NextU64() {
            state += 0x9E3779B97F4A7C15UL;
            ulong z = state;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        /// <summary>Uniform in [0, 1) with 53 bits.</summary>
        public double NextDouble() => (NextU64() >> 11) * (1.0 / 9007199254740992.0);
        public double Range(double lo, double hi) => lo + (hi - lo) * NextDouble();
        public int Int(int lo, int hiInclusive) => lo + (int)(NextU64() % (ulong)(hiInclusive - lo + 1));
        public bool Chance(double p) => NextDouble() < p;
        public T Pick<T>(IReadOnlyList<T> items) => items[Int(0, items.Count - 1)];
        public double Sign() => Chance(0.5) ? 1.0 : -1.0;

        public CourseRng Fork(string key) => new CourseRng(Fnv1a64(Seed, key));

        /// <summary>FNV-1a-64 over the seed's 8 little-endian bytes, then the key's UTF-8.</summary>
        public static ulong Fnv1a64(ulong seed, string key) {
            ulong h = 14695981039346656037UL;
            for (int i = 0; i < 8; i++) { h ^= (seed >> (8 * i)) & 0xFF; h *= 1099511628211UL; }
            foreach (byte b in Encoding.UTF8.GetBytes(key)) { h ^= b; h *= 1099511628211UL; }
            return h;
        }
    }
}
