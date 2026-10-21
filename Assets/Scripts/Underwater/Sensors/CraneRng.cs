using System;

namespace Sim.Underwater {
    /// <summary>
    /// Deterministic SplitMix64 + Box-Muller generator. Not System.Random (stream is specified here, not by the runtime),
    /// so a given (seed, call sequence) reproduces bit-for-bit across machines.
    /// </summary>
    public sealed class CraneRng {
        ulong state;
        double spare; bool hasSpare;

        public CraneRng(ulong seed) { Reseed(seed); }

        public void Reseed(ulong seed) { state = seed; hasSpare = false; spare = 0; }

        public static ulong Mix(long episodeSeed, int streamId) {
            ulong z = (ulong)episodeSeed * 0x9E3779B97F4A7C15UL + (ulong)(streamId + 1) * 0xD1B54A32D192ED03UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        ulong NextU64() {
            ulong z = (state += 0x9E3779B97F4A7C15UL);
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        /// <summary>Uniform in (0, 1).</summary>
        public double NextDouble() => ((NextU64() >> 11) + 0.5) * (1.0 / 9007199254740992.0);

        public double NextGaussian() {
            if (hasSpare) { hasSpare = false; return spare; }
            double u1 = NextDouble(), u2 = NextDouble();
            double r = Math.Sqrt(-2.0 * Math.Log(u1)), a = 2.0 * Math.PI * u2;
            spare = r * Math.Sin(a); hasSpare = true;
            return r * Math.Cos(a);
        }
    }
}
