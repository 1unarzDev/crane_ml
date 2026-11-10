// SPDX-License-Identifier: GPL-3.0-or-later
// Minimal NUnit surface so Assets/Scripts/Drones/Harness/Tests compile and run outside Unity.
using System;
namespace NUnit.Framework {
    public class TestAttribute : Attribute { }
    public class AssertionException : Exception { public AssertionException(string m) : base(m) { } }
    public static class Assert {
        static void F(string m) => throw new AssertionException(m);
        public static void AreEqual(double e, double a, double tol, string m = "") { if (Math.Abs(e - a) > tol) F($"expected {e} +- {tol} got {a} {m}"); }
        public static void AreEqual(object e, object a, string m = "") { if (!Equals(e, a)) F($"expected {e} got {a} {m}"); }
        public static void AreNotEqual(object e, object a, string m = "") { if (Equals(e, a)) F($"unexpected equal {e} {m}"); }
        public static void IsTrue(bool c, string m = "") { if (!c) F("false: " + m); }
        public static void IsFalse(bool c, string m = "") { if (c) F("true: " + m); }
        public static void NotNull(object o, string m = "") { if (o == null) F("null " + m); }
        public static void Greater(double a, double b, string m = "") { if (!(a > b)) F($"{a} !> {b} {m}"); }
        public static void GreaterOrEqual(double a, double b, string m = "") { if (!(a >= b)) F($"{a} !>= {b} {m}"); }
        public static void Less(double a, double b, string m = "") { if (!(a < b)) F($"{a} !< {b} {m}"); }
    }
}
