using System;
using System.Linq;
using System.Reflection;
namespace UnityEngine { public static class Application { public static string dataPath = Environment.GetEnvironmentVariable("CRANE_ASSETS") ?? "/home/lunarz/worktrees/crane-drones/Assets"; } }
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
        public static void IsNull(object o, string m = "") { if (o != null) F("not null " + m); }
        public static void Greater(double a, double b, string m = "") { if (!(a > b)) F($"{a} !> {b} {m}"); }
        public static void GreaterOrEqual(double a, double b, string m = "") { if (!(a >= b)) F($"{a} !>= {b} {m}"); }
        public static void Less(double a, double b, string m = "") { if (!(a < b)) F($"{a} !< {b} {m}"); }
    }
}
static class Runner {
    static int Main(string[] args) {
        int fail = 0;
        foreach (var t in typeof(Runner).Assembly.GetTypes().Where(t => t.Namespace == "Crane.Drones.Autonomy.Tests"))
            foreach (var mi in t.GetMethods().Where(m => m.GetCustomAttribute<NUnit.Framework.TestAttribute>() != null)) {
                if (args.Length > 0 && !args.Contains(mi.Name)) continue;
                var sw = System.Diagnostics.Stopwatch.StartNew();
                try { mi.Invoke(Activator.CreateInstance(t), null); Console.WriteLine($"PASS {mi.Name} {sw.Elapsed.TotalSeconds:0.0}s"); }
                catch (TargetInvocationException e) { fail++; Console.WriteLine($"FAIL {mi.Name} {sw.Elapsed.TotalSeconds:0.0}s: {e.InnerException.Message}"); }
            }
        return fail;
    }
}
