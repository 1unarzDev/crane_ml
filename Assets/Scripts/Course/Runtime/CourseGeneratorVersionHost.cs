using System.IO;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Tells <see cref="CourseIdentity"/> which generator this is. The editor reads the
    /// checkout (source hash, git commit); a player reads the Resources file
    /// <c>CourseGeneratorVersion</c> that CranePerformanceBuild writes from the checkout it builds
    /// (Assets/Generated, ignored by git and removed after the build).</summary>
    public static class CourseGeneratorVersionHost {
        public const string ResourceName = "CourseGeneratorVersion";
        public const string ResourceDir = "Assets/Generated/Resources";

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#endif
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void Install() {
            CourseIdentity.Resolve = Load;
            CourseIdentity.Current = null;
        }

        static GeneratorVersion Load() {
#if UNITY_EDITOR
            return CourseIdentity.FromCheckout(Path.GetFullPath(Path.Combine(Application.dataPath, "..")));
#else
            var text = Resources.Load<TextAsset>(ResourceName);
            return text != null ? GeneratorVersion.FromJson((JObj)Json.Parse(text.text)) : new GeneratorVersion();
#endif
        }
    }
}
