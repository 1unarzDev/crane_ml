using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Crane.Theater.EditorTools {
    /// <summary>Run a Theater capture inside the Editor, headless:
    ///   unity-drones --render LOG -executeMethod Crane.Theater.EditorTools.TheaterEditorCapture.Run
    ///     --crane-scene "Scene Name" --crane-theater MANIFEST --crane-theater-out DIR [other --crane-theater-* flags]
    /// Opens the scene named by --crane-scene (no build-settings entry needed), enters play mode, lets
    /// TheaterBoot (RuntimeInitializeOnLoadMethod) render, and exits the Editor with the Theater's exit code
    /// when it logs CRANE_THEATER_EXIT. Application.Quit is ignored in the Editor, hence the log hook.
    /// No player build is needed, so scene, prefab and script changes are picked up after a normal compile.</summary>
    [InitializeOnLoad]
    public static class TheaterEditorCapture {
        static int exitCode = int.MinValue;

        static string Arg(string key) {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++) if (a[i] == key) return a[i + 1];
            return null;
        }

        static TheaterEditorCapture() {
            // runs after every domain reload (entering play mode reloads); only active for a capture launch
            if (Arg("--crane-theater") == null || !Application.isBatchMode) return;
            Application.logMessageReceived += OnLog;
            EditorApplication.update += Tick;
        }

        public static void Run() {
            string name = Arg("--crane-scene");
            if (string.IsNullOrEmpty(name)) throw new ArgumentException("--crane-scene NAME is required");
            string path = null;
            foreach (string guid in AssetDatabase.FindAssets("t:Scene " + name)) {
                string p = AssetDatabase.GUIDToAssetPath(guid);
                if (Path.GetFileNameWithoutExtension(p).Equals(name, StringComparison.OrdinalIgnoreCase)) { path = p; break; }
            }
            if (path == null) throw new FileNotFoundException("scene not found: " + name);
            EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
            Debug.Log("CRANE_THEATER_EDITOR opening " + path + " and entering play mode");
            EditorApplication.EnterPlaymode();
        }

        static void OnLog(string msg, string stack, LogType type) {
            const string key = "CRANE_THEATER_EXIT code=";
            int i = msg.IndexOf(key, StringComparison.Ordinal);
            if (i >= 0 && int.TryParse(msg.Substring(i + key.Length).Trim(), out int c)) exitCode = c;
        }

        static void Tick() {
            if (exitCode == int.MinValue) return;
            int c = exitCode;
            exitCode = int.MinValue;
            EditorApplication.Exit(c);
        }
    }
}
