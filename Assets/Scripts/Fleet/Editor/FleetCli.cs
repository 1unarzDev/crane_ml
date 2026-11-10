using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Crane.Fleet.Sim;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Crane.Fleet.EditorTools {
    /// <summary>Edit-mode runs get their own physics scene: a preview scene per run (FleetPhysicsScene).</summary>
    [InitializeOnLoad]
    static class FleetEditorHooks {
        static FleetEditorHooks() {
            FleetPhysicsScene.EditorFactory = name => EditorSceneManager.NewPreviewScene();
            FleetPhysicsScene.EditorClose = scene => EditorSceneManager.ClosePreviewScene(scene);
        }
    }

    /// <summary>
    /// Batch-mode entry points (-executeMethod):
    ///   Crane.Fleet.EditorTools.FleetCli.Run          seeded run in a fresh empty scene (--crane-seed N, --crane-fleet-out DIR)
    ///   Crane.Fleet.EditorTools.FleetCli.Check        CAD-swap check (--crane-fleet-check REPORT.json)
    ///   Crane.Fleet.EditorTools.FleetCli.BuildPlayer  Linux player whose only scene is the empty fleet scene
    /// Spec directory: --crane-fleet-spec DIR (default Assets/StreamingAssets/Fleet).
    /// </summary>
    public static class FleetCli {
        public const string ScenePath = "Assets/Scenes/Fleet/FleetEmpty.unity";
        const string DefaultSpec = "Assets/StreamingAssets/Fleet";

        static FleetSpec Spec() => FleetSpec.LoadDirectory(FleetArgs.SpecDir(DefaultSpec));

        static void Exit(int code) {
            Debug.Log($"CRANE_FLEET_EXIT code={code}");
            if (Application.isBatchMode) EditorApplication.Exit(code);
        }

        public static void Run() {
            int code = 0;
            try {
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var spec = Spec();
                var opt = FleetArgs.RunOptions("editor", spec);
                if (string.IsNullOrEmpty(opt.SourceCommit)) (opt.SourceCommit, opt.SourceDirty) = Git();
                var run = new FleetRun(spec, opt);
                run.RunToEnd();
                Debug.Log($"CRANE_FLEET_RUN_END id={run.RunId} reason={run.EndReason} t={run.T} steps={run.Step} dir={run.Dir}");
                run.Dispose();
            }
            catch (Exception e) { Debug.LogError("CRANE_FLEET_FAILED " + e); code = 3; }
            Exit(code);
        }

        /// <summary>Several jobs in one editor launch: --crane-fleet-batch "check=REPORT;run=SEED:DIR;run=SEED:DIR".
        /// Each run starts from a fresh empty scene. Exit code 0 only if every job succeeded and checks passed.</summary>
        public static void Batch() {
            int code = 0;
            var (commit, dirty) = Git();
            foreach (string job in FleetArgs.Get("--crane-fleet-batch", "").Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries)) {
                try {
                    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                    int eq = job.IndexOf('=');
                    string kind = job.Substring(0, eq), arg = job.Substring(eq + 1);
                    if (kind == "check") {
                        var r = CadSwapCheck.Run(Spec(), arg);
                        Debug.Log($"CRANE_FLEET_CHECK pass={r.Pass} report={arg} failures={r.Failures.Count}");
                        foreach (string f in r.Failures) Debug.Log("CRANE_FLEET_CHECK_FAIL " + f);
                        if (!r.Pass) code = 1;
                    }
                    else if (kind == "run") {
                        int colon = arg.IndexOf(':');
                        var spec = Spec();
                        var opt = FleetArgs.RunOptions("editor", spec);
                        opt.Seed = long.Parse(arg.Substring(0, colon));
                        opt.OutDir = arg.Substring(colon + 1);
                        if (string.IsNullOrEmpty(opt.SourceCommit)) { opt.SourceCommit = commit; opt.SourceDirty = dirty; }
                        var run = new FleetRun(spec, opt);
                        run.RunToEnd();
                        Debug.Log($"CRANE_FLEET_RUN_END id={run.RunId} reason={run.EndReason} t={run.T} steps={run.Step} dir={run.Dir} physics={run.Phys?.Describe}");
                        run.Dispose();
                    }
                    else throw new ArgumentException("unknown batch job " + kind);
                }
                catch (Exception e) { Debug.LogError("CRANE_FLEET_FAILED " + job + " " + e); code = 3; }
            }
            Exit(code);
        }

        /// <summary>Paired-run harness (headless): --crane-fleet-jobs JOBS.json (see FleetHarness).</summary>
        public static void Jobs() {
            int code = 0;
            try {
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var (commit, dirty) = Git();
                var res = FleetHarness.RunHeadless(FleetArgs.SpecDir(DefaultSpec), FleetArgs.Get("--crane-fleet-jobs"), "editor", commit, dirty);
                Debug.Log($"CRANE_FLEET_JOBS n={res.Count} failed={res.FindAll(r => !r.Success).Count}");
            }
            catch (Exception e) { Debug.LogError("CRANE_FLEET_FAILED " + e); code = 3; }
            Exit(code);
        }

        public static void Check() {
            int code = 0;
            try {
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                string report = FleetArgs.Get("--crane-fleet-check", Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                    ".cache", "crane-fleet", "check", "cad_swap_check.json"));
                var r = CadSwapCheck.Run(Spec(), report);
                Debug.Log($"CRANE_FLEET_CHECK pass={r.Pass} report={report} failures={r.Failures.Count}");
                foreach (string f in r.Failures) Debug.Log("CRANE_FLEET_CHECK_FAIL " + f);
                code = r.Pass ? 0 : 1;
            }
            catch (Exception e) { Debug.LogError("CRANE_FLEET_FAILED " + e); code = 3; }
            Exit(code);
        }

        /// <summary>The empty scene the player boots into; everything else is built at runtime.</summary>
        public static void EnsureScene() {
            if (File.Exists(ScenePath)) return;
            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.Refresh();
        }

        /// <summary>A Lit material in Resources: the runtime colour template, and what makes a build keep Lit variants.</summary>
        public static void EnsureMaterial() {
            string path = "Assets/Resources/" + FleetGeometry.TemplateResource + ".mat";
            if (File.Exists(path)) return;
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var shader = Shader.Find("HDRP/Lit") ?? throw new Exception("HDRP/Lit shader not found");
            AssetDatabase.CreateAsset(new Material(shader) { name = "FleetLit" }, path);
            AssetDatabase.SaveAssets();
            Debug.Log("CRANE_FLEET_MATERIAL created " + path);
        }

        public static void BuildPlayer() {
            int code = 0;
            try {
                EnsureScene();
                EnsureMaterial();
                string output = FleetArgs.Get("--crane-build-output", "Builds/CRANE-Fleet/CRANE.x86_64");
                Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)) ?? ".");
                var options = new BuildPlayerOptions {
                    scenes = new[] { ScenePath }, locationPathName = output,
                    target = BuildTarget.StandaloneLinux64, options = BuildOptions.Development
                };
                BuildReport report = BuildPipeline.BuildPlayer(options);
                if (report.summary.result != BuildResult.Succeeded) throw new Exception("fleet player build failed: " + report.summary.result);
                Debug.Log($"CRANE_BUILD_COMPLETE path={output} bytes={report.summary.totalSize}");
            }
            catch (Exception e) { Debug.LogError("CRANE_FLEET_FAILED " + e); code = 3; }
            Exit(code);
        }

        public static (string, bool) Git() {
            try {
                string Run(string args) {
                    var p = Process.Start(new ProcessStartInfo("git", args) { RedirectStandardOutput = true, UseShellExecute = false, WorkingDirectory = Directory.GetCurrentDirectory() });
                    string s = p.StandardOutput.ReadToEnd();
                    p.WaitForExit();
                    return s.Trim();
                }
                return (Run("rev-parse HEAD"), Run("status --porcelain --untracked-files=no").Length > 0);
            }
            catch (Exception) { return ("", false); }
        }
    }
}
