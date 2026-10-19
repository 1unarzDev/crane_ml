// EXPLORATORY read-only asset/body diagnosis, never a repaired scene or pilot.
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Crane.MainVenueV8;

public static class CourseDiagnosisV8 {
    [Serializable] public sealed class Reference { public string guid, resolved_path; public bool exists; }
    [Serializable] public sealed class Actuator { public string path, component_type; }
    [Serializable] public sealed class Body { public string path, body_kind; public bool root, immovable; public float mass; }
    [Serializable] public sealed class Inspection {
        public string asset_path, source_sha256;
        public List<Reference> prefab_references=new List<Reference>();
        public List<Body> bodies=new List<Body>();
        public List<Actuator> actuators=new List<Actuator>();
        public List<string> missing_prefab_instances=new List<string>();
        public int missing_scripts;
    }
    [Serializable] public sealed class Report {
        public string status="EXPLORATORY_READ_ONLY_ASSET_DIAGNOSIS_NOT_RUNTIME_QUALIFICATION";
        public List<Inspection> inspections=new List<Inspection>();
    }
    static string Hash(string path) { using(var h=SHA256.Create()) using(var f=File.OpenRead(path)) return BitConverter.ToString(h.ComputeHash(f)).Replace("-","").ToLowerInvariant(); }
    static void Inspect(GameObject root,Inspection row) {
        foreach(var transform in root.GetComponentsInChildren<Transform>(true)) {
            var owner=transform.gameObject;
            if(PrefabUtility.IsPrefabAssetMissing(owner))row.missing_prefab_instances.Add(RuntimeReadinessV8.PathOf(transform));
            row.missing_scripts+=GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(owner);
            var rigid=owner.GetComponent<Rigidbody>();
            if(rigid!=null)row.bodies.Add(new Body {path=RuntimeReadinessV8.PathOf(transform),body_kind="Rigidbody",root=transform==root.transform,immovable=rigid.isKinematic,mass=rigid.mass});
            var articulation=owner.GetComponent<ArticulationBody>();
            if(articulation!=null)row.bodies.Add(new Body {path=RuntimeReadinessV8.PathOf(transform),body_kind="ArticulationBody",root=articulation.isRoot,immovable=articulation.immovable,mass=articulation.mass});
            foreach(var component in owner.GetComponents<MonoBehaviour>()) {
                if(component==null)continue;
                string type=component.GetType().FullName;
                if(type.EndsWith("Thruster",StringComparison.Ordinal)||type.EndsWith("OmniXController",StringComparison.Ordinal))
                    row.actuators.Add(new Actuator {path=RuntimeReadinessV8.PathOf(transform),component_type=type});
            }
        }
    }
    public static void Export() {
        string[] args=Environment.GetCommandLineArgs();int index=Array.IndexOf(args,"--crane-v8-diagnosis-output");
        if(index<0||index+1>=args.Length)throw new ArgumentException("Explicit fresh diagnosis output required");
        string output=Path.GetFullPath(args[index+1]);
        if(File.Exists(output))throw new IOException("Diagnosis output already exists");
        var report=new Report();
        const string scenePath="Assets/Scenes/Roboboat Course.unity";
        string before=Hash(scenePath);
        var scene=EditorSceneManager.OpenScene(scenePath,OpenSceneMode.Single);
        var source=new Inspection {asset_path=scenePath,source_sha256=before};
        var seen=new HashSet<string>();
        foreach(Match match in Regex.Matches(File.ReadAllText(scenePath),@"m_SourcePrefab: \{fileID: \d+, guid: ([0-9a-f]{32})")) {
            string guid=match.Groups[1].Value;if(!seen.Add(guid))continue;
            string path=AssetDatabase.GUIDToAssetPath(guid);
            source.prefab_references.Add(new Reference {guid=guid,resolved_path=path,exists=!string.IsNullOrEmpty(path)&&File.Exists(path)});
        }
        foreach(var root in scene.GetRootGameObjects())Inspect(root,source);
        if(Hash(scenePath)!=before)throw new IOException("Scene changed during diagnosis");
        report.inspections.Add(source);
        const string prefabPath="Assets/Robot Descriptions/omni_boat/omni_boat_simplified/OmniBoat.prefab";
        before=Hash(prefabPath);
        GameObject contents=PrefabUtility.LoadPrefabContents(prefabPath);
        try {
            var candidate=new Inspection {asset_path=prefabPath,source_sha256=before};
            Inspect(contents,candidate);report.inspections.Add(candidate);
        } finally {PrefabUtility.UnloadPrefabContents(contents);}
        if(Hash(prefabPath)!=before)throw new IOException("Prefab changed during diagnosis");
        using(var stream=new StreamWriter(new FileStream(output,FileMode.CreateNew,FileAccess.Write)))stream.WriteLine(JsonUtility.ToJson(report,true));
        Debug.Log("CRANE_V8_COURSE_DIAGNOSIS_WRITTEN "+output);
    }
}
