#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class CraneShoreVisualInspection {
    [Serializable] private sealed class Item {
        public string name, type, asset, material;
        public Vector3 position, size, center;
        public float minHeight, maxHeight;
    }
    [Serializable] private sealed class Report { public List<Item> items = new(); }
    public static void Inspect() {
        EditorSceneManager.OpenScene("Assets/Scenes/Roboboat Course.unity");
        var report = new Report();
        foreach (var terrain in UnityEngine.Object.FindObjectsByType<Terrain>(FindObjectsInactive.Include)) {
            var data = terrain.terrainData;
            var heights = data.GetHeights(0, 0, data.heightmapResolution, data.heightmapResolution);
            float min = float.PositiveInfinity, max = float.NegativeInfinity;
            foreach (float h in heights) { min = Mathf.Min(min,h); max = Mathf.Max(max,h); }
            report.items.Add(new Item {name=terrain.name, type="Terrain", asset=AssetDatabase.GetAssetPath(data),
                position=terrain.transform.position, size=data.size, minHeight=min*data.size.y,
                maxHeight=max*data.size.y, material=AssetDatabase.GetAssetPath(terrain.materialTemplate)});
        }
        foreach (var renderer in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsInactive.Include)) {
            string path = PathOf(renderer.transform);
            if (path.IndexOf("dock",StringComparison.OrdinalIgnoreCase)<0 && path.IndexOf("ocean",StringComparison.OrdinalIgnoreCase)<0) continue;
            report.items.Add(new Item {name=path,type=renderer.GetType().Name,position=renderer.transform.position,
                center=renderer.bounds.center,size=renderer.bounds.size,material=AssetDatabase.GetAssetPath(renderer.sharedMaterial)});
        }
        foreach (var collider in UnityEngine.Object.FindObjectsByType<Collider>(FindObjectsInactive.Include)) {
            string path=PathOf(collider.transform);
            if(path.IndexOf("dock",StringComparison.OrdinalIgnoreCase)<0)continue;
            report.items.Add(new Item {name=path,type=collider.GetType().Name,position=collider.transform.position,
                center=collider.bounds.center,size=collider.bounds.size});
        }
        string output=Path.GetFullPath("../../artifacts/nav2-docking-reproduction-20261002/shore-inspection.json");
        Directory.CreateDirectory(Path.GetDirectoryName(output));
        File.WriteAllText(output,JsonUtility.ToJson(report,true));
        Debug.Log("CRANE_SHORE_INSPECTION_COMPLETE path="+output);
    }
    private static string PathOf(Transform value) {return value.parent==null?value.name:PathOf(value.parent)+"/"+value.name;}
    public static void ApplyDepthFade() {
        var scene=EditorSceneManager.OpenScene("Assets/Scenes/Roboboat Course.unity");
        Terrain shore=GameObject.Find("Shore").GetComponent<Terrain>();
        TerrainData source=shore.GetComponent<TerrainCollider>().terrainData;
        const string output="Assets/Terrain/ShoreVisualFade.asset";
        const float depth=20f, width=14f;
        TerrainData visualData=AssetDatabase.LoadAssetAtPath<TerrainData>(output);
        if(visualData==null) { visualData=UnityEngine.Object.Instantiate(source); AssetDatabase.CreateAsset(visualData,output); }
        else EditorUtility.CopySerialized(source,visualData);
        visualData.name="Shore Visual Fade";
        int resolution=source.heightmapResolution;
        float[,] heights=source.GetHeights(0,0,resolution,resolution);
        Vector3 size=source.size;
        for(int z=0;z<resolution;z++) for(int x=0;x<resolution;x++) {
            float px=x*size.x/(resolution-1), pz=z*size.z/(resolution-1);
            float edge=Mathf.Min(Mathf.Min(px,size.x-px),Mathf.Min(pz,size.z-pz));
            float edgeFade=1f-Mathf.SmoothStep(0f,1f,Mathf.Clamp01(edge/width));
            float localHeight=heights[z,x]*size.y;
            float worldHeight=shore.transform.position.y+localHeight;
            // Preserve dry shore; only submerge the underwater outer rim.
            float underwater=Mathf.SmoothStep(0f,1f,Mathf.InverseLerp(-0.15f,-0.65f,worldHeight));
            heights[z,x]=(localHeight+depth-depth*edgeFade*underwater)/(size.y+depth);
        }
        visualData.size=new Vector3(size.x,size.y+depth,size.z);
        visualData.SetHeights(0,0,heights);
        Transform existing=shore.transform.Find("Shore Fade Visual");
        GameObject visualObject=existing==null?new GameObject("Shore Fade Visual"):existing.gameObject;
        visualObject.transform.SetParent(shore.transform,false);
        visualObject.transform.localPosition=new Vector3(0,-depth,0);
        visualObject.layer=shore.gameObject.layer;
        Terrain visual=visualObject.GetComponent<Terrain>();
        if(visual==null)visual=visualObject.AddComponent<Terrain>();
        EditorUtility.CopySerialized(shore,visual);
        visual.terrainData=visualData;
        visual.drawHeightmap=true;
        shore.drawHeightmap=false;
        EditorUtility.SetDirty(visualData);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        if(shore.GetComponent<TerrainCollider>().terrainData!=source || visualObject.GetComponent<Collider>()!=null)
            throw new InvalidOperationException("Visual fade unexpectedly altered collider wiring.");
        Debug.Log("CRANE_SHORE_FADE_COMPLETE width=14 depth=20 originalCollider="+AssetDatabase.GetAssetPath(source)+" visual="+output);
    }
}
#endif
