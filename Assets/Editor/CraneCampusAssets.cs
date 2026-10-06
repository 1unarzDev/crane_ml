#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using Sim.Physics.Land;
public static class CraneCampusAssets {
    public static void Build() {
        AssetDatabase.Refresh();
        foreach(string guid in AssetDatabase.FindAssets("t:Texture2D",new[]{"Assets/Resources/CampusMaterials"})) {
            string path=AssetDatabase.GUIDToAssetPath(guid);var importer=AssetImporter.GetAtPath(path) as TextureImporter;
            if(importer==null)continue;
            if(path.Contains("_nor_gl_"))importer.textureType=TextureImporterType.NormalMap;
            // Small 1k campus maps stay resident. Dynamic procedural meshes must not
            // depend on an imported renderer's texture-streaming density metadata.
            importer.isReadable=false;importer.mipmapEnabled=true;importer.streamingMipmaps=false;importer.SaveAndReimport();
        }
        if(!AssetDatabase.IsValidFolder("Assets/Resources/CampusMaterials/Profiles"))AssetDatabase.CreateFolder("Assets/Resources/CampusMaterials","Profiles");
        var manifest=JsonUtility.FromJson<CampusManifest>(Resources.Load<TextAsset>(CraneIndustrialCampus.Resource).text);
        foreach(var profile in manifest.surfaceProfiles) {
            string path="Assets/Resources/CampusMaterials/Profiles/"+profile.id+".mat";
            var material=AssetDatabase.LoadAssetAtPath<Material>(path);
            if(material==null){material=new Material(Shader.Find("HDRP/Lit"));AssetDatabase.CreateAsset(material,path);}
            string texture=profile.id=="asphalt"?"asphalt_02":profile.id=="compacted-dirt"?"gravel":"concrete_floor_02";
            material.SetTexture("_BaseColorMap",Resources.Load<Texture2D>("CampusMaterials/"+texture+"_diff_1k"));
            material.SetTexture("_NormalMap",Resources.Load<Texture2D>("CampusMaterials/"+texture+"_nor_gl_1k"));
            CraneIndustrialCampus.ValidateVisualMaterial(material);EditorUtility.SetDirty(material);
        }
        string robotPath="Assets/Resources/CampusMaterials/Profiles/robot-opaque.mat";
        var robotMaterial=AssetDatabase.LoadAssetAtPath<Material>(robotPath);
        if(robotMaterial==null){robotMaterial=new Material(Shader.Find("HDRP/Lit"));AssetDatabase.CreateAsset(robotMaterial,robotPath);}
        robotMaterial.SetTexture("_BaseColorMap",null);robotMaterial.SetTexture("_NormalMap",null);robotMaterial.SetFloat("_SurfaceType",0);robotMaterial.SetFloat("_Smoothness",.3f);robotMaterial.SetColor("_BaseColor",new Color(.025f,.19f,.36f));
        CraneIndustrialCampus.ValidateVisualMaterial(robotMaterial);EditorUtility.SetDirty(robotMaterial);
        // Native material assets retain the exact normal/mask/instancing variants
        // required by runtime equipment; no reliance on stripped runtime-only variants.
        string equipmentFolder="Assets/Resources/CampusProps/Materials";
        if(!AssetDatabase.IsValidFolder(equipmentFolder))AssetDatabase.CreateFolder("Assets/Resources/CampusProps","Materials");
        foreach(string guid in AssetDatabase.FindAssets("t:Texture2D",new[]{"Assets/Resources/CampusProps"})) {
            string path=AssetDatabase.GUIDToAssetPath(guid);string name=System.IO.Path.GetFileNameWithoutExtension(path);
            if(!name.EndsWith("_diff_1k"))continue;
            string resource=name.Substring(0,name.Length-8);string materialPath=equipmentFolder+"/"+resource+".mat";
            var material=AssetDatabase.LoadAssetAtPath<Material>(materialPath);
            if(material==null){material=new Material(Shader.Find("HDRP/Lit"));AssetDatabase.CreateAsset(material,materialPath);}
            material.SetColor("_BaseColor",Color.white);material.SetTexture("_BaseColorMap",AssetDatabase.LoadAssetAtPath<Texture2D>(path));
            material.SetTexture("_NormalMap",Resources.Load<Texture2D>("CampusProps/"+resource+"_nor_gl_1k"));
            material.SetTexture("_MaskMap",Resources.Load<Texture2D>("CampusProps/"+resource+"_mask_1k"));
            material.SetFloat("_NormalScale",.55f);material.SetFloat("_SurfaceType",0);material.enableInstancing=true;
            CraneIndustrialCampus.ValidateVisualMaterial(material);EditorUtility.SetDirty(material);
        }
        AssetDatabase.SaveAssets();
        CranePerformanceBuild.BuildLinuxWorker();
    }
    public static void ValidateMaterials() {
        var material=new Material(Shader.Find("HDRP/Lit"));
        material.SetTexture("_NormalMap",Resources.Load<Texture2D>("CampusMaterials/concrete_floor_02_nor_gl_1k"));
        Debug.Log("CAMPUS_MATERIAL_BEFORE normal="+material.IsKeywordEnabled("_NORMALMAP"));
        CraneIndustrialCampus.ValidateVisualMaterial(material);
        if(!material.IsKeywordEnabled("_NORMALMAP"))throw new System.Exception("Campus normal-map material regression");
        Debug.Log("CAMPUS_MATERIAL_PASS normal="+material.IsKeywordEnabled("_NORMALMAP"));
        Object.DestroyImmediate(material);
    }
}
#endif
