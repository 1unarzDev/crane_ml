#if UNITY_EDITOR
using UnityEditor;

/// <summary>Only CC0 workshop maps; no project/global texture settings.</summary>
public sealed class CraneCampusPropTextureImport : AssetPostprocessor
{
    void OnPreprocessTexture()
    {
        if (!assetPath.StartsWith("Assets/Resources/CampusProps/")) return;
        var importer = (TextureImporter)assetImporter;
        importer.maxTextureSize = 1024;
        importer.mipmapEnabled = true;
        importer.textureCompression = TextureImporterCompression.Compressed;
        if (assetPath.Contains("_nor_gl_"))
        {
            importer.textureType = TextureImporterType.NormalMap;
            importer.sRGBTexture = false;
        }
        else if (assetPath.Contains("_mask_") || assetPath.Contains("_metal_") || assetPath.Contains("_rough_"))
            importer.sRGBTexture = false;
        else importer.sRGBTexture = true;
    }
}

#endif
