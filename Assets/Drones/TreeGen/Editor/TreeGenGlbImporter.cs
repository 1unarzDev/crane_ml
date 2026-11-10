// SPDX-License-Identifier: GPL-3.0-or-later
using System.IO;
using UnityEditor.AssetImporters;

namespace Crane.Drones.TreeGenImport.Editor
{
    /// <summary>Imports TreeGen meshes copied into the project as *.tgglb (a TreeGen-specific extension, so this never
    /// claims generic .glb files). Main object: the Mesh (submesh 0 bark, 1 leaves).</summary>
    [ScriptedImporter(1, "tgglb")]
    public class TreeGenGlbImporter : ScriptedImporter
    {
        public override void OnImportAsset(AssetImportContext ctx)
        {
            var parts = TreeGenGlb.Read(ctx.assetPath);
            var dir = Path.GetFileName(Path.GetDirectoryName(ctx.assetPath));
            var mesh = TreeGenGlb.BuildMesh(parts, dir + "_" + Path.GetFileNameWithoutExtension(ctx.assetPath));
            ctx.AddObjectToAsset("mesh", mesh);
            ctx.SetMainObject(mesh);
        }
    }
}
