#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using UnityEditor;
using UnityEngine;
using Sim.Physics.Land;

/// <summary>Explicit offline migration of licensed render meshes, never physics.</summary>
public static class CraneCampusRobotAssets {
    [Serializable] class Sources { public Source[] files; }
    [Serializable] class Source { public string id,file,url,sha256; }
    public static void Import() {
        string directory=CraneIndustrialCampus.Arg(Environment.GetCommandLineArgs(),"--crane-campus-robot-source-dir","/tmp/crane-campus-references/turtlebot-visuals");
        float cellSize=float.Parse(CraneIndustrialCampus.Arg(Environment.GetCommandLineArgs(),"--crane-campus-robot-cell-meters","0.0008"),System.Globalization.CultureInfo.InvariantCulture);
        string suffix=CraneIndustrialCampus.Arg(Environment.GetCommandLineArgs(),"--crane-campus-robot-asset-suffix","");
        if(cellSize<.0001f||cellSize>.003f)throw new ArgumentOutOfRangeException("visual clustering cell");
        var sources=JsonUtility.FromJson<Sources>(File.ReadAllText("Assets/Resources/CampusRobot/SOURCE_LICENSE.json"));
        foreach(var source in sources.files) {
            byte[] bytes=File.ReadAllBytes(Path.Combine(directory,source.file));
            using var hash=SHA256.Create();string sha=BitConverter.ToString(hash.ComputeHash(bytes)).Replace("-","").ToLowerInvariant();
            if(sha!=source.sha256)throw new InvalidDataException("Robot visual source checksum: "+source.file);
            uint triangles=BitConverter.ToUInt32(bytes,80);
            if(bytes.LongLength!=84L+50L*triangles)throw new InvalidDataException("Binary STL size");
            var vertices=new List<Vector3>();var normals=new List<Vector3>();var counts=new List<int>();var indices=new List<int>();
            var clusters=new Dictionary<(Vector3Int,Vector3Int),int>();
            for(int t=0;t<triangles;t++) {
                int start=84+t*50;Vector3 normal=Convert(Read(bytes,start),1).normalized;int[] ids=new int[3];
                for(int v=0;v<3;v++) {
                    Vector3 point=Convert(Read(bytes,start+12+12*v),.001f);
                    var cell=new Vector3Int(Mathf.RoundToInt(point.x/cellSize),Mathf.RoundToInt(point.y/cellSize),Mathf.RoundToInt(point.z/cellSize));
                    var facing=new Vector3Int(Mathf.RoundToInt(normal.x*8),Mathf.RoundToInt(normal.y*8),Mathf.RoundToInt(normal.z*8));
                    if(!clusters.TryGetValue((cell,facing),out int id)){id=vertices.Count;clusters.Add((cell,facing),id);vertices.Add(Vector3.zero);normals.Add(Vector3.zero);counts.Add(0);}
                    vertices[id]+=point;normals[id]+=normal;counts[id]++;ids[v]=id;
                }
                if(ids[0]!=ids[1]&&ids[1]!=ids[2]&&ids[0]!=ids[2])indices.AddRange(new[]{ids[0],ids[2],ids[1]});
            }
            for(int i=0;i<vertices.Count;i++){vertices[i]/=counts[i];normals[i]=normals[i].normalized;}
            // Collapsed faces leave unused clusters; remove them before serialization.
            var compactVertices=new List<Vector3>();var compactNormals=new List<Vector3>();var remap=new Dictionary<int,int>();
            for(int i=0;i<indices.Count;i++) {
                int old=indices[i];
                if(!remap.TryGetValue(old,out int current)){current=compactVertices.Count;remap.Add(old,current);compactVertices.Add(vertices[old]);compactNormals.Add(normals[old]);}
                indices[i]=current;
            }
            var mesh=new Mesh{name="ROBOTIS Waffle "+source.id,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};
            mesh.SetVertices(compactVertices);mesh.SetNormals(compactNormals);mesh.SetTriangles(indices,0);mesh.RecalculateBounds();
            Bounds bounds=mesh.bounds;
            string path="Assets/Resources/CampusRobot/"+source.id+suffix+".asset";
            var existing=AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if(existing==null)AssetDatabase.CreateAsset(mesh,path);else{EditorUtility.CopySerialized(mesh,existing);EditorUtility.SetDirty(existing);UnityEngine.Object.DestroyImmediate(mesh);}
            Debug.Log($"CAMPUS_ROBOT_VISUAL id={source.id} originalTriangles={triangles} reducedTriangles={indices.Count/3} vertices={compactVertices.Count} bounds={bounds}");
        }
        AssetDatabase.SaveAssets();
    }
    static Vector3 Read(byte[] bytes,int index)=>new Vector3(BitConverter.ToSingle(bytes,index),BitConverter.ToSingle(bytes,index+4),BitConverter.ToSingle(bytes,index+8));
    static Vector3 Convert(Vector3 value,float scale)=>new Vector3(-value.y,value.z,value.x)*scale;
}
#endif
