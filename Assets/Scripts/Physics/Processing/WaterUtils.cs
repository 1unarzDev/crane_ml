using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using Unity.Mathematics;
using Sim.Utils.Performance;

namespace Sim.Physics.Processing {
    public static class WaterUtils {
        public static WaterSearchResult Search(WaterSurface targetSurface, Vector3 position, float error=0.01f, int maxIterations=8, bool includeDeformers=false, bool debug=false) {
            using var marker = CraneProfiler.WaterQuery.Auto();
            WaterSearchParameters searchParameters = new();
            WaterSearchResult searchResult = new();
           
            searchParameters.startPositionWS = position;
            searchParameters.targetPositionWS = position;
            searchParameters.error = error;
            searchParameters.maxIterations = maxIterations;
            searchParameters.includeDeformation = includeDeformers;
            searchParameters.excludeSimulation = false;

            // Can get both current and water height
            if (targetSurface.ProjectPointOnWaterSurface(searchParameters, out searchResult)) return searchResult;
            else {
                if (debug) Debug.LogWarning("Water search failed");
                return new WaterSearchResult();
            }
        }
    }
}
