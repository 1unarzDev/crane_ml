// EXPLORATORY bounded geometric acquisition. Coordinate error is ASSUMED.
// Separate schemas preserve old v8 observations and zero-uncertainty semantics.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.MainVenueV8 {
    [DefaultExecutionOrder(32000)]
    public sealed class LandmarkSensorRecorderV9 : MonoBehaviour {
        [Serializable] sealed class Measurement {
            public string schema="crane-landmark-observations/v9",model_id="conditional-planar-landmark/v9";
            public double t,position_error_bound_m;
            public List<LandmarkObservationV8> observations;
        }
        GeometricLandmarkV8[] landmarks;
        LandmarkMotionBoundedV9 estimator;
        StreamWriter measurements,stream;
        double previousTime=-1;
        bool configured;
        public int EmittedRows {get;private set;}
        public void Configure(GeometricLandmarkV8[] configured,string sensorPath,string measurementPath,double positionErrorBoundM) {
            if(this.configured||configured==null||configured.Length<3)throw new ArgumentException("Fresh sensor and three explicit static landmarks required");
            var ids=new HashSet<string>();foreach(var landmark in configured)
                if(landmark==null||!landmark.IsStatic||!ids.Add(landmark.id))throw new ArgumentException("Unique static landmark bindings required");
            estimator=new LandmarkMotionBoundedV9(positionErrorBoundM);
            if(sensorPath==measurementPath||File.Exists(sensorPath)||File.Exists(measurementPath))throw new IOException("Distinct new-only landmark outputs required");
            measurements=new StreamWriter(new FileStream(measurementPath,FileMode.CreateNew,FileAccess.Write));
            try {stream=new StreamWriter(new FileStream(sensorPath,FileMode.CreateNew,FileAccess.Write));}
            catch {measurements.Dispose();measurements=null;throw;}
            landmarks=(GeometricLandmarkV8[])configured.Clone();this.configured=true;
        }
        void FixedUpdate() {if(stream!=null)Capture(Time.fixedTimeAsDouble-Time.fixedDeltaTime);}
        static string Number(double value)=>value.ToString("R",CultureInfo.InvariantCulture);
        public void Capture(double time) {
            if(stream==null||!CraneFaultRuntimeV8.Finite(time)||time<0||time<=previousTime)throw new ArgumentException("Advancing configured landmark time required");
            previousTime=time;var observations=new List<LandmarkObservationV8>();
            if(Vector3.Dot(transform.up,Vector3.up)>.99999f)foreach(var landmark in landmarks) {
                LandmarkObservationV8 observation;if(landmark.Observe(transform,20,out observation))observations.Add(observation);
            }
            measurements.WriteLine(JsonUtility.ToJson(new Measurement {t=time,position_error_bound_m=estimator.PositionErrorBoundM,observations=observations}));measurements.Flush();
            var result=estimator.Observe(time,observations);if(result==null)return;
            stream.WriteLine("{\"schema\":\"crane-unity-sensor-event/v9\",\"model_id\":\"conditional-planar-landmark/v9\",\"position_error_bound_m\":"+Number(estimator.PositionErrorBoundM)+",\"channel\":\"independent_motion\",\"source_id\":\"sensor/independent_motion/landmark-v9\",\"clock_id\":\"unity-fixed\",\"units\":{\"speed\":\"m/s\",\"uncertainty\":\"m/s\"},\"sample\":{\"t\":"+Number(time)+",\"speed\":"+Number(result.forward_mps)+",\"uncertainty\":"+Number(result.uncertainty_mps)+"}}");
            stream.Flush();EmittedRows++;
        }
        public void Close() {stream?.Dispose();measurements?.Dispose();stream=null;measurements=null;}
        void OnDestroy() {Close();}
        void OnApplicationQuit() {Close();}
    }
}
