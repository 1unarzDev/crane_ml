// EXPLORATORY ideal 360-degree line-of-sight landmark range/bearing acquisition.
// No injected cause, command, encoder, body velocity or evaluator label is read.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.MainVenueV8 {
    [DefaultExecutionOrder(32000)]
    public sealed class LandmarkSensorRecorderV8 : MonoBehaviour {
        [Serializable] sealed class Measurement {public string schema="crane-landmark-observations/v8";public double t;public List<LandmarkObservationV8> observations;}
        GeometricLandmarkV8[] landmarks;
        readonly LandmarkMotionEstimatorV8 estimator=new LandmarkMotionEstimatorV8();
        StreamWriter measurements,stream;
        double previousTime=-1;
        bool configured;
        public int EmittedRows {get;private set;}
        public void Configure(GeometricLandmarkV8[] configured,string sensorPath,string measurementPath) {
            if(this.configured||configured==null||configured.Length<3)throw new ArgumentException("Fresh sensor and at least three explicit landmarks required");
            var ids=new HashSet<string>();foreach(var landmark in configured)
                if(landmark==null||!landmark.IsStatic||!ids.Add(landmark.id))throw new ArgumentException("Unique static landmark bindings required");
            if(sensorPath==measurementPath||File.Exists(sensorPath)||File.Exists(measurementPath))throw new IOException("Distinct new-only landmark output files required");
            landmarks=configured;
            measurements=new StreamWriter(new FileStream(measurementPath,FileMode.CreateNew,FileAccess.Write));
            try {stream=new StreamWriter(new FileStream(sensorPath,FileMode.CreateNew,FileAccess.Write));}
            catch {measurements.Dispose();measurements=null;throw;}
            this.configured=true;
        }
        void FixedUpdate() {if(stream!=null)Capture(Time.fixedTimeAsDouble-Time.fixedDeltaTime);}
        public void Capture(double time) {
            if(stream==null||!CraneFaultRuntimeV8.Finite(time)||time<0||time<=previousTime)throw new ArgumentException("Advancing configured landmark sensor time required");
            previousTime=time;
            var observations=new List<LandmarkObservationV8>();
            // This model is planar; never silently project a tilted camera.
            if(Vector3.Dot(transform.up,Vector3.up)>.99999f)foreach(var landmark in landmarks) {
                LandmarkObservationV8 observation;if(landmark.Observe(transform,20,out observation))observations.Add(observation);
            }
            measurements.WriteLine(JsonUtility.ToJson(new Measurement {t=time,observations=observations}));measurements.Flush();
            var estimate=estimator.Observe(time,observations);
            if(estimate==null)return;
            // uncertainty=0 states this explicit ideal noiseless geometry model;
            // it is not a measured calibration bound or a fit-residual surrogate.
            stream.WriteLine("{\"schema\":\"crane-unity-sensor-event/v8\",\"channel\":\"independent_motion\",\"source_id\":\"sensor/independent_motion\",\"clock_id\":\"unity-fixed\",\"units\":{\"speed\":\"m/s\",\"uncertainty\":\"m/s\"},\"sample\":{\"t\":"+time.ToString("R",CultureInfo.InvariantCulture)+",\"speed\":"+estimate.forward_mps.ToString("R",CultureInfo.InvariantCulture)+",\"uncertainty\":0}}");
            stream.Flush();EmittedRows++;
        }
        public void Close() {stream?.Dispose();measurements?.Dispose();stream=null;measurements=null;}
        void OnDestroy() {Close();}
        void OnApplicationQuit() {Close();}
    }
}
