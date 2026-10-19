// EXPLORATORY ideal geometric landmark sensing; not a calibrated real sensor.
// The estimator accepts observations only, never a physics body/reference speed.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.MainVenueV8 {
    [Serializable] public sealed class LandmarkObservationV8 {
        public string id;
        public Vector3 relative_m;
    }
    public sealed class LandmarkMotionEstimateV8 {
        public double time_s;
        public float forward_mps, residual_m;
        public int matched_landmarks;
    }
    public sealed class LandmarkMotionEstimatorV8 {
        Dictionary<string,Vector3> previous;
        double previousTime;
        static Dictionary<string,Vector3> Validate(IList<LandmarkObservationV8> observations) {
            var result=new Dictionary<string,Vector3>();
            foreach(var observation in observations) {
                if(observation==null||string.IsNullOrWhiteSpace(observation.id)||!CraneFaultRuntimeV8.Finite(observation.relative_m.sqrMagnitude)||result.ContainsKey(observation.id))
                    throw new ArgumentException("Unique finite landmark observations required");
                result.Add(observation.id,observation.relative_m);
            }
            return result;
        }
        public LandmarkMotionEstimateV8 Observe(double time,IList<LandmarkObservationV8> observations) {
            if(!CraneFaultRuntimeV8.Finite(time)||time<0||observations==null)throw new ArgumentException("Finite sensor time/observations required");
            var current=Validate(observations);
            if(previous!=null&&time<=previousTime)throw new ArgumentException("Sensor time must advance");
            var old=previous;double dt=time-previousTime;
            previous=current;previousTime=time;
            if(old==null||dt>.5)return null; // no interpolation across missing coverage
            var ids=new List<string>();foreach(var id in current.Keys)if(old.ContainsKey(id))ids.Add(id);
            if(ids.Count<3)return null;
            Vector3 a=Vector3.zero,b=Vector3.zero;
            foreach(var id in ids) {a+=current[id];b+=old[id];}a/=ids.Count;b/=ids.Count;
            double dot=0,cross=0,xx=0,zz=0,xz=0;
            foreach(var id in ids) {
                Vector3 c=current[id]-a,p=old[id]-b;
                dot+=c.x*p.x+c.z*p.z;cross+=c.z*p.x-c.x*p.z;
                xx+=c.x*c.x;zz+=c.z*c.z;xz+=c.x*c.z;
            }
            if(xx*zz-xz*xz<1e-8||dot*dot+cross*cross<1e-12)return null;
            float yaw=(float)(Math.Atan2(cross,dot)*Mathf.Rad2Deg);
            Quaternion rotation=Quaternion.Euler(0,yaw,0);
            Vector3 displacement=b-rotation*a;
            float residual=0;
            foreach(var id in ids)residual=Mathf.Max(residual,(rotation*current[id]+displacement-old[id]).magnitude);
            // Reprojection is an outlier check, not a calibrated uncertainty bound.
            if(residual>.005f)return null;
            float forward=(Quaternion.Inverse(rotation)*displacement).z/(float)dt;
            if(!CraneFaultRuntimeV8.Finite(forward))return null;
            return new LandmarkMotionEstimateV8 {time_s=time,forward_mps=forward,residual_m=residual,matched_landmarks=ids.Count};
        }
    }
    public sealed class GeometricLandmarkV8 : MonoBehaviour {
        public string id;
        public BoxCollider panel;
        Vector3 position,scale;Quaternion rotation;
        bool configured;
        public void Bind(string identity,BoxCollider collider) {
            if(configured||string.IsNullOrWhiteSpace(identity)||collider==null||collider.transform!=transform||collider.attachedRigidbody!=null||GetComponentInParent<ArticulationBody>()!=null)
                throw new ArgumentException("Explicit static landmark panel required");
            id=identity;panel=collider;position=transform.position;rotation=transform.rotation;scale=transform.lossyScale;configured=true;
        }
        public bool IsStatic => configured&&panel!=null&&panel.enabled&&gameObject.activeInHierarchy&&panel.attachedRigidbody==null&&
            GetComponentInParent<ArticulationBody>()==null&&transform.position==position&&transform.rotation==rotation&&transform.lossyScale==scale;
        public Vector3 FeaturePoint => transform.TransformPoint(panel.center+Vector3.back*panel.size.z*.5f);
        public bool Observe(Transform sensor,float maxRange,out LandmarkObservationV8 observation) {
            observation=null;if(!IsStatic)return false;
            Vector3 feature=FeaturePoint,delta=feature-sensor.position;
            float distance=delta.magnitude;
            if(distance<=0||distance>maxRange||Vector3.Dot(-transform.forward,sensor.position-feature)<=0)return false;
            RaycastHit hit;
            if(!UnityEngine.Physics.Raycast(sensor.position,delta/distance,out hit,distance+.001f,~0,QueryTriggerInteraction.Collide)||hit.collider!=panel||(hit.point-feature).magnitude>.001f)return false;
            observation=new LandmarkObservationV8 {id=id,relative_m=sensor.InverseTransformDirection(delta/distance)*hit.distance};
            return true;
        }
    }
}
