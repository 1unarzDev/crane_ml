// EXPLORATORY physical shaft/PhysX WheelCollider binding; never virtual reference speed.
using System;
using UnityEngine;

namespace Crane.MainVenueV8 {
    public sealed class EncoderChannelV8 {
        public EncoderBindingV8 descriptor;
        public Rigidbody robot, shaft;
        public WheelCollider wheel;
        public WheelContactV8 contact;
        bool previousValid;
        double previousTime;
        Quaternion previousRotation;
        public static EncoderChannelV8 Resolve(Rigidbody robot, EncoderBindingV8 binding) {
            var child = robot.transform.Find(binding.path);
            if (child == null) throw new ArgumentException("Bound physical wheel is absent");
            string kind = string.IsNullOrEmpty(binding.kind) ? "hinge" : binding.kind;
            var result = new EncoderChannelV8 { robot = robot, descriptor = binding };
            if (kind == "wheel_collider") {
                result.wheel = child.GetComponent<WheelCollider>();
                if (result.wheel == null || result.wheel.attachedRigidbody != robot || Math.Abs(result.wheel.radius - binding.radius_m) > 1e-5)
                    throw new ArgumentException("WheelCollider/root/radius binding mismatch");
            } else if (kind == "hinge") {
                result.shaft = child.GetComponent<Rigidbody>();
                var joint = child.GetComponent<HingeJoint>();
                if (result.shaft == null || result.shaft.isKinematic || joint == null || joint.connectedBody != robot || child.GetComponent<Collider>() == null)
                    throw new ArgumentException("Encoder requires a real robot-connected contact wheel");
            } else throw new ArgumentException("Unknown physical encoder binding kind");
            return result;
        }
        public void AttachContactObserver() {
            if (shaft != null) { contact = shaft.gameObject.AddComponent<WheelContactV8>(); contact.robot = robot; }
        }
        public float SurfaceSpeed => wheel != null ? wheel.rpm * (2 * Mathf.PI / 60) * wheel.radius * descriptor.polarity :
            Vector3.Dot(shaft.angularVelocity - robot.angularVelocity, shaft.transform.TransformDirection(descriptor.axis_local)) * descriptor.radius_m * descriptor.polarity;
        public bool Touching => wheel != null ? wheel.isGrounded : contact != null && contact.Touching;
        public bool ObserveInterval(double time,out float speed) {
            speed=0;
            if(!CraneFaultRuntimeV8.Finite(time)||time<0||(previousValid&&time<=previousTime))throw new ArgumentException("Advancing encoder time required");
            Quaternion rotation;Vector3 axis;
            if(wheel!=null) {
                Vector3 position;Quaternion world;
                wheel.GetWorldPose(out position,out world);
                var steeringFrame=wheel.transform.rotation*Quaternion.Euler(0,wheel.steerAngle,0);
                rotation=Quaternion.Inverse(steeringFrame)*world;axis=Vector3.right;
            } else {
                rotation=Quaternion.Inverse(robot.rotation)*shaft.rotation;
                axis=previousValid?previousRotation*descriptor.axis_local:rotation*descriptor.axis_local;
            }
            bool ready=previousValid;double dt=time-previousTime;
            var prior=previousRotation;previousRotation=rotation;previousTime=time;previousValid=true;
            if(!ready||dt>.5)return false;
            Quaternion delta=rotation*Quaternion.Inverse(prior);float degrees;Vector3 deltaAxis;
            delta.ToAngleAxis(out degrees,out deltaAxis);if(degrees>180)degrees-=360;
            if(!CraneFaultRuntimeV8.Finite(deltaAxis.sqrMagnitude))return false;
            // Reject an observed high-rate regime. End-of-interval RPM alone
            // cannot prove a between-sample angular bound; that remains part
            // of sensor qualification, not a guarantee from this guard.
            if(Mathf.Abs(SurfaceSpeed)*dt/descriptor.radius_m>=Mathf.PI)return false;
            speed=degrees*Mathf.Deg2Rad*Vector3.Dot(deltaAxis,axis)*descriptor.radius_m*descriptor.polarity/(float)dt;
            return CraneFaultRuntimeV8.Finite(speed);
        }
    }
    [Serializable] public sealed class MotorBindingV8 {
        public string path, component_type;
    }
}
