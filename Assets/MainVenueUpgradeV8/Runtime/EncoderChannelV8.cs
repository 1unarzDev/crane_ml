// EXPLORATORY physical shaft/PhysX WheelCollider binding; never virtual reference speed.
using System;
using UnityEngine;

namespace Crane.MainVenueV8 {
    public sealed class EncoderChannelV8 {
        public EncoderBindingV8 descriptor;
        public Rigidbody robot, shaft;
        public WheelCollider wheel;
        public WheelContactV8 contact;
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
    }
    [Serializable] public sealed class MotorBindingV8 {
        public string path, component_type;
    }
}
