// EXPLORATORY ideal simulated sensors; calibration/independence unqualified.
// No reference velocity, injected cause, configured force or event schedule emitted.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.MainVenueV8 {
    public sealed class WheelContactV8 : MonoBehaviour {
        readonly HashSet<Collider> contacts = new HashSet<Collider>();
        public Rigidbody robot;
        void OnCollisionEnter(Collision c) { Observe(c); }
        void OnCollisionStay(Collision c) { Observe(c); }
        void Observe(Collision c) { if (c.rigidbody != robot && !c.transform.IsChildOf(robot.transform)) contacts.Add(c.collider); }
        void OnCollisionExit(Collision c) { contacts.Remove(c.collider); }
        public bool Touching { get { contacts.RemoveWhere(c => c == null); return contacts.Count > 0; } }
    }

    [DefaultExecutionOrder(32000)]
    public sealed class CraneSensorRecorderV8 : MonoBehaviour {
        PhysicalBodyV8 body;
        Behaviour[] motors;
        EncoderChannelV8[] wheels;
        EncoderBindingV8[] bindings;
        Vector3 previousVelocity;
        bool previousValid, bumperActive;
        double previousTime;
        float forwardImpulse;
        StreamWriter stream, privateLog;
        public int EmittedRows { get; private set; }
        static string N(double value) {
            if (!CraneFaultRuntimeV8.Finite(value)) throw new InvalidOperationException("Nonfinite sensor output");
            return value.ToString("R", CultureInfo.InvariantCulture);
        }
        public void Configure(FaultSpecV8 spec, string path, StreamWriter privateEvents = null) {
            if (stream != null) throw new InvalidOperationException("Recorder already configured");
            body = PhysicalBodyV8.Resolve(gameObject);
            bindings = spec.encoders;
            wheels = CraneFaultRuntimeV8.ResolveWheels(body.component, bindings);
            foreach (var wheel in wheels) wheel.AttachContactObserver();
            motors = CraneFaultRuntimeV8.ResolveMotors(body.component, spec);
            stream = new StreamWriter(new FileStream(path, FileMode.CreateNew, FileAccess.Write));
            privateLog = privateEvents;
            previousVelocity = body.linearVelocity;
            previousValid = false;
        }
        void OnCollisionEnter(Collision c) { ObserveCollision(c); }
        void OnCollisionStay(Collision c) { ObserveCollision(c); }
        void ObserveCollision(Collision c) {
            if (body == null || !body.contactMassQualified) return;
            // Frontal bumper scope: ordinary floor support is not bumper contact.
            bool frontal = false;
            Vector3 receiverNormal = Vector3.zero;
            foreach (var point in c.contacts) {
                receiverNormal += point.normal;
                if (Vector3.Dot(point.normal, transform.forward) < -.25f) frontal = true;
            }
            if (!frontal) return;
            bumperActive = true;
            // PhysX shape ordering can invert Collision.impulse relative to the
            // receiver. Contact normals point toward this receiver; orient the
            // measured solver impulse into that frame, without using IMU/velocity.
            Vector3 receiverImpulse = c.impulse;
            if (Vector3.Dot(receiverImpulse, receiverNormal) < 0) receiverImpulse = -receiverImpulse;
            forwardImpulse += Vector3.Dot(receiverImpulse, transform.forward);
        }
        void FixedUpdate() {
            if (stream != null) Capture(Time.fixedTimeAsDouble - Time.fixedDeltaTime);
        }
        public void Capture(double completedPhysicsTime) {
            if (stream == null || !CraneFaultRuntimeV8.Finite(completedPhysicsTime) || completedPhysicsTime < 0) throw new InvalidOperationException("Invalid capture state/time");
            if (!previousValid) {
                previousVelocity = body.linearVelocity; previousTime = completedPhysicsTime; previousValid = true;
                bumperActive = false; forwardImpulse = 0; return;
            }
            double dt = completedPhysicsTime - previousTime;
            if (dt <= 0) throw new InvalidOperationException("Sensor time did not advance");
            Vector3 properAcceleration = (body.linearVelocity - previousVelocity) / (float)dt - UnityEngine.Physics.gravity;
            Emit("imu", "{\"acceleration\":\"m/s^2\",\"uncertainty\":\"m/s^2\"}", completedPhysicsTime,
                 "\"acceleration\":" + N(Vector3.Dot(properAcceleration, transform.forward)) + ",\"uncertainty\":0");
            if (body.contactMassQualified) Emit("contact", "{\"active\":\"bool\",\"forward_impulse\":\"N*s\",\"mass\":\"kg\",\"uncertainty\":\"N*s\"}", completedPhysicsTime,
                 "\"active\":" + (bumperActive ? "true" : "false") + ",\"forward_impulse\":" + N(forwardImpulse) + ",\"mass\":" + N(body.contactMass) + ",\"uncertainty\":0");
            bool outputDisabled = Array.Exists(motors, m => !m.enabled);
            if (motors.Length > 0) Emit("motor_status", "{\"fault\":\"enum\"}", completedPhysicsTime,
                                    "\"fault\":\"" + (outputDisabled ? "output_disabled" : "none") + "\"");
            if (wheels.Length > 0) {
                float surfaceSpeed = 0; bool allContact = true;
                for (int i = 0; i < wheels.Length; i++) {
                    surfaceSpeed += wheels[i].SurfaceSpeed;
                    allContact &= wheels[i].Touching;
                }
                Emit("encoders", "{\"speed\":\"m/s\",\"uncertainty\":\"m/s\",\"in_contact\":\"bool\"}", completedPhysicsTime,
                     "\"speed\":" + N(surfaceSpeed / wheels.Length) + ",\"uncertainty\":0,\"in_contact\":" + (allContact ? "true" : "false"));
            }
            previousVelocity = body.linearVelocity; previousTime = completedPhysicsTime;
            bumperActive = false; forwardImpulse = 0;
            stream.Flush();
        }
        void Emit(string channel, string units, double time, string fields) {
            stream.WriteLine("{\"schema\":\"crane-unity-sensor-event/v8\",\"channel\":\"" + channel + "\",\"source_id\":\"sensor/" + channel + "\",\"clock_id\":\"unity-fixed\",\"units\":" + units + ",\"sample\":{\"t\":" + N(time) + "," + fields + "}}");
            EmittedRows++;
        }
        public void Close() {
            if (stream == null) return;
            GetComponent<CraneFaultRuntimeV8>()?.Stop();
            stream.Flush(); stream.Dispose(); stream = null;
            privateLog?.Dispose(); privateLog = null;
        }
        void OnDestroy() { Close(); }
        void OnApplicationQuit() { Close(); }
    }
}
