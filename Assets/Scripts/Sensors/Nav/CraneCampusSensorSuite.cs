using System;
using System.Reflection;
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils.ROS;

namespace Sim.Sensors.Nav {
    /// <summary>Campus-only IMU and virtual wheel odometry. Ground-truth odom/TF remain distinct.</summary>
    public sealed class CraneCampusSensorSuite:MonoBehaviour {
        Rigidbody body;ROSPublisher imuPublisher,odomPublisher;System.Random random;
        Component model;FieldInfo wheelPosition,wheelYaw,wheelLinear,wheelAngular;Vector3 previousVelocity,acceleration,bias;
        float gyroSigma=.002f,accelSigma=.02f;bool errors;
        public void Configure(int seed,bool noisy) {
            body=GetComponent<Rigidbody>();errors=noisy;random=new System.Random(seed+401);
            model=GetComponent(Type.GetType("Sim.Physics.Land.CraneCampusRobotModel, PhysicsAssembly"));
            if(model!=null){wheelPosition=model.GetType().GetField("WheelPosition");wheelYaw=model.GetType().GetField("WheelYaw");wheelLinear=model.GetType().GetField("WheelLinearSpeed");wheelAngular=model.GetType().GetField("WheelAngularSpeed");}
            bias=noisy?new Vector3(.0005f,-.0003f,.0002f):Vector3.zero;
            imuPublisher=gameObject.AddComponent<ROSPublisher>();imuPublisher.Initialize<ImuMsg>("/campus/imu","base_link",Imu,50);
            odomPublisher=gameObject.AddComponent<ROSPublisher>();odomPublisher.Initialize<OdometryMsg>("/campus/wheel_odom","odom",Odom,25);
        }
        void FixedUpdate(){if(body==null)return;acceleration=(body.linearVelocity-previousVelocity)/Time.fixedDeltaTime;previousVelocity=body.linearVelocity;}
        double Gaussian()=>Math.Sqrt(-2*Math.Log(Math.Max(1e-12,random.NextDouble())))*Math.Cos(2*Math.PI*random.NextDouble());
        Vector3 Noise(float sigma)=>errors?new Vector3((float)Gaussian(),(float)Gaussian(),(float)Gaussian())*sigma:Vector3.zero;
        static Vector3Msg Flu(Vector3 v)=>new Vector3Msg(v.z,-v.x,v.y);
        ImuMsg Imu(){Vector3 gyro=transform.InverseTransformDirection(body.angularVelocity)+bias+Noise(gyroSigma);
            Vector3 proper=transform.InverseTransformDirection(acceleration-UnityEngine.Physics.gravity)+Noise(accelSigma);
            return new ImuMsg{header=imuPublisher.CreateHeader(),orientation=body.rotation.To<FLU>(),angular_velocity=Flu(gyro),linear_acceleration=Flu(proper),
                angular_velocity_covariance=new double[]{errors?gyroSigma*gyroSigma:0,0,0,0,errors?gyroSigma*gyroSigma:0,0,0,0,errors?gyroSigma*gyroSigma:0},linear_acceleration_covariance=new double[]{errors?accelSigma*accelSigma:0,0,0,0,errors?accelSigma*accelSigma:0,0,0,0,errors?accelSigma*accelSigma:0}};}
        OdometryMsg Odom(){Vector3 p=model!=null?(Vector3)wheelPosition.GetValue(model):body.position;
            Quaternion q=model!=null?Quaternion.Euler(0,(float)wheelYaw.GetValue(model)*Mathf.Rad2Deg,0):body.rotation;
            var msg=new OdometryMsg{header=odomPublisher.CreateHeader(),child_frame_id="base_link"};
            msg.pose.pose.position=new PointMsg(p.z,-p.x,p.y);msg.pose.pose.orientation=q.To<FLU>();
            msg.twist.twist.linear=new Vector3Msg(model!=null?(float)wheelLinear.GetValue(model):Vector3.Dot(body.linearVelocity,transform.forward),0,0);
            msg.twist.twist.angular=new Vector3Msg(0,0,model!=null?-(float)wheelAngular.GetValue(model):-body.angularVelocity.y);
            return msg;}
    }
}
