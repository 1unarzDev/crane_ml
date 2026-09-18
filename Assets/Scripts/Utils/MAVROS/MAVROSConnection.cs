using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.IO;
using UnityEngine;
using Sim.Utils;
using Sim.Controllers;
using Sim.Utils.Performance;
using Sim.Utils.ROS;

namespace Sim.Sensors.Nav
{
    [Serializable]
    public class SITLCommsJsonIMUData
    {
        public float[] gyro = new float[] { 0.0f, 0.0f, 0.0f };
        public float[] accel_body = new float[] { 0.0f, -Constants.gravity, 0.0f };
    }

    [Serializable]
    public class SITLCommsJsonOutputPacket
    {
        public float timestamp = 0;
        public SITLCommsJsonIMUData imu = new();
        public float[] position = new float[] { 0.0f, 0.0f, 0.0f };
        public float[] attitude = new float[] { 0.0f, 0.0f, 0.0f };
        public float[] velocity = new float[] { 0.0f, 0.0f, 0.0f };
    }

    public class MAVROSConnection : MonoBehaviour
    {
        private const UInt16 ServoPacketMagic = 18458;
        private const int ServoPacketBytes = sizeof(UInt16) * 18 + sizeof(UInt32);
        [Header("Unity References")]
        [SerializeField] private Imu imu;
        [SerializeField] private OmniXController controller;

        [Header("UDP Settings")]
        [SerializeField] private int localPort = 9002;

        [Header("PWM Settings")]
        [SerializeField] private float pwmMin;
        [SerializeField] private float pwmMax;

        [Header("Telemetry Settings")]
        [Tooltip("Telemetry update rate in Hz")]
        [SerializeField] private float hz = 200f;

        private UdpClient socketReceive;
        private UdpClient socketSend;
        private Thread receiveThread;
        private Thread sendThread;
        private volatile bool runThreads = true;
        private bool threadsStarted;
        private volatile bool hasRemoteConnection = false;
        private IPEndPoint remoteEndpoint = new(IPAddress.Any, 0);

        private SITLCommsJsonOutputPacket data = new();
        private readonly object dataLock = new();
        private volatile bool receiveError;
        private string receiveErrorMessage;
        private volatile bool sendError;
        private string sendErrorMessage;
        private volatile bool newConnection;
        private string remoteEndpointString;
        private readonly CraneQueuedAction<UInt16[]> pendingAction =
            new(CraneActionPayloadEncoding.UInt16Array);

        void Start()
        {
            if (imu == null || imu.body == null || controller == null)
                throw new MissingReferenceException(
                    $"{name} SITL bridge requires an initialized IMU and Omni-X controller.");
            if (pwmMax <= pwmMin)
                throw new InvalidOperationException(
                    $"{name} SITL PWM range must satisfy max > min.");
            if (hz <= 0f)
                throw new InvalidOperationException($"{name} SITL telemetry rate must be positive.");
            runThreads = true;
            Debug.Log($"Starting ArduPilot JSON/SITL UDP threads on port {localPort}");

            socketReceive = new UdpClient(localPort);
            socketReceive.Client.ReceiveTimeout = 500;
            socketSend = new UdpClient();

            receiveThread = new Thread(ReceiveDataLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            sendThread = new Thread(SendTelemetryLoop);
            sendThread.IsBackground = true;
            sendThread.Start();
            threadsStarted = true;

        }

        void OnDisable()
        {
            if (!threadsStarted) {
                pendingAction.Clear();
                return;
            }
            runThreads = false;

            try
            {
                socketReceive?.Close();
                socketSend?.Close();
            }
            catch { }

            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join(200);

            if (sendThread != null && sendThread.IsAlive)
                sendThread.Join(200);

            pendingAction.Clear();
            threadsStarted = false;

            Debug.Log("Stopping ArduPilot JSON/SITL UDP threads");
        }

        void Update()
        {
            if (sendError)
            {
                Debug.LogWarning($"Send telemetry error: {sendErrorMessage}");
                sendError = false;
            }

            if (newConnection)
            {
                Debug.Log($"New SITL connection from {remoteEndpointString}");
                newConnection = false;
            }

            if (receiveError)
            {
                Debug.LogWarning($"Receive loop error: {receiveErrorMessage}");
                receiveError = false;
            }

            // Update telemetry from Unity simulation
            lock (dataLock)
            {
                data.timestamp = (float)Clock.time;

                data.imu.gyro = new float[]
                {
                    imu.body.angularVelocity.z,
                    imu.body.angularVelocity.x,
                    imu.body.angularVelocity.y
                };

                data.imu.accel_body = new float[] { 0.0f, 0.0f, -Constants.gravity };

                data.position = new float[]
                {
                    imu.body.position.z,
                    imu.body.position.x,
                    imu.body.position.y
                };

                data.attitude = new float[]
                {
                    imu.body.transform.eulerAngles.z * Mathf.Deg2Rad,
                    imu.body.transform.eulerAngles.x * Mathf.Deg2Rad,
                    imu.body.transform.eulerAngles.y * Mathf.Deg2Rad
                };

                data.velocity = new float[]
                {
                    imu.body.linearVelocity.z,
                    imu.body.linearVelocity.x,
                    imu.body.linearVelocity.y
                };
            }
        }

        void FixedUpdate()
        {
            if (controller.movementOverride) {
                pendingAction.Clear();
                return;
            }
            pendingAction.TryApply(ApplyPwm, out _);
        }

        private void ApplyPwm(UInt16[] pwm) {
            controller.frontLeft.SetCommand(MapPWM(pwm[1]));
            controller.frontRight.SetCommand(MapPWM(pwm[2]));
            controller.rearRight.SetCommand(MapPWM(pwm[3]));
            controller.rearLeft.SetCommand(MapPWM(pwm[0]));
        }

        private float MapPWM(float pwm)
        {
            pwm = Math.Clamp(pwm, pwmMin, pwmMax);
            return controller.config.GetMinCommand() +
                   (pwm - pwmMin) * (controller.config.GetMaxCommand() - controller.config.GetMinCommand()) /
                   (pwmMax - pwmMin);
        }

        private void ReceiveDataLoop()
        {
            while (runThreads)
            {
                try
                {
                    byte[] received = socketReceive.Receive(ref remoteEndpoint);

                    if (received.Length != ServoPacketBytes) {
                        CraneRuntimeMetrics.ReportSitlServoPacket(false, -1);
                        receiveError = true;
                        receiveErrorMessage = $"invalid SITL servo packet length {received.Length}; " +
                                              $"expected {ServoPacketBytes}";
                        continue;
                    }

                    using var reader = new BinaryReader(new MemoryStream(received), Encoding.UTF8, false);
                    UInt16 magic = reader.ReadUInt16();
                    UInt16 frameRate = reader.ReadUInt16();
                    UInt32 frameCount = reader.ReadUInt32();
                    if (magic != ServoPacketMagic || frameRate == 0) {
                        CraneRuntimeMetrics.ReportSitlServoPacket(false, frameCount);
                        receiveError = true;
                        receiveErrorMessage = $"invalid SITL servo header magic={magic} " +
                                              $"frameRate={frameRate}";
                        continue;
                    }

                    if (!hasRemoteConnection)
                    {
                        hasRemoteConnection = true;

                        newConnection = true;
                        remoteEndpointString = remoteEndpoint.ToString();
                    }

                    UInt16[] pwm = new UInt16[16];
                    for (int i = 0; i < 16; i++)
                        pwm[i] = reader.ReadUInt16();

                    pendingAction.Receive(pwm, "sitl:pwm", frameCount, -1);
                    CraneRuntimeMetrics.ReportSitlServoPacket(true, frameCount);
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.TimedOut)
                        continue;
                    break;
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    receiveError = true;
                    receiveErrorMessage = ex.Message;
                }
            }
        }

        private void SendTelemetryLoop()
        {
            float intervalMs = 1000f / hz;
            while (runThreads)
            {
                if (hasRemoteConnection)
                {
                    try
                    {
                        SITLCommsJsonOutputPacket snapshot;
                        lock (dataLock)
                        {
                            // shallow copy for thread safety
                            snapshot = JsonUtility.FromJson<SITLCommsJsonOutputPacket>(
                                JsonUtility.ToJson(data)
                            );
                        }

                        string jsonStr = JsonUtility.ToJson(snapshot) + "\n";
                        byte[] bytes = Encoding.UTF8.GetBytes(jsonStr);
                        socketSend.Send(bytes, bytes.Length, remoteEndpoint);
                        CraneRuntimeMetrics.ReportSitlTelemetry(snapshot.timestamp);
                    }
                    catch (Exception ex)
                    {
                        sendError = true;
                        sendErrorMessage = ex.Message;
                        hasRemoteConnection = false;
                    }
                }

                Thread.Sleep((int)intervalMs);
            }
        }
    }
}
