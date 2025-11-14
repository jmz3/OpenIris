//-----------------------------------------------------------------------
// <copyright file="HeadSensorSeeednRF52840.cs">
//     Copyright (c) 2014-2025 Jorge Otero-Millan, Johns Hopkins University, University of California, Berkeley. All rights reserved.
//     Copyright (c) 2025 Jeremy Zhang, Johns Hopkins University. All rights reserved.
// </copyright>
//-----------------------------------------------------------------------
namespace OpenIris
{
#nullable enable
#pragma warning disable CS1591 // Missing XML comment for publicly visible type or member

    using OpenIris.ImageGrabbing;
    using System;
    using System.Collections.Concurrent;
    using System.Diagnostics;
    using System.IO.Ports;
    using System.Linq;
    using System.Threading;

    /// <summary>
    /// Class to control a motion sensor used together with the micromedical system. The sensor is connected to PC
    /// through USB COM port with continuous streaming at 300Hz
    /// </summary>
    public sealed class HeadSensorSeeednRF52840 : IHeadDataSource, IHeadSensorCalibrable
    {
        private SerialPort? serialPort;
        private readonly string comPort;

        // Thread-safe queue for incoming IMU data
        private readonly ConcurrentQueue<RawIMUData> dataQueue = new ConcurrentQueue<RawIMUData>();
        private readonly int maxQueueSize = 600; // 2 seconds at 300Hz

        // Background thread for reading serial data
        private Thread? readThread;
        private CancellationTokenSource? cancellationTokenSource;

        // Latest data for GrabHeadData
        private RawIMUData? latestData;
        private readonly object latestDataLock = new object();

        // Calibration offsets
        private double calibrationOffsetX = 0;
        private double calibrationOffsetY = 0;
        private double calibrationOffsetZ = 0;

        private EyeTrackingSystemSettingsMicromedicalUSB settings;

        /// <summary>
        /// Raw IMU data structure
        /// </summary>
        private class RawIMUData
        {
            public ulong SensorFrameCount { get; set; }
            public double Timestamp { get; set; }
            public double AccelX { get; set; }
            public double AccelY { get; set; }
            public double AccelZ { get; set; }
            public double GyroX { get; set; }
            public double GyroY { get; set; }
            public double GyroZ { get; set; }
            public DateTime ReceivedTime { get; set; }
        }

        public HeadSensorSeeednRF52840(EyeTrackingSystemSettingsMicromedicalUSB settings)
        {
            this.settings = settings;
            var comPortProperty = settings.GetType().GetProperty("HeadSensorCOMPort");
            if (comPortProperty != null)
            {
                var portValue = comPortProperty.GetValue(settings) as string;
                comPort = !string.IsNullOrEmpty(portValue) ? portValue! : "COM5";
            }
            else
            {
                comPort = "COM5";
            }

            SerialPortInitialize(comPort, 115200);
        }

        /// <summary>
        /// Calibrates the head sensor using the last 2 seconds of data
        /// Constructs a rotation matrix where:
        /// - The averaged gravity vector becomes the negative x-axis
        /// - The z-axis is constructed using [-1,0,0] as reference
        /// - The y-axis completes the orthonormal basis
        /// </summary>
        public void CalibrateHeadSensor()
        {
            if (!settings.CalibrateHeadSensor)
                return;

            Trace.WriteLine("Starting head sensor calibration.");

            // Wait a bit to accumulate data
            Thread.Sleep(2500); // 2.5 seconds to ensure we have 2 seconds of data

            // Get all data from queue (last 2 seconds)
            var calibrationData = dataQueue.ToArray();

            if (calibrationData.Length < 100)
            {
                throw new InvalidOperationException("Not enough data for calibration.");
            }

            // Calculate averages for accelerometer (this is the gravity vector in sensor frame)
            double[] avg_gravity = new double[3]
            {
                calibrationData.Average(d => d.AccelX),
                calibrationData.Average(d => d.AccelY),
                calibrationData.Average(d => d.AccelZ)
            };

            // Construct rotation matrix from averaged gravity vector
            // Step 1: The averaged gravity vector becomes the new x-axis
            double mag_avg = VectorMagnitude(avg_gravity);

            if (mag_avg < 0.1)
            {
                Trace.WriteLine("ERROR: Averaged gravity vector is too small. Calibration failed.");
                return;
            }

            double[] x_new = VectorNormalize(avg_gravity);

            double[] z_ref = new double[] { -1.0, 0.0, 0.0 };

            double[] y_new = VectorCross(z_ref, x_new);

            double y_mag = VectorMagnitude(y_new);
            if (y_mag < 0.001)
            {
                throw new InvalidOperationException("Calibration failed: reference vector is too close to gravity vector.");
            }

            y_new = VectorNormalize(y_new);

            double[] z_new = VectorCross(x_new, y_new);
            z_new = VectorNormalize(z_new);

            // Step 5: Construct the rotation matrix
            // The rotation matrix transforms vectors from sensor frame to body frame
            settings.HeadSensorRotation = new double[3][];
            settings.HeadSensorRotation[0] = x_new;
            settings.HeadSensorRotation[1] = y_new;
            settings.HeadSensorRotation[2] = z_new;


            // Reset calibration offsets since rotation matrix now handles the coordinate transformation
            calibrationOffsetX = 0;
            calibrationOffsetY = 0;
            calibrationOffsetZ = 0;

            Trace.WriteLine("Rotation matrix calculated and applied:");
            Trace.WriteLine($"  Row 0 (X): [{x_new[0]:F6}, {x_new[1]:F6}, {x_new[2]:F6}]");
            Trace.WriteLine($"  Row 1 (Y): [{y_new[0]:F6}, {y_new[1]:F6}, {y_new[2]:F6}]");
            Trace.WriteLine($"  Row 2 (Z): [{z_new[0]:F6}, {z_new[1]:F6}, {z_new[2]:F6}]");

            Trace.WriteLine("Calibration complete.");
        }

        /// <summary>
        /// Calculate the magnitude of a 3D vector
        /// </summary>
        private double VectorMagnitude(double[] v)
        {
            return Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }

        /// <summary>
        /// Normalize a 3D vector
        /// </summary>
        private double[] VectorNormalize(double[] v)
        {
            double mag = VectorMagnitude(v);
            if (mag < 1e-10)
                return new double[] { 0, 0, 0 };

            return new double[] { v[0] / mag, v[1] / mag, v[2] / mag };
        }

        /// <summary>
        /// Calculate the cross product of two 3D vectors: a × b
        /// </summary>
        private double[] VectorCross(double[] a, double[] b)
        {
            return new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            };
        }


        private void SerialPortInitialize(string portName, int baudRate)
        {
            if (serialPort == null || serialPort.PortName != portName || serialPort.BaudRate != baudRate)
            {
                serialPort?.Dispose();
                serialPort = new SerialPort(portName, baudRate)
                {
                    NewLine = "\n",
                    ReadTimeout = 500,
                    WriteTimeout = 100,
                    DtrEnable = true,
                    RtsEnable = true,
                };
                serialPort.Open();

                // Clear any stale data
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();
            }
        }

        /// <summary>
        /// Start the background thread for continuous reading
        /// </summary>
        private void StartReadingThread()
        {
            cancellationTokenSource = new CancellationTokenSource();
            readThread = new Thread(() => SerialReadLoop(cancellationTokenSource.Token))
            {
                IsBackground = true,
                Name = "IMU Serial Reader"
            };
            readThread.Start();
            Trace.WriteLine("IMU reading thread started.");
        }

        /// <summary>
        /// Background loop for reading serial data continuously
        /// </summary>
        private void SerialReadLoop(CancellationToken cancellationToken)
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                try
                {
                    if (serialPort == null || !serialPort.IsOpen)
                    {
                        Thread.Sleep(100);
                        continue;
                    }

                    // Read a line from serial port
                    string line = serialPort.ReadLine();

                    if (string.IsNullOrWhiteSpace(line) || line.StartsWith("STREAMING") || line.StartsWith("Device"))
                    {
                        continue; // Skip status messages
                    }

                    // Parse the CSV data
                    var data = ParseIMUData(line);
                    if (data != null)
                    {
                        // Store in queue
                        dataQueue.Enqueue(data);

                        // Maintain queue size (keep last 2 seconds)
                        while (dataQueue.Count > maxQueueSize)
                        {
                            dataQueue.TryDequeue(out _);
                        }

                        // Update latest data
                        lock (latestDataLock)
                        {
                            latestData = data;
                        }
                    }
                }
                catch (TimeoutException)
                {
                    // Normal timeout, continue
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"[IMU Reader] Error: {ex.Message}");
                    Thread.Sleep(10); // Brief pause on error
                }
            }

            Trace.WriteLine("IMU reading thread stopped.");
        }

        /// <summary>
        /// Parse CSV line into RawIMUData
        /// Format: frame_count,timestamp,ax,ay,az,gx,gy,gz
        /// </summary>
        private RawIMUData? ParseIMUData(string line)
        {
            try
            {
                var tokens = line.Split(',');
                if (tokens.Length < 8)
                    return null;

                return new RawIMUData
                {
                    SensorFrameCount = ulong.Parse(tokens[0]),
                    Timestamp = double.Parse(tokens[1]),
                    AccelX = double.Parse(tokens[2]),
                    AccelY = double.Parse(tokens[3]),
                    AccelZ = double.Parse(tokens[4]),
                    GyroX = double.Parse(tokens[5]),
                    GyroY = double.Parse(tokens[6]),
                    GyroZ = double.Parse(tokens[7]),
                    ReceivedTime = DateTime.UtcNow
                };
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[IMU Parser] Failed to parse line: {line}. Error: {ex.Message}");
                return null;
            }
        }

        private CameraEyeFlyCapture? cameraPrimary;
        private CameraEyeFlyCapture? cameraSecondary;

        private ulong initialCameraFrameNumber;
        private ulong latestCameraFrameNumber;

        public void StartHeadSensorAndSyncWithCamera(CameraEyeFlyCapture cameraRight, CameraEyeFlyCapture cameraLeft)
        {
            if (cameraRight is null) throw new ArgumentNullException(nameof(cameraRight));

            this.cameraPrimary = cameraRight;
            this.cameraSecondary = cameraLeft;

            SendSerialCommand('s');
            Thread.Sleep(100);

            StartReadingThread();

            Thread.Sleep(500);

            TriggerHeadSensorSyncPulse();

            var syncImage = cameraPrimary.GrabImageEye();
            initialCameraFrameNumber = syncImage.TimeStamp.FrameNumberRaw;
            latestCameraFrameNumber = initialCameraFrameNumber;

            Trace.WriteLine($"Initial Camera Frame: {initialCameraFrameNumber}");
            Trace.WriteLine("Camera and head sensor sync complete.");
        }

        public void TriggerHeadSensorSyncPulse()
        {
            SendSerialCommand('1');
            Debug.WriteLine($"[Serial] Sent sync pulse signal to {comPort}");
        }

        private void SendSerialCommand(char command)
        {
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new InvalidOperationException("Serial port is not initialized or open.");
            }

            serialPort.WriteLine(command.ToString());
        }

        public double TestHeadSensorDelay()
        {
            var stopwatch = new Stopwatch();
            stopwatch.Start();

            RawIMUData? testData;
            lock (latestDataLock)
            {
                testData = latestData;
            }

            stopwatch.Stop();

            if (testData != null)
            {
                Trace.WriteLine($"Latest IMU data retrieved in {stopwatch.ElapsedMilliseconds} ms.");
                Trace.WriteLine($"IMU data: timestamp={testData.Timestamp}, ax = {testData.AccelX:F6}, ay = {testData.AccelY:F6}, az = {testData.AccelZ:F6}");
                return stopwatch.ElapsedMilliseconds;
            }
            else
            {
                Trace.WriteLine("No IMU data available.");
                return 0;
            }
        }

        public HeadData? GrabHeadData()
        {
            ImageEyeTimestamp LST = cameraPrimary!.LastTimeStamp;

            if (LST.FrameNumberRaw > latestCameraFrameNumber)
            {
                RawIMUData? currentData;
                lock (latestDataLock)
                {
                    currentData = latestData;
                }

                if (currentData == null)
                {
                    Debug.WriteLine("[IMU] No valid data available.");
                    return null;
                }

                latestCameraFrameNumber = LST.FrameNumberRaw;

                // Apply calibration offsets
                var accelX = currentData.AccelX - calibrationOffsetX;
                var accelY = currentData.AccelY - calibrationOffsetY;
                var accelZ = currentData.AccelZ - calibrationOffsetZ;

                // Apply rotation if needed
                if (settings.UseHeadSensorRotation)
                {
                    var m = settings.HeadSensorRotation;
                    var accelX_temp = accelX * m[0][0] + accelY * m[0][1] + accelZ * m[0][2];
                    var accelY_temp = accelX * m[1][0] + accelY * m[1][1] + accelZ * m[1][2];
                    var accelZ_temp = accelX * m[2][0] + accelY * m[2][1] + accelZ * m[2][2];
                    accelX = accelX_temp;
                    accelY = accelY_temp;
                    accelZ = accelZ_temp;
                }

                return new HeadData
                {
                    TimeStamp = new ImageEyeTimestamp
                    (
                        seconds: 0.0,
                        frameNumber: latestCameraFrameNumber - initialCameraFrameNumber,
                        frameNumberRaw: latestCameraFrameNumber
                    ),

                    GyroX = currentData.GyroX,
                    GyroY = currentData.GyroY,
                    GyroZ = currentData.GyroZ,

                    AccelerometerX = accelX,
                    AccelerometerY = accelY,
                    AccelerometerZ = accelZ,

                    MagnetometerX = 0.0,
                    MagnetometerY = 0.0,
                    MagnetometerZ = 0.0,
                };
            }
            else
            {
                return null;
            }
        }

        public void Stop()
        {
            // Stop streaming
            try
            {
                SendSerialCommand('x');
            }
            catch { }

            // Stop reading thread
            cancellationTokenSource?.Cancel();

            if (readThread != null && readThread.IsAlive)
            {
                readThread.Join(1000); // Wait up to 1 second
            }

            cancellationTokenSource?.Dispose();

            cameraPrimary = null;
            cameraSecondary = null;
            serialPort?.Close();
            serialPort?.Dispose();

            Trace.WriteLine("Head sensor stopped.");
        }
    }
}