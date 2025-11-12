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
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.IO.Ports;
    using System.Linq;
    using System.Net.Sockets;
    using System.Threading;
    using System.Threading.Tasks;
    /// <summary>
    /// Class to control a motion sensor used together with the micromedical system. The sensor is connected to PC
    /// through USB COM port
    /// </summary>
    public sealed class HeadSensorSeeednRF52840 : IHeadDataSource, IHeadSensorCalibrable
    {
        private SerialPort? serialPort; // Private member for the serial port
        private readonly string comPort; // Private member for the COM port
        public string RawDataString = "";

        /// <summary>
        /// Eye tracking system settings.
        /// </summary>
        private EyeTrackingSystemSettingsMicromedicalUSB settings;

        /// <summary>
        /// Constructor for Head sensors. 
        /// </summary>
        /// <param name="settings"></param>
        public HeadSensorSeeednRF52840(EyeTrackingSystemSettingsMicromedicalUSB settings)
        {
            this.settings = settings;
            var comPortProperty = settings.GetType().GetProperty("HeadSensorCOMPort");
            if (comPortProperty != null)
            {
                var portValue = comPortProperty.GetValue(settings) as string;
                comPort = !string.IsNullOrEmpty(portValue) ? portValue! : "COM5"; // Default to "COM5" if not set
            }
            else
            {
                comPort = "COM5";
            }
            SerialPortInitialize(comPort, 115200);
        }

        /// <summary>
        /// Calibrates the head sensor if the CalibrateHeadSensor setting is enabled.
        /// </summary>
        public void CalibrateHeadSensor()
        {
            if (settings.CalibrateHeadSensor)
            {
                Trace.WriteLine("Starting head sensor calibration.");
                // TODO: Implement calibration procedure here
                // It should take several seconds of data and then take the average
                // Use the 
            }
        }

        /// <summary>
        /// This is the data sent by the TEENSY to the camera, minus the 4-byte header "flag" value of 0xffffffff.
        /// Sensors are: Accel X/Y/Z, +/-32768 == +/-2G
        ///              Temperature
        ///              Gyro X/Y/Z, +/-32768 == +/- 500dps
        ///              Magnetometer
        /// </summary>
        class HeadDataPacket
        {
            EyeTrackingSystemSettingsMicromedicalUSB settings;
            string IMUReadings;

            /// <summary>
            /// TRUE when sync pulse present, every 16 frames.
            /// </summary>
            public bool SYNC = false;

            /// <summary>
            /// The number of frames has passed when the packet is received
            /// </summary>
            public UInt32 FrameCount;

            public HeadDataPacket(string IMUReadings, EyeTrackingSystemSettingsMicromedicalUSB settings)
            {
                this.settings = settings;
                this.IMUReadings = IMUReadings;

                if (IMUReadings!="")
                {
                    SYNC = true;
                    FrameCount = UInt32.Parse(IMUReadings.Split(',')[0]);
                }

            }

            /// <summary>
            /// Converts the packet to HeadData
            /// </summary>
            /// <param name="initialSensorFrameNumber"></param>
            /// <param name="initialCameraFrameNumber"></param>
            /// <returns></returns>
            public HeadData ConvertPacketToHeadData(ulong initialSensorFrameNumber, ulong initialCameraFrameNumber)
            {
                // Example of IMUReadings:
                // fc, t0, t1, ax, ay, az, gx, gy, gz

               string[] tokens = IMUReadings.Split(',');

                var currentSensorFrameNumber = ulong.Parse(tokens[0]);

                var AccelerometerX = double.Parse(tokens[3]);
                var AccelerometerY = double.Parse(tokens[4]);
                var AccelerometerZ = double.Parse(tokens[5]);

                if (settings.UseHeadSensorRotation) 
                {
                    var m = settings.HeadSensorRotation;
                    var AccelerometerX_temp = AccelerometerX * m[0][0] + AccelerometerY * m[0][1] + AccelerometerZ * m[0][2];
                    var AccelerometerY_temp = AccelerometerX * m[1][0] + AccelerometerY * m[1][1] + AccelerometerZ * m[1][2];
                    var AccelerometerZ_temp = AccelerometerX * m[2][0] + AccelerometerY * m[2][1] + AccelerometerZ * m[2][2];
                    AccelerometerX = AccelerometerX_temp;
                    AccelerometerY = AccelerometerY_temp;
                    AccelerometerZ = AccelerometerZ_temp;
                }

                return new HeadData
                    {
                        TimeStamp = new ImageEyeTimestamp
                        (
                            seconds: 0.0,
                            frameNumber: currentSensorFrameNumber - initialSensorFrameNumber + initialCameraFrameNumber,
                            frameNumberRaw: currentSensorFrameNumber
                        //frameNumberRaw: initialCameraFrameNumber
                        ),

                        GyroX = double.Parse(tokens[6]),
                        GyroY = double.Parse(tokens[7]),
                        GyroZ = double.Parse(tokens[8]),

                        // The way the sensor is currently placed in the goggles.
                        // The x axes points down
                        // The y axes points forward
                        // The z axes points out to the right
                        // All from the point of view of the subject wearing the goggles
                        // The values correspond with the fraction of 1G that is projected along each axis
                        // When the subject is upright X=-1, y=0, Z=0

                        AccelerometerX = AccelerometerX,
                        AccelerometerY = AccelerometerY,
                        AccelerometerZ = AccelerometerZ,

                        MagnetometerX = 0.0,
                        MagnetometerY = 0.0,
                        MagnetometerZ = 0.0,
                    };
            }
        }


        private CameraEyeFlyCapture? cameraPrimary;
        private CameraEyeFlyCapture? cameraSecondary;

        private ulong initialCameraFrameNumber;
        private ulong initialSensorFrameNumber;

        private ulong latestCameraFrameNumber;

        public void StartHeadSensorAndSyncWithCamera(CameraEyeFlyCapture cameraRight, CameraEyeFlyCapture cameraLeft)
        {
            if (cameraRight is null) throw new ArgumentNullException(nameof(cameraRight));

            this.cameraPrimary = cameraRight;
            this.cameraSecondary = cameraLeft;

            HeadDataPacket? syncPacket = null;
            ImageEye? syncImage = null;
            bool cancel = false;
            double timePacket = double.MaxValue;
            double timeImage = 0;

            Trace.WriteLine("Syncing camera and head sensor.");
            var task = Task.Run(() =>
            {
                while (!cancel && Math.Abs(timeImage - timePacket) > 2 / cameraRight.FrameRate)
                {
                    var image = cameraPrimary.GrabImageEye();
                    PokeAndListenHeadSensorOnce();

                    // If there was no packet sleep a bit and come back later because
                    // the call to GetPacket is not blocking
                    if (serialPort == null | RawDataString == "")
                    {
                        Thread.Sleep(2);
                        continue;
                    }

                    var packet = new HeadDataPacket(RawDataString, settings);

                    if (packet.SYNC)
                    {
                        syncPacket = packet;
                        syncImage = image;
                        timePacket = EyeTrackerDebug.TimeElapsed.TotalSeconds;
                        timeImage = EyeTrackerDebug.TimeElapsed.TotalSeconds;
                    }
                }
            });

            //var task2 = Task.Run(() =>
            //{
                // could potentially add this part to sync two cameras using led
            //});

            var finishedBeforeTimeout = Task.WaitAll(new Task[] { task }, 2000); // 5s timeout

            Trace.WriteLine($"Finished syncing camera and head sensor. Diff time= {Math.Abs(timeImage - timePacket)}");

            if (finishedBeforeTimeout && !task.IsFaulted )
            {
                initialCameraFrameNumber = syncImage.TimeStamp.FrameNumberRaw;
                initialSensorFrameNumber = syncPacket.FrameCount;

                Trace.WriteLine($"Initial Camera Frame{initialCameraFrameNumber}");
                Trace.WriteLine($"Initial Sensor Frame{initialSensorFrameNumber}");
            }
            else
            {
                cancel = true;
                if (task.IsFaulted) throw task.Exception;

                throw new OpenIrisException("Head sensor does not seem to be present or sync pattern not detected. Change the settings.");
            }

            latestCameraFrameNumber = initialCameraFrameNumber;
        }

        public void TriggerHeadSensorSyncPulse()
        {
            SendSerialData("1", appendNewLine: true);
            System.Diagnostics.Debug.WriteLine($"[Serial] Sent start/stop recording signal to {comPort}");
        }

        private void SerialPortInitialize(string portName, int baudRate)
        {
            if (serialPort == null || serialPort.PortName != portName || serialPort.BaudRate != baudRate)
            {
                serialPort?.Dispose(); // Dispose the existing port if it doesn't match
                serialPort = new SerialPort(portName, baudRate)
                {
                    NewLine = "\n",
                    ReadTimeout = 100,
                    WriteTimeout = 100,
                    DtrEnable = true,
                    RtsEnable = true,
                };
                serialPort.Open();
            }
        }

        private void SendSerialData(string payload, bool appendNewLine = true)
        {
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new InvalidOperationException("Serial port is not initialized or open.");
            }

            if (appendNewLine)
                serialPort.WriteLine(payload);
            else
                serialPort.Write(payload);
        }

        public double TestHeadSensorDelay()
        {
            var stopwatch = new Stopwatch();
            stopwatch.Start();
            PokeAndListenHeadSensorOnce();
            stopwatch.Stop();
            if (RawDataString != null)
            {
                //string[] tokens = RawDataString.Split(',');
                //Trace.WriteLine($"Head sensor packet received in {stopwatch.ElapsedMilliseconds} ms.");
                //Trace.WriteLine($"Head sensor data: ax = {tokens[2]}, ay = {tokens[3]}, az = {tokens[4]}, gx = {tokens[5]}, gy = {tokens[6]}, gz = {tokens[7]}");
                return stopwatch.ElapsedMilliseconds;
            }
            else
            {
                Trace.WriteLine("Failed to receive head sensor packet.");
                return 0;
            }
        }

        private string? lastValidRawDataString = null;

        private void PokeAndListenHeadSensorOnce()
        {
            const string payload = "2\n"; // newline so ReadLine() terminates

            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new InvalidOperationException("Serial port is not initialized or open.");
            }

            try
            {
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();

                serialPort.Write(payload);

                // Empty the raw data string before reading new data
                RawDataString = "";
                RawDataString = serialPort.ReadLine();

                // Update the last valid RawDataString
                if (!string.IsNullOrEmpty(RawDataString))
                {
                    lastValidRawDataString = RawDataString;
                }
            }
            catch (TimeoutException ex)
            {
                System.Diagnostics.Debug.WriteLine($"[Serial] Timeout: {ex.Message}");

                // Use the last valid RawDataString in case of a timeout
                if (lastValidRawDataString != null)
                {
                    RawDataString = lastValidRawDataString;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[Serial] Error: {ex.Message}");
                throw;
            }
        }

        public HeadData? GrabHeadData()
        {
            ImageEyeTimestamp LST = cameraPrimary!.LastTimeStamp;

            if (LST.FrameNumberRaw > latestCameraFrameNumber)
            {
                PokeAndListenHeadSensorOnce();
                
                if (string.IsNullOrEmpty(RawDataString))
                {
                    System.Diagnostics.Debug.WriteLine("[Serial] No valid data available to create a packet.");
                    return null;
                }
                
                var packet = new HeadDataPacket(RawDataString, settings);
                latestCameraFrameNumber++;
                return packet.ConvertPacketToHeadData(initialSensorFrameNumber, initialCameraFrameNumber);
            }
            else return null;

        }

        public void Stop()
        {
            cameraPrimary = null;
            cameraSecondary = null;
            serialPort?.Close();
        }
    }

}
