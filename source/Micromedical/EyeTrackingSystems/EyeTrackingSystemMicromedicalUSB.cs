//-----------------------------------------------------------------------
// <copyright file="ImageEyeGrabberMicromedical.cs">
//     Copyright (c) 2014-2023 Jorge Otero-Millan, Johns Hopkins University, University of California, Berkeley. All rights reserved.
// </copyright>
//-----------------------------------------------------------------------
namespace OpenIris
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.ComponentModel.Composition;
    using System.Drawing;
    using OpenIris.ImageGrabbing;
    using OpenIris.HeadTracking;
    using System.IO.Ports;
    using static OpenIris.HeadTracker;

    /// <summary>
    /// Micromedical system.
    /// </summary>
    [Export(typeof(EyeTrackingSystemBase)), PluginDescriptionEyeTrackingSystemAttribute("MicromedicalUSB", typeof(EyeTrackingSystemSettingsMicromedicalUSB))]
    public class EyeTrackingSystemMicromedicalUSB : EyeTrackingSystemMicromedical
    {
        /// <summary>
        /// Gets the cameras. In this case two, left and right eye. 
        /// </summary>
        /// <returns>The list of cameras.</returns>
        protected override EyeCollection<CameraEye> CreateAndStartCameras()
        {
            var settings = Settings as EyeTrackingSystemSettingsMicromedicalUSB;

            // Default region of interest for Micromedical cameras
            var roi = new Rectangle(176, 112, 400, 260);

            var (serialNumberLeft, serialNumberRight) = FindCameras(settings);

            CameraEyeFlyCapture cameraRightEye = null;
            CameraEyeFlyCapture cameraLeftEye = null;

            try
            {
                if (settings.Eye == Eye.Both || settings.Eye == Eye.Right)
                {
                    cameraRightEye = new CameraEyeFlyCapture(Eye.Right, serialNumberRight, settings.FrameRate, roi)
                    {
                        CameraOrientation = CameraOrientation.UprightMirrored,
                        ShouldAdjustFrameRate = false,
                        ShutterDuration = settings.ShutterDuration,
                        Gain = settings.Gain,
                        AutoExposure = settings.AutoExposure,
                        PixelMode = 0,
                        PixelFormat = CameraEyeFlyCapture.CameraPixelFormat.Mono8,
                        BufferSize = 0
                    };
                }

                if (settings.Eye == Eye.Both || settings.Eye == Eye.Left)
                {
                    cameraLeftEye = new CameraEyeFlyCapture(Eye.Left, serialNumberLeft, settings.FrameRate, roi)
                    {
                        CameraOrientation = CameraOrientation.Rotated180Mirrored,
                        ShouldAdjustFrameRate = false,
                        ShutterDuration = settings.ShutterDuration,
                        Gain = settings.Gain,
                        AutoExposure = settings.AutoExposure,
                        PixelMode = 0,
                        PixelFormat = CameraEyeFlyCapture.CameraPixelFormat.Mono8,
                        BufferSize = 0
                    };
                }

                cameraLeftEye?.Start();
                cameraRightEye?.Start();

                // pause to allow camera to start
                System.Threading.Thread.Sleep(500);

                if (settings.UseHeadSensor)
                {
                    System.Diagnostics.Debug.WriteLine("Starting Head Sensor on port " + settings.HeadSensorCOMPort);
                    headSensor = new HeadSensorSeeednRF52840(settings);
                    //headSensor.TriggerHeadSensorSyncPulse();

                    //// pause to allow head sensor to start
                    //System.Threading.Thread.Sleep(5000);
                    double totalDelay = 0.0;
                    for (int i = 0; i < 10; i++)
                    {
                        totalDelay += headSensor.TestHeadSensorDelay();
                    }
                    System.Diagnostics.Debug.WriteLine("Average Head Sensor Delay: " + (totalDelay / 10.0).ToString("F2") + " ms");

                    headSensor.StartHeadSensorAndSyncWithCamera(cameraRightEye, cameraLeftEye);
                }

                // Return the new cameras
                var cameras = new EyeCollection<CameraEye>(cameraLeftEye, cameraRightEye);
                cameraLeftEye = null;
                cameraRightEye = null;
                return cameras;
            }
            catch (Exception ex)
            {
                cameraLeftEye?.Stop();
                cameraRightEye?.Stop();
                headSensor = null;

                throw new InvalidOperationException("Error starting cameras. " + ex.Message, ex);
            }
            finally
            {
                cameraLeftEye?.Dispose();
                cameraRightEye?.Dispose();
            }
        }

        /// <summary>
        /// The head sensor used for tracking head movement and synchronizing with the camera.
        /// </summary>
        protected new HeadSensorSeeednRF52840 headSensor;

        /// <summary>
        /// 
        /// </summary>
        public override IHeadDataSource CreateHeadDataSourceWithCameras() => headSensor;
    }

    /// <summary>
    /// Settings specific to the MicromedicalUSB system.
    /// </summary>
    public class EyeTrackingSystemSettingsMicromedicalUSB : EyeTrackingSystemSettingsMicromedical
    {
        /// <summary>
        /// Initializes a new instance of the EyeTrackingSystemSettingsMicromedicalUSB class.
        /// </summary>
        public EyeTrackingSystemSettingsMicromedicalUSB() { }

        /// <summary>
        /// Gets or sets the COM port ID for the head sensor.
        /// </summary>
        public string HeadSensorCOMPort
        {
            get
            {
                return this.headSensorCOMPort;
            }
            set
            {
                if (value != this.headSensorCOMPort)
                {
                    this.headSensorCOMPort = value;
                    this.OnPropertyChanged(this, "HeadSensorCOMPort");
                }
            }
        }
        private string headSensorCOMPort = "COM5";

        /// <summary>
        /// Whether to enable dynamic calibration of the head sensor.
        /// </summary>
        public bool CalibrateHeadSensor
        {             get
            {
                return this.calibrateHeadSensor;
            }
            set
            {
                if (value != this.calibrateHeadSensor)
                {
                    this.calibrateHeadSensor = value;
                    this.OnPropertyChanged(this, "CalibrateHeadSensor");
                }
            }
        }
        private bool calibrateHeadSensor = false;
    }
}
