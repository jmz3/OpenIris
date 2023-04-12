﻿using System.Threading.Tasks;
using OpenIris;
using OpenIris.ImageGrabbing;
using System.ComponentModel.Composition;
using SpinnakerNET;
using System.Windows.Forms;
using System.Drawing;
using System;
using System.Collections.Generic;
using Spinnaker;
using System.Diagnostics;
using static System.Net.Mime.MediaTypeNames;
using System.Threading;
using System.ComponentModel;

namespace SpinnakerInterface
{
#nullable enable

    [Export(typeof(EyeTrackingSystemBase)), PluginDescriptionEyeTrackingSystem("Spinnaker Dual Camera", typeof(EyeTrackingSystemSettingsSpinnaker_DualCam))]

    class EyeTrackingSystemSpinnaker_DualCam : EyeTrackingSystemBase
    {
        protected CameraEyeSpinnaker? leftEyeCamera = null;
        protected CameraEyeSpinnaker? rightEyeCamera = null;

        public override EyeCollection<CameraEye?>? CreateAndStartCameras()
        {
            var settings = Settings as EyeTrackingSystemSettingsSpinnaker_DualCam;
            try
            {
                var cameraList = CameraEyeSpinnaker.FindCameras(2,settings.Eye,settings.LeftEyeCameraSerialNumber,settings.RightEyeCameraSerialNumber);

                leftEyeCamera = new CameraEyeSpinnaker(
                whichEye: Eye.Left,
                camera: cameraList[0],
                frameRate: (double)settings.FrameRate,
                gain: (int)settings.Gain,
                roi: new Rectangle { Width = 720, Height = 450 });

                rightEyeCamera = new CameraEyeSpinnaker(
                whichEye: Eye.Right,
                camera: cameraList[1],
                frameRate: (double)settings.FrameRate,
                gain: (int)settings.Gain,
                roi: new Rectangle { Width = 720, Height = 450 });

                settings.LeftEyeCameraSerialNumber = cameraList[0].DeviceSerialNumber.ToString();
                settings.RightEyeCameraSerialNumber = cameraList[1].DeviceSerialNumber.ToString();

                //start cameras
                CameraEyeSpinnaker.BeginSynchronizedAcquisition(leftEyeCamera, rightEyeCamera);

                return new EyeCollection<CameraEye?>(leftEyeCamera, rightEyeCamera);
            }
            catch (Exception ex)
            {
                leftEyeCamera?.Stop();
                rightEyeCamera?.Stop();

                throw new InvalidOperationException("Error starting cameras captures or setting GPIOs. " + ex.Message, ex);
            }  
        }
       
    }
    public class EyeTrackingSystemSettingsSpinnaker_DualCam : EyeTrackingSystemSettings
    {
        public EyeTrackingSystemSettingsSpinnaker_DualCam()
        {
            PixPerMm = 7;
            DistanceCameraToEyeMm = 70;
            Eye = Eye.Both;
        }

        [Category("Camera properties"), Description("LeftEye - CameraSerialNumber")]
        public string LeftEyeCameraSerialNumber { get => this.leftEyeCameraSerialNumber; set => SetProperty(ref leftEyeCameraSerialNumber, value, nameof(LeftEyeCameraSerialNumber)); }
        private string leftEyeCameraSerialNumber = null;

        [Category("Camera properties"), Description("LeftEye - CameraSerialNumber")]
        public string RightEyeCameraSerialNumber { get => this.rightEyeCameraSerialNumber; set => SetProperty(ref rightEyeCameraSerialNumber, value, nameof(RightEyeCameraSerialNumber)); }
        private string rightEyeCameraSerialNumber = null;
        [NeedsRestarting]
        [Category("Camera properties"), Description("Gain")]
        public float Gain { get => this.gain; set => SetProperty(ref gain, value, nameof(Gain)); }
        private float gain = 9;
    }
}
