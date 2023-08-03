using System;
using System.Runtime.InteropServices;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.Assertions;

public class ImagePublisher : MonoBehaviour
{
    public string topicName = "unity_camera/color/image_raw";
    public string infoTopicName = "unity_camera/rgb/camera_info";

    // The game object
    public Camera ImageCamera;
    public Transform opticalLink;
    public int width = 640;
    public int height = 480;
    public float publishMessageFrequency = 1.0f / 20.0f;
    private sensor_msgs_Image msg;
    private sensor_msgs_CameraInfo info_msg;

    public byte[] data;
    public int step;
    private float timeElapsed;
    private Texture2D texture2D;
    private Rect rect;
    private RenderTexture finalRT;

    private void GetCameraInfo()
    {
        info_msg = new sensor_msgs_CameraInfo();
        info_msg.width = (uint)width;
        info_msg.height = (uint)height;

        //Focal center currently assumes zero lens shift.
        //Focal center x.
        double cX = width / 2.0;
        //Focal center y.
        double cY = height / 2.0;

        //Get the vertical field of view of the camera taking into account any physical camera settings.
        Assert.IsFalse(ImageCamera.usePhysicalProperties);
        float verticalFieldOfView = ImageCamera.fieldOfView;
        
        double focalLengthInPixels =
            (height / 2.0) / Math.Tan((Mathf.Deg2Rad * verticalFieldOfView) / 2.0);
        //Focal Length (x)
        double fX = focalLengthInPixels;
        //Focal Length (y)
        double fY = focalLengthInPixels;

        //Axis Skew, Assuming none.
        double s = 0.0;
        //http://ksimek.github.io/2013/08/13/intrinsic/
        info_msg.k = new []
        {
            fX, s, cX,
            0, fY, cY,
            0, 0, 1
        };

        info_msg.distortion_model = ROSInterface.AllocateString("plumb_bob");
        info_msg.d = ROSInterface.AllocateDoubleArray(new[]
            {
                0.0, //k1
                0.0, //k2
                0.0, //t1
                0.0, //t2
                0.0 //k3
            }
        );

        info_msg.r = new double[]
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

        info_msg.p = new double[]
        {
            fX, 0, cX, 0,
            0, fY, cY, 0,
            0, 0, 1, 0
        };

        info_msg.binning_x = 0;
        info_msg.binning_y = 0;
        
    }

    void Start()
    {
        texture2D = new Texture2D(width, height, TextureFormat.RGBA32, false);
        data = new byte[width * height * 4];
        rect = new Rect(0, 0, width, height);

        msg.header.frame_id = ROSInterface.AllocateString(opticalLink.name);
        msg.data = ROSInterface.AllocateByteArray(data);
        msg.height = (uint)height;
        msg.width = (uint)width;
        msg.encoding = ROSInterface.AllocateString("rgba8");
        msg.step = (uint)(4 * width);
        msg.is_bigendian = 0;

        GetCameraInfo();

        bool supportsAntialiasing = true;
        bool needsRescale = false;
        var depth = 32;
        var format = RenderTextureFormat.Default;
        var readWrite = RenderTextureReadWrite.Default;
        var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        finalRT = RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
    }


    void Update()
    {
        if (texture2D != null)
            UpdateMessage();
    }

    private void UpdateMessage()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            RenderTexture.active = finalRT;
            ImageCamera.targetTexture = finalRT;
            ImageCamera.Render();
            texture2D.ReadPixels(rect, 0, 0);
            height = texture2D.height;
            width = texture2D.width;
            if (data.Length != height * width * 4)
            {
                data = new byte[height * width * 4];
            }

            string encoding = "rgba8";
            byte is_bigendian = 0;
            step = 4 * width;
            Color32[] colors = texture2D.GetPixels32();
            uint ind = 0;
            for (uint y = 0; y < height; y++)
            {
                for (uint x = 0; x < width; x++)
                {
                    long ind2 = (height - y - 1) * width + x;
                    data[ind] = colors[ind2].r;
                    data[ind + 1] = colors[ind2].g;
                    data[ind + 2] = colors[ind2].b;
                    data[ind + 3] = colors[ind2].a;
                    ind += 4;
                }
            }

            Marshal.Copy(data, 0, msg.data.ptr, (int)msg.data.length);
            ROSInterface.SetROSTime(ref msg.header.stamp);
            ROSInterface.PublishROS(ref msg, topicName);
            ROSInterface.SendTransform(opticalLink);

            ROSInterface.PublishROS(ref info_msg, infoTopicName);

            timeElapsed = 0;
        }
    }
}