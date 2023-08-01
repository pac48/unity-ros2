using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System.Text;
using Random = UnityEngine.Random;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Collections.LowLevel.Unsafe;

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeQuaternion
{
    public double x;
    public double y;
    public double z;
    public double w;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeVector3
{
    public double x;
    public double y;
    public double z;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeTwist
{
    public NativeVector3 linear;
    public NativeVector3 angular;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeTransform
{
    public IntPtr frame_id;
    public IntPtr child_frame_id;
    public NativeVector3 translation;
    public NativeQuaternion rotation;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeOdom
{
    public NativeTransform pose;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] pose_covariance;

    public NativeVector3 linear_velocity;
    public NativeVector3 angular_velocity;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] twist_covariance;
}


[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeInt32
{
    public IntPtr topic;
    public int data;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeImage
{
    public IntPtr topic;
    public int height;
    public int width;
    public int step;
    public IntPtr frame_id;
    public IntPtr encoding;
    public IntPtr data;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeScan
{
    public IntPtr topic;
    public IntPtr frame_id;
    public float angle_min;
    public float angle_max;
    public float angle_increment;
    public float time_increment;
    public float scan_time;
    public float range_min;
    public float range_max;
    public int count;
    public IntPtr ranges;
    public IntPtr intensities;
}

public class ROSPublisher : MonoBehaviour
{
    public bool should_publish;

    public void Publish()
    {
        should_publish = true;
    }
}

public class ROSInterface : MonoBehaviour
{
    // interface
    public List<Transform> transforms;
    public ImagePublisher[] color_cameras;
    public DepthImagePublisher[] depth_cameras;
    public LaserScanPublisher[] laser_scanners;
    public Animator animator;
    public Transform odom_transform;

    // native
    private IntPtr handle;
    // private IntPtr ptr; // pointer to struct 
    private int ptr_size = 0; // pointer to struct 
    private NativeTransform native_transform;
    private NativeInt32 native_eating;
    private NativeInt32 native_taking_medicine;
    private NativeOdom native_odom;
    public NativeTwist native_twist;
    public NativeImage native_image;
    public NativeScan native_scan;

    // helper
    private Vector3 prev_forward;
    private Vector3 prev_position;

    ~ROSInterface()
    {
        if (handle != null)
        {
            Destroy(handle);
        }
    }

    void Start()
    {
        handle = Init();
    }
    // color_cameras = FindObjectsOfType<ImagePublisher>();
    // depth_cameras = FindObjectsOfType<DepthImagePublisher>();
    // laser_scanners = FindObjectsOfType<LaserScanPublisher>();
    //
    // prev_forward = new Vector3();
    // prev_position = new Vector3();
    // // prev_position = odom_transform.position;
    // // prev_forward = odom_transform.forward;
    //
    // handle = Init();
    // native_transform = new NativeTransform();
    //
    // native_eating.topic = Marshal.StringToHGlobalAnsi("person_eating");
    // native_taking_medicine.topic = Marshal.StringToHGlobalAnsi("person_taking_medicine");
    //
    //
    // native_odom = new NativeOdom();
    // native_odom.pose.frame_id = Marshal.StringToHGlobalAnsi("odom");
    // native_odom.pose.child_frame_id = Marshal.StringToHGlobalAnsi(odom_transform.name);
    // native_odom.pose_covariance = new double[36];
    // native_odom.pose_covariance[0] = 0.1;
    // native_odom.pose_covariance[7] = 0.1;
    // native_odom.pose_covariance[14] = 1000000000000.0;
    // native_odom.pose_covariance[21] = 1000000000000.0;
    // native_odom.pose_covariance[28] = 1000000000000.0;
    // native_odom.pose_covariance[35] = 1000000000000.0;
    // native_odom.twist_covariance = new double[36];
    // native_odom.twist_covariance[0] = 1000000000000.0;
    // native_odom.twist_covariance[7] = 1000000000000.0;
    // native_odom.twist_covariance[14] = 1000000000000.0;
    // native_odom.twist_covariance[21] = 1000000000000.0;
    // native_odom.twist_covariance[28] = 1000000000000.0;
    // native_odom.twist_covariance[35] = 1000000000000.0;
    // }

    // void FixedUpdate()
    // {
    //     var velocity = (odom_transform.position - prev_position) / Time.deltaTime;
    //     var tmp = Vector3.Dot(odom_transform.forward, prev_forward);
    //     tmp = Math.Max(tmp, 0.0f);
    //     tmp = Math.Min(tmp, 1.0f);
    //     var omega = Math.Acos(tmp) / Time.deltaTime;
    //     var angular_velocity = Vector3.Cross(odom_transform.forward, prev_forward);
    //     angular_velocity.Normalize();
    //
    //     Vector3 tmp2 = new Vector3();
    //     tmp2.x = Vector3.Dot(odom_transform.forward, velocity);
    //     tmp2.y = Vector3.Dot(odom_transform.up, velocity);
    //     tmp2.z = Vector3.Dot(odom_transform.right, velocity);
    //     var flu = tmp2; //.To<FLU>();
    //     native_odom.linear_velocity.x = flu.x;
    //     native_odom.linear_velocity.y = flu.y;
    //     native_odom.linear_velocity.z = flu.z;
    //
    //     Vector3 tmp3 = new Vector3();
    //     tmp3.x = (float)omega * angular_velocity.x;
    //     tmp3.y = (float)omega * angular_velocity.y;
    //     tmp3.z = (float)omega * angular_velocity.z;
    //     flu = tmp3; //.To<FLU>();
    //     native_odom.angular_velocity.x = flu.x;
    //     native_odom.angular_velocity.y = flu.y;
    //     native_odom.angular_velocity.z = flu.z;
    //
    //     prev_position = odom_transform.position;
    //     prev_forward = odom_transform.forward;
    // }


    void SetTransform(Transform trans, ref NativeTransform native)
    {
        var translation = new NativeVector3();
        var position_FLU = trans.position.To<FLU>();
        translation.x = position_FLU.x;
        translation.y = position_FLU.y;
        translation.z = position_FLU.z;
        native.translation = translation;

        var rotation = new NativeQuaternion();
        var rotation_FLU = trans.rotation.To<FLU>();
        rotation.w = rotation_FLU.w;
        rotation.x = rotation_FLU.x;
        rotation.y = rotation_FLU.y;
        rotation.z = rotation_FLU.z;
        native.rotation = rotation;
    }

    public IntPtr[] getArrayPtr<T>(ref CArray array, int len)
    {
        int elementSize = Marshal.SizeOf(typeof(T));
        if (array.ptr != IntPtr.Zero)
        {
            for (int i = 0; i < array.length; i++)
            {
                if (typeof(T) == typeof(IROSMsg))
                {
                    var msg = (IROSMsg)Marshal.PtrToStructure(array.ptr + i * elementSize, typeof(T));
                    msg.Delete();
                }
                else if (typeof(T) == typeof(IntPtr))
                {
                    Marshal.FreeHGlobal(Marshal.ReadIntPtr(array.ptr, i * elementSize));
                }
            }

            Marshal.FreeHGlobal(array.ptr);
        }

        array.length = len;
        array.ptr = Marshal.AllocHGlobal(elementSize * len);
        IntPtr[] ret = new IntPtr[len];
        for (int i = 0; i < len; i++)
        {
            Marshal.WriteIntPtr(array.ptr, i * elementSize, array.ptr + i * elementSize);
            ret[i] = Marshal.ReadIntPtr(array.ptr, i * elementSize);
        }

        return ret;
    }

    void setStructArray<T>(ref CArray array, T[] msgs) where T : IROSMsg
    {
        IntPtr[] arr = getArrayPtr<T>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.StructureToPtr(msgs[i], arr[i], false);
        }
    }

    void setStringArray(ref CArray array, string[] msgs)
    {
        IntPtr[] arr = getArrayPtr<IntPtr>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.WriteIntPtr(arr[i], 0, Marshal.StringToHGlobalAnsi(msgs[i] + "\0"));
        }
    }

    void setDoubleArray(ref CArray array, double[] msgs)
    {
        IntPtr[] arr = getArrayPtr<double>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.WriteInt64(arr[i], 0, BitConverter.DoubleToInt64Bits(msgs[i]));
        }
    }

    void setString(ref IntPtr ptr, string str)
    {
        if (ptr != IntPtr.Zero)
        {
            Marshal.FreeHGlobal(ptr);
        }

        ptr = Marshal.StringToHGlobalAnsi(str + "\0");
    }

    void Update()
    {
        geometry_msgs_PoseArray val4 = new geometry_msgs_PoseArray();
        geometry_msgs_Pose[] poses = new geometry_msgs_Pose[2];
        poses[0].position.x = 3.1;
        poses[0].position.y = 2.2;
        poses[0].position.z = 1.3;
        poses[1].position.x = 1.1;
        poses[1].position.y = 2.2;
        poses[1].position.z = 3.3;
        setStructArray(ref val4.poses, poses);
        PublishROS(ref val4, "pose_aray");
        setStructArray(ref val4.poses, poses);

        std_msgs_Float32MultiArray val2 = new std_msgs_Float32MultiArray();
        double[] data_vals = { 1, 2, 3 };
        setDoubleArray(ref val2.data, data_vals);
        std_msgs_MultiArrayDimension[] mulit = new std_msgs_MultiArrayDimension[2];
        setString(ref mulit[0].label, "sad");
        mulit[0].size = 3;
        setString(ref mulit[1].label, "no");
        setStructArray(ref val2.layout.dim, mulit);
        PublishROS(ref val2, "array");


        sensor_msgs_JointState val3 = new sensor_msgs_JointState();
        string[] name_arr = { "joint_1", "joint_2", "joint_3" };
        setStringArray(ref val3.name, name_arr);
        double[] position_arr = {1.1,2.2,3.3};
        setDoubleArray(ref val3.position, position_arr);
        PublishROS(ref val3, "joint_state");
        
        sensor_msgs_JointState val33 = new sensor_msgs_JointState();
        ReceiveROS(ref val33, "joint_state");

        std_msgs_String val5 = new std_msgs_String();
        ReceiveROS(ref val5, "chatter");
        string str = Marshal.PtrToStringAnsi(val5.data);
        Debug.Log(str);
    }

    public void PublishROS<T>(ref T msg, string topic_str) where T : unmanaged, IROSMsg
    {
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(msg.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");
        
        IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(msg));
        Marshal.StructureToPtr(msg, ptr, false);
        PublishROSInternal(handle, type, topic, ptr);
        Marshal.FreeHGlobal(ptr);
    }

    public void ReceiveROS<T>(ref T msg, string topic_str) where T : unmanaged, IROSMsg
    {
        // Try to bypass marshalling
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(msg.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");
        IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(msg));
        var tmp = new T();
        Marshal.StructureToPtr(tmp, ptr, false);
        ReceiveROSInternal(handle, type, topic, ptr);
        msg = (T)Marshal.PtrToStructure(ptr, typeof(T));
        Marshal.FreeHGlobal(ptr);
    }

    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr Init();

    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Destroy(IntPtr handle);

    [DllImport("libROSInterface.so", EntryPoint = "Publish", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr input);

    [DllImport("libROSInterface.so", EntryPoint = "Receive", CallingConvention = CallingConvention.Cdecl)]
    private static extern void ReceiveROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr output);
}