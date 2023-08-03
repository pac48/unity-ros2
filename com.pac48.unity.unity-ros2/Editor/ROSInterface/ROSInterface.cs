using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ROSInterface
{
    // native
    private static IntPtr handle;
    private static HashSet<IntPtr> memory;

    // void OnDestroy()
    // {
    //     if (handle != null)
    //     {
    //         DestroyInternal(handle);
    //     }
    // }

    [RuntimeInitializeOnLoadMethod]
    static void Init()
    {
        if (handle == IntPtr.Zero)
        {
            handle = InitInternal(Allocate);
            memory = new HashSet<IntPtr>();
        }
    }

    private static IntPtr Allocate(int numBytes)
    {
        if (handle == IntPtr.Zero) Init();
        IntPtr ptr = Marshal.AllocHGlobal(numBytes);
        memory.Add(ptr);

        return ptr;
    }

    public static void Free(IntPtr ptr)
    {
        if (handle == IntPtr.Zero) Init();
        if (ptr == IntPtr.Zero) return;
        if (memory.Contains(ptr))
        {
            Marshal.FreeHGlobal(ptr);
            memory.Remove(ptr);
        }
    }

    private static void FreeAll()
    {
        foreach (var ptr in memory)
        {
            Marshal.FreeHGlobal(ptr);
        }

        memory = new HashSet<IntPtr>();
    }

    public delegate IntPtr AllocateDelegate(int numBytes);

    private static IntPtr[] AllocateArrayPtr<T>(ref CArray array, int len)
    {
        int elementSize = Marshal.SizeOf(typeof(T));
        array.length = len;
        array.ptr = Allocate(elementSize * len);
        IntPtr[] ret = new IntPtr[len];
        for (int i = 0; i < len; i++)
        {
            Marshal.WriteIntPtr(array.ptr, i * elementSize, array.ptr + i * elementSize);
            ret[i] = Marshal.ReadIntPtr(array.ptr, i * elementSize);
        }

        return ret;
    }

    public static CArray AllocateStructArray<T>(T[] msgs) where T : IROSMsg
    {
        CArray array = new CArray();
        IntPtr[] arr = AllocateArrayPtr<T>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.StructureToPtr(msgs[i], arr[i], false);
        }

        return array;
    }

    public static CArray AllocateStringArray(string[] msgs)
    {
        CArray array = new CArray();
        IntPtr[] arr = AllocateArrayPtr<IntPtr>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.WriteIntPtr(arr[i], 0, AllocateString(msgs[i] + "\0"));
        }

        return array;
    }

    public static CArray AllocateDoubleArray(double[] msgs)
    {
        CArray array = new CArray();
        IntPtr[] arr = AllocateArrayPtr<double>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
        return array;
    }

    public static CArray AllocateFloatArray(float[] msgs)
    {
        CArray array = new CArray();
        IntPtr[] arr = AllocateArrayPtr<float>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
        return array;
    }

    public static CArray AllocateByteArray(byte[] msgs)
    {
        CArray array = new CArray();
        IntPtr[] arr = AllocateArrayPtr<float>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
        return array;
    }

    public static string GetString(IntPtr ptr)
    {
        if (ptr == IntPtr.Zero) return "";
        return Marshal.PtrToStringAnsi(ptr);
    }

    public static IntPtr AllocateString(string str)
    {
        if (handle == IntPtr.Zero) Init();
        IntPtr ptr = Marshal.StringToHGlobalAnsi(str + "\0");
        memory.Add(ptr);
        return ptr;
    }

    public static string[] GetStringArray(CArray array)
    {
        if (array.ptr == IntPtr.Zero) return new string[0];
        string[] ret = new string[array.length];
        int elementSize = Marshal.SizeOf(typeof(IntPtr));
        for (int i = 0; i < array.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(array.ptr, elementSize * i);
            ret[i] = GetString(ptr);
        }

        return ret;
    }

    public static double[] GetDoubleArray(CArray array)
    {
        if (array.ptr == IntPtr.Zero) return new double[0];
        double[] ret = new double[array.length];
        Marshal.Copy(array.ptr, ret, 0, (int)array.length);
        return ret;
    }

    public static T GetStruct<T>(IntPtr ptr) where T : IROSMsg, new()
    {
        if (ptr == IntPtr.Zero) return new T();
        return (T)Marshal.PtrToStructure(ptr, typeof(T));
    }

    public static T[] GetStructArray<T>(CArray array) where T : IROSMsg, new()
    {
        if (array.ptr == IntPtr.Zero) return new T[0];
        T[] ret = new T[array.length];
        int elementSize = Marshal.SizeOf(typeof(T));
        for (int i = 0; i < array.length; i++)
        {
            ret[i] = GetStruct<T>(array.ptr + elementSize * i);
        }

        return ret;
    }

    public static void PublishROS<T>(ref T msg, string topic_str) where T : IROSMsg
    {
        if (handle == IntPtr.Zero) Init();
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(msg.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");

        IntPtr ptr = Allocate(Marshal.SizeOf(msg));
        Marshal.StructureToPtr(msg, ptr, false);
        PublishROSInternal(handle, type, topic, ptr);
    }

    public static T ReceiveROS<T>(string topic_str) where T : IROSMsg, new()
    {
        var tmp = new T();
        if (handle == IntPtr.Zero) Init();
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(tmp.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");
        IntPtr ptr = new IntPtr();
        ReceiveROSInternal(handle, type, topic, ref ptr);
        var msg = GetStruct<T>(ptr);
        return msg;
    }

    public static void SendTransform(Transform obj)
    {
        geometry_msgs_TransformStamped tf = new geometry_msgs_TransformStamped();
        tf.child_frame_id = AllocateString(obj.transform.name);
        tf.header.frame_id = AllocateString("unity"); //TODO this should be parameter

        var position_FLU = obj.position.To<FLU>();
        tf.transform.translation.x = position_FLU.x;
        tf.transform.translation.y = position_FLU.y;
        tf.transform.translation.z = position_FLU.z;

        var rotation_FLU = obj.rotation.To<FLU>();
        tf.transform.rotation.w = rotation_FLU.w;
        tf.transform.rotation.x = rotation_FLU.x;
        tf.transform.rotation.y = rotation_FLU.y;
        tf.transform.rotation.z = rotation_FLU.z;

        SendTransformInternal(handle, ref tf);

        tf.Delete();
    }

    public static void SendTransform(geometry_msgs_TransformStamped tf)
    {
        SendTransformInternal(handle, ref tf);
    }

    public static void SetROSTime(ref builtin_interfaces_Time stamp)
    {
        SetROSTimeInternal(handle, ref stamp);
    }

    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr InitInternal(AllocateDelegate allocator);

    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)]
    private static extern void DestroyInternal(IntPtr handle);

    [DllImport("libROSInterface.so", EntryPoint = "Publish", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr input);

    [DllImport("libROSInterface.so", EntryPoint = "Receive", CallingConvention = CallingConvention.Cdecl)]
    private static extern void ReceiveROSInternal(IntPtr handle, byte[] type, byte[] topic, ref IntPtr output);

    [DllImport("libROSInterface.so", EntryPoint = "SendTransform", CallingConvention = CallingConvention.Cdecl)]
    private static extern void SendTransformInternal(IntPtr handle, ref geometry_msgs_TransformStamped tf);

    [DllImport("libROSInterface.so", EntryPoint = "SetROSTime", CallingConvention = CallingConvention.Cdecl)]
    private static extern void SetROSTimeInternal(IntPtr handle, ref builtin_interfaces_Time stamp);
}