using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.PlayerLoop;

public class ROSInterface : MonoBehaviour
{
    // native
    private static IntPtr handle;
    private static List<IntPtr> memory;

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
            memory = new List<IntPtr>();
        }
    }

    private static IntPtr Allocate(int numBytes)
    {
        if (handle == IntPtr.Zero) Init();
        IntPtr ptr = Marshal.AllocHGlobal(numBytes);
        memory.Add(ptr);
        return ptr;
    }

    public static void FreeAll()
    {
        foreach (var ptr in memory)
        {
            Marshal.FreeHGlobal(ptr);
        }

        memory = new List<IntPtr>();
    }

    public delegate IntPtr AllocateDelegate(int numBytes);

    private static IntPtr AllocateString(string str)
    {
        if (handle == IntPtr.Zero) Init();
        IntPtr ptr = Marshal.StringToHGlobalAnsi(str);
        memory.Add(ptr);
        return ptr;
    }

    public static IntPtr[] AllocateArrayPtr<T>(ref CArray array, int len)
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

    public static void allocateStructArray<T>(ref CArray array, T[] msgs) where T : IROSMsg
    {
        IntPtr[] arr = AllocateArrayPtr<T>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.StructureToPtr(msgs[i], arr[i], false);
        }
    }

    public static void allocateStringArray(ref CArray array, string[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<IntPtr>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.WriteIntPtr(arr[i], 0, AllocateString(msgs[i] + "\0"));
        }
    }

    public static void allocateDoubleArray(ref CArray array, double[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<double>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
    }

    public static void allocateFloatArray(ref CArray array, float[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<float>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
    }

    public static void allocateString(ref IntPtr ptr, string str)
    {
        ptr = AllocateString(str + "\0");
    }

    public static string getString(IntPtr ptr)
    {
        if (ptr == IntPtr.Zero) return "";
        var tmp = Marshal.ReadIntPtr(ptr); // TODO remove this
        return Marshal.PtrToStringAnsi(ptr);
    }

    public static string[] getStringArray(CArray array)
    {
        if (array.ptr == IntPtr.Zero) return new string[0];
        string[] ret = new string[array.length];
        int elementSize = Marshal.SizeOf(typeof(IntPtr));
        for (int i = 0; i < array.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(array.ptr, elementSize * i);
            ret[i] = getString(ptr);
        }

        return ret;
    }

    public static double[] getDoubleArray(CArray array)
    {
        if (array.ptr == IntPtr.Zero) return new double[0];
        double[] ret = new double[array.length];
        Marshal.Copy(array.ptr, ret, 0, (int)array.length);
        return ret;
    }

    public static T getStruct<T>(IntPtr ptr) where T : IROSMsg, new()
    {
        if (ptr == IntPtr.Zero) return new T();
        return (T)Marshal.PtrToStructure(ptr, typeof(T));
    }

    public static T[] getStructArray<T>(CArray array) where T : IROSMsg, new()
    {
        if (array.ptr == IntPtr.Zero) return new T[0];
        T[] ret = new T[array.length];
        int elementSize = Marshal.SizeOf(typeof(T));
        for (int i = 0; i < array.length; i++)
        {
            ret[i] = getStruct<T>(array.ptr + elementSize * i);
        }

        return ret;
    }

    public void PublishROS<T>(ref T msg, string topic_str) where T : unmanaged, IROSMsg
    {
        if (handle == IntPtr.Zero) Init();
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(msg.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");

        IntPtr ptr = Allocate(Marshal.SizeOf(msg));
        Marshal.StructureToPtr(msg, ptr, false);
        PublishROSInternal(handle, type, topic, ptr);
    }

    public T ReceiveROS<T>(string topic_str) where T : unmanaged, IROSMsg
    {
        var tmp = new T();
        if (handle == IntPtr.Zero) Init();
        ASCIIEncoding ascii = new ASCIIEncoding();
        byte[] type = ascii.GetBytes(tmp.GetMsgType() + "\0");
        byte[] topic = ascii.GetBytes(topic_str + "\0");
        IntPtr ptr = new IntPtr();
        ReceiveROSInternal(handle, type, topic, ref ptr);
        var msg = getStruct<T>(ptr);
        return msg;
    }

    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr InitInternal(AllocateDelegate allocator);

    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)]
    private static extern void DestroyInternal(IntPtr handle);

    [DllImport("libROSInterface.so", EntryPoint = "Publish", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr input);

    [DllImport("libROSInterface.so", EntryPoint = "Receive", CallingConvention = CallingConvention.Cdecl)]
    private static extern void ReceiveROSInternal(IntPtr handle, byte[] type, byte[] topic, ref IntPtr output);
}