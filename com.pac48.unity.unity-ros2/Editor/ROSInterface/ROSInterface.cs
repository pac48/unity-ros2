using System;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class ROSInterface : MonoBehaviour
{
    // native
    private IntPtr handle;

    void OnDestroy()
    {
        if (handle != null)
        {
            DestroyInternal(handle);
        }
    }

    void Start()
    {
        if (handle != null)
        {
            handle = InitInternal();
        }
    }

    public IntPtr[] AllocateArrayPtr<T>(ref CArray array, int len)
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

    public void allocateStructArray<T>(ref CArray array, T[] msgs) where T : IROSMsg
    {
        IntPtr[] arr = AllocateArrayPtr<T>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.StructureToPtr(msgs[i], arr[i], false);
        }
    }

    public void allocateStringArray(ref CArray array, string[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<IntPtr>(ref array, msgs.Length);
        for (int i = 0; i < msgs.Length; i++)
        {
            Marshal.WriteIntPtr(arr[i], 0, Marshal.StringToHGlobalAnsi(msgs[i] + "\0"));
        }
    }

    public void allocateDoubleArray(ref CArray array, double[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<double>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
    }

    public void allocateFloatArray(ref CArray array, float[] msgs)
    {
        IntPtr[] arr = AllocateArrayPtr<float>(ref array, msgs.Length);
        Marshal.Copy(msgs, 0, arr[0], (int)array.length);
    }

    public void allocateString(ref IntPtr ptr, string str)
    {
        if (ptr != IntPtr.Zero)
        {
            Marshal.FreeHGlobal(ptr);
        }

        ptr = Marshal.StringToHGlobalAnsi(str + "\0");
    }

    public string getString(IntPtr ptr)
    {
        return Marshal.PtrToStringAnsi(ptr);
    }

    public string[] getStringArray(CArray array)
    {
        string[] ret = new string[array.length];
        int elementSize = Marshal.SizeOf(typeof(IntPtr));
        for (int i = 0; i < array.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(array.ptr, elementSize * i);
            ret[i] = getString(ptr);
        }

        return ret;
    }

    public double[] getDoubleArray(CArray array)
    {
        double[] ret = new double[array.length];
        Marshal.Copy(array.ptr, ret, 0, (int)array.length);
        return ret;
    }

    public T getStruct<T>(IntPtr ptr) where T : IROSMsg
    {
        return (T)Marshal.PtrToStructure(ptr, typeof(T));
    }

    public T[] getStructArray<T>(CArray array) where T : IROSMsg
    {
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
        msg = getStruct<T>(ptr);
        Marshal.FreeHGlobal(ptr);
    }

    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr InitInternal();

    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)]
    private static extern void DestroyInternal(IntPtr handle);

    [DllImport("libROSInterface.so", EntryPoint = "Publish", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr input);

    [DllImport("libROSInterface.so", EntryPoint = "Receive", CallingConvention = CallingConvention.Cdecl)]
    private static extern void ReceiveROSInternal(IntPtr handle, byte[] type, byte[] topic, IntPtr output);
}