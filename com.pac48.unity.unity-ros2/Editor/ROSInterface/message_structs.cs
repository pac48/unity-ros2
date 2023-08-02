using System;
using System.Runtime.InteropServices;

public interface IROSMsg
{
    public string GetMsgType();
    public void Delete();
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct CArray
{
    public IntPtr ptr;
    public long length;
}


[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_RegionOfInterest : IROSMsg
{
    public uint x_offset; // uint
    public uint y_offset; // uint
    public uint height; // uint
    public uint width; // uint
    public bool do_rectify; // bool

    public string GetMsgType()
    {
        return "sensor_msgs::msg::RegionOfInterest";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Vector3 : IROSMsg
{
    public double x; // double
    public double y; // double
    public double z; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Vector3";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int8 : IROSMsg
{
    public sbyte data; // sbyte

    public string GetMsgType()
    {
        return "std_msgs::msg::Int8";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt16 : IROSMsg
{
    public ushort data; // ushort

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt16";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_ColorRGBA : IROSMsg
{
    public float r; // float
    public float g; // float
    public float b; // float
    public float a; // float

    public string GetMsgType()
    {
        return "std_msgs::msg::ColorRGBA";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_NavSatStatus : IROSMsg
{
    public sbyte STATUS_NO_FIX; // = -1 // sbyte
    public sbyte STATUS_FIX; // = 0 // sbyte
    public sbyte STATUS_SBAS_FIX; // = 1 // sbyte
    public sbyte STATUS_GBAS_FIX; // = 2 // sbyte
    public sbyte status; // sbyte
    public ushort SERVICE_GPS; // = 1 // ushort
    public ushort SERVICE_GLONASS; // = 2 // ushort
    public ushort SERVICE_COMPASS; // = 4 // ushort
    public ushort SERVICE_GALILEO; // = 8 // ushort
    public ushort service; // ushort

    public string GetMsgType()
    {
        return "sensor_msgs::msg::NavSatStatus";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Quaternion : IROSMsg
{
    public double x; // double
    public double y; // double
    public double z; // double
    public double w; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Quaternion";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt64 : IROSMsg
{
    public ulong data; // ulong

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt64";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int16 : IROSMsg
{
    public short data; // short

    public string GetMsgType()
    {
        return "std_msgs::msg::Int16";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_ChannelFloat32 : IROSMsg
{
    public IntPtr name; // = nullptr // IntPtr
    public CArray values; // = nullptr // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::ChannelFloat32";
    }

    public void Delete()
    {
        if (name != null)
        {
            Marshal.FreeHGlobal(name);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int32 : IROSMsg
{
    public int data; // int

    public string GetMsgType()
    {
        return "std_msgs::msg::Int32";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_MultiArrayDimension : IROSMsg
{
    public IntPtr label; // = nullptr // IntPtr
    public uint size; // uint
    public uint stride; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::MultiArrayDimension";
    }

    public void Delete()
    {
        if (label != null)
        {
            Marshal.FreeHGlobal(label);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_String : IROSMsg
{
    public IntPtr data; // = nullptr // IntPtr

    public string GetMsgType()
    {
        return "std_msgs::msg::String";
    }

    public void Delete()
    {
        if (data != null)
        {
            Marshal.FreeHGlobal(data);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_JoyFeedback : IROSMsg
{
    public byte TYPE_LED; // = 0 // byte
    public byte TYPE_RUMBLE; // = 1 // byte
    public byte TYPE_BUZZER; // = 2 // byte
    public byte type; // byte
    public byte id; // byte
    public float intensity; // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::JoyFeedback";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Char : IROSMsg
{
    public char data; // char

    public string GetMsgType()
    {
        return "std_msgs::msg::Char";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Empty : IROSMsg
{
    public string GetMsgType()
    {
        return "std_msgs::msg::Empty";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct builtin_interfaces_Time : IROSMsg
{
    public int sec; // int
    public uint nanosec; // uint

    public string GetMsgType()
    {
        return "builtin_interfaces::msg::Time";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt8 : IROSMsg
{
    public byte data; // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt8";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Bool : IROSMsg
{
    public bool data; // bool

    public string GetMsgType()
    {
        return "std_msgs::msg::Bool";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_JoyFeedbackArray : IROSMsg
{
    public CArray array; // = nullptr // sensor_msgs_JoyFeedback

    public string GetMsgType()
    {
        return "sensor_msgs::msg::JoyFeedbackArray";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.getStructArray<sensor_msgs_JoyFeedback>(array))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(array.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Header : IROSMsg
{
    public builtin_interfaces_Time stamp; // builtin_interfaces_Time
    public IntPtr frame_id; // = nullptr // IntPtr

    public string GetMsgType()
    {
        return "std_msgs::msg::Header";
    }

    public void Delete()
    {
        stamp.Delete();
        if (frame_id != null)
        {
            Marshal.FreeHGlobal(frame_id);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Point32 : IROSMsg
{
    public float x; // float
    public float y; // float
    public float z; // float

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Point32";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_QuaternionStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Quaternion quaternion; // geometry_msgs_Quaternion

    public string GetMsgType()
    {
        return "geometry_msgs::msg::QuaternionStamped";
    }

    public void Delete()
    {
        header.Delete();
        quaternion.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Vector3Stamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Vector3 vector; // geometry_msgs_Vector3

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Vector3Stamped";
    }

    public void Delete()
    {
        header.Delete();
        vector.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Range : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public byte ULTRASOUND; // = 0 // byte
    public byte INFRARED; // = 1 // byte
    public byte radiation_type; // byte
    public float field_of_view; // float
    public float min_range; // float
    public float max_range; // float
    public float range; // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Range";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Temperature : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public double temperature; // double
    public double variance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Temperature";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Accel : IROSMsg
{
    public geometry_msgs_Vector3 linear; // geometry_msgs_Vector3
    public geometry_msgs_Vector3 angular; // geometry_msgs_Vector3

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Accel";
    }

    public void Delete()
    {
        linear.Delete();
        angular.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_PointField : IROSMsg
{
    public byte INT8; // = 1 // byte
    public byte UINT8; // = 2 // byte
    public byte INT16; // = 3 // byte
    public byte UINT16; // = 4 // byte
    public byte INT32; // = 5 // byte
    public byte UINT32; // = 6 // byte
    public byte FLOAT32; // = 7 // byte
    public byte FLOAT64; // = 8 // byte
    public IntPtr name; // = nullptr // IntPtr
    public uint offset; // uint
    public byte datatype; // byte
    public uint count; // uint

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointField";
    }

    public void Delete()
    {
        if (name != null)
        {
            Marshal.FreeHGlobal(name);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Transform : IROSMsg
{
    public geometry_msgs_Vector3 translation; // geometry_msgs_Vector3
    public geometry_msgs_Quaternion rotation; // geometry_msgs_Quaternion

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Transform";
    }

    public void Delete()
    {
        translation.Delete();
        rotation.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_MultiArrayLayout : IROSMsg
{
    public CArray dim; // = nullptr // std_msgs_MultiArrayDimension
    public uint data_offset; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::MultiArrayLayout";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.getStructArray<std_msgs_MultiArrayDimension>(dim))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(dim.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_LaserEcho : IROSMsg
{
    public CArray echoes; // = nullptr // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::LaserEcho";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int64 : IROSMsg
{
    public long data; // long

    public string GetMsgType()
    {
        return "std_msgs::msg::Int64";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_FluidPressure : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public double fluid_pressure; // double
    public double variance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::FluidPressure";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Pose2D : IROSMsg
{
    public double x; // double
    public double y; // double
    public double theta; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Pose2D";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Byte : IROSMsg
{
    public byte data; // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::Byte";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // ulong

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Point : IROSMsg
{
    public double x; // double
    public double y; // double
    public double z; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Point";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_JointState : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray name; // = nullptr // IntPtr
    public CArray position; // = nullptr // double
    public CArray velocity; // = nullptr // double
    public CArray effort; // = nullptr // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::JointState";
    }

    public void Delete()
    {
        header.Delete();
        for (int i = 0; i < name.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(name.ptr, Marshal.SizeOf(typeof(IntPtr)) * i);
            if (ptr != null)
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        Marshal.FreeHGlobal(name.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt8MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt8MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_TransformStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr child_frame_id; // = nullptr // IntPtr
    public geometry_msgs_Transform transform; // geometry_msgs_Transform

    public string GetMsgType()
    {
        return "geometry_msgs::msg::TransformStamped";
    }

    public void Delete()
    {
        header.Delete();
        if (child_frame_id != null)
        {
            Marshal.FreeHGlobal(child_frame_id);
        }

        transform.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // int

    public string GetMsgType()
    {
        return "std_msgs::msg::Int32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_PointCloud2 : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public CArray fields; // = nullptr // sensor_msgs_PointField
    public bool is_bigendian; // bool
    public uint point_step; // uint
    public uint row_step; // uint
    public CArray data; // = nullptr // byte
    public bool is_dense; // bool

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointCloud2";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<sensor_msgs_PointField>(fields))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(fields.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_CompressedImage : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr format; // = nullptr // IntPtr
    public CArray data; // = nullptr // byte

    public string GetMsgType()
    {
        return "sensor_msgs::msg::CompressedImage";
    }

    public void Delete()
    {
        header.Delete();
        if (format != null)
        {
            Marshal.FreeHGlobal(format);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PointStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Point point; // geometry_msgs_Point

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PointStamped";
    }

    public void Delete()
    {
        header.Delete();
        point.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Inertia : IROSMsg
{
    public double m; // double
    public geometry_msgs_Vector3 com; // geometry_msgs_Vector3
    public double ixx; // double
    public double ixy; // double
    public double ixz; // double
    public double iyy; // double
    public double iyz; // double
    public double izz; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Inertia";
    }

    public void Delete()
    {
        com.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Float32 : IROSMsg
{
    public float data; // float

    public string GetMsgType()
    {
        return "std_msgs::msg::Float32";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Joy : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray axes; // = nullptr // float
    public CArray buttons; // = nullptr // int

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Joy";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt32 : IROSMsg
{
    public uint data; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt32";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_AccelWithCovariance : IROSMsg
{
    public geometry_msgs_Accel accel; // geometry_msgs_Accel

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] covariance; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::AccelWithCovariance";
    }

    public void Delete()
    {
        accel.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Float64 : IROSMsg
{
    public double data; // double

    public string GetMsgType()
    {
        return "std_msgs::msg::Float64";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct builtin_interfaces_Duration : IROSMsg
{
    public int sec; // int
    public uint nanosec; // uint

    public string GetMsgType()
    {
        return "builtin_interfaces::msg::Duration";
    }

    public void Delete()
    {
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Twist : IROSMsg
{
    public geometry_msgs_Vector3 linear; // geometry_msgs_Vector3
    public geometry_msgs_Vector3 angular; // geometry_msgs_Vector3

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Twist";
    }

    public void Delete()
    {
        linear.Delete();
        angular.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int16MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // short

    public string GetMsgType()
    {
        return "std_msgs::msg::Int16MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int8MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // sbyte

    public string GetMsgType()
    {
        return "std_msgs::msg::Int8MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_CameraInfo : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public IntPtr distortion_model; // = nullptr // IntPtr
    public CArray d; // = nullptr // double

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] k; // double

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] r; // double

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 12)]
    public double[] p; // double

    public uint binning_x; // uint
    public uint binning_y; // uint
    public sensor_msgs_RegionOfInterest roi; // sensor_msgs_RegionOfInterest

    public string GetMsgType()
    {
        return "sensor_msgs::msg::CameraInfo";
    }

    public void Delete()
    {
        header.Delete();
        if (distortion_model != null)
        {
            Marshal.FreeHGlobal(distortion_model);
        }

        roi.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_RelativeHumidity : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public double relative_humidity; // double
    public double variance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::RelativeHumidity";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Wrench : IROSMsg
{
    public geometry_msgs_Vector3 force; // geometry_msgs_Vector3
    public geometry_msgs_Vector3 torque; // geometry_msgs_Vector3

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Wrench";
    }

    public void Delete()
    {
        force.Delete();
        torque.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_MultiEchoLaserScan : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public float angle_min; // float
    public float angle_max; // float
    public float angle_increment; // float
    public float time_increment; // float
    public float scan_time; // float
    public float range_min; // float
    public float range_max; // float
    public CArray ranges; // = nullptr // sensor_msgs_LaserEcho
    public CArray intensities; // = nullptr // sensor_msgs_LaserEcho

    public string GetMsgType()
    {
        return "sensor_msgs::msg::MultiEchoLaserScan";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<sensor_msgs_LaserEcho>(ranges))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(ranges.ptr);
        foreach (var inst in ROSInterface.getStructArray<sensor_msgs_LaserEcho>(intensities))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(intensities.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // long

    public string GetMsgType()
    {
        return "std_msgs::msg::Int64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_TimeReference : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public builtin_interfaces_Time time_ref; // builtin_interfaces_Time
    public IntPtr source; // = nullptr // IntPtr

    public string GetMsgType()
    {
        return "sensor_msgs::msg::TimeReference";
    }

    public void Delete()
    {
        header.Delete();
        time_ref.Delete();
        if (source != null)
        {
            Marshal.FreeHGlobal(source);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_AccelStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Accel accel; // geometry_msgs_Accel

    public string GetMsgType()
    {
        return "geometry_msgs::msg::AccelStamped";
    }

    public void Delete()
    {
        header.Delete();
        accel.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_InertiaStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Inertia inertia; // geometry_msgs_Inertia

    public string GetMsgType()
    {
        return "geometry_msgs::msg::InertiaStamped";
    }

    public void Delete()
    {
        header.Delete();
        inertia.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Illuminance : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public double illuminance; // double
    public double variance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Illuminance";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_ByteMultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::ByteMultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Imu : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Quaternion orientation; // geometry_msgs_Quaternion

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] orientation_covariance; // double

    public geometry_msgs_Vector3 angular_velocity; // geometry_msgs_Vector3

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] angular_velocity_covariance; // double

    public geometry_msgs_Vector3 linear_acceleration; // geometry_msgs_Vector3

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] linear_acceleration_covariance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Imu";
    }

    public void Delete()
    {
        header.Delete();
        orientation.Delete();
        angular_velocity.Delete();
        linear_acceleration.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_PointCloud : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray points; // = nullptr // geometry_msgs_Point32
    public CArray channels; // = nullptr // sensor_msgs_ChannelFloat32

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointCloud";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Point32>(points))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(points.ptr);
        foreach (var inst in ROSInterface.getStructArray<sensor_msgs_ChannelFloat32>(channels))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(channels.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_LaserScan : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public float angle_min; // float
    public float angle_max; // float
    public float angle_increment; // float
    public float time_increment; // float
    public float scan_time; // float
    public float range_min; // float
    public float range_max; // float
    public CArray ranges; // = nullptr // float
    public CArray intensities; // = nullptr // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::LaserScan";
    }

    public void Delete()
    {
        header.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Polygon : IROSMsg
{
    public CArray points; // = nullptr // geometry_msgs_Point32

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Polygon";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Point32>(points))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(points.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Float32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // float

    public string GetMsgType()
    {
        return "std_msgs::msg::Float32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_NavSatFix : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public sensor_msgs_NavSatStatus status; // sensor_msgs_NavSatStatus
    public double latitude; // double
    public double longitude; // double
    public double altitude; // double

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] position_covariance; // double

    public byte COVARIANCE_TYPE_UNKNOWN; // = 0 // byte
    public byte COVARIANCE_TYPE_APPROXIMATED; // = 1 // byte
    public byte COVARIANCE_TYPE_DIAGONAL_KNOWN; // = 2 // byte
    public byte COVARIANCE_TYPE_KNOWN; // = 3 // byte
    public byte position_covariance_type; // byte

    public string GetMsgType()
    {
        return "sensor_msgs::msg::NavSatFix";
    }

    public void Delete()
    {
        header.Delete();
        status.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Float64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // double

    public string GetMsgType()
    {
        return "std_msgs::msg::Float64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_TwistStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Twist twist; // geometry_msgs_Twist

    public string GetMsgType()
    {
        return "geometry_msgs::msg::TwistStamped";
    }

    public void Delete()
    {
        header.Delete();
        twist.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_MagneticField : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Vector3 magnetic_field; // geometry_msgs_Vector3

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
    public double[] magnetic_field_covariance; // double

    public string GetMsgType()
    {
        return "sensor_msgs::msg::MagneticField";
    }

    public void Delete()
    {
        header.Delete();
        magnetic_field.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_MultiDOFJointState : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray joint_names; // = nullptr // IntPtr
    public CArray transforms; // = nullptr // geometry_msgs_Transform
    public CArray twist; // = nullptr // geometry_msgs_Twist
    public CArray wrench; // = nullptr // geometry_msgs_Wrench

    public string GetMsgType()
    {
        return "sensor_msgs::msg::MultiDOFJointState";
    }

    public void Delete()
    {
        header.Delete();
        for (int i = 0; i < joint_names.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(joint_names.ptr, Marshal.SizeOf(typeof(IntPtr)) * i);
            if (ptr != null)
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        Marshal.FreeHGlobal(joint_names.ptr);
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Transform>(transforms))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(transforms.ptr);
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Twist>(twist))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(twist.ptr);
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Wrench>(wrench))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(wrench.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_BatteryState : IROSMsg
{
    public byte POWER_SUPPLY_STATUS_UNKNOWN; // = 0 // byte
    public byte POWER_SUPPLY_STATUS_CHARGING; // = 1 // byte
    public byte POWER_SUPPLY_STATUS_DISCHARGING; // = 2 // byte
    public byte POWER_SUPPLY_STATUS_NOT_CHARGING; // = 3 // byte
    public byte POWER_SUPPLY_STATUS_FULL; // = 4 // byte
    public byte POWER_SUPPLY_HEALTH_UNKNOWN; // = 0 // byte
    public byte POWER_SUPPLY_HEALTH_GOOD; // = 1 // byte
    public byte POWER_SUPPLY_HEALTH_OVERHEAT; // = 2 // byte
    public byte POWER_SUPPLY_HEALTH_DEAD; // = 3 // byte
    public byte POWER_SUPPLY_HEALTH_OVERVOLTAGE; // = 4 // byte
    public byte POWER_SUPPLY_HEALTH_UNSPEC_FAILURE; // = 5 // byte
    public byte POWER_SUPPLY_HEALTH_COLD; // = 6 // byte
    public byte POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE; // = 7 // byte
    public byte POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE; // = 8 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_UNKNOWN; // = 0 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_NIMH; // = 1 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_LION; // = 2 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_LIPO; // = 3 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_LIFE; // = 4 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_NICD; // = 5 // byte
    public byte POWER_SUPPLY_TECHNOLOGY_LIMN; // = 6 // byte
    public std_msgs_Header header; // std_msgs_Header
    public float voltage; // float
    public float temperature; // float
    public float current; // float
    public float charge; // float
    public float capacity; // float
    public float design_capacity; // float
    public float percentage; // float
    public byte power_supply_status; // byte
    public byte power_supply_health; // byte
    public byte power_supply_technology; // byte
    public bool present; // bool
    public CArray cell_voltage; // = nullptr // float
    public CArray cell_temperature; // = nullptr // float
    public IntPtr location; // = nullptr // IntPtr
    public IntPtr serial_number; // = nullptr // IntPtr

    public string GetMsgType()
    {
        return "sensor_msgs::msg::BatteryState";
    }

    public void Delete()
    {
        header.Delete();
        if (location != null)
        {
            Marshal.FreeHGlobal(location);
        }

        if (serial_number != null)
        {
            Marshal.FreeHGlobal(serial_number);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Image : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public IntPtr encoding; // = nullptr // IntPtr
    public byte is_bigendian; // byte
    public uint step; // uint
    public CArray data; // = nullptr // byte

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Image";
    }

    public void Delete()
    {
        header.Delete();
        if (encoding != null)
        {
            Marshal.FreeHGlobal(encoding);
        }
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PolygonStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Polygon polygon; // geometry_msgs_Polygon

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PolygonStamped";
    }

    public void Delete()
    {
        header.Delete();
        polygon.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_TwistWithCovariance : IROSMsg
{
    public geometry_msgs_Twist twist; // geometry_msgs_Twist

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] covariance; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::TwistWithCovariance";
    }

    public void Delete()
    {
        twist.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt16MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // = nullptr // ushort

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt16MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_GridCells : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public float cell_width; // float
    public float cell_height; // float
    public CArray cells; // = nullptr // geometry_msgs_Point

    public string GetMsgType()
    {
        return "nav_msgs::msg::GridCells";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Point>(cells))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(cells.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_Pose : IROSMsg
{
    public geometry_msgs_Point position; // geometry_msgs_Point
    public geometry_msgs_Quaternion orientation; // geometry_msgs_Quaternion

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Pose";
    }

    public void Delete()
    {
        position.Delete();
        orientation.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_AccelWithCovarianceStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_AccelWithCovariance accel; // geometry_msgs_AccelWithCovariance

    public string GetMsgType()
    {
        return "geometry_msgs::msg::AccelWithCovarianceStamped";
    }

    public void Delete()
    {
        header.Delete();
        accel.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_WrenchStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Wrench wrench; // geometry_msgs_Wrench

    public string GetMsgType()
    {
        return "geometry_msgs::msg::WrenchStamped";
    }

    public void Delete()
    {
        header.Delete();
        wrench.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_TwistWithCovarianceStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_TwistWithCovariance twist; // geometry_msgs_TwistWithCovariance

    public string GetMsgType()
    {
        return "geometry_msgs::msg::TwistWithCovarianceStamped";
    }

    public void Delete()
    {
        header.Delete();
        twist.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PoseArray : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray poses; // = nullptr // geometry_msgs_Pose

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PoseArray";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_Pose>(poses))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(poses.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PoseStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_Pose pose; // geometry_msgs_Pose

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PoseStamped";
    }

    public void Delete()
    {
        header.Delete();
        pose.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_MapMetaData : IROSMsg
{
    public builtin_interfaces_Time map_load_time; // builtin_interfaces_Time
    public float resolution; // float
    public uint width; // uint
    public uint height; // uint
    public geometry_msgs_Pose origin; // geometry_msgs_Pose

    public string GetMsgType()
    {
        return "nav_msgs::msg::MapMetaData";
    }

    public void Delete()
    {
        map_load_time.Delete();
        origin.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PoseWithCovariance : IROSMsg
{
    public geometry_msgs_Pose pose; // geometry_msgs_Pose

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] covariance; // double

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PoseWithCovariance";
    }

    public void Delete()
    {
        pose.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_Odometry : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr child_frame_id; // = nullptr // IntPtr
    public geometry_msgs_PoseWithCovariance pose; // geometry_msgs_PoseWithCovariance
    public geometry_msgs_TwistWithCovariance twist; // geometry_msgs_TwistWithCovariance

    public string GetMsgType()
    {
        return "nav_msgs::msg::Odometry";
    }

    public void Delete()
    {
        header.Delete();
        if (child_frame_id != null)
        {
            Marshal.FreeHGlobal(child_frame_id);
        }

        pose.Delete();
        twist.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_Path : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray poses; // = nullptr // geometry_msgs_PoseStamped

    public string GetMsgType()
    {
        return "nav_msgs::msg::Path";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.getStructArray<geometry_msgs_PoseStamped>(poses))
        {
            inst.Delete();
        }

        Marshal.FreeHGlobal(poses.ptr);
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PoseWithCovarianceStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public geometry_msgs_PoseWithCovariance pose; // geometry_msgs_PoseWithCovariance

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PoseWithCovarianceStamped";
    }

    public void Delete()
    {
        header.Delete();
        pose.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_OccupancyGrid : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public nav_msgs_MapMetaData info; // nav_msgs_MapMetaData
    public CArray data; // = nullptr // sbyte

    public string GetMsgType()
    {
        return "nav_msgs::msg::OccupancyGrid";
    }

    public void Delete()
    {
        header.Delete();
        info.Delete();
    }
}