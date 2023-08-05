using System;
using System.Runtime.InteropServices;

public interface IROSMsg
{
    public string GetMsgType();
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct CArray
{
    public IntPtr ptr;
    public long length;
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
public struct sensor_msgs_LaserEcho : IROSMsg
{
    public CArray echoes; // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::LaserEcho";
    }

    public void Delete()
    {
        ROSInterface.Free(echoes.ptr);
        echoes.ptr = IntPtr.Zero;
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
public struct geometry_msgs_Polygon : IROSMsg
{
    public CArray points; // geometry_msgs_Point32

    public string GetMsgType()
    {
        return "geometry_msgs::msg::Polygon";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Point32>(points))
        {
            inst.Delete();
        }

        ROSInterface.Free(points.ptr);
        points.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Header : IROSMsg
{
    public builtin_interfaces_Time stamp; // builtin_interfaces_Time
    public IntPtr frame_id; // IntPtr

    public string GetMsgType()
    {
        return "std_msgs::msg::Header";
    }

    public void Delete()
    {
        stamp.Delete();
        ROSInterface.Free(frame_id);
        frame_id = IntPtr.Zero;
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
public struct sensor_msgs_Joy : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray axes; // float
    public CArray buttons; // int

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Joy";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(axes.ptr);
        axes.ptr = IntPtr.Zero;
        ROSInterface.Free(buttons.ptr);
        buttons.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_CompressedImage : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr format; // IntPtr
    public CArray data; // byte

    public string GetMsgType()
    {
        return "sensor_msgs::msg::CompressedImage";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(format);
        format = IntPtr.Zero;
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct std_msgs_String : IROSMsg
{
    public IntPtr data; // IntPtr

    public string GetMsgType()
    {
        return "std_msgs::msg::String";
    }

    public void Delete()
    {
        ROSInterface.Free(data);
        data = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_ChannelFloat32 : IROSMsg
{
    public IntPtr name; // IntPtr
    public CArray values; // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::ChannelFloat32";
    }

    public void Delete()
    {
        ROSInterface.Free(name);
        name = IntPtr.Zero;
        ROSInterface.Free(values.ptr);
        values.ptr = IntPtr.Zero;
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
    public CArray ranges; // float
    public CArray intensities; // float

    public string GetMsgType()
    {
        return "sensor_msgs::msg::LaserScan";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(ranges.ptr);
        ranges.ptr = IntPtr.Zero;
        ROSInterface.Free(intensities.ptr);
        intensities.ptr = IntPtr.Zero;
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
    public CArray ranges; // sensor_msgs_LaserEcho
    public CArray intensities; // sensor_msgs_LaserEcho

    public string GetMsgType()
    {
        return "sensor_msgs::msg::MultiEchoLaserScan";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<sensor_msgs_LaserEcho>(ranges))
        {
            inst.Delete();
        }

        ROSInterface.Free(ranges.ptr);
        ranges.ptr = IntPtr.Zero;
        foreach (var inst in ROSInterface.GetStructArray<sensor_msgs_LaserEcho>(intensities))
        {
            inst.Delete();
        }

        ROSInterface.Free(intensities.ptr);
        intensities.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_MultiArrayDimension : IROSMsg
{
    public IntPtr label; // IntPtr
    public uint size; // uint
    public uint stride; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::MultiArrayDimension";
    }

    public void Delete()
    {
        ROSInterface.Free(label);
        label = IntPtr.Zero;
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
    public IntPtr name; // IntPtr
    public uint offset; // uint
    public byte datatype; // byte
    public uint count; // uint

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointField";
    }

    public void Delete()
    {
        ROSInterface.Free(name);
        name = IntPtr.Zero;
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
public struct std_msgs_MultiArrayLayout : IROSMsg
{
    public CArray dim; // std_msgs_MultiArrayDimension
    public uint data_offset; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::MultiArrayLayout";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.GetStructArray<std_msgs_MultiArrayDimension>(dim))
        {
            inst.Delete();
        }

        ROSInterface.Free(dim.ptr);
        dim.ptr = IntPtr.Zero;
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
public struct std_msgs_UInt16MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // ushort

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt16MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct nav_msgs_GridCells : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public float cell_width; // float
    public float cell_height; // float
    public CArray cells; // geometry_msgs_Point

    public string GetMsgType()
    {
        return "nav_msgs::msg::GridCells";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Point>(cells))
        {
            inst.Delete();
        }

        ROSInterface.Free(cells.ptr);
        cells.ptr = IntPtr.Zero;
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
public struct std_msgs_Int32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // int

    public string GetMsgType()
    {
        return "std_msgs::msg::Int32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_UInt32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct intera_core_msgs_JointCommand : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public int mode; // int
    public CArray names; // IntPtr
    public CArray position; // double
    public CArray velocity; // double
    public CArray acceleration; // double
    public CArray effort; // double
    public int POSITION_MODE; // = 1 // int
    public int VELOCITY_MODE; // = 2 // int
    public int TORQUE_MODE; // = 3 // int
    public int TRAJECTORY_MODE; // = 4 // int

    public string GetMsgType()
    {
        return "intera_core_msgs::msg::JointCommand";
    }

    public void Delete()
    {
        header.Delete();
        for (int i = 0; i < names.length; i++)
        {
            IntPtr ptr = Marshal.ReadIntPtr(names.ptr, Marshal.SizeOf(typeof(IntPtr)) * i);
            ROSInterface.Free(ptr);
            ptr = IntPtr.Zero;
        }

        ROSInterface.Free(names.ptr);
        names.ptr = IntPtr.Zero;
        ROSInterface.Free(position.ptr);
        position.ptr = IntPtr.Zero;
        ROSInterface.Free(velocity.ptr);
        velocity.ptr = IntPtr.Zero;
        ROSInterface.Free(acceleration.ptr);
        acceleration.ptr = IntPtr.Zero;
        ROSInterface.Free(effort.ptr);
        effort.ptr = IntPtr.Zero;
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
public struct sensor_msgs_JointState : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray name; // IntPtr
    public CArray position; // double
    public CArray velocity; // double
    public CArray effort; // double

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
            ROSInterface.Free(ptr);
            ptr = IntPtr.Zero;
        }

        ROSInterface.Free(name.ptr);
        name.ptr = IntPtr.Zero;
        ROSInterface.Free(position.ptr);
        position.ptr = IntPtr.Zero;
        ROSInterface.Free(velocity.ptr);
        velocity.ptr = IntPtr.Zero;
        ROSInterface.Free(effort.ptr);
        effort.ptr = IntPtr.Zero;
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
public struct std_msgs_ByteMultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::ByteMultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
    public CArray cell_voltage; // float
    public CArray cell_temperature; // float
    public IntPtr location; // IntPtr
    public IntPtr serial_number; // IntPtr

    public string GetMsgType()
    {
        return "sensor_msgs::msg::BatteryState";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(cell_voltage.ptr);
        cell_voltage.ptr = IntPtr.Zero;
        ROSInterface.Free(cell_temperature.ptr);
        cell_temperature.ptr = IntPtr.Zero;
        ROSInterface.Free(location);
        location = IntPtr.Zero;
        ROSInterface.Free(serial_number);
        serial_number = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_Image : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public IntPtr encoding; // IntPtr
    public byte is_bigendian; // byte
    public uint step; // uint
    public CArray data; // byte

    public string GetMsgType()
    {
        return "sensor_msgs::msg::Image";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(encoding);
        encoding = IntPtr.Zero;
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_JoyFeedbackArray : IROSMsg
{
    public CArray array; // sensor_msgs_JoyFeedback

    public string GetMsgType()
    {
        return "sensor_msgs::msg::JoyFeedbackArray";
    }

    public void Delete()
    {
        foreach (var inst in ROSInterface.GetStructArray<sensor_msgs_JoyFeedback>(array))
        {
            inst.Delete();
        }

        ROSInterface.Free(array.ptr);
        array.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_TimeReference : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public builtin_interfaces_Time time_ref; // builtin_interfaces_Time
    public IntPtr source; // IntPtr

    public string GetMsgType()
    {
        return "sensor_msgs::msg::TimeReference";
    }

    public void Delete()
    {
        header.Delete();
        time_ref.Delete();
        ROSInterface.Free(source);
        source = IntPtr.Zero;
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
public struct std_msgs_Int64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // long

    public string GetMsgType()
    {
        return "std_msgs::msg::Int64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct sensor_msgs_CameraInfo : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public IntPtr distortion_model; // IntPtr
    public CArray d; // double

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
        ROSInterface.Free(distortion_model);
        distortion_model = IntPtr.Zero;
        ROSInterface.Free(d.ptr);
        d.ptr = IntPtr.Zero;
        roi.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_PointCloud2 : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public uint height; // uint
    public uint width; // uint
    public CArray fields; // sensor_msgs_PointField
    public bool is_bigendian; // bool
    public uint point_step; // uint
    public uint row_step; // uint
    public CArray data; // byte
    public bool is_dense; // bool

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointCloud2";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<sensor_msgs_PointField>(fields))
        {
            inst.Delete();
        }

        ROSInterface.Free(fields.ptr);
        fields.ptr = IntPtr.Zero;
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct std_msgs_Int8MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // sbyte

    public string GetMsgType()
    {
        return "std_msgs::msg::Int8MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct std_msgs_UInt64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // ulong

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_MultiDOFJointState : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray joint_names; // IntPtr
    public CArray transforms; // geometry_msgs_Transform
    public CArray twist; // geometry_msgs_Twist
    public CArray wrench; // geometry_msgs_Wrench

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
            ROSInterface.Free(ptr);
            ptr = IntPtr.Zero;
        }

        ROSInterface.Free(joint_names.ptr);
        joint_names.ptr = IntPtr.Zero;
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Transform>(transforms))
        {
            inst.Delete();
        }

        ROSInterface.Free(transforms.ptr);
        transforms.ptr = IntPtr.Zero;
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Twist>(twist))
        {
            inst.Delete();
        }

        ROSInterface.Free(twist.ptr);
        twist.ptr = IntPtr.Zero;
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Wrench>(wrench))
        {
            inst.Delete();
        }

        ROSInterface.Free(wrench.ptr);
        wrench.ptr = IntPtr.Zero;
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
public struct std_msgs_UInt8MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // byte

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt8MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct sensor_msgs_PointCloud : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray points; // geometry_msgs_Point32
    public CArray channels; // sensor_msgs_ChannelFloat32

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointCloud";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Point32>(points))
        {
            inst.Delete();
        }

        ROSInterface.Free(points.ptr);
        points.ptr = IntPtr.Zero;
        foreach (var inst in ROSInterface.GetStructArray<sensor_msgs_ChannelFloat32>(channels))
        {
            inst.Delete();
        }

        ROSInterface.Free(channels.ptr);
        channels.ptr = IntPtr.Zero;
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
public struct std_msgs_Int16MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // short

    public string GetMsgType()
    {
        return "std_msgs::msg::Int16MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct std_msgs_Float32MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // float

    public string GetMsgType()
    {
        return "std_msgs::msg::Float32MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct std_msgs_Float64MultiArray : IROSMsg
{
    public std_msgs_MultiArrayLayout layout; // std_msgs_MultiArrayLayout
    public CArray data; // double

    public string GetMsgType()
    {
        return "std_msgs::msg::Float64MultiArray";
    }

    public void Delete()
    {
        layout.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_Path : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray poses; // geometry_msgs_PoseStamped

    public string GetMsgType()
    {
        return "nav_msgs::msg::Path";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_PoseStamped>(poses))
        {
            inst.Delete();
        }

        ROSInterface.Free(poses.ptr);
        poses.ptr = IntPtr.Zero;
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
public struct geometry_msgs_TransformStamped : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr child_frame_id; // IntPtr
    public geometry_msgs_Transform transform; // geometry_msgs_Transform

    public string GetMsgType()
    {
        return "geometry_msgs::msg::TransformStamped";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(child_frame_id);
        child_frame_id = IntPtr.Zero;
        transform.Delete();
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct geometry_msgs_PoseArray : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public CArray poses; // geometry_msgs_Pose

    public string GetMsgType()
    {
        return "geometry_msgs::msg::PoseArray";
    }

    public void Delete()
    {
        header.Delete();
        foreach (var inst in ROSInterface.GetStructArray<geometry_msgs_Pose>(poses))
        {
            inst.Delete();
        }

        ROSInterface.Free(poses.ptr);
        poses.ptr = IntPtr.Zero;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct nav_msgs_OccupancyGrid : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public nav_msgs_MapMetaData info; // nav_msgs_MapMetaData
    public CArray data; // sbyte

    public string GetMsgType()
    {
        return "nav_msgs::msg::OccupancyGrid";
    }

    public void Delete()
    {
        header.Delete();
        info.Delete();
        ROSInterface.Free(data.ptr);
        data.ptr = IntPtr.Zero;
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
public struct nav_msgs_Odometry : IROSMsg
{
    public std_msgs_Header header; // std_msgs_Header
    public IntPtr child_frame_id; // IntPtr
    public geometry_msgs_PoseWithCovariance pose; // geometry_msgs_PoseWithCovariance
    public geometry_msgs_TwistWithCovariance twist; // geometry_msgs_TwistWithCovariance

    public string GetMsgType()
    {
        return "nav_msgs::msg::Odometry";
    }

    public void Delete()
    {
        header.Delete();
        ROSInterface.Free(child_frame_id);
        child_frame_id = IntPtr.Zero;
        pose.Delete();
        twist.Delete();
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