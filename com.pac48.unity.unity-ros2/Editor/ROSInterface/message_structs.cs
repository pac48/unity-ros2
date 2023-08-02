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
public struct std_msgs_UInt32 : IROSMsg
{
    public uint data; // uint

    public string GetMsgType()
    {
        return "std_msgs::msg::UInt32";
    }

    public static std_msgs_UInt32 Create()
    {
        var inst = new std_msgs_UInt32();

        return inst;
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

    public static geometry_msgs_Vector3 Create()
    {
        var inst = new geometry_msgs_Vector3();

        return inst;
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

    public static std_msgs_Int8 Create()
    {
        var inst = new std_msgs_Int8();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Float64 Create()
    {
        var inst = new std_msgs_Float64();

        return inst;
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

    public static std_msgs_ColorRGBA Create()
    {
        var inst = new std_msgs_ColorRGBA();

        return inst;
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

    public static sensor_msgs_ChannelFloat32 Create()
    {
        var inst = new sensor_msgs_ChannelFloat32();
        inst.name = Marshal.StringToHGlobalAnsi("\0");

        return inst;
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

    public static geometry_msgs_Quaternion Create()
    {
        var inst = new geometry_msgs_Quaternion();

        return inst;
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

    public static std_msgs_Byte Create()
    {
        var inst = new std_msgs_Byte();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Inertia Create()
    {
        var inst = new geometry_msgs_Inertia();
        inst.com = geometry_msgs_Vector3.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Transform Create()
    {
        var inst = new geometry_msgs_Transform();
        inst.translation = geometry_msgs_Vector3.Create();
        inst.rotation = geometry_msgs_Quaternion.Create();

        return inst;
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

    public static std_msgs_Bool Create()
    {
        var inst = new std_msgs_Bool();

        return inst;
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

    public static std_msgs_Char Create()
    {
        var inst = new std_msgs_Char();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Pose2D Create()
    {
        var inst = new geometry_msgs_Pose2D();

        return inst;
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

    public static builtin_interfaces_Duration Create()
    {
        var inst = new builtin_interfaces_Duration();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Accel Create()
    {
        var inst = new geometry_msgs_Accel();
        inst.linear = geometry_msgs_Vector3.Create();
        inst.angular = geometry_msgs_Vector3.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Float32 Create()
    {
        var inst = new std_msgs_Float32();

        return inst;
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

    public static std_msgs_Int64 Create()
    {
        var inst = new std_msgs_Int64();

        return inst;
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

    public static builtin_interfaces_Time Create()
    {
        var inst = new builtin_interfaces_Time();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Int32 Create()
    {
        var inst = new std_msgs_Int32();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_String Create()
    {
        var inst = new std_msgs_String();
        inst.data = Marshal.StringToHGlobalAnsi("\0");

        return inst;
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

    public static geometry_msgs_Twist Create()
    {
        var inst = new geometry_msgs_Twist();
        inst.linear = geometry_msgs_Vector3.Create();
        inst.angular = geometry_msgs_Vector3.Create();

        return inst;
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

    public static std_msgs_MultiArrayDimension Create()
    {
        var inst = new std_msgs_MultiArrayDimension();
        inst.label = Marshal.StringToHGlobalAnsi("\0");

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_MultiArrayLayout Create()
    {
        var inst = new std_msgs_MultiArrayLayout();

        return inst;
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

    public static std_msgs_UInt64MultiArray Create()
    {
        var inst = new std_msgs_UInt64MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
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

    public static std_msgs_UInt8 Create()
    {
        var inst = new std_msgs_UInt8();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Point Create()
    {
        var inst = new geometry_msgs_Point();

        return inst;
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

    public static std_msgs_UInt64 Create()
    {
        var inst = new std_msgs_UInt64();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Int8MultiArray Create()
    {
        var inst = new std_msgs_Int8MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_LaserEcho Create()
    {
        var inst = new sensor_msgs_LaserEcho();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_ByteMultiArray Create()
    {
        var inst = new std_msgs_ByteMultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_UInt16MultiArray Create()
    {
        var inst = new std_msgs_UInt16MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Int16MultiArray Create()
    {
        var inst = new std_msgs_Int16MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
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

    public static std_msgs_UInt16 Create()
    {
        var inst = new std_msgs_UInt16();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Header Create()
    {
        var inst = new std_msgs_Header();
        inst.stamp = builtin_interfaces_Time.Create();
        inst.frame_id = Marshal.StringToHGlobalAnsi("\0");

        return inst;
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
    public IntPtr name; // = nullptr // IntPtr
    public uint offset; // uint
    public byte datatype; // byte
    public uint count; // uint

    public string GetMsgType()
    {
        return "sensor_msgs::msg::PointField";
    }

    public static sensor_msgs_PointField Create()
    {
        var inst = new sensor_msgs_PointField();
        inst.name = Marshal.StringToHGlobalAnsi("\0");

        return inst;
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

    public static sensor_msgs_RegionOfInterest Create()
    {
        var inst = new sensor_msgs_RegionOfInterest();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_Imu Create()
    {
        var inst = new sensor_msgs_Imu();
        inst.header = std_msgs_Header.Create();
        inst.orientation = geometry_msgs_Quaternion.Create();
        inst.angular_velocity = geometry_msgs_Vector3.Create();
        inst.linear_acceleration = geometry_msgs_Vector3.Create();

        return inst;
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

    public static sensor_msgs_NavSatStatus Create()
    {
        var inst = new sensor_msgs_NavSatStatus();

        return inst;
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

    public static geometry_msgs_Point32 Create()
    {
        var inst = new geometry_msgs_Point32();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Pose Create()
    {
        var inst = new geometry_msgs_Pose();
        inst.position = geometry_msgs_Point.Create();
        inst.orientation = geometry_msgs_Quaternion.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_RelativeHumidity Create()
    {
        var inst = new sensor_msgs_RelativeHumidity();
        inst.header = std_msgs_Header.Create();

        return inst;
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

    public static geometry_msgs_AccelWithCovariance Create()
    {
        var inst = new geometry_msgs_AccelWithCovariance();
        inst.accel = geometry_msgs_Accel.Create();

        return inst;
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

    public static std_msgs_Empty Create()
    {
        var inst = new std_msgs_Empty();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_MagneticField Create()
    {
        var inst = new sensor_msgs_MagneticField();
        inst.header = std_msgs_Header.Create();
        inst.magnetic_field = geometry_msgs_Vector3.Create();

        return inst;
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

    public static std_msgs_Int16 Create()
    {
        var inst = new std_msgs_Int16();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_JoyFeedback Create()
    {
        var inst = new sensor_msgs_JoyFeedback();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Wrench Create()
    {
        var inst = new geometry_msgs_Wrench();
        inst.force = geometry_msgs_Vector3.Create();
        inst.torque = geometry_msgs_Vector3.Create();

        return inst;
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

    public static sensor_msgs_Joy Create()
    {
        var inst = new sensor_msgs_Joy();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PoseWithCovariance Create()
    {
        var inst = new geometry_msgs_PoseWithCovariance();
        inst.pose = geometry_msgs_Pose.Create();

        return inst;
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

    public static sensor_msgs_FluidPressure Create()
    {
        var inst = new sensor_msgs_FluidPressure();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PoseWithCovarianceStamped Create()
    {
        var inst = new geometry_msgs_PoseWithCovarianceStamped();
        inst.header = std_msgs_Header.Create();
        inst.pose = geometry_msgs_PoseWithCovariance.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_WrenchStamped Create()
    {
        var inst = new geometry_msgs_WrenchStamped();
        inst.header = std_msgs_Header.Create();
        inst.wrench = geometry_msgs_Wrench.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_Illuminance Create()
    {
        var inst = new sensor_msgs_Illuminance();
        inst.header = std_msgs_Header.Create();

        return inst;
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

    public static geometry_msgs_QuaternionStamped Create()
    {
        var inst = new geometry_msgs_QuaternionStamped();
        inst.header = std_msgs_Header.Create();
        inst.quaternion = geometry_msgs_Quaternion.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Int64MultiArray Create()
    {
        var inst = new std_msgs_Int64MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_Range Create()
    {
        var inst = new sensor_msgs_Range();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Float32MultiArray Create()
    {
        var inst = new std_msgs_Float32MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_TwistWithCovariance Create()
    {
        var inst = new geometry_msgs_TwistWithCovariance();
        inst.twist = geometry_msgs_Twist.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_TransformStamped Create()
    {
        var inst = new geometry_msgs_TransformStamped();
        inst.header = std_msgs_Header.Create();
        inst.child_frame_id = Marshal.StringToHGlobalAnsi("\0");
        inst.transform = geometry_msgs_Transform.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Float64MultiArray Create()
    {
        var inst = new std_msgs_Float64MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_PointCloud Create()
    {
        var inst = new sensor_msgs_PointCloud();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_TwistStamped Create()
    {
        var inst = new geometry_msgs_TwistStamped();
        inst.header = std_msgs_Header.Create();
        inst.twist = geometry_msgs_Twist.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_Image Create()
    {
        var inst = new sensor_msgs_Image();
        inst.header = std_msgs_Header.Create();
        inst.encoding = Marshal.StringToHGlobalAnsi("\0");

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PoseArray Create()
    {
        var inst = new geometry_msgs_PoseArray();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_MultiDOFJointState Create()
    {
        var inst = new sensor_msgs_MultiDOFJointState();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_MultiEchoLaserScan Create()
    {
        var inst = new sensor_msgs_MultiEchoLaserScan();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_AccelStamped Create()
    {
        var inst = new geometry_msgs_AccelStamped();
        inst.header = std_msgs_Header.Create();
        inst.accel = geometry_msgs_Accel.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_BatteryState Create()
    {
        var inst = new sensor_msgs_BatteryState();
        inst.header = std_msgs_Header.Create();
        inst.location = Marshal.StringToHGlobalAnsi("\0");
        inst.serial_number = Marshal.StringToHGlobalAnsi("\0");

        return inst;
    }

    public void Delete()
    {
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

    public static nav_msgs_GridCells Create()
    {
        var inst = new nav_msgs_GridCells();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_UInt32MultiArray Create()
    {
        var inst = new std_msgs_UInt32MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_InertiaStamped Create()
    {
        var inst = new geometry_msgs_InertiaStamped();
        inst.header = std_msgs_Header.Create();
        inst.inertia = geometry_msgs_Inertia.Create();

        return inst;
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

    public static sensor_msgs_JointState Create()
    {
        var inst = new sensor_msgs_JointState();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_Temperature Create()
    {
        var inst = new sensor_msgs_Temperature();
        inst.header = std_msgs_Header.Create();

        return inst;
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

    public static sensor_msgs_NavSatFix Create()
    {
        var inst = new sensor_msgs_NavSatFix();
        inst.header = std_msgs_Header.Create();
        inst.status = sensor_msgs_NavSatStatus.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_Int32MultiArray Create()
    {
        var inst = new std_msgs_Int32MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_LaserScan Create()
    {
        var inst = new sensor_msgs_LaserScan();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static nav_msgs_MapMetaData Create()
    {
        var inst = new nav_msgs_MapMetaData();
        inst.map_load_time = builtin_interfaces_Time.Create();
        inst.origin = geometry_msgs_Pose.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_CompressedImage Create()
    {
        var inst = new sensor_msgs_CompressedImage();
        inst.header = std_msgs_Header.Create();
        inst.format = Marshal.StringToHGlobalAnsi("\0");

        return inst;
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

    public static sensor_msgs_JoyFeedbackArray Create()
    {
        var inst = new sensor_msgs_JoyFeedbackArray();

        return inst;
    }

    public void Delete()
    {
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

    public static std_msgs_UInt8MultiArray Create()
    {
        var inst = new std_msgs_UInt8MultiArray();
        inst.layout = std_msgs_MultiArrayLayout.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_PointCloud2 Create()
    {
        var inst = new sensor_msgs_PointCloud2();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PointStamped Create()
    {
        var inst = new geometry_msgs_PointStamped();
        inst.header = std_msgs_Header.Create();
        inst.point = geometry_msgs_Point.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_TimeReference Create()
    {
        var inst = new sensor_msgs_TimeReference();
        inst.header = std_msgs_Header.Create();
        inst.time_ref = builtin_interfaces_Time.Create();
        inst.source = Marshal.StringToHGlobalAnsi("\0");

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_AccelWithCovarianceStamped Create()
    {
        var inst = new geometry_msgs_AccelWithCovarianceStamped();
        inst.header = std_msgs_Header.Create();
        inst.accel = geometry_msgs_AccelWithCovariance.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Vector3Stamped Create()
    {
        var inst = new geometry_msgs_Vector3Stamped();
        inst.header = std_msgs_Header.Create();
        inst.vector = geometry_msgs_Vector3.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static sensor_msgs_CameraInfo Create()
    {
        var inst = new sensor_msgs_CameraInfo();
        inst.header = std_msgs_Header.Create();
        inst.distortion_model = Marshal.StringToHGlobalAnsi("\0");
        inst.roi = sensor_msgs_RegionOfInterest.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_Polygon Create()
    {
        var inst = new geometry_msgs_Polygon();

        return inst;
    }

    public void Delete()
    {
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

    public static nav_msgs_Odometry Create()
    {
        var inst = new nav_msgs_Odometry();
        inst.header = std_msgs_Header.Create();
        inst.child_frame_id = Marshal.StringToHGlobalAnsi("\0");
        inst.pose = geometry_msgs_PoseWithCovariance.Create();
        inst.twist = geometry_msgs_TwistWithCovariance.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_TwistWithCovarianceStamped Create()
    {
        var inst = new geometry_msgs_TwistWithCovarianceStamped();
        inst.header = std_msgs_Header.Create();
        inst.twist = geometry_msgs_TwistWithCovariance.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PoseStamped Create()
    {
        var inst = new geometry_msgs_PoseStamped();
        inst.header = std_msgs_Header.Create();
        inst.pose = geometry_msgs_Pose.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static nav_msgs_OccupancyGrid Create()
    {
        var inst = new nav_msgs_OccupancyGrid();
        inst.header = std_msgs_Header.Create();
        inst.info = nav_msgs_MapMetaData.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static nav_msgs_Path Create()
    {
        var inst = new nav_msgs_Path();
        inst.header = std_msgs_Header.Create();

        return inst;
    }

    public void Delete()
    {
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

    public static geometry_msgs_PolygonStamped Create()
    {
        var inst = new geometry_msgs_PolygonStamped();
        inst.header = std_msgs_Header.Create();
        inst.polygon = geometry_msgs_Polygon.Create();

        return inst;
    }

    public void Delete()
    {
    }
}