cmake_minimum_required(VERSION 3.22)
project(ROSInterface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../com.pac48.unity.unity-ros2/Runtime/Plugins/ CACHE PATH "Installation directory" FORCE)
set(BUILD_SHARED_LIBS ON CACHE PATH "Build shared libraries" FORCE)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(ROSInterface SHARED src/interface.cpp)
add_library(ROSInterfaceGenerated SHARED
    src/generated/sub_pub_interface.cpp
    src/generated/builtin_interfaces_Duration.cpp
    src/generated/builtin_interfaces_Time.cpp
    src/generated/geometry_msgs_Accel.cpp
    src/generated/geometry_msgs_AccelStamped.cpp
    src/generated/geometry_msgs_AccelWithCovariance.cpp
    src/generated/geometry_msgs_AccelWithCovarianceStamped.cpp
    src/generated/geometry_msgs_Inertia.cpp
    src/generated/geometry_msgs_InertiaStamped.cpp
    src/generated/geometry_msgs_Point32.cpp
    src/generated/geometry_msgs_Point.cpp
    src/generated/geometry_msgs_PointStamped.cpp
    src/generated/geometry_msgs_Polygon.cpp
    src/generated/geometry_msgs_PolygonStamped.cpp
    src/generated/geometry_msgs_Pose2D.cpp
    src/generated/geometry_msgs_PoseArray.cpp
    src/generated/geometry_msgs_Pose.cpp
    src/generated/geometry_msgs_PoseStamped.cpp
    src/generated/geometry_msgs_PoseWithCovariance.cpp
    src/generated/geometry_msgs_PoseWithCovarianceStamped.cpp
    src/generated/geometry_msgs_Quaternion.cpp
    src/generated/geometry_msgs_QuaternionStamped.cpp
    src/generated/geometry_msgs_Transform.cpp
    src/generated/geometry_msgs_TransformStamped.cpp
    src/generated/geometry_msgs_Twist.cpp
    src/generated/geometry_msgs_TwistStamped.cpp
    src/generated/geometry_msgs_TwistWithCovariance.cpp
    src/generated/geometry_msgs_TwistWithCovarianceStamped.cpp
    src/generated/geometry_msgs_Vector3.cpp
    src/generated/geometry_msgs_Vector3Stamped.cpp
    src/generated/geometry_msgs_Wrench.cpp
    src/generated/geometry_msgs_WrenchStamped.cpp
    src/generated/nav_msgs_GridCells.cpp
    src/generated/nav_msgs_MapMetaData.cpp
    src/generated/nav_msgs_OccupancyGrid.cpp
    src/generated/nav_msgs_Odometry.cpp
    src/generated/nav_msgs_Path.cpp
    src/generated/sensor_msgs_BatteryState.cpp
    src/generated/sensor_msgs_CameraInfo.cpp
    src/generated/sensor_msgs_ChannelFloat32.cpp
    src/generated/sensor_msgs_CompressedImage.cpp
    src/generated/sensor_msgs_FluidPressure.cpp
    src/generated/sensor_msgs_Illuminance.cpp
    src/generated/sensor_msgs_Image.cpp
    src/generated/sensor_msgs_Imu.cpp
    src/generated/sensor_msgs_JointState.cpp
    src/generated/sensor_msgs_Joy.cpp
    src/generated/sensor_msgs_JoyFeedbackArray.cpp
    src/generated/sensor_msgs_JoyFeedback.cpp
    src/generated/sensor_msgs_LaserEcho.cpp
    src/generated/sensor_msgs_LaserScan.cpp
    src/generated/sensor_msgs_MagneticField.cpp
    src/generated/sensor_msgs_MultiDOFJointState.cpp
    src/generated/sensor_msgs_MultiEchoLaserScan.cpp
    src/generated/sensor_msgs_NavSatFix.cpp
    src/generated/sensor_msgs_NavSatStatus.cpp
    src/generated/sensor_msgs_PointCloud2.cpp
    src/generated/sensor_msgs_PointCloud.cpp
    src/generated/sensor_msgs_PointField.cpp
    src/generated/sensor_msgs_Range.cpp
    src/generated/sensor_msgs_RegionOfInterest.cpp
    src/generated/sensor_msgs_RelativeHumidity.cpp
    src/generated/sensor_msgs_Temperature.cpp
    src/generated/sensor_msgs_TimeReference.cpp
    src/generated/std_msgs_Bool.cpp
    src/generated/std_msgs_Byte.cpp
    src/generated/std_msgs_ByteMultiArray.cpp
    src/generated/std_msgs_Char.cpp
    src/generated/std_msgs_ColorRGBA.cpp
    src/generated/std_msgs_Empty.cpp
    src/generated/std_msgs_Float32.cpp
    src/generated/std_msgs_Float32MultiArray.cpp
    src/generated/std_msgs_Float64.cpp
    src/generated/std_msgs_Float64MultiArray.cpp
    src/generated/std_msgs_Header.cpp
    src/generated/std_msgs_Int16.cpp
    src/generated/std_msgs_Int16MultiArray.cpp
    src/generated/std_msgs_Int32.cpp
    src/generated/std_msgs_Int32MultiArray.cpp
    src/generated/std_msgs_Int64.cpp
    src/generated/std_msgs_Int64MultiArray.cpp
    src/generated/std_msgs_Int8.cpp
    src/generated/std_msgs_Int8MultiArray.cpp
    src/generated/std_msgs_MultiArrayDimension.cpp
    src/generated/std_msgs_MultiArrayLayout.cpp
    src/generated/std_msgs_String.cpp
    src/generated/std_msgs_UInt16.cpp
    src/generated/std_msgs_UInt16MultiArray.cpp
    src/generated/std_msgs_UInt32.cpp
    src/generated/std_msgs_UInt32MultiArray.cpp
    src/generated/std_msgs_UInt64.cpp
    src/generated/std_msgs_UInt64MultiArray.cpp
    src/generated/std_msgs_UInt8.cpp
    src/generated/std_msgs_UInt8MultiArray.cpp
    )
target_include_directories(ROSInterfaceGenerated PUBLIC include)
target_include_directories(ROSInterfaceGenerated PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
target_include_directories(ROSInterfaceGenerated PUBLIC ${builtin_interfaces_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${libstatistics_collector_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcl_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcl_interfaces_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcl_logging_interface_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcl_logging_spdlog_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcl_yaml_param_parser_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rclcpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcpputils_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rcutils_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rmw_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rmw_dds_common_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rmw_fastrtps_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rmw_fastrtps_shared_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rmw_implementation_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosgraph_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_default_runtime_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_generator_c_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_runtime_c_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_runtime_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_fastrtps_c_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_fastrtps_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_interface_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_introspection_c_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${statistics_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${std_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${sensor_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${geometry_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${nav_msgs_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${tf2_ros_INCLUDE_DIRS})
target_include_directories(ROSInterfaceGenerated PUBLIC ${tracetools_INCLUDE_DIRS})




target_link_libraries(ROSInterfaceGenerated PUBLIC ${builtin_interfaces_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${libstatistics_collector_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcl_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcl_interfaces_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcl_logging_interface_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcl_logging_spdlog_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcl_yaml_param_parser_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rclcpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcpputils_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rcutils_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rmw_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rmw_dds_common_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rmw_fastrtps_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rmw_fastrtps_shared_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rmw_implementation_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosgraph_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_default_runtime_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_generator_c_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_runtime_c_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_runtime_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_fastrtps_c_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_fastrtps_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_interface_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_introspection_c_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${rosidl_typesupport_introspection_cpp_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${statistics_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${std_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${sensor_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${geometry_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${nav_msgs_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${tf2_ros_LIBRARIES})
target_link_libraries(ROSInterfaceGenerated PUBLIC ${tracetools_LIBRARIES})

target_link_libraries(ROSInterface PUBLIC ROSInterfaceGenerated)

target_compile_options(ROSInterface PUBLIC -g)

# install targets
set_target_properties(ROSInterfaceGenerated PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS ROSInterfaceGenerated
    LIBRARY DESTINATION lib
    )
set_target_properties(ROSInterface PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS ROSInterface
        LIBRARY DESTINATION lib
        )

add_executable(ROSInterfaceTest test.cpp)
target_include_directories(ROSInterfaceTest PUBLIC include)
target_link_libraries(ROSInterfaceTest ROSInterface)
