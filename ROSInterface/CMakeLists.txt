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
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB generated_sources
    "src/generated/*.cpp"
    )

add_library(ROSInterface SHARED src/interface.cpp)
target_link_libraries(ROSInterface ${intera_core_msgs__rosidl_generator_cpp})
add_library(ROSInterfaceGenerated SHARED ${generated_sources})


set(message_packages
    intera_core_msgs
    )
set(target_list ROSInterfaceGenerated)

foreach (target IN LISTS target_list)
  target_include_directories(${target} PUBLIC include)
  target_include_directories(${target} PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
  target_include_directories(${target} PUBLIC ${builtin_interfaces_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${libstatistics_collector_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcl_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcl_interfaces_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcl_logging_interface_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcl_logging_spdlog_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcl_yaml_param_parser_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rclcpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcpputils_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rcutils_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rmw_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rmw_dds_common_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rmw_fastrtps_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rmw_fastrtps_shared_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rmw_implementation_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosgraph_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_default_runtime_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_generator_c_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_runtime_c_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_runtime_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_fastrtps_c_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_fastrtps_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_interface_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_introspection_c_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${statistics_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${std_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${sensor_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${geometry_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${nav_msgs_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${tf2_ros_INCLUDE_DIRS})
  target_include_directories(${target} PUBLIC ${tracetools_INCLUDE_DIRS})

  # add all generated messages
  foreach (message_package ${message_packages})
    target_include_directories(${target} PUBLIC ros_generated_messages/include/${message_package})
  endforeach ()

  # link all rclcpp libs needed
  target_link_libraries(${target} PUBLIC ${builtin_interfaces_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${libstatistics_collector_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcl_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcl_interfaces_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcl_logging_interface_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcl_logging_spdlog_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcl_yaml_param_parser_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rclcpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcpputils_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rcutils_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rmw_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rmw_dds_common_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rmw_fastrtps_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rmw_fastrtps_shared_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rmw_implementation_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosgraph_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_default_runtime_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_generator_c_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_runtime_c_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_runtime_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_fastrtps_c_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_fastrtps_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_interface_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_introspection_c_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${rosidl_typesupport_introspection_cpp_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${statistics_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${std_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${sensor_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${geometry_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${nav_msgs_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${tf2_ros_LIBRARIES})
  target_link_libraries(${target} PUBLIC ${tracetools_LIBRARIES})
endforeach ()

# install targets
#target_compile_options(ROSInterfaceGenerated PUBLIC -g -O3)
target_link_libraries(ROSInterface PUBLIC ROSInterfaceGenerated)
target_compile_options(ROSInterface PUBLIC -g -O0)
set_target_properties(ROSInterfaceGenerated PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS ROSInterfaceGenerated
    LIBRARY DESTINATION lib
    )

foreach (message_package ${message_packages})
  set(msgs_BINARY_DIR ${CMAKE_SOURCE_DIR}/ros_generated_messages/lib/)
  set(type_support_libs
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_generator_c.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_c.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_cpp.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_fastrtps_c.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_fastrtps_cpp.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_introspection_c.so
      ${msgs_BINARY_DIR}/lib${message_package}__rosidl_typesupport_introspection_cpp.so
      )

  foreach (type_support_lib ${type_support_libs})
    set_property(SOURCE ${type_support_lib} PROPERTY GENERATED 1)
    target_link_libraries(ROSInterface PUBLIC ${type_support_lib})
  endforeach ()

  install(FILES ${type_support_libs}
      DESTINATION lib
      )

endforeach ()

set_target_properties(ROSInterface PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS ROSInterface
    LIBRARY DESTINATION lib
    )


add_executable(ROSInterfaceTest test.cpp)
target_include_directories(ROSInterfaceTest PUBLIC include)
target_link_libraries(ROSInterfaceTest ROSInterface)
