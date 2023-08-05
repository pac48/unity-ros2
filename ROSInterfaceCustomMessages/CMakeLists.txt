cmake_minimum_required(VERSION 3.22)
project(ROSInterfaceCustomMessages)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../ROSInterface/ros_generated_messages/ CACHE PATH "Installation directory" FORCE)

find_package(rosidl_default_generators REQUIRED)

set(message_packages
    intera_core_msgs
    )
foreach (message_package ${message_packages})
  add_subdirectory(${message_package})
endforeach ()