cmake_minimum_required(VERSION 3.5)
project(intera_core_msgs)

# find dependencies
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/JointCommand.msg
    )

rosidl_generate_interfaces(intera_core_msgs
    ${msg_files}
    DEPENDENCIES
    action_msgs
    std_msgs
    builtin_interfaces
    )
