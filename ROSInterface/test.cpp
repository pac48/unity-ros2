#include "interface.h"
#include "generated/tmp.h"
#include "rclcpp/rclcpp.hpp"

int main() {
    auto name = "test";
    rclcpp::init(1, &name);
    intera_core_msgs_JointCommand msg;
    auto node = std::make_shared<rclcpp::Node>("test");
    node->create_publisher<intera_core_msgs::msg::JointCommand>("joint_command", 10);


    sensor_msgs_Image input;
     std::intptr_t handle = NULL;
    Publish(handle, "sensor_msgs/msg/Image", "test_topic", (void *) &input);
    return 0;
}