#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interface.h"

#include "sub_pub_interface.h"


using namespace std::chrono_literals;
using std::placeholders::_1;


class ROSInterface {
public:

    ROSInterface() {
        auto name = "ros_node";
        rclcpp::init(1, &name);
        node_ = std::make_shared<rclcpp::Node>("unity_ros_interface_node");
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(node_);
        auto spin_thread_ = std::make_shared<std::thread>([this]() {
            const std::lock_guard<std::mutex> lock(node_mtx);
            executor_->spin_some();
        });
    }

    ~ROSInterface() {
        rclcpp::shutdown();
    }

    void Publish(const std::string &type, const std::string &topic, void *input) {
        auto key = type + topic;
        if (publisher_map.find(key) == publisher_map.end()) {
            const std::lock_guard<std::mutex> lock(node_mtx);
            publisher_map[key] = createROSPublisher(type, topic, node_);
        }
        publisher_map[key]->publish(input);
    }

    void Receive(const std::string &type, const std::string &topic, void *output) {
        auto key = type + topic;
        if (subscriber_map.find(key) == subscriber_map.end()) {
            const std::lock_guard<std::mutex> lock(node_mtx);
            subscriber_map[key] = createROSSubscriber(type, topic, node_);
        }
        subscriber_map[key]->receive(output);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::unordered_map<std::string, std::shared_ptr<BasePublisher>> publisher_map;
    std::unordered_map<std::string, std::shared_ptr<BaseSubscriber>> subscriber_map;
    std::mutex node_mtx;

};

std::intptr_t Init() {
    return (std::intptr_t) new ROSInterface();
}

void Destroy(std::intptr_t handle) {
    auto ptr = (ROSInterface *) handle;
    delete ptr;
}

void Publish(std::intptr_t handle, char *type, char *topic, void *input) {
    if (handle != 0) {
        auto ptr = (ROSInterface *) handle;
        ptr->Publish(type, topic, input);
    }
}

void Receive(std::intptr_t handle, char *type, char *topic, void *output) {
    if (handle != 0) {
        auto ptr = (ROSInterface *) handle;
        ptr->Receive(type, topic, output);
    }
}

