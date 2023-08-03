#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "interface.h"

#include "sub_pub_interface.h"


using namespace std::chrono_literals;
using std::placeholders::_1;


class ROSInterface {
public:

    ROSInterface(UnityAllocate unity_allocator) {
        auto name = "ros_node";
        if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
            rclcpp::init(1, &name);
        }
        node = std::make_shared<rclcpp::Node>("unity_ros_interface_node");
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(node);
        allocator = unity_allocator;
        spin_thread_ = std::make_shared<std::thread>([this]() {
            while (true) {
                executor_->spin_some();
                const std::lock_guard<std::mutex> lock(node_mtx_);
                if (shutdown_) {
                    break;
                }
            }
        });
    }

    ~ROSInterface() {
        rclcpp::shutdown();
        {
            const std::lock_guard<std::mutex> lock(node_mtx_);
            shutdown_ = true;
        }
        spin_thread_->join();
    }

    void Publish(const std::string &type, const std::string &topic, void *input) {
        auto key = type + topic;
        if (publisher_map_.find(key) == publisher_map_.end()) {
            const std::lock_guard<std::mutex> lock(node_mtx_);
            publisher_map_[key] = createROSPublisher(type, topic, node);
        }
        publisher_map_[key]->publish(input);
    }

    void Receive(const std::string &type, const std::string &topic, void **output) {
        auto key = type + topic;
        if (subscriber_map_.find(key) == subscriber_map_.end()) {
            const std::lock_guard<std::mutex> lock(node_mtx_);
            subscriber_map_[key] = createROSSubscriber(type, topic, node);
        }
        subscriber_map_[key]->receive(output, allocator);

    }

    void SendTransform(geometry_msgs::msg::TransformStamped& t){
        t.header.stamp = node->get_clock()->now();
        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<rclcpp::Node> node;
    UnityAllocate allocator;
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::unordered_map<std::string, std::shared_ptr<BasePublisher>> publisher_map_;
    std::unordered_map<std::string, std::shared_ptr<BaseSubscriber>> subscriber_map_;
    std::shared_ptr<std::thread> spin_thread_;
    std::mutex node_mtx_;
    bool shutdown_ = false;

};

std::intptr_t Init(UnityAllocate allocator) {
    return (std::intptr_t) new ROSInterface(allocator);
}

void Destroy(std::intptr_t handle) {
    auto ptr = (ROSInterface *) handle;
    if (handle != 0) {
        delete ptr;
    }
}

void Publish(std::intptr_t handle, char *type, char *topic, void *input) {
    if (handle != 0) {
        auto ptr = (ROSInterface *) handle;
        ptr->Publish(type, topic, input);
    }
}

void Receive(std::intptr_t handle, char *type, char *topic, void **output) {
    if (handle != 0) {
        auto ptr = (ROSInterface *) handle;
        ptr->Receive(type, topic, output);
    }
}

void SendTransform(std::intptr_t handle, void* tf_msg){
    if (handle != 0 && tf_msg) {
        auto ptr = (ROSInterface *) handle;
        auto tf_struct = (geometry_msgs_TransformStamped*) tf_msg;
        auto t = (geometry_msgs::msg::TransformStamped) *tf_struct;
        ptr->SendTransform(t);
    }
}
void SetROSTime(std::intptr_t handle, void* stamp_msg){
    if (handle != 0 && stamp_msg) {
        auto ptr = (ROSInterface *) handle;
        auto stamp_struct = (builtin_interfaces_Time*) stamp_msg;
        builtin_interfaces::msg::Time ros_stamp_msg = ptr->node->get_clock()->now();
        stamp_struct->sec = ros_stamp_msg.sec;
        stamp_struct->nanosec = ros_stamp_msg.nanosec;
    }
}