#pragma once
#include "memory"
#include "rclcpp/node.hpp"

class BaseSubscriber {
public:
    virtual void receive(void *output) {};
};

class BasePublisher {
public:
    virtual void publish(void *input) {};
};

std::shared_ptr<BasePublisher>
createPublisher(const std::string &type, const std::string &topic, rclcpp::Node::SharedPtr node);

std::shared_ptr<BaseSubscriber>
createSubscriber(const std::string &type, const std::string &topic, rclcpp::Node::SharedPtr node);