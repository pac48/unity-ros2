#pragma once

#include "memory"
#include <rclcpp/rclcpp.hpp>
#include "generated/tmp.h"

class BaseSubscriber {
public:
    virtual void receive(void *output) {};
};

class BasePublisher {
public:
    virtual void publish(void *input) {};
};

std::shared_ptr<BasePublisher>
createROSPublisher(const std::string &type, const std::string &topic, rclcpp::Node::SharedPtr node);

std::shared_ptr<BaseSubscriber>
createROSSubscriber(const std::string &type, const std::string &topic, rclcpp::Node::SharedPtr node);
