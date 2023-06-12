#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.h>

extern rclcpp::Node* lf_node;

namespace reactor {

template <class T>
class ROS2PubEndpoint : UpstreamEndpoint<T> {
    private:
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_

    protected:
        PortCallback set_cb() override{
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<T>&>(port); 
                if constexpr (std::is_same<T, void>::value) {
                    pub_->publish();
                } else {
                    // send std::move(typed_port.get());
                }
            }
        }

    public:
        template <class T>
        ROS2PubEndpoint() : UpstreamEndpoint<T>{ 
            pub_ = lf_node->createPublisher("test", 10);
        }

};

template <class T>
class ROS2SubEndpoint : DownstreamEndpoint<T> {
    private:
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;

        void schedule_this(/*value here*/) override {
            if constexpr (std::is_same<T, void>::value) {
                this->schedule();
            } else {
                //this->schedule(std::move(value here));
            }
        }

    public:
        ROS2SubEndpoint(const std::string& name, Environment* environment, bool is_logical, Duration min_delay) 
        : DownstreamEndpoint<T>(name, environment, is_logical, min_delay){
            while (!lf_node->count_publishers("test"));
            sub_ = lf_node->create_subscription<std_msgs::msg::Empty>("test", 10,
                        std::bind(&schedule_this, this));
        }
};

} // reactor ns

#endif