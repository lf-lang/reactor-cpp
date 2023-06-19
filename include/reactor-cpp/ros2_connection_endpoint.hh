#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

extern rclcpp::Node* lf_node;

namespace reactor {

template <class T>
class ROS2PubEndpoint : public UpstreamEndpoint<T> {
    private:
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;

    protected:
        PortCallback set_cb() override{
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<T>&>(port); 
                if constexpr (std::is_same<T, void>::value) {
                    std_msgs::msg::Empty msg;
                    pub_->publish(msg);
                } else {
                    // send std::move(typed_port.get());
                }
            };
        }

    public:
        ROS2PubEndpoint() : UpstreamEndpoint<T>() { 
            //if constexpr (std::is_same<T, void>::value)
                pub_ = lf_node->create_publisher<std_msgs::msg::Empty>("test", 10);
            // else 
            //     pub_ = lf_node->create_publisher<T>("test", 10);
        }

};

template <class T>
class ROS2SubEndpoint : public DownstreamEndpoint<T> {
    private:
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;

    protected:
        void schedule_this(std::shared_ptr<std_msgs::msg::Empty> p) {
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
            std::function<void(std::shared_ptr<std_msgs::msg::Empty>)> f = 
                std::bind(&ROS2SubEndpoint::schedule_this, this, std::placeholders::_1);
            sub_ = lf_node->create_subscription<std_msgs::msg::Empty>("test", 10,
                        f);
        }
};

} // reactor ns

#endif