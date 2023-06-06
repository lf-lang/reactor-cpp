#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <rclcpp/rclcpp.hpp>

extern rclcpp::Node* lf_node;

namespace reactor {

template <class T>
class ROS2PubEndpoint : UpstreamEndpoint<T> {

};

template <class T>
class ROS2SubEndpoint : DownstreamEndpoint<T> {

};

} // reactor ns

#endif