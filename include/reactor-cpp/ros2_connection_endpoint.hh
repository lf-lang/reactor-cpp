#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "time_barrier.hh"
#include "lf_msgs_ros/msg/tag.hpp"

extern rclcpp::Node* lf_node;

#define TAG_RELEASE_SUFFIX "/released_tags"
#define EMPTY_EVENT_SUFFIX "/request_empty_event"

namespace reactor {
/*
    UserType corredponds to a ROS msg type
    WrappedType is a generated message which includes the UserType message and a Tag
*/
template <class UserType, class WrappedType>
class ROS2PubEndpoint : public UpstreamEndpoint<UserType, WrappedType> {

    static_assert(std::is_same<decltype(WrappedType::tag), lf_msgs_ros::msg::Tag>::value);
    static_assert(std::is_same<decltype(WrappedType::message), UserType>::value);

    private:
        std::shared_ptr<rclcpp::Publisher<WrappedType>> pub_;
        std::shared_ptr<rclcpp::Subscription<lf_msgs_ros::msg::Tag>> empty_event_sub_;
        std::shared_ptr<rclcpp::Publisher<lf_msgs_ros::msg::Tag>> tag_only_pub_;

    protected:
        PortCallback set_cb() override{
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<UserType>&>(port); 
                const auto* scheduler = port.environment()->scheduler();
                auto tag = Tag::from_logical_time(scheduler->logical_time());
                WrappedType wrapped_msg;
                wrapped_msg.tag.time_point = tag.time_point().time_since_epoch().count();
                wrapped_msg.tag.microstep = tag.micro_step();
                wrapped_msg.message = *typed_port.get();
                pub_->publish(std::move(wrapped_msg));
            };
        }

        virtual void create_publisher_tag_only(const std::string& topic_name){
            tag_only_pub_ = lf_node->create_publisher<lf_msgs_ros::msg::Tag>(topic_name + TAG_RELEASE_SUFFIX, 10);
        }

        virtual void sub_empty_event_request(const std::string& topic_name) {
            empty_event_sub_ = lf_node->create_subscription<lf_msgs_ros::msg::Tag>(topic_name + EMPTY_EVENT_SUFFIX, 10,  
                [&](const lf_msgs_ros::msg::Tag::SharedPtr msg){
                    // TODO: should we unlock something here? federates are single threaded rn anyway, but maybe future 
                    // Insert an empty event into the upstream event queue. This ensures that we
                    // will get notified and woken up as soon as the tag becomes safe to process.
                    // It is important to unlock the mutex here. Otherwise we could enter a deadlock as
                    // scheduling the upstream event also requires holding the upstream mutex.
                    //lock.unlock();
                    bool result = this->port_->environment()->scheduler()->schedule_empty_async_at(reactor::Tag(reactor::TimePoint(std::chrono::nanoseconds(msg->time_point)), mstep_t(msg->microstep)));
                    //lock.lock();

                    // If inserting the empty event was not successful, then this is because the upstream
                    // scheduler already processes a later event. In this case, it is safe to assume that
                    // the tag is acquired.
                    if (!result) {
                        tag_only_pub_->publish(*msg.get());
                    }
                });
        }


    public:
        ROS2PubEndpoint(const std::string& topic_name) : UpstreamEndpoint<UserType, WrappedType>() { 
            pub_ = lf_node->create_publisher<WrappedType>(topic_name, 10);
            create_publisher_tag_only(topic_name);
            sub_empty_event_request(topic_name);
        }

        void set_port(Port<UserType>* port) override {
            this->UpstreamEndpoint<UserType, WrappedType>::set_port(port);
            port->environment()->scheduler()->register_release_tag_callback([this](const LogicalTime& tag) {
                lf_msgs_ros::msg::Tag t;
                t.microstep = tag.micro_step();
                t.time_point = tag.time_point().time_since_epoch().count();
                tag_only_pub_->publish(std::move(t));
            });
        }
};

template <class UserType, class WrappedType>
class ROS2SubEndpoint : public DownstreamEndpoint<UserType, WrappedType> {
    static_assert(std::is_same<decltype(WrappedType::tag), lf_msgs_ros::msg::Tag>::value);
    static_assert(std::is_same<decltype(WrappedType::message), UserType>::value);

    private:
        std::shared_ptr<rclcpp::Subscription<WrappedType>> sub_;
        rclcpp::Subscription<lf_msgs_ros::msg::Tag>::SharedPtr tag_only_sub_;
        rclcpp::Publisher<lf_msgs_ros::msg::Tag>::SharedPtr empty_event_pub_;
        LogicalTimeBarrier logical_time_barrier_;

    protected:
        void schedule_this(std::shared_ptr<WrappedType> p) {
            // unwrapping the message using an aliasing constructor
            std::shared_ptr<UserType> inner_ptr(p, &p->message);
            if constexpr(detail::is_trivial<UserType>())
                this->schedule(ImmutableValuePtr<UserType>(*inner_ptr.get()));
            else 
                this->schedule(ImmutableValuePtr<UserType>(std::move(inner_ptr)));
        }

        virtual bool acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, std::condition_variable& cv,
                           const std::function<bool(void)>& abort_waiting) override {
            if (logical_time_barrier_.try_acquire_tag(tag)) {
                return true;
            }
            lf_msgs_ros::msg::Tag t;
            t.microstep = tag.micro_step();
            t.time_point = tag.time_point().time_since_epoch().count();
            empty_event_pub_->publish(std::move(t));

            return logical_time_barrier_.acquire_tag(tag, lock, cv, abort_waiting);
        }

        virtual void sub_to_tag(const std::string& topic_name) {
            tag_only_sub_ = lf_node->create_subscription<lf_msgs_ros::msg::Tag>(topic_name + TAG_RELEASE_SUFFIX, 10,
            [&](const lf_msgs_ros::msg::Tag::SharedPtr msg){
                LogicalTime t;
                t.advance_to(reactor::Tag(reactor::TimePoint(std::chrono::nanoseconds(msg->time_point)), mstep_t(msg->microstep)));
                logical_time_barrier_.release_tag(t);
                this->environment()->scheduler()->notify();
            });
        }

        virtual void pub_empty_event_topic(const std::string& topic_name) {
            empty_event_pub_ = lf_node->create_publisher<lf_msgs_ros::msg::Tag>(topic_name + EMPTY_EVENT_SUFFIX, 10);
        }

    public:
        ROS2SubEndpoint(const std::string& topic_name, const std::string& name, Environment* environment, bool is_logical, Duration min_delay) 
        : DownstreamEndpoint<UserType, WrappedType>(name, environment, is_logical, min_delay){
            while (!lf_node->count_publishers(topic_name));

            
            std::function<void(std::shared_ptr<WrappedType>)> f = 
                std::bind(&ROS2SubEndpoint::schedule_this, this, std::placeholders::_1);
            sub_ = lf_node->create_subscription<WrappedType>(topic_name, 10,
                        f);

            pub_empty_event_topic(topic_name);
            sub_to_tag(topic_name);
        }

       
};

template <class UserType, class WrappedType>
class ROS2PubEndpointPhysical : public ROS2PubEndpoint<UserType, WrappedType> {
    private:
    protected:
        
        virtual void create_publisher_tag_only(const std::string& topic_name) override {
            // not necessary
        }

        virtual void sub_empty_event_request(const std::string& topic_name) override {
            // not necessary
        }

    public:
        ROS2PubEndpointPhysical(const std::string& topic_name) : ROS2PubEndpoint<UserType, WrappedType>(topic_name) { 
        }

        void set_port(Port<UserType>* port) override {
            this->UpstreamEndpoint<UserType, WrappedType>::set_port(port);
            // no registering of tag release callback necessary
        }
        

};

template <class UserType, class WrappedType>
class ROS2SubEndpointPhysical : public ROS2SubEndpoint<UserType, WrappedType> {
    private:
    protected:
        virtual void sub_to_tag(const std::string& topic_name) override {
            // no tag subscription necessary
        }

        virtual void pub_empty_event_topic(const std::string& topic_name) {
            // not necessary
        }

        inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, std::condition_variable& cv,
                          const std::function<bool(void)>& abort_waiting) -> bool override {
            return PhysicalTimeBarrier::acquire_tag(tag, lock, cv, abort_waiting);
        }

    public:
        ROS2SubEndpointPhysical(const std::string& topic_name, const std::string& name, Environment* environment, bool is_logical, Duration min_delay) 
        : ROS2SubEndpoint<UserType, WrappedType>(topic_name, name, environment, is_logical, min_delay){}
};


} // reactor ns

#endif