#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int64.hpp>
#include "time_barrier.hh"
#include "lf_msgs_ros/msg/tag.hpp"
#include "logging.hh"


extern rclcpp::Node* lf_node;
static const rclcpp::QoS RELIABLE_QoS = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
#define TAG_RELEASE_SUFFIX "/released_tags"
#define EMPTY_EVENT_SUFFIX "/request_empty_event"

// maybe TODO: Swap physical and non-physical classes such that non-physical derives from physical
// this probably would make more sense code-wise since e.g. set_cb dont need to be overriden to nothing.
// however, it would decrease the similarity to the enclave connection classes, 
// which could make things a tiny bit more complicated in the future when reworking something.

namespace reactor {
/*
    UserType corresponds to a ROS msg type
    WrappedType is a generated message which includes the UserType message and a Tag
*/
template <class UserType, class WrappedType>
class ROS2PubEndpoint : public UpstreamEndpoint<UserType> {
    static_assert(std::is_same<decltype(WrappedType::tag), lf_msgs_ros::msg::Tag>::value);
    static_assert(std::is_same<decltype(WrappedType::message), UserType>::value);

    private:
        LogicalTime latest_sent_;
        log::NamedLogger lf_logger_;
        std::shared_ptr<rclcpp::Publisher<WrappedType>> pub_;
        std::shared_ptr<rclcpp::Subscription<lf_msgs_ros::msg::Tag>> empty_event_sub_;
        std::shared_ptr<rclcpp::Publisher<lf_msgs_ros::msg::Tag>> tag_only_pub_;
        

        void update_published_tag(const Tag& tag) {
            if (tag <= latest_sent_) return;
            lf_msgs_ros::msg::Tag tag_msg;
            tag_msg.microstep = tag.micro_step();
            tag_msg.time_point = tag.time_point().time_since_epoch().count();
            tag_only_pub_->publish(std::move(tag_msg));
            lf_logger_.debug() << "Updating latest published tag to " << tag << " by tag message";
            latest_sent_.advance_to(tag);
        }

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
                if (tag > latest_sent_) {
                    lf_logger_.debug() << "Updating latest published tag to " << tag << " by wrapped message";
                    latest_sent_.advance_to(tag);
                }
            };
        }

        virtual void create_publisher_tag_only(const std::string& topic_name){
            tag_only_pub_ = lf_node->create_publisher<lf_msgs_ros::msg::Tag>(topic_name + TAG_RELEASE_SUFFIX, RELIABLE_QoS);
        }

        virtual void sub_empty_event_request(const std::string& topic_name) {
            empty_event_sub_ = lf_node->create_subscription<lf_msgs_ros::msg::Tag>(topic_name + EMPTY_EVENT_SUFFIX, RELIABLE_QoS,  
                [&](const lf_msgs_ros::msg::Tag::SharedPtr msg){
                    // Insert an empty event into the upstream event queue. This ensures that we
                    // will get notified and woken up as soon as the tag becomes safe to process.
                    // It is important to unlock the mutex here. Otherwise we could enter a deadlock as
                    // scheduling the upstream event also requires holding the upstream mutex.
                    //lock.unlock();
                    reactor::Tag tag(reactor::TimePoint(std::chrono::nanoseconds(msg->time_point)), mstep_t(msg->microstep));
                    bool result = this->port_->environment()->scheduler()->schedule_empty_async_at(tag);
                    //lock.lock();

                    // If inserting the empty event was not successful, then this is because the upstream
                    // scheduler already processes a later event. In this case, it is safe to assume that
                    // the tag is acquired.
                    if (!result) {
                        // sending most recent tag
                        update_published_tag(reactor::Tag::from_logical_time(this->port_->environment()->scheduler()->logical_time()));
                    }
                });
        }


    public:
        ROS2PubEndpoint(const std::string& topic_name) : UpstreamEndpoint<UserType>(), lf_logger_("Publisher " + topic_name) {
            lf_logger_.debug() << "Publishing on topic " << topic_name;
            pub_ = lf_node->create_publisher<WrappedType>(topic_name, RELIABLE_QoS);
            create_publisher_tag_only(topic_name);
            sub_empty_event_request(topic_name);
        }

        void set_port(Port<UserType>* port) override {
            this->UpstreamEndpoint<UserType>::set_port(port);
            port->environment()->scheduler()->register_release_tag_callback([this](const LogicalTime& tag) {
                update_published_tag(reactor::Tag::from_logical_time(tag));
            });
        }
};

template <class UserType, class WrappedType>
class ROS2SubEndpoint : public DownstreamEndpoint<UserType, std::shared_ptr<WrappedType>> {
    static_assert(std::is_same<decltype(WrappedType::tag), lf_msgs_ros::msg::Tag>::value);
    static_assert(std::is_same<decltype(WrappedType::message), UserType>::value);

    protected:
        std::shared_ptr<rclcpp::Subscription<WrappedType>> sub_;
        rclcpp::Subscription<lf_msgs_ros::msg::Tag>::SharedPtr tag_only_sub_;
        rclcpp::Publisher<lf_msgs_ros::msg::Tag>::SharedPtr empty_event_pub_;
        log::NamedLogger lf_logger_;

        LogicalTimeBarrier publisher_time_barrier_;

        // this tracks the time of publisher_time_barrier_
        // to avoid using a lock for try_acquire_tag()
        // should be fine as this endpoint is the only one releasing tags on that barrier
        // TODO: verify that that makes sense (or use try_acquire_tag with lock)
        LogicalTime publisher_time_barrier_time_;

        void publisher_time_barrier_release(const reactor::Tag& tag) {
            if (publisher_time_barrier_time_ < tag) {
                lf_logger_.debug() << "Releasing " << tag << " on publisher_time_barrier";
                publisher_time_barrier_time_.advance_to(tag);
                publisher_time_barrier_.release_tag(publisher_time_barrier_time_);
            }
        }

        void schedule_this_at_tag(std::shared_ptr<UserType>&& msg, const reactor::Tag& tag) {
            lf_logger_.debug() << "Scheduling this at " << tag;
            if constexpr(detail::is_trivial<UserType>())
                this->schedule_at(ImmutableValuePtr<UserType>(*msg.get()), tag);
            else 
                this->schedule_at(ImmutableValuePtr<UserType>(std::move(msg)), tag);
        }
        

        virtual void schedule_this(std::shared_ptr<WrappedType> wrapped_msg) override{
            // unwrapping the message using an aliasing constructor
            std::shared_ptr<UserType> inner_ptr(wrapped_msg, &wrapped_msg->message);
            reactor::Tag tag(reactor::TimePoint(std::chrono::nanoseconds(wrapped_msg->tag.time_point)), mstep_t(wrapped_msg->tag.microstep));
            publisher_time_barrier_release(tag);
            lf_logger_.debug() << "Got tag from wrapped message " << tag;
            schedule_this_at_tag(std::move(inner_ptr), tag);
            
        }

        virtual bool acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                           const std::function<bool(void)>& abort_waiting) override {

            lf_logger_.debug() <<"Trying to acquire tag " << tag << " at local " << this->environment()->scheduler()->logical_time();
            if (publisher_time_barrier_.try_acquire_tag(tag)) {
                lf_logger_.debug() << "Tag already done " << tag; 
                return true;
            }
            lf_logger_.debug() << "Requesting empty event at tag " << tag;
            lf_msgs_ros::msg::Tag t;
            t.microstep = tag.micro_step();
            t.time_point = tag.time_point().time_since_epoch().count();
            this->empty_event_pub_->publish(std::move(t));

            return publisher_time_barrier_.acquire_tag(tag, lock, abort_waiting);
        }

        virtual void sub_to_tag(const std::string& topic_name) {
            tag_only_sub_ = lf_node->create_subscription<lf_msgs_ros::msg::Tag>(topic_name + TAG_RELEASE_SUFFIX, RELIABLE_QoS,
            [&](const lf_msgs_ros::msg::Tag::SharedPtr msg){
                reactor::Tag tag(reactor::TimePoint(std::chrono::nanoseconds(msg->time_point)), mstep_t(msg->microstep));
                lf_logger_.debug() << "Got tag from release topic " << tag;
                publisher_time_barrier_release(tag);
            });
        }

        virtual void pub_empty_event_topic(const std::string& topic_name) {
            this->empty_event_pub_ = lf_node->create_publisher<lf_msgs_ros::msg::Tag>(topic_name + EMPTY_EVENT_SUFFIX, RELIABLE_QoS);
        }

        ROS2SubEndpoint(const std::string& topic_name, const std::string& name, Environment* environment, const Duration& delay) 
            : DownstreamEndpoint<UserType, std::shared_ptr<WrappedType>>(name, environment, delay),
            publisher_time_barrier_(environment->scheduler()),
            lf_logger_(this->fqn())
        {
            lf_logger_.debug() << "Subscribing to " << topic_name;
            std::function<void(std::shared_ptr<WrappedType>)> f = 
                std::bind(&ROS2SubEndpoint::schedule_this, this, std::placeholders::_1);
            sub_ = lf_node->create_subscription<WrappedType>(topic_name, RELIABLE_QoS, f);

            pub_empty_event_topic(topic_name);
            sub_to_tag(topic_name);
        }

    public:
        ROS2SubEndpoint(const std::string& topic_name, const std::string& name, Environment* environment) 
        : ROS2SubEndpoint(topic_name, name, environment, Duration::zero()){}

};

template <class UserType, class WrappedType>
class ROS2SubEndpointDelayed : public ROS2SubEndpoint<UserType, WrappedType> {
    static_assert(std::is_same<decltype(WrappedType::tag), lf_msgs_ros::msg::Tag>::value);
    static_assert(std::is_same<decltype(WrappedType::message), UserType>::value); 

    protected:
        void schedule_this(std::shared_ptr<WrappedType> wrapped_msg) override final {
            // unwrapping the message using an aliasing constructor
            std::shared_ptr<UserType> inner_ptr(wrapped_msg, &wrapped_msg->message);
            reactor::Tag tag(reactor::TimePoint(std::chrono::nanoseconds(wrapped_msg->tag.time_point)), mstep_t(wrapped_msg->tag.microstep));
            this->lf_logger_.debug() << "Got tag from wrapped message " << tag;
            this->publisher_time_barrier_release(tag);
            reactor::Tag delayed_tag = tag.delay(this->min_delay());
            this->lf_logger_.debug() << "delayed schedule tag " << tag << " to " << delayed_tag;
            this->schedule_this_at_tag(std::move(inner_ptr), delayed_tag);
            
        }

        virtual bool acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                           const std::function<bool(void)>& abort_waiting) override final {
            // need to subtract delay to not get stuck waiting for the same tag in pub and sub
            reactor::Tag publisher_tag = tag.subtract(this->min_delay());
            return this->ROS2SubEndpoint<UserType, WrappedType>::acquire_tag(publisher_tag, lock, abort_waiting);
        }
     
    public:
        ROS2SubEndpointDelayed(const std::string& topic_name, const std::string& name, Environment* environment, const reactor::Duration& delay) 
        : ROS2SubEndpoint<UserType, WrappedType>(topic_name, name, environment, delay)
        {}
};

template <class UserType, class WrappedType>
class ROS2PubEndpointPhysical : public ROS2PubEndpoint<UserType, WrappedType> {
    private:
    protected:
        
        virtual void create_publisher_tag_only(const std::string& topic_name) override final {
            // not necessary
        }

        virtual void sub_empty_event_request(const std::string& topic_name) override final {
            // not necessary
        }

    public:
        ROS2PubEndpointPhysical(const std::string& topic_name) : ROS2PubEndpoint<UserType, WrappedType>(topic_name) { 
        }

        void set_port(Port<UserType>* port) override {
            this->UpstreamEndpoint<UserType>::set_port(port);
            // no registering of tag release callback necessary
        }
};



template <class UserType, class WrappedType>
class ROS2SubEndpointPhysical : public DownstreamEndpoint<UserType, std::shared_ptr<WrappedType>> {
    private:
        std::shared_ptr<rclcpp::Subscription<WrappedType>> sub_;
    protected:
        virtual void schedule_this(std::shared_ptr<WrappedType> wrapped_msg) override {
            // unwrapping the message using an aliasing constructor, ignoring the tag
            std::shared_ptr<UserType> inner_ptr(wrapped_msg, &wrapped_msg->message);
            if constexpr(detail::is_trivial<UserType>())
                this->schedule(ImmutableValuePtr<UserType>(*inner_ptr.get()));
            else 
                this->schedule(ImmutableValuePtr<UserType>(std::move(inner_ptr)));
        }

        inline bool acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                          const std::function<bool(void)>& abort_waiting) override final {
            return PhysicalTimeBarrier::acquire_tag(tag, lock, this->environment()->scheduler(), abort_waiting);
        }

        ROS2SubEndpointPhysical(const std::string& topic_name, const std::string& name, Environment* environment, const Duration& delay) 
        : DownstreamEndpoint<UserType, std::shared_ptr<WrappedType>>(name, environment, delay){
            std::function<void(std::shared_ptr<WrappedType>)> f = 
                std::bind(&ROS2SubEndpointPhysical::schedule_this, this, std::placeholders::_1);
            sub_ = lf_node->create_subscription<WrappedType>(topic_name, RELIABLE_QoS,
                        f);
        }

    public:
        ROS2SubEndpointPhysical(const std::string& topic_name, const std::string& name, Environment* environment)
            : ROS2SubEndpointPhysical<UserType, WrappedType>(topic_name, name, environment, Duration::zero()) {}
};

template <class UserType, class WrappedType>
class ROS2SubEndpointPhysicalDelayed : public ROS2SubEndpointPhysical<UserType, WrappedType> {
    protected: 
        void schedule_this(std::shared_ptr<WrappedType> wrapped_msg) override final {
            // unwrapping the message using an aliasing constructor, ignoring the tag
            std::shared_ptr<UserType> inner_ptr(wrapped_msg, &wrapped_msg->message);
            Tag t(get_physical_time(), mstep_t(0));
            if constexpr(detail::is_trivial<UserType>())
                this->schedule_at(ImmutableValuePtr<UserType>(*inner_ptr.get()), t.delay(this->min_delay()));
            else 
                this->schedule_at(ImmutableValuePtr<UserType>(std::move(inner_ptr)), t.delay(this->min_delay()));
        }
        // TODO: maybe subtract delay from tag in acquire tag?
    
    public: 
        ROS2SubEndpointPhysicalDelayed(const std::string& topic_name, const std::string& name, Environment* environment, const Duration& delay) 
        : ROS2SubEndpointPhysical<UserType, WrappedType>(topic_name, name, environment, delay)
        {}

};


} // reactor ns

#endif