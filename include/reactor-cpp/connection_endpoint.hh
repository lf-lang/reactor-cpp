#ifndef REACTOR_CPP_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_CONNECTION_ENDPOINT_HH

#include <type_traits>
#include "port.hh"

/*
base classes for endpoints for federated comms
*/

namespace reactor {

/*
Downstream inherits from Action bec it produces event on receiver end
*/
template <class UserType, class InternalMessageType>
class DownstreamEndpoint : public Action<UserType> {
    protected:
        std::set<Port<UserType>*> ports_;

        virtual void schedule_this(InternalMessageType) = 0;

    public:
        DownstreamEndpoint(const std::string& name, Reactor* container, bool is_logical, Duration min_delay = Duration::zero())
        : Action<UserType>(name, container, is_logical, min_delay) {}
        DownstreamEndpoint(const std::string& name, Environment* environment, bool is_logical, Duration min_delay = Duration::zero())
        : Action<UserType>(name, environment, is_logical, min_delay) {}
 
        void add_port(Port<UserType>* port) {
            [[maybe_unused]] bool result = ports_.insert(port).second;
            reactor_assert(result);
        }

        void setup() noexcept override {
            Action<UserType>::setup();

            if constexpr (std::is_same<UserType, void>::value) {
                for (auto port : this->ports_) {
                    port->set();
                }
            } else {
                for (auto port : this->ports_) {
                    port->set(std::move(this->get()));
                }
            }

        }
};

template <class UserType>
class UpstreamEndpoint {
    protected: 
        Port<UserType>* port_ = nullptr;

        virtual PortCallback set_cb() = 0;

    public:
        virtual void set_port(Port<UserType>* port) {
            reactor_assert(port_ == nullptr);
            port_ = port;
            port_->register_set_callback(set_cb());
        }
};

// reactor ns
}
#endif