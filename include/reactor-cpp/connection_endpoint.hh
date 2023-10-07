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
template <class UserType, class WrappedType>
class DownstreamEndpoint : public Action<UserType> {
    protected:
        std::set<Port<UserType>*> ports_;

        virtual void schedule_this(/*wrapped type here*/) {
            if constexpr (std::is_same<UserType, void>::value) {
                this->schedule();
            } else {
                //this->schedule(std::move(value here));
            }
        }


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

template <class UserType, class WrappedType>
class UpstreamEndpoint {
    protected: 
        Port<UserType>* port_ = nullptr;

        virtual PortCallback set_cb() {
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<UserType>&>(port); 
                if constexpr (std::is_same<UserType, void>::value) {
                    // send void
                } else {
                    // send std::move(typed_port.get());
                }
            };
        }

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