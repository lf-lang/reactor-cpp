#pragma once

#include "reactor-cpp/reactor-cpp.hh"

namespace sdk
{

template<typename T>
class Input;

template<typename T>
class Output;

template<typename T>
class MultiportOutput;

template<typename T>
class MultiportInput;

class Reactor;

template <typename T>
class Output : public reactor::Output<T> {
    std::set<Output<T>*> accumulated;
    bool is_accumulated = false;

    class WiringProxy {
        public:
            WiringProxy(Output& origin) : origin(origin) {}

            void operator>(Input<T>& input) {
                origin.connect (input);
            }

            void operator>(Output<T>& input) {
                origin.connect (input);
            }

            void operator>(MultiportInput<T>& input) {
                origin.connect (input);
            }

            void operator>>(MultiportInput<T>& input) {
                origin.connect_fanout (input);
            }

            template <typename ReactorType>
            void operator>(std::pair<std::vector<std::unique_ptr<ReactorType>>*, Input<T> ReactorType::*> connections)
            {
                origin.connect (connections.first, connections.second);
            }

            template <typename OtherReactorType>
            void operator>(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
                origin.connect(std::move(other_bank_ports));
            }

            template <typename OtherReactorType>
            void operator>(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
                origin.connect(std::move(other_bank_ports));
            }

        private:
            Output& origin;
    };

    void connect(Input<T>& input);
    void connect(Output<T>& input);
    void connect(MultiportInput<T>& input);
    void connect_fanout(MultiportInput<T>& input);

    template <typename ReactorType>
    void connect(std::vector<std::unique_ptr<ReactorType>>* reactors, Input<T> ReactorType::*member);

    template <typename ReactorType>
    void connect(ReactorBankInputPort<ReactorType, T> &&other_bank_ports);

    template <typename ReactorType>
    void connect(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports);

public:
    using value_type = T;
    Output(const std::string& name, reactor::Reactor* container)
      : reactor::Output<T>(name, container) {}
    
    ~Output() {}

    Output(Output&&) noexcept = default;

    WiringProxy operator--(int) {
        return WiringProxy(*this);
    }

    Output<T>& operator+(Output<T> &output) {
        [[maybe_unused]] bool result = accumulated.insert(&output).second;
        reactor_assert(result);
        is_accumulated = true;
        return *this;
    }
};

    
} // namespace sdk

#include "impl/OutputPort_wiring_impl.hh"