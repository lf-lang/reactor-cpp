#pragma once

#include "reactor-cpp/reactor-cpp.hh"

namespace sdk
{

template <typename T>
class Input : public reactor::Input<T> {
    class WiringProxy {
        public:
            WiringProxy(Input& origin) : origin(origin) {}

            void operator>(Input<T>& input) {
                origin.connect (input);
            }

            void operator>(MultiportInput<T>& input) {
                origin.connect (input);
            }

            void operator>>(MultiportInput<T>& input) {
                origin.connect_fanout (input);
            }

            template <typename OtherReactorType>
            void operator>(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
                origin.connect(std::move(other_bank_ports));
            }

            template <typename OtherReactorType>
            void operator>>(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
                origin.connect_fanout(std::move(other_bank_ports));
            }

        private:
            Input& origin;
    };

    void connect(Input<T>& input);
    void connect(MultiportInput<T>& input);
    void connect_fanout(MultiportInput<T>& input);

    template <typename ReactorType>
    void connect(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports);

    template <typename ReactorType>
    void connect_fanout(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports);

public:
    using value_type = T;
    Input(const std::string& name, reactor::Reactor* container)
      : reactor::Input<T>(name, container) {}

    Input(Input&&) noexcept = default;
    ~Input() {}

    WiringProxy operator--(int) {
        return WiringProxy(*this);
    }
};
    
} // namespace sdk

#include "impl/InputPort_wiring_impl.hh"