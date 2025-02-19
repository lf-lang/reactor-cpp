#pragma once

#include <string>
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

template<typename T>
class MultiportOutput : public reactor::ModifableMultiport<Output<T>> {
    size_t n_inputs;
    std::string name;
    Reactor *reactor;
    class WiringProxy {
    public:
        WiringProxy(MultiportOutput& origin) : origin(origin) {}

        void operator>(Input<T>& input) {
            origin.connect (input);
        }

        void operator>(Output<T>& input) {
            origin.connect (input);
        }

        void operator>(MultiportInput<T>& input) {
            origin.connect (input);
        }

        void operator>(MultiportOutput<T>& input) {
            origin.connect (input);
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
        MultiportOutput& origin;
    };

    void connect(Input<T>& input);
    void connect(Output<T>& input);
    void connect(MultiportInput<T>& input);
    void connect(MultiportOutput<T>& input);

    template <typename ReactorType>
    void connect(std::vector<std::unique_ptr<ReactorType>>* reactors, Input<T> ReactorType::*member);

    template <typename OtherReactorType>
    void connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports);

    template <typename OtherReactorType>
    void connect(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports);

public:
    using value_type = T;
    MultiportOutput(const std::string& name, Reactor* container)
        : name (name), reactor (container) {}

    void set_width (int width)
    {
        this->reserve(width);
        n_inputs = width;
        for (int idx = 0; idx < width; idx++) {
            std::string input_name = name + "_" + std::to_string(idx);
            this->emplace_back(input_name, reactor);
        }
    }

    MultiportOutput(MultiportOutput&&) noexcept = default;
    auto get_nports() -> int { return n_inputs; }

    WiringProxy operator--(int) {
        return WiringProxy(*this);
    }
};

    
} // namespace sdk

#include "impl/OutputMultiport_wiring_impl.hh"