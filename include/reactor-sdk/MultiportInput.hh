#pragma once

#include <string>
#include "reactor-cpp/reactor-cpp.hh"

namespace sdk
{

template<typename T>
class Input;

class Reactor;

template<typename T>
class MultiportInput : public reactor::ModifableMultiport<Input<T>> {
    size_t n_inputs;
    std::string name;
    Reactor *reactor;

    class WiringProxy {
        public:
            WiringProxy(MultiportInput& origin) : origin(origin) {}

            void operator>(Input<T>& input) {
                origin.connect (input);
            }

            void operator>(MultiportInput<T>& input) {
                origin.connect (input);
            }

        private:
            MultiportInput& origin;
    };

    void connect(Input<T>& input);
    void connect(MultiportInput<T>& input);

public:
    using value_type = T;
    MultiportInput(const std::string& name, Reactor* container)
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

    MultiportInput(MultiportInput&&) noexcept = default;
    auto get_nports() -> int { return n_inputs; }

    WiringProxy operator--(int) {
        return WiringProxy(*this);
    }
};

    
} // namespace sdk

#include "impl/InputMultiport_wiring_impl.hh"