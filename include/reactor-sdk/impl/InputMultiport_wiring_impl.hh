#pragma once

namespace sdk
{
template <typename T>
void MultiportInput<T>::connect(Input<T>& input) {
    if (n_inputs > 1) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& output_port : *this) {
        output_port.environment()->draw_connection(output_port, input, reactor::ConnectionProperties{});
        break;
    }
}

template <typename T>
void MultiportInput<T>::connect(MultiportInput<T>& input) {
    auto input_itr = input.begin();

    if (n_inputs < input.get_nports()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (n_inputs > input.get_nports()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    for (auto& output_port : *this) {
        output_port.environment()->draw_connection(output_port, *input_itr, reactor::ConnectionProperties{});
        ++input_itr;
        if (input_itr == input.end())
        {
            break;
        }
    }
}
    
} // namespace sdk
