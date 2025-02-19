#pragma once

#include "Connection.hh"
namespace sdk
{
template <typename T>
void MultiportOutput<T>::connect(Input<T>& input) {
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
void MultiportOutput<T>::connect(Output<T>& input) {
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
void MultiportOutput<T>::connect(MultiportInput<T>& input) {
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

template <typename T>
void MultiportOutput<T>::connect(MultiportOutput<T>& input) {
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

template <typename T>
template <typename ReactorType>
void MultiportOutput<T>::connect(std::vector<std::unique_ptr<ReactorType>>* reactors, Input<T> ReactorType::*member) {
    auto reactor_itr = reactors->begin();

    if (n_inputs < reactors->size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (n_inputs > reactors->size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& output_port : *this) {
        auto *reactor = (*reactor_itr).get();
        output_port.environment()->draw_connection(output_port, reactor->*member, reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors->end())
        {
            break;
        }
    }
}

template <typename T>
template <typename OtherReactorType>
void MultiportOutput<T>::connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = other_bank_ports.begin();

    if (n_inputs < other_bank_ports.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (n_inputs > other_bank_ports.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& output_port : *this) {
        auto *reactor = (*reactor_itr).get();
        output_port.environment()->draw_connection(output_port, reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == other_bank_ports.end())
        {
            break;
        }
    }
}

template <typename T>
template <typename OtherReactorType>
void MultiportOutput<T>::connect(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
    // auto reactor_itr = other_bank_ports.begin();

    // if (n_inputs < other_bank_ports.size()) {
    //     reactor::log::Warn() << "There are more right ports than left ports. "
    //                         << "Not all ports will be connected!";
    // } else if (n_inputs > other_bank_ports.size()) {
    //     reactor::log::Warn() << "There are more left ports than right ports. "
    //                         << "Not all ports will be connected!";
    // }
    // for (auto& output_port : *this) {
    //     auto *reactor = (*reactor_itr).get();
    //     char* reactor_base = reinterpret_cast<char*>(reactor);
    //     Input<T>* port = reinterpret_cast<Input<T>*>(reactor_base + other_bank_ports.get_offset());
    //     output_port.environment()->draw_connection(output_port, *port, reactor::ConnectionProperties{});
    //     ++reactor_itr;
    //     if (reactor_itr == other_bank_ports.end())
    //     {
    //         break;
    //     }
    // }

    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result;

    for (auto& left_port : *this) {
        result = left_ports.insert(&left_port).second;
        reactor_assert(result);
    }

    for (auto &reactor : other_bank_ports) {
        char* reactor_base = reinterpret_cast<char*>(reactor.get());
        Input<T>* port = reinterpret_cast<Input<T>*>(reactor_base + other_bank_ports.get_offset());
        result = right_ports.insert(port).second;
        reactor_assert(result);
    }

    connect_ (left_ports, right_ports, reactor::ConnectionProperties{});
}
    
} // namespace sdk
