#pragma once

#include "Connection.hh"

namespace sdk
{

template <typename T>
void Input<T>::connect(Input<T>& input) {
    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result = left_ports.insert(this).second;
    reactor_assert(result);
    result = right_ports.insert(&input).second;
    reactor_assert(result);
    connect_ (left_ports, right_ports, reactor::ConnectionProperties{});
}

template <typename T>
void Input<T>::connect(MultiportInput<T>& input) {
    auto input_itr = input.begin();

    if (1 < input.get_nports()) {
        reactor::log::Warn() << "There are more right ports than left ports, not all ports would be connected";
    }

    if (input_itr != input.end())
    {
        this->environment()->draw_connection(*this, *input_itr, reactor::ConnectionProperties{});
    }
}

template <typename T>
void Input<T>::connect_fanout(MultiportInput<T>& input) {
    // auto input_itr = input.begin();

    // if (1 < input.get_nports()) {
    //     reactor::log::Warn() << "Fanning out input to all right output ports";
    // }

    // while (input_itr != input.end())
    // {
    //     this->environment()->draw_connection(*this, *input_itr, reactor::ConnectionProperties{});
    //     ++input_itr;
    // }

    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result = left_ports.insert(this).second;
    reactor_assert(result);
    for (auto &right_port : input) {
        result = right_ports.insert(&right_port).second;
        reactor_assert(result);
    }
    connect_fanout_ (left_ports, right_ports, reactor::ConnectionProperties{});
}
    
} // namespace sdk
