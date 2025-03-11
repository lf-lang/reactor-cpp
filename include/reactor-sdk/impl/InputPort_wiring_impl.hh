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
    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result = left_ports.insert(this).second;
    reactor_assert(result);

    for (auto& right_port : input) {
        result = right_ports.insert(&right_port).second;
        reactor_assert(result);
    }
    connect_ (left_ports, right_ports, reactor::ConnectionProperties{});
}

template <typename T>
void Input<T>::connect_fanout(MultiportInput<T>& input) {
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

template <typename T>
template <typename ReactorType>
void Input<T>::connect(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports) {
    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result = left_ports.insert(this).second;
    reactor_assert(result);
    for (auto &p_reactor : other_bank_ports) {
        auto *reactor = p_reactor.get();
        char* reactor_base = reinterpret_cast<char*>(reactor);
        Input<T>* port = reinterpret_cast<Input<T>*>(reactor_base + other_bank_ports.get_offset());
        result = right_ports.insert(port).second;
        reactor_assert(result);
    }
    connect_ (left_ports, right_ports, reactor::ConnectionProperties{});
}

template <typename T>
template <typename ReactorType>
void Input<T>::connect_fanout(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports) {
    std::set<reactor::Port<T>*> left_ports;
    std::set<reactor::Port<T>*> right_ports;
    bool result = left_ports.insert(this).second;
    reactor_assert(result);
    for (auto &p_reactor : other_bank_ports) {
        auto *reactor = p_reactor.get();
        char* reactor_base = reinterpret_cast<char*>(reactor);
        Input<T>* port = reinterpret_cast<Input<T>*>(reactor_base + other_bank_ports.get_offset());
        result = right_ports.insert(port).second;
        reactor_assert(result);
    }
    connect_fanout_ (left_ports, right_ports, reactor::ConnectionProperties{});
}
    
} // namespace sdk
