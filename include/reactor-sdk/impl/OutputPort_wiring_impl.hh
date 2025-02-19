#pragma once

#include "Connection.hh"

namespace sdk
{

template <typename T>
void Output<T>::connect(Input<T>& input) {
    this->environment()->draw_connection(*this, input, reactor::ConnectionProperties{});
}

template <typename T>
void Output<T>::connect(Output<T>& input) {
    this->environment()->draw_connection(*this, input, reactor::ConnectionProperties{});
}

template <typename T>
void Output<T>::connect(MultiportInput<T>& input) {
    if (is_accumulated) {
        auto input_itr = input.begin();

        if (accumulated.size() < input.get_nports()) {
            reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
        } else if (accumulated.size() > input.get_nports()) {
            reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
        }

        this->environment()->draw_connection(*this, *input_itr, reactor::ConnectionProperties{});
        ++input_itr;

        for (auto *l_port : accumulated) {
            if (input_itr != input.end()) {
                break;
            }
            this->environment()->draw_connection(*l_port, *input_itr, reactor::ConnectionProperties{});
            ++input_itr;
        }
        is_accumulated = false;
        accumulated.clear();
    } else {
        // auto input_itr = input.begin();

        // if (1 < input.get_nports()) {
        //     reactor::log::Warn() << "There are more right ports than left ports. "
        //                     << "Not all ports will be connected!";
        // }

        // if (input_itr != input.end())
        // {
        //     this->environment()->draw_connection(*this, *input_itr, reactor::ConnectionProperties{});
        // }
        std::set<reactor::Port<T>*> left_ports;
        std::set<reactor::Port<T>*> right_ports;
        bool result = left_ports.insert(this).second;
        reactor_assert(result);
        for (auto &right_port : input) {
            result = right_ports.insert(&right_port).second;
            reactor_assert(result);
        }
        connect_ (left_ports, right_ports, reactor::ConnectionProperties{});
    }
}

template <typename T>
void Output<T>::connect_fanout(MultiportInput<T>& input) {
    if (is_accumulated) {
        auto input_itr = input.begin();

        if (accumulated.size() < input.get_nports()) {
            reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
        } else if (accumulated.size() > input.get_nports()) {
            reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
        }

        this->environment()->draw_connection(*this, *input_itr, reactor::ConnectionProperties{});
        ++input_itr;

        for (auto *l_port : accumulated) {
            if (input_itr != input.end()) {
                break;
            }
            this->environment()->draw_connection(*l_port, *input_itr, reactor::ConnectionProperties{});
            ++input_itr;
        }
        is_accumulated = false;
        accumulated.clear();
    } else {
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
}

template <typename T>
template <typename ReactorType>
void Output<T>::connect(std::vector<std::unique_ptr<ReactorType>>* reactors, Input<T> ReactorType::*member) {

    if (1 < reactors->size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    }
    for (auto &p_reactor : *reactors) {
        auto *reactor = p_reactor.get();
        this->environment()->draw_connection(*this, reactor->*member, reactor::ConnectionProperties{});
    }
}

template <typename T>
template <typename ReactorType>
void Output<T>::connect(ReactorBankInputPort<ReactorType, T> &&other_bank_ports) {

    if (1 < other_bank_ports.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    }
    for (auto &p_reactor : other_bank_ports) {
        auto *reactor = p_reactor.get();
        this->environment()->draw_connection(*this, reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
    }
}

template <typename T>
template <typename ReactorType>
void Output<T>::connect(ReactorBankInputPortOffset<ReactorType, T> &&other_bank_ports) {

    if (1 < other_bank_ports.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    }
    for (auto &p_reactor : other_bank_ports) {
        auto *reactor = p_reactor.get();
        char* reactor_base = reinterpret_cast<char*>(reactor);
        Input<T>* port = reinterpret_cast<Input<T>*>(reactor_base + other_bank_ports.get_offset());
        this->environment()->draw_connection(*this, *port, reactor::ConnectionProperties{});
    }
}
    
} // namespace sdk
