#pragma once

#include "reactor-cpp/port.hh"

/*
Input
MultiportInput
ReactorBank_Input
ReactorBank_MultiportInput

Output
MultiportOutput
ReactorBank_Output
ReactorBank_MultiportOutput

Input -> Input
Input -> MultiportInput             *
Input -> ReactorBank_Input          *
Input -> ReactorBank_MultiportInput *

Output -> Input
Output -> MultiportInput             *
Output -> ReactorBank_Input          *
Output -> ReactorBank_MultiportInput *
Output -> Output
Output -> MultiportOutput

MultiportInput -> Input
MultiportInput -> MultiportInput
MultiportInput -> ReactorBank_Input
MultiportInput -> ReactorBank_MultiportInput

MultiportOutput -> Input
MultiportOutput -> MultiportInput
MultiportOutput -> ReactorBank_Input
MultiportOutput -> ReactorBank_MultiportInput
MultiportOutput -> Output
MultiportOutput -> MultiportOutput

ReactorBank_Output -> Input
ReactorBank_Output -> MultiportInput
ReactorBank_Output -> ReactorBank_Input
ReactorBank_Output -> ReactorBank_MultiportInput
ReactorBank_Output -> Output
ReactorBank_Output -> MultiportOutput

ReactorBank_MultiportOutput -> Input
ReactorBank_MultiportOutput -> MultiportInput
ReactorBank_MultiportOutput -> ReactorBank_Input
ReactorBank_MultiportOutput -> ReactorBank_MultiportInput
ReactorBank_MultiportOutput -> Output
ReactorBank_MultiportOutput -> MultiportOutput

*/

template <typename T>
void display_(std::set<reactor::Port<T>*> &left_ports, std::set<reactor::Port<T>*> &right_ports) {
    reactor::log::Warn() << "Left Ports:";
    for (auto *left_port : left_ports) {
        reactor::log::Warn() << "\t" << left_port->fqn();
    }
    reactor::log::Warn() << "Right Ports:";
    for (auto *right_port : right_ports) {
        reactor::log::Warn() << "\t" << right_port->fqn();
    }
}

template <typename T>
void connect_(  std::set<reactor::Port<T>*> &left_ports, std::set<reactor::Port<T>*> &right_ports,
                reactor::ConnectionProperties &&property) {
    if (left_ports.size() < right_ports.size()) {
        reactor::log::Warn() << "There are more right ports (" << right_ports.size() << ") than left ports (" << left_ports.size() << ")";
        display_ (left_ports, right_ports);
    } else if (left_ports.size() > right_ports.size()) {
        reactor::log::Warn() << "There are more left ports (" << left_ports.size() << ") than right ports (" << right_ports.size() << ")";
        display_ (left_ports, right_ports);
    }

    auto right_port_itr = right_ports.begin();
    for (auto *left_port : left_ports) {
        if (right_port_itr == right_ports.end()) {
            break;
        }
        left_port->environment()->draw_connection(left_port, (*right_port_itr), reactor::ConnectionProperties{});
        ++right_port_itr;
    }
}

template <typename T>
void connect_fanout_(std::set<reactor::Port<T>*> &left_ports, std::set<reactor::Port<T>*> &right_ports,
                    reactor::ConnectionProperties &&property) {
    assert (left_ports.size() == 1);
    if (left_ports.size() < right_ports.size()) {
        reactor::log::Warn() << "There are more right ports (" << right_ports.size() << ") than left ports (" << left_ports.size() << ")";
        display_ (left_ports, right_ports);
    } else if (left_ports.size() > right_ports.size()) {
        reactor::log::Warn() << "There are more left ports (" << left_ports.size() << ") than right ports (" << right_ports.size() << ")";
        display_ (left_ports, right_ports);
    }

    reactor::log::Warn() << "Fanning out left port to all right ports";

    auto left_port_itr = left_ports.begin();
    for (auto *right_port : right_ports) {
        (*left_port_itr)->environment()->draw_connection((*left_port_itr), right_port, reactor::ConnectionProperties{});
    }

}