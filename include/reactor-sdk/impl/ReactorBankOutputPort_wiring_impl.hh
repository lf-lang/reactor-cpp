#pragma once

namespace sdk
{

template <typename ReactorType, typename T>
void ReactorBankOutputPort<ReactorType, T>::connect(Input<T>& input) {
    auto reactor_itr = reactors.begin();

    if (1 < reactors.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto *reactor = (*reactor_itr).get();
    input.environment()->draw_connection(reactor->*member, input, reactor::ConnectionProperties{});
}

template <typename ReactorType, typename T>
void ReactorBankOutputPort<ReactorType, T>::connect(MultiportInput<T>& input) {
    auto reactor_itr = reactors.begin();

    if (input.get_nports() > reactors.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (input.get_nports() < reactors.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& input_port : input) {
        auto *reactor = (*reactor_itr).get();
        input_port.environment()->draw_connection(reactor->*member, input_port, reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            break;
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPort<ReactorType, T>::connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = other_bank_ports.size();

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& p_right_reactor : other_bank_ports) {
        auto *reactor = (*reactor_itr).get();
        auto *right_reactor = p_right_reactor.get();
        (reactor->*member).environment()->draw_connection(reactor->*member, right_reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            break;
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPort<ReactorType, T>::connect(ReactorBankInputMultiPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = 0;

    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        right_ports += (right_reactor->*(other_bank_ports.get_member())).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        if (reactor_itr == reactors.end())
        {
            break;
        }
        
        for (auto& right_port : right_reactor->*(other_bank_ports.get_member())) {
            auto *reactor = (*reactor_itr).get();
            (reactor->*member).environment()->draw_connection(reactor->*member, right_port, reactor::ConnectionProperties{});
            ++reactor_itr;
            if (reactor_itr == reactors.end())
            {
                break;
            }
        }
    }
}


// ReactorBankOutputPortOffset
template <typename ReactorType, typename T>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(Input<T>& input) {
    auto reactor_itr = reactors.begin();

    if (1 < reactors.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto *reactor = (*reactor_itr).get();
    char* reactor_base = reinterpret_cast<char*>(reactor);
    Output<T>* port = reinterpret_cast<Output<T>*>(reactor_base + offset);
    input.environment()->draw_connection(*port, input, reactor::ConnectionProperties{});
}

template <typename ReactorType, typename T>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(MultiportInput<T>& input) {
    auto reactor_itr = reactors.begin();

    if (input.get_nports() > reactors.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (input.get_nports() < reactors.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& input_port : input) {
        auto *reactor = (*reactor_itr).get();
        char* reactor_base = reinterpret_cast<char*>(reactor);
        Output<T>* port = reinterpret_cast<Output<T>*>(reactor_base + offset);
        input_port.environment()->draw_connection(*port, input_port, reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            break;
        }
    }
}

template <typename ReactorType, typename T>
void ReactorBankOutputPortOffset<ReactorType, T>::connect_fanout(MultiportInput<T>& input) {
    auto reactor_itr = reactors.begin();

    if (input.get_nports() > reactors.size()) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Fanning Out!";
    } else if (input.get_nports() < reactors.size()) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& input_port : input) {
        auto *reactor = (*reactor_itr).get();
        char* reactor_base = reinterpret_cast<char*>(reactor);
        Output<T>* port = reinterpret_cast<Output<T>*>(reactor_base + offset);
        input_port.environment()->draw_connection(*port, input_port, reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            reactor_itr = reactors.begin();
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = other_bank_ports.size();

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& p_right_reactor : other_bank_ports) {
        auto *reactor = (*reactor_itr).get();
        char* l_reactor_base = reinterpret_cast<char*>(reactor);
        Output<T>* l_port = reinterpret_cast<Output<T>*>(l_reactor_base + offset);
        auto *right_reactor = p_right_reactor.get();
        (*l_port).environment()->draw_connection(*l_port, right_reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            break;
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(ReactorBankInputMultiPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = 0;

    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        right_ports += (right_reactor->*(other_bank_ports.get_member())).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        if (reactor_itr == reactors.end())
        {
            break;
        }
        
        for (auto& right_port : right_reactor->*(other_bank_ports.get_member())) {
            auto *reactor = (*reactor_itr).get();
            char* l_reactor_base = reinterpret_cast<char*>(reactor);
            Output<T>* l_port = reinterpret_cast<Output<T>*>(l_reactor_base + offset);
            (*l_port).environment()->draw_connection(*l_port, right_port, reactor::ConnectionProperties{});
            ++reactor_itr;
            if (reactor_itr == reactors.end())
            {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = other_bank_ports.size();

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& p_right_reactor : other_bank_ports) {
        auto *reactor = (*reactor_itr).get();
        char* l_reactor_base = reinterpret_cast<char*>(reactor);
        Output<T>* l_port = reinterpret_cast<Output<T>*>(l_reactor_base + offset);
        auto *right_reactor = p_right_reactor.get();
        char* r_reactor_base = reinterpret_cast<char*>(right_reactor);
        Input<T>* r_port = reinterpret_cast<Input<T>*>(r_reactor_base + other_bank_ports.get_offset());
        (*l_port).environment()->draw_connection(*l_port, *r_port, reactor::ConnectionProperties{});
        ++reactor_itr;
        if (reactor_itr == reactors.end())
        {
            break;
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputPortOffset<ReactorType, T>::connect(ReactorBankInputMultiPortOffset<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = reactors.size();
    size_t right_ports = 0;

    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        char* r_reactor_base = reinterpret_cast<char*>(right_reactor);
        MultiportInput<T>* r_port = reinterpret_cast<MultiportInput<T>*>(r_reactor_base + other_bank_ports.get_offset());
        right_ports += (*r_port).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }
    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        if (reactor_itr == reactors.end())
        {
            break;
        }
        
        char* r_reactor_base = reinterpret_cast<char*>(right_reactor);
        MultiportInput<T>* r_port = reinterpret_cast<MultiportInput<T>*>(r_reactor_base + other_bank_ports.get_offset());
        for (auto& right_port : *r_port) {
            auto *reactor = (*reactor_itr).get();
            char* l_reactor_base = reinterpret_cast<char*>(reactor);
            Output<T>* l_port = reinterpret_cast<Output<T>*>(l_reactor_base + offset);
            (*l_port).environment()->draw_connection(*l_port, right_port, reactor::ConnectionProperties{});
            ++reactor_itr;
            if (reactor_itr == reactors.end())
            {
                break;
            }
        }
    }
}

} // namespace sdk