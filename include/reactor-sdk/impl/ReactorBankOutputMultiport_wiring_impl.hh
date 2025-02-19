#pragma once

namespace sdk
{
template <typename ReactorType, typename T>
void ReactorBankOutputMultiPort<ReactorType, T>::connect(MultiportInput<T>& input) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = input.get_nports();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        left_ports += (left_reactor->*member).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_port_itr = input.begin();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_port_itr == input.end()) {
            break;
        }

        for (auto &l_port : left_reactor->*member) {
            l_port.environment()->draw_connection(l_port, *right_port_itr, reactor::ConnectionProperties{});
            ++right_port_itr;
            if (right_port_itr == input.end()) {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPort<ReactorType, T>::connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = other_bank_ports.size();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        left_ports += (left_reactor->*member).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_reactor_itr = other_bank_ports.begin();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_reactor_itr == other_bank_ports.end()) {
            break;
        }

        for (auto &l_port : left_reactor->*member) {
            auto *right_reactor = (*right_reactor_itr).get();
            l_port.environment()->draw_connection(l_port, right_reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
            ++right_reactor_itr;
            if (right_reactor_itr == other_bank_ports.end()) {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPort<ReactorType, T>::connect(ReactorBankInputMultiPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        left_ports += (left_reactor->*member).get_nports();
    }

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

    auto right_reactor_itr = other_bank_ports.begin();
    auto right_port_itr =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).begin();
    auto right_port_itr_end =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).end();
    size_t right_port_count = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_port_count == right_ports) {
            break;
        }

        for (auto &l_port : left_reactor->*member) {
            l_port.environment()->draw_connection(l_port, *right_port_itr, reactor::ConnectionProperties{});
            ++right_port_count;
            if (right_port_count == right_ports) {
                break;
            }
            ++right_port_itr;
            if (right_port_itr == right_port_itr_end) {
                ++right_reactor_itr;
                right_port_itr =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).begin();
                right_port_itr_end =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).end();
            }
        }
    }
}


template <typename ReactorType, typename T>
void ReactorBankOutputMultiPortOffset<ReactorType, T>::connect(MultiportInput<T>& input) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = input.get_nports();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_port = reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        left_ports += (*l_port).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_port_itr = input.begin();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_port_itr == input.end()) {
            break;
        }

        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        for (auto &l_port : *l_ports) {
            l_port.environment()->draw_connection(l_port, *right_port_itr, reactor::ConnectionProperties{});
            ++right_port_itr;
            if (right_port_itr == input.end()) {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPortOffset<ReactorType, T>::connect(ReactorBankInputPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = other_bank_ports.size();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        left_ports += (*l_ports).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_reactor_itr = other_bank_ports.begin();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_reactor_itr == other_bank_ports.end()) {
            break;
        }

        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        for (auto &l_port : *l_ports) {
            auto *right_reactor = (*right_reactor_itr).get();
            l_port.environment()->draw_connection(l_port, right_reactor->*(other_bank_ports.get_member()), reactor::ConnectionProperties{});
            ++right_reactor_itr;
            if (right_reactor_itr == other_bank_ports.end()) {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPortOffset<ReactorType, T>::connect(ReactorBankInputMultiPort<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        left_ports += (*l_ports).get_nports();
    }

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

    auto right_reactor_itr = other_bank_ports.begin();
    auto right_port_itr =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).begin();
    auto right_port_itr_end =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).end();
    size_t right_port_count = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_port_count == right_ports) {
            break;
        }

        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        for (auto &l_port : *l_ports) {
            l_port.environment()->draw_connection(l_port, *right_port_itr, reactor::ConnectionProperties{});
            ++right_port_count;
            if (right_port_count == right_ports) {
                break;
            }
            ++right_port_itr;
            if (right_port_itr == right_port_itr_end) {
                ++right_reactor_itr;
                right_port_itr =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).begin();
                right_port_itr_end =  (*right_reactor_itr).get()->*(other_bank_ports.get_member()).end();
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPortOffset<ReactorType, T>::connect(ReactorBankInputPortOffset<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = other_bank_ports.size();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        left_ports += (*l_ports).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_reactor_itr = other_bank_ports.begin();

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_reactor_itr == other_bank_ports.end()) {
            break;
        }

        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        for (auto &l_port : *l_ports) {
            auto *right_reactor = (*right_reactor_itr).get();
            char* r_reactor_base = reinterpret_cast<char*>(right_reactor);
            Input<T>* r_port= reinterpret_cast<Input<T>*>(r_reactor_base + other_bank_ports.get_offset());
            l_port.environment()->draw_connection(l_port, *r_port, reactor::ConnectionProperties{});
            ++right_reactor_itr;
            if (right_reactor_itr == other_bank_ports.end()) {
                break;
            }
        }
    }
}

template <typename ReactorType, typename T>
template <typename OtherReactorType>
void ReactorBankOutputMultiPortOffset<ReactorType, T>::connect(ReactorBankInputMultiPortOffset<OtherReactorType, T> &&other_bank_ports) {
    auto reactor_itr = reactors.begin();
    size_t left_ports = 0;
    size_t right_ports = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();
        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        left_ports += (*l_ports).get_nports();
    }

    for (auto& p_right_reactor : other_bank_ports) {
        auto *right_reactor = p_right_reactor.get();
        char* r_reactor_base = reinterpret_cast<char*>(right_reactor);
        MultiportInput<T>* r_ports = reinterpret_cast<MultiportInput<T>*>(r_reactor_base + other_bank_ports.get_offset());
        right_ports += (*r_ports).get_nports();
    }

    if (left_ports < right_ports) {
        reactor::log::Warn() << "There are more right ports than left ports. "
                            << "Not all ports will be connected!";
    } else if (left_ports > right_ports) {
        reactor::log::Warn() << "There are more left ports than right ports. "
                            << "Not all ports will be connected!";
    }

    auto right_reactor_itr = other_bank_ports.begin();
    char* r_reactor_base = reinterpret_cast<char*>(*right_reactor_itr);
    MultiportInput<T>* r_ports = reinterpret_cast<MultiportInput<T>*>(r_reactor_base + other_bank_ports.get_offset());
    auto right_port_itr =  (*r_ports).begin();
    auto right_port_itr_end =  (*r_ports).end();
    size_t right_port_count = 0;

    for (auto& p_left_reactor : reactors) {
        auto *left_reactor = p_left_reactor.get();

        if (right_port_count == right_ports) {
            break;
        }

        char* l_reactor_base = reinterpret_cast<char*>(left_reactor);
        MultiportOutput<T>* l_ports= reinterpret_cast<MultiportOutput<T>*>(l_reactor_base + offset);
        for (auto &l_port : *l_ports) {
            l_port.environment()->draw_connection(l_port, *right_port_itr, reactor::ConnectionProperties{});
            ++right_port_count;
            if (right_port_count == right_ports) {
                break;
            }
            ++right_port_itr;
            if (right_port_itr == right_port_itr_end) {
                ++right_reactor_itr;
                r_reactor_base = reinterpret_cast<char*>(*right_reactor_itr);
                r_ports = reinterpret_cast<MultiportInput<T>*>(r_reactor_base + other_bank_ports.get_offset());
                right_port_itr =  (*r_ports).begin();
                right_port_itr_end =  (*r_ports).end();
            }
        }
    }
}
    
} // namespace sdk
