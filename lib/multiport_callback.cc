/*
* Copyright (C) 2022 TU Dresden
* All rights reserved.
*
* Authors:
*   Christian Menard
*   Tassilo Tanneberger
*/
#include "reactor-cpp/multiport_callback.hh"

namespace callback {
template <class T, class A>
PortBankCallBack<T, A>::PortBankCallBack(const PortBankCallBack& other)
    : data_(other.data_), active_ports_(other.active_ports_) {}

}
