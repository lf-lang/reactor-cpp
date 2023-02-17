/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 *   Christian Menard
 */

#include "reactor-cpp/multiport.hh"
#include "reactor-cpp/port.hh"

auto reactor::BaseMultiport::get_set_callback(std::size_t index) noexcept -> reactor::PortCallback {
  // tells the parent multiport that this port has been set.
  return [this, index](const BasePort& port) {
    // if the port is present, the callback was already invoked before
    if (!port.is_present()) {
      this->set_present(index);
    }
  };
}

void reactor::BaseMultiport::set_present(std::size_t index) {
  auto calculated_index = size_.fetch_add(1, std::memory_order_relaxed);

  reactor_assert(calculated_index < present_ports_.size());

  present_ports_[calculated_index] = index;
}

void reactor::BaseMultiport::register_port(BasePort& port, size_t idx) {
  // need to add one new slot t the present list
  reactor_assert(this->present_ports_.size() == idx);
  this->present_ports_.emplace_back(0);

  // and we need to register callbacks on the port
  port.register_set_callback(this->get_set_callback(idx));
  port.register_clean_callback(this->get_clean_callback());
}
