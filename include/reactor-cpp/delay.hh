/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */
#ifndef REACTOR_CPP_DELAY_HH
#define REACTOR_CPP_DELAY_HH

#include "port.hh"

namespace reactor {

template <class T>
class Delay : public Port<T>, Action<T> {
private:
  Duration delay_;
public:
  Delay(const std::string& name, Reactor* container, Duration delay)
      : Port<T>(name, PortType::Delay, container), Action<T>(name, container, true, delay), delay_(delay)
  {
  }
  Delay(const std::string& name, Reactor* container, BaseMultiport* multiport, std::size_t index, Duration delay)
      : Port<T>(name, PortType::Delay, container, multiport, index), Action<T>(name, container, true, delay), delay_(delay) {

  }

  Delay(Delay&&) = default; // NOLINT(performance-noexcept-move-constructor)

};
}
#endif // REACTOR_CPP_DELAY_HH
