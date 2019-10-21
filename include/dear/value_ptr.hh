/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <memory>

namespace dear {

/**
 * A smart pointer to a value.
 *
 * We use an alias here to be able to extend the functionality of a value
 * pointer later on.
 */
template <class T>
using ValuePtr = std::shared_ptr<T>;

template <class T, class... Args>
ValuePtr<T> make_value(Args&&... args) {
  return make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace dear
