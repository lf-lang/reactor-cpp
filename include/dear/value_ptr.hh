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
 * A unique pointer to a value.
 *
 * The value pointer has the same semantics as std::unique_ptr. Currently, it
 * is just a wrapper for std::unique_ptr. We need the value pointer as an
 * additional layer of indirection, as we will need to incorporate other types
 * of pointers at some later point. For instance, the APD uses its own smart
 * pointers to provide access to event samples.
 */
template <class T>
class ValuePtr {
 private:
  std::unique_ptr<T> internal_ptr;

  ValuePtr(const ValuePtr&) = delete;

 public:
  constexpr ValuePtr() : internal_ptr(nullptr) {}
  explicit constexpr ValuePtr(nullptr_t) : internal_ptr(nullptr) {}
  explicit ValuePtr(T* ptr) : internal_ptr(ptr) {}
  ValuePtr(ValuePtr<T>&& ptr) : internal_ptr(std::move(ptr.internal_ptr)) {}

  ValuePtr& operator=(ValuePtr<T>&& ptr) {
    this->internal_ptr = std::move(ptr.internal_ptr);
  }
  ValuePtr& operator=(nullptr_t) { this->internal_ptr = nullptr; }

  T* get() const { return internal_ptr.get(); }

  explicit operator bool() const { return get() == nullptr; }
  T& operator*() const { return *get(); }
  T* operator->() const { return get(); }
};

template <class T, class... Args>
ValuePtr<T> make_value(Args&&... args) {
  return ValuePtr<T>(new T(std::forward<Args>(args)...));
}

template <class T, class U>
bool operator==(const ValuePtr<T>& x, const ValuePtr<U>& y) {
  return x.get() == y.get();
}
template <class T>
bool operator==(const ValuePtr<T>& x, nullptr_t) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(nullptr_t, const ValuePtr<T>& x) {
  return x.get() == nullptr;
}

template <class T, class U>
bool operator!=(const ValuePtr<T>& x, const ValuePtr<U>& y) {
  return x.get() != y.get();
}
template <class T>
bool operator!=(const ValuePtr<T>& x, nullptr_t) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(nullptr_t, const ValuePtr<T>& x) {
  return x.get() != nullptr;
}

}  // namespace dear
