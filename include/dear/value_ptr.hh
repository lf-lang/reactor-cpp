/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <memory>
#include <type_traits>

namespace dear {

template <class T>
class ImmutableValuePtr;

template <class T>
class MutableValuePtr {
 public:
  using type = T;

 private:
  std::unique_ptr<type> internal_ptr;

  explicit MutableValuePtr(type* ptr) : internal_ptr(ptr) {}

 public:
  MutableValuePtr(const MutableValuePtr&) = delete;
  MutableValuePtr(MutableValuePtr&&) = default;

  constexpr MutableValuePtr() : internal_ptr(nullptr) {}
  explicit constexpr MutableValuePtr(nullptr_t) : internal_ptr(nullptr) {}

  MutableValuePtr& operator=(nullptr_t) {
    this->internal_ptr = nullptr;
    return *this;
  }

  type* get() const { return internal_ptr.get(); }

  explicit operator bool() const { return get() == nullptr; }
  type& operator*() const { return *get(); }
  type* operator->() const { return get(); }

  friend class ImmutableValuePtr<T>;

  template <class U, class... Args>
  friend MutableValuePtr<U> make_mutable_value(Args&&... args);
};

template <class T>
class ImmutableValuePtr {
 public:
  using type = T;

 private:
  std::shared_ptr<type> internal_ptr;

  explicit ImmutableValuePtr(type* ptr) : internal_ptr(ptr) {}

 public:
  constexpr ImmutableValuePtr() : internal_ptr(nullptr) {}
  ImmutableValuePtr(const ImmutableValuePtr&) = default;
  ImmutableValuePtr(ImmutableValuePtr&&) = default;

  explicit constexpr ImmutableValuePtr(nullptr_t) : internal_ptr(nullptr) {}
  explicit ImmutableValuePtr(MutableValuePtr<T>&& ptr)
      : internal_ptr(std::move(ptr.internal_ptr)) {}

  ImmutableValuePtr& operator=(nullptr_t) {
    this->internal_ptr = nullptr;
    return *this;
  }
  ImmutableValuePtr& operator=(const ImmutableValuePtr& ptr) {
    this->internal_ptr = ptr.internal_ptr;
    return *this;
  }
  ImmutableValuePtr& operator=(ImmutableValuePtr&& ptr) {
    this->internal_ptr = std::move(ptr.internal_ptr);
    return *this;
  }

  type* get() const { return internal_ptr.get(); }

  explicit operator bool() const { return get() == nullptr; }
  type& operator*() const { return *get(); }
  type* operator->() const { return get(); }

  MutableValuePtr<T> get_mutable_copy() {
    return MutableValuePtr<T>(new T(*internal_ptr));
  }

  template <class U, class... Args>
  friend ImmutableValuePtr<U> make_immutable_value(Args&&... args);
};

template <class T, class... Args>
ImmutableValuePtr<T> make_immutable_value(Args&&... args) {
  return ImmutableValuePtr<T>(new T(std::forward<Args>(args)...));
}

template <class T, class... Args>
MutableValuePtr<T> make_mutable_value(Args&&... args) {
  return MutableValuePtr<T>(new T(std::forward<Args>(args)...));
}

template <class T, class U>
bool operator==(const MutableValuePtr<T>& x, const MutableValuePtr<U>& y) {
  return x.get() == y.get();
}
template <class T, class U>
bool operator==(const ImmutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) {
  return x.get() == y.get();
}
template <class T, class U>
bool operator==(const ImmutableValuePtr<T>& x, const MutableValuePtr<U>& y) {
  return x.get() == y.get();
}
template <class T, class U>
bool operator==(const MutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) {
  return x.get() == y.get();
}
template <class T>
bool operator==(const MutableValuePtr<T>& x, nullptr_t) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(nullptr_t, const MutableValuePtr<T>& x) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(const ImmutableValuePtr<T>& x, nullptr_t) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(nullptr_t, const ImmutableValuePtr<T>& x) {
  return x.get() == nullptr;
}

template <class T, class U>
bool operator!=(const MutableValuePtr<T>& x, const MutableValuePtr<U>& y) {
  return x.get() != y.get();
}
template <class T, class U>
bool operator!=(const ImmutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) {
  return x.get() != y.get();
}
template <class T, class U>
bool operator!=(const ImmutableValuePtr<T>& x, const MutableValuePtr<U>& y) {
  return x.get() != y.get();
}
template <class T, class U>
bool operator!=(const MutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) {
  return x.get() != y.get();
}
template <class T>
bool operator!=(const MutableValuePtr<T>& x, nullptr_t) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(nullptr_t, const MutableValuePtr<T>& x) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(const ImmutableValuePtr<T>& x, nullptr_t) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(nullptr_t, const ImmutableValuePtr<T>& x) {
  return x.get() != nullptr;
}

}  // namespace dear
