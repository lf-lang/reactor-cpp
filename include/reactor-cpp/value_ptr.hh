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

namespace reactor {

template <class T>
class ImmutableValuePtr;

template <class T>
class MutableValuePtr {
 private:
  std::unique_ptr<T> internal_ptr;

  explicit MutableValuePtr(T* ptr) : internal_ptr(ptr) {}

 public:
  MutableValuePtr(const MutableValuePtr&) = delete;
  MutableValuePtr(MutableValuePtr&&) = default;

  constexpr MutableValuePtr() : internal_ptr(nullptr) {}
  explicit constexpr MutableValuePtr(std::nullptr_t) : internal_ptr(nullptr) {}

  MutableValuePtr& operator=(std::nullptr_t) {
    this->internal_ptr = nullptr;
    return *this;
  }

  T* get() const { return internal_ptr.get(); }

  explicit operator bool() const { return get() == nullptr; }
  T& operator*() const { return *get(); }
  T* operator->() const { return get(); }

  friend class ImmutableValuePtr<T>;

  template <class U, class... Args>
  friend MutableValuePtr<U> make_mutable_value(Args&&... args);
};

template <class T>
class ImmutableValuePtr {
 private:
  std::shared_ptr<T> internal_ptr;

  explicit ImmutableValuePtr(T* ptr) : internal_ptr(ptr) {}

 public:
  constexpr ImmutableValuePtr() : internal_ptr(nullptr) {}
  ImmutableValuePtr(const ImmutableValuePtr&) = default;
  ImmutableValuePtr(ImmutableValuePtr&&) = default;

  explicit constexpr ImmutableValuePtr(std::nullptr_t)
      : internal_ptr(nullptr) {}
  explicit ImmutableValuePtr(MutableValuePtr<T>&& ptr)
      : internal_ptr(std::move(ptr.internal_ptr)) {}

  ImmutableValuePtr& operator=(std::nullptr_t) {
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

  T* get() const { return internal_ptr.get(); }

  explicit operator bool() const { return get() == nullptr; }
  T& operator*() const { return *get(); }
  T* operator->() const { return get(); }

  MutableValuePtr<T> get_mutable_copy() const {
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
bool operator==(const MutableValuePtr<T>& x, std::nullptr_t) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(std::nullptr_t, const MutableValuePtr<T>& x) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(const ImmutableValuePtr<T>& x, std::nullptr_t) {
  return x.get() == nullptr;
}
template <class T>
bool operator==(std::nullptr_t, const ImmutableValuePtr<T>& x) {
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
bool operator!=(const MutableValuePtr<T>& x, std::nullptr_t) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(std::nullptr_t, const MutableValuePtr<T>& x) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(const ImmutableValuePtr<T>& x, std::nullptr_t) {
  return x.get() != nullptr;
}
template <class T>
bool operator!=(std::nullptr_t, const ImmutableValuePtr<T>& x) {
  return x.get() != nullptr;
}

}  // namespace reactor
