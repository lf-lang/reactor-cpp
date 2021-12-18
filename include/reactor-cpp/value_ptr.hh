/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

/**
 * @file Defines classes for smart value pointers as well as comparison
 * operators and factory functions.
 */

#pragma once

#include <memory>
#include <type_traits>

namespace reactor {

// forward declaration
template <class T>
class ImmutableValuePtr;

/**
 * @brief Smart pointer to a mutable value.
 *
 * @rst
 * Manages the lifetime of a value in conjunction with
 * :class:`ImmutableValuePtr`. Implements ownership semantics and enforces
 * unique ownership of a mutable value. :class:`MutableValuePtr` internally
 * wraps around :std-memory:`unique_ptr`. The unique ownership ensures that no
 * other reactor can reference the value while it is allowed to change.  In
 * order to share the associated value, an instance of :class:`MutableValuePtr`
 * needs to be converted to an :class:`ImmutableValuePtr` making the associated
 * value immutable.
 * @endrst
 * @tparam T type of the value managed by this class
 * @author Christian Menard
 */
template <class T>
class MutableValuePtr {
 private:
  /// The internal unique smart pointer that this class builds upon.
  std::unique_ptr<T> internal_ptr;

  /**
   * Constructor from an existing raw pointer.
   *
   * @rst
   * Constructs a :class:`MutableValuePtr` such that is obtains ownership of
   * ``value``.  This is intended only for usage by the
   * :func:`make_mutable_value()` factory function and the
   * :func:`make_immutable_copy()` method of :class:`ImmutableValuePtr`.
   * @endrst
   */
  explicit MutableValuePtr(T* value) : internal_ptr(value) {}

 public:
  /**
   * Default constructor.
   * @rst
   * Constructs a :class:`MutableValuePtr` that owns nothing.
   * @endrst
   */
  constexpr MutableValuePtr() : internal_ptr(nullptr) {}
  /**
   * Copy constructor (Deleted).
   * @rst
   * Since :class:`MutableValuePtr` enforces unique ownership, there cannot
   * be two instances pointing to the same object and therefore copying cannot
   * be allowed,
   * @endrst
   */
  MutableValuePtr(const MutableValuePtr&) = delete;
  /**
   * Move constructor.
   * @rst
   * Constructs a :class:`MutableValuePtr` by transferring ownership from
   * another :class:`MutableValuePtr` instance ``ptr``. ``ptr`` looses
   * ownership and will own nothing.
   * @endrst
   */
  MutableValuePtr(MutableValuePtr&& ptr) noexcept = default;
  /**
   * Constuctor from ``nullptr``.
   * @rst
   * Constructs a :class:`MutableValuePtr<T>` that owns nothing.
   * @endrst
   */
  explicit constexpr MutableValuePtr(std::nullptr_t) : internal_ptr(nullptr) {}

  /**
   * Move assignment operator.
   * @rst
   * Transfers ownership from ``ptr`` to this :class:`MutableValuePtr`
   * instance. If this instance previously owned a value, the value is deleted.
   * @endrst
   */
  auto operator=(MutableValuePtr&& ptr) noexcept -> MutableValuePtr& {
    this->internal_ptr = std::move(ptr.internal_ptr);
    return *this;
  }
  /**
   * Assignment operator from ``nullptr``.
   *
   * Releases ownership. If this instance previously owned a value, the
   * value is deleted.
   */
  auto operator=(std::nullptr_t) noexcept -> MutableValuePtr& {
    this->internal_ptr = nullptr;
    return *this;
  }

  /**
   * Retrieve a raw pointer to the managed value.
   */
  auto get() const noexcept -> T* { return internal_ptr.get(); }

  /**
   * Cast to ``bool``. Checks if there is an associated value.
   *
   * @return ``false`` if there is no associated value (``internal_ptr ==
   *      nullptr``), ``true`` otherwise
   */
  explicit operator bool() const { return get() == nullptr; }

  /**
   * Dereference the pointer to the managed value.
   *
   * The behavior is undefined if ``get() == nullptr``.
   */
  T& operator*() const { return *get(); }
  /**
   * Dereference the pointer to the managed value.
   *
   * Provides access to members of the associated value via ``->``. The
   * behavior is undefined if ``get() == nullptr``.
   */
  T* operator->() const { return get(); }

  // Give ImutableValuePtr access to the private constructor. This is required
  // for creating a MutableValuePtr from an ImutableValuePtr in
  // get_mutable_copy()
  friend class ImmutableValuePtr<T>;

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  friend MutableValuePtr<U> make_mutable_value(Args&&... args);
};

/**
 * @brief Smart pointer to an immutable value.
 *
 * @rst
 * Manages the lifetime of a value in conjunction with
 * :class:`MutableValuePtr`. Implements ownership semantics and allows shared
 * ownership of an immutable value.  :class:`ImmutableValuePtr` internally
 * wraps around :std-memory:`shared_ptr`. The shared ownership semantics
 * enables multiple reactors to share a value which is only safe if the value
 * is immutable.  In order to modify the associated value, an instance of
 * :class:`ImutableValuePtr` needs to be converted to an
 * :class:`MutableValuePtr` making the associated value mutable. This can be
 * achieved by calling :func:`get_mutable_copy()`.
 * @endrst
 * @tparam T type of the value managed by this class
 * @author Christian Menard
 */
template <class T>
class ImmutableValuePtr {
public:
    /// A type alias that adds ``const`` to ``T``
    using const_T = typename std::add_const<T>::type;

private:
    /// The internal shared smart pointer that this class builds upon.
    std::shared_ptr<T> internal_ptr;

    /**
     * Constructor from an existing raw pointer.
     *
     * @rst
     * Constructs an :class:`ImutableValuePtr<T>` such that is obtains ownership
     * of ``value``.  This is intended only for usage by the
     * :func:`make_immutable_value()` factory function.
     * @endrst
     */
    explicit ImmutableValuePtr(T* value) : internal_ptr(value) {}

public:
    /**
     * Default constructor.
     * @rst
     * Constructs an :class:`ImmutableValuePtr<T>` that owns nothing.
     * @endrst
     */
    constexpr ImmutableValuePtr() : internal_ptr(nullptr) {}
    /**
     * Copy constructor.
     * @rst
     * Constructs an :class:`ImutableValuePtr` by copying another
     * :class:`ImutableValuePtr` instance ``ptr``. Both pointers have the same
     * associated value and, therefore, share ownership.
     * @endrst
     */
    ImmutableValuePtr(const ImmutableValuePtr& ptr) = default;
    /**
     * Move constructor.
     * @rst
     * Constructs an :class:`ImutableValuePtr` by transferring ownership from
     * another :class:`ImmutableValuePtr` instance ``ptr``. ``ptr`` looses
     * ownership and will own nothing.
     * @endrst
     */
    ImmutableValuePtr(ImmutableValuePtr&& ptr)  noexcept = default;
    /**
     * Constuctor from ``nullptr``.
     * @rst
     * Constructs an :class:`ImutableValuePtr<T>` that owns nothing.
     * @endrst
     */
    explicit constexpr ImmutableValuePtr(std::nullptr_t) : internal_ptr(nullptr) {}
    /**
     * @rst
     * Move constructor from :class:`MutableValuePtr`.
     *
     * Constructs an :class:`ImmutableValuePtr` by transferring ownership from a
     * :class:`MutableValuePtr` instance ``ptr``. ``ptr`` looses ownership and
     * will own nothing. This effectively converts the mutable value initially
     * associated with ``ptr`` to an immutable value.
     * @endrst
     */
    explicit ImmutableValuePtr(MutableValuePtr<T>&& ptr) : internal_ptr(std::move(ptr.internal_ptr)) {}

    /**
     * Assignment operator from ``nullptr``.
     *
     * @rst
     * Releases ownership. If this instance previously owned a value that is not
     * owned by any other instance of class:`ImmutableValuePtr`, the value is
     * deleted.
     * @endrst
     */
    auto operator=(std::nullptr_t) -> ImmutableValuePtr&{
        this->internal_ptr = nullptr;
        return *this;
    }
    /**
     * @rst
     * Assignment operator from another :class:`ImmutableValuePtr`.
     *
     * Replaces the managed value of this instance by the one managed by
     * ``ptr``. Both instances share the ownership. If this instance previously
     * owned a value that is not owned by any other instance of
     * class:`ImmutableValuePtr`, the value is deleted.
     * @endrst
     */
    auto operator=(const ImmutableValuePtr& ptr) -> ImmutableValuePtr& {
        this->internal_ptr = ptr.internal_ptr;
        return *this;
    }
    /**
     * @rst
     * Move assignment operator from another :class:`ImmutableValuePtr`.
     *
     * Replaces the managed value of this instance by the one managed by ``ptr``.
     * This moves the ownership from ``ptr`` to this instance. If this instance
     * previously owned a value that is not owned by any other instance of
     * class:`ImmutableValuePtr`, the value is deleted.
     * @endrst
     */
    auto operator=(ImmutableValuePtr&& ptr)  noexcept -> ImmutableValuePtr& {
        this->internal_ptr = std::move(ptr.internal_ptr);
        return *this;
    }

    /**
     * Retrieve a raw pointer to the managed value.
     *
     * Since the associated value is immutable, this only provides const access
     * to the value.
     */
    auto get() const -> const_T* { return internal_ptr.get(); }

    /**
     * Cast to ``bool``. Checks if there is an associated value.
     *
     * @return ``false`` if there is no associated value (``internal_ptr ==
     *      nullptr``), ``true`` otherwise
     */
    explicit operator bool() const { return get() == nullptr; }

    /**
     * Dereference the pointer to the managed value.
     *
     * Since the associated value is immutable, this only provides const access
     * to the value.
     *
     * The behavior is undefined if ``get() == nullptr``.
     */
    auto operator*() const -> const_T& { return *get(); }
    /**
     * Dereference the pointer to the managed value.
     *
     * Since the associated value is immutable, this only provides const access
     * to the value.
     *
     * Provides access to members of the associated value via ``->``. The
     * behavior is undefined if ``get() == nullptr``.
     */
    auto operator->() const -> const_T* {
        return get();
    }

    /**
     * Create a mutable copy of the value associated with this instance.
     *
     * @rst
     * This is the only allowed mechanism to convert a :class:`ImmutableValuePtr`
     * to a :class:`MutableValuePtr`. In fact, it does not perform a conversion
     * but copies the associated value of this instance and gives ownership of
     * the copy to a newly created :class:`MutableValuePtr`.
     * @endrst
     *
     * Requires that ``T`` is copy constructable. The behavior is undefined if
     * ``get() == nullptr``.
     * @return a mutable value pointer
     */
    auto get_mutable_copy() const -> MutableValuePtr<T> {
        return MutableValuePtr<T>(new T(*internal_ptr));
    }

    // Give the factory function make_mutable_value() access to the private
    // constructor
    template <class U, class... Args>
    friend ImmutableValuePtr<U> make_immutable_value(Args&&... args);
};

/**
 * @rst
 * Create an instance of :class:`ImmutableValuePtr`.
 *
 * Creates and initializes a new instance of ``T`` and returns a new
 * :class:`ImmutableValuePtr` owning this value. This is analogues to
 * :std-memory:`make_shared`.
 * @endrst
 * @tparam T type of the value to be created
 * @tparam Args types of ``T``'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * givven ``args``.
 * @param args Arguments to be forwarded to ``T``'s constructor
 * @return a new immutable value pointer
 */
template <class T, class... Args>
auto make_immutable_value(Args&&... args) -> ImmutableValuePtr<T>  {
    return ImmutableValuePtr<T>(new T(std::forward<Args>(args)...));
}

/**
 * @rst
 * Create an instance of :class:`MmutableValuePtr`.

 * Creates and initializes a new instance of ``T`` and returns a new
 * :class:`MmutableValuePtr` owning this value. This is analogues to
 * :std-memory:`make_unique`.
 * @endrst
 * @tparam T type of the value to be created
 * @tparam Args types of ``T``'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * givven ``args``.
 * @param args Arguments to be forwarded to ``T``'s constructor
 * @return a new mutable value pointer
 */
template <class T, class... Args>
auto make_mutable_value(Args&&... args) -> MutableValuePtr<T> {
    return MutableValuePtr<T>(new T(std::forward<Args>(args)...));
}

// Comparison operators

template <class T, class U>
auto operator==(const MutableValuePtr<T>& x, const MutableValuePtr<U>& y) noexcept -> bool {
    return x.get() == y.get();
}
template <class T, class U>
auto operator==(const ImmutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) noexcept -> bool {
    return x.get() == y.get();
}
template <class T, class U>
auto operator==(const ImmutableValuePtr<T>& x, const MutableValuePtr<U>& y) noexcept -> bool {
    return x.get() == y.get();
}
template <class T, class U>
auto operator==(const MutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) noexcept -> bool {
    return x.get() == y.get();
}
template <class T>
auto operator==(const MutableValuePtr<T>& x, std::nullptr_t) noexcept -> bool {
    return x.get() == nullptr;
}
template <class T>
auto operator==(std::nullptr_t, const MutableValuePtr<T>& x) noexcept -> bool {
    return x.get() == nullptr;
}
template <class T>
auto operator==(const ImmutableValuePtr<T>& x, std::nullptr_t) noexcept -> bool {
    return x.get() == nullptr;
}
template <class T>
auto operator==(std::nullptr_t, const ImmutableValuePtr<T>& x) noexcept -> bool {
    return x.get() == nullptr;
}

template <class T, class U>
auto operator!=(const MutableValuePtr<T>& x, const MutableValuePtr<U>& y) noexcept -> bool {
    return x.get() != y.get();
}

template <class T, class U>
auto operator!=(const ImmutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) -> bool {
    return x.get() != y.get();
}
template <class T, class U>
auto operator!=(const ImmutableValuePtr<T>& x, const MutableValuePtr<U>& y) -> bool {
    return x.get() != y.get();
}
template <class T, class U>
auto operator!=(const MutableValuePtr<T>& x, const ImmutableValuePtr<U>& y) -> bool {
    return x.get() != y.get();
}
template <class T>
auto operator!=(const MutableValuePtr<T>& x, std::nullptr_t) -> bool {
    return x.get() != nullptr;
}
template <class T>
auto operator!=(std::nullptr_t, const MutableValuePtr<T>& x) -> bool {
    return x.get() != nullptr;
}
template <class T>
auto operator!=(const ImmutableValuePtr<T>& x, std::nullptr_t) -> bool {
    return x.get() != nullptr;
}
template <class T>
auto operator!=(std::nullptr_t, const ImmutableValuePtr<T>& x) -> bool {
    return x.get() != nullptr;
}

}  // namespace reactor
