/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MULTIPORT_HH
#define REACTOR_CPP_MULTIPORT_HH

#include <algorithm>
#include <atomic>
#include <iostream>
#include <type_traits>
#include <vector>

#include "assert.hh"
#include "port.hh"

namespace reactor {

class BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions,-warnings-as-errors
protected:
  std::atomic<std::size_t> size_{0};         // NOLINT
  std::vector<std::size_t> present_ports_{}; // NOLINT

public:
  BaseMultiport() = default;
  ~BaseMultiport() = default;

  // tells the parent multiport that this port has been set.
  [[nodiscard]] inline auto set_present(std::size_t index) -> bool;

  // resets parent multiport
  inline void clear() noexcept { size_.store(0, std::memory_order_relaxed); }

  // this returns lambdas which are given to the port for callback
  [[nodiscard]] inline auto get_set_callback(std::size_t index) noexcept -> std::function<bool(const BasePort&)>;
  [[nodiscard]] inline auto get_clean_callback() noexcept -> std::function<bool(const BasePort&)>;
};

template <class T, class A = std::allocator<T>>
class Multiport : public BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions
protected:
  std::vector<T> data_{}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

public:
  using value_type = typename A::value_type;
  using size_type = typename A::size_type;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  Multiport() noexcept = default;
  ~Multiport() noexcept = default;

  auto operator==(const Multiport& other) const noexcept -> bool {
    return std::equal(std::begin(data_), std::end(data_), std::begin(other.data_), std::end(other.data_));
  }
  auto operator!=(const Multiport& other) const noexcept -> bool { return !(*this == other); };
  inline auto operator[](std::size_t index) noexcept -> T& { return data_[index]; }
  inline auto operator[](std::size_t index) const noexcept -> const T& { return data_[index]; }

  inline auto begin() noexcept -> iterator { return data_.begin(); };
  inline auto begin() const noexcept -> const_iterator { return data_.begin(); };
  inline auto cbegin() const noexcept -> const_iterator { return data_.cbegin(); };
  inline auto end() noexcept -> iterator { return data_.end(); };
  inline auto end() const noexcept -> const_iterator { return data_.end(); };
  inline auto cend() const noexcept -> const_iterator { return data_.cend(); };

  inline auto size() const noexcept -> size_type { return data_.size(); };
  [[nodiscard]] inline auto empty() const noexcept -> bool { return data_.empty(); };

  [[nodiscard]] inline auto present_indices_unsorted() const noexcept -> std::vector<std::size_t> {
    return std::vector<std::size_t>(std::begin(present_ports_), std::begin(present_ports_) + size_.load());
  }

  [[nodiscard]] inline auto present_indices_sorted() const noexcept -> std::vector<std::size_t> {
    std::sort(std::begin(present_ports_), std::begin(present_ports_) + size_.load());
    return std::vector<std::size_t>(std::begin(present_ports_), std::begin(present_ports_) + size_.load());
  }
};

template <class T, class A = std::allocator<T>> class ModifableMultiport : public Multiport<T, A> { // NOLINT
public:
  ModifableMultiport()
      : Multiport<T>() {} // NOLINT
  ~ModifableMultiport() = default;

  inline void reserve(std::size_t size) noexcept {
    this->data_.reserve(size);
    this->present_ports_.reserve(size);
  }

  inline void push_back(const T& elem) noexcept {
    this->data_.push_back(elem);
    register_latest_port();
  }

  template <class... Args> inline void emplace_back(Args&&... args) noexcept {
    this->data_.emplace_back(args...);
    register_latest_port();
  }

  private:
    void register_latest_port() {
      reactor_assert(this->data_.size() > 0);

      // need to add one mow slot t the present list
      this->present_ports_.emplace_back(0);
      reactor_assert(this->data_.size() == this->present_ports_.size());

      // and we need to register callbacks on the port
      auto idx = this->data_.size() - 1;
      this->data_.back().register_set_handler(this->get_set_callback(idx));
      this->data_.back().register_clean_handler(this->get_clean_callback());
    }
};
} // namespace reactor

#endif // REACTOR_CPP_MULTIPORT_HH
