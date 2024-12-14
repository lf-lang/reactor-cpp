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
#include <cstddef>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

#include "assert.hh"
#include "fwd.hh"

namespace reactor {

class BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions,-warnings-as-errors
private:
  std::atomic<std::size_t> present_ports_size_{0};
  std::vector<std::size_t> present_ports_{};
  std::string multiport_name_;
  std::string fqn_;

  // record that the port with the given index has been set
  void set_present(std::size_t index);

  // reset the list of set port indexes
  void reset() noexcept { present_ports_size_.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto get_set_callback(std::size_t index) noexcept -> PortCallback;
  const PortCallback clean_callback_{[this]([[maybe_unused]] const BasePort& port) { this->reset(); }};

  [[nodiscard]] auto get_clean_callback() const noexcept -> const PortCallback& { return clean_callback_; }

protected:
  [[nodiscard]] auto present_ports() const -> const auto& { return present_ports_; }
  [[nodiscard]] auto present_ports_size() const -> auto { return present_ports_size_.load(); }

  void present_ports_reserve(size_t n) { present_ports_.reserve(n); }
  void present_ports_pop_back() { present_ports_.pop_back(); }

  void register_port(BasePort& port, size_t idx);

public:
  explicit BaseMultiport(std::string name)
      : multiport_name_(std::move(name)){};
  ~BaseMultiport() = default;
  [[nodiscard]] auto name() const -> std::string { return multiport_name_; }
};

template <class T> class MutationChangeMultiportSize;

template <class T, class A = std::allocator<T>>
class Multiport : public BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions
protected:
  std::vector<T> ports_{}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  friend MutationChangeMultiportSize<T>;

public:
  using value_type = typename A::value_type;
  using size_type = typename A::size_type;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  explicit Multiport(const std::string& name) noexcept
      : BaseMultiport(name){};
  ~Multiport() noexcept = default;

  auto operator==(const Multiport& other) const noexcept -> bool {
    return std::equal(std::begin(ports_), std::end(ports_), std::begin(other.ports_), std::end(other.ports_));
  }
  auto operator!=(const Multiport& other) const noexcept -> bool { return !(*this == other); };
  auto operator[](std::size_t index) noexcept -> T& { return ports_[index]; }
  auto operator[](std::size_t index) const noexcept -> const T& { return ports_[index]; }

  [[nodiscard]] auto begin() noexcept -> iterator { return ports_.begin(); };
  [[nodiscard]] auto begin() const noexcept -> const_iterator { return ports_.begin(); };
  [[nodiscard]] auto cbegin() const noexcept -> const_iterator { return ports_.cbegin(); };
  [[nodiscard]] auto end() noexcept -> iterator { return ports_.end(); };
  [[nodiscard]] auto end() const noexcept -> const_iterator { return ports_.end(); };
  [[nodiscard]] auto cend() const noexcept -> const_iterator { return ports_.cend(); };

  [[nodiscard]] auto size() const noexcept -> size_type { return ports_.size(); };
  [[nodiscard]] auto empty() const noexcept -> bool { return ports_.empty(); };

  [[nodiscard]] auto present_indices_unsorted() const noexcept -> std::vector<std::size_t> {
    return std::vector<std::size_t>(std::begin(present_ports()), std::begin(present_ports()) + present_ports_size());
  }

  [[nodiscard]] auto present_indices_sorted() const noexcept -> std::vector<std::size_t> {
    std::vector<std::size_t> indices(std::begin(present_ports()), std::begin(present_ports()) + present_ports_size());
    std::sort(std::begin(indices), std::end(indices));
    return indices;
  }
};

template <class T, class A = std::allocator<T>> class ModifableMultiport : public Multiport<T, A> {
public:
  ModifableMultiport(const std::string& name)
      : Multiport<T, A>(name) {}

  void reserve(std::size_t size) noexcept {
    this->ports_.reserve(size);
    this->present_ports_reserve(size);
  }

  void push_back(const T&& elem) noexcept {
    this->ports_.push_back(std::move(elem));
    this->register_port(this->ports_.back(), this->ports_.size() - 1);
  }

  void pop_back() {
    this->ports_.pop_back();
    this->present_ports_pop_back();
  }

  template <class... Args> void emplace_back(Args&&... args) noexcept {
    this->ports_.emplace_back(std::forward<Args>(args)...);
    this->register_port(this->ports_.back(), this->ports_.size() - 1);
  }
};
} // namespace reactor

#endif // REACTOR_CPP_MULTIPORT_HH
