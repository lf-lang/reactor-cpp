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
protected:
  std::atomic<std::size_t> size_{0};         // NOLINT cppcoreguidelines-non-private-member-variables-in-classes
  std::vector<std::size_t> present_ports_{}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

private:
  std::string name_{};
  Reactor* container_ = nullptr;

  // record that the port with the given index has been set
  void set_present(std::size_t index);

  // reset the list of set port indexes
  void reset() noexcept { size_.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto get_set_callback(std::size_t index) noexcept -> PortCallback;
  const PortCallback clean_callback_{[this]([[maybe_unused]] const BasePort& port) { this->reset(); }};

  [[nodiscard]] auto get_clean_callback() const noexcept -> const PortCallback& { return clean_callback_; }

protected:
  [[nodiscard]] auto present_ports() const -> const auto& { return present_ports_; }
  [[nodiscard]] auto present_ports_size() const -> auto { return size_.load(); }

  void present_ports_reserve(size_t n) { present_ports_.reserve(n); }

  void register_port(BasePort& port, size_t idx);

public:
  BaseMultiport(std::string name, Reactor* container)
      : name_(std::move(name))
      , container_(container) {}
  ~BaseMultiport() = default;

  [[nodiscard]] auto name() const noexcept -> const std::string& { return name_; }
  [[nodiscard]] auto container() const noexcept -> Reactor* { return container_; }
};

template <class T, class A = std::allocator<T>>
class Multiport : public BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions
protected:
  std::vector<T> ports_{}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  template <class... Args> void emplace_back(Args&&... args) noexcept {
    static_assert(std::is_constructible_v<T, Args...>);
    ports_.emplace_back(std::forward<Args>(args)...);
    register_port(ports_.back(), ports_.size() - 1);
  }

public:
  using value_type = typename A::value_type;
  using size_type = typename A::size_type;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  Multiport(const std::string& name, Reactor* container) noexcept
      : BaseMultiport(name, container) {}
  ~Multiport() noexcept = default;

  auto operator==(const Multiport& other) const noexcept -> bool {
    return std::equal(std::begin(ports_), std::end(ports_), std::begin(other.ports_), std::end(other.ports_));
  }
  auto operator!=(const Multiport& other) const noexcept -> bool { return !(*this == other); };
  auto operator[](std::size_t index) noexcept -> T& { return ports_[index]; }
  auto operator[](std::size_t index) const noexcept -> const T& { return ports_[index]; }

  auto begin() noexcept -> iterator { return ports_.begin(); };
  auto begin() const noexcept -> const_iterator { return ports_.begin(); };
  auto cbegin() const noexcept -> const_iterator { return ports_.cbegin(); };
  auto end() noexcept -> iterator { return ports_.end(); };
  auto end() const noexcept -> const_iterator { return ports_.end(); };
  auto cend() const noexcept -> const_iterator { return ports_.cend(); };

  auto size() const noexcept -> size_type { return ports_.size(); };
  [[nodiscard]] auto empty() const noexcept -> bool { return ports_.empty(); };
  void resize(std::size_t new_size) {
    reactor_assert(size() >= new_size);

    for (auto i = size(); i > new_size; i--) {
      ports_.pop_back();
      present_ports_.pop_back();
    }
  }

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
  ModifableMultiport(std::string name, Reactor* container)
      : Multiport<T, A>(name, container) {}

  void reserve(std::size_t size) noexcept {
    this->ports_.reserve(size);
    this->present_ports_reserve(size);
  }

  void push_back(const T& elem) noexcept {
    this->ports_.push_back(elem);
    this->register_port(this->ports_.back(), this->ports_.size() - 1);
  }

  void create_new_port() noexcept {
    std::string _lf_port_name = this->name() + "_" + std::to_string(this->size());
    Reactor* container = this->container();
    this->emplace_back(_lf_port_name, container);
  }
};
} // namespace reactor

#endif // REACTOR_CPP_MULTIPORT_HH
