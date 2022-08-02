/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MULTIPORT_CALLBACK_HH
#define REACTOR_CPP_MULTIPORT_CALLBACK_HH

#include <algorithm>
#include <atomic>
#include <iostream>
#include <type_traits>
#include <vector>

namespace multiport {

// fancy custom type_trait taken from stackoverflow 
// which checks if given class has the member function deactivate
template <typename T>
class has_deactivate
{
    using one = char; 
    struct two { char x[2]; }; //NOLINT modernize-use-using

    template <typename C> static auto test( decltype(&C::has_deactivate) ) -> one;
    template <typename C> static auto test(...) -> two;    

public:
    enum { value = sizeof(test<T>(0)) == sizeof(char) };
};

// struct which gets handed to the ports to they can talk back
// to the portbank
struct LockedPortList {
    std::atomic<std::size_t>* size_ = nullptr;
    std::vector<std::size_t>* active_ports_ = nullptr;
};

template <class T, class A = std::allocator<T>>
class PortBankCallBack { //NOLINT cppcoreguidelines-special-member-functions
private:
  std::vector<T> data_{};
  std::vector<std::size_t> active_ports_{};
  std::atomic<std::size_t> size_ = 0;

public:
  using allocator_type = A;
  using value_type = typename A::value_type;
  using reference = typename A::reference;
  using const_reference = typename A::const_reference;
  using difference_type = typename A::difference_type;
  using size_type = typename A::size_type;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  PortBankCallBack() noexcept = default;

  ~PortBankCallBack() noexcept {
    // we need to tell all the connected ports that this portbank is freed
    if constexpr (std::is_pointer<T>::value) {
      if constexpr (!has_deactivate<std::remove_pointer_t<T>>::value) {
        for (auto i = 0; i < data_.size(); i++) {
          data_[i]->deactivate();
        }
      }
    } else {
      if constexpr (has_deactivate<T>::value) {
        for (auto i = 0; i < data_.size(); i++) {
            data_[i].deactivate();
        }
      }
    }
  };

  auto operator==(const PortBankCallBack& other) const noexcept -> bool {
    return std::equal(std::begin(data_), std::end(data_), std::begin(other.data_), std::end(other.data_));
  }
  auto operator!=(const PortBankCallBack& other) const noexcept -> bool { return !(*this == other); };
  inline auto operator[](std::size_t index) noexcept -> T& { return data_[index]; }
  inline auto operator[](std::size_t index) const noexcept -> const T& { return data_[index]; }

  inline auto begin() noexcept -> iterator { return data_.begin(); };
  inline auto begin() const noexcept -> const_iterator { return data_.begin(); };
  inline auto cbegin() const noexcept -> const_iterator { return data_.cbegin(); };
  inline auto end() noexcept -> iterator { return data_.end(); };
  inline auto end() const noexcept -> const_iterator { return data_.end(); };
  inline auto cend() const noexcept -> const_iterator { return data_.cend(); };

  inline void swap(PortBankCallBack& other) { std::swap(data_, other.data_); };
  inline auto size() const noexcept -> size_type { return data_.size(); };
  inline auto max_size() const noexcept -> size_type { return data_.size(); };
  [[nodiscard]] inline auto empty() const noexcept -> bool { return data_.empty(); };

  [[nodiscard]] inline auto get_active_ports() noexcept -> LockedPortList {
    return LockedPortList {
        &size_,
        &active_ports_
    };
  }

  inline void reserve(std::size_t size) noexcept {
    data_.reserve(size);
    active_ports_.reserve(size * 2);
  }

  inline void push_back(const T& elem) noexcept {
    data_.push_back(elem);
  }

  template <class... Args>
  inline void emplace_back(Args&&... args) noexcept {
    data_.emplace_back(args...);
  }

  template <class... Args>
  inline void set(std::size_t index, Args&&... args) noexcept {
    data_[index].set(args...);
  }

  [[nodiscard]] inline auto active_ports_indices() const noexcept -> std::vector<std::size_t> { 
    std::vector<std::size_t>ports_copy;
    ports_copy.reserve(active_ports_.capacity());

    auto not_contains = [&](std::size_t index) {
        return std::find(std::begin(ports_copy),std::end(ports_copy), index) == std::end(ports_copy);
    };

    {
        std::size_t size = size_.load();

        for (auto i = 0; i < size; i++) {
            auto j = active_ports_[i];
            if (data_[j].is_present() && not_contains(j)) {
                ports_copy.push_back(j);
            }
        }

        // this is just ugly and higly dangerous 
        // but other wise I would need to strip out all the const qualifierts
        ((PortBankCallBack*)this)->size_ = ports_copy.size();
        ((PortBankCallBack*)this)->active_ports_ = std::move(ports_copy);    
    }

    return ports_copy; 
  }
};
} // namespace multiport

#endif // REACTOR_CPP_MULTIPORT_CALLBACK_HH


