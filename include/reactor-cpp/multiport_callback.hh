//https://stackoverflow.com/questions/7758580/writing-your-own-stl-container/7759622#7759622

#ifndef REACTOR_CPP_MULTIPORT_CALLBACK_HH
#define REACTOR_CPP_MULTIPORT_CALLBACK_HH

#include "portbank.hh"

#include <iostream>
#include <array>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <algorithm>
#include <type_traits>
#include <mutex>

namespace multiport {

template <typename T>
class has_deactivate
{
    typedef char one;
    struct two { char x[2]; };

    template <typename C> static one test( decltype(&C::has_deactivate) ) ;
    template <typename C> static two test(...);    

public:
    enum { value = sizeof(test<T>(0)) == sizeof(char) };
};
   

struct LockedPortList {
    std::mutex* mutex_ = nullptr;
    std::vector<std::size_t>* active_ports_ = nullptr;
};

template <class T, class A = std::allocator<T>>
class PortBankCallBack { 
private:
  std::vector<T> data_{};
  std::mutex mutex_;
  std::vector<std::size_t> active_ports_{};

public:
  using allocator_type = A;
  using value_type = typename A::value_type;
  using reference = typename A::reference;
  using const_reference = typename A::const_reference;
  using difference_type = typename A::difference_type;
  using size_type = typename A::size_type;

  //using iterator = typename std::array<T, Size>::iterator;
  //using const_iterator = typename std::array<T, Size>::const_iterator;
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  PortBankCallBack() {};

  ~PortBankCallBack() {
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

  auto operator==(const PortBankCallBack& other) const -> bool {
    return std::equal(std::begin(data_), std::end(data_), std::begin(other.data_), std::end(other.data_));
  }
  auto operator!=(const PortBankCallBack& other) const -> bool { return !(*this == other); };
  auto operator[](std::size_t index) -> T& { return data_[index]; }
  auto operator[](std::size_t index) const -> const T& { return data_[index]; }

  auto begin() -> iterator { return data_.begin(); };
  auto begin() const -> const_iterator { return data_.begin(); };
  auto cbegin() const -> const_iterator { return data_.cbegin(); };
  auto end() -> iterator { return data_.end(); };
  auto end() const -> const_iterator { return data_.end(); };
  auto cend() const -> const_iterator { return data_.cend(); };

  void swap(PortBankCallBack& other) { std::swap(data_, other.data_); };
  auto size() const noexcept -> size_type { return data_.size(); };
  auto max_size() const noexcept -> size_type { return data_.size(); };
  [[nodiscard]] auto empty() const -> bool { return data_.empty(); };
  
  inline auto get_active_ports() -> LockedPortList {
    return LockedPortList {
        &mutex_,
        &active_ports_
    }; 
  } 

  inline void reserve(std::size_t size) noexcept {
    data_.reserve(size);
    active_ports_.reserve(size);
  }

  inline void push_back(const T& elem) noexcept {
    data_.push_back(elem);
  }

  template <class... Args>
  inline void emplace_back(Args&&... args) noexcept  {
    data_.emplace_back(args...);
  }

  void print() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i: active_ports_) {
        std::cout << i << ", ";
    }
    std::cout << std::endl;
  }

  template <class... Args>
  inline void set(std::size_t index, Args&&... args) noexcept {
    data_[index].set(args...);
  }

  inline auto active_ports_indices() const noexcept -> std::vector<std::size_t> { 
    std::lock_guard<std::mutex> lock(((PortBankCallBack*)this)->mutex_);
    std::vector<std::size_t>ports_copy;
    ports_copy.reserve(active_ports_.size() / 2);

    auto not_contains = [&](std::size_t index) {
        return std::find(std::begin(ports_copy),std::end(ports_copy), index) == std::end(ports_copy);
    };

    for (auto i : active_ports_) {
        if (data_[i].is_present() && not_contains(i)) {
            ports_copy.push_back(i);
        }
    }

    ((PortBankCallBack*)this)->active_ports_ = ports_copy;
    return ports_copy; 
  }
};
}

#endif //REACTOR_CPP_MULTIPORT_CALLBACK_HH

