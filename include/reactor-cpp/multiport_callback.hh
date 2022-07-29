//https://stackoverflow.com/questions/7758580/writing-your-own-stl-container/7759622#7759622

#include "portbank.hh"

#include <iostream>
#include <array>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <algorithm>

namespace multiport {

template <class T, class A = std::allocator<T>>
class PortBankCallBack { 
private:
  std::vector<T> data_{};
  std::vector<std::size_t> active_ports_{};
  PortBankCallBack<T, A>* right_site_ {nullptr}; 

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

  PortBankCallBack() {
    //for (auto i = 0; i < data_.size(); i++) {
    //  data_[i] = T{this};
    //}
  };
  PortBankCallBack(const PortBankCallBack&);
  ~PortBankCallBack() = default;

  auto operator=(const PortBankCallBack<T>& other) -> PortBankCallBack& {
    std::copy(std::begin(other.data_), std::end(other.data_), std::begin(data_));
    std::copy(std::begin(other.active_ports_), std::end(other.active_ports_), std::begin(active_ports_));
    return *this;
  }
  auto operator==(const PortBankCallBack& other) const -> bool {
    return std::equal(std::begin(data_), std::end(data_), std::begin(other.data_), std::end(other.data_));
  }
  auto operator!=(const PortBankCallBack& other) const -> bool { return !(*this == other); };
  auto operator[](std::size_t index) -> T& { 
      active_ports_.push_back(index);
      return data_[index]; 
  }
  auto operator[](std::size_t index) const -> const T& { 
      return data_[index]; 
  }

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
  
  inline void set_right_site(PortBankCallBack<T, A>* right_site) noexcept {
    right_site_ = right_site;
  }

  inline void reserve(std::size_t size) noexcept {
    data_.reserve(size);
    active_ports_.reserve(size);
  }

  inline void push_back(const T& elem) noexcept {
    if (elem->is_present()){
        active_ports_.push_back(active_ports_.size());
    }
    data_.push_back(elem);
  }

  template <class... Args>
  inline void emplace_back(Args&&... args) noexcept  {
    data_.emplace_back(args...);
  }

  template <class... Args>
  inline void set(std::size_t index, Args&&... args) noexcept {
    active_ports_.push_back(index);
    std::cout << "setting: " << index << " filled: " << active_ports_.size() << std::endl;
    data_[index].set(args...);
  }

  inline void activate(T* port) noexcept {
    auto position = (port - data_.data()) / sizeof(T);
    active_ports_.push_back(position);
  }

  inline void filter() noexcept {
    active_ports_.erase(
        std::remove_if(active_ports_.begin(), active_ports_.end(),
            [&](std::size_t index) { return !data_[index].is_present(); }),
            active_ports_.end());
  }

  void update_active_ports() {
    active_ports_.clear();

    for (auto i = 0ul; i < active_ports_.size(); i++) {
      if ((*this)[i].set_) {
        active_ports_.push_back(i);
      }
    }
  }

  inline auto active_ports_indices() const noexcept -> std::vector<std::size_t> { 
    std::vector<std::size_t>ports_copy;
    ports_copy.reserve(active_ports_.size() / 2);

    auto not_contains = [&](std::size_t index) {
        return std::find(std::begin(ports_copy),std::end(ports_copy), index) == std::end(ports_copy);
    };

    std::cout << "size befor: " << active_ports_.size() << std::endl;
    for (auto i : active_ports_) {
        if (data_[i].is_present() && not_contains(i)) {
            ports_copy.push_back(i);
        }
    }
    std::cout << "size after: " << ports_copy.size() << std::endl;
    //((PortBankCallBack*)this)->active_ports_ = ports_copy;
    //std::cout << "cleared: " << active_ports_.size() << std::endl;
    return ports_copy; 
  }

  auto active_ports() const noexcept -> std::vector<T> {
    std::vector<T> collection{};
    collection.reserve(active_ports_.size());

    for (std::size_t index : active_ports_) {
      collection.push_back(data_[index]);
    }

    return collection;
  }

};
}
