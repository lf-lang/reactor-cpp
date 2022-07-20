//https://stackoverflow.com/questions/7758580/writing-your-own-stl-container/7759622#7759622

#include <iostream>
#include <array>
#include <memory>
#include <unordered_set>
#include <vector>
#include <chrono>

namespace callback {

template <class T, class A = std::allocator<T>> class PortBankCallBack {
private:
  std::vector<T> data_{};
  std::vector<std::size_t> active_ports_{};

public:
  typedef A allocator_type;
  typedef typename A::value_type value_type;
  typedef typename A::reference reference;
  typedef typename A::const_reference const_reference;
  typedef typename A::difference_type difference_type;
  typedef typename A::size_type size_type;

  //using iterator = typename std::array<T, Size>::iterator;
  //using const_iterator = typename std::array<T, Size>::const_iterator;
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  PortBankCallBack() {
    for (auto i = 0; i < data_.size(); i++) {
      data_[i] = T{this};
    }
  };
  PortBankCallBack(const PortBankCallBack&);
  ~PortBankCallBack() = default;

  auto operator=(const PortBankCallBack<T>& other) -> PortBankCallBack& {
    std::copy(std::begin(other), std::end(other), std::begin(data_));
    return *this;
  }
  auto operator==(const PortBankCallBack& other) const -> bool {
    return std::equal(std::begin(data_), std::end(data_), std::begin(other.data_), std::end(other.data_));
  }
  auto operator!=(const PortBankCallBack& other) const -> bool { return !(*this == other); };
  auto operator[](std::size_t index) -> T& { return data_[index]; }
  auto operator[](std::size_t index) const -> T { return data_[index]; }
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

  void update_active_ports() {
    active_ports_.clear();

    for (auto i = 0ul; i < active_ports_.size(); i++) {
      if ((*this)[i].set_) {
        active_ports_.push_back(i);
      }
    }
  }

  inline auto active_ports_indices() const noexcept -> const std::vector<std::size_t>& { return active_ports_; }

  auto active_ports() const noexcept -> std::vector<T> {
    std::vector<T> collection{};
    collection.reserve(active_ports_.size());

    for (std::size_t index : active_ports_) {
      collection.push_back(data_[index]);
    }

    return collection;
  }

  void port_call_back(T* port) {
    auto index = static_cast<std::size_t>(port - (&*std::begin(data_)));
    active_ports_.push_back(index);
  }
};
}