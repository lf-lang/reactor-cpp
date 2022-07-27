//
// Created by revol-xut on 7/24/22.
//

#ifndef REACTOR_CPP_PORTBANK_HH
#define REACTOR_CPP_PORTBANK_HH

#include <memory>

namespace multiport {

template <class T, class iterator, class const_iterator, class A = std::allocator<T>>
class PortBank {
public:
  using allocator_type = A;
  using value_type = typename A::value_type;
  using reference = typename A::reference;
  using const_reference = typename A::const_reference;
  using difference_type = typename A::difference_type;
  using size_type = typename A::size_type;

  virtual auto operator=(const PortBank& other) -> PortBank& = 0;
  virtual auto operator==(const PortBank& other) const -> bool  = 0;
  virtual auto operator!=(const PortBank& other) const -> bool = 0;
  virtual auto operator[](std::size_t index) -> T& = 0;
  virtual auto operator[](std::size_t index) const -> T = 0;
  virtual auto begin() -> iterator = 0;
  virtual auto begin() const -> const_iterator = 0;
  virtual auto cbegin() const -> const_iterator = 0;
  virtual auto end() -> iterator = 0;
  virtual auto end() const -> const_iterator = 0;
  virtual auto cend() const -> const_iterator = 0;

  //void swap(PortBank& other) { std::swap(data_, other.data_); };
  virtual auto size() const noexcept -> size_type = 0;
  virtual auto max_size() const noexcept -> size_type = 0;
  [[nodiscard]] virtual auto empty() const -> bool = 0;
};
}


#endif // REACTOR_CPP_PORTBANK_HH
