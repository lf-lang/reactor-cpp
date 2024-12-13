//
// Created by tanneberger on 11/18/24.
//

#ifndef MUTATION_BANK_HH
#define MUTATION_BANK_HH

#include <vector>

#include "../mutations.hh"
#include "../reactor.hh"

namespace reactor {
class Reactor;
class Environment;

template <class T> class MutationChangeBankSize : public reactor::Mutation {
  std::vector<T>* bank_ = nullptr;
  std::size_t desired_size_ = 0;
  std::size_t size_before_application_ = 0;
  Reactor* reactor_ = nullptr;
  std::function<T(Reactor* reactor, std::size_t index)> create_lambda_;

  void change_size(std::size_t);

public:
  MutationChangeBankSize(std::vector<T>* bank, Reactor* reactor, std::size_t size,
                         std::function<T(Reactor* reactor, std::size_t index)>);
  MutationChangeBankSize() = default;
  explicit MutationChangeBankSize(const std::vector<T>& other)
      : bank_(other.bank_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_) {}
  ~MutationChangeBankSize() override = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // MUTATION_BANK_HH
