//
// Created by tanneberger on 11/18/24.
//

#ifndef REACTOR_CPP_MUTATIONS_BANK_HH
#define REACTOR_CPP_MUTATIONS_BANK_HH

#include <vector>

#include "../mutations.hh"
#include "../reactor.hh"

namespace reactor {
class Reactor;
class Environment;

template <class T> class MutationChangeBankSize final : public Mutation {
  std::vector<T>* bank_ = nullptr;
  std::size_t desired_size_ = 0;
  std::size_t size_before_application_ = 0;
  Reactor* reactor_ = nullptr;
  std::function<T(Reactor* parent_reactor, std::size_t index)> create_lambda_;

  void change_size(std::size_t new_size);

public:
  MutationChangeBankSize() = default;
  MutationChangeBankSize(const MutationChangeBankSize& other) noexcept
      : bank_(other.bank_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_)
      , reactor_(other.reactor_)
      , create_lambda_(other.create_lambda_){};
  MutationChangeBankSize(MutationChangeBankSize&& other) noexcept
      : bank_(other.bank_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_)
      , reactor_(other.reactor_)
      , create_lambda_(other.create_lambda_){};
  explicit MutationChangeBankSize(std::vector<T>* bank, Reactor* reactor, std::size_t size,
                                  std::function<T(Reactor* parent_reactor, std::size_t index)> create_lambda);
  ~MutationChangeBankSize() override = default;
  auto operator=(const MutationChangeBankSize& other) -> MutationChangeBankSize& = default;
  auto operator=(MutationChangeBankSize&& other) -> MutationChangeBankSize& = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_BANK_HH
