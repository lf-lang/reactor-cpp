//
// Created by tanneberger on 11/11/24.
//

#ifndef MUTATION_MULTIPORT_HH
#define MUTATION_MULTIPORT_HH

#include <vector>

#include "../multiport.hh"
#include "../mutations.hh"
#include "../port.hh"

namespace reactor {
class Reactor;
class Environment;

template <class T> class MutationChangeOutputMultiportSize : public Mutation {
private:
  ModifableMultiport<Output<T>>* multiport_ = nullptr;
  std::set<Reaction*> anti_dependencies_{};
  std::size_t desired_size_ = 0;
  std::size_t size_before_application_ = 0;
  Reactor* reactor_{};

  void change_size(std::size_t);

public:
  MutationChangeOutputMultiportSize(ModifableMultiport<Output<T>>* multiport, Reactor* reactor,
                                    std::set<Reaction*>& anti_dependencies, std::size_t size);
  MutationChangeOutputMultiportSize() = default;
  MutationChangeOutputMultiportSize(const MutationChangeOutputMultiportSize& other)
      : multiport_(other.multiport_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_) {}
  ~MutationChangeOutputMultiportSize() override = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // MUTATION_MULTIPORT_HH
