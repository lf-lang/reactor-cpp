//
// Created by tanneberger on 2/10/25.
//

#ifndef TOPOLOGY_LOCK_HH
#define TOPOLOGY_LOCK_HH

#include "./mutations.hh"
#include "./reaction.hh"
#include <vector>

namespace reactor {

/**
 * @brief This lock tracks the running mutations and prohibits access to parts of the toplogy
 */
class TopologyLock {
private:
  std::vector<Mutation*> running_mutations_;

public:
  TopologyLock() = default;
  ~TopologyLock() = default;

  void start_mutation(Mutation* mutation) noexcept;
  void end_mutatuon(Mutation* mutation) noexcept;

  auto can_execute(Mutation* mutation) -> bool;
  auto can_execute(Reaction* reaction) -> bool;
};
} // namespace reactor

#endif // TOPOLOGY_LOCK_HH
