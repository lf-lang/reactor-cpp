#ifndef MUTATIONS_HH
#define MUTATIONS_HH

namespace reactor {
class Reactor;
class Environment;

enum MutationResult {
  Success = 0,
  NotMatchingBankSize = 1,
};

class Mutation {
public:
  virtual ~Mutation() = default;
  virtual auto run() -> MutationResult = 0;
  virtual auto rollback() -> MutationResult = 0;
};

} // namespace reactor

#endif // MUTATIONS_HH
