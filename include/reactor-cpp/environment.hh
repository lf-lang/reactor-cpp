/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ENVIRONMENT_HH
#define REACTOR_CPP_ENVIRONMENT_HH

#include <set>
#include <string>
#include <vector>

#include "reactor.hh"
#include "scheduler.hh"

namespace reactor {

constexpr unsigned int default_number_worker = 1;
constexpr unsigned int default_max_reaction_index = 0;
constexpr bool default_run_forever = false;
constexpr bool default_fast_fwd_execution = false;

class Environment {
public:
  enum class Phase { Construction = 0, Assembly = 1, Startup = 2, Execution = 3, Shutdown = 4, Deconstruction = 5 };

private:
  using Dependency = std::pair<Reaction*, Reaction*>;
  const unsigned int num_workers_{default_number_worker};
  unsigned int max_reaction_index_{default_max_reaction_index};
  const bool run_forever_{default_run_forever};
  const bool fast_fwd_execution_{default_fast_fwd_execution};

  std::set<Reactor*> top_level_reactors_{};
  std::set<Reaction*> reactions_{};
  std::vector<Dependency> dependencies_{};

  Scheduler scheduler_;
  Phase phase_{Phase::Construction};
  TimePoint start_time_{};

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

public:
  explicit Environment(unsigned int num_workers, bool run_forever = default_run_forever,
                       bool fast_fwd_execution = default_fast_fwd_execution)
      : num_workers_(num_workers)
      , run_forever_(run_forever)
      , fast_fwd_execution_(fast_fwd_execution)
      , scheduler_(this) {}

  void register_reactor(Reactor* reactor);
  void assemble();
  auto startup() -> std::thread;
  void sync_shutdown();
  void async_shutdown();

  // Debugging methods
  void export_dependency_graph(const std::string& path);
  void dump_to_yaml(const std::string& path);

  static void dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger);
  static void dump_instance_to_yaml(std::ofstream& yaml, const Reactor& reactor);
  static void dump_port_to_yaml(std::ofstream& yaml, const BasePort& port);
  static void dump_reaction_to_yaml(std::ofstream& yaml, const Reaction& reaction);

  [[nodiscard]] auto top_level_reactors() const noexcept -> const auto& { return top_level_reactors_; }
  [[nodiscard]] auto phase() const noexcept -> Phase { return phase_; }
  [[nodiscard]] auto scheduler() const noexcept -> const Scheduler* { return &scheduler_; }

  auto scheduler() noexcept -> Scheduler* { return &scheduler_; }

  [[nodiscard]] auto logical_time() const noexcept -> const LogicalTime& { return scheduler_.logical_time(); }
  [[nodiscard]] auto start_time() const noexcept -> const TimePoint& { return start_time_; }

  static auto physical_time() noexcept -> TimePoint { return get_physical_time(); }

  [[nodiscard]] auto num_workers() const noexcept -> unsigned int { return num_workers_; }
  [[nodiscard]] auto fast_fwd_execution() const noexcept -> bool { return fast_fwd_execution_; }
  [[nodiscard]] auto run_forever() const noexcept -> bool { return run_forever_; }
  [[nodiscard]] auto max_reaction_index() const noexcept -> unsigned int { return max_reaction_index_; }
};
} // namespace reactor

#endif // REACTOR_CPP_ENVIRONMENT_HH
