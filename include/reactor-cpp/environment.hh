/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_ENVIRONMENT_HH
#define REACTOR_CPP_ENVIRONMENT_HH

#include <set>
#include <string>
#include <vector>

#include "connection_properties.hh"
#include "fwd.hh"
#include "graph.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/time.hh"
#include "scheduler.hh"

namespace reactor {

constexpr unsigned int default_number_worker = 1;
constexpr unsigned int default_max_reaction_index = 0;
constexpr bool default_run_forever = false;
constexpr bool default_fast_fwd_execution = false;

enum class Phase { Construction = 0, Assembly = 1, Startup = 2, Execution = 3, Shutdown = 4, Deconstruction = 5 };

using GraphElement = BasePort*;

class Environment {
private:
  using Dependency = std::pair<Reaction*, Reaction*>;

  // name of the environment for debugging
  const std::string name_{};

  // logger
  const log::NamedLogger log_;

  // parameters
  const unsigned int num_workers_{default_number_worker};
  unsigned int max_reaction_index_{default_max_reaction_index};
  bool run_forever_{default_run_forever};
  const bool fast_fwd_execution_{default_fast_fwd_execution};

  // reactors that this program consists of
  std::set<Reactor*> top_level_reactors_{};

  // all reactions that exists in this program
  std::set<Reaction*> reactions_{};

  // this is how the reaction graph is represented
  std::vector<Dependency> dependencies_{};

  // port component bucket
  std::vector<class BasePort*> ports_{};

  // Set of actions that act as an input to the reactor program in this environment
  std::set<BaseAction*> input_actions_{};

  // The environment containing this environment. nullptr if this is the top environment
  Environment* containing_environment_{nullptr};
  // Set of all environments contained by this environment
  std::set<Environment*> contained_environments_{};
  // Pointer to the top level environment
  Environment* top_environment_{nullptr};

  // scheduler used for execution
  Scheduler scheduler_;

  // phase of the program
  Phase phase_{Phase::Construction};

  // start time of execution
  Tag start_tag_{};

  const Duration timeout_{};

  // mutex that coordinates the global shut down over multiple workers
  std::mutex shutdown_mutex_{};

  Graph<GraphElement, ConnectionProperties> graph_{};
  Graph<GraphElement, ConnectionProperties> optimized_graph_{};

  std::size_t tag_counter_{0};

  auto startup(const TimePoint& start_time) -> std::thread;

public:
  explicit Environment(unsigned int num_workers, bool fast_fwd_execution = default_fast_fwd_execution,
                       const Duration& timeout = Duration::max());
  explicit Environment(const std::string& name, Environment* containing_environment);

  // just returns the name of this environment
  auto name() -> const std::string& { return name_; }

  // this function add components to the reactor graph
  auto register_port(class BasePort* port) noexcept -> std::size_t;

  void register_reaction(Reaction* reaction) noexcept { reactions_.insert(reaction); }

  // this method draw a connection between two graph elements with some properties
  void draw_connection(std::size_t source, std::size_t sink, ConnectionProperties properties);
  void draw_connection(const BasePort& source, const BasePort& sink, ConnectionProperties properties);
  void draw_connection(const BasePort* source, const BasePort* sink, ConnectionProperties properties);

  void insert_reactor(Reactor* reactor);
  void insert_input_action(BaseAction* action);

  void assemble();
  void optimize();
  void calculate_indices();
  void build_dependency_graph(Reactor* reactor);

  auto startup() -> std::thread;
  void sync_shutdown();
  void async_shutdown();

  [[nodiscard]] auto top_level_reactors() const noexcept -> const auto& { return top_level_reactors_; }

  [[nodiscard]] auto phase() const noexcept -> Phase { return phase_; }
  [[nodiscard]] auto scheduler() const noexcept -> const Scheduler* { return &scheduler_; }

  auto scheduler() noexcept -> Scheduler* { return &scheduler_; }

  [[nodiscard]] auto logical_time() const noexcept -> const LogicalTime& { return scheduler_.logical_time(); }
  [[nodiscard]] auto start_tag() const noexcept -> const Tag& { return start_tag_; }
  [[nodiscard]] auto timeout() const noexcept -> const Duration& { return timeout_; }

  static auto physical_time() noexcept -> TimePoint { return get_physical_time(); }

  [[nodiscard]] auto num_workers() const noexcept -> unsigned int { return num_workers_; }
  [[nodiscard]] auto fast_fwd_execution() const noexcept -> bool { return fast_fwd_execution_; }

  [[nodiscard]] auto run_forever() const noexcept -> bool { return run_forever_; }
  [[nodiscard]] auto max_reaction_index() const noexcept -> unsigned int { return max_reaction_index_; }

  // Debugging methods
  void export_dependency_graph(const std::string& path);
  void dump_to_yaml(const std::string& path);

  static void dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger);
  static void dump_instance_to_yaml(std::ofstream& yaml, const Reactor& reactor);
  static void dump_port_to_yaml(std::ofstream& yaml, const BasePort& port);
  static void dump_reaction_to_yaml(std::ofstream& yaml, const Reaction& reaction);

  friend Scheduler;
};
} // namespace reactor

#endif // REACTOR_CPP_ENVIRONMENT_HH
