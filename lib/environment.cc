/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/environment.hh"

#include <algorithm>
#include <fstream>
#include <map>
#include <thread>
#include <vector>

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/time.hh"

namespace reactor {

Environment::Environment(unsigned int num_workers, bool fast_fwd_execution, const Duration& timeout)
    : log_("Environment")
    , num_workers_(num_workers)
    , fast_fwd_execution_(fast_fwd_execution)
    , top_environment_(this)
    , scheduler_(this)
    , timeout_(timeout) {}

Environment::Environment(const std::string& name, Environment* containing_environment)
    : name_(name)
    , log_("Environment " + name)
    , num_workers_(containing_environment->num_workers_)
    , fast_fwd_execution_(containing_environment->fast_fwd_execution_)
    , containing_environment_(containing_environment)
    , top_environment_(containing_environment_->top_environment_)
    , scheduler_(this)
    , timeout_(containing_environment->timeout()) {
  [[maybe_unused]] bool result = containing_environment->contained_environments_.insert(this).second;
  reactor_assert(result);
}

void Environment::insert_reactor(Reactor* reactor) {
  reactor_assert(reactor != nullptr);
  validate(this->phase() == Phase::Construction, "Reactors may only be registered during construction phase!");
  validate(reactor->is_top_level(), "The environment may only contain top level reactors!");
  [[maybe_unused]] bool result = top_level_reactors_.insert(reactor).second;
  reactor_assert(result);
}

void Environment::insert_input_action(BaseAction* action) {
  reactor_assert(action != nullptr);
  validate(this->phase() == Phase::Construction || this->phase() == Phase::Assembly,
           "Input actions may only be registered during construction or assembly phase!");
  [[maybe_unused]] bool result = input_actions_.insert(action).second;
  reactor_assert(result);
  run_forever_ = true;
}

void Environment::sync_shutdown() {
  {
    std::lock_guard<std::mutex> lock{shutdown_mutex_};

    if (phase_ >= Phase::Shutdown) {
      // sync_shutdown() was already called -> abort
      return;
    }

    validate(phase_ == Phase::Execution, "sync_shutdown() may only be called during execution phase!");
    phase_ = Phase::Shutdown;
  }

  // the following will only be executed once
  log_.debug() << "Terminating the execution";

  for (auto* reactor : top_level_reactors_) {
    reactor->shutdown();
  }

  phase_ = Phase::Deconstruction;
  scheduler_.stop();
}

void Environment::async_shutdown() {
  auto lock_guard = scheduler_.lock();
  sync_shutdown();
}

auto Environment::startup() -> std::thread {
  validate(this == top_environment_, "startup() may only be called on the top environment");
  return startup(get_physical_time());
}

auto Environment::startup(const TimePoint& start_time) -> std::thread {
  validate(this->phase() == Phase::Assembly, "startup() may only be called during assembly phase!");

  log_.debug() << "Starting the execution";
  phase_ = Phase::Startup;

  this->start_tag_ = Tag::from_physical_time(start_time);
  // start up initialize all reactors

  for (auto* reactor : top_level_reactors_) {
    reactor->startup();
  }

  // start processing events
  phase_ = Phase::Execution;

  return std::thread([this, start_time]() {
    std::vector<std::thread> threads;
    threads.reserve(contained_environments_.size());
    // startup all contained environments recursively
    for (auto* env : contained_environments_) {
      threads.emplace_back(env->startup(start_time));
    }
    // start the local scheduler and wait until it returns
    this->scheduler_.start();

    // then join all the created threads
    for (auto& thread : threads) {
      thread.join();
    }
  });
}
void Environment::draw_connection(std::size_t source, std::size_t sink, ConnectionProperties properties) {
  reactor::assert_phase(this, Phase::Assembly);

  if (top_environment_ == nullptr || top_environment_ == this) {
    log::Debug() << "drawing connection: " << source << " --> " << sink;
    graph_.add_edge(source, sink, properties);
  } else {
    top_environment_->draw_connection(source, sink, properties);
  }
}

void Environment::draw_connection(const BasePort& source, const BasePort& sink, ConnectionProperties properties) {
  this->draw_connection(source.get_index(), sink.get_index(), properties);
}

void Environment::draw_connection(const BasePort* source, const BasePort* sink, ConnectionProperties properties) {
  this->draw_connection(source->get_index(), sink->get_index(), properties);
}

auto Environment::register_port(BasePort* port) noexcept -> std::size_t {
  if (top_environment_ == nullptr || top_environment_ == this) {
    auto index = ports_.size();
    port->set_index(index);
    ports_.push_back(port);
    return index;
  }

  return top_environment_->register_port(port);
}

void recursive_assemble(Reactor* container) { // NOLINT
  container->assemble();

  for (auto* reactor : container->reactors()) {
    recursive_assemble(reactor);
  }
}
void Environment::assemble() {
  phase_ = Phase::Assembly;

  // constructing all the reactors
  // this mainly tell the reactors that they should connect their ports and actions not ports and ports

  log::Debug() << "start assembly of reactors";
  for (auto* reactor : top_level_reactors_) {
    recursive_assemble(reactor);
  }

  log::Debug() << "start optimization on port graph";
  this->optimize();

  log::Debug() << "instantiating port graph declaration";

  if (top_environment_ == nullptr || top_environment_ == this) {
    log::Debug() << "graph: ";
    log::Debug() << optimized_graph_;

    auto graph = optimized_graph_.get_edges();
    // this generates the port graph
    for (auto const& [source, sinks] : graph) {

      auto source_port = source.first;
      auto properties = source.second;

      if (properties.type_ == ConnectionType::Normal) {
        for (const auto destination_port : sinks) {
          ports_[destination_port]->set_inward_binding(ports_[source_port]);
          ports_[source_port]->add_outward_binding(ports_[destination_port]);
          log::Debug() << "from: " << ports_[source_port]->fqn() << "(" << source_port << ")"
                       << " --> to: " << ports_[destination_port]->fqn() << "(" << destination_port << ")";
        }
      } else {
        std::vector<BasePort*> pointers;
        std::transform(std::begin(sinks), std::end(sinks), std::back_inserter(pointers),
                       [this](std::size_t index) { return this->ports_[index]; });

        ports_[source_port]->pull_connection(properties, pointers);

        log::Debug() << "from: " << ports_[source_port]->container()->fqn() << " |-> to: " << pointers.size()
                     << " objects";
      }
    }
  } else {
    std::cout << "NO TOP Environment: " << optimized_graph_.get_edges().size() << std::endl << std::flush;
  }

  log::Debug() << "Building the Graph";
  for (auto* reactor : top_level_reactors_) {
    build_dependency_graph(reactor);
  }

  calculate_indices();

  // this assembles all the contained environments aka enclaves
  for (auto* env : contained_environments_) {
    env->assemble();
  }
}

void Environment::optimize() {
#ifdef GRAPH_OPTIMIZATIONS
  constexpr bool enable_optimizations = GRAPH_OPTIMIZATIONS;
#else
  constexpr bool enable_optimizations = false;
#endif

  if constexpr (enable_optimizations) {
    static std::map<std::pair<ConnectionType, ConnectionType>, ConnectionType> construction_table = {
        // Normal + x
        {std::make_pair<ConnectionType, ConnectionType>(Normal, Normal), Normal},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, Delayed), Delayed},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, Enclaved), Enclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, Physical), Physical},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, DelayedEnclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, PhysicalEnclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Normal, Plugin), Plugin},
        // Delayed + x
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, Normal), Delayed},
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, Delayed), Delayed},
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, Enclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, Physical), Invalid}, //!!!
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, DelayedEnclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, PhysicalEnclaved), Invalid}, //!!!
        {std::make_pair<ConnectionType, ConnectionType>(Delayed, Plugin), Invalid},
        // Enclaved + x
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, Normal), Enclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, Delayed), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, Enclaved), Enclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, Physical), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, DelayedEnclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, PhysicalEnclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Enclaved, Plugin), Invalid},
        // Physical + x
        {std::make_pair<ConnectionType, ConnectionType>(Physical, Normal), Physical},
        {std::make_pair<ConnectionType, ConnectionType>(Physical, Delayed), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Physical, Enclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Physical, Physical), Physical},
        {std::make_pair<ConnectionType, ConnectionType>(Physical, DelayedEnclaved), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Physical, PhysicalEnclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(Physical, Plugin), Invalid},
        // DelayedEnclaved + x
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, Normal), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, Delayed), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, Enclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, Physical), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, DelayedEnclaved), DelayedEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, PhysicalEnclaved), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(DelayedEnclaved, Plugin), Invalid},
        // PhysicalEnclaved + x
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, Normal), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, Delayed), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, Enclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, Physical), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, DelayedEnclaved), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, PhysicalEnclaved), PhysicalEnclaved},
        {std::make_pair<ConnectionType, ConnectionType>(PhysicalEnclaved, Plugin), Invalid},
        // Plugin + x = Invalid
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, Normal), Invalid},           // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, Delayed), Invalid},          // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, Enclaved), Invalid},         // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, Physical), Invalid},         // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, DelayedEnclaved), Invalid},  // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, PhysicalEnclaved), Invalid}, // !!!
        {std::make_pair<ConnectionType, ConnectionType>(Plugin, Plugin), Invalid},           // !!!
    };

    // discards all current changes
    optimized_graph_.clear();

    // getting all the sources from the graph
    auto keys = graph_.keys();

    // generating all the possible destinations for all sources
    for (auto source : keys) {
      auto spanning_tree = graph_.spanning_tree(source);

      for (const auto& [destination, path] : spanning_tree) {
        ConnectionProperties merged_properties{};
        std::size_t current_source = source;

        for (auto element : path) {
          auto property = element.first;

          auto return_type =
              construction_table[std::pair<ConnectionType, ConnectionType>(merged_properties.type_, property.type_)];

          // invalid will split the connections
          if (return_type == Invalid) {
            // first add connection until this point
            optimized_graph_.add_edge(current_source, element.second, merged_properties);

            // updating the source of the connection and resetting the properties
            current_source = element.second;
            merged_properties = property;

          } else {
            // merging the connections
            merged_properties.type_ = return_type;

            // adding up delays
            merged_properties.delay_ += property.delay_;

            // updating target enclave if not nullptr
            merged_properties.enclave_ =
                (property.enclave_ != nullptr) ? property.enclave_ : merged_properties.enclave_;
          }
        }

        // add merged connection
        optimized_graph_.add_edge(current_source, destination, merged_properties);
      }
    }
  } else {
    // no optimizations
    optimized_graph_ = graph_;
  }
}

void Environment::calculate_indices() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto* reaction : reactions_) {
    graph[reaction];
  }
  for (auto dependencies : dependencies_) {
    graph[dependencies.first].insert(dependencies.second);
  }

  log_.debug() << "Reactions sorted by index:";
  unsigned int index = 0;
  while (!graph.empty()) {
    // find nodes with degree zero and assign index
    std::set<Reaction*> degree_zero;
    for (auto& edge : graph) {
      if (edge.second.empty()) {
        edge.first->set_index(index);
        degree_zero.insert(edge.first);
      }
    }

    if (degree_zero.empty()) {
      export_dependency_graph("/tmp/reactor_dependency_graph.dot");
      throw ValidationError("There is a loop in the dependency graph. Graph was written to "
                            "/tmp/reactor_dependency_graph.dot");
    }

    auto dbg = log_.debug();
    dbg << index << ": ";
    for (auto* reaction : degree_zero) {
      dbg << reaction->fqn() << ", ";
    }

    // reduce graph
    for (auto* reaction : degree_zero) {
      graph.erase(reaction);
    }
    for (auto& edge : graph) {
      for (auto* reaction : degree_zero) {
        edge.second.erase(reaction);
      }
    }

    index++;
  }

  max_reaction_index_ = index - 1;
}

void Environment::build_dependency_graph(Reactor* reactor) {
  // obtain dependencies from each contained reactor
  for (auto* sub_reactor : reactor->reactors()) {
    build_dependency_graph(sub_reactor);
  }
  // get reactions_ from this reactor; also order reactions_ by their priority
  std::map<int, Reaction*> priority_map;
  for (auto* reaction : reactor->reactions()) {
    reactions_.insert(reaction);
    auto result = priority_map.emplace(reaction->priority(), reaction);
    validate(result.second, "priorities must be unique for all reactions_ of the same reactor");
  }

  // connect all reactions_ this reaction depends on
  for (auto* reaction : reactor->reactions()) {
    for (auto* dependency : reaction->dependencies()) {
      auto* source = dependency;
      while (source->has_inward_binding()) {
        source = source->untyped_inward_binding();
      }
      for (auto* antidependency : source->anti_dependencies()) {
        dependencies_.emplace_back(reaction, antidependency);
      }
    }
  }

  // connect reactions_ by priority
  if (priority_map.size() > 1) {
    auto iterator = priority_map.begin();
    auto next = std::next(iterator);
    while (next != priority_map.end()) {
      dependencies_.emplace_back(next->second, iterator->second);
      iterator++;
      next = std::next(iterator);
    }
  }
}

// function used exclusively for debug prints in the exporter
auto dot_name([[maybe_unused]] ReactorElement* reactor_element) -> std::string {
  std::string fqn{reactor_element->fqn()};
  std::replace(fqn.begin(), fqn.end(), '.', '_');
  return fqn;
}

void Environment::export_dependency_graph(const std::string& path) {
  std::ofstream dot;
  dot.open(path);

  // sort all reactions_ by their index
  std::map<unsigned int, std::vector<Reaction*>> reactions_by_index;
  for (auto* reaction : reactions_) {
    reactions_by_index[reaction->index()].push_back(reaction);
  }

  // start the graph
  dot << "digraph {\n";
  dot << "rankdir=LR;\n";

  // place reactions_ of the same index in the same subgraph
  for (auto& index_reactions : reactions_by_index) {
    dot << "subgraph {\n";
    dot << "rank=same;\n";
    for (auto* reaction : index_reactions.second) {
      dot << dot_name(reaction) << " [label=\"" << reaction->fqn() << "\"];" << std::endl;
    }
    dot << "}\n";
  }

  // establish an order between subgraphs
  Reaction* reaction_from_last_index = nullptr;
  for (auto& index_reactions : reactions_by_index) {
    Reaction* reaction_from_this_index = index_reactions.second.front();
    if (reaction_from_last_index != nullptr) {
      dot << dot_name(reaction_from_last_index) << " -> " << dot_name(reaction_from_this_index) << " [style=invis];\n";
    }
    reaction_from_last_index = reaction_from_this_index;
  }

  // add all the dependencies
  for (auto dependency : dependencies_) {
    dot << dot_name(dependency.first) << " -> " << dot_name(dependency.second) << '\n';
  }
  dot << "}\n";

  dot.close();

  log_.info() << "Reaction graph was written to " << path;
}

void Environment::dump_to_yaml(const std::string& path) {
  std::ofstream yaml(path);
  yaml << "---" << std::endl;
  yaml << "top_level_instances:" << std::endl;
  for (const auto* reactors : top_level_reactors_) {
    yaml << "  - " << reactors->fqn() << std::endl;
  }
  yaml << "all_reactor_instances:" << std::endl;
  for (const auto* reactors : top_level_reactors_) {
    dump_instance_to_yaml(yaml, *reactors);
  }
  yaml << "reaction_dependencies:" << std::endl;
  for (auto& iterator : dependencies_) {
    yaml << "  - from: " << iterator.first->fqn() << std::endl;
    yaml << "  - to: " << iterator.second->fqn() << std::endl;
  }

  log_.info() << "Program structure was dumped to " << path;
}

void Environment::dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger) {
  yaml << "      - name: " << trigger.name() << std::endl;
  if (dynamic_cast<const StartupTrigger*>(&trigger) != nullptr) {
    yaml << "        type: startup" << std::endl;
  } else if (dynamic_cast<const ShutdownTrigger*>(&trigger) != nullptr) {
    yaml << "        type: shutdown" << std::endl;
  } else if (dynamic_cast<const Timer*>(&trigger) != nullptr) {
    yaml << "        type: timer" << std::endl;
  } else if (trigger.is_logical()) {
    yaml << "        type: logical action" << std::endl;
  } else {
    yaml << "        type: physical action" << std::endl;
  }
  yaml << "        trigger_of:" << std::endl;
  for (auto* const reaction : trigger.triggers()) {
    yaml << "          - " << reaction->fqn() << std::endl;
  }
  yaml << "        effect_of:" << std::endl;
  for (auto* const reaction : trigger.schedulers()) {
    yaml << "          - " << reaction->fqn() << std::endl;
  }
}

void Environment::dump_instance_to_yaml(std::ofstream& yaml, const Reactor& reactor) {
  yaml << "  " << reactor.fqn() << ':' << std::endl;
  yaml << "    name: " << reactor.name() << std::endl;
  if (reactor.is_top_level()) {
    yaml << "    container: null" << std::endl;
  } else {
    yaml << "    container: " << reactor.container()->fqn() << std::endl;
  }
  yaml << "    reactor_instances:" << std::endl;
  for (auto* const current_reactor : reactor.reactors()) {
    yaml << "      - " << current_reactor->fqn() << std::endl;
  }
  yaml << "    inputs:" << std::endl;
  for (auto* const inputs : reactor.inputs()) {
    dump_port_to_yaml(yaml, *inputs);
  }
  yaml << "    outputs:" << std::endl;
  for (auto* const outputs : reactor.outputs()) {
    dump_port_to_yaml(yaml, *outputs);
  }
  yaml << "    triggers:" << std::endl;
  for (auto* const actions : reactor.actions()) {
    dump_trigger_to_yaml(yaml, *actions);
  }
  yaml << "    reactions:" << std::endl;
  for (auto* const current_reaction : reactor.reactions()) {
    dump_reaction_to_yaml(yaml, *current_reaction);
  }

  for (auto* const reactors : reactor.reactors()) {
    dump_instance_to_yaml(yaml, *reactors);
  }
}

void Environment::dump_port_to_yaml(std::ofstream& yaml, const BasePort& port) {
  yaml << "      " << port.name() << ':' << std::endl;
  if (port.has_inward_binding()) {
    yaml << "        upstream_port: " << port.untyped_inward_binding()->fqn() << std::endl;
  } else {
    yaml << "        upstream_port: null" << std::endl;
  }
  yaml << "        downstream_ports: " << std::endl;
  for (auto* const outward_binding : port.outward_bindings()) {
    yaml << "          - " << outward_binding->fqn() << std::endl;
  }
  yaml << "        trigger_of: " << std::endl;
  for (auto* const trigger : port.triggers()) {
    yaml << "          - " << trigger->fqn() << std::endl;
  }
  yaml << "        source_of: " << std::endl;
  for (auto* const dependency : port.dependencies()) {
    if (port.triggers().find(dependency) == port.triggers().end()) {
      yaml << "          - " << dependency->fqn() << std::endl;
    }
  }
  yaml << "        effect_of: " << std::endl;
  for (auto* const antidepencency : port.anti_dependencies()) {
    yaml << "          - " << antidepencency->fqn() << std::endl;
  }
}

void Environment::dump_reaction_to_yaml(std::ofstream& yaml, const Reaction& reaction) {
  yaml << "      - name: " << reaction.name() << std::endl;
  yaml << "        priority: " << reaction.priority() << std::endl;
  yaml << "        level: " << reaction.index() << std::endl;
  yaml << "        triggers:" << std::endl;
  for (auto* const trigger : reaction.action_triggers()) {
    yaml << "          - " << trigger->fqn() << std::endl;
  }
  for (auto* const trigger : reaction.port_triggers()) {
    yaml << "          - " << trigger->fqn() << std::endl;
  }
  yaml << "        sources:" << std::endl;
  for (auto* dependency : reaction.dependencies()) {
    if (reaction.port_triggers().find(dependency) == reaction.port_triggers().end()) {
      yaml << "          - " << dependency->fqn() << std::endl;
    }
  }
  yaml << "        effects:" << std::endl;
  for (auto* const antidependency : reaction.antidependencies()) {
    yaml << "          - " << antidependency->fqn() << std::endl;
  }
  for (auto* const scheduable_action : reaction.scheduable_actions()) {
    yaml << "          - " << scheduable_action->fqn() << std::endl;
  }
}

} // namespace reactor
