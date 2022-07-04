/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/environment.hh"

#include <algorithm>
#include <fstream>
#include <map>

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

void Environment::register_reactor(Reactor* reactor) {
  reactor_assert(reactor != nullptr);
  validate(this->phase() == Phase::Construction, "Reactors may only be registered during construction phase!");
  validate(reactor->is_top_level(), "The environment may only contain top level reactors!");
  reactor_assert(top_level_reactors_.insert(reactor).second);
}

void recursive_assemble(Reactor* container) { // NOLINT
  container->assemble();
  for (auto* reactor : container->reactors()) {
    recursive_assemble(reactor);
  }
}

void Environment::assemble() {
  validate(this->phase() == Phase::Construction, "assemble() may only be called during construction phase!");
  phase_ = Phase::Assembly;
  for (auto* reactor : top_level_reactors_) {
    recursive_assemble(reactor);
  }

  // build the dependency graph
  for (auto* reactor : top_level_reactors_) {
    build_dependency_graph(reactor);
  }
  calculate_indexes();

  // Testbed for the new graph structure
  ReactionDependencyGraph graph{top_level_reactors_};
  ReactionDependencyGraph reduced_graph = graph.transitive_reduction();

  graph.export_graphviz("graph.dot");
  reduced_graph.export_graphviz("reduced_graph.dot");
}

void Environment::build_dependency_graph(Reactor* reactor) { // NOLINT
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
        source = source->inward_binding();
      }
      for (auto* antidependency : source->antidependencies()) {
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

void Environment::sync_shutdown() {
  validate(this->phase() == Phase::Execution, "sync_shutdown() may only be called during execution phase!");
  phase_ = Phase::Shutdown;

  log::Info() << "Terminating the execution";

  for (auto* reactor : top_level_reactors_) {
    reactor->shutdown();
  }

  phase_ = Phase::Deconstruction;
  scheduler_.stop();
}

void Environment::async_shutdown() {
  scheduler_.lock();
  sync_shutdown();
  scheduler_.unlock();
}

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

  log::Info() << "Reaction graph was written to " << path;
}

void Environment::calculate_indexes() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto* reaction : reactions_) {
    graph[reaction];
  }
  for (auto dependencies : dependencies_) {
    graph[dependencies.first].insert(dependencies.second);
  }

  log::Debug() << "Reactions sorted by index:";
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

    log::Debug dbg;
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

auto Environment::startup() -> std::thread {
  validate(this->phase() == Phase::Assembly, "startup() may only be called during assembly phase!");

  // build the dependency graph
  for (auto* reactor : top_level_reactors_) {
    build_dependency_graph(reactor);
  }
  calculate_indexes();

  log::Info() << "Starting the execution";
  phase_ = Phase::Startup;

  start_time_ = get_physical_time();
  // start up initialize all reactors
  for (auto* reactor : top_level_reactors_) {
    reactor->startup();
  }

  // start processing events
  phase_ = Phase::Execution;
  return std::thread([this]() { this->scheduler_.start(); });
}

void Environment::dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger) {
  yaml << "      - name: " << trigger.name() << std::endl;
  if (dynamic_cast<const StartupAction*>(&trigger) != nullptr) {
    yaml << "        type: startup" << std::endl;
  } else if (dynamic_cast<const ShutdownAction*>(&trigger) != nullptr) {
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
    yaml << "        upstream_port: " << port.inward_binding()->fqn() << std::endl;
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
  for (auto* const antidepencency : port.antidependencies()) {
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

  log::Info() << "Program structure was dumped to " << path;
}

} // namespace reactor
