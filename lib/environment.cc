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
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

void Environment::register_reactor(Reactor* reactor) {
  ASSERT(reactor != nullptr);
  VALIDATE(this->phase() == Phase::Construction,
           "Reactors may only be registered during construction phase!");
  VALIDATE(reactor->is_top_level(),
           "The environment may only contain top level reactors!");
  auto r = _top_level_reactors.insert(reactor);
  ASSERT(r.second);
}

void recursive_assemble(Reactor* container) {
  container->assemble();
  for (auto r : container->reactors()) {
    recursive_assemble(r);
  }
}

void Environment::assemble() {
  VALIDATE(this->phase() == Phase::Construction,
           "assemble() may only be called during construction phase!");
  _phase = Phase::Assembly;
  for (auto r : _top_level_reactors) {
    recursive_assemble(r);
  }

  // build the dependency graph
  for (auto r : _top_level_reactors) {
    build_dependency_graph(r);
  }
  calculate_indexes();
}

void Environment::build_dependency_graph(Reactor* reactor) {
  // obtain dependencies from each contained reactor
  for (auto r : reactor->reactors()) {
    build_dependency_graph(r);
  }
  // get reactions from this reactor; also order reactions by their priority
  std::map<int, Reaction*> priority_map;
  for (auto r : reactor->reactions()) {
    reactions.insert(r);
    auto result = priority_map.emplace(r->priority(), r);
    VALIDATE(result.second,
             "priorities must be unique for all reactions of the same reactor");
  }

  // connect all reactions this reaction depends on
  for (auto r : reactor->reactions()) {
    for (auto d : r->dependencies()) {
      auto source = d;
      while (source->has_inward_binding()) {
        source = source->inward_binding();
      }
      for (auto ad : source->antidependencies()) {
        dependencies.push_back(std::make_pair(r, ad));
      }
    }
  }

  // connect reactions by priority
  if (priority_map.size() > 1) {
    auto it = priority_map.begin();
    auto next = std::next(it);
    while (next != priority_map.end()) {
      dependencies.push_back(std::make_pair(next->second, it->second));
      it++;
      next = std::next(it);
    }
  }
}

void dump_reaction_to_yaml(std::ofstream& yaml, const Reaction& r) {
  yaml << "      - name: " << r.name() << std::endl;
  yaml << "        priority: " << r.priority() << std::endl;
  yaml << "        level: " << r.index() << std::endl;
  yaml << "        triggers:" << std::endl;
  for (const auto t : r.action_triggers()) {
    yaml << "          - " << t->fqn() << std::endl;
  }
  for (const auto t : r.port_triggers()) {
    yaml << "          - " << t->fqn() << std::endl;
  }
  yaml << "        sources:" << std::endl;
  for (auto d : r.dependencies()) {
    if (r.port_triggers().find(d) == r.port_triggers().end()) {
      yaml << "          - " << d->fqn() << std::endl;
    }
  }
  yaml << "        effects:" << std::endl;
  for (const auto a : r.antidependencies()) {
    yaml << "          - " << a->fqn() << std::endl;
  }
  for (const auto a : r.scheduable_actions()) {
    yaml << "          - " << a->fqn() << std::endl;
  }
}

void dump_port_to_yaml(std::ofstream& yaml, const BasePort& port) {
  yaml << "      " << port.name() << ':' << std::endl;
  if (port.has_inward_binding()) {
    yaml << "        upstream_port: " << port.inward_binding()->fqn()
         << std::endl;
  } else {
    yaml << "        upstream_port: null" << std::endl;
  }
  yaml << "        downstream_ports: " << std::endl;
  for (const auto d : port.outward_bindings()) {
    yaml << "          - " << d->fqn() << std::endl;
  }
  yaml << "        trigger_of: " << std::endl;
  for (const auto t : port.triggers()) {
    yaml << "          - " << t->fqn() << std::endl;
  }
  yaml << "        source_of: " << std::endl;
  for (const auto d : port.dependencies()) {
    if (port.triggers().find(d) == port.triggers().end()) {
      yaml << "          - " << d->fqn() << std::endl;
    }
  }
  yaml << "        effect_of: " << std::endl;
  for (const auto a : port.antidependencies()) {
    yaml << "          - " << a->fqn() << std::endl;
  }
}

void dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger) {
  yaml << "      - name: " << trigger.name() << std::endl;
  if (dynamic_cast<const StartupAction*>(&trigger)) {
    yaml << "        type: startup" << std::endl;
  } else if (dynamic_cast<const ShutdownAction*>(&trigger)) {
    yaml << "        type: shutdown" << std::endl;
  } else if (dynamic_cast<const Timer*>(&trigger)) {
    yaml << "        type: timer" << std::endl;
  } else if (trigger.is_logical()) {
    yaml << "        type: logical action" << std::endl;
  } else {
    yaml << "        type: physical action" << std::endl;
  }
  yaml << "        trigger_of:" << std::endl;
  for (const auto t : trigger.triggers()) {
    yaml << "          - " << t->fqn() << std::endl;
  }
  yaml << "        effect_of:" << std::endl;
  for (const auto s : trigger.schedulers()) {
    yaml << "          - " << s->fqn() << std::endl;
  }
}

void dump_instance_to_yaml(std::ofstream& yaml, const Reactor& reactor) {
  yaml << "  " << reactor.fqn() << ':' << std::endl;
  yaml << "    name: " << reactor.name() << std::endl;
  if (reactor.is_top_level()) {
    yaml << "    container: null" << std::endl;
  } else {
    yaml << "    container: " << reactor.container()->fqn() << std::endl;
  }
  yaml << "    reactor_instances:" << std::endl;
  for (const auto r : reactor.reactors()) {
    yaml << "      - " << r->fqn() << std::endl;
  }
  yaml << "    inputs:" << std::endl;
  for (const auto i : reactor.inputs()) {
    dump_port_to_yaml(yaml, *i);
  }
  yaml << "    outputs:" << std::endl;
  for (const auto o : reactor.outputs()) {
    dump_port_to_yaml(yaml, *o);
  }
  yaml << "    triggers:" << std::endl;
  for (const auto a : reactor.actions()) {
    dump_trigger_to_yaml(yaml, *a);
  }
  yaml << "    reactions:" << std::endl;
  for (const auto r : reactor.reactions()) {
    dump_reaction_to_yaml(yaml, *r);
  }

  for (const auto r : reactor.reactors()) {
    dump_instance_to_yaml(yaml, *r);
  }
}

void Environment::dump_to_yaml(const std::string& path) {
  std::ofstream yaml(path);
  yaml << "---" << std::endl;
  yaml << "top_level_instances:" << std::endl;
  for (const auto* r : _top_level_reactors) {
    yaml << "  - " << r->fqn() << std::endl;
  }
  yaml << "all_reactor_instances:" << std::endl;
  for (const auto* r : _top_level_reactors) {
    dump_instance_to_yaml(yaml, *r);
  }
}

std::thread Environment::startup() {
  VALIDATE(this->phase() == Phase::Assembly,
           "startup() may only be called during assembly phase!");

  log::Info() << "Starting the execution";
  _phase = Phase::Startup;

  _start_time = get_physical_time();
  // startup all reactors
  for (auto r : _top_level_reactors) {
    r->startup();
  }

  // start processing events
  _phase = Phase::Execution;
  return std::thread([this]() { this->_scheduler.start(); });
}

void Environment::sync_shutdown() {
  VALIDATE(this->phase() == Phase::Execution,
           "sync_shutdown() may only be called during execution phase!");
  _phase = Phase::Shutdown;

  log::Info() << "Terminating the execution";

  for (auto r : _top_level_reactors) {
    r->shutdown();
  }

  _phase = Phase::Deconstruction;

  _scheduler.stop();
}

void Environment::async_shutdown() {
  _scheduler.lock();
  sync_shutdown();
  _scheduler.unlock();
}

std::string dot_name(ReactorElement* r) {
  std::string fqn = r->fqn();
  std::replace(fqn.begin(), fqn.end(), '.', '_');
  return fqn;
}

void Environment::export_dependency_graph(const std::string& path) {
  std::ofstream dot;
  dot.open(path);

  // sort all reactions by their index
  std::map<unsigned, std::vector<Reaction*>> reactions_by_index;
  for (auto r : reactions) {
    reactions_by_index[r->index()].push_back(r);
  }

  // start the graph
  dot << "digraph {\n";
  dot << "rankdir=LR;\n";

  // place reactions of the same index in the same subgraph
  for (auto& index_reactions : reactions_by_index) {
    dot << "subgraph {\n";
    dot << "rank=same;\n";
    for (auto r : index_reactions.second) {
      dot << dot_name(r) << " [label=\"" << r->fqn() << "\"];" << std::endl;
    }
    dot << "}\n";
  }

  // establish an order between subgraphs
  Reaction* reaction_from_last_index = nullptr;
  for (auto& index_reactions : reactions_by_index) {
    Reaction* reaction_from_this_index = index_reactions.second.front();
    if (reaction_from_last_index != nullptr) {
      dot << dot_name(reaction_from_last_index) << " -> "
          << dot_name(reaction_from_this_index) << " [style=invis];\n";
    }
    reaction_from_last_index = reaction_from_this_index;
  }

  // add all the dependencies
  for (auto d : dependencies) {
    dot << dot_name(d.first) << " -> " << dot_name(d.second) << '\n';
  }
  dot << "}\n";

  dot.close();

  log::Info() << "Reaction graph was written to " << path;
}

void Environment::calculate_indexes() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto r : reactions) {
    graph[r];
  }
  for (auto d : dependencies) {
    graph[d.first].insert(d.second);
  }

  log::Debug() << "Reactions sorted by index:";
  unsigned index = 0;
  while (graph.size() != 0) {
    // find nodes with degree zero and assign index
    std::set<Reaction*> degree_zero;
    for (auto& kv : graph) {
      if (kv.second.size() == 0) {
        kv.first->set_index(index);
        degree_zero.insert(kv.first);
      }
    }

    if (degree_zero.size() == 0) {
      export_dependency_graph("/tmp/reactor_dependency_graph.dot");
      throw reactor::ValidationError(
          "There is a loop in the dependency graph. Graph was written to "
          "/tmp/reactor_dependency_graph.dot");
    }

    log::Debug dbg;
    dbg << index << ": ";
    for (auto r : degree_zero) {
      dbg << r->fqn() << ", ";
    }

    // reduce graph
    for (auto r : degree_zero) {
      graph.erase(r);
    }
    for (auto& kv : graph) {
      for (auto r : degree_zero) {
        kv.second.erase(r);
      }
    }

    index++;
  }

  _max_reaction_index = index - 1;
}

}  // namespace reactor
