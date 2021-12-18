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

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

void Environment::register_reactor(Reactor* reactor) {
    reactor_assert(reactor != nullptr);
    validate(this->phase() == Phase::Construction,
           "Reactors may only be registered during construction phase!");
    validate(reactor->is_top_level(),
           "The environment may only contain top level reactors!");
    reactor_assert(top_level_reactors_.insert(reactor).second);
}

void recursive_assemble(Reactor* container) {
    container->assemble();
    for (auto r : container->reactors()) {
        recursive_assemble(r);
    }
}

void Environment::assemble() {
    validate(this->phase() == Phase::Construction,
           "assemble() may only be called during construction phase!");
    phase_ = Phase::Assembly;
    for (auto r : top_level_reactors_) {
        recursive_assemble(r);
    }
}

void Environment::build_dependency_graph(Reactor* reactor) {
    // obtain dependencies from each contained reactor
    for (auto r : reactor->reactors()) {
        build_dependency_graph(r);
    }
    // get reactions_ from this reactor; also order reactions_ by their priority
    std::map<int, Reaction*> priority_map;
    for (auto r : reactor->reactions()) {
        reactions_.insert(r);
        auto result = priority_map.emplace(r->priority(), r);
        validate(result.second,"priorities must be unique for all reactions_ of the same reactor");
    }

    // connect all reactions_ this reaction depends on
    for (auto r : reactor->reactions()) {
        for (auto d : r->dependencies()) {
            auto source = d;
            while (source->has_inward_binding()) {
                source = source->inward_binding();
            }
            for (auto ad : source->antidependencies()) {
                dependencies_.emplace_back(r, ad);
            }
        }
    }

    // connect reactions_ by priority
    if (priority_map.size() > 1) {
        auto it = priority_map.begin();
        auto next = std::next(it);
        while (next != priority_map.end()) {
            dependencies_.emplace_back(next->second, it->second);
            it++;
            next = std::next(it);
        }
    }
}

std::thread Environment::startup() {
    validate(this->phase() == Phase::Assembly,
           "startup() may only be called during assembly phase!");

    // build the dependency graph
    for (auto r : top_level_reactors_) {
        build_dependency_graph(r);
    }
    calculate_indexes();

    log::Info() << "Starting the execution";
    phase_ = Phase::Startup;

    start_time_ = get_physical_time();
    // start up initialize all reactors
    for (auto r : top_level_reactors_) {
        r->startup();
    }

    // start processing events
    phase_ = Phase::Execution;
    return std::thread([this]() { this->scheduler_.start(); });
}

void Environment::sync_shutdown() {
    validate(this->phase() == Phase::Execution,
           "sync_shutdown() may only be called during execution phase!");
    phase_ = Phase::Shutdown;

    log::Info() << "Terminating the execution";

    for (auto r : top_level_reactors_) {
        r->shutdown();
    }

    phase_ = Phase::Deconstruction;
    scheduler_.stop();
}

void Environment::async_shutdown() {
    scheduler_.lock();
    sync_shutdown();
    scheduler_.unlock();
}

std::string dot_name(ReactorElement* r) {
    std::string fqn = r->fqn();
    std::replace(fqn.begin(), fqn.end(), '.', '_');
    return fqn;
}

void Environment::export_dependency_graph(const std::string& path) {
    std::ofstream dot;
    dot.open(path);

    // sort all reactions_ by their index
    std::map<unsigned, std::vector<Reaction*>> reactions_by_index;
    for (auto r : reactions_) {
        reactions_by_index[r->index()].push_back(r);
    }

    // start the graph
    dot << "digraph {\n";
    dot << "rankdir=LR;\n";

    // place reactions_ of the same index in the same subgraph
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
    for (auto d : dependencies_) {
        dot << dot_name(d.first) << " -> " << dot_name(d.second) << '\n';
    }
    dot << "}\n";

    dot.close();

    log::Info() << "Reaction graph was written to " << path;
}

void Environment::calculate_indexes() {
    // build the graph
    std::map<Reaction*, std::set<Reaction*>> graph;
    for (auto r : reactions_) {
        graph[r];
    }
    for (auto d : dependencies_) {
        graph[d.first].insert(d.second);
    }

    log::Debug() << "Reactions sorted by index:";
    unsigned int index = 0;
    while (!graph.empty()) {
        // find nodes with degree zero and assign index
        std::set<Reaction*> degree_zero;
        for (auto& kv : graph) {
            if (kv.second.empty()) {
                kv.first->set_index(index);
                degree_zero.insert(kv.first);
            }
        }

        if (degree_zero.empty()) {
            export_dependency_graph("/tmp/reactor_dependency_graph.dot");
            throw ValidationError(
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

    max_reaction_index_ = index - 1;
}

}  // namespace reactor
