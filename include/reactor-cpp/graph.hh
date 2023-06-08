/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_GRAPH_HH
#define REACTOR_CPP_GRAPH_HH

#include <iostream>
#include <map>
#include <optional>
#include <vector>

namespace reactor {
// this graph is special, because to every edge properties or type T are annotated
// TODO: maybe give some restrictions on type T e.g. Size and so on
template <class E, class P> class PropertyGraph {
private:
  std::map<E, std::vector<std::pair<P, E>>> graph_;

  // custom compare operator this is required if u want special key values in std::map
  // this is required for the PropertyGraph::get_edges() method
  using map_key = std::pair<E, P>;
  struct map_key_compare {
    // TODO: check this maybe will cause some very funny TM problems later
    auto operator()(const map_key& left_site, const map_key& right_site) const -> bool {
      return left_site.first < right_site.first;
    }
  };

public:
  PropertyGraph() noexcept = default;
  PropertyGraph(const PropertyGraph& graph) noexcept
      : graph_(graph.graph_) {}
  PropertyGraph(const PropertyGraph&& graph) noexcept
      : graph_(std::move(graph.graph_)) {}
  ~PropertyGraph() noexcept = default;

  auto operator=(PropertyGraph other) noexcept -> PropertyGraph& {
    graph_ = other.graph_;
    return *this;
  }

  auto operator=(PropertyGraph&& other) noexcept -> PropertyGraph& {
    graph_ = std::move(other.graph_);
    return *this;
  }

  // adds a single edge to the graph structure
  void add_edge(E source, E destination, P properties) noexcept {
    if (graph_.find(source) != std::end(graph_)) {
      std::vector<std::pair<P, E>> edges{std::make_pair(properties, destination)};
      graph_[source] = edges;
    } else {
      graph_[source].emplace_back(properties, destination);
    }
  }

  // this groups connections by same source and properties
  [[nodiscard]] auto get_edges() const noexcept -> std::map<map_key, std::vector<E>, map_key_compare> {
    std::map<map_key, std::vector<E>, map_key_compare> all_edges{};
    for (auto const& [source, sinks] : graph_) {

      for (const auto& sink : sinks) {
        auto key = std::make_pair(source, sink.first);
        all_edges.try_emplace(key, std::vector<E>{});
        all_edges[key].push_back(sink.second);
      }
    }

    return all_edges;
  };

  // deletes the content of the graph
  void clear() noexcept { graph_.clear(); }

  // returns all the sources from the graph
  [[nodiscard]] auto keys() const noexcept -> std::vector<E> {
    std::vector<E> keys{};
    for (auto it = graph_.begin(); it != graph_.end(); ++it) {
      keys.push_back(it->first);
    }
    return keys;
  }

  // returns the spanning tree of a given source including properties
  [[nodiscard]] auto spanning_tree(E source) noexcept -> std::map<E, std::vector<std::pair<P, E>>> {
    std::map<E, std::vector<std::pair<P, E>>> tree{};
    std::vector<E> work_nodes{source};

    while (!work_nodes.empty()) {
      auto parent = *work_nodes.begin();

      for (auto child : graph_[parent]) {
        // figuring out the properties until this node
        std::vector<std::pair<P, E>> parent_properties{};
        if (tree.find(parent) != std::end(tree)) {
          // this if should always be the case except the first time when tree is empty
          parent_properties = tree[parent]; // TODO: make sure this is a copy otherwise we change this properties as
                                            // well
        }

        // appending the new property and inserting into the tree
        parent_properties.push_back(child);
        work_nodes.push_back(child.second);
        tree[child.second] = parent_properties;
      }

      work_nodes.erase(std::begin(work_nodes));
    }

    return tree;
  }

  [[nodiscard]] auto get_destinations(E source) const noexcept -> std::vector<std::pair<P, E>> {
    return graph_[source];
  }

  [[nodiscard]] auto get_upstream(E vertex) const noexcept -> std::optional<E> {
    for (const auto& [source, sinks] : graph_) {
      if (sinks.second.contains(vertex)) {
        return source;
      }
    }
  }
};
} // namespace reactor
#endif // REACTOR_CPP_GRAPH_HH