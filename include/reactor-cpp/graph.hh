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
// this graph is special, because to every edge properties are annotated
template <class E, class P> class Graph {
private:
  std::map<E, std::vector<std::pair<P, E>>> graph_;

  // custom compare operator this is required if u want special key values in std::map
  // this is required for the Graph::get_edges() method
  using map_key = std::pair<E, P>;
  struct map_key_compare {
    auto operator()(const map_key& left_site, const map_key& right_site) const -> bool {
      return left_site.first < right_site.first ||
             (left_site.second == right_site.second && left_site.second < right_site.second);
    }
  };

public:
  Graph() noexcept = default;
  Graph(const Graph& graph) noexcept
      : graph_(graph.graph_) {}
  Graph(const Graph&& graph) noexcept
      : graph_(std::move(graph.graph_)) {}
  ~Graph() noexcept = default;

  auto operator=(const Graph other) noexcept -> Graph& {
    graph_ = other.graph_;
    return *this;
  }

  auto operator=(Graph&& other) noexcept -> Graph& {
    graph_ = std::move(other.graph_);
    return *this;
  }

  // adds a single edge to the graph structure
  void add_edge(E source, E destination, P properties) noexcept {
    if (graph_.find(source) == std::end(graph_)) {
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

  auto remove_edge(E source, E destinations) noexcept {
    if (graph_.find(source) == std::end(graph_)) {
      return;
    }
    auto conns = std::find_if(std::begin(graph_[source]), std::end(graph_[source]),
                              [destinations](auto val) { return val.second == destinations; });

    if (conns != std::end(graph_[source])) {
      graph_[source].erase(conns);
    }
  }

  friend auto operator<<(std::ostream& outstream, const Graph& graph) -> std::ostream& {
    for (auto const& [source, destinations] : graph.graph_) {
      for (auto destination : destinations) {
        outstream << source << " --> " << destination.second << std::endl;
      }
    }
    return outstream;
  }
};
} // namespace reactor
#endif // REACTOR_CPP_GRAPH_HH