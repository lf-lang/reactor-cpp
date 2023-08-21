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
  std::set<E> nodes_{};

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

  using Path = std::vector<std::pair<P, E>>;

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
    nodes_.insert(source);
    nodes_.insert(destination);
    if (graph_.find(source) == std::end(graph_)) {
      Path edges{std::make_pair(properties, destination)};
      graph_[source] = edges;
    } else {
      graph_[source].emplace_back(properties, destination);
    }
  }

  auto get_nodes() -> std::set<E> { return nodes_; }

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

  // the return type looks a little bit cursed what is happening here ?
  // we have a map from the destination as a key to a list of paths through the graph.
  // A path here is modelled by a list of edges (with properties and the next vertex).
  auto naive_spanning_tree(E source) noexcept -> std::vector<std::vector<std::pair<P, E>>> {
    return recursive_spanning_tree(source, std::vector<E>{});
  }

  // this function goes recursively though the graph and tries to find every possible path
  auto recursive_spanning_tree(E source_node, std::vector<E> visited_nodes)
      -> std::vector<std::vector<std::pair<P, E>>> {
    std::vector<Path> paths{};

    if (graph_[source_node].empty()) {
      return std::vector<Path>{Path{}};
    }

    for (auto child : graph_[source_node]) {
      E current_node = child.second;

      // means this node has not been visited yey
      if (std::find(std::begin(visited_nodes), std::end(visited_nodes), current_node) == std::end(visited_nodes)) {

        // creating a temporary vector where the currently selected vertex is appended
        auto temp_nodes = visited_nodes;
        temp_nodes.push_back(current_node);

        for (auto path : recursive_spanning_tree(current_node, temp_nodes)) {
          path.insert(std::begin(path), child);
          paths.push_back(path);
        }
      }
    }

    return paths;
  }

  auto shortest_path(E source, E destination) -> std::optional<Path> {
    // TODO: maybe build proper djikstra here

    auto spanning_tre = naive_spanning_tree(source);
    std::vector<Path> relevant_paths{};

    std::copy_if(spanning_tre.begin(), spanning_tre.end(), std::back_inserter(relevant_paths),
                 [&, destination](Path path) { return path[path.size() - 1].second == destination; });

    if (relevant_paths.empty()) {
      return std::nullopt;
    }

    Path best_path = *relevant_paths.begin();

    for (auto path : relevant_paths) {
      if (path.size() < best_path.size()) {
        best_path = path;
      }
    }

    return best_path;
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

  [[nodiscard]] auto to_mermaid() const noexcept -> std::string {
    std::size_t index{0};
    std::map<E, std::string> name_map{};
    std::string mermaid_string = "graph TD;\n";

    auto name_resolver = [&](E object) -> std::string {
      char names[] = "ABCDEFGHIJKLMNOPQRSTUVGXYZabcdefghijklmnopqrstuvgxyz"; // NOLINT
      if (name_map.find(object) == std::end(name_map)) {
        name_map[object] = names[index];
        index++;
        return std::string{names[index - 1], 1};
      }
      return name_map[object];
    };

    for (const auto& [source, destinations] : graph_) {
      for (auto dest : destinations) {
        mermaid_string += std::string("    ") + name_resolver(source) + std::string("-->") +
                          name_resolver(dest.second) + std::string(";\n");
      }
    }
    return mermaid_string;
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