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
#include <type_traits>
#include <vector>

namespace reactor {
class GraphElement {
public:
  GraphElement() noexcept = default;
  GraphElement(const GraphElement& graph) noexcept = default;
  GraphElement(GraphElement&& graph) noexcept = default;

  virtual ~GraphElement() noexcept = default;
  [[nodiscard]] virtual auto connected_to_downstream_actions() const noexcept -> bool = 0;
  [[nodiscard]] virtual auto connected_to_upstream_actions() const noexcept -> bool = 0;
  [[nodiscard]] virtual auto rating() const noexcept -> std::size_t = 0;

  auto operator=([[maybe_unused]] const GraphElement& other) noexcept -> GraphElement& = default;
  auto operator=([[maybe_unused]] GraphElement&& other) noexcept -> GraphElement& = default;
};

// this graph is special, because to every edge properties are annotated
template <class X> class Graph {
  // static_assert(std::is_base_of_v<GraphElement, X>);
  using E = X*;
  using P = ConnectionProperties;

private:
  using Path = std::vector<std::tuple<E, P, E>>;
  std::map<E, std::vector<std::pair<P, E>>> graph_{};
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
      std::vector<std::pair<P, E>> edges;
      edges.emplace_back(properties, destination);
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
  auto naive_spanning_tree(E source) noexcept -> std::vector<std::vector<std::tuple<E, P, E>>> {
    return recursive_spanning_tree(source, std::vector<E>{});
  }

  // this function goes recursively though the graph and tries to find every possible path
  auto recursive_spanning_tree(E source_node, std::vector<E> visited_nodes)
      -> std::vector<std::vector<std::tuple<E, P, E>>> {
    std::vector<Path> paths{};

    if (graph_[source_node].empty()) {
      return std::vector<Path>{Path{}};
    }

    // if this node has an action we need to append the path
    if (source_node->connected_to_downstream_actions()) {
      paths.push_back(Path{});
    }

    for (auto child : graph_[source_node]) {
      E current_node = child.second;

      // we dont need to check for cycles because lf semantics assure that there wont be any cycles
      for (auto path : recursive_spanning_tree(current_node, visited_nodes)) {
        path.push_back(std::make_tuple(source_node, child.first, current_node));
        paths.push_back(path);
      }
    }

    return paths;
  }

  [[nodiscard]] auto get_destinations(E source) const noexcept -> std::vector<std::pair<P, E>> {
    return this->graph_.at(source);
  }

  [[nodiscard]] auto to_mermaid() const noexcept -> std::string {
    std::string mermaid_string = "graph TD;\n";
    std::size_t index{0};
    std::map<E, std::string> name_map{};

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

  void optimize(Graph<X>& optimized_graph) {
    optimized_graph.clear();

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

    // getting all the sources from the graph
    auto keys = this->keys();

    std::vector<E> has_downstreams{};
    std::copy_if(keys.begin(), keys.end(), std::back_inserter(has_downstreams),
                 [](auto element) { return element->connected_to_downstream_actions(); });

    std::vector<E> has_upstreams{};
    std::copy_if(keys.begin(), keys.end(), std::back_inserter(has_upstreams),
                 [](auto element) { return element->connected_to_upstream_actions(); });

    // generating all the possible destinations for all sources
    for (auto* source : has_upstreams) {
      auto spanning_tree = naive_spanning_tree(source);

      for (auto& path : spanning_tree) {
        ConnectionProperties merged_properties{};
        auto* final_destination = std::get<2>(*std::begin(path));
        std::size_t current_rating = 0;

        for (auto edge : path) {
          auto property = std::get<1>(edge);
          // auto source_port = std::get<0>(edge);
          auto* destination_port = std::get<2>(edge);

          current_rating += destination_port->rating();

          if (current_rating > 0) {
            auto return_type =
                construction_table[std::pair<ConnectionType, ConnectionType>(merged_properties.type_, property.type_)];
            // invalid will split the connections
            if (return_type == Invalid) {
              // first add connection until this point
              optimized_graph.add_edge(destination_port, final_destination, merged_properties); // NOLINT

              // resetting the properties and destination_port
              final_destination = destination_port;
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
        }
        optimized_graph.add_edge(std::get<0>(*(std::end(path) - 1)), final_destination, merged_properties);
      }
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