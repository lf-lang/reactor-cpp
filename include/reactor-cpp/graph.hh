/*
* Copyright (C) 2019 TU Dresden
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
/// this graph is special, because to every edge properties or type T are annotated
// TODO: maybe give some restrictions on type T e.g. Size and so on
template <class E, class P> class PropertyGraph {
private:
 std::map<E, std::vector<std::pair<P, E>>> graph_;
 using map_key = std::pair<E, P>;

 struct map_key_compare {
   auto operator()(const map_key &l, const map_key &r) const -> bool {
     return l.first < r.first;
   }
 };

public:
 PropertyGraph() noexcept = default;
 ~PropertyGraph() noexcept = default;

 void add_edge(E source, E destination, P properties) noexcept {
   if (!graph_.contains(source)) {
     std::vector<std::pair<P, E>> edges{std::make_pair(properties, destination)};
     graph_[source] = edges;
   } else {
     graph_[source].emplace_back(properties, destination);
   }
 }

 [[nodiscard]] auto get_destinations(E source) const noexcept -> std::vector<std::pair<P, E>> {
   return graph_[source];
 }

 [[nodiscard]] auto get_edges() const noexcept -> std::map<map_key, std::vector<E>, map_key_compare> {
   std::map<map_key , std::vector<E>, map_key_compare> all_edges{};
   for (auto const& [source, sinks] : graph_) {

     for (const auto& sink : sinks) {
       auto key = std::make_pair(source, sink.first);
       all_edges.try_emplace(key, std::vector<E>{});
       all_edges[key].push_back(sink.second);
       std::cout << "from: " << source << " --> to: " << sink.second << std::endl;
     }
   }

   return all_edges;
 };

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