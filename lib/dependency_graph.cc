#include <fstream>

#include "reactor-cpp/dependency_graph.hh"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>

using namespace boost;

void reactor::generate_test_graph() {
  // create a typedef for the Graph type
  using Graph = adjacency_list<vecS, vecS, bidirectionalS>;

  // Make convenient labels for the vertices
  enum { A, B, C, D, E, N };
  const int num_vertices = N;

  // writing out the edges in the graph
  using Edge = std::pair<int, int>;
  Edge edge_array[] = {Edge(A, B), Edge(A, D), Edge(C, A), Edge(D, C), Edge(C, E), Edge(B, D), Edge(D, E)}; // NOLINT

  // declare a graph object
  Graph g(num_vertices);

  // add the edges to the graph object
  for (auto& i : edge_array) {
    add_edge(i.first, i.second, g);
  }

  std::ofstream dot_file("test.dot");
  write_graphviz(dot_file, g);
}
