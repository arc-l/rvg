#include <VisibilityGraph/Graph.h>

using namespace RotationalVisibilityGraph;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

int main() {
  std::vector<std::shared_ptr<Vertex<T>>> vertices = {
      std::make_shared<Vertex<T>>(2, -1),
      std::make_shared<Vertex<T>>(4, -1),
      std::make_shared<Vertex<T>>(4, -2),
      std::make_shared<Vertex<T>>(2, -2)
  };
  std::vector<std::shared_ptr<Vertex<T>>> vertices1 = {
      std::make_shared<Vertex<T>>(2, -1),
      std::make_shared<Vertex<T>>(4, -1),
      std::make_shared<Vertex<T>>(4, -2),
      std::make_shared<Vertex<T>>(2, -2)
  };
  Graph<T> graph;
  for (size_t i = 0; i < vertices.size(); i++) {
    graph.addEdge(vertices[i], vertices[(i + 1) % vertices.size()]);
  }
  for (size_t i = 0; i < vertices1.size(); i++) {
    graph.addEdge(vertices1[i], vertices1[(i + 1) % vertices1.size()]);
  }
  auto verticesGraph = graph.getVertices();
  Utils::print("Vertices size", verticesGraph.size());
  auto adjacencyList = graph.getAdjacencyList();
  Utils::print("Adjacency List", 1, "a ", 1);
  for (const auto &i : adjacencyList) {
    Utils::print(*i.first, "[");
    for (const auto &j : i.second) {
      Utils::print(*j, " ");
    }
    Utils::print("]");
  }

  return 0;
}