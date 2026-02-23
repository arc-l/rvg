#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_GRAPH_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_GRAPH_H_

#include <VisibilityGraph/Graph.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace RotationalVisibilityGraph {
using T = double;

template<typename T>
void bind_graph(py::module &m) {
  using GraphT = Graph<T>;
  using VertexPtr = std::shared_ptr<Vertex<T>>;

  py::class_<GraphT>(m, "graph")
      .def(py::init<>())
      .def("addEdge", &GraphT::addEdge, py::arg("v1"), py::arg("v2"))
      .def("addEdges", &GraphT::addEdges, py::arg("edges"))
      .def("mergeGraph", &GraphT::mergeGraph, py::arg("other"))
      .def("getNeighbors",
           [](const GraphT &self, const VertexPtr &v) {
             std::vector<VertexPtr> neighbors;
             const auto &neighborSet = self.getNeighbors(v);
             neighbors.reserve(neighborSet.size());
             for (const auto &n : neighborSet) {
               neighbors.push_back(n);
             }
             return neighbors;
           },
           py::arg("v"))
      .def("getVertices", [](const GraphT &self) {
        std::vector<VertexPtr> vertices;
        const auto &vertexSet = self.getVertices();
        vertices.reserve(vertexSet.size());
        for (const auto &v : vertexSet) {
          vertices.push_back(v);
        }
        return vertices;
      })
      .def("getEdges", [](const GraphT &self) {
        std::vector<std::pair<Vertex<T>, Vertex<T>>> edges;
        const auto &edgeSet = self.getEdges();
        edges.reserve(edgeSet.size());
        for (const auto &e : edgeSet) {
          edges.emplace_back(e.v1(), e.v2());
        }
        return edges;
      })
      .def("size", &GraphT::size)
      .def("setWeight", &GraphT::setWeight, py::arg("euclideanWeight"), py::arg("rotationalWeight"))
      .def("shortestPath", &GraphT::shortestPath, py::arg("start"), py::arg("goal"), py::arg("verbose") = false)
      .def("numConnectedComponents", &GraphT::numConnectedComponents)
      .def("draw", py::overload_cast<const std::string &, bool>(&GraphT::draw), py::arg("figPath"), py::arg("show") = false)
      .def("draw3D", &GraphT::draw3D, py::arg("figPath"), py::arg("show") = false)
      .def("buildEdges", &GraphT::buildEdges)
      .def("drawScript", py::overload_cast<>(&GraphT::draw, py::const_));
}

void init_graph(py::module &m) { bind_graph<T>(m); }

}  // namespace RotationalVisibilityGraph

#endif  // ROTATIONALVISIBILITYGRAPH_PYTHON_GRAPH_H_
