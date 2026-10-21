#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_VISIBILITY_GRAPH_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_VISIBILITY_GRAPH_H_

#include <VisibilityGraph/VisibilityGraph.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace RotationalVisibilityGraph {
using T = double;

template<typename T>
void bind_visibility_graph(py::module &m) {
    py::class_<typename VisibilityGraph<T>::TwoTuple>(m, "TwoTuple")
        .def(py::init<>())  // Default constructor
        .def_readwrite("first", &VisibilityGraph<T>::TwoTuple::first)
        .def_readwrite("second", &VisibilityGraph<T>::TwoTuple::second)
        .def("__eq__", &VisibilityGraph<T>::TwoTuple::operator==)
        .def("__repr__",
             [](const typename VisibilityGraph<T>::TwoTuple &tuple) {
                 return "TwoTuple(" + std::to_string(tuple.first) + ", " + std::to_string(tuple.second) + ")";
             });

    py::enum_<VertexPropagationBackend>(m, "VertexPropagationBackend")
        .value("Auto", VertexPropagationBackend::Auto)
        .value("Cpu", VertexPropagationBackend::Cpu)
        .value("CpuReference", VertexPropagationBackend::CpuReference)
        .value("Cuda", VertexPropagationBackend::Cuda);
    m.def("cudaVertexPropagationAvailable", &cudaVertexPropagationAvailable);

    py::class_<VisibilityGraph<T>>(m, "rvg")
        .def(py::init([](const Polygon<T> &robot,
                         const Polygon<T> &border,
                         const std::vector<Polygon<T>> &obstacles,
                         int resolution,
                         int numThreads,
                         bool optimal = false,
                         bool verbose = false,
                         VertexPropagationBackend propagationBackend = VertexPropagationBackend::Auto) {
            return new VisibilityGraph<T>(robot, border, obstacles, resolution, numThreads, optimal, verbose, propagationBackend);
        }),
        py::arg("robot"),
        py::arg("border"),
        py::arg("obstacles"),
        py::arg("resolution"),
        py::arg("numThreads"),
        py::arg("optimal") = false,
        py::arg("verbose") = false,
        py::arg("propagationBackend") = VertexPropagationBackend::Auto,
        "Constructs a VisibilityGraph with the specified parameters.")


        .def("getVertexPropagationStatistics", [](const VisibilityGraph<T> &self) {
            const auto &stats = self.getVertexPropagationStatistics();
            py::dict result;
            result["requestedBackend"] = stats.requestedBackend;
            result["effectiveBackend"] = stats.effectiveBackend;
            result["forwardRounds"] = stats.forwardRounds;
            result["backwardRounds"] = stats.backwardRounds;
            result["candidatePairs"] = stats.candidatePairs;
            result["cpuExactCandidates"] = stats.cpuExactCandidates;
            result["cpuCertifiedAccepts"] = stats.cpuCertifiedAccepts;
            result["cpuCertifiedRejects"] = stats.cpuCertifiedRejects;
            result["cpuUncertainCandidates"] = stats.cpuUncertainCandidates;
            result["cudaCertifiedAccepts"] = stats.cudaCertifiedAccepts;
            result["cudaCertifiedRejects"] = stats.cudaCertifiedRejects;
            result["cpuExactFallbacks"] = stats.cpuExactFallbacks;
            result["flattenMilliseconds"] = stats.flattenMilliseconds;
            result["scalarFilterMilliseconds"] = stats.scalarFilterMilliseconds;
            result["cpuMilliseconds"] = stats.cpuMilliseconds;
            result["transferMilliseconds"] = stats.transferMilliseconds;
            result["kernelMilliseconds"] = stats.kernelMilliseconds;
            result["exactFallbackMilliseconds"] = stats.exactFallbackMilliseconds;
            result["materializationMilliseconds"] = stats.materializationMilliseconds;
            result["totalMilliseconds"] = stats.totalMilliseconds;
            result["cudaFallback"] = stats.cudaFallback;
            result["cudaError"] = stats.cudaError;
            return result;
        })
        .def("getConstructionStatistics", [](const VisibilityGraph<T> &self) {
            const auto &stats = self.getConstructionStatistics();
            py::dict result;
            result["layerBuildMilliseconds"] = stats.layerBuildMilliseconds;
            result["nativeMaterializationMilliseconds"] = stats.nativeMaterializationMilliseconds;
            result["propagationMilliseconds"] = stats.propagationMilliseconds;
            result["finalizationMilliseconds"] = stats.finalizationMilliseconds;
            result["totalMilliseconds"] = stats.totalMilliseconds;
            return result;
        })

        .def("setWeight", [](VisibilityGraph<T> &self, T euclideanWeight, T rotationalWeight) {
            self.setWeight(euclideanWeight, rotationalWeight);
        }, py::arg("euclideanWeight"), py::arg("rotationalWeight"))

        .def("shortestPath", [](VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal, int interpolationDensity, bool unwrap) {
            return self.shortestPath(start, goal, interpolationDensity, unwrap);
        }, py::arg("start"), py::arg("goal"), py::arg("interpolationDensity") = 0, py::arg("unwrap") = true)
        .def("addStartAndGoal", [](VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal) {
            self._addStartAndGoal(start, goal);
        }, py::arg("start"), py::arg("goal"))
        .def("addVertex", [](VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> vertex) {
            return self.addVertex(vertex);
        }, py::arg("vertex"))

        .def("getPathLength", [](const VisibilityGraph<T> &self) {
            return self.getPathLength();
        })

        .def("getTotalRotation", [](const VisibilityGraph<T> &self) {
            return self.getTotalRotation();
        })

        .def("getTotalTime", [](const VisibilityGraph<T> &self) {
            return self.getTotalTime();
        })

        .def("getBuildTime", [](const VisibilityGraph<T> &self) {
            return self.getBuildTime();
        })

        .def("getSearchTime", [](const VisibilityGraph<T> &self) {
            return self.getSearchTime();
        })

        .def("draw", [](VisibilityGraph<T> &self, const std::string &figPath, bool title, bool path, bool graph, bool show) {
            self.draw(figPath, title, path, graph, show);
        }, py::arg("figPath"), py::arg("title") = true, py::arg("path") = true, py::arg("graph") = true, py::arg("show") = false)

        .def("animation", [](VisibilityGraph<T> &self, const std::string &figPath, bool title) {
            self.animation(figPath, title);
        }, py::arg("figPath"), py::arg("title") = true)

        .def("getLayers", [](const VisibilityGraph<T> &self) {
            return self.getLayers();
        })
        .def("getEdgeLayersForward", [](const VisibilityGraph<T> &self) {
            return self.getEdgeLayersForward();
        })
        .def("getEdgeLayersBackward", [](const VisibilityGraph<T> &self) {
            return self.getEdgeLayersBackward();
        })
        .def("getAdjacentVertices", [](const VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> v) {
            return self.getAdjacentVertices(v);
        }, py::arg("v"))
        .def("verticalDecomposition", [](VisibilityGraph<T> &self) {
            return self.verticalDecomposition();
        })
        .def("getGraphVertices", [](const VisibilityGraph<T> &self) {
            return self.getGraphVertices();
        })
        .def("getGraph",
             py::overload_cast<>(&VisibilityGraph<T>::getGraph),
             py::return_value_policy::reference_internal)
        ;
        
}
// Explicitly instantiate the bindings for float and double
void init_visibility_graph(py::module &m) { bind_visibility_graph<T>(m); }

}  // namespace RotationalVisibilityGraph

#endif  // ROTATIONALVISIBILITYGRAPH_PYTHON_VISIBILITY_GRAPH_H_
