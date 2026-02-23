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

    py::class_<VisibilityGraph<T>>(m, "rvg")
        // Constructor with exception handling
        .def(py::init([](const Polygon<T> &robot,
                         const Polygon<T> &border,
                         const std::vector<Polygon<T>> &obstacles,
                         int resolution,
                         bool fineApprox,
                         int numThreads,
                         bool optimal = false,
                         bool verbose = false) {
            try {
                return new VisibilityGraph<T>(robot, border, obstacles, resolution, fineApprox, numThreads, optimal, verbose);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }),
        py::arg("robot"),
        py::arg("border"),
        py::arg("obstacles"),
        py::arg("resolution"),
        py::arg("fineApprox"),
        py::arg("numThreads"),
        py::arg("optimal") = false,
        py::arg("verbose") = false,
        "Constructs a VisibilityGraph with the specified parameters.")

        // Methods with exception handling
        .def("setWeight", [](VisibilityGraph<T> &self, T euclideanWeight, T rotationalWeight) {
            try {
                self.setWeight(euclideanWeight, rotationalWeight);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }, py::arg("euclideanWeight"), py::arg("rotationalWeight"))

        .def("shortestPath", [](VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal, int interpolationDensity, bool unwrap) {
            try {
                return self.shortestPath(start, goal, interpolationDensity, unwrap);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }, py::arg("start"), py::arg("goal"), py::arg("interpolationDensity") = 0, py::arg("unwrap") = true)

        .def("getPathLength", [](const VisibilityGraph<T> &self) {
            try {
                return self.getPathLength();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })

        .def("getTotalRotation", [](const VisibilityGraph<T> &self) {
            try {
                return self.getTotalRotation();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })

        .def("getTotalTime", [](const VisibilityGraph<T> &self) {
            try {
                return self.getTotalTime();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })

        .def("getBuildTime", [](const VisibilityGraph<T> &self) {
            try {
                return self.getBuildTime();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })

        .def("getSearchTime", [](const VisibilityGraph<T> &self) {
            try {
                return self.getSearchTime();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })

        .def("draw", [](VisibilityGraph<T> &self, const std::string &figPath, bool title, bool path, bool graph, bool show) {
            try {
                self.draw(figPath, title, path, graph, show);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }, py::arg("figPath"), py::arg("title") = true, py::arg("path") = true, py::arg("graph") = true, py::arg("show") = false)

        .def("animation", [](VisibilityGraph<T> &self, const std::string &figPath, bool title) {
            try {
                self.animation(figPath, title);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }, py::arg("figPath"), py::arg("title") = true)

        .def("debugMaxPropagation", [](VisibilityGraph<T> &self) {
            try {
                self.debugMaxPropagation();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })
        .def("getLayers", [](const VisibilityGraph<T> &self) {
            try {
                return self.getLayers();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })
        .def("getEdgeLayersForward", [](const VisibilityGraph<T> &self) {
            try {
                return self.getEdgeLayersForward();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })
        .def("getEdgeLayersBackward", [](const VisibilityGraph<T> &self) {
            try {
                return self.getEdgeLayersBackward();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })
        .def("getAdjacentVertices", [](const VisibilityGraph<T> &self, std::shared_ptr<Vertex<T>> v) {
            try {
                return self.getAdjacentVertices(v);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }, py::arg("v"))
        .def("verticalDecomposition", [](VisibilityGraph<T> &self) {
            try {
                return self.verticalDecomposition();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        })
        .def("getGraphVertices", [](const VisibilityGraph<T> &self) {
            try {
                return self.getGraphVertices();
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
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
