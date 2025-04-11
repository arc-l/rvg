#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_LAYER_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_LAYER_H_

#include <VisibilityGraph/Layer.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <memory>

namespace py = pybind11;

namespace RotationalVisibilityGraph {
using T = double;

template<typename T>
void bind_layer(py::module &m) {
    py::class_<Layer<T>>(m, "Layer")
        // Constructors
        .def(py::init<>())
        .def(py::init([](T theta_lb, 
                         T theta_ub, 
                         T roundUpTheta, 
                         bool fineApprox, 
                         bool simplifiedGeometry) {
            try{
                return new Layer<T>(theta_lb, theta_ub, roundUpTheta, fineApprox, simplifiedGeometry);
            } catch (const std::exception &e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                throw py::error_already_set();
            }
        }),
             py::arg("theta_lb"), 
             py::arg("theta_ub"), 
             py::arg("roundUpTheta"),
             py::arg("fineApprox"), 
             py::arg("simplifiedGeometry")
             )

        .def(py::init<const Layer<T>&>(), py::arg("layer"))

        // Member Functions
        .def("buildVisibilityGraph", &Layer<T>::buildVisibilityGraph, 
             py::arg("robot"), py::arg("border"), py::arg("obstacles"))

        .def("getThetaLb", &Layer<T>::getThetaLb, py::return_value_policy::reference)
        .def("getThetaUb", &Layer<T>::getThetaUb, py::return_value_policy::reference)
        .def("getVertex", &Layer<T>::getVertex, py::arg("index"))
        .def("getGrownObs", &Layer<T>::getGrownObsRVGPolygon)
        .def("getHoles", &Layer<T>::getHolesPolygon)
        .def("getShrinkedBorder", &Layer<T>::getShrinkedBorder)
        .def("hasHoles", &Layer<T>::hasHoles)
        .def("borderIsHole", &Layer<T>::borderIsHole)
        .def("legalConfig", &Layer<T>::legalConfig, py::arg("v"))
        // .def("getVisibleArea", &Layer<T>::getVisibleArea, py::arg("v"))
        .def("getEdges", &Layer<T>::getEdges)
        // .def("getNeighbor", &Layer<T>::getNeighbor, py::arg("v"))
        .def("isVertexOnHole", &Layer<T>::isVertexOnHole, py::arg("vertex"))
        .def("isFeasible", &Layer<T>::isFeasible)
        .def("getRobotBBox", &Layer<T>::getRobotBBox)
        .def("getRobotBBoxInverted", &Layer<T>::getRobotBBoxInverted)
        .def("getVisibleAreaPolygons", &Layer<T>::getVisibleAreaPolygons)
        .def("getVisibleAreaPolygon", &Layer<T>::getVisibleAreaPolygon, py::arg("x"), py::arg("y"))
        ;
}

// Explicit instantiation of the binding for double
void init_layer(py::module &m) { bind_layer<T>(m); }

}  // namespace RotationalVisibilityGraph

#endif  // ROTATIONALVISIBILITYGRAPH_PYTHON_LAYER_H_
