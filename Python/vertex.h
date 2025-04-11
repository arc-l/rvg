#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_VERTEX_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_VERTEX_H_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <VisibilityGraph/Vertex.h>

namespace py = pybind11;

namespace RotationalVisibilityGraph {

template<typename T>
void bind_vertex(py::module &m) {
  using VertexT = Vertex<T>;

  py::class_<VertexT, std::shared_ptr<VertexT>>(m, "vertex")
      .def(py::init<>())  // Default constructor
      .def(py::init<T, T>())  // Constructor with x, y
      .def(py::init<T, T, T, T, T>(),
          py::arg("x"), py::arg("y"), py::arg("theta_lb"), py::arg("theta_ub"), py::arg("theta"), 
          "Constructor with all parameters" 
          )  // Constructor with all params

          // Bind member functions
      .def("hash", &VertexT::hash)
      .def("setTheta", &VertexT::setTheta)
      .def("setBounds", &VertexT::setBounds)
      .def("setPos", &VertexT::setPos)
      .def("getX", &VertexT::getX)
      .def("getY", &VertexT::getY)
      .def("getTheta", &VertexT::getTheta)
      .def("getThetaLb", &VertexT::getThetaLb)
      .def("getThetaUb", &VertexT::getThetaUb)
      .def("getCoord", &VertexT::getCoord)
      // .def("getPoint", &VertexT::getPoint)
      .def("dist", &VertexT::dist)
      .def("rotationalDist", &VertexT::rotationalDist)
      .def("hasTheta", &VertexT::hasTheta)

          // Operators
      .def("__eq__", &VertexT::operator==)
      .def("__ne__", &VertexT::operator!=)
      .def("__lt__", &VertexT::operator<)
      .def("__add__", &VertexT::operator+)

          // Print method for __repr__
      .def("__repr__",
           [](const VertexT &vertex) {
             std::stringstream ss;
             ss << vertex;
             return ss.str();
           });

}

// Explicitly instantiate the bindings for float and double
void init_vertex(py::module &m) {
  bind_vertex<double>(m);
}

}  // namespace RotationalVisibilityGraph

#endif //ROTATIONALVISIBILITYGRAPH_PYTHON_VERTEX_H_
