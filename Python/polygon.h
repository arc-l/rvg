#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_POLYGON_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_POLYGON_H_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <VisibilityGraph/VisibilityGraph.h> // Include your original header file

namespace py = pybind11;

namespace RotationalVisibilityGraph {
using T = double;
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

template<typename T>
void bind_polygon(py::module &m) {
  using PolygonT = Polygon<T>;

  py::class_<PolygonT>(m, "polygon")
      .def(py::init<>())  // Default constructor
      .def(py::init<const std::vector<Vertex<T>> &, bool, const Vertex<T> &>(),
           py::arg("vertices"),
           py::arg("sortVertices"),
           py::arg("center"))

      .def(py::init<const std::vector<Vertex<T>> &, bool>(),
           py::arg("vertices"),
           py::arg("sortVertices"))

      .def(py::init<const PolygonT &>())
      .def(py::init<const typename PolygonT::Polygon_2 &>())
      .def(py::init<const typename PolygonT::Polygon_2 &, const Vertex<T> &>())

          // Member functions
      .def("size", &PolygonT::size)
      .def("__getitem__", py::overload_cast<int>(&PolygonT::operator[], py::const_))
      .def("__setitem__", py::overload_cast<int>(&PolygonT::operator[]))
      .def("translate", &PolygonT::translate)
      .def("rotate", py::overload_cast<T>(&PolygonT::rotate))
      .def("rotate", py::overload_cast<T, const Vertex<T> &>(&PolygonT::rotate))
      .def("rotateCopy", py::overload_cast<T>(&PolygonT::rotateCopy, py::const_))
      .def("rotateCopy", py::overload_cast<T, const Vertex<T> &>(&PolygonT::rotateCopy, py::const_))
      .def("rotateCGALCopy", &PolygonT::rotateCGALCopy)
      .def("moveTo", &PolygonT::moveTo)
      .def("moveToCopy", &PolygonT::moveToCopy)
      .def("isConvex", &PolygonT::isConvex)
      .def("draw", py::overload_cast<Polygon_set_2 &>(&PolygonT::draw, py::const_))
      .def("draw", py::overload_cast<>(&PolygonT::draw, py::const_))
      .def("invert", &PolygonT::invert)
      .def("getPolygon", &PolygonT::getPolygon, py::return_value_policy::reference)
      .def("getVertices", &PolygonT::getVertices, py::return_value_policy::reference)
      .def("scale", &PolygonT::scale)
      .def("getClockWise", &PolygonT::getClockWise)
      .def("getCounterClockWise", &PolygonT::getCounterClockWise)
      .def("getComplement", &PolygonT::getComplement)
      .def("getCentroid", &PolygonT::getCentroid, py::return_value_policy::reference)
      .def("getX", &PolygonT::getX)
      .def("getY", &PolygonT::getY)
      .def("intersects", &PolygonT::intersects)
      .def("intersectsEdge", &PolygonT::intersectsEdge)
      .def("contains", &PolygonT::contains)
      .def("__eq__", &PolygonT::operator==)
      .def("bbox", &PolygonT::bbox)
      .def("merge", &PolygonT::merge)
      .def("convexDecomposition", &PolygonT::convexDecomposition)
      .def("draw", py::overload_cast<const std::string &, const std::string &, bool>(&PolygonT::draw, py::const_))
      .def("draw", py::overload_cast<const std::string &, bool>(&PolygonT::draw, py::const_))

          // Print method for __repr__
      .def("__repr__",
           [](const PolygonT &polygon) {
             return "<rvg.Polygon>";
           });
}

// Explicitly instantiate the bindings for float and double
void init_polygon(py::module &m) {
  bind_polygon<T>(m);
}

}  // namespace RotationalVisibilityGraph


#endif //ROTATIONALVISIBILITYGRAPH_PYTHON_POLYGON_H_
