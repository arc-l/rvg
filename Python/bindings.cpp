#include <Python/vertex.h>
#include <Python/polygon.h>
#include <Python/visibility_graph.h>
#include <Python/layer.h>
#include <Python/utils.h>

int add(int i, int j) {
  return i + j;
}

PYBIND11_MODULE(rvg, m) {
  RotationalVisibilityGraph::init_vertex(m);
  RotationalVisibilityGraph::init_polygon(m);
  RotationalVisibilityGraph::init_layer(m);
  RotationalVisibilityGraph::init_visibility_graph(m);
  RotationalVisibilityGraph::Utils::init_utils(m);
  m.doc() = "pybind11 example plugin"; // optional module docstring
  m.def("add", &add, "A function that adds two numbers");
}
