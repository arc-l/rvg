#include <Python/vertex.h>
#include <Python/polygon.h>
#include <Python/visibility_graph.h>
#include <Python/layer.h>
#include <Python/graph.h>
#include <Python/utils.h>

PYBIND11_MODULE(rvg, m) {
  RotationalVisibilityGraph::init_vertex(m);
  RotationalVisibilityGraph::init_polygon(m);
  RotationalVisibilityGraph::init_layer(m);
  RotationalVisibilityGraph::init_graph(m);
  RotationalVisibilityGraph::init_visibility_graph(m);
  RotationalVisibilityGraph::Utils::init_utils(m);
  m.doc() = "Rotation-stacked visibility graphs for polygonal motion planning.";
  m.attr("__version__") = "0.1.0";
}
