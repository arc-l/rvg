#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#ifdef VISIBILITY_QUERY_TEV
#include <CGAL/Triangular_expansion_visibility_2.h>
#elif defined(VISIBILITY_QUERY_RSV)
#include <CGAL/Rotational_sweep_visibility_2.h>
#endif
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <iostream>
#include <vector>
#include <VisibilityGraph/VisibilityCheck.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>

// Define the used kernel and arrangement
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;
using T = double;
int main() {
  // Defining the input geometry
  std::vector<Point_2> polygon1 = {Point_2(0, 0), Point_2(1, 0), Point_2(1, 1), Point_2(0, 1)};
  std::vector<Point_2> polygon2 = {Point_2(-2, 0), Point_2(-1, 0), Point_2(-1, 1), Point_2(-2, 1)};
  std::vector<Point_2> border1 = {Point_2(-3, -1), Point_2(4, -1), Point_2(4, 3), Point_2(-3, 3)};
  std::vector<Point_2> polygon3 = {Point_2(0, -0.5), Point_2(1, -0.5), Point_2(1, 0.5), Point_2(0, 0.5)};
  std::vector<Point_2> polygon4 = {Point_2(2, -0.5), Point_2(3, -0.5), Point_2(3, 0.5), Point_2(2, 0.5)};
  std::vector<Point_2> border2 = {Point_2(-3, -1), Point_2(4, -1), Point_2(4, 3), Point_2(-3, 3)};
  std::vector<Polygon_2> polygons1 = {Polygon_2(polygon1.begin(), polygon1.end()), Polygon_2(polygon2.begin(), polygon2.end())};
  std::vector<Polygon_2> polygons2 = {Polygon_2(polygon3.begin(), polygon3.end()), Polygon_2(polygon4.begin(), polygon4.end())};

  std::vector<Polygon_with_holes_2> polygonsJoined;
  CGAL::join(polygons1.begin(), polygons1.end(), polygons2.begin(), polygons2.end(), std::back_inserter(polygonsJoined));
  std::vector<Polygon_2> polygons;
  for (auto &p : polygonsJoined) {
    polygons.push_back(p.outer_boundary());
  }

  Polygon_with_holes_2 borderSum;
  CGAL::join(Polygon_2(border1.begin(), border1.end()), Polygon_2(border2.begin(), border2.end()), borderSum);
  polygons.push_back(borderSum.outer_boundary());

  // insert geometry into the arrangement
  std::vector<Segment_2> segments;
  for (auto &p : polygons) {
    if (p.is_clockwise_oriented()) {
      p.reverse_orientation();
    }
    for (auto edge = p.edges_begin(); edge != p.edges_end(); ++edge) {
      segments.emplace_back(edge->source(), edge->target());
    }
  }

  // Draw the geometry
  std::string pythonScript;
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "import numpy as np\n";
  pythonScript += "segments = np.array([";
  for (auto &segment : segments) {
    pythonScript += "[[" + std::to_string(CGAL::to_double(segment.source().x())) + ", " + std::to_string(CGAL::to_double(segment.source().y())) + "], ";
    pythonScript += "[" + std::to_string(CGAL::to_double(segment.target().x())) + ", " + std::to_string(CGAL::to_double(segment.target().y())) + "]], ";
  }
  pythonScript += "])\n";
  pythonScript += "for segment in segments:\n";
  pythonScript += "    plt.plot([segment[0][0], segment[1][0]], [segment[0][1], segment[1][1]], 'k-')\n";

//  Arrangement_2 env = obsToArrangement<T>(polygons);
  Arrangement_2 env;
  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());
  CGAL::draw(env);
  std::shared_ptr<VQ> vq = std::make_shared<VQ>(env);
  std::vector<Point_2> allPoints = polygon1;
  allPoints.insert(allPoints.end(), polygon2.begin(), polygon2.end());
  allPoints.insert(allPoints.end(), polygon3.begin(), polygon3.end());
  allPoints.insert(allPoints.end(), polygon4.begin(), polygon4.end());
  for (const auto &v : allPoints) {
    std::vector<Point_2> visibleVerticesAll = visibleVerticesOnHoles<T>(v, env, vq);
    // draw connections of v and the visible vertices
    pythonScript += "v = [" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "]\n";
    for (auto &visibleVertex : visibleVerticesAll) {
      pythonScript += "plt.plot([v[0], " + std::to_string(CGAL::to_double(visibleVertex.x())) + "], [v[1], " + std::to_string(CGAL::to_double(visibleVertex.y())) + "], 'ro-')\n";
    }
  }

  Point_2 v(0.5, 0.5);
  std::vector<Point_2> visibleVertices = visibleVerticesOnHoles<T>(v, env, vq);
  // draw connections of v and the visible vertices
  pythonScript += "v = [" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "]\n";
  for (auto &visibleVertex : visibleVertices) {
    pythonScript += "plt.plot([v[0], " + std::to_string(CGAL::to_double(visibleVertex.x())) + "], [v[1], " + std::to_string(CGAL::to_double(visibleVertex.y())) + "], 'go-')\n";
  }

  v = Point_2 (2,2);
  visibleVertices = visibleVerticesOnHoles<T>(v, env, vq);
  // draw connections of v and the visible vertices
  pythonScript += "v = [" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "]\n";
  for (auto &visibleVertex : visibleVertices) {
    pythonScript += "plt.plot([v[0], " + std::to_string(CGAL::to_double(visibleVertex.x())) + "], [v[1], " + std::to_string(CGAL::to_double(visibleVertex.y())) + "], 'go-')\n";
  }

  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  std::string pythonSavePath = "draw.py";
  std::ofstream file(pythonSavePath);
  file << pythonScript;
  file.close();

#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  system(command.c_str());

  return 0;
}
