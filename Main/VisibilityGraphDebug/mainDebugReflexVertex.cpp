#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>
#include <CGAL/Arrangement_2.h>
#include <Utils/Pragma.h>
#ifdef VISIBILITY_QUERY_TEV
#include <CGAL/Triangular_expansion_visibility_2.h>
#elif defined(VISIBILITY_QUERY_RSV)
#include <CGAL/Rotational_sweep_visibility_2.h>
#endif

DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

using namespace RotationalVisibilityGraph;
using T = double;

bool isReflex(const Vertex_const_handle &v, const Arrangement_2 & env);

int main() {
  std::vector<Point_2> vertices0 = {
      Point_2(-20, 0),
      Point_2(-16, 0),
      Point_2(-11, 5),
      Point_2(-4, 5),
      Point_2(1, 0),
      Point_2(6, 0),
      Point_2(11, -5),
      Point_2(14, -5),
      Point_2(20, 1),
      Point_2(20, 40),
      Point_2(-20, 40)
  };
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(-1, 0),
      Vertex<T>(-1, 2),
      Vertex<T>(0, 2),
      Vertex<T>(0, 0)
  };
  Polygon<T> robot(vertices, false);
  robot.moveTo(0, 0, 0);
  vertices = robot.getVertices();
  Polygon<T> robot1 = robot.rotateCopy(PI / 18);
  for (const auto &v : robot1.getVertices()) {
    vertices.push_back(v);
  }
  Polygon<T> robot2(vertices, true);

  std::vector<Point_2> vertices1;
  for (const auto &v : robot2.getVertices()) {
    vertices1.emplace_back(Point_2(v.getX(), v.getY()));
  }

  Polygon_2 polygon0(vertices0.begin(), vertices0.end());
  Polygon_2 polygon1(vertices1.begin(), vertices1.end());
  if (polygon0.is_clockwise_oriented()) {
    polygon0.reverse_orientation();
  }
  if (polygon1.is_clockwise_oriented()) {
    polygon1.reverse_orientation();
  }

  std::vector<Point_2> borderPoints = {
      Point_2(-45, -45),
      Point_2(45, -45),
      Point_2(45, 45),
      Point_2(-45, 45)
  };
  Polygon_2 border(borderPoints.begin(), borderPoints.end());

  Polygon_with_holes_2 sum = CGAL::minkowski_sum_by_reduced_convolution_2(polygon0, polygon1);
  std::cout << "Number of holes =" << sum.number_of_holes() << std::endl;
  Polygon_2 polygon2 = sum.outer_boundary();
  std::vector<Polygon_2> polygons = {polygon2, border};
  std::vector<Segment_2> segments;
  for (const Polygon_2 &polygon : polygons) {
    for (const auto &edge : polygon.edges()) {
      segments.push_back(edge);
    }
  }

  Arrangement_2 env;
  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

  std::vector<Point_2> reflexVertices;
  for(const auto & v: env.vertex_handles()){
    std::cout << "v=" << v->point() << " isReflex=" << isReflex(v, env) << std::endl;
    if(isReflex(v, env)){
      reflexVertices.push_back(v->point());
    }
  }

  std::string pythonScript = "import numpy as np\n";
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "\n";
  pythonScript += "fig, ax = plt.subplots()\n";
  pythonScript += "ax.set_facecolor('gainsboro')\n";
  pythonScript += "border = np.array([\n";
  for (const auto &v : borderPoints) {
    pythonScript += "\t[" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "],\n";
  }
  pythonScript += "\t[" + std::to_string(CGAL::to_double(borderPoints[0].x())) + ", " + std::to_string(CGAL::to_double(borderPoints[0].y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-g')\n";
  pythonScript += "border = np.array([\n";
  for (const auto &v : vertices0) {
    pythonScript += "\t[" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "],\n";
  }
  pythonScript += "\t[" + std::to_string(CGAL::to_double(vertices0[0].x())) + ", " + std::to_string(CGAL::to_double(vertices0[0].y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-g')\n";
  Polygon_2::Vertex_circulator first, curr, prev;
  Polygon_2 cleanPolygon;
  Polygon_2 tmpPolygon = polygon2;
  while (true) {
    bool foundSame = false;
    first = curr = tmpPolygon.vertices_circulator();
    std::cout << "first=" << first->x() << " " << first->y() << std::endl;
    std::vector<Point_2> tmpCleanedPolygon;
    bool firstAdded = false;
    do {
      Polygon_2::Vertex_circulator nextTwo = curr + 2;
      Polygon_2::Vertex_circulator next = curr + 1;
      std::cout << "curr=" << curr->x() << " " << curr->y() << " next=" << next->x() << " " << next->y() << " nextTwo=" << nextTwo->x() << " " << nextTwo->y() << std::endl;
      if (*next == *curr) {
        curr++;
        foundSame = true;
        tmpCleanedPolygon.push_back(*curr);
        std::cout << "************Found Next Same************\n";
        if (curr == first)
          break;
        curr++;
      } else if (*nextTwo == *curr) {
        std::cout << "************Found Next Two Same************\n";
        foundSame = true;
        if (next == first) {
          std::cout << "************Next is First************\n";
          if (firstAdded) {
            tmpCleanedPolygon.erase(tmpCleanedPolygon.begin());
            firstAdded = false;
          }
          break;
        }
        if (nextTwo == first) {
          std::cout << "************Next Two is First************\n";
          break;
        }
        curr += 2;
        std::cout << "Added curr=" << curr->x() << " " << curr->y() << std::endl;
        tmpCleanedPolygon.push_back(*curr);
        curr++;
      } else {
        tmpCleanedPolygon.push_back(*curr);
        if (curr == first) {
          firstAdded = true;
          std::cout << "************First Added************\n";
        }
        curr++;
      }
    } while (curr != first);
    tmpPolygon = Polygon_2(tmpCleanedPolygon.begin(), tmpCleanedPolygon.end());
    std::cout << "foundSame=" << foundSame << std::endl;
    if (!foundSame) {
      cleanPolygon = tmpPolygon;
      break;
    }
  }
  first = curr = cleanPolygon.vertices_circulator();
  pythonScript += "sum = np.array([\n";
  do {
    std::cout << "x=" << curr->x() << " y=" << curr->y() << std::endl;
    pythonScript += "\t[" + std::to_string(CGAL::to_double(curr->x())) + ", " + std::to_string(CGAL::to_double(curr->y())) + "],\n";
    curr++;
  } while (curr != first);
  pythonScript += "\t[" + std::to_string(CGAL::to_double(first->x())) + ", " + std::to_string(CGAL::to_double(first->y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(sum[:, 0], sum[:, 1], '-o', color='deeppink', markersize=0.5, linewidth=0.5)\n";

  for(const auto &v: reflexVertices){
    pythonScript += "ax.plot(" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + ", 'ro')\n";
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
};

bool isReflex(const Vertex_const_handle &v, const Arrangement_2 & env){
  Point_2 vl, vr;
  typename Arrangement_2::Halfedge_around_vertex_const_circulator first, curr;
  first = curr = v->incident_halfedges();
  while (true) {
    if (curr->is_on_outer_ccb())
      vl = curr->source()->point();
    if (curr->is_on_inner_ccb())
      vr = curr->source()->point();
    curr++;
    if (curr == first)
      break;
  }
  bool isReflex = CGAL::right_turn(vl, v->point(), vr);
  return isReflex;
}
