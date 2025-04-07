#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_set_2.h>

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

int main() {
  std::vector<Point_2> points = {
      Point_2(-5, -5),
      Point_2(5, -5),
      Point_2(5, 5),
      Point_2(-5, 5)
  };
  Polygon_2 polygon(points.begin(), points.end());
  std::vector<Point_2> points1 = {
      Point_2(0, -1),
      Point_2(1, -1),
      Point_2(1, 0),
      Point_2(0, 0)
  };
  Polygon_2 polygon1(points1.begin(), points1.end());
  Polygon_set_2 s;
  s.insert(polygon);
  s.insert(polygon1);
  CGAL::draw(s);
  std::vector<Polygon_with_holes_2> intersections;
  Utils::print("intersection?", CGAL::do_intersect(polygon, polygon1));
  CGAL::intersection(polygon, polygon1, std::back_inserter(intersections));
  print(intersections.size());
  for (const auto &intersection : intersections) {
    print(intersection.outer_boundary());
    CGAL::draw(intersection);
  }
  return 0;
}

