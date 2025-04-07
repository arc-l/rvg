#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type> Convex_hull_traits_2;
using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main() {
  std::vector<Point_2> vertices0 = {
      Point_2(0, 0),
      Point_2(4, 0),
      Point_2(4, 4),
      Point_2(3, 4),
      Point_2(3, 1),
      Point_2(2, 1),
      Point_2(2, 4),
      Point_2(0, 4)
  };
  Polygon_2 polygon0(vertices0.begin(), vertices0.end());
  Polygon<T> polygon00(polygon0);
  polygon00.draw("polygon00.png", "", true);
  std::vector<Point_2> vertices1 = {
      Point_2(0, 0),
      Point_2(4, 0),
      Point_2(4, 4),
      Point_2(2, 4),
      Point_2(2, 1),
      Point_2(1, 1),
      Point_2(1, 4),
      Point_2(0, 4)
  };
  Polygon_2 polygon1(vertices1.begin(), vertices1.end());
  Polygon<T> polygon11(polygon1);
  polygon11.draw("polygon11.png", "", true);
  Polygon_with_holes_2 join0;
  CGAL::join(polygon0, polygon1, join0);
  Polygon_2 joinPoly0 = join0.outer_boundary();
  Polygon<T> joinPolygon0(joinPoly0);
  joinPolygon0.draw("joinPolygon0.png", "", true);
  // complement
  Polygon_with_holes_2 complement0, complement1, join1;
  CGAL::complement(polygon0, complement0);
  CGAL::complement(polygon1, complement1);
  CGAL::join(complement0, complement1, join1);
  Polygon_2 joinPoly1 = *join1.holes_begin();
  Polygon<T> joinPolygon1(joinPoly1);
  joinPolygon1.draw("joinPolygon1.png", "", true);
  return 0;
}
