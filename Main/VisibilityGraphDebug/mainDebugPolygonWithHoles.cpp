#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>

using namespace RotationalVisibilityGraph;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
int main() {
  std::vector<Vertex<double>> vertices = {
      Vertex<T>(2, -1),
      Vertex<T>(4, -1),
      Vertex<T>(4, -2),
      Vertex<T>(2, -2)
  };
  Polygon<double> polygon(vertices, true);
  std::vector<Vertex<double>> vertices1 = {
      Vertex<T>(2, 1),
      Vertex<T>(4, 1),
      Vertex<T>(4, 2),
      Vertex<T>(2, 2)
  };
  Polygon<double> polygon1 = Polygon(vertices1, true);
  T map_size = 10.;
  std::vector<Vertex<T>> vertices2 = {
      Vertex<T>(-map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., map_size / 2.),
      Vertex<T>(-map_size / 2., map_size / 2.)
  };
  Polygon<T> map = Polygon<T>(vertices2, true);
  std::vector<Polygon_2> polygons;
    polygons.push_back(polygon.getPolygon());
    polygons.push_back(polygon1.getPolygon());
  Polygon_with_holes_2 pwh(map.getPolygon(), polygons.begin(), polygons.end());
  CGAL::draw(pwh);

  return 0;
}
