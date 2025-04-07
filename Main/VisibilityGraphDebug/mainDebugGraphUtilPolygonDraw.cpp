#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/GraphUtils.h>

using namespace RotationalVisibilityGraph;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

int main() {
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(-2, -2),
      Vertex<T>(-0, -1),
      Vertex<T>(-1, -1),
      Vertex<T>(-1, -0),
  };
  std::shared_ptr<Polygon<T>> polygon = std::make_shared<Polygon<T>>(vertices, true);
  std::vector<Vertex<T>> vertices1 = {
      Vertex<T>(0, 0),
      Vertex<T>(1, 0),
      Vertex<T>(1, 1),
      Vertex<T>(0, 1)
  };
  std::shared_ptr<Polygon<T>> polygon1 = std::make_shared<Polygon<T>>(vertices1, true);

  std::vector<std::shared_ptr<Polygon<T>>> polygons = {polygon, polygon1};
  PolygonDrawer<T> drawer(polygons);
  drawer.draw();
  polygon1->moveTo(2, 2, 0.78);
  drawer.draw();
  polygon1->invert();
  drawer.draw();

  return 0;
}
