#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>

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
  Polygon<T> polygon1(vertices, true);
  std::vector<Vertex<T>> vertices1 = {
      Vertex<T>(0, 0),
      Vertex<T>(1, 0),
      Vertex<T>(1, 1),
      Vertex<T>(0, 1)
  };
  Polygon<T> polygon = Polygon(vertices1, true);
  polygon.draw("../Results/polygon.png", "polygon", true);
  polygon1.draw("../Results/polygon.png", "polygon1", true);
  polygon1.moveTo(2, 2, 0.78);
  polygon1.draw("../Results/polygon.png", "polygon1", true);

  Polygon<T> polygon2 = polygon1.rotateCopy(0);
  polygon2.draw("../Results/polygon.png", "polygon2", true);

  polygon1.invert();

  // debug is convex
  std::vector<Vertex<T>> concave_vertices = {
      Vertex<T>(0, 0),
      Vertex<T>(1, 0),
      Vertex<T>(0.5, 0.5),
      Vertex<T>(1, 1),
      Vertex<T>(0, 1)
  };
  Polygon<T> polygon_concave = Polygon<T>(concave_vertices, true);
  Utils::print("Is convex", polygon_concave.isConvex());
  polygon_concave.draw("../Results/polygon_concave.png", "concave polygon", true);

  auto convex_polygons = polygon_concave.convexDecomposition();
  for (auto &p : convex_polygons) {
    p.draw("../Results/polygon_convex.png", "convex component", true);
  }

  return 0;
}