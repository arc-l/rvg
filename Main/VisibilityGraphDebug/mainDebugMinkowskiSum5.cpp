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

int main()
{
  std::vector<Point_2> vertices0 = {
      Point_2(-10, 10),
      Point_2(-5, 10),
      Point_2(-5, 6),
      Point_2(5, 6),
      Point_2(5, 10),
      Point_2(10, 10),
      Point_2(10, -10),
      Point_2(-10, -10)};
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(-1, 0),
      Vertex<T>(-1, 2),
      Vertex<T>(0, 2),
      Vertex<T>(0, 0)};
  Polygon<T> robot(vertices, false);
  robot.moveTo(0, 0, 0);
  vertices = robot.getVertices();
  Polygon<T> robot1 = robot.rotateCopy(PI / 6);
  for (const auto &v : robot1.getVertices())
  {
    vertices.push_back(v);
  }
  Polygon<T> robot2(vertices, true);
  robot2.moveTo(0, 0, 0);
  std::vector<Point_2> vertices1;
  for (const auto &v : robot2.getVertices())
  {
    vertices1.emplace_back(Point_2(v.getX(), v.getY()));
  }

  Polygon_2 polygon0(vertices0.begin(), vertices0.end());
  Polygon_2 polygon1(vertices1.begin(), vertices1.end());
  Polygon<T> robotSum(polygon1);

  if (polygon0.is_clockwise_oriented())
  {
    polygon0.reverse_orientation();
  }
  if (polygon1.is_clockwise_oriented())
  {
    polygon1.reverse_orientation();
  }

  // complement of polygon0
  Polygon_with_holes_2 complement;
  CGAL::complement(polygon0, complement);

  Polygon_with_holes_2 sum = CGAL::minkowski_sum_by_reduced_convolution_2(complement, polygon1);
  print("Number of holes: ", sum.number_of_holes());
  Polygon_2 polygon2 = *sum.holes_begin();
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  Polygon<T> border(polygon0);
  pythonScript += border.draw("obs");
  Polygon<T> polygonSum(polygon2);
  pythonScript += polygonSum.draw("start");
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('Results/sum.png', dpi=500)\n";
  pythonScript += "plt.show()\n";
  {
    std::string pythonSavePath = "build/draw.py";
    std::ofstream file(pythonSavePath);
    file << pythonScript;
    file.close();
#ifdef PYTHON_EXECUTABLE
    std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
    std::string command = "python3 " + pythonSavePath;
#endif
    system(command.c_str());
  }
  return 0;
}