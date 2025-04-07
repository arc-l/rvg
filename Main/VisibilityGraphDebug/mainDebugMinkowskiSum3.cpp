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
  std::string robotPoint_2 = "std::vector<Point_2> vertices1 = {\n";
  for (const auto &v : robot2.getVertices())
  {
    vertices1.emplace_back(Point_2(v.getX(), v.getY()));
    robotPoint_2 += "Point_2(" + std::to_string(v.getX()) + ", " + std::to_string(v.getY()) + "),\n";
  }
  robotPoint_2 += "};\n";
  Utils::print(robotPoint_2);

  Polygon_2 polygon0(vertices0.begin(), vertices0.end());
  Polygon_2 polygon1(vertices1.begin(), vertices1.end());
  Polygon<T> robotSum(polygon1);
  std::string robotDrawScript;
  PYTHON_IMPORTS(robotDrawScript)
  robotDrawScript += robotSum.draw("start");
  robotDrawScript += "plt.axis('equal')\n";
  robotDrawScript += "plt.savefig('Results/robotSum.png', dpi=500)\n";
  robotDrawScript += "plt.show()\n";
  std::string robotDrawPath = "build/drawRobot.py";
  {
    std::ofstream file(robotDrawPath);
    file << robotDrawScript;
    file.close();
#ifdef PYTHON_EXECUTABLE
    std::string command = std::string(PYTHON_EXECUTABLE) + " " + robotDrawPath;
#else
    std::string command = "python3 " + pythonSavePath;
#endif
    system(command.c_str());
  }

  if (polygon0.is_clockwise_oriented())
  {
    polygon0.reverse_orientation();
  }
  if (polygon1.is_clockwise_oriented())
  {
    polygon1.reverse_orientation();
  }

  polygon0.reverse_orientation();
  Polygon_with_holes_2 sum = CGAL::minkowski_sum_by_reduced_convolution_2(polygon0, polygon1);
  std::cout << "Number of holes =" << sum.number_of_holes() << std::endl;
  Polygon_2 polygon2 = sum.outer_boundary();
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  Polygon<T> border(polygon0);
  pythonScript += border.draw("obs");
  Polygon_2::Vertex_circulator first, curr, prev;
  Polygon_2 cleanPolygon;
  Polygon_2 tmpPolygon = polygon2;
  while (true)
  {
    bool foundSame = false;
    first = curr = tmpPolygon.vertices_circulator();
    std::cout << "first=" << first->x() << " " << first->y() << std::endl;
    std::vector<Point_2> tmpCleanedPolygon;
    bool firstAdded = false;
    do
    {
      Polygon_2::Vertex_circulator nextTwo = curr + 2;
      Polygon_2::Vertex_circulator next = curr + 1;
      std::cout << "curr=" << curr->x() << " " << curr->y() << " next=" << next->x() << " " << next->y() << " nextTwo=" << nextTwo->x() << " " << nextTwo->y() << std::endl;
      if (*next == *curr)
      {
        curr++;
        foundSame = true;
        tmpCleanedPolygon.push_back(*curr);
        std::cout << "************Found Next Same************\n";
        if (curr == first)
          break;
        curr++;
      }
      else if (*nextTwo == *curr)
      {
        std::cout << "************Found Next Two Same************\n";
        foundSame = true;
        if (next == first)
        {
          std::cout << "************Next is First************\n";
          if (firstAdded)
          {
            tmpCleanedPolygon.erase(tmpCleanedPolygon.begin());
            firstAdded = false;
          }
          break;
        }
        if (nextTwo == first)
        {
          std::cout << "************Next Two is First************\n";
          break;
        }
        curr += 2;
        std::cout << "Added curr=" << curr->x() << " " << curr->y() << std::endl;
        tmpCleanedPolygon.push_back(*curr);
        curr++;
      }
      else
      {
        tmpCleanedPolygon.push_back(*curr);
        if (curr == first)
        {
          firstAdded = true;
          std::cout << "************First Added************\n";
        }
        curr++;
      }
    } while (curr != first);
    tmpPolygon = Polygon_2(tmpCleanedPolygon.begin(), tmpCleanedPolygon.end());
    std::cout << "foundSame=" << foundSame << std::endl;
    if (!foundSame)
    {
      cleanPolygon = tmpPolygon;
      break;
    }
  }
  // first = curr = cleanPolygon.vertices_circulator();
  Polygon<T> polygonSum(polygon2);
  pythonScript += polygonSum.draw("start");
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('Results/sum.png', dpi=500)\n";
  pythonScript += "plt.show()\n";
  std::string pythonSavePath = "build/draw.py";
  {
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