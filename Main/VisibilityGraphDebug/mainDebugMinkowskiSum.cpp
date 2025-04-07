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

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type> Convex_hull_traits_2;
using namespace RotationalVisibilityGraph;
using T = double;

int main() {
  std::vector<Point_2> verticesUpper = {
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
  std::vector<Point_2> verticesLower = {
      Point_2(-20, -2),
      Point_2(-15, -2),
      Point_2(-10, 3),
      Point_2(-5, 3),
      Point_2(0, -2),
      Point_2(5, -2),
      Point_2(10, -7),
      Point_2(15, -7),
      Point_2(20, -2),
      Point_2(20, -40),
      Point_2(-20, -40)
  };
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(-3, 0),
      Vertex<T>(-2, 0),
      Vertex<T>(-2, 2),
      Vertex<T>(-3, 2)
  };
  Polygon<T> robot(vertices, false);
  robot.moveTo(0, 0, 0);
  Polygon<T> robot1 = robot.rotateCopy(-5 * PI / 18);
  vertices = robot1.getVertices();
  Polygon<T> robot2 = robot.rotateCopy(-4 * PI / 18);
  vertices.insert(vertices.end(), robot2.getVertices().begin(), robot2.getVertices().end());
  Polygon<T> sumRobot(vertices, true);

  std::vector<Point_2> vertices1;
  for (const auto &v : sumRobot.getVertices()) {
    vertices1.emplace_back(Point_2(v.getX(), v.getY()));
  }

  Polygon_2 polygonUpper(verticesUpper.begin(), verticesUpper.end());
  Polygon_2 polygonLower(verticesLower.begin(), verticesLower.end());
  Polygon_2 polygon1(vertices1.begin(), vertices1.end());
  if (polygonUpper.is_clockwise_oriented()) {
    polygonUpper.reverse_orientation();
  }
  if (polygonLower.is_clockwise_oriented()) {
    polygonLower.reverse_orientation();
  }
  if (polygon1.is_clockwise_oriented()) {
    polygon1.reverse_orientation();
  }

  CGAL::draw(polygon1);

//  polygonUpper.reverse_orientation();
  Polygon_with_holes_2 sum1 = CGAL::minkowski_sum_by_reduced_convolution_2(polygonUpper, polygon1);
  Polygon_with_holes_2 sum2 = CGAL::minkowski_sum_by_reduced_convolution_2(polygonLower, polygon1);
  Polygon_2 sum1Polygon = sum1.outer_boundary();
  Polygon_2 sum2Polygon = sum2.outer_boundary();
  // join the two polygons
  Polygon_with_holes_2 res;
  CGAL::join(sum1Polygon, sum2Polygon, res);
  CGAL::draw(res);
  std::string pythonScript = "import numpy as np\n";
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "\n";
  pythonScript += "fig, ax = plt.subplots()\n";
  pythonScript += "ax.set_facecolor('gainsboro')\n";
  pythonScript += "border = np.array([\n";
  for (const auto &v : verticesUpper) {
    pythonScript += "\t[" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "],\n";
  }
  pythonScript += "\t[" + std::to_string(CGAL::to_double(verticesUpper[0].x())) + ", " + std::to_string(CGAL::to_double(verticesUpper[0].y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-g')\n";
  pythonScript += "border = np.array([\n";
  for (const auto &v : verticesLower) {
    pythonScript += "\t[" + std::to_string(CGAL::to_double(v.x())) + ", " + std::to_string(CGAL::to_double(v.y())) + "],\n";
  }
  pythonScript += "\t[" + std::to_string(CGAL::to_double(verticesLower[0].x())) + ", " + std::to_string(CGAL::to_double(verticesLower[0].y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-g')\n";
  Polygon_2::Vertex_circulator first, curr, prev;
  Polygon_2 cleanPolygon;
  Polygon_2 tmpPolygon = sum1Polygon;
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
  pythonScript += "sum1 = np.array([\n";
  do {
    std::cout << "x=" << curr->x() << " y=" << curr->y() << std::endl;
    pythonScript += "\t[" + std::to_string(CGAL::to_double(curr->x())) + ", " + std::to_string(CGAL::to_double(curr->y())) + "],\n";
    curr++;
  } while (curr != first);
  pythonScript += "\t[" + std::to_string(CGAL::to_double(first->x())) + ", " + std::to_string(CGAL::to_double(first->y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(sum1[:, 0], sum1[:, 1], '-o', color='deeppink', markersize=0.5, linewidth=1)\n";
  first = curr = sum2Polygon.vertices_circulator();
  pythonScript += "sum1 = np.array([\n";
  do {
    std::cout << "x=" << curr->x() << " y=" << curr->y() << std::endl;
    pythonScript += "\t[" + std::to_string(CGAL::to_double(curr->x())) + ", " + std::to_string(CGAL::to_double(curr->y())) + "],\n";
    curr++;
  } while (curr != first);
  pythonScript += "\t[" + std::to_string(CGAL::to_double(first->x())) + ", " + std::to_string(CGAL::to_double(first->y())) + "],\n";
  pythonScript += "])\n";
  pythonScript += "ax.plot(sum1[:, 0], sum1[:, 1], '-o', color='tab:blue', markersize=0.5, linewidth=1)\n";
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