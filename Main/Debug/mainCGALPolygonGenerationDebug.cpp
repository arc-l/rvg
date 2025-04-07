#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<K> Triangulation;
typedef Triangulation::Finite_faces_iterator FaceIterator;

// Generate random points within the boundary
std::vector<Point_2> generateRandomPoints(int numPoints, double minX, double maxX, double minY, double maxY) {
  std::vector<Point_2> points;
  for (int i = 0; i < numPoints; ++i) {
    double x = minX + static_cast<double>(rand()) / RAND_MAX * (maxX - minX);
    double y = minY + static_cast<double>(rand()) / RAND_MAX * (maxY - minY);
    points.push_back(Point_2(x, y));
  }
  return points;
}

// Generate 2D polygonal map using Delaunay triangulation
void generatePolygonalMap(int numPoints, double minX, double maxX, double minY, double maxY) {
  std::vector<Point_2> points = generateRandomPoints(numPoints, minX, maxX, minY, maxY);

  // Ensure all points lie within the boundary
  for (auto& point : points) {
    point = Point_2(std::max(minX, std::min(maxX, point.x())), std::max(minY, std::min(maxY, point.y())));
  }

  Triangulation triangulation;
  triangulation.insert(points.begin(), points.end());

  // Output the triangles
  for (FaceIterator fit = triangulation.finite_faces_begin(); fit != triangulation.finite_faces_end(); ++fit) {
    Triangulation::Face_handle face = fit;
    std::cout << "Triangle vertices: " << std::endl;
    for (int i = 0; i < 3; ++i) {
      Point_2 vertex = face->vertex(i)->point();
      std::cout << "(" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
    }
    std::cout << std::endl;
  }
}

int main() {
  // Define the boundary of the map
  double minX = 10.0, maxX = 90.0, minY = 10.0, maxY = 90.0; // Example boundary

  // Generate the polygonal map
  generatePolygonalMap(20, minX, maxX, minY, maxY); // Adjust the number of points as needed

  return 0;
}
