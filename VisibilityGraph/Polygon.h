#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_POLYGON_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_POLYGON_H_

#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Edge.h>
#include <CGAL/Simple_cartesian.h>
#include <vector>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_set_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>

namespace RotationalVisibilityGraph {
template<typename T>
class Polygon {
 public:
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  Polygon();
  Polygon(const std::vector<Vertex<T>> &vertices,  bool sortVertices, const Vertex<T> &center);
  Polygon(const std::vector<Vertex<T>> &vertices,  bool sortVertices);
  Polygon(const Polygon<T> &other);
  Polygon(const Polygon_2 &polygon);
  Polygon(const Polygon_2 &polygon, const Vertex<T> &center);
  [[nodiscard]] int size() const;
  const Vertex<T> &operator[](int index) const;
  Vertex<T> &operator[](int index);
  void translate(T x, T y);
  void rotate(T theta);
  void rotate(T theta, const Vertex<T> &center);
  Polygon<T> rotateCopy(T theta) const;
  Polygon<T> rotateCopy(T theta, const Vertex<T> &center) const;
  Polygon_2 rotateCGALCopy(T theta) const;
  void moveTo(T x, T y, T theta);
  Polygon<T> moveToCopy(T x, T y, T theta) const;
  [[nodiscard]] bool isConvex() const;
  void draw(Polygon_set_2 &S) const;
  void draw() const;
  void invert();
  [[nodiscard]] const Polygon_2 &getPolygon() const;
  const std::vector<Vertex<T>> &getVertices() const;
  Polygon<T> scale(T scale) const;
  [[nodiscard]] Polygon_2 getClockWise() const;
  [[nodiscard]] Polygon_2 getCounterClockWise() const;
  [[nodiscard]] Polygon_with_holes_2 getComplement() const;
  const Vertex<T> &getCentroid() const;
  std::vector<T> getX() const;
  std::vector<T> getY() const;
  std::vector<T> getTheta() const;
  bool intersects(const Polygon<T> &other) const;
  bool intersectsEdge(const Edge<T> &edge) const;
  bool contains(const Vertex<T> &v) const;
  bool operator==(const Polygon<T> &other) const;
  Polygon<T> bbox() const;
  Polygon<T> merge(const Polygon<T> &other) const;
  std::vector<Polygon<T>> convexDecomposition() const;
  void draw(const std::string &name, const std::string & title, bool show = false) const;
  [[nodiscard]] std::string draw(const std::string& type, bool dashed=false) const;
  [[nodiscard]] std::string draw3D(const std::string& type, bool dashed=false) const;
  Polygon<T> findMaximumInscribedCircle(int numVertices) const;

 private:
  Vertex<T> _calculateCentroid() const;
  void _sortVertices();
  T _centroidAngle(const Vertex<T> &v) const;
  void _updatePolygon();
  std::vector<Vertex<T>> _vertices;
  Vertex<T> _centroid;
  Polygon_2 _polygon;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_POLYGON_H_
