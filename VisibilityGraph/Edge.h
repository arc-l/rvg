#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_EDGE_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_EDGE_H_
#include "Vertex.h"

namespace RotationalVisibilityGraph {
template<typename T>
class Edge {
 public:
  struct Hash {
    std::size_t operator()(const Edge<T> &edge) const;
    bool operator()(const Vertex<T> &lhs, const Vertex<T> &rhs) const;
  };
  DECL_CGAL_CARTESIAN_TYPES_T
  Edge(const Vertex<T> &v1, const Vertex<T> &v2);
  Edge(const Edge<T> &other);
  const Vertex<T> &v1() const;
  const Vertex<T> &v2() const;
  bool operator==(const Edge<T> &other) const;
  bool operator!=(const Edge<T> &other) const;
  const Vertex<T> &getAdjacent(const Vertex<T> &v) const;
  const Segment_2 &getSegment() const;
  bool hasVertex(const Vertex<T> &v) const;
  [[nodiscard("Check the return value to ensure the hash is used.")]] size_t hash() const;
  T length() const;
  bool intersects(const Edge<T> &other) const;
  Vertex<T> intersectionPoint(const Edge<T> &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Edge<T> &edge) {
    os << edge._v1 << " -> " << edge._v2;
    return os;
  }
  std::vector<T> getX() const;
  std::vector<T> getY() const;
  std::vector<T> getTheta() const;
  std::string draw(const std::string &settings) const;

 private:
  Vertex<T> _v1;
  Vertex<T> _v2;
  Segment_2 _segment;
};
}
#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_EDGE_H_
