#include "Edge.h"

namespace RotationalVisibilityGraph {
template<typename T>
Edge<T>::Edge(const Vertex<T> &v1, const Vertex<T> &v2) : _v1(v1), _v2(v2) {
  _segment = Segment_2(v1.getPoint(), v2.getPoint());
}

template<typename T>
Edge<T>::Edge(const Edge<T> &other) : _v1(other._v1), _v2(other._v2), _segment(other._segment) {}

template<typename T>
size_t Edge<T>::Hash::operator()(const Edge<T> &edge) const {
  return edge.hash();
}

template<typename T>
bool Edge<T>::Hash::operator()(const Vertex<T> &lhs, const Vertex<T> &rhs) const {
  return lhs == rhs;
}

template<typename T>
const Vertex<T> &Edge<T>::v1() const {
  return _v1;
}

template<typename T>
const Vertex<T> &Edge<T>::v2() const {
  return _v2;
}

template<typename T>
bool Edge<T>::operator==(const Edge<T> &other) const {
  return (_v1 == other._v1 && _v2 == other._v2) || (_v1 == other._v2 && _v2 == other._v1);
}

template<typename T>
bool Edge<T>::operator!=(const Edge<T> &other) const {
  return !(*this == other);
}

template<typename T>
bool Edge<T>::hasVertex(const Vertex<T> &v) const {
  return v == _v1 || v == _v2;
}

template<typename T>
const Vertex<T> &Edge<T>::getAdjacent(const Vertex<T> &v) const {
  if (v == _v1)
    return _v2;
  if (v == _v2)
    return _v1;
  throw std::invalid_argument("Vertex not in edge.");
}

template<typename T>
size_t Edge<T>::hash() const {
  return _v1.hash() ^ _v2.hash();
}

template<typename T>
T Edge<T>::length() const {
  return _v1.dist(_v2);
}

template<typename T>
bool Edge<T>::intersects(const Edge<T> &other) const {
  return CGAL::do_intersect(_segment, other._segment);
}

template<typename T>
Vertex<T> Edge<T>::intersectionPoint(const Edge<T> &other) const {
  ASSERT_MSG(intersects(other), "Edges do not intersect.")
  auto intersection = CGAL::intersection(_segment, other._segment);
  if (const Point_2 *p = boost::get<Point_2>(&*intersection))
    return Vertex<T>(CGAL::to_double(p->x()), CGAL::to_double(p->y()));
  else if (const Segment_2 *s = boost::get<Segment_2>(&*intersection))
    return Vertex<T>(CGAL::to_double(s->source().x()), CGAL::to_double(s->source().y()));
}

template<typename T>
const typename Edge<T>::Segment_2 &Edge<T>::getSegment() const {
  return _segment;
}

template<typename T>
std::vector<T> Edge<T>::getX() const {
  return {_v1.getX(), _v2.getX()};
}

template<typename T>
std::vector<T> Edge<T>::getY() const {
  return {_v1.getY(), _v2.getY()};
}

template<typename T>
std::vector<T> Edge<T>::getTheta() const {
  return {_v1.getTheta(), _v2.getTheta()};
}


template<typename T>
std::string Edge<T>::draw(const std::string & settings) const {
  std::string pythonScript = "edge = plt.plot([" + std::to_string(_v1.getX()) + ", " + std::to_string(_v2.getX()) + "], ["
      + std::to_string(_v1.getY()) + ", " + std::to_string(_v2.getY()) + "], " + settings + ")\n";
  return pythonScript;
}

template
class Edge<float>;
template
class Edge<double>;

}

