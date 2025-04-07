#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEX_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEX_H_

#include <optional>
#include <tuple>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <Utils/Pragma.h>

namespace RotationalVisibilityGraph {
template<typename T>
class Vertex {
 public:
  struct Hash {
    size_t operator()(const Vertex<T> &vertex) const;
    bool operator()(const Vertex<T> &lhs, const Vertex<T> &rhs) const;
  };
  struct SharedPtrVertexHash{
    size_t operator()(const std::shared_ptr<Vertex<T>> &vertex) const;
  };
  struct SharedPtrVertexEqual{
    bool operator()(const std::shared_ptr<Vertex<T>> &lhs, const std::shared_ptr<Vertex<T>> &rhs) const;
  };

  DECL_CGAL_CARTESIAN_TYPES_T
  Vertex(T x, T y, T theta_lb, T theta_ub, T theta, T roundUpTheta, bool hasWithTheta);
  Vertex(T x, T y);
  Vertex();
  Vertex(const Vertex<T> &vertex);
  [[nodiscard("Check the return value to ensure the hash is used.")]] size_t hash() const;
  bool operator==(const Vertex<T> &rhs) const;
  bool operator!=(const Vertex<T> &rhs) const;
  bool operator<(const Vertex<T> &rhs) const;
  Vertex operator+(const Vertex<T> &rhs) const;
  friend std::ostream &operator<<(std::ostream &os, const Vertex<T> &vertex) {
    os << "Vertex(" << "x:" << vertex._x << " y:" << vertex._y;
    if (vertex._theta.has_value()) {
      os << " theta:" << vertex._theta.value();
    }
    if (vertex._thetaLb.has_value()) {
      os << " theta_lb:" << vertex._thetaLb.value();
    }
    if (vertex._thetaUb.has_value()) {
      os << " theta_ub:" << vertex._thetaUb.value();
    }
    if(vertex._roundUpTheta.has_value()) {
      os << " roundUpTheta:" << vertex._roundUpTheta.value();
    }
    os << ")";
    return os;
  }
  void setTheta(T theta);
  void setBounds(T theta_lb, T theta_ub);
  void setPos(T x, T y);
  const T &getX() const;
  const T &getY() const;
  const T &getTheta() const;
  const T &getThetaLb() const;
  const T &getThetaUb() const;
  Vertex<T> mergeThetaRange(const Vertex<T>& other) const;
  std::tuple<T, T> getCoord() const;
  [[nodiscard]] const Point_2 &getPoint() const;
  T dist(const Vertex<T> &other) const;
  T rotationalDist(const Vertex<T> &other) const;
  T distanceToEdge(const Vertex<T>& a, const Vertex<T>& b) const;
  [[nodiscard]] bool hasTheta() const;
  [[nodiscard]] bool hashWithTheta() const;

 private:
  T _x;
  T _y;
  Point_2 _point;
  std::optional<T> _thetaLb;
  std::optional<T> _thetaUb;
  std::optional<T> _theta;
  std::optional<T> _roundUpTheta;
  bool _hashWithTheta;
  bool _overlapInterval(T lb1, T ub1, T lb2, T ub2, T cycle, T &newLb, T& newUb) const;

};
}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEX_H_
