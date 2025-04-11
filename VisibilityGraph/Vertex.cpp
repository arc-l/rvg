#include "Vertex.h"
#include <Utils/Utils.h>

namespace RotationalVisibilityGraph {
template<typename T>
Vertex<T>::Vertex() : _x(0), _y(0) {
  _point = Point_2(0, 0);
  _hashWithTheta = false;
}

template<typename T>
Vertex<T>::Vertex(T x, T y, T theta_lb, T theta_ub, T theta)
    : _x(x), _y(y), _thetaLb(theta_lb), _thetaUb(theta_ub), _theta(theta), _hashWithTheta(true) {
  _point = Point_2(x, y);
}

template<typename T>
Vertex<T>::Vertex(T x, T y) : _x(x), _y(y) {
  _point = Point_2(x, y);
  _hashWithTheta = false;
}

template<typename T>
Vertex<T>::Vertex(const Vertex<T> &vertex) {
  _x = vertex._x;
  _y = vertex._y;
  _thetaLb = vertex._thetaLb;
  _thetaUb = vertex._thetaUb;
  _theta = vertex._theta;
  _point = Point_2(_x, _y);
  _hashWithTheta = vertex._hashWithTheta;
}

template<typename T>
size_t Vertex<T>::Hash::operator()(const Vertex<T> &vertex) const {
  return vertex.hash();
}

template<typename T>
bool Vertex<T>::Hash::operator()(const Vertex<T> &lhs, const Vertex<T> &rhs) const {
  return lhs == rhs;
}

template<typename T>
size_t Vertex<T>::SharedPtrVertexHash::operator()(const std::shared_ptr<Vertex<T>> &vertex) const {
  return vertex->hash();
}

template<typename T>
bool Vertex<T>::SharedPtrVertexEqual::operator()(const std::shared_ptr<Vertex<T>> &lhs, const std::shared_ptr<Vertex<T>> &rhs) const {
  return *lhs == *rhs;
}

template<typename T>
size_t Vertex<T>::hash() const {
  size_t seed = 0;
  int x = static_cast<int>(_x * 100000);
  int y = static_cast<int>(_y * 100000);
  Utils::hash_combine(seed, std::hash<T>{}(x));
  Utils::hash_combine(seed, std::hash<T>{}(y));
  if (_hashWithTheta) {
    int theta = static_cast<int>(_theta.value() * 100000);
    Utils::hash_combine(seed, std::hash<T>{}(theta));
  }
  return seed;
}

template<typename T>
bool Vertex<T>::operator==(const Vertex<T> &rhs) const {
  // tolerance for floating point comparison
  if (_hashWithTheta) {
    return std::abs(_x - rhs._x) < 1e-5 && std::abs(_y - rhs._y) < 1e-5 && std::abs(_theta.value() - rhs._theta.value()) < 1e-5;
  }
  return std::abs(_x - rhs._x) < 1e-5 && std::abs(_y - rhs._y) < 1e-5;
}

template<typename T>
bool Vertex<T>::operator!=(const Vertex<T> &rhs) const {
  return !(rhs == *this);
}

template<typename T>
bool Vertex<T>::operator<(const Vertex<T> &rhs) const {
  return this->hash() < rhs.hash();
}

template<typename T>
Vertex<T> Vertex<T>::operator+(const Vertex<T> &rhs) const {
  return Vertex<T>(_x + rhs._x, _y + rhs._y);
}

template<typename T>
void Vertex<T>::setTheta(T theta) {
  _theta = theta;
  _hashWithTheta = true;
}

template<typename T>
void Vertex<T>::setBounds(T theta_lb, T theta_ub) {
  _thetaLb = theta_lb;
  _thetaUb = theta_ub;
  _hashWithTheta = true;
}

template<typename T>
std::tuple<T, T> Vertex<T>::getCoord() const {
  return std::make_tuple(_x, _y);
}

template<typename T>
const typename Vertex<T>::Point_2 &Vertex<T>::getPoint() const {
  return _point;
}

template<typename T>
void Vertex<T>::setPos(T x, T y) {
  _x = x;
  _y = y;
  _point = Point_2(x, y);
}

template<typename T>
const T &Vertex<T>::getX() const {
  return _x;
}

template<typename T>
const T &Vertex<T>::getY() const {
  return _y;
}

template<typename T>
const T &Vertex<T>::getThetaLb() const {
  return _thetaLb.value();
}

template<typename T>
const T &Vertex<T>::getThetaUb() const {
  return _thetaUb.value();
}

template<typename T>
const T &Vertex<T>::getTheta() const {
  return _theta.value();
}

template<typename T>
T Vertex<T>::dist(const Vertex<T> &other) const {
  return std::sqrt(std::pow(_x - other._x, 2) + std::pow(_y - other._y, 2));
}

template<typename T>
bool Vertex<T>::hasTheta() const {
  return _theta.has_value();
}

template<typename T>
bool Vertex<T>::_overlapInterval(T lb1, T ub1, T lb2, T ub2, T cycle, T &newLb, T &newUb) const {
  if (fabs(ub1 - cycle) < 1e-5 || fabs(ub2 - cycle) < 1e-5) {
    //deal with full span or right end included
    bool is_full_span1 = fabs(fabs(ub1 - lb1) - cycle) < 1e-5;
    bool is_full_span2 = fabs(fabs(ub2 - lb2) - cycle) < 1e-5;
    if (is_full_span1 || is_full_span2) {
      // Full coverage of the circle
      newLb = 0;
      newUb = cycle;
      return true; // Full coverage of the circle
    }

    if (fabs(ub1 - cycle) < 1e-5) {
      if (lb2 < ub2) {
        if (lb1 <= ub2) {
          newLb = std::min(lb1, lb2);
          newUb = ub1;
          return true;
        } else {
          if (fabs(lb2) < 1e-5) {
            newLb = lb1;
            newUb = ub2;
            return true;
          }
          return false;
        }
      } else {
        newLb = std::min(lb2, lb1);
        newUb = ub2;
        if (newLb <= newUb) {
          newLb = 0;
          newUb = cycle;
        }
        return true;
      }
    }
    if (fabs(ub2 - cycle) < 1e-5) {
      if (lb1 < ub1) {
        if (lb2 <= ub1) {
          newLb = std::min(lb1, lb2);
          newUb = ub2;
          return true;
        } else {
          if (fabs(lb1) < 1e-5) {
            newLb = lb2;
            newUb = ub1;
            return true;
          }
          return false;
        }
      } else {
        newLb = std::min(lb1, lb2);
        newUb = ub1;
        if (newLb <= newUb) {
          newLb = 0;
          newUb = cycle;
        }
        return true;
      }
    }
    return false;
  } else {
    // deal with normal case
    if (lb1 <= ub1 && lb2 <= ub2) {
      if (lb1 <= ub2 && lb2 <= ub1) {
        newLb = std::min(lb1, lb2);
        newUb = std::max(ub1, ub2);
        return true;
      }
      return false;
    } else if (lb1 < ub1 && lb2 > ub2) {
      // second interval wraps around
      T tmpNewLb1, tmpNewUb1, tmpNewLb2, tmpNewUb2;
      bool overlap1 = _overlapInterval(lb1, ub1, 0, ub2, cycle, tmpNewLb1, tmpNewUb1);
      bool overlap2 = _overlapInterval(lb1, ub1, lb2, cycle, cycle, tmpNewLb2, tmpNewUb2);
      if (overlap1 && !overlap2) {
        newLb = lb2;
        newUb = tmpNewUb1;
        return true;
      }
      if (overlap2 && !overlap1) {
        newLb = tmpNewLb2;
        newUb = ub2;
        return true;
      }
      if (overlap1 && overlap2) {
        newLb = 0;
        newUb = cycle;
        return true;
      }
      return false;
    } else if (lb2 < ub2 && lb1 > ub1) {
      // first interval wraps around
      T tmpNewLb1, tmpNewUb1, tmpNewLb2, tmpNewUb2;
      bool overlap1 = _overlapInterval(0, ub1, lb2, ub2, cycle, tmpNewLb1, tmpNewUb1);
      bool overlap2 = _overlapInterval(lb1, cycle, lb2, ub2, cycle, tmpNewLb2, tmpNewUb2);
      if (overlap1 && !overlap2) {
        newLb = lb1;
        newUb = tmpNewUb1;
        return true;
      }
      if (overlap2 && !overlap1) {
        newLb = tmpNewLb2;
        newUb = ub1;
        return true;
      }
      if (overlap1 && overlap2) {
        newLb = 0;
        newUb = cycle;
        return true;
      }
      return false;
    } else if (lb1 > ub1 && lb2 > ub2) {
      // both intervals wraps around
      newLb = std::min(lb1, lb2);
      newUb = std::max(ub1, ub2);
      if (newLb <= newUb) {
        newLb = 0;
        newUb = cycle;
      }
      return true;
    }

    return false;
  }
}

template<typename T>
T Vertex<T>::rotationalDist(const Vertex<T> &other) const {
  T theta1 = _theta.value();
  T theta2 = other._theta.value();
  T diff = std::abs(theta1 - theta2);
  return diff;
}

template <typename T>
T Vertex<T>::distanceToEdge(const Vertex<T>& a, const Vertex<T>& b) const {
    // Calculate the perpendicular distance from point `this` to the line segment `ab`
    auto [px, py] = this->getCoord();
    auto [ax, ay] = a.getCoord();
    auto [bx, by] = b.getCoord();

    T dx = bx - ax;
    T dy = by - ay;
    T lengthSq = dx * dx + dy * dy;
    T t = ((px - ax) * dx + (py - ay) * dy) / lengthSq;
    t = std::max(T(0), std::min(T(1), t));
    T projX = ax + t * dx;
    T projY = ay + t * dy;
    return std::sqrt((px - projX) * (px - projX) + (py - projY) * (py - projY));
}

template
class Vertex<float>;
template
class Vertex<double>;
}
