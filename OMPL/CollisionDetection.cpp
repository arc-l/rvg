#include "CollisionDetection.h"

namespace RotationalVisibilityGraph {

template<typename T>
ValidityChecker<T>::ValidityChecker(const std::vector<Polygon<T>> &obstacles,
                                    Polygon<T> &robot,
                                    const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si), _obstacles(obstacles), _robot(robot) {};

template<typename T>
bool ValidityChecker<T>::isValid(const ob::State *state) const {
  const auto *state2D = state->as<ob::SE2StateSpace::StateType>();
  T x = state2D->getX();
  T y = state2D->getY();
  T theta = state2D->getYaw();
  Polygon<T> robot = _robot.moveToCopy(x, y, theta);
  for (const auto &obstacle : _obstacles) {
    if (robot.intersects(obstacle)) {
      return false;
    }
  }
  return true;
}

//template<typename T>
//double ValidityChecker<T>::clearance(const ob::State *state) const {
//  // We know we're working with a RealVectorStateSpace in this
//  // example, so we downcast state into the specific type.
//  const auto *state2D = state->as<ob::SE2StateSpace::StateType>();
//
//  T x = state2D->getX();
//  T y = state2D->getY();
//  T theta = state2D->getYaw();
//  _robot.moveTo(x, y, theta);
//  for (const auto &obstacle : _obstacles) {
//    if (_robot.intersects(obstacle)) {
//      return 0.0;
//    }
//  }
////  return std::numeric_limits<double>::infinity();
//  return 0.1;
//}

template
class ValidityChecker<double>;
template
class ValidityChecker<float>;
}
