#ifndef ROTATIONALVISIBILITYGRAPH_OMPL_COLLISIONDETECTION_H_
#define ROTATIONALVISIBILITYGRAPH_OMPL_COLLISIONDETECTION_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>

namespace RotationalVisibilityGraph{
namespace ob = ompl::base;
namespace og = ompl::geometric;

template<typename T>
class ValidityChecker : public ob::StateValidityChecker
{
 public:
  ValidityChecker(const std::vector<Polygon<T>> &obstacles, Polygon<T> &robot, const ob::SpaceInformationPtr& si);
  bool isValid(const ob::State* state) const;
//  double clearance(const ob::State* state) const;
 private:
    const std::vector<Polygon<T>> &_obstacles;
    Polygon<T> &_robot;
};
}

#endif //ROTATIONALVISIBILITYGRAPH_OMPL_COLLISIONDETECTION_H_
