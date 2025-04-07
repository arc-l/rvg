#ifndef ROTATIONALVISIBILITYGRAPH_OMPL_OMPLENV_H_
#define ROTATIONALVISIBILITYGRAPH_OMPL_OMPLENV_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>
#include <iostream>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <OMPL/CollisionDetection.h>

namespace RotationalVisibilityGraph {
namespace ob = ompl::base;
namespace og = ompl::geometric;

template<typename T>
class Env {
 public:
  enum SolverType {
    RRTStar,
    AITStar,
    BITStar,
    EITStar,
    FMT,
    PRMStar,
    LazyPRMStar,
    LBTRRT
  };
  Env(const std::vector<Polygon<T>> &obstacles, const Polygon<T> &border, Polygon<T> &robot, SolverType solverType);
  void incrementalPlan(T time, T& dist);
  void setStartGoal(const Vertex<T> &start, const Vertex<T> &goal);
  void growRoadMap(T time);
  void clearQuery();
  void reset();
  void draw(const std::string &figPath, bool show=false);
  void animation(const std::string &figPath);
 private:
  const std::vector<Polygon<T>> &_obstacles;
  const Polygon<T> &_border;
  Polygon<T> &_robot;
  Vertex<T> _start, _goal;
  SolverType _solverType;
  std::shared_ptr<ob::SpaceInformation> _si;
  std::shared_ptr<ValidityChecker<T>> _validityChecker;
  std::shared_ptr<ob::Planner> _planner;
  std::shared_ptr<ob::ProblemDefinition> _pdef;
  std::shared_ptr<ob::SE2StateSpace> _space;
  std::shared_ptr<ob::PathLengthOptimizationObjective> _opt;
  std::vector<Vertex<T>> _sol;
  std::string _drawSetup() const;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_OMPL_OMPLENV_H_
