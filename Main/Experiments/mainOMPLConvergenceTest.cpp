#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/VisibilityGraph.h>
#include <VisibilityGraph/Utils.h>
#include <OMPL/CollisionDetection.h>
#include <OMPL/OMPLEnv.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T


int main(int argc, char *argv[]) {
  if (argc < 3) throw std::runtime_error("No input file provided");
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  std::string plannerType = argv[2];
  T pathLength = std::stod(argv[3]);
  T time = 10;
  Polygon<T> robot = getRobot<T>(pt);
  std::vector<Polygon<T>> obstacles = getObstacles<T>(pt);
  int mapSize = get<int>(*pt.RootElement()->FirstChildElement("environment"), "mapSize", 100);
  Polygon<T> map = getMap<T>(mapSize);
  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  auto start = *getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  auto goal = *getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  if (start.getTheta() > PI) {
    start.setTheta(start.getTheta() - 2 * PI);
  }
  if (goal.getTheta() > PI) {
    goal.setTheta(goal.getTheta() - 2 * PI);
  }
  std::vector<T> tolerance = {2, 1.5, 1.3, 1.2, 1.1, 1.05, 1.02, 1.01};
  setPrecision<T>(3);
  Env<T>::SolverType solverType;
  if (plannerType == "RRTStar") solverType = Env<T>::SolverType::RRTStar;
  else if (plannerType == "AITStar") solverType = Env<T>::SolverType::AITStar;
  else if (plannerType == "BITStar") solverType = Env<T>::SolverType::BITStar;
  else if (plannerType == "FMT") solverType = Env<T>::SolverType::FMT;
  else throw std::runtime_error("Invalid planner type");
  Env<T> omplEnv(obstacles, map, robot, solverType);
  T dist = std::numeric_limits<T>::max();
  T timeout = 6000;
  omplEnv.setStartGoal(start, goal);
  for (const auto &tol : tolerance) {
    T omplTotalTime = 0.;
    omplEnv.reset();
    while (dist > pathLength * tol && omplTotalTime < timeout) {
      omplEnv.incrementalPlan(time, dist);
      omplTotalTime += time;
    }
    if (omplTotalTime > timeout)
      Utils::print("Tol=", tol, "Timeout", "distance =", dist);
    else
      Utils::print("Tol=", tol, "time=", omplTotalTime, "distance =", dist);
  }
  return 0;
}

