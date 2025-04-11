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
  if (argc < 4) throw std::runtime_error("No input file provided");
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  std::string plannerType = argv[2];
  T time = std::stod(argv[3]);
  int iters = 1;
  if (argc > 4) iters = std::stoi(argv[4]);
  std::string figPath;
  if (argc > 5) figPath = argv[5];
  Polygon<T> robot = getRobot<T>(pt);
  std::vector<Polygon<T>> obstacles = getObstacles<T>(pt);
  Polygon<T> map;
  bool useBoundary = get<bool>(*pt.RootElement()->FirstChildElement("environment"), "useBoundary", false);
  if (useBoundary) {
    map = getBoundary<T>(pt);
  } else {
    int mapSize = get<int>(*pt.RootElement()->FirstChildElement("environment"), "mapSize", 100);
    map = getMap<T>(mapSize);
  }
  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  auto start = getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  auto goal = getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  if (start->getTheta() > PI) {
    start->setTheta(start->getTheta() - 2 * PI);
  }
  if (goal->getTheta() > PI) {
    goal->setTheta(goal->getTheta() - 2 * PI);
  }
  print("Setup: ",
        "Start",
        start,
        "Goal",
        goal,
        "Draw Graph",
        figPath
        );
  setPrecision<T>(3);
  Env<T>::SolverType solverType;
  if (plannerType == "RRTStar") solverType = Env<T>::SolverType::RRTStar;
  else if (plannerType == "AITStar") solverType = Env<T>::SolverType::AITStar;
  else if (plannerType == "BITStar") solverType = Env<T>::SolverType::BITStar;
  else if (plannerType == "EITStar") solverType = Env<T>::SolverType::EITStar;
  else if (plannerType == "FMT") solverType = Env<T>::SolverType::FMT;
  else if (plannerType == "PRMStar") solverType = Env<T>::SolverType::PRMStar;
  else if (plannerType == "LazyPRMStar") solverType = Env<T>::SolverType::LazyPRMStar;
  else if (plannerType == "LBTRRT") solverType = Env<T>::SolverType::LBTRRT;
  else throw std::runtime_error("Invalid planner type");
  std::vector<T> dists;
  for (int i = 0; i < iters; i++) {
    Env<T> omplEnv(obstacles, map, robot, solverType);
    T dist = 0;
    omplEnv.setStartGoal(*start, *goal);
    omplEnv.reset();
    omplEnv.incrementalPlan(time, dist);
    Utils::print("distance:", dist);
    dists.push_back(dist);
    omplEnv.animation(figPath);
    figPath = figPath.substr(0, figPath.find_last_of('.')) + ".png";
    omplEnv.draw(figPath, false);
  }
  Utils::print("distances = ", dists);
  return 0;
}
