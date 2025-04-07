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
  int resolution = std::stoi(argv[2]);
  std::string plannerType = argv[3];
  std::string figPath;
  if (argc > 4) figPath = argv[4];
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
  bool considerSymmetry = get<bool>(*plannerSettings, "considerSymmetry", true);
  print("Setup: ",
        "Start",
        start,
        "Goal",
        goal,
        "Resolutions",
        resolution,
        "Draw Graph",
        figPath,
        "Consider Symmetry",
        considerSymmetry);
  T distsRVG, rvgBuildTime, rvgSearchTime;
  setPrecision<T>(3);
  bool hashWithTheta = true;
  bool incremental = false;
  bool simpilfiedGeometry = true;
  int numThreads = 16;
  VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, considerSymmetry, hashWithTheta, simpilfiedGeometry, numThreads, incremental);
//  visibilityGraph.plotLayers("");
  visibilityGraph.setWeight(1.0, 0.);
  std::string obsPath = figPath.substr(0, figPath.find_last_of('.')) + "_obs.txt";
  visibilityGraph.shortestPath(start, goal);
  visibilityGraph.draw(figPath);
  T pathLength = visibilityGraph.getPathLength();
  distsRVG = pathLength;
  rvgBuildTime = visibilityGraph.getBuildTime();
  rvgSearchTime = visibilityGraph.getSearchTime();
  print("RVG BuildTime(s): ", rvgBuildTime);
  print("RVG SearchTime(s):", rvgSearchTime);
  print("RVG Path Length:  ", distsRVG);
  Env<T>::SolverType solverType;
  if (plannerType == "RRTStar") solverType = Env<T>::SolverType::RRTStar;
  else if (plannerType == "AITStar") solverType = Env<T>::SolverType::AITStar;
  else if (plannerType == "BITStar") solverType = Env<T>::SolverType::BITStar;
  else if (plannerType == "FMT") solverType = Env<T>::SolverType::FMT;
  else throw std::runtime_error("Invalid planner type");
  std::vector<T> dists;
  T totalTime = rvgBuildTime + rvgSearchTime;
  for (int i = 0; i < 10; i++) {
    Env<T> omplEnv(obstacles, map, robot, solverType);
    omplEnv.setStartGoal(*start, *goal);
    T dist = 0;
    omplEnv.incrementalPlan(totalTime, dist);
    Utils::print("distance:", dist);
    dists.push_back(dist);
  }
  Utils::print("OMPL Path Length: ", dists);

  return 0;
}
