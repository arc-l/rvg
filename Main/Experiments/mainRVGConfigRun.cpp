#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/VisibilityGraph.h>
#include <VisibilityGraph/Utils.h>

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
  int resolution = std::stoi(argv[2]);
  int numThreads = std::stoi(argv[3]);
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
  std::shared_ptr<Vertex<T>> start = getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  std::shared_ptr<Vertex<T>> goal = getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  print("Setup: ",
        "Start",
        *start,
        "Goal",
        *goal,
        "Resolutions",
        resolution,
        "Draw Graph",
        figPath
        );
  std::vector<T> distsRVG, rvgBuildTime, rvgSearchTime;
  setPrecision<T>(3);
  bool fineApprox = false;
  bool optimal = true;
  bool verbose = true;
  VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, fineApprox, numThreads, optimal, verbose);
  const auto &layers = visibilityGraph.getLayers();
  visibilityGraph.setWeight(1.0, 0.0);
  visibilityGraph.shortestPath(start, goal, 0);
  T pathLength = visibilityGraph.getPathLength();
  // T totalTime = visibilityGraph.getTotalTime();
  distsRVG.push_back(pathLength);
  rvgBuildTime.push_back(visibilityGraph.getBuildTime());
  rvgSearchTime.push_back(visibilityGraph.getSearchTime());
  print("RVG BuildTime(s): ", rvgBuildTime);
  print("RVG SearchTime(s):", rvgSearchTime);
  print("RVG Path Length:  ", distsRVG);
  // visibilityGraph.drawVisibleAreas();
//  visibilityGraph.debugMaxPropagation();
  if(figPath.empty()) return 0;
  // visibilityGraph.animation(figPath, false);
  // change the figPath's extension to .png
  figPath = figPath.substr(0, figPath.find_last_of('.')) + ".png";
  visibilityGraph.draw(figPath, false, true, true, false);
  // visibilityGraph.draw3D(false);
  return 0;

}