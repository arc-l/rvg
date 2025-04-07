#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/SamplingBasedRVG.h>
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
  bool considerSymmetry = get<bool>(*plannerSettings, "considerSymmetry", true);
  print("Setup: ",
        "Start",
        *start,
        "Goal",
        *goal,
        "Resolutions",
        resolution,
        "Draw Graph",
        figPath,
        "Consider Symmetry",
        considerSymmetry);
  std::vector<T> distsRVG, rvgBuildTime, rvgSearchTime;
  setPrecision<T>(3);
  bool hashWithTheta = true;
  bool fineApprox = false;
  bool verbose = true;
  SamplingBasedRVG<T> sbrvg = SamplingBasedRVG<T>(robot, map, obstacles, considerSymmetry, hashWithTheta, fineApprox, numThreads, verbose);
//   sbrvg.setWeight(1.0, 0.0);
//   sbrvg.shortestPath(start, goal, 0);
//   T pathLength = sbrvg.getPathLength();
//   // T totalTime = sbrvg.getTotalTime();
//   distsRVG.push_back(pathLength);
//   rvgBuildTime.push_back(sbrvg.getBuildTime());
//   rvgSearchTime.push_back(sbrvg.getSearchTime());
//   print("RVG BuildTime(s): ", rvgBuildTime);
//   print("RVG SearchTime(s):", rvgSearchTime);
//   print("RVG Path Length:  ", distsRVG);
//   // sbrvg.drawVisibleAreas();
// //  sbrvg.debugMaxPropagation();
//   if(figPath.empty()) return 0;
//   // sbrvg.animation(figPath, false);
//   // change the figPath's extension to .png
//   figPath = figPath.substr(0, figPath.find_last_of('.')) + ".png";
//   sbrvg.draw(figPath, false, true, false);
  // sbrvg.draw3D(false);
  return 0;

}