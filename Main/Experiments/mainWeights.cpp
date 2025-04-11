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
  tinyxml2::XMLDocument pt;
  pt.LoadFile("../Configs/env3.xml");
  Polygon<T> robot = getRobot<T>(pt);
  int resolution = 36;
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
  const auto start = getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  const auto goal = getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  bool considerSymmetry = get<bool>(*plannerSettings, "considerSymmetry", true);
  print("Setup: ",
        "Start",
        start,
        "Goal",
        goal,
        "Resolutions",
        resolution,
        "Consider Symmetry",
        considerSymmetry);
  std::vector<T> distsRVG, rvgBuildTime, rvgSearchTime;
  setPrecision<T>(3);
  bool fineApprox = false;
  bool verbose = true;
  int numThreads = 1;
  VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, considerSymmetry, fineApprox, numThreads, verbose);
  std::vector<T> euclideanWeights = {1.0, 0.5, 0.48, 0.0};
  std::vector<T> rotationalWeights = {0.0, 0.5, 0.52, 1.0};
  for (size_t i = 0; i < euclideanWeights.size(); i++) {
    T euclideanWeight = euclideanWeights[i];
    T rotationalWeight = rotationalWeights[i];
    std::string figPath = "../Results/RVG_Weight" + std::to_string(euclideanWeight) + "_" + std::to_string(rotationalWeight) + ".png";
    std::string gifPath = "../Results/RVG_Weight" + std::to_string(euclideanWeight) + "_" + std::to_string(rotationalWeight) + ".gif";
    visibilityGraph.setWeight(euclideanWeight, rotationalWeight);
    visibilityGraph.shortestPath(start, goal, 5);
    // visibilityGraph.draw(figPath, true, true, true);
    visibilityGraph.animation(gifPath, false);
    T pathLength = visibilityGraph.getPathLength();
    distsRVG.push_back(pathLength);
    rvgBuildTime.push_back(visibilityGraph.getBuildTime());
    rvgSearchTime.push_back(visibilityGraph.getSearchTime());
    print("RVG BuildTime(s): ", rvgBuildTime);
    print("RVG SearchTime(s):", rvgSearchTime);
    print("RVG Path Length:  ", distsRVG);
  }

  return 0;
}