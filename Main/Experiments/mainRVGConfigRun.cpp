#include <iomanip>
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

namespace {

template <typename U>
size_t graphEdgeCount(const Graph<U> &graph) {
  size_t degreeSum = 0;
  for (const auto &entry : graph.getAdjacencyList()) {
    degreeSum += entry.second.size();
  }
  return degreeSum / 2;
}

VertexPropagationBackend parsePropagationBackend(const std::string &value) {
  if (value == "auto") return VertexPropagationBackend::Auto;
  if (value == "cpu") return VertexPropagationBackend::Cpu;
  if (value == "cpu_reference") return VertexPropagationBackend::CpuReference;
  if (value == "cuda") return VertexPropagationBackend::Cuda;
  throw std::invalid_argument(
      "Unknown propagation backend '" + value +
      "'; expected auto, cpu, cpu_reference, or cuda"
  );
}


}

int main(int argc, char *argv[]) {
  if (argc < 4) throw std::runtime_error("Expected CONFIG RESOLUTION THREADS");
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  int resolution = std::stoi(argv[2]);
  int numThreads = std::stoi(argv[3]);
  std::string figPath;
  VertexPropagationBackend propagationBackend = VertexPropagationBackend::Auto;
  for (int argument = 4; argument < argc; ++argument) {
    const std::string value = argv[argument];
    if (value == "--propagation-backend" && argument + 1 < argc) {
      propagationBackend = parsePropagationBackend(argv[++argument]);
    } else if (value.rfind("--propagation-backend=", 0) == 0) {
      propagationBackend = parsePropagationBackend(value.substr(22));
    } else if (figPath.empty()) {
      figPath = value;
    } else {
      throw std::invalid_argument("Unexpected argument '" + value + "'");
    }
  }
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
  std::cout << std::fixed << std::setprecision(3);
  bool optimal = true;
  bool verbose = true;
  VisibilityGraph<T> visibilityGraph(
      robot,
      map,
      obstacles,
      resolution,
      numThreads,
      optimal,
      verbose,
      propagationBackend
  );
  visibilityGraph.setWeight(1.0, 0.1);
  visibilityGraph.shortestPath(start, goal, 5);
  T pathLength = visibilityGraph.getPathLength();
  // T totalTime = visibilityGraph.getTotalTime();
  distsRVG.push_back(pathLength);
  rvgBuildTime.push_back(visibilityGraph.getBuildTime());
  rvgSearchTime.push_back(visibilityGraph.getSearchTime());
  print("RVG BuildTime(s): ", rvgBuildTime);
  print("RVG SearchTime(s):", rvgSearchTime);
  print("RVG Path Length:  ", distsRVG);
  const Graph<T> &graph = visibilityGraph.getGraph();
  print("RVG #vertices:", graph.vertexCount());
  print("RVG #edges:", graph.edgeCount());
  // visibilityGraph.drawVisibleAreas();
  if(figPath.empty()) return 0;
  visibilityGraph.animation(figPath, false);
  // change the figPath's extension to .png
  figPath = figPath.substr(0, figPath.find_last_of('.')) + ".png";
  // for (int i = 0; i < layers.size(); i++) {
  //   const auto &layer = layers[i];
  //   std::string layerFigPath = figPath.substr(0, figPath.find_last_of('.')) + "_layer_" + std::to_string(i) + ".png";
  //   layer.draw(layerFigPath, map, obstacles, nullptr, false);
  // }
  visibilityGraph.draw(figPath, false, true, true, false, -1);
  // visibilityGraph.draw3D(false);
  return 0;

}
