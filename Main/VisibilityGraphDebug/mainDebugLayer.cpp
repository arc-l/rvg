#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Layer.h>
#include <VisibilityGraph/Utils.h>
#include <cmath>

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;

typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main(int argc, char *argv[]) {
  if (argc < 3) throw std::runtime_error("No input file provided");
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  std::string figPath = argv[2];
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
  T resolution = 36;
  bool fineApprox = false;
  for (int i = 0; i < resolution; i++) {
    Graph<T> graph;
    Layer<T> layer = Layer<T>(2 * PI * i / resolution, 2 * PI * (i + 1) / resolution, fineApprox, false);
    layer.buildVisibilityGraph(robot, map, obstacles);
    std::vector<Vertex_const_handle> vertexHandles = layer.getVertexHandles();
    graph.addEdges(layer.getEdges());
    // substitute figpath with a string that contains the index of the layer
    std::string newFigPath = "../Results/layer_" + std::to_string(i) + ".png";
    layer.drawRobotBB(robot);
    layer.draw(newFigPath, map, obstacles, &graph, true);
  }
  return 0;
}