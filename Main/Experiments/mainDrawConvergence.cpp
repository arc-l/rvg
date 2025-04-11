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
  const auto start = getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  const auto goal = getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  print("Setup: ",
        "Start",
        start,
        "Goal",
        goal,
        "Resolutions",
        resolution,
        "Draw Graph",
        figPath
        );
  std::vector<T> distsRVG, rvgBuildTime, rvgSearchTime;
  setPrecision<T>(3);
  bool fineApprox = false;
  bool verbose = true;
  VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, fineApprox, numThreads, verbose);
  std::string pythonScript;
  PYTHON_IMPORTS_3D(pythonScript)
  pythonScript += visibilityGraph.drawLayers3D("obs3d");
  pythonScript += "plt.savefig('../Results/semi-algebraic-set-fit"+std::to_string(resolution)+".png')\n";
  pythonScript += "plt.show()";
  std::string pythonSavePath = "semi-algebraic-set-fit"+std::to_string(resolution)+".py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);

  return 0;
}