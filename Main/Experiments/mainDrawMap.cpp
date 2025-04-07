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
  std::string figPath = argv[2];
  print("Config file path:", argv[1]);
  print("Fig path:", figPath);
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
  auto start = *getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  auto goal = *getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += "border = np.array([\n";
  std::vector<T> borderX = map.getX(), borderY = map.getY();
  for (size_t i = 0; i < map.getX().size(); i++) {
    pythonScript += "\t[" + std::to_string(borderX[i]) + ", " + std::to_string(borderY[i]) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-', color='darkcyan', linewidth=0.5)\n";
  for (const auto &obs : obstacles) {
    std::vector<T> obsX = obs.getX(), obsY = obs.getY();
    pythonScript += "obs = np.array([\n";
    for (size_t i = 0; i < obsX.size(); i++) {
      pythonScript += "\t[" + std::to_string(obsX[i]) + ", " + std::to_string(obsY[i]) + "],\n";
    }
    pythonScript += "])\n";
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-', color='darkcyan', markersize=0.5, linewidth=0.5)\n";
  }
  // start and goal
  robot.moveTo(start.getX(), start.getY(), start.getTheta());
  std::vector<T> startPolygonX = robot.getX(), startPolygonY = robot.getY();
  pythonScript += "start = np.array([\n";
  for (size_t i = 0; i < startPolygonX.size(); i++) {
    pythonScript += "\t[" + std::to_string(startPolygonX[i]) + ", " + std::to_string(startPolygonY[i]) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(start[:, 0], start[:, 1], '-o', color='deeppink', markersize=0.5, linewidth=0.5)\n";
  robot.moveTo(goal.getX(), goal.getY(), goal.getTheta());
  std::vector<T> goalPolygonX = robot.getX(), goalPolygonY = robot.getY();
  pythonScript += "goal = np.array([\n";
  for (size_t i = 0; i < goalPolygonX.size(); i++) {
    pythonScript += "\t[" + std::to_string(goalPolygonX[i]) + ", " + std::to_string(goalPolygonY[i]) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(goal[:, 0], goal[:, 1], '-o', color='darkviolet', markersize=0.5, linewidth=0.5)\n";
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('" + figPath + "', dpi=500)\n";
  // pythonScript += "plt.show()\n";
  std::string pythonSavePath = "map.py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
#ifdef PYTHON_EXECUTABLE
  Utils::print("Python path:", PYTHON_EXECUTABLE);
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  system(command.c_str());
  std::string rmCommand = "rm " + pythonSavePath;
  system(rmCommand.c_str());

  return 0;
}
