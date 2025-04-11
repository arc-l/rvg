#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/RectangleObstacleMapGenerator.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/VisibilityGraph.h>
#include <VisibilityGraph/Utils.h>
#include <OMPL/OMPLEnv.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main() {
  Polygon<T> robot =
      Polygon<T>(
          std::vector<Vertex<T>>{
              Vertex<T>(-3, 0.5),
              Vertex<T>(-2, 0.5),
              Vertex<T>(-2, 2),
              Vertex<T>(-3, 2)
          },
          false);

  int mapSize = 50;
  int numberOfObstacles = 10;
  double minObstacleSize = 3;
  double maxObstacleSize = 5;
  std::shared_ptr<MapGenerator<T>>
      mapGenerator = std::make_shared<RectangleObstacleMapGenerator<T>>(mapSize, numberOfObstacles, minObstacleSize, maxObstacleSize, robot);
  mapGenerator->generateMap();

  std::vector<Vertex<T>> starts;
  std::vector<Vertex<T>> goals;
  for (int i = 0; i < 100; i++) {
    print("Generating start and goal ", i);
    mapGenerator->generateStartAndGoal();
    starts.push_back(mapGenerator->getStart());
    goals.push_back(mapGenerator->getGoal());
  }
  std::vector<Polygon<T>> obstacles = mapGenerator->getObstacles();
  Polygon<T> map = mapGenerator->getBorder();

  int resolution = 18;
  bool incremental = false;
  bool simpilfiedGeometry = true;
  int numThreads = 16;
  VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, simpilfiedGeometry, numThreads, incremental);
  visibilityGraph.setWeight(1.0, 0.);
  T buildTime = visibilityGraph.getBuildTime();
  T totoalQueryTime = 0;
  std::vector<T> distsRVG;
  for (size_t i = 0; i < starts.size(); i++) {
    Vertex<T> start = starts[i];
    Vertex<T> goal = goals[i];
    std::shared_ptr<Vertex<T>> startPtr = std::make_shared<Vertex<T>>(start);
    std::shared_ptr<Vertex<T>> goalPtr = std::make_shared<Vertex<T>>(goal);
    visibilityGraph.shortestPath(startPtr, goalPtr);
    visibilityGraph.draw("");
    totoalQueryTime += visibilityGraph.getSearchTime();
    distsRVG.push_back(visibilityGraph.getPathLength());
  }

  Env<T> omplEnv(obstacles, map, robot, Env<T>::SolverType::PRMStar);
  omplEnv.growRoadMap(buildTime);
  T totalOMPLQueryTime = 0;
  std::vector<T> distsOMPL;
  for (size_t i = 0; i < starts.size(); i++) {
    omplEnv.clearQuery();
    Vertex<T> start = starts[i];
    Vertex<T> goal = goals[i];
    if (start.getTheta() > PI) {
      start.setTheta(start.getTheta() - 2 * PI);
    }
    if (goal.getTheta() > PI) {
      goal.setTheta(goal.getTheta() - 2 * PI);
    }
    omplEnv.setStartGoal(start, goal);
    T dist = 0;
    Timer timer(false);
    omplEnv.incrementalPlan(0, dist);
    totalOMPLQueryTime += timer.elapsed();
    distsOMPL.push_back(dist);
  }
  print("RVG Build Time: ", buildTime);
  print("RVG Total Query Time: ", totoalQueryTime);
  print("RVG Distances: ", distsRVG);
  print("OMPL Total Query Time: ", totalOMPLQueryTime);
  print("OMPL Distances: ", distsOMPL);

  return 0;
}
