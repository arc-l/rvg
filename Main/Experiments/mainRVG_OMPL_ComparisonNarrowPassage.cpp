#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/VisibilityGraph.h>
#include <OMPL/CollisionDetection.h>
#include <OMPL/OMPLEnv.h>

using namespace RotationalVisibilityGraph;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main(int argc, char *argv[]) {
  if (argc != 3) {
    Utils::print("Usage: ./mainRVG_OMPL_ComparisonNarrowPassage <plannerType> <gapWidth>");
    return 1;
  }

  std::string plannerType = argv[1];
  T gapWidth = std::stod(argv[2]);

  Polygon<T> robot =
      Polygon<T>(
          std::vector<Vertex<T>>{
              Vertex<T>(-3, 0.5),
              Vertex<T>(-2, 0.5),
              Vertex<T>(-2, 2),
              Vertex<T>(-3, 2)
          },
          false);
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(2, -gapWidth / 2),
      Vertex<T>(4, -gapWidth / 2),
      Vertex<T>(4, -7),
      Vertex<T>(2, -7)
  };
  Polygon<T> polygon(vertices, true);
  std::vector<Vertex<T>> vertices1 = {
      Vertex<T>(2, gapWidth / 2),
      Vertex<T>(4, gapWidth / 2),
      Vertex<T>(4, 7),
      Vertex<T>(2, 7)
  };
  Polygon<T> polygon1 = Polygon(vertices1, true);
  T map_size = 20.;
  std::vector<Vertex<T>> vertices2 = {
      Vertex<T>(-map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., map_size / 2.),
      Vertex<T>(-map_size / 2., map_size / 2.)
  };
  Polygon<T> map = Polygon<T>(vertices2, true);
  std::vector<Polygon<T>> obstacles = {polygon, polygon1};
  Vertex<T> start = robot.getCentroid();
  start.setBounds(0, 2 * PI);
  start.setTheta(0);
  Vertex<T> goal(8, 0, 0, 2 * PI, PI / 2, 2 * PI, false);
  std::shared_ptr<Vertex<T>> startPtr = std::make_shared<Vertex<T>>(start);
  std::shared_ptr<Vertex<T>> goalPtr = std::make_shared<Vertex<T>>(goal);

  std::vector<T> distsRVG, rvgTime;
  if (plannerType == "RRTStar" || plannerType == "AITStar") {
    std::vector<T> distsOMPL;
    Env<T>::SolverType solverType;
    if (plannerType == "RRTStar")
      solverType = Env<T>::SolverType::RRTStar;
    else if (plannerType == "AITStar")
      solverType = Env<T>::SolverType::AITStar;
    Env<T> omplEnv(obstacles, map, robot, solverType);
    omplEnv.setStartGoal(start, goal);
    T time = 10;
    for (int cnt = 0; cnt < 10; cnt++) {
      T dist = 0;
      omplEnv.reset();
      omplEnv.incrementalPlan(time, dist);
      distsOMPL.push_back(dist);
    }
    Utils::writeVectorToFile(distsOMPL, "../Results/" + plannerType + "_dists_" + plannerType + "_" + std::to_string(gapWidth) + ".txt");
  } else if (plannerType == "RVG") {
    std::vector<int> resolutions = {9, 18, 36, 60, 72, 90, 180, 360};
    for (int resolution : resolutions) {
      bool hashWithTheta = true;
      bool incremental = true;
      bool simpilfiedGeometry = true;
      bool considerSymmetry = true;
      int numThreads = 16;
      VisibilityGraph<T> visibilityGraph = VisibilityGraph<T>(robot, map, obstacles, resolution, considerSymmetry, hashWithTheta, simpilfiedGeometry, numThreads, incremental);
      visibilityGraph.shortestPath(startPtr, goalPtr);
      T pathLength = visibilityGraph.getPathLength();
      T totalTime = visibilityGraph.getTotalTime();
      distsRVG.push_back(pathLength);
      rvgTime.push_back(totalTime);
    }
    Utils::writeVectorToFile(distsRVG, "../Results/RVG_dists_" + std::to_string(gapWidth) + ".txt");
    Utils::writeVectorToFile(rvgTime, "../Results/RVG_times_" + std::to_string(gapWidth) + ".txt");
  } else {
    Utils::print("Invalid planner type");
    return 1;
  }
  return 0;
}
