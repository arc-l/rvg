#include <OMPL/CollisionDetection.h>
#include <OMPL/OMPLEnv.h>

using namespace RotationalVisibilityGraph;
typedef double T;

int main() {
//  Polygon<T> robot =
//      Polygon<T>(
//          std::vector<Vertex<T>>{
//              Vertex<T>(-3, 0),
//              Vertex<T>(-2, 0),
//              Vertex<T>(-2, 2),
//              Vertex<T>(-3, 2)
//          },
//          false,
//          false,
//          false);
  Polygon<T> robot =
      Polygon<T>(
          std::vector<Vertex<T>>{
              Vertex<T>(-3, 0),
              Vertex<T>(-2.9, 0),
              Vertex<T>(-2.9, 0.1),
              Vertex<T>(-3, 0.1)
          },
          false);
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(2, -1),
      Vertex<T>(4, -1),
      Vertex<T>(4, -2),
      Vertex<T>(2, -2)
  };
  Polygon<T> polygon(vertices, true);
  std::vector<Vertex<T>> vertices1 = {
      Vertex<T>(2, 1),
      Vertex<T>(4, 1),
      Vertex<T>(4, 2),
      Vertex<T>(2, 2)
  };
  Polygon<T> polygon1 = Polygon(vertices1, true);
  std::vector<Vertex<T>> vertices2 = {
      Vertex<T>(-5., -3.),
      Vertex<T>(10., -3.),
      Vertex<T>(10., 3.),
      Vertex<T>(-5., 3.)
  };
  Polygon<T> map = Polygon<T>(vertices2, true);
  std::vector<Polygon<T>> obstacles = {polygon, polygon1};
  Vertex<T> start = robot.getCentroid();
  start.setBounds(0, 0);
  start.setTheta(0);
  Vertex<T> goal(8, 0, PI / 2., PI / 2., PI / 2, 2*PI, false);
//  Env<T>::SolverType solverType = Env<T>::SolverType::AITStar;
  Env<T>::SolverType solverType = Env<T>::SolverType::RRTStar;
  Env<T> omplEnv(obstacles, map, robot, solverType);
  omplEnv.setStartGoal(start, goal);
  std::vector<T> times = {0.05, 0.1, 0.5, 1, 5, 10, 15, 30};
//  std::vector<T> times = {180};
  std::vector<T> dists;
  for (auto time : times) {
    T dist = std::numeric_limits<T>::max();
//    omplEnv.plan(start, goal, time, dist);
    omplEnv.reset();
    omplEnv.incrementalPlan(time, dist);
    dists.push_back(dist);
  }
  Utils::print(dists);
  return 0;
}