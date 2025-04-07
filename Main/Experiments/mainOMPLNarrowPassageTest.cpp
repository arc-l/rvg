#include <OMPL/CollisionDetection.h>
#include <OMPL/OMPLEnv.h>

using namespace RotationalVisibilityGraph;
typedef double T;

int main() {
  Polygon<T> robot =
      Polygon<T>(
          std::vector<Vertex<T>>{
              Vertex<T>(-3, 0),
              Vertex<T>(-2, 0),
              Vertex<T>(-2, 2),
              Vertex<T>(-3, 2)
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
      Vertex<T>(10.,  3.),
      Vertex<T>(-5.,  3.)
  };
  Polygon<T> map = Polygon<T>(vertices2, true);
  std::vector<Polygon<T>> obstacles = {polygon, polygon1};
  Vertex<T> start = robot.getCentroid();
  start.setBounds(0, 0);
  start.setTheta(0);
  Vertex<T> goal(8, 0, PI / 2., PI / 2., PI / 2, 2 * PI, false);
//  Env<T>::SolverType solverType = Env<T>::SolverType::AITStar;
  Env<T>::SolverType solverType = Env<T>::SolverType::RRTStar;
//  Env<T>::SolverType solverType = Env<T>::SolverType::RRTConnect;
  Env<T> omplEnv(obstacles, map, robot, solverType);
  omplEnv.setStartGoal(start, goal);
  T time = 10;
  T totalTime = 0;
  T dist = std::numeric_limits<T>::max();
  while (dist < 10 || dist > 11) {
    omplEnv.reset();
    omplEnv.incrementalPlan(time, dist);
    totalTime += time;
    Utils::print("Time: " + std::to_string(totalTime) + " dist: " + std::to_string(dist));
  }
  Utils::print("Total time: " + std::to_string(totalTime));
  return 0;
}
