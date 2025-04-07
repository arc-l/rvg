#include "OMPLEnv.h"
#include <cstdlib>

namespace RotationalVisibilityGraph {
template<typename T>
Env<T>::Env(const std::vector<Polygon<T>> &obstacles,
            const Polygon<T> &border,
            Polygon<T> &robot,
            SolverType solverType) : _obstacles(
    obstacles), _border(border), _robot(robot), _solverType(solverType) {
  ob::RealVectorBounds bounds(2);
  auto xs = _border.getX();
  auto ys = _border.getY();
  auto min_x = *std::min_element(xs.begin(), xs.end());
  auto max_x = *std::max_element(xs.begin(), xs.end());
  auto min_y = *std::min_element(ys.begin(), ys.end());
  auto max_y = *std::max_element(ys.begin(), ys.end());
  Utils::print(
      "min_x: " + std::to_string(min_x) + " max_x: " + std::to_string(max_x) + " min_y: " + std::to_string(min_y)
          + " max_y: " + std::to_string(max_y));
  bounds.setLow(0, min_x);
  bounds.setLow(1, min_y);
  bounds.setHigh(0, max_x);
  bounds.setHigh(1, max_y);
  _space = std::make_shared<ob::SE2StateSpace>();
  _space->setBounds(bounds);
  _space->setSubspaceWeight(1, 0);
  _si = std::make_shared<ob::SpaceInformation>(_space);
  _validityChecker = std::make_shared<ValidityChecker<T>>(_obstacles, _robot, _si);
  _si->setStateValidityChecker(_validityChecker);
  switch (_solverType) {
    case SolverType::RRTStar:
      Utils::print("Using RRTStar");
      _planner = std::make_shared<og::RRTstar>(_si);
      _planner->as<og::RRTstar>()->setBatchSize(100);
      Utils::print("Range: ", _planner->as<og::RRTstar>()->getRange());
      Utils::print("Batch size: ", _planner->as<og::RRTstar>()->getBatchSize());
      break;
    case SolverType::AITStar:
      Utils::print("Using AITStar");
      _planner = std::make_shared<og::AITstar>(_si);
      _planner->as<og::AITstar>()->setBatchSize(100);
      break;
    case SolverType::BITStar:
      Utils::print("Using BITStar");
      _planner = std::make_shared<og::BITstar>(_si);
      break;
    case SolverType::EITStar:
      Utils::print("Using EITStar");
      _planner = std::make_shared<og::EITstar>(_si);
      break;
    case SolverType::FMT:
      Utils::print("Using FMT");
      _planner = std::make_shared<og::FMT>(_si);
      break;
    case SolverType::PRMStar:
      Utils::print("Using PRMStar");
      _planner = std::make_shared<og::PRMstar>(_si);
      break;
    case SolverType::LazyPRMStar:
      Utils::print("Using LazyPRMStar");
      _planner = std::make_shared<og::LazyPRMstar>(_si);
      break;
    case SolverType::LBTRRT:
      Utils::print("Using LBT-RRT");
      _planner = std::make_shared<og::LBTRRT>(_si);
      break;
    default:
      throw std::runtime_error("Solver type not supported");
  }
  _pdef = std::make_shared<ob::ProblemDefinition>(_si);
  _pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(_si));
  _planner->setProblemDefinition(_pdef);
  _planner->setup();
}

template<typename T>
void Env<T>::reset() {
  _planner->clear();
}

template<typename T>
void Env<T>::setStartGoal(const Vertex<T> &startVertex, const Vertex<T> &goalVertex) {
  _start = startVertex;
  _goal = goalVertex;
  ob::ScopedState<ob::SE2StateSpace> start(_space);
  start->setX(startVertex.getX());
  start->setY(startVertex.getY());
  start->setYaw(startVertex.getTheta());
  Utils::print("start:", startVertex.getX(), startVertex.getY(), startVertex.getTheta());
  Utils::print("set start:", start->getX(), start->getY(), start->getYaw());
  ob::ScopedState<ob::SE2StateSpace> goal(_space);
  goal->setX(goalVertex.getX());
  goal->setY(goalVertex.getY());
  goal->setYaw(goalVertex.getTheta());
  Utils::print("goal:", goalVertex.getX(), goalVertex.getY(), goalVertex.getTheta());
  Utils::print("set goal:", goal->getX(), goal->getY(), goal->getYaw());
  _pdef->setStartAndGoalStates(start, goal, 0);
}

template<typename T>
void Env<T>::incrementalPlan(T time, T &dist) {
  ob::PlannerStatus solved = _planner->ob::Planner::solve(time);
  Utils::print("Solved:", solved);
  if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
    ob::PathPtr path = _pdef->getSolutionPath();
    std::cout
        << _planner->getName()
        << " found a solution of length "
        << _pdef->getSolutionPath()->length()
        << " with an optimization objective value of "
        << _pdef->getSolutionPath()->cost(_pdef->getOptimizationObjective()) << std::endl;
    const auto &states = path->as<og::PathGeometric>()->getStates();
    _sol.clear();
    for (size_t i = 0; i < states.size(); i++) {
      const auto *state = states[i]->as<ob::SE2StateSpace::StateType>();
      _sol.push_back(Vertex<T>(state->getX(), state->getY(), 0, 2*PI, state->getYaw(), 2*PI, true));
    }
    Utils::print("Solution=", _sol);
    dist = 0;
    for (size_t i = 1; i < states.size(); i++) {
      const auto *state0 = states[i - 1]->as<ob::SE2StateSpace::StateType>();
      const auto *state1 = states[i]->as<ob::SE2StateSpace::StateType>();
      dist += std::sqrt(std::pow(state0->getX() - state1->getX(), 2) + std::pow(state0->getY() - state1->getY(), 2));
    }
  } 
  else {
    std::cout << "No solution found" << std::endl;
  }
}

template<typename T>
void Env<T>::growRoadMap(T time) {
  if (_solverType == SolverType::PRMStar)
    _planner->as<og::PRMstar>()->growRoadmap(time);
  else
    throw std::runtime_error("This function is only supported for PRMStar");
}

template<typename T>
void Env<T>::clearQuery() {
  if (_solverType == SolverType::PRMStar)
  {
    _planner->as<og::PRMstar>()->clearQuery();
    _pdef->clearSolutionPaths();
    _pdef->clearStartStates();
    _pdef->clearGoal();
  }
  else
    throw std::runtime_error("This function is only supported for PRMStar");
}

template<typename T>
void Env<T>::draw(const std::string & figPath, bool show){
  if (_sol.empty()) {
    Utils::print("No solution found! Cannot draw path");
    return;
  }
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript);
  pythonScript += _drawSetup();
  pythonScript += "path = np.array([\n";
  for (const auto &vertex : _sol) {
    pythonScript += "\t[" + std::to_string(vertex.getX()) + ", " +
                    std::to_string(vertex.getY()) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(path[:, 0], path[:, 1], '-o', color='navy', markersize=1.0, linewidth=1.0)\n";
  // robot along the path
  if (_sol.size() >= 2) {
    for (size_t nodeNum = 1; nodeNum < _sol.size() - 1; nodeNum++) {
      const auto &vertex = _sol[nodeNum];
      Polygon<T> tmpRobot = _robot.moveToCopy(
          vertex.getX(), vertex.getY(), vertex.getTheta());
      pythonScript += tmpRobot.draw("path");
    }
  }
  pythonScript += "plt.axis('equal')\n";
  // pythonScript += "plt.title('path length=" + std::to_string(dist) + "')\n";
  pythonScript += "plt.savefig('" + figPath + "', dpi=500)\n";
  if(show) pythonScript += "plt.show()\n";
  std::string pythonSavePath = "path.py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);
}

template<typename T>
void Env<T>::animation(const std::string & figPath){
  if (_sol.empty()) {
    Utils::print("No solution found! Cannot create animation");
    return;
  }

  if(figPath.empty()) return;

  std::string pythonScript;
  // use pillow to create gif
  PYTHON_IMPORTS(pythonScript)
  pythonScript += "image_paths = []\n";
  for (size_t i = 0; i < _sol.size(); i++) {
    pythonScript += "fig, ax = plt.subplots()\n";
    pythonScript += "ax.set_facecolor('gainsboro')\n";
    pythonScript += "plt.axis('equal')\n";
    pythonScript += _drawSetup();
    const auto &vertex = _sol[i];
    Polygon<T> tmpRobot = _robot.moveToCopy(vertex.getX(), vertex.getY(), vertex.getTheta());
    pythonScript += tmpRobot.draw("path");
    pythonScript += "plt.savefig('tmp" + std::to_string(i) +
                    ".png', dpi=300, bbox_inches='tight')\n";
    pythonScript += "plt.close(fig)\n";
    pythonScript += "image_paths.append('tmp" + std::to_string(i) + ".png')\n";
  }
  pythonScript += "images = [Image.open(img) for img in image_paths]\n";
  pythonScript += "images[0].save('" + figPath +
                  "', save_all=True, append_images=images[1:], optimize=False, "
                  "duration=50, loop=0)\n";
  pythonScript += "for img in image_paths:\n";
  pythonScript += "\tos.remove(img)\n";
  std::string pythonSavePath = "animation.py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);
}

template <typename T>
std::string Env<T>::_drawSetup() const {
  std::string pythonScript;
  pythonScript += _border.draw("obs");
  for (const auto &obs : _obstacles) {
    pythonScript += obs.draw("obs");
  }
  // start and goal
  Polygon<T> tmpRobot = _robot.moveToCopy(_start.getX(), _start.getY(), _start.getTheta());
  pythonScript += tmpRobot.draw("start");
  tmpRobot = _robot.moveToCopy(_goal.getX(), _goal.getY(), _goal.getTheta());
  pythonScript += tmpRobot.draw("goal");
  return pythonScript;
}


template
class Env<double>;
template
class Env<float>;
}
