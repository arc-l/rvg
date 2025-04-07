#ifndef ROTATIONALVISIBILITYGRAPH_PYTHON_UTILS_H_
#define ROTATIONALVISIBILITYGRAPH_PYTHON_UTILS_H_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tinyxml2.h>
#include <VisibilityGraph/Utils.h>
#include <VisibilityGraph/Polygon.h>

namespace py = pybind11;
namespace RotationalVisibilityGraph::Utils {
using T = double;

Polygon<T> getRobotWrapper(const std::string &path) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(path.c_str());
  return getRobot<T>(pt);
}

Polygon<T> getBoundaryWrapper(const std::string &path) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(path.c_str());
  const auto environment= pt.RootElement()->FirstChildElement("environment");
  bool useBoundary = get<bool>(*environment, "useBoundary", false);
  if (useBoundary) {
    return getBoundary<T>(pt);
  } else {
    int mapSize = get<int>(*environment, "mapSize", 100);
    return getMap<T>(mapSize);
  }
}

std::vector<Polygon<T>> getObstaclesWrapper(const std::string &path) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(path.c_str());
  return getObstacles<T>(pt);
}

Polygon<T> getMapWrapper(int mapSize) {
  return getMap<T>(mapSize);
}

std::shared_ptr<Vertex<T>> getStartWrapper(const std::string &path) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(path.c_str());
  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  const auto start = getVertex<T>(*plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex"));
  return start;
}

std::shared_ptr<Vertex<T>> getGoalWrapper(const std::string &path) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(path.c_str());
  const auto plannerSettings = pt.RootElement()->FirstChildElement("plannerSettings");
  const auto goal = getVertex<T>(*plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex"));
  return goal;
}

void init_utils(py::module &m) {
  m.def("get_robot", &RotationalVisibilityGraph::Utils::getRobotWrapper, "Get the robot polygon from XML");
  m.def("get_boundary", &RotationalVisibilityGraph::Utils::getBoundaryWrapper, "Get the boundary polygon from XML");
  m.def("get_obstacles", &RotationalVisibilityGraph::Utils::getObstaclesWrapper, "Get the obstacles polygons from XML");
  m.def("get_map", &RotationalVisibilityGraph::Utils::getMapWrapper, "Generate a map polygon of a given size");
  m.def("get_start", &RotationalVisibilityGraph::Utils::getStartWrapper, "Get the start vertex from XML");
  m.def("get_goal", &RotationalVisibilityGraph::Utils::getGoalWrapper, "Get the goal vertex from XML");
}

}// namespace RotationalVisibilityGraph

#endif //ROTATIONALVISIBILITYGRAPH_PYTHON_UTILS_H_
