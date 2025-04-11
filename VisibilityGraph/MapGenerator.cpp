#include "MapGenerator.h"
#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/VisibilityCheck.h>
#include <VisibilityGraph/Utils.h>
#include <random>
#include <CGAL/draw_arrangement_2.h>

namespace RotationalVisibilityGraph {
template<typename T>
MapGenerator<T>::MapGenerator(int mapSize, int numObstacles, T maxObstacleSize, const Polygon<T> &robot) :
    _mapSize(mapSize),
    _numberObstacles(numObstacles),
    _maxObstacleSize(maxObstacleSize),
    _robot(robot) {
  const auto &centroid = _robot.getCentroid();
  _robotRadius = centroid.dist(_robot[0]);
  _marginSize = _robotRadius * 3;
  _generateBorder();
}

template<typename T>
Vertex<T> MapGenerator<T>::getStart() const {
  return _start;
}

template<typename T>
Vertex<T> MapGenerator<T>::getGoal() const {
  return _goal;
}

template<typename T>
std::vector<Polygon<T>> MapGenerator<T>::getObstacles() const {
  return _obstacles;
}

template<typename T>
Polygon<T> MapGenerator<T>::getBorder() const {
  return _border;
}

template<typename T>
int MapGenerator<T>::_randomInt(int min, int max) {
  std::mt19937 gen(std::random_device{}());
  std::uniform_int_distribution<int> dis_int(min, max);
  int random_int = dis_int(gen);
  return random_int;
}

template<typename T>
T MapGenerator<T>::_randomReal(T min, T max) {
  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<T> dis_real(min, max);
  T random_real = dis_real(gen);
  return random_real;
}

template<typename T>
std::vector<T> MapGenerator<T>::_randomVector(int size, T min, T max) {
  std::vector<T> random_vector;
  for (int i = 0; i < size; i++) {
    random_vector.push_back(_randomReal(min, max));
  }
  return random_vector;
}

template<typename T>
Polygon<T> MapGenerator<T>::_generateObstacle() {
  int numVertices = _randomInt(3, 10);
  std::vector<T> obstacle = _randomVector(numVertices * 2, -_maxObstacleSize / 2., _maxObstacleSize / 2.);
  std::vector<Vertex<T>> vertices;
  for (int i = 0; i < numVertices; i++) {
    vertices.emplace_back(obstacle[i * 2], obstacle[i * 2 + 1]);
  }
  Polygon<T> polygon(vertices, true);
  return polygon;
}

template<typename T>
void MapGenerator<T>::_generateBorder() {
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(-_mapSize / 2., -_mapSize / 2.),
      Vertex<T>(_mapSize / 2., -_mapSize / 2.),
      Vertex<T>(_mapSize / 2., _mapSize / 2.),
      Vertex<T>(-_mapSize / 2., _mapSize / 2.)
  };
  _border = Polygon<T>(vertices, false);
  std::vector<Vertex<T>> marginVertices = {
      Vertex<T>(-_mapSize / 2. + _marginSize, -_mapSize / 2. + _marginSize),
      Vertex<T>(_mapSize / 2. - _marginSize, -_mapSize / 2. + _marginSize),
      Vertex<T>(_mapSize / 2. - _marginSize, _mapSize / 2. - _marginSize),
      Vertex<T>(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize)
  };
  _margin = Polygon<T>(marginVertices, false);
}

template<typename T>
void MapGenerator<T>::_generateObstacles(bool incremental) {
  if (!incremental) _obstacles.clear();
  for (int i = 0; i < _numberObstacles; i++) {
    Polygon<T> obstacle = _generateObstacle();
    std::vector<T> centroid = _randomVector(2, -_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize);
    obstacle.moveTo(centroid[0], centroid[1], 0);
    while (_outsideBorder(obstacle)) {
      centroid = _randomVector(2, -_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize);
      obstacle.moveTo(centroid[0], centroid[1], 0);
    }
    _obstacles.push_back(obstacle);
  }
  // merge the intersecting obstacles
  while (true) {
    bool foundIntersection = false;
    for (size_t i = 0; i < _obstacles.size(); i++) {
      size_t j = i + 1;
      while (j < _obstacles.size()) {
        if (_obstacles[i].intersects(_obstacles[j])) {
          foundIntersection = true;
          _obstacles[i] = _obstacles[i].merge(_obstacles[j]);
          _obstacles.erase(_obstacles.begin() + j);
        } else j++;
      }
    }
    if (!foundIntersection) break;
  }
}

template<typename T>
void MapGenerator<T>::generateStartAndGoal() {
  _start = _getLegalConfiguration();
  Utils::print("start=", _start);
  std::vector<Polygon_2> map, printMap;
  for (const auto &obs : _obstacles) {
    map.push_back(obs.getPolygon());
  }
  map.push_back(_border.getPolygon());
  auto env = obsToArrangement<T>(map);
  std::shared_ptr<VQ> vq = std::make_shared<VQ>(env);
  auto visibleArea = visibleAreaInMap<T>(_start.getPoint(), env, vq);
  _goal = _getLegalConfiguration();
  int cnt = 0;
  while (true) {
    const auto &p = _goal.getPoint();
    CGAL::Arr_naive_point_location<Arrangement_2> visPl(visibleArea);
    CGAL::Arr_point_location_result<Arrangement_2>::Type visObj = visPl.locate(p);
    auto fh = boost::get<Arrangement_2::Face_const_handle>(&visObj);
    if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) || (fh && !(*fh)->is_unbounded()) || _start.dist(_goal) < std::sqrt(2) * _mapSize / 2.) {
      if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) || (fh && !(*fh)->is_unbounded())) {
        _goal = _getLegalConfiguration();
      }
      else{
        _goal = _getLegalConfiguration();
        cnt++;
        if (cnt > 1000) {
          break;
        }
      }
    } else break;
  }
  Utils::print("goal=", _goal);
}

template<typename T>
void MapGenerator<T>::drawMap(const std::string &identifier) const {
  std::string pythonScript = "import numpy as np\n";
  pythonScript += "import matplotlib.pyplot as plt\n";
  pythonScript += "\n";
  pythonScript += "fig, ax = plt.subplots()\n";
  pythonScript += "ax.set_facecolor('gainsboro')\n";
  pythonScript += "border = np.array([\n";
  std::vector<T> borderX = _border.getX(), borderY = _border.getY();
  for (size_t i = 0; i < _border.getX().size(); i++) {
    pythonScript += "\t[" + std::to_string(borderX[i]) + ", " + std::to_string(borderY[i]) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(border[:, 0], border[:, 1], '-g')\n";
  for (const auto &obs : _obstacles) {
    std::vector<T> obsX = obs.getX(), obsY = obs.getY();
    pythonScript += "obs = np.array([\n";
    for (size_t i = 0; i < obsX.size(); i++) {
      pythonScript += "\t[" + std::to_string(obsX[i]) + ", " + std::to_string(obsY[i]) + "],\n";
    }
    pythonScript += "])\n";
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-o', color='c', markersize=1.2)\n";
  }
  // start and goal
  pythonScript +=
      "start = np.array([[" + std::to_string(_start.getX()) + ", " + std::to_string(_start.getY()) + "]])\n";
  pythonScript += "ax.plot(start[:, 0], start[:, 1], 'ro')\n";
  pythonScript +=
      "goal = np.array([[" + std::to_string(_goal.getX()) + ", " + std::to_string(_goal.getY()) + "]])\n";
  pythonScript += "ax.plot(goal[:, 0], goal[:, 1], 'bo')\n";
  pythonScript +=
      "plt.title('RandomMap MapSize=" + std::to_string(_mapSize) + " #Obstalces=" + std::to_string(_numberObstacles)
          + " MaxObsSize=" + std::to_string(_maxObstacleSize) + "')\n";
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('path" + identifier + "_" + std::to_string(_mapSize) + "_" + std::to_string(_numberObstacles) + "_"
      + std::to_string(_maxObstacleSize) + ".png', dpi=500)\n";
  std::string pythonSavePath = "path" + std::to_string(_mapSize) + "_" + std::to_string(_numberObstacles) + "_"
      + std::to_string(_maxObstacleSize) + ".py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  system(command.c_str());
}

template<typename T>
bool MapGenerator<T>::_outsideBorder(const Polygon<T> &obs) const {
  return !std::all_of(obs.getVertices().begin(), obs.getVertices().end(), [&](const Vertex<T> &v) {
    return _margin.contains(v);
  });
}

template<typename T>
Vertex<T> MapGenerator<T>::_getLegalConfiguration() {
  Vertex<T> vertex(
      _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
      _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
      0,
      2 * PI,
      _randomReal(0, 2 * PI)
  );
  while (!_isLegalConfiguration(vertex)) {
    vertex = Vertex<T>(
        _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
        _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
        0,
        2 * PI,
        _randomReal(0, 2 * PI)
    );
  }
  return vertex;
}

template<typename T>
bool MapGenerator<T>::_circleIntersectsObstacle(const Polygon<T> &obs, const Vertex<T> &center, T radius) const {
  const auto &polygon = obs.getPolygon();
  const auto &p = center.getPoint();
  return std::any_of(polygon.edges_begin(), polygon.edges_end(), [&](const auto &edge) {
    return CGAL::squared_distance(edge, p) < radius * radius;
  });
}

template<typename T>
bool MapGenerator<T>::_isLegalConfiguration(const Vertex<T> &v) const {
  std::vector<Polygon_2> map;
  for (const auto &obs : _obstacles)
    map.push_back(obs.getPolygon());
  const auto &p = v.getPoint();
  return (!IN_ANY_POLYGON(p, map)) && !std::any_of(_obstacles.begin(), _obstacles.end(), [&](const Polygon<T> &obs) {
    return _circleIntersectsObstacle(obs, v, _robotRadius);
  });
}

template<typename T>
void MapGenerator<T>::generateMap() {
  reset();
  _generateObstacles(false);
  generateStartAndGoal();
}

template<typename T>
void MapGenerator<T>::reset() {
  _obstacles.clear();
  _start = _goal = Vertex<T>();
}

template<typename T>
void MapGenerator<T>::writeMap(const std::string &filename) const {
  // write to tinyxml2
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement *root = doc.NewElement("root");
  doc.InsertFirstChild(root);
  tinyxml2::XMLElement *environment = doc.NewElement("environment");
  root->InsertFirstChild(environment);
  tinyxml2::XMLElement *robot = doc.NewElement("robot");
  environment->InsertFirstChild(robot);
  Utils::putPolygon<T>(doc, *robot, _robot);
  tinyxml2::XMLElement *obstacles = doc.NewElement("obstacles");
  environment->InsertEndChild(obstacles);
  for (const auto &obs : _obstacles) {
    Utils::putPolygon<T>(doc, *obstacles, obs);
  }
  tinyxml2::XMLElement *mapSize = doc.NewElement("mapSize");
  mapSize->SetText(_mapSize);
  environment->InsertEndChild(mapSize);
  tinyxml2::XMLElement *plannerSettings = doc.NewElement("plannerSettings");
  root->InsertEndChild(plannerSettings);
  tinyxml2::XMLElement *start = doc.NewElement("start");
  plannerSettings->InsertFirstChild(start);
  Utils::putVertex<T>(doc, *start, _start);
  tinyxml2::XMLElement *goal = doc.NewElement("goal");
  plannerSettings->InsertEndChild(goal);
  Utils::putVertex<T>(doc, *goal, _goal);
  doc.SaveFile(filename.c_str());
}

template<typename T>
void MapGenerator<T>::generateFixedStartAndGoalMap(const Vertex<T> &start, const Vertex<T> &goal) {
  MapGenerator<T>::reset();
  _start = start;
  _goal = goal;
  MapGenerator<T>::_generateObstacles(false);
  while (!MapGenerator<T>::_isLegalConfiguration(_start) || !MapGenerator<T>::_isLegalConfiguration(_goal) || _isStartAndGoalVisible()) {
    MapGenerator<T>::_generateObstacles(false);
  }
}

template<typename T>
void MapGenerator<T>::generateFixedStartAndGoalMap() {
  // start and goal are fixed to be at the top left and bottom right corners
  Vertex<T> start(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize, 0, 0, 0);
  Vertex<T> goal(_mapSize / 2. - _marginSize, -_mapSize / 2. + _marginSize, 0, 0, 0);
  Utils::print("start=", start);
  Utils::print("goal=", goal);
  generateFixedStartAndGoalMap(start, goal);
}

template<typename T>
bool MapGenerator<T>::_isStartAndGoalVisible() const {
  std::vector<Polygon_2> map, printMap;
  for (const auto &obs : _obstacles) {
    map.push_back(obs.getPolygon());
  }
  map.push_back(_border.getPolygon());
  auto env = obsToArrangement<T>(map);
  std::shared_ptr<VQ> vq = std::make_shared<VQ>(env);
  auto visibleArea = visibleAreaInMap<T>(_start.getPoint(), env, vq);
  const auto &p = _goal.getPoint();
  CGAL::Arr_naive_point_location<Arrangement_2> visPl(visibleArea);
  CGAL::Arr_point_location_result<Arrangement_2>::Type visObj = visPl.locate(p);
  auto fh = boost::get<Arrangement_2::Face_const_handle>(&visObj);
  if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) || (fh && !(*fh)->is_unbounded()))
    return true;
  else return false;
}


template
class MapGenerator<double>;
template
class MapGenerator<float>;
}
