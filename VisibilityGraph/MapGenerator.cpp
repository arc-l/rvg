#include "MapGenerator.h"
#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/VisibilityCheck.h>
#include <VisibilityGraph/VisibilityGraph.h>
#include <VisibilityGraph/Utils.h>
#include <CGAL/draw_arrangement_2.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <list>
#include <sstream>
#include <stdexcept>

namespace RotationalVisibilityGraph {

namespace {

constexpr int MAX_OBSTACLE_PLACEMENT_ATTEMPTS = 10000;
constexpr int MAX_MAP_GENERATION_ATTEMPTS = 100;

}

template<typename T>
MapGenerator<T>::MapGenerator(int mapSize,
                              int numObstacles,
                              T maxObstacleSize,
                              const Polygon<T> &robot)
    : MapGenerator(
          mapSize,
          numObstacles,
          maxObstacleSize,
          robot,
          std::random_device{}()
      ) {}

template<typename T>
MapGenerator<T>::MapGenerator(int mapSize,
                              int numObstacles,
                              T maxObstacleSize,
                              const Polygon<T> &robot,
                              std::uint32_t seed) :
    _mapSize(mapSize),
    _numberObstacles(numObstacles),
    _maxObstacleSize(maxObstacleSize),
    _robot(robot),
    _seed(seed),
    _rng(seed),
    _benchmarkMode(false),
    _protectStartAndGoal(false),
    _difficulty(MapDifficulty::Medium),
    _sampledObstacleCount(0),
    _borderCrossingCount(0),
    _successfulAttempt(0),
    _staticRvgResolution(0) {
  const auto &centroid = _robot.getCentroid();
  _robotRadius = centroid.dist(_robot[0]);
  _marginSize = _robotRadius * 3;
  _generateBorder();
}

template<typename T>
MapGenerator<T>::MapGenerator(int mapSize,
                              MapDifficulty difficulty,
                              std::uint32_t seed,
                              const Polygon<T> &robot)
    : MapGenerator(
          mapSize,
          std::max(
              1,
              static_cast<int>(std::lround(
                  getDifficultyProfile(difficulty).obstacleCount *
                  static_cast<double>(mapSize) *
                  static_cast<double>(mapSize) / 900.0
              ))
          ),
          static_cast<T>(
              getDifficultyProfile(difficulty).maxObstacleSize *
              static_cast<double>(mapSize) / 30.0
          ),
          robot,
          seed
      ) {
  _benchmarkMode = true;
  _difficulty = difficulty;
}

template<typename T>
MapDifficultyProfile MapGenerator<T>::getDifficultyProfile(MapDifficulty difficulty) {
  switch (difficulty) {
    case MapDifficulty::VeryEasy:
      return {difficulty, 0.0, 10, 6.0};
    case MapDifficulty::Easy:
      return {difficulty, 0.25, 25, 4.875};
    case MapDifficulty::Medium:
      return {difficulty, 0.5, 50, 3.75};
    case MapDifficulty::Hard:
      return {difficulty, 0.75, 75, 2.625};
    case MapDifficulty::VeryHard:
      return {difficulty, 1.0, 100, 1.5};
  }
  throw std::invalid_argument("Unknown map difficulty");
}

template<typename T>
std::string MapGenerator<T>::difficultyName(MapDifficulty difficulty) {
  switch (difficulty) {
    case MapDifficulty::VeryEasy:
      return "VeryEasy";
    case MapDifficulty::Easy:
      return "Easy";
    case MapDifficulty::Medium:
      return "Medium";
    case MapDifficulty::Hard:
      return "Hard";
    case MapDifficulty::VeryHard:
      return "VeryHard";
  }
  throw std::invalid_argument("Unknown map difficulty");
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
std::uint32_t MapGenerator<T>::getSeed() const {
  return _seed;
}

template<typename T>
int MapGenerator<T>::getRequestedObstacleCount() const {
  return _numberObstacles;
}

template<typename T>
T MapGenerator<T>::getMaxObstacleSize() const {
  return _maxObstacleSize;
}

template<typename T>
int MapGenerator<T>::getSampledObstacleCount() const {
  return _sampledObstacleCount;
}

template<typename T>
int MapGenerator<T>::getBorderCrossingCount() const {
  return _borderCrossingCount;
}

template<typename T>
int MapGenerator<T>::getSuccessfulAttempt() const {
  return _successfulAttempt;
}

template<typename T>
int MapGenerator<T>::getStaticRvgResolution() const {
  return _staticRvgResolution;
}

template<typename T>
int MapGenerator<T>::_randomInt(int min, int max) {
  std::uniform_int_distribution<int> dis_int(min, max);
  return dis_int(_rng);
}

template<typename T>
T MapGenerator<T>::_randomReal(T min, T max) {
  std::uniform_real_distribution<T> dis_real(min, max);
  return dis_real(_rng);
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
  if (!incremental) {
    _obstacles.clear();
    _generateBorder();
    _sampledObstacleCount = 0;
    _borderCrossingCount = 0;
  }

  int generatedObstacles = 0;
  int placementAttempts = 0;
  while (generatedObstacles < _numberObstacles) {
    if (placementAttempts >= MAX_OBSTACLE_PLACEMENT_ATTEMPTS) {
      std::ostringstream message;
      message << "Unable to place obstacle " << generatedObstacles + 1
              << " of " << _numberObstacles << " for seed " << _seed;
      throw std::runtime_error(message.str());
    }
    ++placementAttempts;
    Polygon<T> obstacle = _generateObstacle();
    if (obstacle.size() < 3 || !obstacle.getPolygon().is_simple()) {
      continue;
    }
    std::vector<T> centroid = _randomVector(
        2,
        static_cast<T>(-_mapSize / 2.0),
        static_cast<T>(_mapSize / 2.0)
    );
    obstacle.moveTo(centroid[0], centroid[1], 0);
    const ObstaclePlacement placement = _integrateObstacle(obstacle);
    if (placement == ObstaclePlacement::Rejected) {
      continue;
    }
    ++generatedObstacles;
    placementAttempts = 0;
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
typename MapGenerator<T>::ObstaclePlacement MapGenerator<T>::_integrateObstacle(
    const Polygon<T> &obstacle
) {
  if (obstacle.size() < 3 || !obstacle.getPolygon().is_simple()) {
    return ObstaclePlacement::Rejected;
  }

  Polygon_set_2 currentFreeSpace(_border.getCounterClockWise());
  Polygon_set_2 obstacleSet(obstacle.getCounterClockWise());
  Polygon_set_2 insidePart = obstacleSet;
  insidePart.intersection(currentFreeSpace);
  if (insidePart.is_empty()) {
    return ObstaclePlacement::Rejected;
  }

  Polygon_set_2 outsidePart = obstacleSet;
  outsidePart.difference(currentFreeSpace);
  if (outsidePart.is_empty()) {
    _obstacles.push_back(obstacle);
    ++_sampledObstacleCount;
    return ObstaclePlacement::Interior;
  }

  Polygon_set_2 remainingFreeSpace = currentFreeSpace;
  remainingFreeSpace.difference(obstacleSet);
  std::list<Polygon_with_holes_2> components;
  remainingFreeSpace.polygons_with_holes(std::back_inserter(components));
  if (components.size() != 1 || components.front().has_holes()) {
    return ObstaclePlacement::Rejected;
  }

  Polygon_2 boundary = components.front().outer_boundary();
  if (boundary.is_clockwise_oriented()) {
    boundary.reverse_orientation();
  }
  if (boundary.size() < 3 || !boundary.is_simple() || boundary.area() == 0) {
    return ObstaclePlacement::Rejected;
  }

  Polygon<T> candidateBorder(boundary);
  if (_protectStartAndGoal &&
      (!_hasBorderClearance(_start, candidateBorder) ||
       !_hasBorderClearance(_goal, candidateBorder))) {
    return ObstaclePlacement::Rejected;
  }
  if (!std::all_of(
          _obstacles.begin(),
          _obstacles.end(),
          [&](const Polygon<T> &existingObstacle) {
            return _polygonIsInsideBorder(existingObstacle, candidateBorder);
          }
      )) {
    return ObstaclePlacement::Rejected;
  }

  _border = candidateBorder;
  ++_sampledObstacleCount;
  ++_borderCrossingCount;
  return ObstaclePlacement::Border;
}

template<typename T>
bool MapGenerator<T>::_polygonIsInsideBorder(
    const Polygon<T> &polygon,
    const Polygon<T> &border
) const {
  if (polygon.size() < 3 || border.size() < 3) {
    return false;
  }
  Polygon_set_2 outside(polygon.getCounterClockWise());
  outside.difference(Polygon_set_2(border.getCounterClockWise()));
  return outside.is_empty();
}

template<typename T>
bool MapGenerator<T>::_hasBorderClearance(
    const Vertex<T> &configuration,
    const Polygon<T> &border
) const {
  const auto &point = configuration.getPoint();
  const auto &polygon = border.getPolygon();
  if (CGAL::bounded_side_2(
          polygon.vertices_begin(),
          polygon.vertices_end(),
          point,
          K()
      ) != CGAL::ON_BOUNDED_SIDE) {
    return false;
  }
  return std::all_of(
      polygon.edges_begin(),
      polygon.edges_end(),
      [&](const auto &edge) {
        return CGAL::squared_distance(edge, point) >=
            _robotRadius * _robotRadius;
      }
  );
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
  return !_polygonIsInsideBorder(obs, _border);
}

template<typename T>
Vertex<T> MapGenerator<T>::_getLegalConfiguration() {
  for (int attempt = 0; attempt < MAX_OBSTACLE_PLACEMENT_ATTEMPTS; ++attempt) {
    Vertex<T> vertex(
        _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
        _randomReal(-_mapSize / 2. + _marginSize, _mapSize / 2. - _marginSize),
        0,
        2 * PI,
        _randomReal(0, 2 * PI)
    );
    if (_isLegalConfiguration(vertex)) {
      return vertex;
    }
  }
  throw std::runtime_error(
      "Unable to sample a legal configuration for seed " +
      std::to_string(_seed)
  );
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
  return _hasBorderClearance(v, _border) &&
      (!IN_ANY_POLYGON(p, map)) &&
      !std::any_of(
          _obstacles.begin(),
          _obstacles.end(),
          [&](const Polygon<T> &obs) {
            return _circleIntersectsObstacle(obs, v, _robotRadius);
          }
      );
}

template<typename T>
void MapGenerator<T>::generateMap() {
  reset();
  _generateObstacles(false);
  generateStartAndGoal();
  _successfulAttempt = 1;
}

template<typename T>
void MapGenerator<T>::reset() {
  _obstacles.clear();
  _start = _goal = Vertex<T>();
  _protectStartAndGoal = false;
  _sampledObstacleCount = 0;
  _borderCrossingCount = 0;
  _successfulAttempt = 0;
  _staticRvgResolution = 0;
  _generateBorder();
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
  tinyxml2::XMLElement *useBoundary = doc.NewElement("useBoundary");
  useBoundary->SetText(1);
  environment->InsertEndChild(useBoundary);
  tinyxml2::XMLElement *boundary = doc.NewElement("boundary");
  environment->InsertEndChild(boundary);
  Utils::putPolygon<T>(doc, *boundary, _border);
  tinyxml2::XMLElement *obstacles = doc.NewElement("obstacles");
  environment->InsertEndChild(obstacles);
  for (const auto &obs : _obstacles) {
    Utils::putPolygon<T>(doc, *obstacles, obs);
  }
  tinyxml2::XMLElement *mapSize = doc.NewElement("mapSize");
  mapSize->SetText(_mapSize);
  environment->InsertEndChild(mapSize);
  if (_benchmarkMode) {
    const MapDifficultyProfile profile = getDifficultyProfile(_difficulty);
    tinyxml2::XMLElement *metadata = doc.NewElement("benchmarkMetadata");
    metadata->SetAttribute("difficulty", difficultyName(_difficulty).c_str());
    metadata->SetAttribute("normalizedDifficulty", profile.normalizedDifficulty);
    metadata->SetAttribute("seed", static_cast<unsigned>(_seed));
    metadata->SetAttribute("successfulAttempt", _successfulAttempt);
    metadata->SetAttribute("staticRvgResolution", _staticRvgResolution);
    metadata->SetAttribute("sampledObstacleCount", _sampledObstacleCount);
    metadata->SetAttribute("interiorObstacleCount", static_cast<int>(_obstacles.size()));
    metadata->SetAttribute("borderCrossingCount", _borderCrossingCount);
    environment->InsertEndChild(metadata);
  }
  tinyxml2::XMLElement *plannerSettings = doc.NewElement("plannerSettings");
  root->InsertEndChild(plannerSettings);
  tinyxml2::XMLElement *start = doc.NewElement("start");
  plannerSettings->InsertFirstChild(start);
  Utils::putVertex<T>(doc, *start, _start);
  tinyxml2::XMLElement *goal = doc.NewElement("goal");
  plannerSettings->InsertEndChild(goal);
  Utils::putVertex<T>(doc, *goal, _goal);
  if (doc.SaveFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Unable to write map file: " + filename);
  }
}

template<typename T>
void MapGenerator<T>::generateFixedStartAndGoalMap(const Vertex<T> &start, const Vertex<T> &goal) {
  reset();
  _start = start;
  _goal = goal;
  _protectStartAndGoal = true;
  for (int attempt = 1; attempt <= MAX_MAP_GENERATION_ATTEMPTS; ++attempt) {
    _generateObstacles(false);
    if (_isLegalConfiguration(_start) &&
        _isLegalConfiguration(_goal) &&
        !_isStartAndGoalVisible()) {
      _successfulAttempt = attempt;
      return;
    }
  }
  throw std::runtime_error(
      "Unable to generate fixed-start map for seed " + std::to_string(_seed)
  );
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
void MapGenerator<T>::generateSolvableFixedStartAndGoalMap(
    const Vertex<T> &start,
    const Vertex<T> &goal,
    int resolution,
    int numThreads,
    bool protectEndpointBorderClearance
) {
  reset();
  _start = start;
  _goal = goal;
  _protectStartAndGoal = protectEndpointBorderClearance;
  for (int attempt = 1; attempt <= MAX_MAP_GENERATION_ATTEMPTS; ++attempt) {
    _generateObstacles(false);
    if (_isLegalConfiguration(_start) &&
        _isLegalConfiguration(_goal) &&
        hasStaticRvgSolution(resolution, numThreads)) {
      _successfulAttempt = attempt;
      _staticRvgResolution = resolution;
      return;
    }
  }
  std::ostringstream message;
  message << "Unable to generate a static-RVG-solvable "
          << (_benchmarkMode ? difficultyName(_difficulty) : "custom")
          << " map after " << MAX_MAP_GENERATION_ATTEMPTS
          << " attempts for seed " << _seed;
  throw std::runtime_error(message.str());
}

template<typename T>
void MapGenerator<T>::generateSolvableFixedStartAndGoalMap(
    int resolution,
    int numThreads,
    bool protectEndpointBorderClearance
) {
  Vertex<T> start(
      -_mapSize / 2. + _marginSize,
      _mapSize / 2. - _marginSize,
      0,
      0,
      0
  );
  Vertex<T> goal(
      _mapSize / 2. - _marginSize,
      -_mapSize / 2. + _marginSize,
      0,
      0,
      0
  );
  Utils::print("start=", start);
  Utils::print("goal=", goal);
  generateSolvableFixedStartAndGoalMap(
      start,
      goal,
      resolution,
      numThreads,
      protectEndpointBorderClearance
  );
}

template<typename T>
bool MapGenerator<T>::hasStaticRvgSolution(int resolution, int numThreads) const {
  VisibilityGraph<T> planner(
      _robot,
      _border,
      _obstacles,
      resolution,
      numThreads
  );
  planner.getGraph().setWeight(static_cast<T>(1.0), static_cast<T>(0.1));
  std::shared_ptr<Vertex<T>> start =
      std::make_shared<Vertex<T>>(_start);
  std::shared_ptr<Vertex<T>> goal =
      std::make_shared<Vertex<T>>(_goal);
  if (!planner.addVertex(start) || !planner.addVertex(goal)) {
    return false;
  }
  return !planner.getGraph().shortestPath(start, goal, false).empty();
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
