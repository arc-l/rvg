#include "Layer.h"
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Gps_traits_2.h>
#include <CGAL/offset_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <VisibilityGraph/Utils.h>
#include <unordered_map>
#include <climits>
#include <unistd.h>

namespace RotationalVisibilityGraph {
template<typename T>
Layer<T>::Layer() = default;

template<typename T>
Layer<T>::Layer(T theta_lb,
                T theta_ub,
                T roundUpTheta,
                bool fineApprox,
                bool hashWithTheta,
                bool simplifiedGeometry
)
    : _theta_lb(theta_lb),
      _theta_ub(theta_ub),
      _roundUpTheta(roundUpTheta),
      _fineApprox(fineApprox),
      _hashWithTheta(hashWithTheta),
      _simplifiedGeometry(simplifiedGeometry) {
        _infeasible = false;
      }

template<typename T>
Layer<T>::Layer(const Layer<T> &layer)
    : _fineApprox(layer._fineApprox),
      _simplifiedGeometry(layer._simplifiedGeometry),
      _hashWithTheta(layer._hashWithTheta),
      _theta_lb(layer._theta_lb),
      _theta_ub(layer._theta_ub),
      _roundUpTheta(layer._roundUpTheta) {
  _robotBBox = layer._robotBBox;
  _robotBBoxInverted = layer._robotBBoxInverted;
  _shrunkBorder = layer._shrunkBorder;
  _segments = layer._segments;
  _env = layer._env;
  _holeEnv = layer._holeEnv;
  _grownObs = layer._grownObs;
  _holes = layer._holes;
  _isReflexCache = layer._isReflexCache;
  _vertices = layer._vertices;
  _points = layer._points;
  _infeasible = layer._infeasible;
  _visiblePolygonCache = layer._visiblePolygonCache;
}

template<typename T>
void Layer<T>::buildVisibilityGraph(const Polygon<T> &robot,
                                    const Polygon<T> &border,
                                    const std::vector<Polygon<T>> &obstacles) {
  _initRobotBBox(robot);
  _shrinkBorder(border);
  _buildMap(obstacles);
  if(_infeasible) return;
  _connectVisibleVertices();
}

template<typename T>
Polygon<T> Layer<T>::_underestimate(const Polygon<T> &_robot, const Vertex<T>& center){
  Polygon<T> robotLb = _robot.rotateCopy(_theta_lb, center);
  Polygon<T> robotUb = _robot.rotateCopy(_theta_ub, center);
  std::vector<Polygon<T>> robots;
  std::vector<Point_2> points;
  robots.push_back(robotLb);
  robots.push_back(robotUb);
  if (_fineApprox) {
    int resolution = int(fabs(_theta_lb - _theta_ub) / (PI / 180.));
    for (int i = 1; i < resolution; i++) {
      T theta = _theta_lb + i * (_theta_ub - _theta_lb) / resolution;
      robots.push_back(_robot.rotateCopy(theta));
    }
  }
  for (const auto &robot : robots) {
    for (const auto &vertex : robot.getVertices())
      points.push_back(Point_2(vertex.getX(), vertex.getY()));
  }
  std::vector<std::size_t> indices(points.size()), out;
  std::iota(indices.begin(), indices.end(), 0);
  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points)));
  std::vector<Vertex<T>> vertices;
  for (std::size_t i : out) {
    vertices.push_back(Vertex<T>(CGAL::to_double(points[i].x()), CGAL::to_double(points[i].y())));
  }
  return Polygon<T>(vertices, false, _robot.getCentroid());
}

template<typename T>
Polygon<T> Layer<T>::_overestimate(const Polygon<T> &_robot, const Vertex<T>& center){
// #define DRAW_OVERESTIMATE
  Polygon<T> robotLb = _robot.rotateCopy(_theta_lb, center);
  Polygon<T> robotUb = _robot.rotateCopy(_theta_ub, center);
  std::vector<Vertex<T>> vertices;
  std::vector<Point_2> points;
  std::vector<Point_2> convexPoints;
  for (size_t i = 0; i < (size_t) robotLb.size(); i++)
  {
    const Vertex<T> & v1 = robotUb[i];
    const Vertex<T> & v2 = robotLb[(i + 1) % robotLb.size()];
    const Vertex<T> & v3 = robotUb[(i + 1) % robotLb.size()];
    const Vertex<T> & v4 = robotLb[(i + 2) % robotLb.size()];
    std::optional<Vertex<T>> intersection = _lineIntersection(v1, v2, v3, v4);
    points.push_back(Point_2(robotLb[i].getX(), robotLb[i].getY()));
    points.push_back(Point_2(robotUb[i].getX(), robotUb[i].getY()));
    if(intersection.has_value()){
      points.push_back(Point_2(intersection->getX(), intersection->getY()));
      // vertices.push_back(*intersection);
    }
  }
  std::vector<std::size_t> indices(points.size()), out;
  std::iota(indices.begin(), indices.end(), 0);
  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points)));
  for (std::size_t i : out) {
    // vertices.push_back(Vertex<T>(CGAL::to_double(points[i].x()), CGAL::to_double(points[i].y())));
    convexPoints.push_back(points[i]);
  }

  // remove collinear points
  for (size_t i = 0; i < convexPoints.size(); i++) {
    const Point_2 &v1 = convexPoints[i];
    const Point_2 &v2 = convexPoints[(i + 1) % convexPoints.size()];
    const Point_2 &v3 = convexPoints[(i + 2) % convexPoints.size()];
    Vertex<T> a(CGAL::to_double(v1.x()), CGAL::to_double(v1.y()));
    Vertex<T> b(CGAL::to_double(v2.x()), CGAL::to_double(v2.y()));
    Vertex<T> c(CGAL::to_double(v3.x()), CGAL::to_double(v3.y()));
    T cosTheta = (a.getX()-b.getX()) * (c.getX()-b.getX()) + (a.getY()-b.getY()) * (c.getY()-b.getY());
    cosTheta = cosTheta / (sqrt(pow(a.getX()-b.getX(), 2) + pow(a.getY()-b.getY(), 2)) * sqrt(pow(c.getX()-b.getX(), 2) + pow(c.getY()-b.getY(), 2)));

    if (std::abs(std::abs(cosTheta) - 1) < RVG_EPS) {
      convexPoints.erase(convexPoints.begin() + (i + 1) % convexPoints.size());
      i--;
    }
  }

  for (const auto &point : convexPoints) {
    vertices.push_back(Vertex<T>(CGAL::to_double(point.x()), CGAL::to_double(point.y())));
  }


#ifdef DRAW_OVERESTIMATE 
    std::string pythonScript;
    PYTHON_IMPORTS(pythonScript)
    pythonScript += robotLb.draw("start");
    pythonScript += robotUb.draw("goal");
    Polygon<T> res = Polygon<T>(vertices, false, _robot.getCentroid());
    pythonScript += res.draw("obs");
    pythonScript += "plt.axis('equal')\n";
    pythonScript += "plt.show()\n";
    Utils::writeStringToFile<T>(pythonScript, "overestimate.py");
    Utils::runPythonScriptAndRemove<T>("overestimate.py");
#endif
  return Polygon<T>(vertices, false, _robot.getCentroid());

}

template<typename T>
Polygon<T> Layer<T>::_underestimate(const Polygon<T> & _robot){
  return _underestimate(_robot, _robot.getCentroid());
}

template<typename T>
Polygon<T> Layer<T>::_overestimate(const Polygon<T> & _robot){
  return _overestimate(_robot, _robot.getCentroid());
}

template<typename T>
void Layer<T>::_underestimateRobotBBox(const Polygon<T> & _robot){
  if (_robot.isConvex()) {
    _robotBBox = _underestimate(_robot);
    _robotBBoxInverted = _robotBBox;
    _robotBBoxInverted.moveTo(0, 0, 0);
    _robotBBoxInverted.invert();
  } else {
    std::vector<Polygon<T>> robotConvexComponents = _robot.convexDecomposition();
    std::vector<Polygon_2> robotConvexComponentsJoined;
    for (const auto &robotConvexComponent : robotConvexComponents) {
      Polygon_2 convexHull = _underestimate(robotConvexComponent, _robot.getCentroid()).getCounterClockWise();
      if (convexHull.is_clockwise_oriented()) {
        convexHull.reverse_orientation();
      }
      robotConvexComponentsJoined.push_back(convexHull);
    }
    std::vector<Polygon_with_holes_2> joinedRobotBBox;
    CGAL::join(robotConvexComponentsJoined.begin(),
               robotConvexComponentsJoined.end(),
               std::back_inserter(joinedRobotBBox));
    ASSERT_MSG(joinedRobotBBox.size() == 1, "The joined robot bbox should have only one component")
    _robotBBox = Polygon<T>(joinedRobotBBox[0].outer_boundary(), _robot.getCentroid());
    _robotBBoxInverted = _robotBBox;
    _robotBBoxInverted.moveTo(0, 0, 0);
    _robotBBoxInverted.invert();
  }
}

template<typename T>
void Layer<T>::_overestimateRobotBBox(const Polygon<T> & _robot){
  if (_robot.isConvex()) {
    _robotBBox = _overestimate(_robot);
    _robotBBoxInverted = _robotBBox;
    _robotBBoxInverted.moveTo(0, 0, 0);
    _robotBBoxInverted.invert();
  } else {
    std::vector<Polygon<T>> robotConvexComponents = _robot.convexDecomposition();
    std::vector<Polygon_2> robotConvexComponentsJoined;
    for (const auto &robotConvexComponent : robotConvexComponents) {
      Polygon_2 convexHull = _overestimate(robotConvexComponent, _robot.getCentroid()).getCounterClockWise();
      if (convexHull.is_clockwise_oriented()) {
        convexHull.reverse_orientation();
      }
      robotConvexComponentsJoined.push_back(convexHull);
    }
    std::vector<Polygon_with_holes_2> joinedRobotBBox;
    CGAL::join(robotConvexComponentsJoined.begin(),
               robotConvexComponentsJoined.end(),
               std::back_inserter(joinedRobotBBox));
    ASSERT_MSG(joinedRobotBBox.size() == 1, "The joined robot bbox should have only one component")
    _robotBBox = Polygon<T>(joinedRobotBBox[0].outer_boundary(), _robot.getCentroid());
    _robotBBoxInverted = _robotBBox;
    _robotBBoxInverted.moveTo(0, 0, 0);
    _robotBBoxInverted.invert();
  }
}

template<typename T>
void Layer<T>::_initRobotBBox(const Polygon<T> &_robot) {
// #define UNDERESTIMATE
#ifdef UNDERESTIMATE
  _underestimateRobotBBox(_robot);
#else
  _overestimateRobotBBox(_robot);
#endif

#ifdef UNDERESTIMATE
  #undef UNDERESTIMATE
#endif
}

template<typename T>
void Layer<T>::_shrinkBorder(const Polygon<T> &border) {
// #define DEBUG
  Polygon_2 borderCGAL = border.getCounterClockWise();
  CGAL::complement(borderCGAL, _complementBorder);
  _complementBorder = CGAL::minkowski_sum_2(_complementBorder, _robotBBoxInverted.getPolygon());
  ASSERT_MSG(_complementBorder.number_of_holes() == 1, "The border should have only one hole")
  Polygon_2 hole = *_complementBorder.holes_begin();
  _shrunkBorder = Polygon<T>(hole);
#ifdef DEBUG
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript+= border.draw("obs");
  pythonScript+=_shrunkBorder.draw("start");
  pythonScript+=_robotBBoxInverted.draw("goal");
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  std::string pythonSavePath = "shrinkedBorder.py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  int status = system(command.c_str());
  if(status != 0) throw std::runtime_error("Failed to run python script");
  status = system("rm shrinkedBorder.py");
  if(status != 0) throw std::runtime_error("Failed to remove the script");
#endif
}

template<typename T>
void Layer<T>::_buildMap(const std::vector<Polygon<T>> &obstacles) {
  std::vector<Polygon_2> grownObsBeforeMerge;
  std::vector<Polygon_with_holes_2> mergedObs;
  for (const Polygon<T> &obstacle : obstacles) {
    Polygon_with_holes_2 grownObs = CGAL::minkowski_sum_2(obstacle.getPolygon(), _robotBBoxInverted.getPolygon());
    // ASSERT_MSG(grownObs.number_of_holes() == 0, "The grown obstacle should have no hole")
    grownObsBeforeMerge.push_back(grownObs.outer_boundary());
  }

  CGAL::join(grownObsBeforeMerge.begin(), grownObsBeforeMerge.end(), std::back_inserter(mergedObs));
  for (const Polygon_with_holes_2 &mergedObstacle : mergedObs) {
    if (mergedObstacle.has_holes()) {
      for (const Polygon_2 &hole : mergedObstacle.holes()) {
        Polygon_2 newHole = hole;
        if (newHole.is_clockwise_oriented())
          newHole.reverse_orientation();
        _holes.push_back(newHole);
        for (const auto &vertex : hole.vertices()) {
          _holePoints.push_back(vertex);
        }
      }
    }
    Polygon_2 tmpPolygon = mergedObstacle.outer_boundary();
    if (tmpPolygon.is_empty()) continue;
    Polygon_2 cleanedPolygon;
    Polygon_2::Vertex_circulator first, curr, prev;
    first = curr = prev = tmpPolygon.vertices_circulator();
    do {
      if (curr == prev) {
        cleanedPolygon.push_back(*curr);
        curr++;
        continue;
      }
      if (CGAL::squared_distance(*curr, *prev) < RVG_EPS) {
        curr = tmpPolygon.erase(curr);
      } else {
        cleanedPolygon.push_back(*curr);
        curr++;
        prev++;
      }
    } while (curr != first);
    if (cleanedPolygon.is_clockwise_oriented())
      cleanedPolygon.reverse_orientation();
    _grownObs.push_back(cleanedPolygon);
  }

  auto iter = _grownObs.begin();
  while (iter != _grownObs.end()) {
    if (CGAL::do_intersect(*iter, _complementBorder)) {
      CGAL::join(*iter, _complementBorder, _complementBorder);
      iter = _grownObs.erase(iter);
    } else {
      iter++;
    }
  }

  _borderIsHole = _grownObs.empty();

// #define PLOT
  size_t maxHoleId = 0;
  T maxSpan = 0;
  if(_complementBorder.number_of_holes() == 0){
#ifdef PLOT
    std::string pythonScript;
    PYTHON_IMPORTS(pythonScript)
    pythonScript += _shrunkBorder.draw("obs");
    for(const auto &obs : obstacles){
      pythonScript += obs.draw("obs");
    }
    pythonScript += "plt.axis('equal')\n";
    pythonScript += "plt.show()\n";
    std::string pythonSavePath = "layer.py";
    std::ofstream pythonFile(pythonSavePath);
    pythonFile << pythonScript;
    pythonFile.close();
#ifdef PYTHON_EXECUTABLE
    std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
    std::string command = "python3 " + pythonSavePath;
#endif
    int status = system(command.c_str());
    if(status != 0) throw std::runtime_error("Failed to run python script");
    status = system("rm layer.py");
    if(status != 0) throw std::runtime_error("Failed to remove the script");
#endif

    _infeasible = true;
    return;
  }
  std::deque<Polygon_2> holes = _complementBorder.holes();
  for (size_t holeId = 0; holeId < holes.size(); holeId++) {
    Polygon_2 newHole = holes[holeId];
    if (newHole.is_clockwise_oriented())
      newHole.reverse_orientation();
    bool holeIsBorder = false;
    // check if a hole contains any grown obstacles, if so, then this hole is the new border.
    if (!_borderIsHole) {
      bool intersection = false;
      for (const auto &obs : _grownObs) {
        if (!CGAL::do_intersect(newHole, obs)) {
          intersection = true;
          break;
        }
      }
      if (!intersection) {
        _shrunkBorder = Polygon<T>(newHole);
        holeIsBorder = true;
      }
    } else {
      Point_2 topVertex = *CGAL::top_vertex_2(newHole.vertices_begin(), newHole.vertices_end());
      Point_2 bottomVertex = *CGAL::bottom_vertex_2(newHole.vertices_begin(), newHole.vertices_end());
      Point_2 leftVertex = *CGAL::left_vertex_2(newHole.vertices_begin(), newHole.vertices_end());
      Point_2 rightVertex = *CGAL::right_vertex_2(newHole.vertices_begin(), newHole.vertices_end());
      T xSpan = CGAL::to_double(rightVertex.x()) - CGAL::to_double(leftVertex.x());
      T ySpan = CGAL::to_double(topVertex.y()) - CGAL::to_double(bottomVertex.y());
      T span = std::sqrt(xSpan * xSpan + ySpan * ySpan);
      if (span > maxSpan) {
        maxSpan = span;
        maxHoleId = holeId;
      }
    }
    if (holeIsBorder) {
      continue;
    }
    if (newHole.is_clockwise_oriented())
      newHole.reverse_orientation();
    _holes.push_back(newHole);
    for (const auto &vertex : newHole.vertices()) {
      _holePoints.push_back(vertex);
    }
#ifdef PLOT
    Polygon<T> holePolygon = Polygon<T>(hole);
    pythonScript += holePolygon.draw("obs");
#endif
  }
#ifdef PLOT
  for(const auto &obs : _grownObs){
    Polygon<T> obsPolygon = Polygon<T>(obs);
    pythonScript += obsPolygon.draw("obs");
  }
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  std::string pythonSavePath = "layer.py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  int status = system(command.c_str());
    if(status != 0) throw std::runtime_error("Failed to run python script");
  status = system("rm layer.py");
    if(status != 0) throw std::runtime_error("Failed to remove the script");
#endif
  if (_borderIsHole) {
    _shrunkBorder = Polygon<T>(holes[maxHoleId]);
  }

  if (!_grownObs.empty()) {
    std::vector<Polygon_2> polygons = _grownObs;
    polygons.push_back(_shrunkBorder.getCounterClockWise());
    for (const Polygon_2 &polygon : polygons) {
      for (const auto &edge : polygon.edges()) {
        if (edge.squared_length() < RVG_EPS) Utils::print("Edge with length 0: ", edge);
        _segments.push_back(edge);
      }
    }
    CGAL::insert_non_intersecting_curves(_env, _segments.begin(), _segments.end());
  }

  // hole env
  std::vector<Segment_2> holeSegments;
  for (const auto &hole : _holes) {
    for (const auto &edge : hole.edges()) {
      holeSegments.push_back(edge);
    }
  }
  CGAL::insert_non_intersecting_curves(_holeEnv, holeSegments.begin(), holeSegments.end());
}

template<typename T>
void Layer<T>::_connectVisibleVertices() {
  // outer obstacles
  _vq = std::make_shared<VQ>(_env);
  PL_2 envPL(_env);
  for (const auto &v : _env.vertex_handles()) {
    Arrangement_2 visibleArea;
    if (_isOnBorder(v->point())) {
      if (!_isReflex(v, false, true)) continue;
      visibleArea = visibleAreaOnHoles<T>(v->point(), _env, _vq);
    } else {
      if (!_isReflex(v, false, false)) continue;
      visibleArea = visibleAreaOnObs<T>(v->point(), _env, _vq);
    }
    std::shared_ptr<Vertex<T>> v1 = std::make_shared<Vertex<T>>(
        CGAL::to_double(v->point().x()),
        CGAL::to_double(v->point().y()),
        _theta_lb,
        _theta_ub,
        (_theta_ub + _theta_lb) / 2.,
        _roundUpTheta,
        _hashWithTheta
    );
    std::pair<Point_2, Point_2> neighbor1 = _getNeighbor(v);

    PL_2 pl(visibleArea);
    //TODO: Debug traversing visibleArea
    //    for (const auto &p : visibleArea.vertex_handles()) {
    //      if (!Utils::isVertexOfArrangement<T>(p->point(), envPL)) continue;
    for (const auto &p : _env.vertex_handles()) {
      if (p->point() == v->point()) continue;
      if (!_isReflex(p, false, _isOnBorder(p->point()))) continue;
      std::pair<Point_2, Point_2> neighbor2 = _getNeighbor(p);
      if (!Utils::isBitangent<T>(v->point(),
                                 neighbor1.first,
                                 neighbor2.second,
                                 p->point(),
                                 neighbor2.first,
                                 neighbor2.second))
        continue;
      CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(p->point());
      if (boost::get<Arrangement_2::Vertex_const_handle>(&obj)) {
        std::shared_ptr<Vertex<T>> v2 = std::make_shared<Vertex<T>>(
            CGAL::to_double(p->point().x()),
            CGAL::to_double(p->point().y()),
            _theta_lb,
            _theta_ub,
            (_theta_ub + _theta_lb) / 2.,
            _roundUpTheta,
            _hashWithTheta
        );
        _neighborCache.insert({*v1, neighbor1});
        _neighborCache.insert({*v2, neighbor2});
        _isVertexOnHoleCache.insert({*v1, false});
        _isVertexOnHoleCache.insert({*v2, false});
        _visiblePolygonCache.insert({*v1, Utils::arrangementToPolygon<T>(visibleArea)});
        if (std::find(_points.begin(), _points.end(), v->point()) == _points.end()) {
          _points.push_back(v->point());
          _vertices.push_back(*v1);
        }
        if (std::find(_points.begin(), _points.end(), p->point()) == _points.end()) {
          _points.push_back(p->point());
          _vertices.push_back(*v2);
        }
        _edges.push_back(std::make_pair(v1, v2));
      }
    }
  }

  // holes
  if (_holePoints.empty()) return;
  _holeVQ = std::make_shared<VQ>(_holeEnv);
  for (const auto &v : _holeEnv.vertex_handles()) {
    bool vOnBorder = _isOnBorder(v->point());
    if (vOnBorder)
      if (!_isReflex(v, true, true)) continue;
    Arrangement_2 visibleArea = visibleAreaOnHoles<T>(v->point(), _holeEnv, _holeVQ);
    std::shared_ptr<Vertex<T>> v1 = std::make_shared<Vertex<T>>(
        CGAL::to_double(v->point().x()),
        CGAL::to_double(v->point().y()),
        _theta_lb,
        _theta_ub,
        (_theta_ub + _theta_lb) / 2.,
        _roundUpTheta,
        _hashWithTheta
    );
    std::pair<Point_2, Point_2> neighbor1 = _getNeighbor(v);

    PL_2 pl(visibleArea);
    for (const auto &p : _holeEnv.vertex_handles()) {
      if (p->point() == v->point()) continue;
      std::pair<Point_2, Point_2> neighbor2 = _getNeighbor(p);
      bool pOnBorder = _isOnBorder(p->point());
      if (pOnBorder) {
        if (!_isReflex(p, true, true)) continue;
        if (vOnBorder && !Utils::isBitangent<T>(v->point(),
                                                neighbor1.first,
                                                neighbor1.second,
                                                p->point(),
                                                neighbor2.first,
                                                neighbor2.second))
          continue;
      }
      CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(p->point());
      if (boost::get<Arrangement_2::Vertex_const_handle>(&obj)) {
      std::shared_ptr<Vertex<T>> v2 = std::make_shared<Vertex<T>>(
            CGAL::to_double(p->point().x()),
            CGAL::to_double(p->point().y()),
            _theta_lb,
            _theta_ub,
            (_theta_ub + _theta_lb) / 2.,
            _roundUpTheta,
            _hashWithTheta
        );
        _visiblePolygonCache.insert({*v1, Utils::arrangementToPolygon<T>(visibleArea)});
        _neighborCache.insert({*v1, neighbor1});
        _neighborCache.insert({*v2, neighbor2});
        _isVertexOnHoleCache.insert({*v1, true});
        _isVertexOnHoleCache.insert({*v2, true});
        if (std::find(_points.begin(), _points.end(), v->point()) == _points.end()) {
          _points.push_back(v->point());
          _vertices.push_back(*v1);
        }
        if (std::find(_points.begin(), _points.end(), p->point()) == _points.end()) {
          _points.push_back(p->point());
          _vertices.push_back(*v2);
        }
        _edges.push_back(std::make_pair(v1, v2));
      }
    }
  }
}

template<typename T>
bool Layer<T>::_isReflex(const Vertex_const_handle &v, bool isHoleVertex, bool isOnBorder) {
  Vertex<T> p(CGAL::to_double(v->point().x()), CGAL::to_double(v->point().y()));
  if (_isReflexCache.find(p) != _isReflexCache.end()) {
    return _isReflexCache.at(p);
  }
  Point_2 vl, vr;
  typename Arrangement_2::Halfedge_around_vertex_const_circulator first, curr;
  first = curr = v->incident_halfedges();
  while (true) {
    if (curr->is_on_outer_ccb())
      vl = curr->source()->point();
    if (curr->is_on_inner_ccb())
      vr = curr->source()->point();
    curr++;
    if (curr == first)
      break;
  }
  bool isReflex;
  if (isHoleVertex || isOnBorder) {
    isReflex = CGAL::right_turn(vl, v->point(), vr);
    _isReflexCache.insert({p, isReflex});
  } else {
    isReflex = CGAL::left_turn(vl, v->point(), vr);
    _isReflexCache.insert({p, isReflex});
  }
  return isReflex;
}

template<typename T>
bool Layer<T>::_isOnBorder(const Point_2 &v) const {
  const auto &mapBorder = _shrunkBorder.getCounterClockWise();
  return CGAL::bounded_side_2(mapBorder.begin(), mapBorder.end(), v, K()) == CGAL::ON_BOUNDARY;
}

template<typename T>
const T &Layer<T>::getThetaLb() const {
  return _theta_lb;
}

template<typename T>
const T &Layer<T>::getThetaUb() const {
  return _theta_ub;
}

template<typename T>
const std::vector<typename Layer<T>::Polygon_2> &Layer<T>::getGrownObs() const {
  return _grownObs;
}

template<typename T>
std::vector<Polygon<T>> Layer<T>::getGrownObsRVGPolygon() const {
  std::vector<Polygon<T>> grownObs;
  for(const auto &obs : _grownObs){
    Polygon<T> polygon = Polygon<T>(obs);
    grownObs.push_back(polygon);
  }
  return grownObs;
}

template<typename T>
const std::vector<typename Layer<T>::Point_2> &Layer<T>::getPoints() const {
  return _points;
}

template<typename T>
const typename Layer<T>::Point_2 &Layer<T>::getPoint(int index) const {
  return _points[index];
}

template<typename T>
const typename Layer<T>::Arrangement_2 &Layer<T>::getEnv() const {
  return _env;
}

template<typename T>
const std::vector<typename Layer<T>::Polygon_2> &Layer<T>::getHoles() const {
  return _holes;
}

template<typename T>
std::vector<Polygon<T>> Layer<T>::getHolesPolygon() const {
  std::vector<Polygon<T>> holes;
  for(const auto &hole : _holes){
    Polygon<T> polygon = Polygon<T>(hole);
    holes.push_back(polygon);
  }
  return holes;
}

template<typename T>
bool Layer<T>::hasHoles() const {
  return !_holes.empty();
}

template<typename T>
void Layer<T>::draw(const std::string &figPath,
                    const Polygon<T> &border,
                    const std::vector<Polygon<T>> &obstacles,
                    Graph<T> *graph,
                    bool show
) const {
  if (figPath.empty())
    return;
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += border.draw("obs");
// obstacles
  for (
    const auto &obs
      : obstacles) {
    pythonScript += obs.draw("obs");
  }
// grown obstacles
  for (
    const auto &obstacle
      : _grownObs) {
    Polygon<T> obs(obstacle);
    pythonScript += obs.draw("start");
  }

// shrinked border
  pythonScript += _shrunkBorder.draw("goal");

// holes
  for (
    const auto &holePolygon
      : _holes) {
    Polygon<T> hole(holePolygon);
    pythonScript += hole.draw("hole");
  }

// graph
  if (graph) {
    pythonScript += graph->
        draw();
  }


  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  if (show) pythonScript += "plt.show()\n";
  std::string pythonSavePath = "layer.py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
#else
  std::string command = "python3 " + pythonSavePath;
#endif
  int status = system(command.c_str());
  if (status != 0) throw std::runtime_error("Failed to run python script");
}

template<typename T>
bool Layer<T>::borderIsHole() const {
  return _borderIsHole;
}

template<typename T>
const Polygon<T> &Layer<T>::getShrinkedBorder() const {
  return _shrunkBorder;
}

template<typename T>
std::shared_ptr<VQ> Layer<T>::getVQ() const {
  return _vq;
}

template<typename T>
std::shared_ptr<VQ> Layer<T>::getHoleVQ() const {
  return _holeVQ;
}

template<typename T>
const typename Layer<T>::Arrangement_2 &Layer<T>::getHoleEnv() const {
  return _holeEnv;
}

template<typename T>
const std::unordered_map<Vertex<T>,
                         typename Layer<T>::Polygon_2,
                         typename Vertex<T>::Hash>
Layer<T>::getVisibleAreaCache() const {
  return _visiblePolygonCache;
}

template<typename T>
std::string Layer<T>::draw(bool curr) const {
  std::string pythonScript;
  if (curr)
    pythonScript += _shrunkBorder.draw("goal", false);
  else
    pythonScript += _shrunkBorder.draw("goal", true);

  for (const auto &obs : _grownObs) {
    Polygon<T> obsPolygon = Polygon<T>(obs);
    if (curr)
      pythonScript += obsPolygon.draw("obs", false);
    else
      pythonScript += obsPolygon.draw("path", true);
  }
  for (const auto &hole : _holes) {
    Polygon<T> holePolygon = Polygon<T>(hole);
    if (curr)
      pythonScript += holePolygon.draw("hole", false);
    else
      pythonScript += holePolygon.draw("start", true);
  }
  return pythonScript;
}

template<typename T>
bool Layer<T>::legalConfig(const Vertex<T> &v) const {
  if (_borderIsHole) {
    const Point_2 &p = v.getPoint();
    if (IN_ANY_POLYGON(p, _holes)) return true;
    if (_shrunkBorder.contains(v)) return true;
    else return false;
  } else {
    const Point_2 &p = v.getPoint();
    if (IN_ANY_POLYGON(p, _holes)) return true;
    if (IN_ANY_POLYGON(p, _grownObs)) return false;
    return true;
  }
}

template<typename T>
std::shared_ptr<typename Layer<T>::Arrangement_2> Layer<T>::getVisibleArea(const Vertex<T> &v) const {
  if(_infeasible) 
    throw std::runtime_error("The layer is infeasible");
  const Point_2 &p = v.getPoint();
  if (!_holes.empty() && IN_ANY_POLYGON(p, _holes)) {
    return std::make_shared<Arrangement_2>(visibleAreaOnHoles<T>(p, _holeEnv, _holeVQ));
  }
  if(IN_POLYGON(p, _shrunkBorder.getPolygon()) && !_grownObs.empty() && !IN_ANY_POLYGON(p, _grownObs)){
    return std::make_shared<Arrangement_2>(visibleAreaInMap<T>(p, _env, _vq));
  }
  else
    return NULL;
}

template<typename T>
std::unordered_map<Vertex<T>, Polygon<T>, typename Vertex<T>::Hash> Layer<T>::getVisibleAreaPolygons() const{
  std::unordered_map<Vertex<T>, Polygon<T>, typename Vertex<T>::Hash> visiblePolygons;
  for( const auto& pairs: _visiblePolygonCache){
    Polygon<T> polygon(pairs.second);
    visiblePolygons.insert({pairs.first, polygon});
  }
  return visiblePolygons;
}

template<typename T>
Polygon<T> Layer<T>::getVisibleAreaPolygon(T x, T y) const{
  Vertex<T> v(x, y, _theta_lb, _theta_ub, (_theta_lb + _theta_ub) / 2., _roundUpTheta, _hashWithTheta);
  Polygon<T> polygon(_visiblePolygonCache.at(v));
  return polygon;
}

template<typename T>
const std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &
Layer<T>::getEdges() const {
  return _edges;
}

template<typename T>
const std::vector<typename Layer<T>::Vertex_const_handle> Layer<T>::getVertexHandles() const {
  return _vertexHandles;
}

template<typename T>
const typename Layer<T>::Vertex_const_handle &Layer<T>::getVertexHandle(int index) const {
  return _vertexHandles[index];
}

template<typename T>
std::pair<typename Layer<T>::Point_2,
          typename Layer<T>::Point_2> Layer<T>::_getNeighbor(const Vertex_const_handle &handle) const {
  std::vector<Point_2> points;
  typename Arrangement_2::Halfedge_around_vertex_const_circulator first, curr;
  first = curr = handle->incident_halfedges();
  do {
    points.push_back(curr->source()->point());
  } while (++curr != first);
  return std::make_pair(points[0], points[1]);
}

template<typename T>
const std::pair<typename Layer<T>::Point_2,
                typename Layer<T>::Point_2> &Layer<T>::getNeighbor(const Vertex<T> &v) const {
  return _neighborCache.at(v);
}

template<typename T>
bool Layer<T>::isVertexOnHole(const Vertex<T> &vertex) const {
  return _isVertexOnHoleCache.at(vertex);
}

template<typename T>
const Vertex<T> &Layer<T>::getVertex(int index) const {
  return _vertices[index];
}

template<typename T>
bool Layer<T>::isFeasible() const {
  return !_infeasible;
}

template<typename T>
std::string Layer<T>::drawGrownObs3D(const std::string&type) const{
  std::string pythonScript;
  for (const auto &obstacle: _grownObs) {
    Polygon<T> obs(obstacle);
    auto vertices = obs.getVertices();
    for(auto & v: vertices){
      v.setBounds(_theta_lb, _theta_ub);
      v.setTheta((_theta_lb + _theta_ub) / 2.);
    }
    obs = Polygon<T>(vertices, false);
    pythonScript += obs.draw3D(type);
  }
  for (const auto &obstacle: _holes) {
    Polygon<T> obs(obstacle);
    auto vertices = obs.getVertices();
    for(auto & v: vertices){
      v.setBounds(_theta_lb, _theta_ub);
      v.setTheta((_theta_lb + _theta_ub) / 2.);
    }
    obs = Polygon<T>(vertices, false);
    pythonScript += obs.draw3D(type);
  }
  return pythonScript;
}

template<typename T>
std::string Layer<T>::drawVisibleAreas()const{
  std::string pythonScript;
  for( const auto& pairs: _visiblePolygonCache){
    Polygon<T> polygon(pairs.second);
    pythonScript += polygon.draw("default");
  }
  return pythonScript;
}

template<typename T>
std::string Layer<T>::drawVisibleAreas(int cnt)const{
  std::string pythonScript;
  int i = 0;
  for( const auto& pairs: _visiblePolygonCache){
    i++;
    Polygon<T> polygon(pairs.second);
    pythonScript += polygon.draw("default");
    if (i == cnt) break;
  }
  return pythonScript;
}

template<typename T>
void Layer<T>::drawRobotBB(const Polygon<T> & robot) const{
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  Polygon<T> robotLb = robot.rotateCopy(_theta_lb, robot.getCentroid());
  Polygon<T> robotUb = robot.rotateCopy(_theta_ub, robot.getCentroid());
  pythonScript += robotLb.draw("start");
  pythonScript += robotUb.draw("goal");
  pythonScript += _robotBBox.draw("obs");
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  Utils::writeStringToFile<T>(pythonScript, "overestimate.py");
  Utils::runPythonScriptAndRemove<T>("overestimate.py");
}

template<typename T>
const Polygon<T> &Layer<T>::getRobotBBox() const {
  return _robotBBox;
}

template<typename T>
Polygon<T> Layer<T>::getRobotBBoxInverted() const {
  Polygon<T> robotBBoxInverted = _robotBBox;
  robotBBoxInverted.moveTo(0, 0, 0);
  robotBBoxInverted.invert();
  return robotBBoxInverted;
}

template<typename T>
std::optional<Vertex<T>> Layer<T>::_lineIntersection(const Vertex<T>& v1, const Vertex<T>& v2, const Vertex<T>& v3, const Vertex<T>& v4){
  // Extract coordinates
  T x1 = v1.getX(), y1 = v1.getY();
  T x2 = v2.getX(), y2 = v2.getY();
  T x3 = v3.getX(), y3 = v3.getY();
  T x4 = v4.getX(), y4 = v4.getY();
  // Compute the determinants
  T denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  // Check for parallel lines
  if (denominator == static_cast<T>(0)) {
      return std::nullopt; // Lines are parallel
  }
  // Calculate intersection point
  T px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
  T py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
  return Vertex<T>(px, py);
}

template
class Layer<double>;
template
class Layer<float>;
} // namespace RotationalVisibilityGraph

