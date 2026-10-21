#include "Layer.h"
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Gps_traits_2.h>
#include <CGAL/offset_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <VisibilityGraph/Utils.h>
#include <Utils/Utils.h>
#include <unordered_map>
#include <climits>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unistd.h>

namespace RotationalVisibilityGraph {

namespace {

template<typename T>
void logInvalidLayerPolygon(const char *label,
                            T thetaLb,
                            T thetaUb,
                            const typename Polygon<T>::Polygon_2 &polygon) {
  std::ostringstream oss;
  oss << "Skipping invalid " << label << " for theta range ["
      << thetaLb << ", " << thetaUb << "] with vertices:";
  for (const auto &vertex : polygon.vertices()) {
    oss << " (" << CGAL::to_double(vertex.x()) << ", "
        << CGAL::to_double(vertex.y()) << ")";
  }
  std::cerr << oss.str() << std::endl;
}

template<typename T>
bool isUsablePolygon(const Polygon<T> &polygon) {
  return polygon.getPolygon().size() >= 3;
}

template<typename PolygonType>
void appendTopologyPolygonPlot(std::string &pythonScript,
                               const PolygonType &polygon,
                               const std::string &faceColor,
                               const std::string &edgeColor,
                               const std::string &label,
                               double alpha,
                               double lineWidth,
                               const std::string &lineStyle,
                               int zOrder,
                               const std::string &annotation) {
  if (polygon.is_empty()) {
    return;
  }
  std::ostringstream xValues;
  std::ostringstream yValues;
  xValues << std::setprecision(17);
  yValues << std::setprecision(17);
  double annotationX = 0;
  double annotationY = 0;
  size_t vertexCount = 0;
  for (const auto &vertex : polygon.vertices()) {
    if (vertexCount > 0) {
      xValues << ',';
      yValues << ',';
    }
    const double x = CGAL::to_double(vertex.x());
    const double y = CGAL::to_double(vertex.y());
    xValues << x;
    yValues << y;
    annotationX += x;
    annotationY += y;
    ++vertexCount;
  }
  if (vertexCount == 0) {
    return;
  }
  annotationX /= static_cast<double>(vertexCount);
  annotationY /= static_cast<double>(vertexCount);
  pythonScript += "x = [" + xValues.str() + "]\n";
  pythonScript += "y = [" + yValues.str() + "]\n";
  pythonScript +=
      "ax.fill(x, y, facecolor='" + faceColor +
      "', edgecolor='" + edgeColor +
      "', alpha=" + std::to_string(alpha) +
      ", linewidth=" + std::to_string(lineWidth) +
      ", linestyle='" + lineStyle +
      "', zorder=" + std::to_string(zOrder) +
      ", label='" + label + "')\n";
  if (!annotation.empty()) {
    pythonScript +=
        "ax.text(" + std::to_string(annotationX) + ", " +
        std::to_string(annotationY) + ", '" + annotation +
        "', color='" + edgeColor +
        "', fontsize=7, fontweight='bold', ha='center', va='center', zorder=" +
        std::to_string(zOrder + 1) + ")\n";
  }
}

template<typename T, typename PolygonType>
void drawHoleObstacleClassification(
    const std::deque<PolygonType> &holes,
    const std::vector<PolygonType> &grownObstacles,
    T thetaLb,
    T thetaUb
) {
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += "fig.set_size_inches(12, 10)\n";

  bool labeledAll = false;
  bool labeledSome = false;
  bool labeledNone = false;
  for (size_t holeId = 0; holeId < holes.size(); ++holeId) {
    PolygonType hole = holes[holeId];
    if (hole.is_clockwise_oriented()) {
      hole.reverse_orientation();
    }
    std::vector<size_t> intersectingObstacles;
    for (size_t obstacleId = 0; obstacleId < grownObstacles.size();
         ++obstacleId) {
      if (CGAL::do_intersect(hole, grownObstacles[obstacleId])) {
        intersectingObstacles.push_back(obstacleId);
      }
    }

    const bool intersectsAll = !grownObstacles.empty() &&
        intersectingObstacles.size() == grownObstacles.size();
    const bool intersectsSome = !intersectingObstacles.empty() &&
        !intersectsAll;
    std::string faceColor;
    std::string edgeColor;
    std::string label = "_nolegend_";
    if (intersectsAll) {
      faceColor = "#BBF7D0";
      edgeColor = "#15803D";
      if (!labeledAll) {
        label = "Environment border (intersects all grown obstacles)";
        labeledAll = true;
      }
    } else if (intersectsSome) {
      faceColor = "#FED7AA";
      edgeColor = "#EA580C";
      if (!labeledSome) {
        label = "Environment border (intersects some grown obstacles)";
        labeledSome = true;
      }
    } else {
      faceColor = "#DDD6FE";
      edgeColor = "#7C3AED";
      if (!labeledNone) {
        label = "Obstacle-free hole (intersects no grown obstacles)";
        labeledNone = true;
      }
    }

    std::string annotation = "H" + std::to_string(holeId) + " [";
    if (intersectingObstacles.empty()) {
      annotation += "none";
    } else {
      for (size_t index = 0; index < intersectingObstacles.size(); ++index) {
        if (index > 0) annotation += ",";
        annotation += "G" + std::to_string(intersectingObstacles[index]);
      }
    }
    annotation += "]";
    appendTopologyPolygonPlot(
        pythonScript,
        hole,
        faceColor,
        edgeColor,
        label,
        0.38,
        2.2,
        "-",
        10,
        annotation
    );
  }

  for (size_t obstacleId = 0; obstacleId < grownObstacles.size();
       ++obstacleId) {
    appendTopologyPolygonPlot(
        pythonScript,
        grownObstacles[obstacleId],
        "#FCA5A5",
        "#B91C1C",
        obstacleId == 0 ? "Remaining grown obstacles" : "_nolegend_",
        0.70,
        1.8,
        "-",
        20,
        "G" + std::to_string(obstacleId)
    );
  }

  std::ostringstream title;
  title << std::fixed << std::setprecision(1)
        << "Pre-classification topology [" << thetaLb * 180.0 / M_PI
        << " deg, " << thetaUb * 180.0 / M_PI << " deg]"
        << " | complement holes=" << holes.size()
        << " | remaining grown obstacles=" << grownObstacles.size();
  pythonScript += "ax.set_title('" + title.str() + "')\n";
  pythonScript += "ax.set_xlabel('x')\n";
  pythonScript += "ax.set_ylabel('y')\n";
  pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
  pythonScript +=
      "ax.grid(True, color='#CBD5E1', linewidth=0.6, alpha=0.7)\n";
  pythonScript += "handles, labels = ax.get_legend_handles_labels()\n";
  pythonScript += "legend = dict(zip(labels, handles))\n";
  pythonScript +=
      "ax.legend(legend.values(), legend.keys(), loc='best', fontsize=8, "
      "framealpha=0.92)\n";
  pythonScript += "fig.tight_layout()\n";

  const long long thetaLbMilliDegrees = std::llround(
      thetaLb * 180.0 / M_PI * 1000.0
  );
  const long long thetaUbMilliDegrees = std::llround(
      thetaUb * 180.0 / M_PI * 1000.0
  );
  const std::string figPath =
      "Results/layer_hole_obstacle_debug_" +
      std::to_string(thetaLbMilliDegrees) + "_" +
      std::to_string(thetaUbMilliDegrees) + ".png";
  pythonScript +=
      "fig.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  const std::string pythonSavePath = figPath + ".py";
  if (!Utils::writeStringToFile<T>(pythonScript, pythonSavePath)) {
    throw std::runtime_error("Failed to write topology debug script");
  }
  if (!Utils::runPythonScriptAndRemove<T>(pythonSavePath)) {
    throw std::runtime_error("Failed to run and remove topology debug script");
  }
}

}

template<typename T>
Layer<T>::Layer() = default;

template<typename T>
Layer<T>::Layer(T theta_lb,
                T theta_ub,
                bool simplifiedGeometry
)
    : _theta_lb(theta_lb),
      _theta_ub(theta_ub),
      _simplifiedGeometry(simplifiedGeometry) {
        _infeasible = false;
      }

template<typename T>
Layer<T>::Layer(const Layer<T> &layer)
    : _simplifiedGeometry(layer._simplifiedGeometry),
      _hasHoles(layer._hasHoles),
      _borderIsHole(layer._borderIsHole),
      _bordersAreHoles(layer._bordersAreHoles),
      _infeasible(layer._infeasible),
      _theta_lb(layer._theta_lb),
      _theta_ub(layer._theta_ub),
      _vq(layer._vq),
      _holeVQ(layer._holeVQ),
      _vertices(layer._vertices),
      _points(layer._points),
      _holePoints(layer._holePoints),
      _vertexHandles(layer._vertexHandles),
      _grownObs(layer._grownObs),
      _holes(layer._holes),
      _borderHoles(layer._borderHoles),
      _edges(layer._edges),
      _shrunkBorder(layer._shrunkBorder),
      _robotBBox(layer._robotBBox),
      _robotBBoxInverted(layer._robotBBoxInverted),
      _shrunkBorders(layer._shrunkBorders),
      _shrunkBorderExactList(layer._shrunkBorderExactList),
      _segments(layer._segments),
      _complementBorder(layer._complementBorder),
      _env(layer._env),
      _holeEnv(layer._holeEnv),
      _isReflexCache(layer._isReflexCache),
      _isVertexOnHoleCache(layer._isVertexOnHoleCache),
      _visiblePolygonCache(layer._visiblePolygonCache),
      _neighborCache(layer._neighborCache)
   {
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
  return Polygon<T>(vertices, _robot.getCentroid(), false);
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
  return Polygon<T>(vertices, _robot.getCentroid(), false);

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
#ifdef DEBUG
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript+= border.draw("obs");
  for(const auto &hole : _complementBorder.holes()){
    Polygon<T> holePolygon = Polygon<T>(hole);
    pythonScript+= holePolygon.draw("hole");
  }
  pythonScript+=_robotBBoxInverted.draw("goal");
  pythonScript += "plt.axis('equal')\n";
  // pythonScript += "plt.show()\n";
  pythonScript += "plt.savefig(";
  pythonScript += "'layer_";
  pythonScript += std::to_string(_theta_lb);
  pythonScript += ".png', dpi=300)";
  std::string pythonSavePath = "shrinkedBorder.py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
  RotationalVisibilityGraph::Utils::runPythonScriptAndRemove<T>(pythonSavePath);
#endif
  // ASSERT_MSG(_complementBorder.number_of_holes() == 1, "The border should have only one hole")
  for (const auto &hole: _complementBorder.holes()) {
    _borderHoles.push_back(hole);
    Polygon<T> shrunkBorder(hole);
    if (!isUsablePolygon(shrunkBorder)) {
      logInvalidLayerPolygon<T>("shrunk border hole", _theta_lb, _theta_ub, hole);
      continue;
    }
    _shrunkBorders.push_back(shrunkBorder);
  }
  //Utils::print("#shrunk borders for layer [", _theta_lb, ", ", _theta_ub, "]: ", _shrunkBorders.size());
  //Utils::print("#_borderHoles for layer [", _theta_lb, ", ", _theta_ub, "]: ", _borderHoles.size());
  if (_complementBorder.number_of_holes() == 0 || _shrunkBorders.empty()) {
    _infeasible = true;
    return;
  }
  _shrunkBorder = _shrunkBorders.front();
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
        if (hole.size() < 3) {
          logInvalidLayerPolygon<T>("merged obstacle hole", _theta_lb, _theta_ub, hole);
          continue;
        }
        Polygon_2 sanitizedHole = hole;
        if (sanitizedHole.is_clockwise_oriented()) {
          sanitizedHole.reverse_orientation();
        }
        _holes.push_back(sanitizedHole);
        for (const auto &vertex : sanitizedHole.vertices()) {
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
    Polygon<T> grownObstacle(cleanedPolygon);
    if (!isUsablePolygon(grownObstacle)) {
      logInvalidLayerPolygon<T>("grown obstacle", _theta_lb, _theta_ub, cleanedPolygon);
      continue;
    }
    _grownObs.push_back(grownObstacle.getCounterClockWise());
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
    _infeasible = true;
    return;
  }
  std::deque<Polygon_2> holes = _complementBorder.holes();
  _shrunkBorderExactList.clear();
#ifdef PLOT
  drawHoleObstacleClassification<T>(holes, _grownObs, _theta_lb, _theta_ub);
#endif
#undef PLOT
  for (size_t holeId = 0; holeId < holes.size(); holeId++) {
    Polygon_2 newHole = holes[holeId];
    if (newHole.is_clockwise_oriented())
      newHole.reverse_orientation();
    bool holeIsBorder = false;
    // A complement hole containing a remaining grown obstacle is an outer
    // boundary of an environment component, not an obstacle-free hole.
    if (!_borderIsHole) {
      bool containsGrownObstacle = false;
      for (const auto &obs : _grownObs) {
        if (CGAL::do_intersect(newHole, obs)) {
          containsGrownObstacle = true;
          break;
        }
      }
      if (containsGrownObstacle) {
        Polygon<T> shrunkBorder(newHole);
        if (!isUsablePolygon(shrunkBorder)) {
          logInvalidLayerPolygon<T>("selected shrunk border", _theta_lb, _theta_ub, newHole);
          continue;
        }
        if (_shrunkBorderExactList.empty()) {
          _shrunkBorder = shrunkBorder;
        }
        _shrunkBorderExactList.push_back(newHole);
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
    if (newHole.size() < 3) {
      logInvalidLayerPolygon<T>("map hole", _theta_lb, _theta_ub, newHole);
      continue;
    }
    Polygon_2 sanitizedHole = newHole;
    if (sanitizedHole.is_clockwise_oriented()) {
      sanitizedHole.reverse_orientation();
    }
    _holes.push_back(sanitizedHole);
    for (const auto &vertex : sanitizedHole.vertices()) {
      _holePoints.push_back(vertex);
    }
  }
  if (_borderIsHole) {
    Polygon_2 largestHole = holes[maxHoleId];
    if (largestHole.is_clockwise_oriented()) {
      largestHole.reverse_orientation();
    }
    Polygon<T> shrunkBorder(largestHole);
    if (!isUsablePolygon(shrunkBorder)) {
      logInvalidLayerPolygon<T>("largest shrunk border", _theta_lb, _theta_ub, largestHole);
      _infeasible = true;
      return;
    }
    _shrunkBorder = shrunkBorder;
    _shrunkBorderExactList.push_back(largestHole);
  }

  if (!_grownObs.empty() && !_shrunkBorderExactList.empty()) {
    std::vector<Polygon_2> polygons = _grownObs;
    polygons.insert(
        polygons.end(),
        _shrunkBorderExactList.begin(),
        _shrunkBorderExactList.end()
    );
    for (const Polygon_2 &polygon : polygons) {
      for (const auto &edge : polygon.edges()) {
        if (edge.squared_length() < RVG_EPS) Utils::print("Env Edge with small length: ", edge, " distance = ", std::to_string(std::sqrt(CGAL::to_double(edge.squared_length()))));
        _segments.push_back(edge);
      }
    }
    CGAL::insert_non_intersecting_curves(_env, _segments.begin(), _segments.end());
  }

  // hole env
  std::vector<Segment_2> holeSegments;
  for (const auto &hole : _holes) {
    for (const auto &edge : hole.edges()) {
      if (edge.squared_length() < RVG_EPS) Utils::print("Hole Edge with small length: ", edge, " distance = ", std::to_string(std::sqrt(CGAL::to_double(edge.squared_length()))));
      holeSegments.push_back(edge);
    }
  }
  CGAL::insert_non_intersecting_curves(_holeEnv, holeSegments.begin(), holeSegments.end());
}

template<typename T>
void Layer<T>::_connectVisibleVertices() {
  // outer obstacles
  _vq = std::make_shared<VQ>(_env);
  for (const auto &v : _env.vertex_handles()) {
    // print out v
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
        (_theta_ub + _theta_lb) / 2.
    );
    std::pair<Point_2, Point_2> neighbor1 = _getNeighbor(v);

    PL_2 pl(visibleArea);
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
            (_theta_ub + _theta_lb) / 2.
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
    // bool vertexWeWant = false;
    // if (v->point().x() > 10 && v->point().x() < 15 && v->point().y() < -10 && v->point().y() > -15) vertexWeWant = true;
    bool vOnBorder = _isOnBorder(v->point());
    // if (vertexWeWant) Utils::print("Hole vertex: ", v->point(), " on border: ", vOnBorder);
    // if (vertexWeWant) Utils::print("Vertex isReflex: ", _isReflex(v, true, vOnBorder));
    if (vOnBorder)
      if (!_isReflex(v, true, true)) continue;
    Arrangement_2 visibleArea = visibleAreaOnHoles<T>(v->point(), _holeEnv, _holeVQ);
    std::shared_ptr<Vertex<T>> v1 = std::make_shared<Vertex<T>>(
        CGAL::to_double(v->point().x()),
        CGAL::to_double(v->point().y()),
        _theta_lb,
        _theta_ub,
        (_theta_ub + _theta_lb) / 2.
    );
    std::pair<Point_2, Point_2> neighbor1 = _getNeighbor(v);
    _visiblePolygonCache.insert({*v1, Utils::arrangementToPolygon<T>(visibleArea)});
    _neighborCache.insert({*v1, neighbor1});
    _isVertexOnHoleCache.insert({*v1, true});
    if (std::find(_points.begin(), _points.end(), v->point()) == _points.end()) {
      _points.push_back(v->point());
      _vertices.push_back(*v1);
    }

    PL_2 pl(visibleArea);
    for (const auto &p : _holeEnv.vertex_handles()) {
      if (p->point() == v->point()) continue;
      std::pair<Point_2, Point_2> neighbor2 = _getNeighbor(p);
      // bool pvertexWeWant = false;
      // if (p->point().x() > 10 && p->point().x() < 15 && p->point().y() < -10 && p->point().y() > -15) pvertexWeWant = true;
      bool pOnBorder = _isOnBorder(p->point());
      // if (pvertexWeWant) Utils::print("pHole vertex: ", p->point(), " on border: ", pOnBorder);
      // if (pvertexWeWant) Utils::print("pVertex isReflex: ", _isReflex(p, true, pOnBorder));
      // if (vertexWeWant) Utils::print("Checking bitangent between ", v->point(), " and ", p->point());
      // if (vertexWeWant) Utils::print("Neighbor1: ", neighbor1.first, " ", neighbor1.second, " Neighbor2: ", neighbor2.first, " ", neighbor2.second);
      // if (vertexWeWant) Utils::print("Is bitangent: ", Utils::isBitangent<T>(v->point(),
                                                                                                //  neighbor1.first,
                                                                                                //  neighbor1.second,
                                                                                                //  p->point(),
                                                                                                //  neighbor2.first,
                                                                                                //  neighbor2.second));
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
        // if(vertexWeWant) Utils::print("Adding edge between ", v->point(), " and ", p->point());
      std::shared_ptr<Vertex<T>> v2 = std::make_shared<Vertex<T>>(
            CGAL::to_double(p->point().x()),
            CGAL::to_double(p->point().y()),
            _theta_lb,
            _theta_ub,
            (_theta_ub + _theta_lb) / 2.
        );
        _neighborCache.insert({*v2, neighbor2});
        _isVertexOnHoleCache.insert({*v2, true});
        if (std::find(_points.begin(), _points.end(), p->point()) == _points.end()) {
          _points.push_back(p->point());
          _vertices.push_back(*v2);
        }
        _edges.push_back(std::make_pair(v1, v2));
        //TODO: edge (v1, v2) and (v2, v1) are all added. We should remove this redundancy.
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
    if (curr->is_on_outer_ccb()) {
      vl = curr->source()->point();
    }
    if (curr->is_on_inner_ccb()) {
      vr = curr->source()->point();
    }
    curr++;
    if (curr == first)
      break;
  }
  bool isReflex;
  if (_borderIsHole){
    if (isOnBorder){
      isReflex = CGAL::right_turn(vl, v->point(), vr);
      _isReflexCache.insert({p, isReflex});
    } else {
      isReflex = CGAL::left_turn(vl, v->point(), vr);
      _isReflexCache.insert({p, isReflex});
    }
  }
  else{
    if (isHoleVertex || isOnBorder) {
      isReflex = CGAL::right_turn(vl, v->point(), vr);
      // isReflex = CGAL::left_turn(vl, v->point(), vr);
      _isReflexCache.insert({p, isReflex});
    } else {
      isReflex = CGAL::left_turn(vl, v->point(), vr);
      _isReflexCache.insert({p, isReflex});
    }
  }
  // if (isHoleVertex || isOnBorder) {
  //   isReflex = CGAL::right_turn(vl, v->point(), vr);
  //   // isReflex = CGAL::left_turn(vl, v->point(), vr);
  //   _isReflexCache.insert({p, isReflex});
  // } else {
  //   isReflex = CGAL::left_turn(vl, v->point(), vr);
  //   _isReflexCache.insert({p, isReflex});
  // }
  return isReflex;
}

template<typename T>
bool Layer<T>::_isOnBorder(const Point_2 &v) const {
  for (const auto &border : _shrunkBorderExactList) {
    if (CGAL::bounded_side_2(
            border.vertices_begin(), border.vertices_end(), v, K()
        ) == CGAL::ON_BOUNDARY) {
      return true;
    }
  }
  for (const auto &hole : _borderHoles) {
    //   Point_2 closestVertex;
    //   double closestSquaredDistance = std::numeric_limits<double>::max();
    //   bool closestExactEqual = false;
    //   for (const auto &vertex : hole.vertices()) {
    //     const double squaredDistance = CGAL::to_double(CGAL::squared_distance(vertex, v));
    //     if (squaredDistance < closestSquaredDistance) {
    //       closestSquaredDistance = squaredDistance;
    //       closestVertex = vertex;
    //       closestExactEqual = vertex == v;
    //     }
    //   }
    //   auto result = CGAL::bounded_side_2(hole.vertices_begin(), hole.vertices_end(), v, K());
    //   Utils::print(
    //       "query", v,
    //       "bounded_side_2(query)", result,
    //       "on boundary:", result == CGAL::ON_BOUNDARY,
    //       "closest vertex:", closestVertex,
    //       "closest squared distance:", closestSquaredDistance,
    //       "closest exact equal:", closestExactEqual
    //   );
    if (CGAL::bounded_side_2(hole.vertices_begin(), hole.vertices_end(), v, K()) == CGAL::ON_BOUNDARY)
      return true;
  }
  return false;
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
  (void)border;
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += "fig.set_size_inches(12, 10)\n";

  for (size_t index = 0; index < obstacles.size(); ++index) {
    appendTopologyPolygonPlot(
        pythonScript,
        obstacles[index].getPolygon(),
        "#CBD5E1",
        "#64748B",
        index == 0 ? "Original obstacles" : "_nolegend_",
        0.22,
        0.7,
        "-",
        10,
        ""
    );
  }

  for (size_t index = 0; index < _borderHoles.size(); ++index) {
    appendTopologyPolygonPlot(
        pythonScript,
        _borderHoles[index],
        "#BAE6FD",
        "#0284C7",
        index == 0 ? "borderHoles (_borderHoles)" : "_nolegend_",
        0.12,
        2.0,
        "--",
        20,
        "BH" + std::to_string(index)
    );
  }

  for (size_t index = 0; index < _shrunkBorderExactList.size(); ++index) {
    appendTopologyPolygonPlot(
        pythonScript,
        _shrunkBorderExactList[index],
        "none",
        "#047857",
        index == 0
            ? "shrunkBorderExactList (_shrunkBorderExactList)"
            : "_nolegend_",
        1.0,
        3.5,
        "-",
        40,
        "SBE" + std::to_string(index)
    );
  }

  for (size_t index = 0; index < _grownObs.size(); ++index) {
    appendTopologyPolygonPlot(
        pythonScript,
        _grownObs[index],
        "#FCA5A5",
        "#DC2626",
        index == 0 ? "grownObs (_grownObs)" : "_nolegend_",
        0.32,
        1.4,
        "-",
        30,
        ""
    );
  }

  for (size_t index = 0; index < _holes.size(); ++index) {
    appendTopologyPolygonPlot(
        pythonScript,
        _holes[index],
        "#E9D5FF",
        "#9333EA",
        index == 0 ? "holes (_holes)" : "_nolegend_",
        0.55,
        1.8,
        ":",
        50,
        "H" + std::to_string(index)
    );
  }

  if (graph) {
    pythonScript += graph->drawOnAxis("ax", 60);
  }

  std::ostringstream title;
  title << std::fixed << std::setprecision(1)
        << "Layer [" << _theta_lb * 180.0 / M_PI << " deg, "
        << _theta_ub * 180.0 / M_PI << " deg]"
        << " | border holes=" << _borderHoles.size()
        << " | environment borders=" << _shrunkBorderExactList.size()
        << " | grown obstacles=" << _grownObs.size()
        << " | holes=" << _holes.size()
        << " | borderIsHole=" << (_borderIsHole ? "true" : "false");
  pythonScript += "ax.set_title('" + title.str() + "')\n";
  pythonScript += "ax.set_xlabel('x')\n";
  pythonScript += "ax.set_ylabel('y')\n";
  pythonScript += "ax.set_aspect('equal', adjustable='box')\n";
  pythonScript += "ax.grid(True, color='#CBD5E1', linewidth=0.6, alpha=0.7)\n";
  pythonScript += "handles, labels = ax.get_legend_handles_labels()\n";
  pythonScript += "legend = dict(zip(labels, handles))\n";
  pythonScript += "ax.legend(legend.values(), legend.keys(), loc='best', fontsize=8, framealpha=0.92)\n";
  pythonScript += "fig.tight_layout()\n";
  pythonScript += "fig.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  if (show) pythonScript += "plt.show()\n";
  std::string pythonSavePath = figPath + ".py";
  if (!Utils::writeStringToFile<T>(pythonScript, pythonSavePath)) {
    throw std::runtime_error("Failed to write python script");
  }
  if (!Utils::runPythonScriptAndRemove<T>(pythonSavePath)) {
    throw std::runtime_error("Failed to run and remove python script");
  }
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
  const Point_2 &p = v.getPoint();
  if (IN_ANY_POLYGON(p, _holes)) return true;
  if (IN_ANY_POLYGON(p, _grownObs)) return false;
  return IN_ANY_POLYGON(p, _shrunkBorderExactList);
}

template<typename T>
std::shared_ptr<typename Layer<T>::Arrangement_2> Layer<T>::getVisibleArea(const Vertex<T> &v) const {
  if(_infeasible) 
    throw std::runtime_error("The layer is infeasible");
  const Point_2 &p = v.getPoint();
  if (!_holes.empty() && IN_ANY_POLYGON(p, _holes)) {
    return std::make_shared<Arrangement_2>(visibleAreaOnHoles<T>(p, _holeEnv, _holeVQ));
  }
  if (!_shrunkBorderExactList.empty() &&
      IN_ANY_POLYGON(p, _shrunkBorderExactList) &&
      !_grownObs.empty() &&
      !IN_ANY_POLYGON(p, _grownObs)) {
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
  Vertex<T> v(x, y, _theta_lb, _theta_ub, (_theta_lb + _theta_ub) / 2.);
  Polygon<T> polygon(_visiblePolygonCache.at(v));
  return polygon;
}

template<typename T>
const std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &
Layer<T>::getEdges() const {
  return _edges;
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
std::pair<typename Layer<T>::Point_2,
          typename Layer<T>::Point_2> Layer<T>::getNeighbor(const Vertex<T> &v) const {
  const auto found = _neighborCache.find(v);
  if (found != _neighborCache.end()) {
    return found->second;
  }
  // Vertex equality is tolerant while Vertex::hash() is quantized.  Values on
  // opposite sides of a quantization boundary can therefore compare equal but
  // occupy different buckets.  Search the cache and exact arrangements before
  // declaring a generated layer invalid.
  for (const auto &entry : _neighborCache) {
    if (entry.first == v) {
      return entry.second;
    }
  }
  for (auto handle = _env.vertices_begin(); handle != _env.vertices_end(); ++handle) {
    if (handle->point() == v.getPoint()) {
      return _getNeighbor(handle);
    }
  }
  for (auto handle = _holeEnv.vertices_begin(); handle != _holeEnv.vertices_end(); ++handle) {
    if (handle->point() == v.getPoint()) {
      return _getNeighbor(handle);
    }
  }
  throw std::out_of_range("Layer neighbor is not present in either arrangement");
}

template<typename T>
bool Layer<T>::isVertexOnHole(const Vertex<T> &vertex) const {
  const auto found = _isVertexOnHoleCache.find(vertex);
  if (found != _isVertexOnHoleCache.end()) {
    return found->second;
  }
  for (const auto &entry : _isVertexOnHoleCache) {
    if (entry.first == vertex) {
      return entry.second;
    }
  }
  return std::find(_holePoints.begin(), _holePoints.end(), vertex.getPoint()) !=
      _holePoints.end();
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
