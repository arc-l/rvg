#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_Layer_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_Layer_H_

#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Edge.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Graph.h>
#include <VisibilityGraph/VisibilityCheck.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

namespace RotationalVisibilityGraph {
template<typename T>
class Layer {
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T
 public:
  Layer();
  Layer(T theta_lb,
        T theta_ub,
        T roundUpTheta,
        bool fineApprox,
        bool hashWithTheta,
        bool simplifiedGeometry
  );
  Layer(const Layer<T> &layer);
  void buildVisibilityGraph(const Polygon<T> &robot, const Polygon<T> &border, const std::vector<Polygon<T>> &obstacles);
  const T &getThetaLb() const;
  const T &getThetaUb() const;
  [[nodiscard]] const std::vector<Point_2> &getPoints() const;
  [[nodiscard]] const Point_2 &getPoint(int index) const;
  [[nodiscard]] const Vertex<T> &getVertex(int index) const;
  [[nodiscard]] const std::vector<Vertex_const_handle> getVertexHandles() const;
  [[nodiscard]] const Vertex_const_handle &getVertexHandle(int index) const;
  [[nodiscard]] const std::vector<Polygon_2> &getGrownObs() const;
  [[nodiscard]] std::vector<Polygon<T>> getGrownObsRVGPolygon() const;
  [[nodiscard]] const Arrangement_2 &getEnv() const;
  [[nodiscard]] const Arrangement_2 &getHoleEnv() const;
  [[nodiscard]] const std::vector<Polygon_2> &getHoles() const;
  [[nodiscard]] std::vector<Polygon<T>> getHolesPolygon() const;
  [[nodiscard]] const Polygon<T> &getShrinkedBorder() const;
  [[nodiscard]] std::shared_ptr<VQ> getVQ() const;
  [[nodiscard]] std::shared_ptr<VQ> getHoleVQ() const;
  [[nodiscard]] bool hasHoles() const;
  [[nodiscard]] bool borderIsHole() const;
  const std::unordered_map<Vertex<T>, Polygon_2, typename Vertex<T>::Hash> getVisibleAreaCache() const;
  void draw(const std::string &figPath, const Polygon<T> &border, const std::vector<Polygon<T>> &obstacles, Graph<T> *graph, bool show = false) const;
  std::string drawGrownObs3D(const std::string& type) const;
  std::string draw(bool curr) const;
  std::string drawVisibleAreas(int cnt) const;
  std::string drawVisibleAreas() const;
  void drawRobotBB(const Polygon<T> & robot) const;
  bool legalConfig(const Vertex<T> &v) const;
  std::shared_ptr<Arrangement_2> getVisibleArea(const Vertex<T> &v) const;
  std::unordered_map<Vertex<T>, Polygon<T>, typename Vertex<T>::Hash> getVisibleAreaPolygons() const;
  Polygon<T> getVisibleAreaPolygon(T x, T y) const;
  const std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &getEdges() const;
  const std::pair<Point_2, Point_2> &getNeighbor(const Vertex<T> &v) const;
  bool isVertexOnHole(const Vertex<T> &vertex) const;
  bool isFeasible() const;
  const Polygon<T> &getRobotBBox() const;
  Polygon<T> getRobotBBoxInverted() const;

 private:
  bool _fineApprox;
  bool _simplifiedGeometry;
  bool _hashWithTheta;
  bool _hasHoles;
  bool _borderIsHole;
  bool _infeasible;
  T _theta_lb, _theta_ub, _roundUpTheta;
  std::shared_ptr<VQ> _vq, _holeVQ;
  std::vector<Vertex<T>> _vertices;
  std::vector<Point_2> _points, _holePoints;
  std::vector<Vertex_const_handle> _vertexHandles;
  std::vector<Polygon_2> _grownObs;
  std::vector<Polygon_2> _holes;
  std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> _edges;
  Polygon<T> _shrunkBorder, _robotBBox, _robotBBoxInverted;
  std::vector<Segment_2> _segments;
  Polygon_with_holes_2 _complementBorder;
  Arrangement_2 _env;
  Arrangement_2 _holeEnv;
  std::unordered_map<Vertex<T>, bool, typename Vertex<T>::Hash> _isReflexCache;
  std::unordered_map<Vertex<T>, bool, typename Vertex<T>::Hash> _isVertexOnHoleCache;
  std::unordered_map<Vertex<T>, Polygon_2, typename Vertex<T>::Hash> _visiblePolygonCache;
  std::unordered_map<Vertex<T>, std::pair<Point_2, Point_2>, typename Vertex<T>::Hash> _neighborCache;
  void _initRobotBBox(const Polygon<T> &robot);
  Polygon<T> _underestimate(const Polygon<T> &robot);
  Polygon<T> _overestimate(const Polygon<T> &robot);
  Polygon<T> _underestimate(const Polygon<T> &robot, const Vertex<T>& center);
  Polygon<T> _overestimate(const Polygon<T> &robot, const Vertex<T>& center);
  void _underestimateRobotBBox(const Polygon<T> &robot);
  void _overestimateRobotBBox(const Polygon<T> &robot);
  std::optional<Vertex<T>> _lineIntersection(const Vertex<T>& v1, const Vertex<T>& v2, const Vertex<T>& v3, const Vertex<T>& v4);
  void _shrinkBorder(const Polygon<T> &border);
  void _buildMap(const std::vector<Polygon<T>> &obstacles);
  void _connectVisibleVertices();
  bool _isReflex(const Vertex_const_handle &v, bool isHoleVertex, bool isOnBorder);
  [[nodiscard]] bool _isOnBorder(const Point_2 &v) const;
  [[nodiscard]] std::pair<Point_2, Point_2> _getNeighbor(const Vertex_const_handle &handle) const;
};
}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_
