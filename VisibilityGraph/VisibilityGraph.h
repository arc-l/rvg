#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_

#include <VisibilityGraph/Layer.h>

namespace RotationalVisibilityGraph {
template<typename T>
class VisibilityGraph {
 public:
  struct TwoTuple {
    T first, second;
    bool operator==(const TwoTuple &rhs) const;
  };
  struct TwoTupleHash {
    size_t operator()(const TwoTuple &layer) const;
    bool operator()(const TwoTuple &lhs, const TwoTuple &rhs) const;
  };
  typedef typename VisibilityGraph<T>::TwoTuple TwoTuple;
  typedef typename VisibilityGraph<T>::TwoTupleHash TwoTupleHash;
  typedef typename Vertex<T>::Hash VertexHash;
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T
  VisibilityGraph(
      const Polygon<T> &robot,
      const Polygon<T> &border,
      const std::vector<Polygon<T>> &obstacles,
      bool fineApprox,
      int numThreads,
      bool verbose = false
  );
  VisibilityGraph(
      const Polygon<T> &robot,
      const Polygon<T> &border,
      const std::vector<Polygon<T>> &obstacles,
      int resolution,
      bool fineApprox,
      int numThreads,
      bool optimal = false,
      bool verbose = false
  );
  void setWeight(T euclideanWeight, T rotationalWeight);
  virtual std::vector<Vertex<T>> shortestPath(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end, int interpolationDensity=0, bool unwrap=true);
  T getPathLength() const;
  T getTotalRotation() const;
  T getTotalTime() const;
  T getBuildTime() const;
  T getSearchTime() const;
  const std::vector<Layer<T>> &getLayers() const; 
  ~VisibilityGraph() = default;
  void draw(const std::string &figPath, bool title=true, bool path=true, bool graph=true, bool show=false, int wayPoints=-1);
  void drawLayers3D(const std::string &figPath, const std::string& type, bool show);
  void drawVisibleAreas() const;
  std::string drawLayers3D(const std::string& type) const;
  void animation(const std::string& figPath, bool title=true);
  void drawSetup() const;
  std::string draw3D(bool graph) const;
  void debugMaxPropagation();
  std::vector<std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>> getEdgeLayersForward() const;
  std::vector<std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>> getEdgeLayersBackward() const;
  std::vector<std::shared_ptr<Vertex<T>>> getAdjacentVertices(std::shared_ptr<Vertex<T>> v) const;
  std::vector<Polygon<T>> verticalDecomposition();
  typedef typename Vertex<T>::SharedPtrVertexHash SharedPtrVertexHash;
  typedef typename Vertex<T>::SharedPtrVertexEqual SharedPtrVertexEqual;
  std::vector<std::shared_ptr<Vertex<T>>> getGraphVertices() const;
  Graph<T> &getGraph();
  const Graph<T> &getGraph() const;
  void _addStartAndGoal(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal);
  

 protected:
  bool _fineApprox;
  bool _verbose;
  std::vector<Layer<T>> _layers;
  Polygon<T> _robot, _border, _realRobot;
  std::vector<Polygon<T>> _obstacles;
  Graph<T> _graph;
  std::vector<Vertex<T>> _sol;
  Vertex<T> _start, _goal;
  T _buildTime, _searchTime, _totalTime, _pathLength, _totalRotation;
  T _euclideanWeight, _rotationalWeight;
  int _symmetricCycle;
  int _numThreads;
  bool _isSymmetric;
  T _roundUpTheta;
  void _interpolation(int density);
  void _unwrapPath();
  virtual void _resolutionSymmetryCheck();
  [[nodiscard]] std::string _drawSetup() const;

 private:
  int _resolution;
  bool _optimal;
  std::vector<std::unordered_set<TwoTuple, TwoTupleHash>> _layerVertices;
  std::vector<std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>> _edgeLayersForward, _edgeLayersBackward;
  void _buildLayers();
  void _connectLayers();
  void _connectLayersParallelizedVertices();
};
}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_
