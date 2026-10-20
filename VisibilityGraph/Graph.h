#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_

#include <VisibilityGraph/Edge.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace RotationalVisibilityGraph {

struct GraphMergeStatistics {
  std::size_t sourceVertices = 0;
  std::size_t sourceEdges = 0;
  std::size_t insertedVertices = 0;
  std::size_t insertedEdges = 0;
  std::size_t duplicateEdges = 0;
  std::size_t overlayArcs = 0;
  bool adoptedStorage = false;
  bool compacted = false;
  double vertexRemapMilliseconds = 0;
  double edgeInsertionMilliseconds = 0;
  double compactionMilliseconds = 0;
  double totalMilliseconds = 0;
};

struct GraphQueryStatistics {
  std::size_t expandedVertices = 0;
  std::size_t relaxedArcs = 0;
  std::size_t overlayArcsBefore = 0;
  bool compacted = false;
  double indexMaintenanceMilliseconds = 0;
  double searchMilliseconds = 0;
  double totalMilliseconds = 0;
};

template<typename T>
class Graph {
 public:
  using VertexId = std::uint32_t;
  static constexpr VertexId InvalidVertexId =
      std::numeric_limits<VertexId>::max();
  using VertexHash = typename Vertex<T>::Hash;
  using SharedPtrVertexHash = typename Vertex<T>::SharedPtrVertexHash;
  using SharedPtrVertexEqual = typename Vertex<T>::SharedPtrVertexEqual;

  struct DeltaArc {
    VertexId neighbor = InvalidVertexId;
    T weight = static_cast<T>(0);
  };

  Graph();
  Graph(const Graph<T> &other);
  Graph(Graph<T> &&other) noexcept = default;
  Graph<T> &operator=(const Graph<T> &other);
  Graph<T> &operator=(Graph<T> &&other) noexcept = default;

  VertexId addVertex(const std::shared_ptr<Vertex<T>> &vertex);
  void addEdge(std::shared_ptr<Vertex<T>> first,
               std::shared_ptr<Vertex<T>> second);
  void addEdges(const std::vector<std::pair<
      std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &edges);
  void addIndexedEdges(
      const std::vector<std::shared_ptr<Vertex<T>>> &vertices,
      const std::vector<std::pair<VertexId, VertexId>> &edges,
      int numThreads
  );
  void finalizeStatic();
  void mergeGraph(const Graph<T> &other);
  void mergeGraph(Graph<T> &&other);

  bool findVertexId(const std::shared_ptr<Vertex<T>> &vertex,
                    VertexId &vertexId) const;
  bool containsVertex(const std::shared_ptr<Vertex<T>> &vertex) const;
  const std::shared_ptr<Vertex<T>> &vertexById(VertexId vertexId) const;
  bool hasEdge(VertexId first, VertexId second) const;
  std::size_t vertexCount() const;
  std::size_t edgeCount() const;
  std::size_t overlayArcCount() const;

  template <typename Function>
  void forEachNeighbor(VertexId vertexId, Function &&function) const {
    if (vertexId >= _verticesById.size()) {
      return;
    }
    if (_baseOffsets.size() == _verticesById.size() + 1) {
      for (std::size_t arc = _baseOffsets[vertexId];
           arc < _baseOffsets[vertexId + 1]; ++arc) {
        function(_baseNeighbors[arc], _baseWeights[arc]);
      }
    }
    if (vertexId < _deltaAdjacency.size()) {
      for (const DeltaArc &arc : _deltaAdjacency[vertexId]) {
        function(arc.neighbor, arc.weight);
      }
    }
  }

  const std::unordered_set<std::shared_ptr<Vertex<T>>,
                           SharedPtrVertexHash,
                           SharedPtrVertexEqual> &
  getNeighbors(std::shared_ptr<Vertex<T>> vertex) const;
  const std::unordered_map<
      std::shared_ptr<Vertex<T>>,
      std::unordered_set<std::shared_ptr<Vertex<T>>,
                         SharedPtrVertexHash,
                         SharedPtrVertexEqual>,
      SharedPtrVertexHash,
      SharedPtrVertexEqual> &getAdjacencyList() const;
  const std::unordered_set<Edge<T>, typename Edge<T>::Hash> &getEdges() const;
  const std::unordered_set<std::shared_ptr<Vertex<T>>,
                           SharedPtrVertexHash,
                           SharedPtrVertexEqual> &getVertices() const;

  int size() const;
  void setWeight(T euclideanWeight, T rotationalWeight);
  std::vector<Vertex<T>> shortestPath(
      std::shared_ptr<Vertex<T>> start,
      std::shared_ptr<Vertex<T>> goal,
      bool verbose
  ) const;
  std::vector<Vertex<T>> shortestPathDijkstra(
      std::shared_ptr<Vertex<T>> start,
      std::shared_ptr<Vertex<T>> goal,
      bool verbose = false
  ) const;
  int numConnectedComponents() const;
  void draw(const std::string &figPath, bool show = false);
  void draw3D(const std::string &figPath, bool show = false);
  void buildEdges();
  std::string draw(const std::string &axisName) const;
  std::string drawOnAxis(const std::string &axisName,
                         int zOrder = 2) const;
  std::string draw() const;
  const GraphMergeStatistics &getLastMergeStatistics() const;
  const GraphQueryStatistics &getLastQueryStatistics() const;
  ~Graph() = default;

 private:
  using NeighborSet = std::unordered_set<
      std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual>;
  using AdjacencyMap = std::unordered_map<
      std::shared_ptr<Vertex<T>>, NeighborSet,
      SharedPtrVertexHash, SharedPtrVertexEqual>;

  static std::uint64_t edgeKey(VertexId first, VertexId second);
  T edgeWeight(VertexId first, VertexId second) const;
  bool baseHasEdge(VertexId first, VertexId second) const;
  bool addEdgeIds(VertexId first, VertexId second);
  void collectUndirectedEdges(
      std::vector<std::pair<VertexId, VertexId>> &edges
  ) const;
  void compactOverlay() const;
  double prepareForSearch() const;
  void mergeGraphImpl(const Graph<T> &other);
  void adoptStorage(Graph<T> &other);
  void clearPrimaryStorage();
  void invalidateCompatibilityViews() const;
  void materializeVertexView() const;
  void materializeAdjacencyView() const;
  void materializeEdgeView() const;
  std::vector<Vertex<T>> shortestPathIndexed(
      std::shared_ptr<Vertex<T>> start,
      std::shared_ptr<Vertex<T>> goal,
      bool verbose,
      bool useHeuristic
  ) const;

  // A vertex without orientation permits zero-cost changes of angle.
  bool _allVerticesHaveTheta = true;
  T _euclideanWeight = static_cast<T>(1);
  T _rotationalWeight = static_cast<T>(0);
  std::vector<std::shared_ptr<Vertex<T>>> _verticesById;
  std::unordered_map<std::shared_ptr<Vertex<T>>, VertexId,
                     SharedPtrVertexHash, SharedPtrVertexEqual> _vertexIds;

  mutable std::vector<std::size_t> _baseOffsets{0};
  mutable std::vector<VertexId> _baseNeighbors;
  mutable std::vector<T> _baseWeights;
  mutable std::vector<std::vector<DeltaArc>> _deltaAdjacency;
  mutable std::unordered_set<std::uint64_t> _overlayEdgeKeys;
  mutable std::size_t _baseEdgeCount = 0;
  mutable std::size_t _overlayArcCount = 0;

  mutable AdjacencyMap _adjacencyList;
  mutable std::unordered_set<Edge<T>, typename Edge<T>::Hash> _edges;
  mutable std::unordered_set<std::shared_ptr<Vertex<T>>,
                             SharedPtrVertexHash,
                             SharedPtrVertexEqual> _vertices;
  mutable bool _vertexViewValid = false;
  mutable bool _adjacencyViewValid = false;
  mutable bool _edgeViewValid = false;
  mutable GraphMergeStatistics _lastMergeStatistics;
  mutable GraphQueryStatistics _lastQueryStatistics;
};

}  // namespace RotationalVisibilityGraph

#endif  // ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_
