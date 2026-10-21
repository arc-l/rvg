#include "Graph.h"


#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>

namespace RotationalVisibilityGraph {

namespace {
using GraphClock = std::chrono::steady_clock;

double elapsedMilliseconds(GraphClock::time_point start) {
  return std::chrono::duration<double, std::milli>(
      GraphClock::now() - start
  ).count();
}
}  // namespace

template<typename T>
Graph<T>::Graph() = default;

template<typename T>
Graph<T>::Graph(const Graph<T> &other)
    : _allVerticesHaveTheta(other._allVerticesHaveTheta),
      _euclideanWeight(other._euclideanWeight),
      _rotationalWeight(other._rotationalWeight),
      _verticesById(other._verticesById),
      _vertexIds(other._vertexIds),
      _baseOffsets(other._baseOffsets),
      _baseNeighbors(other._baseNeighbors),
      _baseWeights(other._baseWeights),
      _deltaAdjacency(other._deltaAdjacency),
      _overlayEdgeKeys(other._overlayEdgeKeys),
      _baseEdgeCount(other._baseEdgeCount),
      _overlayArcCount(other._overlayArcCount),
      _lastMergeStatistics(other._lastMergeStatistics),
      _lastQueryStatistics(other._lastQueryStatistics) {}

template<typename T>
Graph<T> &Graph<T>::operator=(const Graph<T> &other) {
  if (this == &other) {
    return *this;
  }
  _allVerticesHaveTheta = other._allVerticesHaveTheta;
  _euclideanWeight = other._euclideanWeight;
  _rotationalWeight = other._rotationalWeight;
  _verticesById = other._verticesById;
  _vertexIds = other._vertexIds;
  _baseOffsets = other._baseOffsets;
  _baseNeighbors = other._baseNeighbors;
  _baseWeights = other._baseWeights;
  _deltaAdjacency = other._deltaAdjacency;
  _overlayEdgeKeys = other._overlayEdgeKeys;
  _baseEdgeCount = other._baseEdgeCount;
  _overlayArcCount = other._overlayArcCount;
  _lastMergeStatistics = other._lastMergeStatistics;
  _lastQueryStatistics = other._lastQueryStatistics;
  _adjacencyList.clear();
  _edges.clear();
  _vertices.clear();
  _vertexViewValid = false;
  _adjacencyViewValid = false;
  _edgeViewValid = false;
  return *this;
}

template<typename T>
std::uint64_t Graph<T>::edgeKey(VertexId first, VertexId second) {
  if (second < first) {
    std::swap(first, second);
  }
  return (static_cast<std::uint64_t>(first) << 32U) |
      static_cast<std::uint64_t>(second);
}

template<typename T>
T Graph<T>::edgeWeight(VertexId first, VertexId second) const {
  const auto &from = _verticesById[first];
  const auto &to = _verticesById[second];
  const T rotationalDistance = from->hasTheta() && to->hasTheta()
      ? from->rotationalDist(*to)
      : static_cast<T>(0);
  return _euclideanWeight * from->dist(*to) +
      _rotationalWeight * rotationalDistance;
}

template<typename T>
typename Graph<T>::VertexId Graph<T>::addVertex(
    const std::shared_ptr<Vertex<T>> &vertex
) {
  const auto found = _vertexIds.find(vertex);
  if (found != _vertexIds.end()) {
    return found->second;
  }
  if (_verticesById.size() >= InvalidVertexId) {
    throw std::overflow_error("Graph exceeded the 32-bit vertex ID limit");
  }
  const std::size_t previousVertexCount = _verticesById.size();
  const bool baseIsFinalized =
      _baseOffsets.size() == previousVertexCount + 1;
  const VertexId vertexId = static_cast<VertexId>(previousVertexCount);
  _verticesById.push_back(vertex);
  _allVerticesHaveTheta = _allVerticesHaveTheta && vertex->hasTheta();
  _vertexIds.emplace(vertex, vertexId);
  _deltaAdjacency.emplace_back();
  if (baseIsFinalized) {
    _baseOffsets.push_back(_baseOffsets.back());
  }
  invalidateCompatibilityViews();
  return vertexId;
}

template<typename T>
bool Graph<T>::baseHasEdge(VertexId first, VertexId second) const {
  if (_baseOffsets.size() != _verticesById.size() + 1 ||
      first >= _verticesById.size()) {
    return false;
  }
  return std::binary_search(
      _baseNeighbors.begin() + static_cast<std::ptrdiff_t>(_baseOffsets[first]),
      _baseNeighbors.begin() + static_cast<std::ptrdiff_t>(_baseOffsets[first + 1]),
      second
  );
}

template<typename T>
bool Graph<T>::addEdgeIds(VertexId first, VertexId second) {
  if (first >= _verticesById.size() || second >= _verticesById.size()) {
    throw std::out_of_range("Graph edge has an invalid vertex ID");
  }
  const std::uint64_t key = edgeKey(first, second);
  if (_overlayEdgeKeys.find(key) != _overlayEdgeKeys.end() ||
      baseHasEdge(first, second)) {
    return false;
  }
  _overlayEdgeKeys.insert(key);
  _deltaAdjacency[first].push_back({second, edgeWeight(first, second)});
  ++_overlayArcCount;
  if (first != second) {
    _deltaAdjacency[second].push_back({first, edgeWeight(second, first)});
    ++_overlayArcCount;
  }
  invalidateCompatibilityViews();
  return true;
}

template<typename T>
void Graph<T>::addEdge(std::shared_ptr<Vertex<T>> first,
                       std::shared_ptr<Vertex<T>> second) {
  ASSERT_MSG(!std::isnan(first->getX()) && !std::isnan(first->getY()),
             "Vertex 1 is NaN");
  ASSERT_MSG(!std::isnan(second->getX()) && !std::isnan(second->getY()),
             "Vertex 2 is NaN");
  addEdgeIds(addVertex(first), addVertex(second));
}

template<typename T>
void Graph<T>::addEdges(const std::vector<std::pair<
    std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &edges) {
  const std::size_t maximumNewVertices = edges.size() * 2;
  _verticesById.reserve(_verticesById.size() + maximumNewVertices);
  _vertexIds.reserve(_vertexIds.size() + maximumNewVertices);
  _deltaAdjacency.reserve(_deltaAdjacency.size() + maximumNewVertices);
  _overlayEdgeKeys.reserve(_overlayEdgeKeys.size() + edges.size());
  std::vector<std::pair<VertexId, VertexId>> indexedEdges;
  indexedEdges.reserve(edges.size());
  for (const auto &edge : edges) {
    ASSERT_MSG(!std::isnan(edge.first->getX()) &&
                   !std::isnan(edge.first->getY()),
               "Vertex 1 is NaN");
    ASSERT_MSG(!std::isnan(edge.second->getX()) &&
                   !std::isnan(edge.second->getY()),
               "Vertex 2 is NaN");
    VertexId first = addVertex(edge.first);
    VertexId second = addVertex(edge.second);
    if (second < first) {
      std::swap(first, second);
    }
    indexedEdges.emplace_back(first, second);
  }
  std::sort(indexedEdges.begin(), indexedEdges.end());
  indexedEdges.erase(
      std::unique(indexedEdges.begin(), indexedEdges.end()),
      indexedEdges.end()
  );
  for (const auto &edge : indexedEdges) {
    addEdgeIds(edge.first, edge.second);
  }
}

template<typename T>
void Graph<T>::addIndexedEdges(
    const std::vector<std::shared_ptr<Vertex<T>>> &vertices,
    const std::vector<std::pair<VertexId, VertexId>> &edges,
    int numThreads
) {
  (void)numThreads;
  _verticesById.reserve(_verticesById.size() + vertices.size());
  _vertexIds.reserve(_vertexIds.size() + vertices.size());
  _deltaAdjacency.reserve(_deltaAdjacency.size() + vertices.size());
  _overlayEdgeKeys.reserve(_overlayEdgeKeys.size() + edges.size());
  std::vector<VertexId> destinationIds(vertices.size());
  for (std::size_t index = 0; index < vertices.size(); ++index) {
    destinationIds[index] = addVertex(vertices[index]);
  }
  std::vector<std::pair<VertexId, VertexId>> canonicalEdges;
  canonicalEdges.reserve(edges.size());
  for (const auto &edge : edges) {
    if (edge.first >= destinationIds.size() ||
        edge.second >= destinationIds.size()) {
      throw std::out_of_range("Indexed graph edge has an invalid vertex ID");
    }
    VertexId first = destinationIds[edge.first];
    VertexId second = destinationIds[edge.second];
    if (second < first) {
      std::swap(first, second);
    }
    canonicalEdges.emplace_back(first, second);
  }
  std::sort(canonicalEdges.begin(), canonicalEdges.end());
  canonicalEdges.erase(
      std::unique(canonicalEdges.begin(), canonicalEdges.end()),
      canonicalEdges.end()
  );
  for (const auto &edge : canonicalEdges) {
    addEdgeIds(edge.first, edge.second);
  }
}

template<typename T>
void Graph<T>::collectUndirectedEdges(
    std::vector<std::pair<VertexId, VertexId>> &edges
) const {
  edges.reserve(edges.size() + edgeCount());
  if (_baseOffsets.size() == _verticesById.size() + 1) {
    for (VertexId source = 0; source < _verticesById.size(); ++source) {
      for (std::size_t arc = _baseOffsets[source];
           arc < _baseOffsets[source + 1]; ++arc) {
        const VertexId target = _baseNeighbors[arc];
        if (source <= target) {
          edges.emplace_back(source, target);
        }
      }
    }
  }
  for (VertexId source = 0; source < _deltaAdjacency.size(); ++source) {
    for (const DeltaArc &arc : _deltaAdjacency[source]) {
      if (source <= arc.neighbor) {
        edges.emplace_back(source, arc.neighbor);
      }
    }
  }
}

template<typename T>
void Graph<T>::compactOverlay() const {
  if (_overlayEdgeKeys.empty() &&
      _baseOffsets.size() == _verticesById.size() + 1) {
    return;
  }

  const bool baseValid =
      _baseOffsets.size() == _verticesById.size() + 1;
  for (auto &delta : _deltaAdjacency) {
    std::sort(
        delta.begin(), delta.end(),
        [](const DeltaArc &left, const DeltaArc &right) {
          return left.neighbor < right.neighbor;
        }
    );
  }
  std::vector<std::size_t> offsets(_verticesById.size() + 1, 0);
  for (VertexId source = 0; source < _verticesById.size(); ++source) {
    const std::size_t baseArcCount = baseValid
        ? _baseOffsets[source + 1] - _baseOffsets[source]
        : 0;
    offsets[source + 1] = offsets[source] + baseArcCount +
        _deltaAdjacency[source].size();
  }

  std::vector<VertexId> neighbors(offsets.back());
  std::vector<T> weights(neighbors.size());
  for (VertexId source = 0; source < _verticesById.size(); ++source) {
    std::size_t outputArc = offsets[source];
    std::size_t baseArc = baseValid ? _baseOffsets[source] : 0;
    const std::size_t baseEnd = baseValid ? _baseOffsets[source + 1] : 0;
    std::size_t deltaArc = 0;
    const auto &delta = _deltaAdjacency[source];
    while (baseArc < baseEnd && deltaArc < delta.size()) {
      const VertexId baseNeighbor = _baseNeighbors[baseArc];
      const DeltaArc &overlayNeighbor = delta[deltaArc];
      if (baseNeighbor < overlayNeighbor.neighbor) {
        neighbors[outputArc] = baseNeighbor;
        weights[outputArc++] = _baseWeights[baseArc++];
      } else {
        neighbors[outputArc] = overlayNeighbor.neighbor;
        weights[outputArc++] = overlayNeighbor.weight;
        ++deltaArc;
      }
    }
    while (baseArc < baseEnd) {
      neighbors[outputArc] = _baseNeighbors[baseArc];
      weights[outputArc++] = _baseWeights[baseArc++];
    }
    while (deltaArc < delta.size()) {
      neighbors[outputArc] = delta[deltaArc].neighbor;
      weights[outputArc++] = delta[deltaArc].weight;
      ++deltaArc;
    }
  }
  _baseOffsets = std::move(offsets);
  _baseNeighbors = std::move(neighbors);
  _baseWeights = std::move(weights);
  _baseEdgeCount += _overlayEdgeKeys.size();
  _deltaAdjacency.assign(_verticesById.size(), {});
  _overlayEdgeKeys.clear();
  _overlayArcCount = 0;
  invalidateCompatibilityViews();
}

template<typename T>
void Graph<T>::finalizeStatic() {
  compactOverlay();
}

template<typename T>
double Graph<T>::prepareForSearch() const {
  const std::size_t threshold = std::max<std::size_t>(
      4096, _baseNeighbors.size() / 4
  );
  if (_overlayArcCount < threshold) {
    return 0;
  }
  const auto start = GraphClock::now();
  compactOverlay();
  const double milliseconds = elapsedMilliseconds(start);
  _lastMergeStatistics.compacted = true;
  _lastMergeStatistics.compactionMilliseconds += milliseconds;
  return milliseconds;
}

template<typename T>
void Graph<T>::setWeight(T euclideanWeight, T rotationalWeight) {
  if (_euclideanWeight == euclideanWeight &&
      _rotationalWeight == rotationalWeight) {
    return;
  }
  _euclideanWeight = euclideanWeight;
  _rotationalWeight = rotationalWeight;
  if (_baseOffsets.size() == _verticesById.size() + 1) {
    for (VertexId source = 0; source < _verticesById.size(); ++source) {
      for (std::size_t arc = _baseOffsets[source];
           arc < _baseOffsets[source + 1]; ++arc) {
        _baseWeights[arc] = edgeWeight(source, _baseNeighbors[arc]);
      }
    }
  }
  for (VertexId source = 0; source < _deltaAdjacency.size(); ++source) {
    for (DeltaArc &arc : _deltaAdjacency[source]) {
      arc.weight = edgeWeight(source, arc.neighbor);
    }
  }
}

template<typename T>
bool Graph<T>::findVertexId(const std::shared_ptr<Vertex<T>> &vertex,
                            VertexId &vertexId) const {
  const auto found = _vertexIds.find(vertex);
  if (found == _vertexIds.end()) {
    return false;
  }
  vertexId = found->second;
  return true;
}

template<typename T>
bool Graph<T>::containsVertex(
    const std::shared_ptr<Vertex<T>> &vertex
) const {
  return _vertexIds.find(vertex) != _vertexIds.end();
}

template<typename T>
const std::shared_ptr<Vertex<T>> &Graph<T>::vertexById(
    VertexId vertexId
) const {
  return _verticesById.at(vertexId);
}

template<typename T>
bool Graph<T>::hasEdge(VertexId first, VertexId second) const {
  if (first >= _verticesById.size() || second >= _verticesById.size()) {
    return false;
  }
  return baseHasEdge(first, second) ||
      _overlayEdgeKeys.find(edgeKey(first, second)) !=
          _overlayEdgeKeys.end();
}

template<typename T>
std::size_t Graph<T>::vertexCount() const {
  return _verticesById.size();
}

template<typename T>
std::size_t Graph<T>::edgeCount() const {
  return _baseEdgeCount + _overlayEdgeKeys.size();
}

template<typename T>
std::size_t Graph<T>::overlayArcCount() const {
  return _overlayArcCount;
}

template<typename T>
void Graph<T>::mergeGraphImpl(const Graph<T> &other) {
  const auto remapStart = GraphClock::now();
  std::vector<VertexId> destinationIds(other.vertexCount());
  for (VertexId sourceId = 0; sourceId < other.vertexCount(); ++sourceId) {
    const std::size_t before = vertexCount();
    destinationIds[sourceId] = addVertex(other.vertexById(sourceId));
    if (vertexCount() != before) {
      ++_lastMergeStatistics.insertedVertices;
    }
  }
  _lastMergeStatistics.vertexRemapMilliseconds =
      elapsedMilliseconds(remapStart);

  const auto edgeStart = GraphClock::now();
  std::vector<std::pair<VertexId, VertexId>> sourceEdges;
  other.collectUndirectedEdges(sourceEdges);
  for (const auto &sourceEdge : sourceEdges) {
    if (addEdgeIds(destinationIds[sourceEdge.first],
                   destinationIds[sourceEdge.second])) {
      ++_lastMergeStatistics.insertedEdges;
    } else {
      ++_lastMergeStatistics.duplicateEdges;
    }
  }
  _lastMergeStatistics.edgeInsertionMilliseconds =
      elapsedMilliseconds(edgeStart);
  _lastMergeStatistics.overlayArcs = _overlayArcCount;
}

template<typename T>
void Graph<T>::clearPrimaryStorage() {
  _allVerticesHaveTheta = true;
  _verticesById.clear();
  _vertexIds.clear();
  _baseOffsets.assign(1, 0);
  _baseNeighbors.clear();
  _baseWeights.clear();
  _deltaAdjacency.clear();
  _overlayEdgeKeys.clear();
  _baseEdgeCount = 0;
  _overlayArcCount = 0;
  invalidateCompatibilityViews();
}

template<typename T>
void Graph<T>::adoptStorage(Graph<T> &other) {
  const bool weightsMatch =
      _euclideanWeight == other._euclideanWeight &&
      _rotationalWeight == other._rotationalWeight;
  _allVerticesHaveTheta = other._allVerticesHaveTheta;
  _verticesById = std::move(other._verticesById);
  _vertexIds = std::move(other._vertexIds);
  _baseOffsets = std::move(other._baseOffsets);
  _baseNeighbors = std::move(other._baseNeighbors);
  _baseWeights = std::move(other._baseWeights);
  _deltaAdjacency = std::move(other._deltaAdjacency);
  _overlayEdgeKeys = std::move(other._overlayEdgeKeys);
  _baseEdgeCount = other._baseEdgeCount;
  _overlayArcCount = other._overlayArcCount;
  other.clearPrimaryStorage();
  if (!weightsMatch) {
    setWeight(_euclideanWeight, _rotationalWeight);
  }
  invalidateCompatibilityViews();
}

template<typename T>
void Graph<T>::mergeGraph(const Graph<T> &other) {
  if (this == &other) {
    return;
  }
  const auto totalStart = GraphClock::now();
  _lastMergeStatistics = GraphMergeStatistics();
  _lastMergeStatistics.sourceVertices = other.vertexCount();
  _lastMergeStatistics.sourceEdges = other.edgeCount();
  mergeGraphImpl(other);
  _lastMergeStatistics.totalMilliseconds = elapsedMilliseconds(totalStart);
}

template<typename T>
void Graph<T>::mergeGraph(Graph<T> &&other) {
  if (this == &other) {
    return;
  }
  const auto totalStart = GraphClock::now();
  _lastMergeStatistics = GraphMergeStatistics();
  _lastMergeStatistics.sourceVertices = other.vertexCount();
  _lastMergeStatistics.sourceEdges = other.edgeCount();
  if (vertexCount() == 0) {
    _lastMergeStatistics.insertedVertices = other.vertexCount();
    _lastMergeStatistics.insertedEdges = other.edgeCount();
    _lastMergeStatistics.adoptedStorage = true;
    adoptStorage(other);
  } else {
    mergeGraphImpl(other);
    other.clearPrimaryStorage();
  }
  _lastMergeStatistics.overlayArcs = _overlayArcCount;
  _lastMergeStatistics.totalMilliseconds = elapsedMilliseconds(totalStart);
}

template<typename T>
std::vector<Vertex<T>> Graph<T>::shortestPathIndexed(
    std::shared_ptr<Vertex<T>> start,
    std::shared_ptr<Vertex<T>> goal,
    bool verbose,
    bool useHeuristic
) const {
  const auto totalStart = GraphClock::now();
  _lastQueryStatistics = GraphQueryStatistics();
  _lastQueryStatistics.overlayArcsBefore = _overlayArcCount;
  _lastQueryStatistics.indexMaintenanceMilliseconds = prepareForSearch();
  _lastQueryStatistics.compacted =
      _lastQueryStatistics.overlayArcsBefore != 0 &&
      _overlayArcCount == 0;
  const auto searchStart = GraphClock::now();

  VertexId startId = InvalidVertexId;
  VertexId goalId = InvalidVertexId;
  if (!findVertexId(start, startId) || !findVertexId(goal, goalId)) {
    if (verbose) {
      Utils::print("Start or Goal vertex not in graph");
    }
    _lastQueryStatistics.searchMilliseconds = elapsedMilliseconds(searchStart);
    _lastQueryStatistics.totalMilliseconds = elapsedMilliseconds(totalStart);
    return {};
  }

  const T infinity = std::numeric_limits<T>::max();
  std::vector<T> distance(vertexCount(), infinity);
  std::vector<VertexId> parent(vertexCount(), InvalidVertexId);
  std::vector<std::uint8_t> closed(vertexCount(), 0);
  const auto heuristic = [&](VertexId vertexId) {
    if (!useHeuristic) {
      return static_cast<T>(0);
    }
    const auto &vertex = _verticesById[vertexId];
    const auto &goalVertex = _verticesById[goalId];
    T result = std::max(static_cast<T>(0), _euclideanWeight) *
        vertex->dist(*goalVertex);
    if (_allVerticesHaveTheta) {
      result += std::max(static_cast<T>(0), _rotationalWeight) *
          vertex->rotationalDist(*goalVertex);
    }
    return result;
  };

  struct QueueEntry {
    T estimate;
    T distance;
    VertexId vertex;
  };
  struct QueueGreater {
    bool operator()(const QueueEntry &left, const QueueEntry &right) const {
      if (left.estimate != right.estimate) {
        return left.estimate > right.estimate;
      }
      if (left.distance != right.distance) {
        return left.distance > right.distance;
      }
      return left.vertex > right.vertex;
    }
  };
  std::priority_queue<QueueEntry,
                      std::vector<QueueEntry>,
                      QueueGreater> queue;
  distance[startId] = static_cast<T>(0);
  queue.push({heuristic(startId), static_cast<T>(0), startId});

  while (!queue.empty()) {
    const QueueEntry entry = queue.top();
    queue.pop();
    if (entry.distance != distance[entry.vertex] || closed[entry.vertex]) {
      continue;
    }
    closed[entry.vertex] = 1;
    ++_lastQueryStatistics.expandedVertices;
    if (entry.vertex == goalId) {
      break;
    }
    forEachNeighbor(entry.vertex, [&](VertexId neighbor, T weight) {
      ++_lastQueryStatistics.relaxedArcs;
      if (closed[neighbor]) {
        return;
      }
      const T candidate = entry.distance + weight;
      if (candidate < distance[neighbor]) {
        distance[neighbor] = candidate;
        parent[neighbor] = entry.vertex;
        queue.push({candidate + heuristic(neighbor), candidate, neighbor});
      }
    });
  }

  std::vector<Vertex<T>> path;
  if (distance[goalId] != infinity) {
    std::vector<VertexId> reversed;
    for (VertexId current = goalId; current != startId;
         current = parent[current]) {
      if (current == InvalidVertexId || parent[current] == InvalidVertexId) {
        reversed.clear();
        break;
      }
      reversed.push_back(current);
    }
    if (startId == goalId || !reversed.empty()) {
      reversed.push_back(startId);
      path.reserve(reversed.size());
      for (auto iterator = reversed.rbegin(); iterator != reversed.rend();
           ++iterator) {
        path.push_back(*_verticesById[*iterator]);
      }
    }
  }


  _lastQueryStatistics.searchMilliseconds = elapsedMilliseconds(searchStart);
  _lastQueryStatistics.totalMilliseconds = elapsedMilliseconds(totalStart);
  return path;
}

template<typename T>
std::vector<Vertex<T>> Graph<T>::shortestPath(
    std::shared_ptr<Vertex<T>> start,
    std::shared_ptr<Vertex<T>> goal,
    bool verbose
) const {
  return shortestPathIndexed(std::move(start), std::move(goal), verbose, true);
}

template<typename T>
std::vector<Vertex<T>> Graph<T>::shortestPathDijkstra(
    std::shared_ptr<Vertex<T>> start,
    std::shared_ptr<Vertex<T>> goal,
    bool verbose
) const {
  return shortestPathIndexed(std::move(start), std::move(goal), verbose, false);
}

template<typename T>
int Graph<T>::numConnectedComponents() const {
  std::vector<std::uint8_t> visited(vertexCount(), 0);
  int components = 0;
  for (VertexId root = 0; root < vertexCount(); ++root) {
    if (visited[root]) {
      continue;
    }
    ++components;
    std::vector<VertexId> pending{root};
    visited[root] = 1;
    while (!pending.empty()) {
      const VertexId current = pending.back();
      pending.pop_back();
      forEachNeighbor(current, [&](VertexId neighbor, T) {
        if (!visited[neighbor]) {
          visited[neighbor] = 1;
          pending.push_back(neighbor);
        }
      });
    }
  }
  return components;
}

template<typename T>
void Graph<T>::invalidateCompatibilityViews() const {
  _vertexViewValid = false;
  _adjacencyViewValid = false;
  _edgeViewValid = false;
}

template<typename T>
void Graph<T>::materializeVertexView() const {
  if (_vertexViewValid) {
    return;
  }
  _vertices.clear();
  _vertices.reserve(vertexCount());
  _vertices.insert(_verticesById.begin(), _verticesById.end());
  _vertexViewValid = true;
}

template<typename T>
void Graph<T>::materializeAdjacencyView() const {
  if (_adjacencyViewValid) {
    return;
  }
  materializeVertexView();
  _adjacencyList.clear();
  _adjacencyList.reserve(vertexCount());
  for (VertexId source = 0; source < vertexCount(); ++source) {
    NeighborSet &neighbors = _adjacencyList[_verticesById[source]];
    forEachNeighbor(source, [&](VertexId target, T) {
      neighbors.insert(_verticesById[target]);
    });
  }
  _adjacencyViewValid = true;
}

template<typename T>
void Graph<T>::materializeEdgeView() const {
  if (_edgeViewValid) {
    return;
  }
  _edges.clear();
  std::vector<std::pair<VertexId, VertexId>> indexedEdges;
  collectUndirectedEdges(indexedEdges);
  _edges.reserve(indexedEdges.size());
  for (const auto &edge : indexedEdges) {
    _edges.insert(Edge<T>(
        *_verticesById[edge.first], *_verticesById[edge.second]
    ));
  }
  _edgeViewValid = true;
}

template<typename T>
const typename Graph<T>::NeighborSet &Graph<T>::getNeighbors(
    std::shared_ptr<Vertex<T>> vertex
) const {
  materializeAdjacencyView();
  return _adjacencyList.at(vertex);
}

template<typename T>
const typename Graph<T>::AdjacencyMap &Graph<T>::getAdjacencyList() const {
  materializeAdjacencyView();
  return _adjacencyList;
}

template<typename T>
const std::unordered_set<Edge<T>, typename Edge<T>::Hash> &
Graph<T>::getEdges() const {
  materializeEdgeView();
  return _edges;
}

template<typename T>
const std::unordered_set<std::shared_ptr<Vertex<T>>,
                         typename Graph<T>::SharedPtrVertexHash,
                         typename Graph<T>::SharedPtrVertexEqual> &
Graph<T>::getVertices() const {
  materializeVertexView();
  return _vertices;
}

template<typename T>
int Graph<T>::size() const {
  return static_cast<int>(vertexCount());
}

template<typename T>
void Graph<T>::buildEdges() {
  materializeEdgeView();
}

template<typename T>
void Graph<T>::draw(const std::string &figPath, bool show) {
  (void)figPath;
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  for (const auto &edge : getEdges()) {
    pythonScript += "ax.plot([" + std::to_string(edge.v1().getX()) + ", " +
        std::to_string(edge.v2().getX()) + "], [" +
        std::to_string(edge.v1().getY()) + ", " +
        std::to_string(edge.v2().getY()) +
        "], color='black', linewidth=0.5)\n";
  }
  for (const auto &vertex : getVertices()) {
    pythonScript += "ax.plot(" + std::to_string(vertex->getX()) + ", " +
        std::to_string(vertex->getY()) +
        ", 'o', color='bisque', markersize=1)\n";
  }
  pythonScript += "plt.axis('off')\n";
  if (show) {
    pythonScript += "plt.show()\n";
  }
  Utils::writeStringToFile<T>(pythonScript, "draw.py");
  Utils::runPythonScriptAndRemove<T>("draw.py");
}

template<typename T>
void Graph<T>::draw3D(const std::string &figPath, bool show) {
  std::string pythonScript;
  PYTHON_IMPORTS_3D(pythonScript)
  for (const auto &edge : getEdges()) {
    pythonScript += "ax.plot([" + std::to_string(edge.v1().getX()) + ", " +
        std::to_string(edge.v2().getX()) + "], [" +
        std::to_string(edge.v1().getY()) + ", " +
        std::to_string(edge.v2().getY()) + "], [" +
        std::to_string(edge.v1().getTheta()) + ", " +
        std::to_string(edge.v2().getTheta()) +
        "], color='black', linewidth=0.5)\n";
  }
  pythonScript += "plt.savefig('" + figPath +
      "', bbox_inches='tight', dpi=500)\n";
  if (show) {
    pythonScript += "plt.show()\n";
  }
  Utils::writeStringToFile<T>(pythonScript, "draw.py");
  Utils::runPythonScriptAndRemove<T>("draw.py");
}

template<typename T>
std::string Graph<T>::draw() const {
  return drawOnAxis("ax");
}

template<typename T>
std::string Graph<T>::draw(const std::string &axisName) const {
  return drawOnAxis(axisName);
}

template<typename T>
std::string Graph<T>::drawOnAxis(const std::string &axisName,
                                 int zOrder) const {
  std::string pythonScript;
  for (const auto &edge : getEdges()) {
    const std::vector<T> edgeX = edge.getX();
    const std::vector<T> edgeY = edge.getY();
    pythonScript += axisName + ".plot([" + std::to_string(edgeX[0]) +
        ", " + std::to_string(edgeX[1]) + "], [" +
        std::to_string(edgeY[0]) + ", " + std::to_string(edgeY[1]) +
        "], '-', color='lightcoral', linewidth=0.5, zorder=" +
        std::to_string(zOrder) + ")\n";
  }
  for (const auto &vertex : getVertices()) {
    pythonScript += axisName + ".plot(" + std::to_string(vertex->getX()) +
        ", " + std::to_string(vertex->getY()) +
        ", 'o', color='bisque', markersize=0.5, zorder=" +
        std::to_string(zOrder + 1) + ")\n";
  }
  return pythonScript;
}

template<typename T>
const GraphMergeStatistics &Graph<T>::getLastMergeStatistics() const {
  return _lastMergeStatistics;
}

template<typename T>
const GraphQueryStatistics &Graph<T>::getLastQueryStatistics() const {
  return _lastQueryStatistics;
}

template class Graph<double>;
template class Graph<float>;

}  // namespace RotationalVisibilityGraph
