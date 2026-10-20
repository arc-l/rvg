#include <VisibilityGraph/Graph.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

using namespace RotationalVisibilityGraph;

namespace {

using Scalar = double;

bool require(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

bool hasNeighbor(
    const Graph<Scalar> &graph,
    const std::shared_ptr<Vertex<Scalar>> &from,
    const std::shared_ptr<Vertex<Scalar>> &to
) {
    const auto &neighbors = graph.getNeighbors(from);
    return neighbors.find(to) != neighbors.end();
}

template <typename T>
T pathCost(const std::vector<Vertex<T>> &path, T alpha, T beta)
{
    T result = 0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        const Vertex<T> &from = path[index - 1];
        const Vertex<T> &to = path[index];
        result += alpha * from.dist(to);
        if (from.hasTheta() && to.hasTheta()) {
            result += beta * from.rotationalDist(to);
        }
    }
    return result;
}

template <typename T>
bool testIndexedSearchParity()
{
    const T pi = static_cast<T>(3.14159265358979323846);
    const auto start = std::make_shared<Vertex<T>>(0, 0, 0, 0, 0);
    const auto rotating = std::make_shared<Vertex<T>>(1, 0, pi / 2, pi / 2, pi / 2);
    const auto goal = std::make_shared<Vertex<T>>(2, 0, 0, 0, 0);
    const auto detour = std::make_shared<Vertex<T>>(1, 1, 0, 0, 0);
    const auto isolated = std::make_shared<Vertex<T>>(20, 20, 0, 0, 0);
    Graph<T> graph;
    graph.addEdge(start, rotating);
    graph.addEdge(rotating, goal);
    graph.addEdge(start, detour);
    graph.addEdge(detour, goal);
    graph.addVertex(isolated);
    graph.finalizeStatic();
    graph.setWeight(static_cast<T>(1), static_cast<T>(2));

    const auto astar = graph.shortestPath(start, goal, false);
    const auto dijkstra = graph.shortestPathDijkstra(start, goal);
    const T tolerance = std::is_same<T, float>::value
        ? static_cast<T>(1e-4) : static_cast<T>(1e-10);
    if (!require(!astar.empty() && !dijkstra.empty(),
                 "A* and Dijkstra must find the weighted path") ||
        !require(std::abs(pathCost(astar, T(1), T(2)) -
                          pathCost(dijkstra, T(1), T(2))) <= tolerance,
                 "A* and indexed Dijkstra costs differ") ||
        !require(graph.shortestPath(start, isolated, false).empty(),
                 "unreachable indexed goal unexpectedly has a path") ||
        !require(graph.shortestPath(start, start, false).size() == 1,
                 "start-equals-goal must return one vertex") ||
        !require(!graph.shortestPath(start, goal, false).empty(),
                 "repeated indexed query changed connectivity")) {
        return false;
    }
    // A 2D intermediate vertex has no rotational edge cost. Charging
    // the full remaining angle as an A* heuristic would overestimate it.
    const auto mixedStart = std::make_shared<Vertex<T>>(0, 0, 0, 0, 0);
    const auto oriented = std::make_shared<Vertex<T>>(0, 1, 0, 0, 0);
    const auto unoriented = std::make_shared<Vertex<T>>(1, 1);
    const auto mixedGoal = std::make_shared<Vertex<T>>(2, 0, pi, pi, pi);
    Graph<T> mixed;
    mixed.addEdges({{mixedStart, oriented}, {oriented, unoriented},
                    {unoriented, mixedGoal}, {mixedStart, mixedGoal}});
    mixed.finalizeStatic();
    mixed.setWeight(T(1), T(1));
    Graph<T> copied = mixed;
    Graph<T> assigned;
    assigned = mixed;
    Graph<T> adopted;
    adopted.setWeight(T(1), T(1));
    adopted.mergeGraph(std::move(mixed));
    Graph<T> merged;
    merged.setWeight(T(1), T(1));
    merged.addEdge(mixedStart, mixedGoal);
    merged.finalizeStatic();
    merged.mergeGraph(copied);
    for (const Graph<T> *candidate : {&copied, &assigned, &adopted, &merged}) {
        const auto route = candidate->shortestPath(mixedStart, mixedGoal, false);
        const auto oracle = candidate->shortestPathDijkstra(mixedStart, mixedGoal);
        if (!require(route.size() == 4, "mixed-orientation A* missed the cheaper route") ||
            !require(std::abs(pathCost(route, T(1), T(1)) -
                              pathCost(oracle, T(1), T(1))) <= tolerance,
                     "mixed-orientation A* differs from Dijkstra")) {
            return false;
        }
    }
    return true;
}

}

int main()
{
    const auto p = std::make_shared<Vertex<Scalar>>(0, 0, 0, 0, 0);
    const auto q = std::make_shared<Vertex<Scalar>>(1, 0, 0, 0, 0);
    const auto r = std::make_shared<Vertex<Scalar>>(2, 0, 0, 0, 0);
    const auto s = std::make_shared<Vertex<Scalar>>(10, 0, 0, 0, 0);
    const auto t = std::make_shared<Vertex<Scalar>>(11, 0, 0, 0, 0);

    Graph<Scalar> left;
    left.setWeight(1, 0);
    left.addEdge(p, q);
    left.buildEdges();
    if (!require(!left.getEdges().empty(), "edge cache setup failed")) {
        return 1;
    }

    Graph<Scalar> right;
    right.addEdge(q, r);
    right.addEdge(s, t);
    left.mergeGraph(right);

    if (!require(left.getEdges().size() == 3, "lazy edge view must reflect merged edges") ||
        !require(left.size() == 5, "merge must preserve all five vertices") ||
        !require(left.numConnectedComponents() == 2, "merge must preserve disconnected components") ||
        !require(hasNeighbor(left, p, q) && hasNeighbor(left, q, p), "original adjacency changed") ||
        !require(hasNeighbor(left, q, r) && hasNeighbor(left, r, q), "merged adjacency is not symmetric") ||
        !require(hasNeighbor(left, s, t) && hasNeighbor(left, t, s), "disjoint adjacency was lost")) {
        return 1;
    }

    const std::vector<Vertex<Scalar>> path = left.shortestPath(p, r, false);
    if (!require(path.size() == 3, "merged graph must connect p to r") ||
        !require(path.front() == *p && path.back() == *r, "merged shortest path endpoints changed")) {
        return 1;
    }

    const size_t vertexCount = left.getVertices().size();
    const size_t qDegree = left.getNeighbors(q).size();
    left.mergeGraph(right);
    if (!require(left.getVertices().size() == vertexCount, "duplicate merge changed vertex count") ||
        !require(left.getNeighbors(q).size() == qDegree, "duplicate merge changed adjacency")) {
        return 1;
    }

    Graph<Scalar> movedSource;
    const auto u = std::make_shared<Vertex<Scalar>>(3, 0, 0, 0, 0);
    movedSource.addEdge(r, u);
    left.mergeGraph(std::move(movedSource));
    if (!require(hasNeighbor(left, r, u) && hasNeighbor(left, u, r),
                 "rvalue merge lost adjacency") ||
        !require(movedSource.size() == 0, "rvalue merge must transfer source adjacency")) {
        return 1;
    }

    Graph<Scalar> adoptionSource;
    const auto adoptionA =
        std::make_shared<Vertex<Scalar>>(30, 0, 0, 0, 0);
    const auto adoptionB =
        std::make_shared<Vertex<Scalar>>(31, 0, 0, 0, 0);
    adoptionSource.addEdge(adoptionA, adoptionB);
    adoptionSource.finalizeStatic();
    adoptionSource.getAdjacencyList();
    Graph<Scalar> adopted;
    adopted.mergeGraph(std::move(adoptionSource));
    const GraphMergeStatistics &adoptionStatistics =
        adopted.getLastMergeStatistics();
    if (!require(adoptionStatistics.adoptedStorage,
                 "empty destination did not adopt moved storage") ||
        !require(adoptionSource.vertexCount() == 0 &&
                     adoptionSource.edgeCount() == 0,
                 "adopted source was not emptied") ||
        !require(adopted.vertexCount() == 2 && adopted.edgeCount() == 1,
                 "adoption changed graph topology")) {
        return 1;
    }

    Graph<Scalar> copied = adopted;
    const auto adoptionC =
        std::make_shared<Vertex<Scalar>>(32, 0, 0, 0, 0);
    copied.addEdge(adoptionB, adoptionC);
    if (!require(copied.edgeCount() == 2 && adopted.edgeCount() == 1,
                 "graph copy did not isolate primary storage") ||
        !require(copied.getEdges().size() == 2,
                 "copied graph failed to rebuild lazy edge view")) {
        return 1;
    }

    Graph<Scalar> hybrid;
    hybrid.addEdge(p, q);
    hybrid.finalizeStatic();
    hybrid.addEdge(q, r);
    Graph<Scalar> csrSource;
    csrSource.addEdge(r, u);
    csrSource.finalizeStatic();
    hybrid.mergeGraph(csrSource);
    const std::size_t hybridEdges = hybrid.edgeCount();
    hybrid.mergeGraph(csrSource);
    if (!require(hybridEdges == 3 && hybrid.edgeCount() == hybridEdges,
                 "CSR-to-overlay duplicate merge changed topology") ||
        !require(hybrid.getLastMergeStatistics().duplicateEdges == 1,
                 "duplicate merge was not reported")) {
        return 1;
    }

    Graph<Scalar> compactionGraph;
    constexpr std::size_t compactionEdgeCount = 2050;
    std::vector<std::shared_ptr<Vertex<Scalar>>> compactionVertices;
    compactionVertices.reserve(compactionEdgeCount + 1);
    for (std::size_t index = 0; index <= compactionEdgeCount; ++index) {
        compactionVertices.push_back(std::make_shared<Vertex<Scalar>>(
            static_cast<Scalar>(index), 100
        ));
        if (index != 0) {
            compactionGraph.addEdge(
                compactionVertices[index - 1], compactionVertices[index]
            );
        }
    }
    if (!require(compactionGraph.overlayArcCount() >= 4096,
                 "compaction fixture did not cross the overlay threshold") ||
        !require(!compactionGraph.shortestPath(
                     compactionVertices.front(),
                     compactionVertices.back(),
                     false
                 ).empty(),
                 "compaction query lost the chain path") ||
        !require(compactionGraph.getLastQueryStatistics().compacted,
                 "threshold query did not compact the overlay") ||
        !require(compactionGraph.overlayArcCount() == 0 &&
                     compactionGraph.edgeCount() == compactionEdgeCount,
                 "compaction changed edge count or left overlay arcs")) {
        return 1;
    }

    if (!testIndexedSearchParity<float>() ||
        !testIndexedSearchParity<double>()) {
        return 1;
    }
    return 0;
}
