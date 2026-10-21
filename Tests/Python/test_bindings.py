"""Smoke-test the supported Python geometry and static planning interface."""
import math
import rvg

assert not hasattr(rvg, "add")
assert not hasattr(rvg.rvg, "debugMaxPropagation")
vertex = rvg.vertex
robot = rvg.polygon([vertex(-0.1, -0.1), vertex(0.1, -0.1), vertex(0.1, 0.1), vertex(-0.1, 0.1)], False)
border = rvg.polygon([vertex(-3, -3), vertex(3, -3), vertex(3, 3), vertex(-3, 3)], True)
planner = rvg.rvg(robot, border, [], 8, 2)
theta = math.pi / 8
start = vertex(-1, 0, theta, theta, theta)
goal = vertex(1, 0, theta, theta, theta)
assert planner.addVertex(start)
assert planner.addVertex(goal)
planner.setWeight(1, 0.1)
path = planner.shortestPath(start, goal)
assert len(path) >= 2
assert path[0].dist(start) < 1e-9
assert path[-1].dist(goal) < 1e-9
assert 0 < len(planner.getLayers()) <= 8  # Symmetry can fold equivalent layers.
assert planner.getGraph().shortestPath(start, goal)
assert rvg.__version__ == "0.1.0"


# Exercise backend selection, statistics, indexed counts, and exact CPU parity.
assert isinstance(rvg.cudaVertexPropagationAvailable(), bool)
graphs = []
for backend in (rvg.VertexPropagationBackend.Cpu, rvg.VertexPropagationBackend.CpuReference):
    current = rvg.rvg(robot, border, [], 8, 2, optimal=True, propagationBackend=backend)
    stats = current.getVertexPropagationStatistics()
    assert stats["requestedBackend"] == backend
    assert stats["forwardRounds"] <= 8 and stats["backwardRounds"] <= 8
    assert current.getConstructionStatistics()["totalMilliseconds"] >= 0
    current.setWeight(1, 0.1)
    route = current.shortestPath(start, goal)
    assert route
    graph = current.getGraph()
    assert graph.vertexCount() == len(graph.getVertices())
    assert graph.edgeCount() == len(graph.getEdges())
    assert graph.shortestPathDijkstra(start, goal)
    key = lambda v: (v.getX(), v.getY(), v.getTheta())
    graphs.append({tuple(sorted((key(a), key(b)))) for a, b in graph.getEdges()})
assert graphs[0] == graphs[1]
