"""Minimal smoke test for the Python bindings defined in Python/graph.h.

Run after installing the package, e.g.:
    pip install -e .
    python python-example/test_graph_bindings.py
"""

from __future__ import annotations

from rvg import graph, vertex

def xy(v) -> tuple[float, float]:
    return (float(v.getX()), float(v.getY()))

def check(label: str, condition: bool, detail: str = "") -> None:
    status = "PASS" if condition else "FAIL"
    if detail:
        print(f"[{status}] {label}: {detail}")
    else:
        print(f"[{status}] {label}")


def main() -> None:
    g = graph()
    theta_lb = 0.0
    theta_ub = 6.283185307179586

    def vtx(x: float, y: float, theta: float = 0.0):
        return vertex(x, y, theta_lb, theta_ub, theta)

    # Component 1: A - B - C (with direct A - C edge so shortest path can be weighted)
    a = vtx(0.0, 0.0, 0.0)
    b = vtx(1.0, 0.0, 0.0)
    c = vtx(2.0, 0.0, 0.0)
    g.addEdge(a, b)
    g.addEdge(b, c)
    g.addEdge(a, c)

    # Component 2: D - E
    d = vtx(10.0, 10.0, 0.0)
    e = vtx(11.0, 10.0, 0.0)
    g.addEdge(d, e)

    check("graph.size()", g.size() == 5, f"expected=5 actual={g.size()}")
    check("len(getVertices())", len(g.getVertices()) == 5, f"actual={len(g.getVertices())}")
    check(
        "numConnectedComponents()",
        g.numConnectedComponents() == 2,
        f"expected=2 actual={g.numConnectedComponents()}",
    )

    neighbors_b = {xy(v) for v in g.getNeighbors(b)}
    check(
        "getNeighbors(b)",
        neighbors_b == {(0.0, 0.0), (2.0, 0.0)},
        f"neighbors={neighbors_b}",
    )

    # Prefer the direct A->C edge by ignoring rotation cost.
    g.setWeight(1.0, 0.0)
    path = g.shortestPath(a, c, False)
    path_xy = [xy(v) for v in path]
    check(
        "shortestPath(a, c)",
        path_xy == [(0.0, 0.0), (2.0, 0.0)],
        f"path={path_xy}",
    )

    # An unreachable query should return an empty list.
    unreachable = g.shortestPath(a, vtx(999.0, 999.0, 0.0), False)
    check(
        "shortestPath(a, missing_vertex)",
        unreachable == [],
        f"path={unreachable}",
    )

    # Build edge cache and inspect binding return types.
    g.buildEdges()
    edges = g.getEdges()
    check("getEdges() after buildEdges()", len(edges) > 0, f"count={len(edges)}")
    check("edge tuple arity", all(len(edge) == 2 for edge in edges))

    script = g.drawScript()
    check(
        "drawScript()",
        isinstance(script, str) and len(script) > 0,
        f"type={type(script).__name__} len={len(script) if isinstance(script, str) else 'n/a'}",
    )

    # mergeGraph() smoke tests
    g_left = graph()
    g_right = graph()
    p = vtx(100.0, 0.0, 0.0)
    q = vtx(101.0, 0.0, 0.0)
    r = vtx(102.0, 0.0, 0.0)
    s = vtx(200.0, 0.0, 0.0)
    t = vtx(201.0, 0.0, 0.0)

    g_left.addEdge(p, q)
    g_right.addEdge(q, r)  # overlaps on q (same coordinates)
    g_right.addEdge(s, t)  # disjoint component

    left_size_before = g_left.size()
    right_size = g_right.size()
    g_left.mergeGraph(g_right)

    check(
        "mergeGraph() increases/merges vertices",
        g_left.size() == 5,
        f"left_before={left_size_before} right={right_size} merged={g_left.size()}",
    )
    check(
        "mergeGraph() preserves components with overlap",
        g_left.numConnectedComponents() == 2,
        f"components={g_left.numConnectedComponents()}",
    )

    neighbors_q = {xy(v) for v in g_left.getNeighbors(q)}
    check(
        "mergeGraph() merges adjacency on shared vertex",
        neighbors_q == {(100.0, 0.0), (102.0, 0.0)},
        f"neighbors(q)={neighbors_q}",
    )

    g_left.setWeight(1.0, 0.0)
    merged_path = g_left.shortestPath(p, r, False)
    merged_path_xy = [xy(v) for v in merged_path]
    check(
        "mergeGraph() enables path across merged overlap",
        merged_path_xy == [(100.0, 0.0), (101.0, 0.0), (102.0, 0.0)],
        f"path={merged_path_xy}",
    )

    size_after_first_merge = g_left.size()
    g_left.mergeGraph(g_right)
    check(
        "mergeGraph() duplicate merge is stable",
        g_left.size() == size_after_first_merge,
        f"before={size_after_first_merge} after={g_left.size()}",
    )

    print("graph binding smoke test passed")
    print(f"vertices={len(g.getVertices())}, edges={len(edges)}, components={g.numConnectedComponents()}")


if __name__ == "__main__":
    main()
