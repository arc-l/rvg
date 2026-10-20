# Static RVG CPU and CUDA update

## Source and scope

This update extracts the static planner from `arc-l/drvg` branch `experiments`
at commit `53daeef0`. That revision includes the CUDA branch (`2ecb3f29`),
complete fixed-point propagation, CPU AABB filtering, and indexed graph storage
and search. It precedes all commits on the car/nonholonomic branch. Dynamic
observation, exploration, temporary goals, and car planning are not imported.
Static utility APIs (`MapGenerator`, `Edge`, `Polygon`, and `Utils`) retain
the helpers present before dRVG cleanup commit `4d5956b9`; the original RVG
examples still use them. Their acceleration-era implementation is unchanged.

The official repository's existing local fixes are preserved in the newer
implementations: multiple shrunken border components, isolated hole-vertex
visibility caches, and duplicate query-vertex insertion.

## Behavior and API changes

- `optimal=true` computes independent forward and backward propagation closures,
  including orientation wraparound. This intentionally replaces the older
  incomplete sweep and can change graph size and path cost.
- `optimal=false` runs one global adjacent-layer propagation round.
- `CpuReference` uses direct CGAL predicates. `Cpu` first rejects candidates
  outside conservative visibility-polygon bounding boxes, then uses CGAL for
  survivors. The scalar orientation/PIP interval classifier is retained as an
  implementation utility, but it is not the default CPU hot path.
- `Cuda` filters candidates with outward-rounded intervals. Uncertain cases go
  through the same exact CPU predicate. `Auto` uses CUDA at 65,536 or more candidates
  per round and replays on CPU if CUDA fails.
- The graph uses stable integer IDs, compact sorted adjacency, and an insertion
  overlay. A* and the Dijkstra test oracle share precomputed edge weights.
  Existing pointer-based graph accessors remain available. Translation and
  rotation weights must be nonnegative.
- C++ and Python constructors no longer accept `fineApprox`. Remove that
  argument; never substitute `true`. The construction is conservative.
- The constructor's final optional argument selects `VertexPropagationBackend`.
  Python also exposes propagation/construction statistics and indexed counts.
- Both `float` and `double` remain explicitly instantiated in C++; Python uses
  `double`.
- The review fixed an A* bug for graphs mixing oriented and unoriented vertices.
  Rotational lower bounds are disabled in that case because edges through an
  unoriented vertex have no rotation cost. Fully oriented RVG queries retain the
  rotational heuristic.
- Exact arithmetic requires ordinary floating-point semantics; the build does
  not enable fast-math or CPU-specific AVX flags.

## Validation

Use the CPU and CUDA configure/build/test commands in the README. Regression
coverage includes exact CPU/reference topology and propagation rounds,
float/double search, graph merging and compaction, disconnected configuration
space, solvable-map fixtures, and Python backend selection. With CUDA enabled,
the device test checks strict interior/exterior, boundary, overlapping interval,
and collinear decisions, then compares graph topology and query costs with CPU.

`mainBenchmarkVertexPropagation` is the static benchmark executable; the
included 20 hard-map configurations are inputs, not newly measured results.
This port does not claim a new end-to-end CPU or CUDA speedup measurement.

Local validation used the `afosr` environment and `-j32` for every build:
the Release CPU/Python suite passed 6/6 tests, and the CUDA suite passed 6/6
tests on an RTX 4090 with CUDA 13.1. GPU parity ran (it was not skipped),
including float/double construction and both propagation modes. The original
optional static OMPL comparisons also compiled against the pinned submodule.
