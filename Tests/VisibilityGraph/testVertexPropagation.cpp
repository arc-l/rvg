#include <Utils/Utils.h>
#include <VisibilityGraph/Utils.h>
#include <VisibilityGraph/VisibilityGraph.h>
#ifdef RVG_HAS_CUDA
#include <VisibilityGraph/VertexPropagationCuda.h>
#endif

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace RotationalVisibilityGraph;

namespace {

using T = double;

using DebugEdgeLayers = std::vector<std::vector<std::pair<
    std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>
>>>;

struct Scenario {
  Polygon<T> robot;
  Polygon<T> border;
  std::vector<Polygon<T>> obstacles;
  std::shared_ptr<Vertex<T>> start;
  std::shared_ptr<Vertex<T>> goal;
};

bool require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

Scenario loadScenario(const std::filesystem::path &path) {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Unable to load " + path.string());
  }
  Scenario scenario;
  scenario.robot = Utils::getRobot<T>(document);
  scenario.obstacles = Utils::getObstacles<T>(document);
  const auto *environment =
      document.RootElement()->FirstChildElement("environment");
  if (Utils::get<bool>(*environment, "useBoundary", false)) {
    scenario.border = Utils::getBoundary<T>(document);
  } else {
    scenario.border = Utils::getMap<T>(
        Utils::get<int>(*environment, "mapSize", 100)
    );
  }
  const auto *plannerSettings =
      document.RootElement()->FirstChildElement("plannerSettings");
  scenario.start = Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("start")
           ->FirstChildElement("Vertex")
  );
  scenario.goal = Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("goal")
           ->FirstChildElement("Vertex")
  );
  return scenario;
}

template <typename Scalar>
std::string vertexKey(const Vertex<Scalar> &vertex) {
  std::ostringstream output;
  output << std::hexfloat << vertex.getX() << ',' << vertex.getY();
  if (vertex.hasTheta()) {
    output << ',' << vertex.getThetaLb() << ',' << vertex.getThetaUb()
           << ',' << vertex.getTheta();
  }
  return output.str();
}

template <typename Scalar>
std::set<std::string> canonicalVertices(const Graph<Scalar> &graph) {
  std::set<std::string> result;
  for (const auto &vertex : graph.getVertices()) {
    result.insert(vertexKey(*vertex));
  }
  return result;
}

template <typename Scalar>
std::set<std::string> canonicalEdges(const Graph<Scalar> &graph) {
  std::set<std::string> result;
  for (const auto &entry : graph.getAdjacencyList()) {
    const std::string source = vertexKey(*entry.first);
    for (const auto &neighbor : entry.second) {
      std::string first = source;
      std::string second = vertexKey(*neighbor);
      if (second < first) {
        std::swap(first, second);
      }
      result.insert(first + "|" + second);
    }
  }
  return result;
}

std::set<std::string> canonicalLayerEdges(const DebugEdgeLayers &layers) {
  std::set<std::string> result;
  for (std::size_t layer = 0; layer < layers.size(); ++layer) {
    for (const auto &edge : layers[layer]) {
      result.insert(
          std::to_string(layer) + ":" + vertexKey(*edge.first) +
          "->" + vertexKey(*edge.second)
      );
    }
  }
  return result;
}

template <typename Scalar>
std::set<std::string> canonicalLayerEdges(
    const std::vector<std::vector<std::pair<
        std::shared_ptr<Vertex<Scalar>>,
        std::shared_ptr<Vertex<Scalar>>>>> &layers
) {
  std::set<std::string> result;
  for (std::size_t layer = 0; layer < layers.size(); ++layer) {
    for (const auto &edge : layers[layer]) {
      result.insert(
          std::to_string(layer) + ":" + vertexKey(*edge.first) +
          "->" + vertexKey(*edge.second)
      );
    }
  }
  return result;
}

bool validateCpuFixedPoint(
    const Scenario &scenario,
    bool optimal,
    VisibilityGraph<T> &graph
) {
  const VertexPropagationStatistics &statistics =
      graph.getVertexPropagationStatistics();
  const std::size_t layerCount = graph.getLayers().size();
  if (!require(
          statistics.requestedBackend == VertexPropagationBackend::Cpu,
          "CPU oracle did not retain its requested backend"
      ) ||
      !require(
          statistics.effectiveBackend == VertexPropagationBackend::Cpu,
          "CPU oracle did not use the CPU backend"
      ) ||
      !require(statistics.candidatePairs > 0,
               "propagation did not evaluate any candidates") ||
      !require(statistics.totalMilliseconds >= statistics.cpuMilliseconds,
               "CPU phase timing exceeds total propagation timing")) {
    return false;
  }
  if (optimal) {
    return require(statistics.forwardRounds <= layerCount,
                   "forward propagation exceeded the layer hard cap") &&
        require(statistics.backwardRounds <= layerCount,
                "backward propagation exceeded the layer hard cap") &&
        require(statistics.forwardRounds > 0,
                "optimal forward propagation ran no rounds") &&
        require(statistics.backwardRounds > 0,
                "optimal backward propagation ran no rounds");
  }
  return require(statistics.forwardRounds == 1,
                 "non-optimal forward propagation must use one round") &&
      require(statistics.backwardRounds == 1,
              "non-optimal backward propagation must use one round");
}

bool compareCudaWithCpu(const Scenario &scenario, bool optimal) {
  VisibilityGraph<T> cpu(
      scenario.robot,
      scenario.border,
      scenario.obstacles,
      8,
      4,
      optimal,
      false,
      VertexPropagationBackend::Cpu
  );
  VisibilityGraph<T> cuda(
      scenario.robot,
      scenario.border,
      scenario.obstacles,
      8,
      4,
      optimal,
      false,
      VertexPropagationBackend::Cuda
  );
  if (!validateCpuFixedPoint(scenario, optimal, cpu)) {
    return false;
  }
  const VertexPropagationStatistics &cudaStatistics =
      cuda.getVertexPropagationStatistics();
  if (!require(
          cudaStatistics.effectiveBackend == VertexPropagationBackend::Cuda,
          "explicit CUDA propagation did not use CUDA"
      ) ||
      !require(
          cpu.getVertexPropagationStatistics().forwardRounds ==
              cudaStatistics.forwardRounds,
          "CPU/CUDA forward round counts differ"
      ) ||
      !require(
          cpu.getVertexPropagationStatistics().backwardRounds ==
              cudaStatistics.backwardRounds,
          "CPU/CUDA backward round counts differ"
      ) ||
      !require(
          canonicalLayerEdges(cpu.getEdgeLayersForward()) ==
              canonicalLayerEdges(cuda.getEdgeLayersForward()),
          "CPU/CUDA forward propagation edge records differ"
      ) ||
      !require(
          canonicalLayerEdges(cpu.getEdgeLayersBackward()) ==
              canonicalLayerEdges(cuda.getEdgeLayersBackward()),
          "CPU/CUDA backward propagation edge records differ"
      ) ||
      !require(canonicalVertices(cpu.getGraph()) ==
                   canonicalVertices(cuda.getGraph()),
               "CPU/CUDA canonical graph vertex sets differ") ||
      !require(canonicalEdges(cpu.getGraph()) == canonicalEdges(cuda.getGraph()),
               "CPU/CUDA canonical graph edge sets differ") ||
      !require(cpu.getGraph().numConnectedComponents() ==
                   cuda.getGraph().numConnectedComponents(),
               "CPU/CUDA connectivity differs")) {
    return false;
  }

  cpu.setWeight(1.0, 0.1);
  cuda.setWeight(1.0, 0.1);
  const auto cpuPath = cpu.shortestPath(
      std::make_shared<Vertex<T>>(*scenario.start),
      std::make_shared<Vertex<T>>(*scenario.goal),
      0
  );
  const auto cudaPath = cuda.shortestPath(
      std::make_shared<Vertex<T>>(*scenario.start),
      std::make_shared<Vertex<T>>(*scenario.goal),
      0
  );
  return require(!cpuPath.empty() && !cudaPath.empty(),
                 "CPU/CUDA query did not return a path") && require(
      std::abs(cpu.getPathLength() - cuda.getPathLength()) <= 1e-9,
      "CPU/CUDA shortest-path costs differ"
  );
}

template <typename Scalar>
bool compareCpuReferenceForScalar(
    const std::filesystem::path &path,
    bool optimal,
    VertexPropagationBackend backend = VertexPropagationBackend::Cpu
) {
  tinyxml2::XMLDocument document;
  if (!require(
          document.LoadFile(path.string().c_str()) == tinyxml2::XML_SUCCESS,
          "unable to load float propagation parity scenario"
      )) {
    return false;
  }
  Polygon<Scalar> robot = Utils::getRobot<Scalar>(document);
  std::vector<Polygon<Scalar>> obstacles =
      Utils::getObstacles<Scalar>(document);
  const auto *environment =
      document.RootElement()->FirstChildElement("environment");
  Polygon<Scalar> border = Utils::get<bool>(
      *environment, "useBoundary", false
  ) ? Utils::getBoundary<Scalar>(document)
    : Utils::getMap<Scalar>(Utils::get<int>(*environment, "mapSize", 100));

  VisibilityGraph<Scalar> filtered(
      robot, border, obstacles, 8, 4, optimal, false,
      backend
  );
  VisibilityGraph<Scalar> reference(
      robot, border, obstacles, 8, 4, optimal, false,
      VertexPropagationBackend::CpuReference
  );
  const VertexPropagationStatistics &filteredStatistics =
      filtered.getVertexPropagationStatistics();
  const VertexPropagationStatistics &referenceStatistics =
      reference.getVertexPropagationStatistics();
  return require(
             filteredStatistics.forwardRounds ==
                     referenceStatistics.forwardRounds &&
                 filteredStatistics.backwardRounds ==
                     referenceStatistics.backwardRounds,
             "float filtered/reference propagation rounds differ"
         ) &&
      require(
          canonicalLayerEdges(filtered.getEdgeLayersForward()) ==
              canonicalLayerEdges(reference.getEdgeLayersForward()),
          "float filtered/reference forward records differ"
      ) &&
      require(
          canonicalLayerEdges(filtered.getEdgeLayersBackward()) ==
              canonicalLayerEdges(reference.getEdgeLayersBackward()),
          "float filtered/reference backward records differ"
      ) &&
      require(
          canonicalVertices(filtered.getGraph()) ==
              canonicalVertices(reference.getGraph()),
          "float filtered/reference vertices differ"
      ) &&
      require(
          canonicalEdges(filtered.getGraph()) ==
              canonicalEdges(reference.getGraph()),
          "float filtered/reference edges differ"
      ) &&
      require(
          filtered.getGraph().numConnectedComponents() ==
              reference.getGraph().numConnectedComponents(),
          "float filtered/reference connectivity differs"
      );
}

#ifdef RVG_HAS_CUDA
detail::PropagationInterval interval(double lower, double upper) {
  return {lower, upper};
}

detail::PropagationIntervalPoint point(double x, double y) {
  return {interval(x, x), interval(y, y)};
}

bool testCudaCertifiedPredicates() {
  std::vector<detail::CudaSourceGeometry> sources(5);
  sources[0] = {point(0, 0), point(-1, 0), point(0, 1), 1};
  sources[1] = {point(2, 0), point(1, 0), point(2, 1), 1};
  sources[2] = {point(1, 0), point(0, 0), point(1, 1), 1};
  sources[3] = {
      {interval(0.999999, 1.000001), interval(0, 0)},
      point(0, 0),
      point(1, 1),
      1
  };
  sources[4] = {point(0, 0), point(-1, 0), point(1, 0), 0};

  detail::CudaTargetGeometry target;
  target.point = point(0.5, 0);
  target.neighborLeft = point(1, 0);
  target.neighborRight = point(0.5, -0.5);
  target.boundingX = interval(-1, 1);
  target.boundingY = interval(-1, 1);
  target.polygonOffset = 0;
  target.polygonCount = 4;
  target.onHole = 0;
  const std::vector<detail::PropagationIntervalPoint> polygonPoints = {
      point(-1, -1),
      point(1, -1),
      point(1, 1),
      point(-1, 1)
  };
  std::string error;
  detail::CudaPropagationContext *context =
      detail::createCudaPropagationContext(
          sources, {target}, polygonPoints, error
      );
  if (!require(context != nullptr,
               "failed to create CUDA predicate test context: " + error)) {
    return false;
  }
  std::vector<detail::CudaPropagationWorkItem> workItems;
  for (std::uint32_t source = 0; source < sources.size(); ++source) {
    workItems.push_back({source, 0, 1, source});
  }
  detail::CudaPropagationBatchResult result;
  const bool evaluated = detail::evaluateCudaPropagationBatch(
      context, workItems, static_cast<std::uint32_t>(sources.size()),
      result, error
  );
  detail::destroyCudaPropagationContext(context);
  if (!require(evaluated,
               "failed to evaluate CUDA predicate cases: " + error)) {
    return false;
  }
  return require(
             result.accepted == std::vector<std::uint32_t>{0},
             "CUDA predicate did not certify the strict interior candidate"
         ) &&
      require(
          result.uncertain == std::vector<std::uint32_t>({2, 3, 4}),
          "boundary, overlapping-interval, and collinear cases must be uncertain"
      );
}
#endif


}  // namespace

int main(int argc, char **argv) {
  const bool compareCuda = argc > 1 && std::string(argv[1]) == "--cuda";
  if (compareCuda && !cudaVertexPropagationAvailable()) {
    std::cout << "CUDA device unavailable; skipping propagation parity test\n";
    return 77;
  }
  const std::filesystem::path scenarioPath =
      std::filesystem::path(RVG_CONFIG_DIR) /
      "Baselines/mapfixedStartAndGoalVeryEasySmall0.xml";
  const Scenario scenario = loadScenario(scenarioPath);
  if (compareCuda) {
#ifdef RVG_HAS_CUDA
    return testCudaCertifiedPredicates() &&
            compareCudaWithCpu(scenario, false) &&
            compareCudaWithCpu(scenario, true) &&
            compareCpuReferenceForScalar<float>(
                scenarioPath, false, VertexPropagationBackend::Cuda) &&
            compareCpuReferenceForScalar<float>(
                scenarioPath, true, VertexPropagationBackend::Cuda)
        ? 0
        : 1;
#else
    return 77;
#endif
  }

  for (const bool optimal : {false, true}) {
    VisibilityGraph<T> graph(
        scenario.robot,
        scenario.border,
        scenario.obstacles,
        8,
        4,
        optimal,
        false,
        VertexPropagationBackend::Cpu
    );
    if (!validateCpuFixedPoint(scenario, optimal, graph)) {
      return 1;
    }
    VisibilityGraph<T> reference(
        scenario.robot,
        scenario.border,
        scenario.obstacles,
        8,
        4,
        optimal,
        false,
        VertexPropagationBackend::CpuReference
    );
    const VertexPropagationStatistics &referenceStatistics =
        reference.getVertexPropagationStatistics();
    if (!require(
            referenceStatistics.requestedBackend ==
                VertexPropagationBackend::CpuReference,
            "reference backend did not retain its request"
        ) ||
        !require(
            referenceStatistics.effectiveBackend ==
                VertexPropagationBackend::Cpu,
            "reference backend did not execute on CPU"
        ) ||
        !require(canonicalLayerEdges(graph.getEdgeLayersForward()) ==
                     canonicalLayerEdges(reference.getEdgeLayersForward()),
                 "filtered/reference forward edge records differ") ||
        !require(canonicalLayerEdges(graph.getEdgeLayersBackward()) ==
                     canonicalLayerEdges(reference.getEdgeLayersBackward()),
                 "filtered/reference backward edge records differ") ||
        !require(canonicalVertices(graph.getGraph()) ==
                     canonicalVertices(reference.getGraph()),
                 "filtered/reference vertices differ") ||
        !require(canonicalEdges(graph.getGraph()) ==
                     canonicalEdges(reference.getGraph()),
                 "filtered/reference graph edges differ") ||
        !require(
            graph.getVertexPropagationStatistics().forwardRounds ==
                referenceStatistics.forwardRounds &&
                graph.getVertexPropagationStatistics().backwardRounds ==
                referenceStatistics.backwardRounds,
            "filtered/reference propagation rounds differ"
        )) {
      return 1;
    }
  }
  // Exercise multi-threaded query insertion even on machines without CUDA.
  for (int repetition = 0; repetition < 12; ++repetition) {
    VisibilityGraph<T> graph(scenario.robot, scenario.border, scenario.obstacles,
                             8, 4, false, false, VertexPropagationBackend::Cpu);
    graph.setWeight(1.0, 0.1);
    if (!require(!graph.shortestPath(
                     std::make_shared<Vertex<T>>(*scenario.start),
                     std::make_shared<Vertex<T>>(*scenario.goal)).empty(),
                 "parallel query insertion lost a legal endpoint")) {
      return 1;
    }
  }
  if (!compareCpuReferenceForScalar<float>(scenarioPath, false) ||
      !compareCpuReferenceForScalar<float>(scenarioPath, true)) {
    return 1;
  }
  return 0;
}
