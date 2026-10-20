#include <Utils/Utils.h>
#include <VisibilityGraph/Utils.h>
#include <VisibilityGraph/VisibilityGraph.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace RotationalVisibilityGraph;

namespace {

using T = double;

struct Scenario {
  Polygon<T> robot;
  Polygon<T> border;
  std::vector<Polygon<T>> obstacles;
  std::shared_ptr<Vertex<T>> start;
  std::shared_ptr<Vertex<T>> goal;
};

struct Measurement {
  double buildMilliseconds = 0;
  double queryMilliseconds = 0;
  double buildAndQueryMilliseconds = 0;
  double propagationMilliseconds = 0;
  double layerBuildMilliseconds = 0;
  double nativeMaterializationMilliseconds = 0;
  double finalizationMilliseconds = 0;
  double flattenMilliseconds = 0;
  double scalarFilterMilliseconds = 0;
  double cpuMilliseconds = 0;
  double transferMilliseconds = 0;
  double kernelMilliseconds = 0;
  double exactFallbackMilliseconds = 0;
  double materializationMilliseconds = 0;
  std::size_t candidates = 0;
  std::size_t exactCandidates = 0;
  std::size_t exactFallbacks = 0;
  std::size_t certifiedAccepts = 0;
  std::size_t certifiedRejects = 0;
  std::size_t uncertainCandidates = 0;
  std::size_t vertices = 0;
  std::size_t edges = 0;
};

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
      *plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex")
  );
  scenario.goal = Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex")
  );
  return scenario;
}

std::size_t graphEdgeCount(const Graph<T> &graph) {
  return graph.edgeCount();
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 != 0) {
    return values[middle];
  }
  return (values[middle - 1] + values[middle]) / 2;
}

Measurement buildOnce(
    const Scenario &scenario,
    int resolution,
    int threads,
    VertexPropagationBackend backend
) {
  VisibilityGraph<T> graph(
      scenario.robot,
      scenario.border,
      scenario.obstacles,
      resolution,
      threads,
      true,
      false,
      backend
  );
  const VisibilityGraphConstructionStatistics &construction =
      graph.getConstructionStatistics();
  const VertexPropagationStatistics &statistics =
      graph.getVertexPropagationStatistics();
  Measurement measurement;
  measurement.buildMilliseconds = construction.totalMilliseconds;
  measurement.propagationMilliseconds = statistics.totalMilliseconds;
  measurement.layerBuildMilliseconds = construction.layerBuildMilliseconds;
  measurement.nativeMaterializationMilliseconds =
      construction.nativeMaterializationMilliseconds;
  measurement.finalizationMilliseconds =
      construction.finalizationMilliseconds;
  measurement.flattenMilliseconds = statistics.flattenMilliseconds;
  measurement.scalarFilterMilliseconds = statistics.scalarFilterMilliseconds;
  measurement.cpuMilliseconds = statistics.cpuMilliseconds;
  measurement.transferMilliseconds = statistics.transferMilliseconds;
  measurement.kernelMilliseconds = statistics.kernelMilliseconds;
  measurement.exactFallbackMilliseconds =
      statistics.exactFallbackMilliseconds;
  measurement.materializationMilliseconds =
      statistics.materializationMilliseconds;
  measurement.candidates = statistics.candidatePairs;
  measurement.exactCandidates = statistics.cpuExactCandidates;
  measurement.exactFallbacks = statistics.cpuExactFallbacks;
  measurement.certifiedAccepts = statistics.cpuCertifiedAccepts;
  measurement.certifiedRejects = statistics.cpuCertifiedRejects;
  measurement.uncertainCandidates = statistics.cpuUncertainCandidates;
  measurement.vertices = graph.getGraph().vertexCount();
  measurement.edges = graphEdgeCount(graph.getGraph());
  graph.setWeight(1.0, 0.1);
  graph.shortestPath(scenario.start, scenario.goal, 0);
  measurement.queryMilliseconds = graph.getSearchTime() * 1000.0;
  measurement.buildAndQueryMilliseconds =
      measurement.buildMilliseconds + measurement.queryMilliseconds;
  return measurement;
}

std::string mapName(int index) {
  std::ostringstream output;
  output << "hard_random_polygon_" << std::setw(2) << std::setfill('0')
         << index << ".xml";
  return output.str();
}

std::vector<int> parseResolutions(const std::string &value) {
  std::vector<int> result;
  std::istringstream input(value);
  std::string token;
  while (std::getline(input, token, ',')) {
    const int resolution = std::stoi(token);
    if (resolution <= 0) {
      throw std::invalid_argument("Resolutions must be positive");
    }
    result.push_back(resolution);
  }
  if (result.empty()) {
    throw std::invalid_argument("At least one resolution is required");
  }
  return result;
}

std::vector<VertexPropagationBackend> parseBackends(const std::string &value) {
  std::vector<VertexPropagationBackend> result;
  std::istringstream input(value);
  std::string token;
  while (std::getline(input, token, ',')) {
    if (token == "cpu_reference") {
      result.push_back(VertexPropagationBackend::CpuReference);
    } else if (token == "cpu") {
      result.push_back(VertexPropagationBackend::Cpu);
    } else if (token == "cuda") {
      result.push_back(VertexPropagationBackend::Cuda);
    } else {
      throw std::invalid_argument("Unknown propagation backend: " + token);
    }
  }
  if (result.empty()) {
    throw std::invalid_argument("At least one backend is required");
  }
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  const std::filesystem::path configDirectory = argc > 1
      ? std::filesystem::path(argv[1])
      : std::filesystem::path(RVG_HARD_BENCHMARK_CONFIG_DIR);
  const std::filesystem::path outputPath = argc > 2
      ? std::filesystem::path(argv[2])
      : std::filesystem::path("vertex_propagation_benchmark.csv");
  const int mapCount = argc > 3 ? std::stoi(argv[3]) : 20;
  const int measuredRuns = argc > 4 ? std::stoi(argv[4]) : 5;
  const int threads = argc > 5 ? std::stoi(argv[5]) : 24;
  const std::vector<int> resolutions = parseResolutions(
      argc > 6 ? argv[6] : "90,180,360"
  );
  const std::vector<VertexPropagationBackend> backends = parseBackends(
      argc > 7 ? argv[7] : "cpu_reference,cpu"
  );
  if (mapCount <= 0 || mapCount > 20 || measuredRuns <= 0 || threads <= 0) {
    throw std::invalid_argument(
        "Expected 1-20 maps, at least one measured run, and positive threads"
    );
  }
  const bool requestsCuda = std::find(
      backends.begin(), backends.end(), VertexPropagationBackend::Cuda
  ) != backends.end();
  if (requestsCuda && !cudaVertexPropagationAvailable()) {
    std::cerr << "A CUDA device is required for the propagation benchmark\n";
    return 77;
  }

  std::ofstream output(outputPath);
  if (!output) {
    throw std::runtime_error("Unable to open " + outputPath.string());
  }
  output << "map,resolution,backend,run,build_ms,query_ms,build_query_ms,"
            "layer_build_ms,native_materialization_ms,propagation_ms,"
            "flatten_ms,scalar_filter_ms,cpu_ms,transfer_ms,kernel_ms,"
            "exact_fallback_ms,propagation_materialization_ms,finalization_ms,"
            "candidates,exact_candidates,exact_fallbacks,certified_accepts,"
            "certified_rejects,uncertain_candidates,vertices,edges\n";

  std::vector<double> cpuSpeedups;
  std::vector<double> cudaPropagationSpeedups;
  for (int mapIndex = 0; mapIndex < mapCount; ++mapIndex) {
    const std::string name = mapName(mapIndex);
    const Scenario scenario = loadScenario(configDirectory / name);
    for (const int resolution : resolutions) {
      std::map<VertexPropagationBackend, std::vector<double>> totals;
      std::map<VertexPropagationBackend, std::vector<double>> propagationTotals;
      std::map<VertexPropagationBackend, std::tuple<
          std::size_t, std::size_t, std::size_t
      >> topology;
      for (const VertexPropagationBackend backend : backends) {
        buildOnce(scenario, resolution, threads, backend);
        for (int run = 0; run < measuredRuns; ++run) {
          const Measurement measurement =
              buildOnce(scenario, resolution, threads, backend);
          totals[backend].push_back(measurement.buildAndQueryMilliseconds);
          propagationTotals[backend].push_back(
              measurement.propagationMilliseconds
          );
          const auto graphSignature = std::make_tuple(
              measurement.vertices,
              measurement.edges,
              measurement.candidates
          );
          const auto insertion = topology.emplace(backend, graphSignature);
          if (!insertion.second && insertion.first->second != graphSignature) {
            throw std::runtime_error(
                "Propagation graph changed between measured runs"
            );
          }
          output << name << ',' << resolution << ','
                 << vertexPropagationBackendName(backend) << ',' << run << ','
                 << std::setprecision(17)
                 << measurement.buildMilliseconds << ','
                 << measurement.queryMilliseconds << ','
                 << measurement.buildAndQueryMilliseconds << ','
                 << measurement.layerBuildMilliseconds << ','
                 << measurement.nativeMaterializationMilliseconds << ','
                 << measurement.propagationMilliseconds << ','
                 << measurement.flattenMilliseconds << ','
                 << measurement.scalarFilterMilliseconds << ','
                 << measurement.cpuMilliseconds << ','
                 << measurement.transferMilliseconds << ','
                 << measurement.kernelMilliseconds << ','
                 << measurement.exactFallbackMilliseconds << ','
                 << measurement.materializationMilliseconds << ','
                 << measurement.finalizationMilliseconds << ','
                 << measurement.candidates << ','
                 << measurement.exactCandidates << ','
                 << measurement.exactFallbacks << ','
                 << measurement.certifiedAccepts << ','
                 << measurement.certifiedRejects << ','
                 << measurement.uncertainCandidates << ','
                 << measurement.vertices << ',' << measurement.edges << '\n';
        }
      }
      const auto expectedTopology = topology.at(backends.front());
      for (const VertexPropagationBackend backend : backends) {
        if (topology.at(backend) != expectedTopology) {
          throw std::runtime_error(
              "Propagation graph sizes or candidate counts differ by backend"
          );
        }
        std::cout << name << " resolution=" << resolution << ' '
                  << vertexPropagationBackendName(backend) << " median="
                  << median(totals.at(backend)) << " ms\n";
      }
      const auto reference = totals.find(VertexPropagationBackend::CpuReference);
      const auto cpu = totals.find(VertexPropagationBackend::Cpu);
      if (reference != totals.end() && cpu != totals.end()) {
        const double speedup = median(reference->second) / median(cpu->second);
        cpuSpeedups.push_back(speedup);
        std::cout << "  CPU build+query speedup=" << speedup << "x\n";
      }
      const auto cpuPropagation =
          propagationTotals.find(VertexPropagationBackend::Cpu);
      const auto cudaPropagation =
          propagationTotals.find(VertexPropagationBackend::Cuda);
      if (cpuPropagation != propagationTotals.end() &&
          cudaPropagation != propagationTotals.end()) {
        const double speedup = median(cpuPropagation->second) /
                               median(cudaPropagation->second);
        cudaPropagationSpeedups.push_back(speedup);
        std::cout << "  CUDA propagation speedup=" << speedup << "x\n";
      }
    }
  }

  if (!cpuSpeedups.empty()) {
    const double suiteMedianSpeedup = median(cpuSpeedups);
    const bool lowResolutionGuard = std::all_of(
        resolutions.begin(), resolutions.end(),
        [](int resolution) { return resolution <= 36; }
    );
    std::cout << "Suite median CPU/reference build+query speedup: "
              << suiteMedianSpeedup << "x\n";
    if (lowResolutionGuard && suiteMedianSpeedup < 1.0 / 1.1) {
      std::cerr << "Low-resolution guard failed: slowdown exceeded 10%\n";
      return 2;
    }
    if (!lowResolutionGuard) {
      std::cout << "The 1.5x acceptance comparison requires the clean "
                   "pre-change end-to-end baseline, not CpuReference.\n";
    }
  }
  if (!cudaPropagationSpeedups.empty()) {
    std::cout << "Suite median CUDA propagation speedup: "
              << median(cudaPropagationSpeedups) << "x\n";
  }
  std::cout << "CSV: " << outputPath << '\n';
  return 0;
}
