#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATION_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATION_H_

#include <cstddef>
#include <string>

namespace RotationalVisibilityGraph {

enum class VertexPropagationBackend { Auto, Cpu, CpuReference, Cuda };

inline const char *vertexPropagationBackendName(VertexPropagationBackend backend) {
  switch (backend) {
    case VertexPropagationBackend::Auto: return "auto";
    case VertexPropagationBackend::Cpu: return "cpu";
    case VertexPropagationBackend::CpuReference: return "cpu_reference";
    case VertexPropagationBackend::Cuda: return "cuda";
  }
  return "unknown";
}

struct VertexPropagationStatistics {
  VertexPropagationBackend requestedBackend = VertexPropagationBackend::Auto;
  VertexPropagationBackend effectiveBackend = VertexPropagationBackend::Cpu;
  std::size_t forwardRounds = 0;
  std::size_t backwardRounds = 0;
  std::size_t candidatePairs = 0;
  std::size_t cpuExactCandidates = 0;
  std::size_t cpuCertifiedAccepts = 0;
  std::size_t cpuCertifiedRejects = 0;
  std::size_t cpuUncertainCandidates = 0;
  std::size_t cudaCertifiedAccepts = 0;
  std::size_t cudaCertifiedRejects = 0;
  std::size_t cpuExactFallbacks = 0;
  double flattenMilliseconds = 0;
  double scalarFilterMilliseconds = 0;
  double cpuMilliseconds = 0;
  double transferMilliseconds = 0;
  double kernelMilliseconds = 0;
  double exactFallbackMilliseconds = 0;
  double materializationMilliseconds = 0;
  double totalMilliseconds = 0;
  bool cudaFallback = false;
  std::string cudaError;
};

bool cudaVertexPropagationAvailable();

}  // namespace RotationalVisibilityGraph

#endif  // ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATION_H_
