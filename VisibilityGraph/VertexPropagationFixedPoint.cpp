#include <VisibilityGraph/VisibilityGraph.h>

#include <VisibilityGraph/Utils.h>
#include <VisibilityGraph/VertexPropagationCuda.h>
#include <VisibilityGraph/VertexPropagationInterval.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace RotationalVisibilityGraph {
namespace {

constexpr std::size_t CUDA_PAIR_THRESHOLD = 65536;

template <typename T>
struct ExactPropagationSource {
  using Point_2 = typename VisibilityGraph<T>::Point_2;
  std::uint32_t originLayer = 0;
  std::uint32_t pointIndex = 0;
  Point_2 point;
  Point_2 neighborLeft;
  Point_2 neighborRight;
  bool onHole = false;
};

template <typename T>
struct ExactPropagationTarget {
  using Point_2 = typename VisibilityGraph<T>::Point_2;
  using Polygon_2 = typename VisibilityGraph<T>::Polygon_2;
  std::uint32_t nativeSource = 0;
  Vertex<T> vertex;
  Point_2 neighborLeft;
  Point_2 neighborRight;
  Polygon_2 visibleArea;
  bool onHole = false;
};

struct PropagationFrontierItem {
  std::uint32_t source = 0;
  std::uint32_t layer = 0;
};

struct PropagationWorkItem {
  std::uint32_t source = 0;
  std::uint32_t fromLayer = 0;
  std::uint32_t toLayer = 0;
  std::int8_t direction = 0;
  std::uint32_t targetOffset = 0;
  std::uint32_t targetCount = 0;
  std::uint32_t candidateOffset = 0;
};

struct PropagationCandidateRef {
  std::uint32_t source = 0;
  std::uint32_t target = 0;
  std::uint32_t workItem = 0;
};

struct PropagationPairRecord {
  std::uint32_t source = 0;
  std::uint32_t target = 0;
  std::uint32_t fromLayer = 0;
  std::uint32_t toLayer = 0;
  std::int8_t direction = 0;
};

template <typename T>
bool exactPropagationCandidateAccepted(
    const ExactPropagationSource<T> &source,
    const ExactPropagationTarget<T> &target
) {
  if (!source.onHole && !target.onHole &&
      !Utils::isBitangent<T>(
          source.point,
          source.neighborLeft,
          source.neighborRight,
          target.vertex.getPoint(),
          target.neighborLeft,
          target.neighborRight
      )) {
    return false;
  }
  if (target.visibleArea.is_empty()) {
    return false;
  }
  return target.visibleArea.bounded_side(source.point) !=
      CGAL::ON_UNBOUNDED_SIDE;
}

template <typename Point>
detail::PropagationIntervalPoint intervalPoint(const Point &point) {
  const auto x = CGAL::to_interval(point.x());
  const auto y = CGAL::to_interval(point.y());
  return {{x.first, x.second}, {y.first, y.second}};
}

template <typename T>
std::shared_ptr<Vertex<T>> makePropagationVertex(
    const ExactPropagationSource<T> &source,
    const Layer<T> &layer
) {
  const T thetaLower = layer.getThetaLb();
  const T thetaUpper = layer.getThetaUb();
  return std::make_shared<Vertex<T>>(
      CGAL::to_double(source.point.x()),
      CGAL::to_double(source.point.y()),
      thetaLower,
      thetaUpper,
      (thetaLower + thetaUpper) / static_cast<T>(2)
  );
}

}  // namespace

template <typename T>
void VisibilityGraph<T>::_connectLayersFixedPoint() {
  using Clock = std::chrono::steady_clock;
  const auto propagationStart = Clock::now();
  _propagationStatistics = VertexPropagationStatistics();
  _propagationStatistics.requestedBackend = _propagationBackend;
  _propagationStatistics.effectiveBackend = VertexPropagationBackend::Cpu;

  const std::uint32_t layerCount =
      static_cast<std::uint32_t>(_layers.size());
  _edgeLayersForward.assign(layerCount, {});
  _edgeLayersBackward.assign(layerCount, {});
  if (layerCount == 0) {
    return;
  }

#ifndef RVG_HAS_CUDA
  if (_propagationBackend == VertexPropagationBackend::Cuda) {
    throw std::runtime_error(
        "CUDA vertex propagation was requested, but RVG was built without CUDA"
    );
  }
#endif

  const auto flattenStart = Clock::now();
  std::vector<ExactPropagationSource<T>> sources;
  std::vector<std::vector<std::uint32_t>> nativeSources(layerCount);
  for (std::uint32_t layerId = 0; layerId < layerCount; ++layerId) {
    const Layer<T> &layer = _layers[layerId];
    nativeSources[layerId].reserve(layer.getPoints().size());
    for (std::uint32_t pointIndex = 0;
         pointIndex < layer.getPoints().size(); ++pointIndex) {
      const Vertex<T> &vertex = layer.getVertex(pointIndex);
      const auto neighbors = layer.getNeighbor(vertex);
      ExactPropagationSource<T> source;
      source.originLayer = layerId;
      source.pointIndex = pointIndex;
      source.point = layer.getPoint(pointIndex);
      source.neighborLeft = neighbors.first;
      source.neighborRight = neighbors.second;
      source.onHole = layer.isVertexOnHole(vertex);
      nativeSources[layerId].push_back(
          static_cast<std::uint32_t>(sources.size())
      );
      sources.push_back(std::move(source));
    }
  }

  std::vector<ExactPropagationTarget<T>> targets;
  std::vector<std::vector<std::uint32_t>> layerTargets(layerCount);
  for (std::uint32_t layerId = 0; layerId < layerCount; ++layerId) {
    const Layer<T> &layer = _layers[layerId];
    auto cache = layer.getVisibleAreaCache();
    std::unordered_map<Vertex<T>, std::uint32_t, typename Vertex<T>::Hash>
        nativeSourceByVertex;
    nativeSourceByVertex.reserve(nativeSources[layerId].size());
    for (const std::uint32_t sourceId : nativeSources[layerId]) {
      nativeSourceByVertex.emplace(
          layer.getVertex(sources[sourceId].pointIndex), sourceId
      );
    }
    std::vector<ExactPropagationTarget<T>> localTargets;
    localTargets.reserve(cache.size());
    for (const auto &entry : cache) {
      ExactPropagationTarget<T> target;
      target.vertex = entry.first;
      const auto nativeSource = nativeSourceByVertex.find(target.vertex);
      if (nativeSource == nativeSourceByVertex.end()) {
        throw std::runtime_error(
            "A visibility target does not have a native source vertex"
        );
      }
      target.nativeSource = nativeSource->second;
      const auto neighbors = layer.getNeighbor(entry.first);
      target.neighborLeft = neighbors.first;
      target.neighborRight = neighbors.second;
      target.visibleArea = entry.second;
      target.onHole = layer.isVertexOnHole(entry.first);
      localTargets.push_back(std::move(target));
    }
    std::sort(
        localTargets.begin(),
        localTargets.end(),
        [](const auto &left, const auto &right) {
          return std::tie(
                     left.vertex.getX(),
                     left.vertex.getY(),
                     left.vertex.getTheta()
                 ) <
              std::tie(
                     right.vertex.getX(),
                     right.vertex.getY(),
                     right.vertex.getTheta()
                 );
        }
    );
    layerTargets[layerId].reserve(localTargets.size());
    for (auto &target : localTargets) {
      layerTargets[layerId].push_back(
          static_cast<std::uint32_t>(targets.size())
      );
      targets.push_back(std::move(target));
    }
  }
  _propagationStatistics.flattenMilliseconds +=
      std::chrono::duration<double, std::milli>(Clock::now() - flattenStart)
          .count();

  const auto intervalStart = Clock::now();
  std::vector<detail::PropagationSourceGeometry> intervalSources;
  intervalSources.reserve(sources.size());
  for (const auto &source : sources) {
    detail::PropagationSourceGeometry geometry;
    geometry.point = intervalPoint(source.point);
    geometry.neighborLeft = intervalPoint(source.neighborLeft);
    geometry.neighborRight = intervalPoint(source.neighborRight);
    geometry.onHole = source.onHole ? 1 : 0;
    intervalSources.push_back(geometry);
  }
  std::vector<detail::PropagationTargetGeometry> intervalTargets;
  std::vector<detail::PropagationIntervalPoint> intervalPolygonPoints;
  intervalTargets.reserve(targets.size());
  for (const auto &target : targets) {
    detail::PropagationTargetGeometry geometry;
    geometry.point = intervalPoint(target.vertex.getPoint());
    geometry.neighborLeft = intervalPoint(target.neighborLeft);
    geometry.neighborRight = intervalPoint(target.neighborRight);
    geometry.polygonOffset = static_cast<std::uint32_t>(
        intervalPolygonPoints.size()
    );
    geometry.polygonCount =
        static_cast<std::uint32_t>(target.visibleArea.size());
    geometry.onHole = target.onHole ? 1 : 0;
    geometry.boundingX = {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    };
    geometry.boundingY = geometry.boundingX;
    for (const auto &point : target.visibleArea.vertices()) {
      const auto interval = intervalPoint(point);
      geometry.boundingX.lower =
          std::min(geometry.boundingX.lower, interval.x.lower);
      geometry.boundingX.upper =
          std::max(geometry.boundingX.upper, interval.x.upper);
      geometry.boundingY.lower =
          std::min(geometry.boundingY.lower, interval.y.lower);
      geometry.boundingY.upper =
          std::max(geometry.boundingY.upper, interval.y.upper);
      intervalPolygonPoints.push_back(interval);
    }
    intervalTargets.push_back(geometry);
  }
  _propagationStatistics.flattenMilliseconds +=
      std::chrono::duration<double, std::milli>(Clock::now() - intervalStart)
          .count();
#ifdef RVG_HAS_CUDA
  using CudaContextPointer = std::unique_ptr<
      detail::CudaPropagationContext,
      void (*)(detail::CudaPropagationContext *)
  >;
  CudaContextPointer cudaContext(nullptr, detail::destroyCudaPropagationContext);
  bool cudaDisabled = false;

  const auto initializeCuda = [&]() -> bool {
    if (cudaContext || cudaDisabled) {
      return static_cast<bool>(cudaContext);
    }
    std::string error;
    if (!detail::cudaPropagationRuntimeAvailable(error)) {
      _propagationStatistics.cudaError = error;
      cudaDisabled = true;
      return false;
    }
    const auto transferStart = Clock::now();
    cudaContext.reset(detail::createCudaPropagationContext(
        intervalSources,
        intervalTargets,
        intervalPolygonPoints,
        error
    ));
    _propagationStatistics.transferMilliseconds +=
        std::chrono::duration<double, std::milli>(Clock::now() - transferStart)
            .count();
    if (!cudaContext) {
      _propagationStatistics.cudaError = error;
      cudaDisabled = true;
      return false;
    }
    return true;
  };
#endif

  std::vector<PropagationPairRecord> pairRecords;
  const int threadCount = std::max(1, _numThreads);
  std::vector<std::vector<PropagationCandidateRef>> acceptedByThread(threadCount);
  std::vector<std::vector<PropagationCandidateRef>> exactBatchByThread(
      threadCount
  );
  for (auto &batch : exactBatchByThread) {
    batch.reserve(4096);
  }
  std::vector<double> exactFallbackByThread(threadCount, 0);
  std::vector<PropagationWorkItem> roundWorkItems;
  std::vector<PropagationCandidateRef> roundAcceptedCandidates;
  std::vector<std::uint8_t> roundWorkAccepted;

  const auto evaluateWorkItems = [&](
      const std::vector<PropagationWorkItem> &workItems,
      std::uint32_t candidateCount,
      std::vector<PropagationCandidateRef> &acceptedCandidates
  ) {
    acceptedCandidates.clear();
    if (candidateCount == 0) {
      return;
    }
    _propagationStatistics.candidatePairs += candidateCount;
    bool useCuda = _propagationBackend == VertexPropagationBackend::Cuda;
    if (_propagationBackend == VertexPropagationBackend::Auto) {
      useCuda = candidateCount >= CUDA_PAIR_THRESHOLD;
    }

    const auto appendReferences = [&](
        const std::vector<std::uint32_t> &indices,
        std::vector<PropagationCandidateRef> &destination
    ) {
      std::size_t workIndex = 0;
      destination.reserve(destination.size() + indices.size());
      for (const std::uint32_t index : indices) {
        while (workIndex < workItems.size() &&
               index >= workItems[workIndex].candidateOffset +
                   workItems[workIndex].targetCount) {
          ++workIndex;
        }
        if (workIndex == workItems.size()) {
          throw std::runtime_error(
              "CUDA returned an out-of-range propagation candidate"
          );
        }
        const PropagationWorkItem &work = workItems[workIndex];
        destination.push_back({
            work.source,
            work.targetOffset + index - work.candidateOffset,
            static_cast<std::uint32_t>(workIndex)
        });
      }
    };

#ifdef RVG_HAS_CUDA
    if (useCuda && initializeCuda()) {
      std::vector<detail::CudaPropagationWorkItem> cudaWorkItems;
      cudaWorkItems.reserve(workItems.size());
      for (const PropagationWorkItem &work : workItems) {
        cudaWorkItems.push_back({
            work.source,
            work.targetOffset,
            work.targetCount,
            work.candidateOffset
        });
      }
      detail::CudaPropagationBatchResult cudaResult;
      std::string error;
      if (detail::evaluateCudaPropagationBatch(
              cudaContext.get(),
              cudaWorkItems,
              candidateCount,
              cudaResult,
              error
          )) {
        _propagationStatistics.effectiveBackend =
            VertexPropagationBackend::Cuda;
        _propagationStatistics.transferMilliseconds +=
            cudaResult.transferMilliseconds;
        _propagationStatistics.kernelMilliseconds +=
            cudaResult.kernelMilliseconds;
        _propagationStatistics.cudaCertifiedAccepts +=
            cudaResult.accepted.size();
        _propagationStatistics.cpuExactFallbacks +=
            cudaResult.uncertain.size();
        _propagationStatistics.cpuExactCandidates +=
            cudaResult.uncertain.size();
        _propagationStatistics.cudaCertifiedRejects +=
            candidateCount - cudaResult.accepted.size() -
            cudaResult.uncertain.size();
        appendReferences(cudaResult.accepted, acceptedCandidates);

        std::vector<PropagationCandidateRef> uncertainCandidates;
        appendReferences(cudaResult.uncertain, uncertainCandidates);
        std::vector<std::uint8_t> uncertainAccepted(
            uncertainCandidates.size(), 0
        );
        const auto fallbackStart = Clock::now();
#pragma omp parallel for num_threads(threadCount) schedule(static)
        for (std::size_t uncertainIndex = 0;
             uncertainIndex < uncertainCandidates.size(); ++uncertainIndex) {
          const PropagationCandidateRef &candidate =
              uncertainCandidates[uncertainIndex];
          uncertainAccepted[uncertainIndex] =
              exactPropagationCandidateAccepted<T>(
                  sources[candidate.source], targets[candidate.target]
              ) ? 1 : 0;
        }
        for (std::size_t uncertainIndex = 0;
             uncertainIndex < uncertainCandidates.size(); ++uncertainIndex) {
          if (uncertainAccepted[uncertainIndex]) {
            acceptedCandidates.push_back(
                uncertainCandidates[uncertainIndex]
            );
          }
        }
        _propagationStatistics.exactFallbackMilliseconds +=
            std::chrono::duration<double, std::milli>(
                Clock::now() - fallbackStart
            ).count();
        return;
      }
      _propagationStatistics.cudaError = error;
      cudaContext.reset();
      cudaDisabled = true;
      if (_propagationBackend == VertexPropagationBackend::Cuda) {
        throw std::runtime_error(
            "CUDA vertex propagation failed: " + error
        );
      }
      _propagationStatistics.cudaFallback = true;
      _propagationStatistics.effectiveBackend = VertexPropagationBackend::Cpu;
    } else if (useCuda) {
      if (_propagationBackend == VertexPropagationBackend::Cuda) {
        throw std::runtime_error(
            "CUDA vertex propagation is unavailable: " +
            _propagationStatistics.cudaError
        );
      }
      _propagationStatistics.cudaFallback = true;
      _propagationStatistics.effectiveBackend = VertexPropagationBackend::Cpu;
    }
#else
    (void)useCuda;
    (void)appendReferences;
#endif

    const auto cpuStart = Clock::now();
    const bool referenceBackend =
        _propagationBackend == VertexPropagationBackend::CpuReference;
    for (auto &local : acceptedByThread) {
      local.clear();
    }
    std::fill(exactFallbackByThread.begin(), exactFallbackByThread.end(), 0);
    std::size_t certifiedAccepts = 0;
    std::size_t certifiedRejects = 0;
    std::size_t uncertainCount = 0;
    std::size_t exactCandidates = 0;
#pragma omp parallel num_threads(threadCount) reduction(+:certifiedAccepts,certifiedRejects,uncertainCount,exactCandidates)
    {
      constexpr std::size_t ExactBatchSize = 4096;
      const int threadId = omp_get_thread_num();
      std::vector<PropagationCandidateRef> &local = acceptedByThread[threadId];
      std::vector<PropagationCandidateRef> &exactBatch =
          exactBatchByThread[threadId];
      exactBatch.clear();
      const auto flushExactBatch = [&]() {
        if (exactBatch.empty()) {
          return;
        }
        const auto fallbackStart = Clock::now();
        for (const PropagationCandidateRef &candidate : exactBatch) {
          if (exactPropagationCandidateAccepted<T>(
                  sources[candidate.source], targets[candidate.target]
              )) {
            local.push_back(candidate);
          }
        }
        exactFallbackByThread[threadId] +=
            std::chrono::duration<double, std::milli>(
                Clock::now() - fallbackStart
            ).count();
        exactBatch.clear();
      };
#pragma omp for schedule(static)
      for (std::size_t workIndex = 0;
           workIndex < workItems.size(); ++workIndex) {
        const PropagationWorkItem &work = workItems[workIndex];
        for (std::uint32_t localTarget = 0;
             localTarget < work.targetCount; ++localTarget) {
          const std::uint32_t target = work.targetOffset + localTarget;
          const PropagationCandidateRef candidate{
              work.source,
              target,
              static_cast<std::uint32_t>(workIndex)
          };
          if (referenceBackend) {
            if (exactPropagationCandidateAccepted<T>(
                sources[work.source], targets[target]
            )) {
              local.push_back(candidate);
            }
            ++exactCandidates;
            continue;
          }
          const detail::ScalarCandidateClassification classification =
              detail::classifyScalarCandidateAabb(
                  intervalSources[work.source],
                  intervalTargets[target]
              );
          if (classification ==
              detail::ScalarCandidateClassification::Accept) {
            local.push_back(candidate);
            ++certifiedAccepts;
          } else if (classification ==
                     detail::ScalarCandidateClassification::Reject) {
            ++certifiedRejects;
          } else {
            exactBatch.push_back(candidate);
            if (exactBatch.size() == ExactBatchSize) {
              flushExactBatch();
            }
            ++uncertainCount;
            ++exactCandidates;
          }
        }
      }
      flushExactBatch();
    }
    const double classificationElapsed =
        std::chrono::duration<double, std::milli>(Clock::now() - cpuStart)
            .count();
    const double exactFallbackElapsed = referenceBackend
        ? 0
        : *std::max_element(
              exactFallbackByThread.begin(), exactFallbackByThread.end()
          );
    if (!referenceBackend) {
      _propagationStatistics.scalarFilterMilliseconds +=
          std::max(0.0, classificationElapsed - exactFallbackElapsed);
      _propagationStatistics.exactFallbackMilliseconds +=
          exactFallbackElapsed;
    }
    _propagationStatistics.cpuExactFallbacks += uncertainCount;
    _propagationStatistics.cpuCertifiedAccepts += certifiedAccepts;
    _propagationStatistics.cpuCertifiedRejects += certifiedRejects;
    _propagationStatistics.cpuUncertainCandidates += uncertainCount;
    _propagationStatistics.cpuExactCandidates += exactCandidates;

    std::size_t acceptedCount = 0;
    for (const auto &local : acceptedByThread) {
      acceptedCount += local.size();
    }
    acceptedCandidates.reserve(acceptedCount);
    for (auto &local : acceptedByThread) {
      acceptedCandidates.insert(
          acceptedCandidates.end(),
          local.begin(), local.end()
      );
    }
    const double cpuElapsed = std::chrono::duration<double, std::milli>(
        Clock::now() - cpuStart
    ).count();
    _propagationStatistics.cpuMilliseconds += cpuElapsed;
  };

  const auto processRound = [&](
      const std::vector<PropagationFrontierItem> &frontier,
      int requestedDirection,
      bool propagate,
      std::vector<std::uint8_t> &membership
  ) {
    std::vector<PropagationWorkItem> &workItems = roundWorkItems;
    workItems.clear();
    workItems.reserve(
        frontier.size() * (requestedDirection == 0 ? 2 : 1)
    );
    std::uint64_t candidateCount = 0;
    const auto appendDirection = [&](
        const PropagationFrontierItem &frontierItem,
        int direction
    ) {
      const std::uint32_t toLayer = direction > 0
          ? (frontierItem.layer + 1) % layerCount
          : (frontierItem.layer + layerCount - 1) % layerCount;
      const auto &destinationTargets = layerTargets[toLayer];
      if (candidateCount + destinationTargets.size() >
          std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error(
            "A propagation round exceeded the 32-bit candidate limit"
        );
      }
      workItems.push_back({
          frontierItem.source,
          frontierItem.layer,
          toLayer,
          static_cast<std::int8_t>(direction),
          destinationTargets.empty() ? 0 : destinationTargets.front(),
          static_cast<std::uint32_t>(destinationTargets.size()),
          static_cast<std::uint32_t>(candidateCount)
      });
      candidateCount += destinationTargets.size();
    };
    for (const auto &frontierItem : frontier) {
      if (requestedDirection >= 0) {
        appendDirection(frontierItem, 1);
      }
      if (requestedDirection <= 0) {
        appendDirection(frontierItem, -1);
      }
    }

    std::vector<PropagationCandidateRef> &acceptedCandidates =
        roundAcceptedCandidates;
    evaluateWorkItems(
        workItems,
        static_cast<std::uint32_t>(candidateCount),
        acceptedCandidates
    );
    roundWorkAccepted.assign(workItems.size(), 0);
    for (const PropagationCandidateRef &candidate : acceptedCandidates) {
      const auto &work = workItems[candidate.workItem];
      roundWorkAccepted[candidate.workItem] = 1;
      pairRecords.push_back({
          candidate.source,
          candidate.target,
          work.fromLayer,
          work.toLayer,
          work.direction
      });
    }

    std::vector<PropagationFrontierItem> nextFrontier;
    if (!propagate) {
      return nextFrontier;
    }
    nextFrontier.reserve(workItems.size());
    for (std::size_t workIndex = 0; workIndex < workItems.size(); ++workIndex) {
      if (!roundWorkAccepted[workIndex]) {
        continue;
      }
      const auto &work = workItems[workIndex];
      const std::size_t membershipIndex =
          static_cast<std::size_t>(work.source) * layerCount + work.toLayer;
      if (membership[membershipIndex]) {
        continue;
      }
      membership[membershipIndex] = 1;
      const auto &source = sources[work.source];
      _layerVertices[work.toLayer].insert({
          static_cast<T>(source.originLayer),
          static_cast<T>(source.pointIndex)
      });
      nextFrontier.push_back({work.source, work.toLayer});
    }
    return nextFrontier;
  };

  std::vector<PropagationFrontierItem> initialFrontier;
  initialFrontier.reserve(sources.size());
  for (std::uint32_t sourceId = 0; sourceId < sources.size(); ++sourceId) {
    initialFrontier.push_back({sourceId, sources[sourceId].originLayer});
  }

  if (!_optimal) {
    std::vector<std::uint8_t> unusedMembership;
    processRound(initialFrontier, 0, false, unusedMembership);
    _propagationStatistics.forwardRounds = 1;
    _propagationStatistics.backwardRounds = 1;
  } else {
    const auto runClosure = [&](int direction, std::size_t &roundCounter) {
      std::vector<std::uint8_t> membership(
          static_cast<std::size_t>(sources.size()) * layerCount, 0
      );
      for (std::uint32_t sourceId = 0; sourceId < sources.size(); ++sourceId) {
        membership[
            static_cast<std::size_t>(sourceId) * layerCount +
            sources[sourceId].originLayer
        ] = 1;
      }
      std::vector<PropagationFrontierItem> frontier = initialFrontier;
      for (std::uint32_t round = 0;
           round < layerCount && !frontier.empty(); ++round) {
        ++roundCounter;
        frontier = processRound(frontier, direction, true, membership);
      }
      if (!frontier.empty()) {
        throw std::runtime_error(
            "Vertex propagation failed to converge within the layer count"
        );
      }
    };
    runClosure(1, _propagationStatistics.forwardRounds);
    runClosure(-1, _propagationStatistics.backwardRounds);
  }

  const auto materializationStart = Clock::now();
  std::sort(
      pairRecords.begin(),
      pairRecords.end(),
      [](const auto &left, const auto &right) {
        return std::tie(
                   left.direction,
                   left.fromLayer,
                   left.toLayer,
                   left.source,
                   left.target
               ) <
            std::tie(
                   right.direction,
                   right.fromLayer,
                   right.toLayer,
                   right.source,
                   right.target
               );
      }
  );
  pairRecords.erase(
      std::unique(
          pairRecords.begin(),
          pairRecords.end(),
          [](const auto &left, const auto &right) {
            return left.direction == right.direction &&
                left.fromLayer == right.fromLayer &&
                left.toLayer == right.toLayer &&
                left.source == right.source &&
                left.target == right.target;
          }
      ),
      pairRecords.end()
  );

  std::unordered_map<Vertex<T>, std::uint32_t, typename Vertex<T>::Hash>
      sourceGeometryLookup;
  sourceGeometryLookup.reserve(sources.size());
  std::vector<std::uint32_t> sourceGeometryIds(sources.size());
  std::vector<std::uint32_t> sourceGeometryRepresentatives;
  sourceGeometryRepresentatives.reserve(sources.size());
  for (std::uint32_t sourceId = 0; sourceId < sources.size(); ++sourceId) {
    const Vertex<T> position(
        CGAL::to_double(sources[sourceId].point.x()),
        CGAL::to_double(sources[sourceId].point.y())
    );
    const auto insertion = sourceGeometryLookup.emplace(
        position, static_cast<std::uint32_t>(sourceGeometryLookup.size())
    );
    sourceGeometryIds[sourceId] = insertion.first->second;
    if (insertion.second) {
      sourceGeometryRepresentatives.push_back(sourceId);
    }
  }

  const std::size_t propagationStateCount =
      sourceGeometryRepresentatives.size() * layerCount;
  const std::uint32_t missingVertexId =
      std::numeric_limits<std::uint32_t>::max();
  std::vector<std::uint32_t> propagatedVertexIds(
      propagationStateCount, missingVertexId
  );
  std::vector<std::shared_ptr<Vertex<T>>> graphVertices;
  graphVertices.reserve(std::min(
      propagationStateCount, pairRecords.size() * static_cast<std::size_t>(2)
  ));
  const auto propagatedVertexId = [&](
      std::uint32_t sourceId,
      std::uint32_t layerId
  ) -> std::uint32_t {
    const std::uint32_t geometryId = sourceGeometryIds[sourceId];
    const std::size_t stateIndex =
        static_cast<std::size_t>(geometryId) * layerCount + layerId;
    std::uint32_t &vertexId = propagatedVertexIds[stateIndex];
    if (vertexId != missingVertexId) {
      return vertexId;
    }
    if (graphVertices.size() >= missingVertexId) {
      throw std::runtime_error(
          "Vertex propagation exceeded the 32-bit materialization limit"
      );
    }
    vertexId = static_cast<std::uint32_t>(graphVertices.size());
    graphVertices.push_back(
        makePropagationVertex<T>(
            sources[sourceGeometryRepresentatives[geometryId]],
            _layers[layerId]
        )
    );
    return vertexId;
  };

  std::vector<std::pair<std::uint32_t, std::uint32_t>> indexedEdges;
  indexedEdges.reserve(pairRecords.size() * 2);
  std::tuple<std::int8_t, std::uint32_t, std::uint32_t, std::uint32_t>
      previousTransition{
          std::numeric_limits<std::int8_t>::min(), 0, 0, 0
      };
  for (const auto &record : pairRecords) {
    const auto transition = std::make_tuple(
        record.direction,
        record.fromLayer,
        record.toLayer,
        record.source
    );
    const std::uint32_t fromVertexId =
        propagatedVertexId(record.source, record.fromLayer);
    const std::uint32_t toVertexId =
        propagatedVertexId(record.source, record.toLayer);
    const std::uint32_t targetVertexId = propagatedVertexId(
        targets[record.target].nativeSource, record.toLayer
    );
    const auto &fromVertex = graphVertices[fromVertexId];
    const auto &toVertex = graphVertices[toVertexId];
    const auto &targetVertex = graphVertices[targetVertexId];
    auto &debugEdges = record.direction > 0
        ? _edgeLayersForward[record.fromLayer]
        : _edgeLayersBackward[record.fromLayer];
    if (transition != previousTransition) {
      indexedEdges.emplace_back(fromVertexId, toVertexId);
      debugEdges.emplace_back(fromVertex, toVertex);
      previousTransition = transition;
    }
    indexedEdges.emplace_back(toVertexId, targetVertexId);
    debugEdges.emplace_back(toVertex, targetVertex);
  }
  _graph.addIndexedEdges(graphVertices, indexedEdges, threadCount);
  _propagationStatistics.materializationMilliseconds =
      std::chrono::duration<double, std::milli>(
          Clock::now() - materializationStart
      ).count();
  _propagationStatistics.totalMilliseconds =
      std::chrono::duration<double, std::milli>(
          Clock::now() - propagationStart
      ).count();

  if (_verbose) {
    Utils::print(
        "Vertex propagation backend:",
        vertexPropagationBackendName(
            _propagationStatistics.effectiveBackend
        ),
        "forward rounds:", _propagationStatistics.forwardRounds,
        "backward rounds:", _propagationStatistics.backwardRounds,
        "candidate pairs:", _propagationStatistics.candidatePairs,
        "CUDA accepts:", _propagationStatistics.cudaCertifiedAccepts,
        "exact fallbacks:", _propagationStatistics.cpuExactFallbacks,
        "flatten(ms):", _propagationStatistics.flattenMilliseconds,
        "CPU(ms):", _propagationStatistics.cpuMilliseconds,
        "transfer(ms):", _propagationStatistics.transferMilliseconds,
        "kernel(ms):", _propagationStatistics.kernelMilliseconds,
        "fallback(ms):", _propagationStatistics.exactFallbackMilliseconds,
        "materialize(ms):", _propagationStatistics.materializationMilliseconds,
        "total(ms):", _propagationStatistics.totalMilliseconds
    );
  }
}

template void VisibilityGraph<double>::_connectLayersFixedPoint();
template void VisibilityGraph<float>::_connectLayersFixedPoint();

}  // namespace RotationalVisibilityGraph
