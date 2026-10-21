#include <VisibilityGraph/VertexPropagationCuda.h>

#include <cub/device/device_select.cuh>
#include <thrust/iterator/counting_iterator.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <string>
#include <vector>

namespace RotationalVisibilityGraph::detail {
namespace {

constexpr std::size_t MAX_WORKSPACE_BYTES = 512ULL * 1024ULL * 1024ULL;
constexpr std::size_t ESTIMATED_BYTES_PER_CANDIDATE = 32;

const char *cudaErrorText(cudaError_t status) {
  return cudaGetErrorString(status);
}

bool cudaCallSucceeded(cudaError_t status, const char *operation,
                       std::string &error) {
  if (status == cudaSuccess) {
    return true;
  }
  error = std::string(operation) + ": " + cudaErrorText(status);
  return false;
}

__device__ PropagationInterval intervalSubtract(
    PropagationInterval left,
    PropagationInterval right
) {
  return {
      __dsub_rd(left.lower, right.upper),
      __dsub_ru(left.upper, right.lower)
  };
}

__device__ PropagationInterval intervalMultiply(
    PropagationInterval left,
    PropagationInterval right
) {
  const double lowerProducts[4] = {
      __dmul_rd(left.lower, right.lower),
      __dmul_rd(left.lower, right.upper),
      __dmul_rd(left.upper, right.lower),
      __dmul_rd(left.upper, right.upper)
  };
  const double upperProducts[4] = {
      __dmul_ru(left.lower, right.lower),
      __dmul_ru(left.lower, right.upper),
      __dmul_ru(left.upper, right.lower),
      __dmul_ru(left.upper, right.upper)
  };
  double lower = lowerProducts[0];
  double upper = upperProducts[0];
#pragma unroll
  for (int index = 1; index < 4; ++index) {
    lower = fmin(lower, lowerProducts[index]);
    upper = fmax(upper, upperProducts[index]);
  }
  return {lower, upper};
}

__device__ PropagationInterval orientationInterval(
    const PropagationIntervalPoint &first,
    const PropagationIntervalPoint &second,
    const PropagationIntervalPoint &third
) {
  const PropagationInterval firstX = intervalSubtract(second.x, first.x);
  const PropagationInterval firstY = intervalSubtract(second.y, first.y);
  const PropagationInterval secondX = intervalSubtract(third.x, first.x);
  const PropagationInterval secondY = intervalSubtract(third.y, first.y);
  return intervalSubtract(
      intervalMultiply(firstX, secondY),
      intervalMultiply(firstY, secondX)
  );
}

__device__ int certifiedSign(PropagationInterval interval) {
  if (interval.lower > 0) {
    return 1;
  }
  if (interval.upper < 0) {
    return -1;
  }
  return 0;
}

enum class CertifiedComparison : std::uint8_t {
  False,
  True,
  Uncertain
};

__device__ CertifiedComparison certifiedLessEqual(
    PropagationInterval left,
    PropagationInterval right
) {
  if (left.upper <= right.lower) {
    return CertifiedComparison::True;
  }
  if (left.lower > right.upper) {
    return CertifiedComparison::False;
  }
  return CertifiedComparison::Uncertain;
}

__device__ CertifiedComparison certifiedGreater(
    PropagationInterval left,
    PropagationInterval right
) {
  if (left.lower > right.upper) {
    return CertifiedComparison::True;
  }
  if (left.upper <= right.lower) {
    return CertifiedComparison::False;
  }
  return CertifiedComparison::Uncertain;
}

enum class CandidateClassification : std::uint8_t {
  Reject,
  Accept,
  Uncertain
};

__device__ CandidateClassification classifyCandidate(
    const CudaSourceGeometry &source,
    const CudaTargetGeometry &target,
    const PropagationIntervalPoint *polygonPoints
) {
  if (source.point.x.upper < target.boundingX.lower ||
      source.point.x.lower > target.boundingX.upper ||
      source.point.y.upper < target.boundingY.lower ||
      source.point.y.lower > target.boundingY.upper) {
    return CandidateClassification::Reject;
  }

  if (!source.onHole && !target.onHole) {
    const int sourceLeft = certifiedSign(orientationInterval(
        target.point, source.point, source.neighborLeft
    ));
    const int sourceRight = certifiedSign(orientationInterval(
        target.point, source.point, source.neighborRight
    ));
    const int targetLeft = certifiedSign(orientationInterval(
        source.point, target.point, target.neighborLeft
    ));
    const int targetRight = certifiedSign(orientationInterval(
        source.point, target.point, target.neighborRight
    ));
    if (sourceLeft == 0 || sourceRight == 0 ||
        targetLeft == 0 || targetRight == 0) {
      return CandidateClassification::Uncertain;
    }
    if (sourceLeft != sourceRight || targetLeft != targetRight) {
      return CandidateClassification::Reject;
    }
  }

  if (target.polygonCount < 3) {
    return CandidateClassification::Reject;
  }
  int winding = 0;
  for (std::uint32_t edgeIndex = 0;
       edgeIndex < target.polygonCount; ++edgeIndex) {
    const PropagationIntervalPoint &first =
        polygonPoints[target.polygonOffset + edgeIndex];
    const PropagationIntervalPoint &second = polygonPoints[
        target.polygonOffset + (edgeIndex + 1) % target.polygonCount
    ];
    const bool overlapsEdgeBounds =
        source.point.x.upper >= fmin(first.x.lower, second.x.lower) &&
        source.point.x.lower <= fmax(first.x.upper, second.x.upper) &&
        source.point.y.upper >= fmin(first.y.lower, second.y.lower) &&
        source.point.y.lower <= fmax(first.y.upper, second.y.upper);
    if (overlapsEdgeBounds &&
        certifiedSign(orientationInterval(first, second, source.point)) == 0) {
      return CandidateClassification::Uncertain;
    }
    const CertifiedComparison firstBelow =
        certifiedLessEqual(first.y, source.point.y);
    if (firstBelow == CertifiedComparison::Uncertain) {
      return CandidateClassification::Uncertain;
    }
    if (firstBelow == CertifiedComparison::True) {
      const CertifiedComparison secondAbove =
          certifiedGreater(second.y, source.point.y);
      if (secondAbove == CertifiedComparison::Uncertain) {
        return CandidateClassification::Uncertain;
      }
      if (secondAbove == CertifiedComparison::True) {
        const int sign = certifiedSign(
            orientationInterval(first, second, source.point)
        );
        if (sign == 0) {
          return CandidateClassification::Uncertain;
        }
        if (sign > 0) {
          ++winding;
        }
      }
    } else {
      const CertifiedComparison secondBelow =
          certifiedLessEqual(second.y, source.point.y);
      if (secondBelow == CertifiedComparison::Uncertain) {
        return CandidateClassification::Uncertain;
      }
      if (secondBelow == CertifiedComparison::True) {
        const int sign = certifiedSign(
            orientationInterval(first, second, source.point)
        );
        if (sign == 0) {
          return CandidateClassification::Uncertain;
        }
        if (sign < 0) {
          --winding;
        }
      }
    }
  }
  return winding == 0
      ? CandidateClassification::Reject
      : CandidateClassification::Accept;
}

__global__ void classifyWorkItemsKernel(
    const CudaSourceGeometry *sources,
    const CudaTargetGeometry *targets,
    const PropagationIntervalPoint *polygonPoints,
    const CudaPropagationWorkItem *workItems,
    std::uint8_t *acceptedFlags,
    std::uint8_t *uncertainFlags,
    std::uint32_t workItemCount
) {
  const std::uint32_t workIndex =
      static_cast<std::uint32_t>(blockIdx.x);
  if (workIndex >= workItemCount) {
    return;
  }
  const CudaPropagationWorkItem work = workItems[workIndex];
  for (std::uint32_t localTarget = threadIdx.x;
       localTarget < work.targetCount;
       localTarget += blockDim.x) {
    const std::uint32_t index = work.candidateOffset + localTarget;
    const CandidateClassification classification = classifyCandidate(
        sources[work.source],
        targets[work.targetOffset + localTarget],
        polygonPoints
    );
    acceptedFlags[index] =
        classification == CandidateClassification::Accept ? 1 : 0;
    uncertainFlags[index] =
        classification == CandidateClassification::Uncertain ? 1 : 0;
  }
}

template <typename Value>
void releaseDevice(Value *&pointer) {
  if (pointer) {
    cudaFree(pointer);
    pointer = nullptr;
  }
}

}  // namespace

struct CudaPropagationContext {
  CudaSourceGeometry *sources = nullptr;
  CudaTargetGeometry *targets = nullptr;
  PropagationIntervalPoint *polygonPoints = nullptr;
  CudaPropagationWorkItem *workItems = nullptr;
  std::uint8_t *acceptedFlags = nullptr;
  std::uint8_t *uncertainFlags = nullptr;
  std::uint32_t *acceptedIndices = nullptr;
  std::uint32_t *uncertainIndices = nullptr;
  int *selectedCounts = nullptr;
  void *selectionTemporary = nullptr;
  std::size_t selectionTemporaryBytes = 0;
  std::size_t candidateCapacity = 0;
  std::size_t workItemCapacity = 0;
};

namespace {

void releaseCandidateWorkspace(CudaPropagationContext *context) {
  releaseDevice(context->workItems);
  releaseDevice(context->acceptedFlags);
  releaseDevice(context->uncertainFlags);
  releaseDevice(context->acceptedIndices);
  releaseDevice(context->uncertainIndices);
  releaseDevice(context->selectedCounts);
  if (context->selectionTemporary) {
    cudaFree(context->selectionTemporary);
    context->selectionTemporary = nullptr;
  }
  context->selectionTemporaryBytes = 0;
  context->candidateCapacity = 0;
  context->workItemCapacity = 0;
}

bool ensureCandidateWorkspace(
    CudaPropagationContext *context,
    std::size_t requestedCapacity,
    std::size_t requestedWorkItemCapacity,
    std::string &error
) {
  if (context->candidateCapacity >= requestedCapacity &&
      context->workItemCapacity >= requestedWorkItemCapacity) {
    return true;
  }
  releaseCandidateWorkspace(context);
  const std::size_t workItemBytes =
      requestedWorkItemCapacity * sizeof(CudaPropagationWorkItem);
  if (!cudaCallSucceeded(
          cudaMalloc(&context->workItems, workItemBytes),
          "cudaMalloc propagation work items", error) ||
      !cudaCallSucceeded(
          cudaMalloc(&context->acceptedFlags, requestedCapacity),
          "cudaMalloc accepted flags", error) ||
      !cudaCallSucceeded(
          cudaMalloc(&context->uncertainFlags, requestedCapacity),
          "cudaMalloc uncertain flags", error) ||
      !cudaCallSucceeded(
          cudaMalloc(
              &context->acceptedIndices,
              requestedCapacity * sizeof(std::uint32_t)
          ),
          "cudaMalloc accepted indices", error) ||
      !cudaCallSucceeded(
          cudaMalloc(
              &context->uncertainIndices,
              requestedCapacity * sizeof(std::uint32_t)
          ),
          "cudaMalloc uncertain indices", error) ||
      !cudaCallSucceeded(
          cudaMalloc(&context->selectedCounts, 2 * sizeof(int)),
          "cudaMalloc selected counts", error)) {
    releaseCandidateWorkspace(context);
    return false;
  }

  std::size_t acceptedTemporaryBytes = 0;
  std::size_t uncertainTemporaryBytes = 0;
  thrust::counting_iterator<std::uint32_t> indices(0);
  if (!cudaCallSucceeded(
          cub::DeviceSelect::Flagged(
              nullptr,
              acceptedTemporaryBytes,
              indices,
              context->acceptedFlags,
              context->acceptedIndices,
              context->selectedCounts,
              static_cast<int>(requestedCapacity)
          ),
          "CUB accepted workspace query", error) ||
      !cudaCallSucceeded(
          cub::DeviceSelect::Flagged(
              nullptr,
              uncertainTemporaryBytes,
              indices,
              context->uncertainFlags,
              context->uncertainIndices,
              context->selectedCounts + 1,
              static_cast<int>(requestedCapacity)
          ),
          "CUB uncertain workspace query", error)) {
    releaseCandidateWorkspace(context);
    return false;
  }
  context->selectionTemporaryBytes =
      std::max(acceptedTemporaryBytes, uncertainTemporaryBytes);
  if (context->selectionTemporaryBytes > 0 &&
      !cudaCallSucceeded(
          cudaMalloc(
              &context->selectionTemporary,
              context->selectionTemporaryBytes
          ),
          "cudaMalloc CUB workspace", error)) {
    releaseCandidateWorkspace(context);
    return false;
  }
  context->workItemCapacity = requestedWorkItemCapacity;
  context->candidateCapacity = requestedCapacity;
  return true;
}

}  // namespace

bool cudaPropagationRuntimeAvailable(std::string &error) {
  int deviceCount = 0;
  const cudaError_t status = cudaGetDeviceCount(&deviceCount);
  if (status != cudaSuccess) {
    error = std::string("cudaGetDeviceCount: ") + cudaErrorText(status);
    cudaGetLastError();
    return false;
  }
  if (deviceCount <= 0) {
    error = "No CUDA device is available";
    return false;
  }
  return true;
}

CudaPropagationContext *createCudaPropagationContext(
    const std::vector<CudaSourceGeometry> &sources,
    const std::vector<CudaTargetGeometry> &targets,
    const std::vector<PropagationIntervalPoint> &polygonPoints,
    std::string &error
) {
  auto *context = new (std::nothrow) CudaPropagationContext();
  if (!context) {
    error = "Failed to allocate the CUDA propagation context";
    return nullptr;
  }
  if ((!sources.empty() &&
       (!cudaCallSucceeded(
           cudaMalloc(
               &context->sources,
               sources.size() * sizeof(CudaSourceGeometry)
           ),
           "cudaMalloc source geometry", error) ||
        !cudaCallSucceeded(
           cudaMemcpy(
               context->sources,
               sources.data(),
               sources.size() * sizeof(CudaSourceGeometry),
               cudaMemcpyHostToDevice
           ),
           "cudaMemcpy source geometry", error))) ||
      (!targets.empty() &&
       (!cudaCallSucceeded(
           cudaMalloc(
               &context->targets,
               targets.size() * sizeof(CudaTargetGeometry)
           ),
           "cudaMalloc target geometry", error) ||
        !cudaCallSucceeded(
           cudaMemcpy(
               context->targets,
               targets.data(),
               targets.size() * sizeof(CudaTargetGeometry),
               cudaMemcpyHostToDevice
           ),
           "cudaMemcpy target geometry", error))) ||
      (!polygonPoints.empty() &&
       (!cudaCallSucceeded(
           cudaMalloc(
               &context->polygonPoints,
               polygonPoints.size() *
                   sizeof(PropagationIntervalPoint)
           ),
           "cudaMalloc polygon geometry", error) ||
        !cudaCallSucceeded(
           cudaMemcpy(
               context->polygonPoints,
               polygonPoints.data(),
               polygonPoints.size() *
                   sizeof(PropagationIntervalPoint),
               cudaMemcpyHostToDevice
           ),
           "cudaMemcpy polygon geometry", error)))) {
    destroyCudaPropagationContext(context);
    return nullptr;
  }
  return context;
}

void destroyCudaPropagationContext(CudaPropagationContext *context) {
  if (!context) {
    return;
  }
  releaseCandidateWorkspace(context);
  releaseDevice(context->sources);
  releaseDevice(context->targets);
  releaseDevice(context->polygonPoints);
  delete context;
}

bool evaluateCudaPropagationBatch(
    CudaPropagationContext *context,
    const std::vector<CudaPropagationWorkItem> &workItems,
    std::uint32_t candidateCount,
    CudaPropagationBatchResult &result,
    std::string &error
) {
  using Clock = std::chrono::steady_clock;
  result = CudaPropagationBatchResult();
  if (workItems.empty() || candidateCount == 0) {
    return workItems.empty() && candidateCount == 0;
  }
  if (!context || !context->sources || !context->targets ||
      !context->polygonPoints) {
    error = "CUDA propagation context is incomplete";
    return false;
  }

  std::uint64_t expectedOffset = 0;
  for (const CudaPropagationWorkItem &work : workItems) {
    if (work.candidateOffset != expectedOffset) {
      error = "CUDA propagation work items are not contiguous";
      return false;
    }
    expectedOffset += work.targetCount;
  }
  if (expectedOffset != candidateCount) {
    error = "CUDA propagation candidate count does not match its work items";
    return false;
  }

  std::size_t freeBytes = 0;
  std::size_t totalBytes = 0;
  if (!cudaCallSucceeded(
          cudaMemGetInfo(&freeBytes, &totalBytes),
          "cudaMemGetInfo", error)) {
    return false;
  }
  const std::size_t budget = std::min(MAX_WORKSPACE_BYTES, freeBytes / 4);
  const std::size_t maximumTile = std::max<std::size_t>(
      1,
      std::min<std::size_t>({
          candidateCount,
          budget / ESTIMATED_BYTES_PER_CANDIDATE,
          static_cast<std::size_t>(std::numeric_limits<int>::max())
      })
  );
  if (!ensureCandidateWorkspace(
          context, maximumTile, workItems.size(), error)) {
    return false;
  }

  result.accepted.reserve(candidateCount / 8);
  result.uncertain.reserve(candidateCount / 16);
  std::vector<CudaPropagationWorkItem> tileWorkItems;
  tileWorkItems.reserve(workItems.size());
  std::size_t workIndex = 0;
  std::uint32_t consumedTargets = 0;
  std::uint32_t globalCandidateOffset = 0;
  while (globalCandidateOffset < candidateCount) {
    tileWorkItems.clear();
    std::uint32_t tileCandidateCount = 0;
    while (tileCandidateCount < maximumTile &&
           workIndex < workItems.size()) {
      const CudaPropagationWorkItem &original = workItems[workIndex];
      const std::uint32_t remaining =
          original.targetCount - consumedTargets;
      const std::uint32_t available = static_cast<std::uint32_t>(
          maximumTile - tileCandidateCount
      );
      const std::uint32_t take = std::min(remaining, available);
      tileWorkItems.push_back({
          original.source,
          original.targetOffset + consumedTargets,
          take,
          tileCandidateCount
      });
      tileCandidateCount += take;
      consumedTargets += take;
      if (consumedTargets == original.targetCount) {
        ++workIndex;
        consumedTargets = 0;
      }
    }
    if (tileCandidateCount == 0) {
      error = "CUDA propagation produced an empty candidate tile";
      return false;
    }

    const auto transferToDeviceStart = Clock::now();
    if (!cudaCallSucceeded(
            cudaMemcpy(
                context->workItems,
                tileWorkItems.data(),
                tileWorkItems.size() * sizeof(CudaPropagationWorkItem),
                cudaMemcpyHostToDevice
            ),
            "cudaMemcpy propagation work items", error)) {
      return false;
    }
    result.transferMilliseconds +=
        std::chrono::duration<double, std::milli>(
            Clock::now() - transferToDeviceStart
        ).count();

    const auto kernelStart = Clock::now();
    constexpr std::uint32_t blockSize = 128;
    classifyWorkItemsKernel<<<
        static_cast<std::uint32_t>(tileWorkItems.size()), blockSize
    >>>(
        context->sources,
        context->targets,
        context->polygonPoints,
        context->workItems,
        context->acceptedFlags,
        context->uncertainFlags,
        static_cast<std::uint32_t>(tileWorkItems.size())
    );
    if (!cudaCallSucceeded(
            cudaGetLastError(), "classifyWorkItemsKernel launch", error)) {
      return false;
    }
    thrust::counting_iterator<std::uint32_t> indices(0);
    if (!cudaCallSucceeded(
            cub::DeviceSelect::Flagged(
                context->selectionTemporary,
                context->selectionTemporaryBytes,
                indices,
                context->acceptedFlags,
                context->acceptedIndices,
                context->selectedCounts,
                static_cast<int>(tileCandidateCount)
            ),
            "CUB accepted selection", error) ||
        !cudaCallSucceeded(
            cub::DeviceSelect::Flagged(
                context->selectionTemporary,
                context->selectionTemporaryBytes,
                indices,
                context->uncertainFlags,
                context->uncertainIndices,
                context->selectedCounts + 1,
                static_cast<int>(tileCandidateCount)
            ),
            "CUB uncertain selection", error) ||
        !cudaCallSucceeded(
            cudaDeviceSynchronize(), "CUDA propagation synchronization", error)) {
      return false;
    }
    result.kernelMilliseconds +=
        std::chrono::duration<double, std::milli>(Clock::now() - kernelStart)
            .count();

    const auto transferToHostStart = Clock::now();
    int selectedCounts[2] = {0, 0};
    if (!cudaCallSucceeded(
            cudaMemcpy(
                selectedCounts,
                context->selectedCounts,
                sizeof(selectedCounts),
                cudaMemcpyDeviceToHost
            ),
            "cudaMemcpy selected counts", error)) {
      return false;
    }
    std::vector<std::uint32_t> selectedAccepted(selectedCounts[0]);
    std::vector<std::uint32_t> selectedUncertain(selectedCounts[1]);
    if ((selectedCounts[0] > 0 &&
         !cudaCallSucceeded(
             cudaMemcpy(
                 selectedAccepted.data(),
                 context->acceptedIndices,
                 selectedAccepted.size() * sizeof(std::uint32_t),
                 cudaMemcpyDeviceToHost
             ),
             "cudaMemcpy accepted indices", error)) ||
        (selectedCounts[1] > 0 &&
         !cudaCallSucceeded(
             cudaMemcpy(
                 selectedUncertain.data(),
                 context->uncertainIndices,
                 selectedUncertain.size() * sizeof(std::uint32_t),
                 cudaMemcpyDeviceToHost
             ),
             "cudaMemcpy uncertain indices", error))) {
      return false;
    }
    for (const std::uint32_t index : selectedAccepted) {
      result.accepted.push_back(globalCandidateOffset + index);
    }
    for (const std::uint32_t index : selectedUncertain) {
      result.uncertain.push_back(globalCandidateOffset + index);
    }
    result.transferMilliseconds +=
        std::chrono::duration<double, std::milli>(
            Clock::now() - transferToHostStart
        ).count();
    globalCandidateOffset += tileCandidateCount;
  }
  return true;
}

}  // namespace RotationalVisibilityGraph::detail
