#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATIONCUDA_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATIONCUDA_H_

#include <cstdint>
#include <string>
#include <vector>

namespace RotationalVisibilityGraph::detail {

struct PropagationInterval {
  double lower = 0;
  double upper = 0;
};

struct PropagationIntervalPoint {
  PropagationInterval x;
  PropagationInterval y;
};

struct PropagationSourceGeometry {
  PropagationIntervalPoint point;
  PropagationIntervalPoint neighborLeft;
  PropagationIntervalPoint neighborRight;
  std::uint8_t onHole = 0;
};

struct PropagationTargetGeometry {
  PropagationIntervalPoint point;
  PropagationIntervalPoint neighborLeft;
  PropagationIntervalPoint neighborRight;
  PropagationInterval boundingX;
  PropagationInterval boundingY;
  std::uint32_t polygonOffset = 0;
  std::uint32_t polygonCount = 0;
  std::uint8_t onHole = 0;
};

using CudaSourceGeometry = PropagationSourceGeometry;
using CudaTargetGeometry = PropagationTargetGeometry;

struct CudaPropagationWorkItem {
  std::uint32_t source = 0;
  std::uint32_t targetOffset = 0;
  std::uint32_t targetCount = 0;
  std::uint32_t candidateOffset = 0;
};

struct CudaPropagationBatchResult {
  std::vector<std::uint32_t> accepted;
  std::vector<std::uint32_t> uncertain;
  double transferMilliseconds = 0;
  double kernelMilliseconds = 0;
};

struct CudaPropagationContext;

bool cudaPropagationRuntimeAvailable(std::string &error);

CudaPropagationContext *createCudaPropagationContext(
    const std::vector<CudaSourceGeometry> &sources,
    const std::vector<CudaTargetGeometry> &targets,
    const std::vector<PropagationIntervalPoint> &polygonPoints,
    std::string &error
);

void destroyCudaPropagationContext(CudaPropagationContext *context);

bool evaluateCudaPropagationBatch(
    CudaPropagationContext *context,
    const std::vector<CudaPropagationWorkItem> &workItems,
    std::uint32_t candidateCount,
    CudaPropagationBatchResult &result,
    std::string &error
);

}  // namespace RotationalVisibilityGraph::detail

#endif  // ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VERTEXPROPAGATIONCUDA_H_
