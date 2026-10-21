#include <VisibilityGraph/VertexPropagation.h>

#ifdef RVG_HAS_CUDA
#include <VisibilityGraph/VertexPropagationCuda.h>
#endif

namespace RotationalVisibilityGraph {

bool cudaVertexPropagationAvailable() {
#ifdef RVG_HAS_CUDA
  std::string error;
  return detail::cudaPropagationRuntimeAvailable(error);
#else
  return false;
#endif
}

}  // namespace RotationalVisibilityGraph
