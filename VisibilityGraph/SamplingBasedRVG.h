#ifndef SAMPLING_BASED_ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_
#define SAMPLING_BASED_ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_

#include <VisibilityGraph/VisibilityGraph.h>

namespace RotationalVisibilityGraph {
template<typename T>
class SamplingBasedRVG : public VisibilityGraph<T> {
 public:
//   typedef typename SamplingBasedRVG<T>::TwoTuple TwoTuple;
//   typedef typename SamplingBasedRVG<T>::TwoTupleHash TwoTupleHash;
//   typedef typename Vertex<T>::Hash VertexHash;
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

  using VisibilityGraph<T>::_realRobot;
  using VisibilityGraph<T>::_border;
  using VisibilityGraph<T>::_obstacles;
  using VisibilityGraph<T>::_hashWithTheta;
  using VisibilityGraph<T>::_fineApprox;
  using VisibilityGraph<T>::_numThreads;
  using VisibilityGraph<T>::_verbose;
  using VisibilityGraph<T>::_robot;
  using VisibilityGraph<T>::_isSymmetric;
  using VisibilityGraph<T>::_euclideanWeight;
  using VisibilityGraph<T>::_rotationalWeight;
  using VisibilityGraph<T>::_graph;
  using VisibilityGraph<T>::_symmetricCycle;
  using VisibilityGraph<T>::_roundUpTheta;
  using VisibilityGraph<T>::_layers;
  using VisibilityGraph<T>::_start;
  using VisibilityGraph<T>::_goal;

  SamplingBasedRVG(
      const Polygon<T> &robot,
      const Polygon<T> &border,
      const std::vector<Polygon<T>> &obstacles,
      bool considerSymmetry,
      bool hasWithTheta,
      bool fineApprox,
      int numThreads,
      bool verbose = false
  );
  ~SamplingBasedRVG() = default;
  std::vector<Vertex<T>> shortestPath(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end, int interpolationDensity=0, bool unwrap=true) override;

  private:
    void _resolutionSymmetryCheck() override;
};
}

#endif //SAMPLING_BASED_ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYGRAPH_H_