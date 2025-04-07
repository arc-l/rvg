#include<VisibilityGraph/SamplingBasedRVG.h>
#include <Utils/Timer.h>

namespace RotationalVisibilityGraph {

template <typename T>
SamplingBasedRVG<T>::SamplingBasedRVG(
    const Polygon<T> &robot,
    const Polygon<T> &border,
    const std::vector<Polygon<T>> &obstacles,
    bool considerSymmetry,
    bool hasWithTheta,
    bool fineApprox,
    int numThreads,
    bool verbose
) : VisibilityGraph<T>(robot, border, obstacles, considerSymmetry, hasWithTheta, fineApprox, numThreads, verbose) {
  TRACY_DO(ZoneScopedN("SamplingBasedRVG Constructor");)
  _resolutionSymmetryCheck();
  {
    std::string pythonScript;
    PYTHON_IMPORTS(pythonScript)
    pythonScript += VisibilityGraph<T>::_drawSetup();
    pythonScript += "plt.axis('equal')\n";
    pythonScript += "plt.show()\n";
    Utils::writeStringToFile<T>(pythonScript, "setup.py");
    Utils::runPythonScriptAndRemove<T>("setup.py");
  }
}

template<typename T>
void SamplingBasedRVG<T>::_resolutionSymmetryCheck() {
  TRACY_DO(ZoneScopedN("Resolution Symmetry Check");)
  for (size_t i = 1; i < (size_t) _robot.getVertices().size(); i++)
  {
    T layerAngle = 2. * PI / (T) i;
    Polygon<T> robot = _robot.rotateCopy(layerAngle);
    if (robot == _robot) {
      _roundUpTheta = layerAngle;
      _isSymmetric = true;
    }
  }
}

template <typename T>
std::vector<Vertex<T>> SamplingBasedRVG<T>::shortestPath(
    std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal,
    int interpolationDensity, bool unwrap) {
#define CHECK_OVERLAP(lb1, ub1, lb2, ub2) (lb1 <= ub2 && lb2 <= ub1)
  _start = *start;
  _goal = *goal;
  Utils::Timer timer(false);
  // iteratively build layers
  int tmpResolution = 4; 
  for (int i = 0; i < tmpResolution; i++) {
    T first = i * 2 * M_PI / tmpResolution;
    T second = (i + 1) * 2 * M_PI / tmpResolution;
    Layer<T> layer(first, second, _roundUpTheta, _fineApprox, _hashWithTheta,
                    false);
    layer.buildVisibilityGraph(_robot, _border, _obstacles);
    // OMP_CRITICAL_ {
    //   _layers[i] = layer;
    //   _graph.addEdges(layer.getEdges());
    //   for (size_t pointNum = 0; pointNum < _layers[i].getPoints().size();
    //         pointNum++) {
    //     _layerVertices[i].insert(
    //         {static_cast<T>(i), static_cast<T>(pointNum)});
    //   }
    // }
  }
}

template class SamplingBasedRVG<double>;
template class SamplingBasedRVG<float>;
} // namespace RotationalVisibilityGraph