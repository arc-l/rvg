#include "VisibilityGraph.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_vertical_decomposition_2.h>
#include <CGAL/partition_2.h>
#include <Utils/Timer.h>
#include <VisibilityGraph/Utils.h>
#include <unistd.h>

#include <cstdlib>

namespace RotationalVisibilityGraph {

template <typename T>
bool VisibilityGraph<T>::TwoTuple::operator==(const TwoTuple &rhs) const {
  return first == rhs.first && second == rhs.second;
}

template <typename T>
size_t VisibilityGraph<T>::TwoTupleHash::operator()(
    const TwoTuple &layer) const {
  size_t seed = 0;
  Utils::hash_combine(seed, std::hash<T>{}(layer.first));
  Utils::hash_combine(seed, std::hash<T>{}(layer.second));
  return seed;
}

template <typename T>
bool VisibilityGraph<T>::TwoTupleHash::operator()(const TwoTuple &lhs,
                                                  const TwoTuple &rhs) const {
  return lhs.first == rhs.first && lhs.second == rhs.second;
}

template <typename T>
VisibilityGraph<T>::VisibilityGraph(const Polygon<T> &robot,
                                    const Polygon<T> &border,
                                    const std::vector<Polygon<T>> &obstacles,
                                    bool considerSymmetry,
                                    bool fineApprox, int numThreads,
                                    bool verbose)
    : _realRobot(robot),
      _border(border),
      _obstacles(obstacles),
      _fineApprox(fineApprox),
      _numThreads(numThreads),
      _verbose(verbose) {
  TRACY_DO(ZoneScopedN("VisibilityGraph Constructor");)
  _robot = _realRobot;
  _isSymmetric = false;
  _euclideanWeight = 1.0;
  _rotationalWeight = 0.0;
  _graph.setWeight(_euclideanWeight, _rotationalWeight);
  _symmetricCycle = -1;
}

template <typename T>
VisibilityGraph<T>::VisibilityGraph(const Polygon<T> &robot,
                                    const Polygon<T> &border,
                                    const std::vector<Polygon<T>> &obstacles,
                                    int resolution, bool considerSymmetry,
                                    bool fineApprox,
                                    int numThreads, bool optimal, bool verbose)
    : VisibilityGraph(robot, border, obstacles, considerSymmetry,
                      fineApprox, numThreads, verbose){
  _resolution=resolution;
  _optimal= optimal; 
  if (considerSymmetry)
    _resolutionSymmetryCheck();
  else
    _roundUpTheta = 2 * PI;

  _layerVertices.resize(_isSymmetric ? _symmetricCycle : _resolution);
  Utils::Timer timer(_verbose);
  _buildLayers();
  _buildTime = timer.elapsed();
  if (_verbose)
    Utils::print("Resolution: ", resolution, " Time: ", _buildTime, "s");
}

template <typename T>
void VisibilityGraph<T>::_buildLayers() {
  TRACY_DO(ZoneScopedN("Build Layers");)
  Utils::Timer buildTimer("Build Layers", false);
  if (_isSymmetric) {
    _layers.resize(_symmetricCycle);
    OMP_PARALLEL_FOR_XI(
        _numThreads,
        default(none) shared(_layers, _obstacles, _graph, _layerVertices,
                             _robot, _border, _resolution, _symmetricCycle))
    for (int i = 0; i < _symmetricCycle; i++) {
      TRACY_DO(ZoneScopedN("Build Layer Loop");)
      T first = i * 2 * M_PI / _resolution;
      T second = (i + 1) * 2 * M_PI / _resolution;
      Layer<T> layer(first, second, _roundUpTheta, _fineApprox, false);
      layer.buildVisibilityGraph(_robot, _border, _obstacles);
      OMP_CRITICAL_ {
        _layers[i] = layer;
        _graph.addEdges(layer.getEdges());
        for (size_t pointNum = 0; pointNum < _layers[i].getPoints().size();
             pointNum++) {
          _layerVertices[i].insert(
              {static_cast<T>(i), static_cast<T>(pointNum)});
        }
      }
    }
  } else {
    _layers.resize(_resolution);
    OMP_PARALLEL_FOR_XI(
        _numThreads,
        default(none) shared(_layers, _obstacles, _graph, _layerVertices,
                             _robot, _border, _resolution, _symmetricCycle))
    for (int i = 0; i < _resolution; i++) {
      T first = i * 2 * M_PI / _resolution;
      T second = (i + 1) * 2 * M_PI / _resolution;
      Layer<T> layer(first, second, _roundUpTheta, _fineApprox, false);
      layer.buildVisibilityGraph(_robot, _border, _obstacles);
      OMP_CRITICAL_ {
        _layers[i] = layer;
        _graph.addEdges(layer.getEdges());
        for (size_t pointNum = 0; pointNum < _layers[i].getPoints().size();
             pointNum++) {
          _layerVertices[i].insert(
              {static_cast<T>(i), static_cast<T>(pointNum)});
        }
      }
    }
  }
  // _graph.draw3D("../Results/graph.png", true);
  // drawLayers3D("../Results/semi-algebraic-set"+std::to_string(_resolution)+".png",
  // true);
  if (_verbose)
    Utils::print("Time for building all layers take ", buildTimer.elapsed(),
                 "s");
  Utils::Timer connectTimer("Connect Layers", false);
  _connectLayers();
  if (_verbose)
    Utils::print("Time for connecting all layers take ", connectTimer.elapsed(),
                 "s");
}

template <typename T>
void VisibilityGraph<T>::_connectLayers() {
  TRACY_DO(ZoneScopedN("Connect Layers");)
// #define PARALLELIZE_LAYERS
// TODO: Parallelism on layers should be removed later
#ifdef PARALLELIZE_LAYERS
  _connectLayersParallelizedLayers();
#else
  _connectLayersParallelizedVertices();
#endif
}

template <typename T>
void VisibilityGraph<T>::_connectLayersParallelizedVertices() {
#define NEXT ((i + 1) % _layers.size())
#define PREV ((i - 1 + _layers.size()) % _layers.size())
#define PRINT
// #define PLOT
#define MANIM
#define BACKWARD_PASS
  TRACY_DO(ZoneScopedN("Connect Layers Parallelized Vertices");)
#ifdef MANIM
  _edgeLayersForward.clear();
  _edgeLayersBackward.clear();
  _edgeLayersForward.resize(_layers.size());
  _edgeLayersBackward.resize(_layers.size());
#endif
  std::vector<std::vector<
      std::unordered_map<Vertex<T>, Polygon_2, typename Vertex<T>::Hash>>>
      visibleAreaCacheParallelVector(_layers.size());
  for (size_t i = 0; i < _layers.size(); ++i) {
    const Layer<T> &currLayer = _layers[i];
    // load visibleAreas
    std::vector<
        std::unordered_map<Vertex<T>, Polygon_2, typename Vertex<T>::Hash>>
        visibleAreaCacheParallel;
    for (int threadId = 0; threadId < omp_get_max_threads(); threadId++) {
      visibleAreaCacheParallel.push_back(currLayer.getVisibleAreaCache());
    }
    visibleAreaCacheParallelVector[i] = visibleAreaCacheParallel;
  }

  // forward pass
  for (int i = 0; i < (int)_layers.size(); i++) {
    TRACY_DO(ZoneScopedN("Forward Pass");)
    const Layer<T> &currLayer = _layers[i];
    const Layer<T> &nextLayer = _layers[NEXT];
    T thetaLb = currLayer.getThetaLb();
    T thetaUb = currLayer.getThetaUb();
    T theta = (thetaLb < thetaUb ? (thetaLb + thetaUb) / 2 : 0);
    // std::unordered_set<TwoTuple, TwoTupleHash> addToNextLayer;
    std::vector<std::unordered_set<TwoTuple, TwoTupleHash>>
        addToNextLayerParallel(_numThreads);
    std::vector<std::vector<
        std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>>
        edgesParallel(_numThreads);
    const auto &currVertexIndexSet = _layerVertices[i];
#ifdef PRINT
    if (_verbose)
      Utils::print("Layer", i,
                   "current Layer #vertices = ", currVertexIndexSet.size(),
                   "nextLayer #vertices = ", _layerVertices[i].size());
#endif
    // use OMP
    std::vector<TwoTuple> currVertexIndex(currVertexIndexSet.begin(),
                                          currVertexIndexSet.end());
    OMP_PARALLEL_FOR_XI(
        _numThreads, default(none) shared(
                         currVertexIndex, addToNextLayerParallel,
                         visibleAreaCacheParallelVector, currLayer, nextLayer,
                         thetaLb, thetaUb, theta, i, _graph, edgesParallel))
    for (size_t vertexIndex = 0; vertexIndex < currVertexIndex.size();
         vertexIndex++) {
      TRACY_DO(ZoneScopedN("Forward Pass OMP Loop");)
      int threadId = omp_get_thread_num();
      const auto &vertex = currVertexIndex[vertexIndex];
      const Point_2 &p = _layers[vertex.first].getPoint(vertex.second);
      const Vertex<T> &v = _layers[vertex.first].getVertex(vertex.second);
      const auto &neighbor1 = _layers[vertex.first].getNeighbor(v);
      const auto &visibleAreaCache = visibleAreaCacheParallelVector[NEXT][threadId];
      for (const auto &pair : visibleAreaCache) {
        const Vertex<T> &vObs = pair.first;
        const auto &neighbor2 = _layers[NEXT].getNeighbor(vObs);
        if ((!_layers[vertex.first].isVertexOnHole(v) &&
             !_layers[NEXT].isVertexOnHole(vObs)) &&
            !Utils::isBitangent<T>(p, neighbor1.first, neighbor1.second,
                                   vObs.getPoint(), neighbor2.first,
                                   neighbor2.second))
          continue;
        const Polygon_2 &visibleArea = pair.second;
        if (IN_POLYGON(p, visibleArea) || ON_EDGE(p, visibleArea)) {
          std::shared_ptr<Vertex<T>> vNew = std::make_shared<Vertex<T>>(vObs);
          std::shared_ptr<Vertex<T>> vv = std::make_shared<Vertex<T>>(v);
          vv->setBounds(thetaLb, thetaUb);
          vv->setTheta(theta);
          std::shared_ptr<Vertex<T>> vv1 = std::make_shared<Vertex<T>>(v);
          vv1->setBounds(nextLayer.getThetaLb(), nextLayer.getThetaUb());
          vv1->setTheta((nextLayer.getThetaLb()+nextLayer.getThetaUb())/2.);
          edgesParallel[threadId].emplace_back(vv, vv1);
          edgesParallel[threadId].emplace_back(vv1, vNew);
          addToNextLayerParallel[threadId].insert(vertex);
        }
      }
    }

    {
      TRACY_DO(ZoneScopedN("Add Edges");)
      for (const auto &edgeThread : edgesParallel) {
        _graph.addEdges(edgeThread);
#ifdef MANIM
        for (const auto &edge : edgeThread) {
          _edgeLayersForward[i].emplace_back(edge);
        }
#endif
      }
    }
    if (_optimal) {
      TRACY_DO(ZoneScopedN("Add to Next Layer");)
      for (const auto &addToNextLayerThread : addToNextLayerParallel) {
        if (addToNextLayerThread.size() == 0) continue;
        _layerVertices[NEXT].insert(addToNextLayerThread.begin(),
                                    addToNextLayerThread.end());
      }
    }
  }

#ifdef BACKWARD_PASS
  // backward pass
  for (int i = (int)_layers.size() - 1; i >= 0; i--) {
    TRACY_DO(ZoneScopedN("Backward Pass");)
    Layer<T> &currLayer = _layers[i];
    Layer<T> &nextLayer = _layers[PREV];
    T thetaLb = currLayer.getThetaLb();
    T thetaUb = currLayer.getThetaUb();
    T theta = (thetaLb < thetaUb ? (thetaLb + thetaUb) / 2 : 0);
    // std::unordered_set<TwoTuple, TwoTupleHash> addToNextLayer;
    std::vector<std::unordered_set<TwoTuple, TwoTupleHash>>
        addToNextLayerParallel(_numThreads);
    std::vector<std::vector<
        std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>>
        edgesParallel(_numThreads);
    const auto &currVertexIndexSet = _layerVertices[i];
#ifdef PRINT
    if (_verbose)
      Utils::print("Layer", i,
                   "current Layer #vertices = ", currVertexIndexSet.size(),
                   "nextLayer #vertices = ", _layerVertices[i].size());
#endif
    // use OMP
    std::vector<TwoTuple> currVertexIndex(currVertexIndexSet.begin(),
                                          currVertexIndexSet.end());
    OMP_PARALLEL_FOR_XI(
        _numThreads, default(none) shared(
                         currVertexIndex, addToNextLayerParallel,
                         visibleAreaCacheParallelVector, currLayer, nextLayer,
                         thetaLb, thetaUb, theta, i, _graph, edgesParallel))
    for (size_t vertexIndex = 0; vertexIndex < currVertexIndex.size();
         vertexIndex++) {
      TRACY_DO(ZoneScopedN("Backward Pass OMP Loop");)
      int threadId = omp_get_thread_num();
      const auto &vertex = currVertexIndex[vertexIndex];
      if (vertex.first < i) continue;
      const Point_2 &p = _layers[vertex.first].getPoint(vertex.second);
      const Vertex<T> &v = _layers[vertex.first].getVertex(vertex.second);
      const auto &neighbor1 = _layers[vertex.first].getNeighbor(v);
      const auto &visibleAreaCache =
          visibleAreaCacheParallelVector[PREV][threadId];
      for (const auto &pair : visibleAreaCache) {
        const Vertex<T> &vObs = pair.first;
        const auto &neighbor2 = _layers[PREV].getNeighbor(vObs);
        if ((!_layers[vertex.first].isVertexOnHole(v) &&
             !_layers[PREV].isVertexOnHole(vObs)) &&
            !Utils::isBitangent<T>(p, neighbor1.first, neighbor1.second,
                                   vObs.getPoint(), neighbor2.first,
                                   neighbor2.second))
          continue;
        const Polygon_2 &visibleArea = pair.second;
        if (IN_POLYGON(p, visibleArea) || ON_EDGE(p, visibleArea)) {
          std::shared_ptr<Vertex<T>> vNew = std::make_shared<Vertex<T>>(vObs);
          std::shared_ptr<Vertex<T>> vv = std::make_shared<Vertex<T>>(v);
          vv->setBounds(thetaLb, thetaUb);
          vv->setTheta(theta);
          std::shared_ptr<Vertex<T>> vv1 = std::make_shared<Vertex<T>>(v);
          vv1->setBounds(nextLayer.getThetaLb(), nextLayer.getThetaUb());
          vv1->setTheta((nextLayer.getThetaLb()+nextLayer.getThetaUb())/2.);
          edgesParallel[threadId].emplace_back(vv, vv1);
          edgesParallel[threadId].emplace_back(vv1, vNew);
          addToNextLayerParallel[threadId].insert(vertex);
        }
      }
    }

    {
      TRACY_DO(ZoneScopedN("Add Edges");)
      for (const auto &edgeThread : edgesParallel) {
        _graph.addEdges(edgeThread);
#ifdef MANIM
        for (const auto &edge : edgeThread) {
          _edgeLayersBackward[i].emplace_back(edge);
        }
#endif
      }
    }
    // _layerVertices[PREV].insert(addToNextLayer.begin(),
    // addToNextLayer.end());
    if (_optimal) {
      TRACY_DO(ZoneScopedN("Add to Next Layer");)
      for (const auto &addToNextLayerThread : addToNextLayerParallel) {
        if (addToNextLayerThread.size() == 0) continue;
        _layerVertices[PREV].insert(addToNextLayerThread.begin(),
                                    addToNextLayerThread.end());
      }
    }
  }
  // Utils::print("Number of connected components = ",
  // _graph.numConnectedComponents()); _graph.draw("../Results/graph.png",
  // true); _graph.draw3D("../Results/graph.png", true);
  // draw("../Results/graph.png", false, false, true, true);
  // drawSetup();

#endif
#undef NEXT
#undef PREV
}

template <typename T>
void VisibilityGraph<T>::_addStartAndGoal(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal){
  bool directConnection = false; 
  std::vector<bool> startLegalVec(_layers.size(), false);
  std::vector<bool> goalLegalVec(_layers.size(), false);
  std::vector<Polygon_2> startVisibleAreaVec(_layers.size());
  std::vector<Polygon_2> goalVisibleAreaVec(_layers.size());
  std::vector<std::shared_ptr<Vertex<T>>> startInLayerVec(_layers.size(), nullptr);
  std::vector<std::shared_ptr<Vertex<T>>> goalInLayerVec(_layers.size(), nullptr);
  size_t startLayerIndex, goalLayerIndex;

  // find the layer that contains the start vertex and initialize the visible area
  OMP_PARALLEL_FOR_XI(_numThreads, default(none) shared(directConnection, start, goal, startLegalVec, goalLegalVec, startVisibleAreaVec, goalVisibleAreaVec, startInLayerVec, goalInLayerVec, startLayerIndex, goalLayerIndex))
  for (size_t i = 0; i < _layers.size(); i++) {
    const Layer<T> &layer = _layers[i];
    const T &thetaLb = layer.getThetaLb();
    const T &thetaUb = layer.getThetaUb();
    startLegalVec[i] = layer.legalConfig(*start);
    goalLegalVec[i] = layer.legalConfig(*goal);
    std::shared_ptr<Vertex<T>> startInLayer = std::make_shared<Vertex<T>>(start->getX(), start->getY(), _layers[i].getThetaLb(), _layers[i].getThetaUb(), (_layers[i].getThetaUb()+_layers[i].getThetaLb())/2., _roundUpTheta);
    std::shared_ptr<Vertex<T>> goalInLayer = std::make_shared<Vertex<T>>(goal->getX(), goal->getY(), _layers[i].getThetaLb(), _layers[i].getThetaUb(), (_layers[i].getThetaUb()+_layers[i].getThetaLb())/2., _roundUpTheta);
    startInLayerVec[i] = startInLayer;
    goalInLayerVec[i] = goalInLayer;
    if(thetaLb <= start->getTheta() && start->getTheta() <= thetaUb){
      startLayerIndex = i;
      if(!startLegalVec[i])
        throw std::runtime_error("Start vertex is not legal. Please check the configuration of the start vertex.");
      _graph.addEdge(start, startInLayer);
    }
    if(thetaLb <= goal->getTheta() && goal->getTheta() <= thetaUb){
      goalLayerIndex = i;
      if(!goalLegalVec[i])
        throw std::runtime_error("Goal vertex is not legal. Please check the configuration of the goal vertex.");
      _graph.addEdge(goalInLayer, goal);
    }
    if (startLegalVec[i]) {
      std::shared_ptr<Arrangement_2> visibleArea = layer.getVisibleArea(*start);
      if (visibleArea)
        startVisibleAreaVec[i] = Utils::arrangementToPolygon<T>(*visibleArea);
    }
    if (goalLegalVec[i]) {
      std::shared_ptr<Arrangement_2> visibleArea = layer.getVisibleArea(*goal);
      if (visibleArea)
        goalVisibleAreaVec[i] = Utils::arrangementToPolygon<T>(*visibleArea);
    }
  }

  // propagate the start vertex and the goal vertex
  std::vector<size_t> indices{startLayerIndex, goalLayerIndex};
  bool isGoal = false;
  for(int _ = 0; _ < 2; _++){
    int id = indices[isGoal];
    bool allAvailable = true;
    auto & legalVec = isGoal ? goalLegalVec : startLegalVec;
    const auto & vertexInLayerVec = isGoal ? goalInLayerVec : startInLayerVec;
    int forwardStopIndex, backwardStopIndex;
    for(int i = 0; i < _layers.size(); i++){
      int currIndex = (id + i) % _layers.size();
      int prevIndex = (id + i - 1) % _layers.size();
      if(legalVec[currIndex]){
        std::shared_ptr<Vertex<T>> vertexInLayer = vertexInLayerVec[currIndex];
        std::shared_ptr<Vertex<T>> prevVertexInLayer = vertexInLayerVec[prevIndex];
        _graph.addEdge(prevVertexInLayer, vertexInLayer);
      }
      else {
        allAvailable = false;
        forwardStopIndex = currIndex;
        break;
      }
    }
    if(!allAvailable){
      for(int i = 1; i < _layers.size(); i++){
        int currIndex = (id - i + _layers.size()) % _layers.size();
        int prevIndex = (id - i + 1) % _layers.size();
        if(legalVec[currIndex]){
          std::shared_ptr<Vertex<T>> vertexInLayer = vertexInLayerVec[currIndex];
          std::shared_ptr<Vertex<T>> prevVertexInLayer = vertexInLayerVec[prevIndex];
          _graph.addEdge(prevVertexInLayer, vertexInLayer);
        }
        else{
          backwardStopIndex = currIndex;
          break;
        }
      }
      for(int i = forwardStopIndex; i != backwardStopIndex; i = (i + 1) % _layers.size()){
        legalVec[i] = false;
      }
    }
    isGoal = !isGoal;
  }

  OMP_PARALLEL_FOR_XI(_numThreads, default(none) shared(directConnection, start, goal, startLegalVec, goalLegalVec, startVisibleAreaVec, goalVisibleAreaVec, startInLayerVec, goalInLayerVec))
  for (int i = 0; i < _layers.size(); i++) {
    if(directConnection) continue;
    const Layer<T> &layer = _layers[i];
    if (!layer.isFeasible()) {
      if (_verbose) Utils::print("Layer ", i, " is not feasible, skipping");
      continue;
    }
    const T &thetaLb = layer.getThetaLb();
    const T &thetaUb = layer.getThetaUb();
    std::shared_ptr<Vertex<T>> startInLayer = startInLayerVec[i];
    std::shared_ptr<Vertex<T>> goalInLayer = goalInLayerVec[i];
    const auto &vertices = _layerVertices[i];
    std::vector<TwoTuple> verticesVector(vertices.begin(), vertices.end());
    if(startLegalVec[i]){
      const auto &visiblePolygon = startVisibleAreaVec[i];
      if (goalLegalVec[i]) {
        const auto &p = goal->getPoint();
        if ((IN_POLYGON(p, visiblePolygon) || ON_EDGE(p, visiblePolygon))) {
          OMP_CRITICAL_ {
              directConnection = true;
              _graph.addEdge(startInLayer, goalInLayer);
            }
        }
      }
    }

    bool isGoal = false;
    for(int _ = 0; _< 2; _++){
      const auto & legalVec = isGoal ? goalLegalVec : startLegalVec;
      const auto & vertexInLayerVec = isGoal ? goalInLayerVec : startInLayerVec;
      const auto & visibleAreaVec = isGoal ? goalVisibleAreaVec : startVisibleAreaVec;
      if (legalVec[i]) {
        OMP_PARALLEL_FOR_XI( _numThreads, default(none) shared(directConnection, verticesVector, thetaLb, thetaUb, startInLayer, visibleAreaVec, layer, start, goal, vertexInLayerVec, i))
        for (size_t j = 0; j < verticesVector.size(); j++) {
          const auto &validV = verticesVector[j];
          const auto &p = _layers[validV.first].getPoint(validV.second);
          if (IN_POLYGON(p, visibleAreaVec[i]) || ON_EDGE(p, visibleAreaVec[i])) {
            std::shared_ptr<Vertex<T>> visibleVertex =
                std::make_shared<Vertex<T>>(
                    CGAL::to_double(p.x()), CGAL::to_double(p.y()),
                    layer.getThetaLb(), layer.getThetaUb(),
                    (layer.getThetaLb() + layer.getThetaUb()) / 2., _roundUpTheta);
            OMP_CRITICAL_ { 
              _graph.addEdge(vertexInLayerVec[i], visibleVertex);
            }
          }
        }
      }
      isGoal = !isGoal;
    }
  }
}

template <typename T>
std::vector<Vertex<T>> VisibilityGraph<T>::shortestPath(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal, int interpolationDensity, bool unwrap) {
  // connect start to goal through visibility graph
  _start = *start;
  _goal = *goal;
  Utils::Timer timer(false);
  _addStartAndGoal(start, goal);

  // graph search
  Utils::Timer searchTimer(false);
  // if (directConnection) _sol = {*start, *goal}; else
  _sol = _graph.shortestPath(start, goal, _verbose);
  _searchTime = timer.elapsed();
  _totalTime = _buildTime + _searchTime;
  if (_verbose) Utils::print("Searching shortest path took", _searchTime, "s");
  if (!_sol.empty()) {
    if (unwrap) _unwrapPath();
    if (interpolationDensity > 0) _interpolation(interpolationDensity);
    if (_verbose)
      for (const auto &vertex : _sol) Utils::print(vertex);
    // path length
    T pathLength = 0;
    T totalRotation = 0;
    for (size_t i = 0; i < _sol.size() - 1; i++) {
      pathLength += Edge(_sol[i], _sol[i + 1]).length();
      totalRotation += _sol[i].rotationalDist(_sol[i + 1]);
    }
    _pathLength = pathLength;
    _totalRotation = totalRotation;
    if (_verbose) {
      Utils::print("Path length = ", pathLength);
      Utils::print("Total rotation = ", totalRotation);
    }
  } else {
    if (_verbose) Utils::print("No path found");
  }
  return _sol;
}

template <typename T>
T VisibilityGraph<T>::getPathLength() const {
  return _pathLength;
}

template <typename T>
T VisibilityGraph<T>::getTotalTime() const {
  return _totalTime;
}

template <typename T>
T VisibilityGraph<T>::getBuildTime() const {
  return _buildTime;
}

template <typename T>
T VisibilityGraph<T>::getSearchTime() const {
  return _searchTime;
}

template <typename T>
void VisibilityGraph<T>::_resolutionSymmetryCheck() {
  Polygon<T> robot = _robot;
  T layerAngle = 2. * PI / (T)_resolution;
  int cnt;
  for (cnt = 0; cnt < _resolution / 2. + 1; cnt++) {
    robot.rotate(layerAngle);
    if (robot == _robot) {
      _symmetricCycle = cnt + 1;
      _roundUpTheta = _symmetricCycle * layerAngle;
      _isSymmetric = true;
      break;
    }
  }
  if(!_isSymmetric)
    _roundUpTheta = 2 * PI;
}

template <typename T>
void VisibilityGraph<T>::setWeight(T euclideanWeight, T rotationalWeight) {
  _euclideanWeight = euclideanWeight;
  _rotationalWeight = rotationalWeight;
  _graph.setWeight(euclideanWeight, rotationalWeight);
}

template <typename T>
void VisibilityGraph<T>::draw(const std::string &figPath, bool title, bool path,
                              bool graph, bool show) {
  if (figPath.empty()) return;
  // #define SEMI_ALGEBRAIC_SET
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)

#ifdef SEMI_ALGEBRAIC_SET
  pythonScript += "fig, ax = plt.subplots(dpi=500)\n";
#endif

  if (graph) {
    _graph.buildEdges();
    for (const auto &edge : _graph.getEdges()) {
      std::vector<T> edgeX = edge.getX(), edgeY = edge.getY();
#ifdef SEMI_ALGEBRAIC_SET
      pythonScript += "ax.plot([" + std::to_string(edgeX[0]) + ", " +
                      std::to_string(edgeX[1]) + "], [" +
                      std::to_string(edgeY[0]) + ", " +
                      std::to_string(edgeY[1]) +
                      "], '-', color='darkcyan', linewidth=1.0)\n";
#else
      pythonScript += "ax.plot([" + std::to_string(edgeX[0]) + ", " +
                      std::to_string(edgeX[1]) + "], [" +
                      std::to_string(edgeY[0]) + ", " +
                      std::to_string(edgeY[1]) +
                      "], '-', color='lightcoral', linewidth=0.5)\n";
#endif
    }
    for (const auto &v : _graph.getVertices()) {
#ifdef SEMI_ALGEBRAIC_SET
      pythonScript += "ax.plot(" + std::to_string(v.getX()) + ", " +
                      std::to_string(v.getY()) +
                      ", 'o', color='#EC008C', markersize=1.0)\n";
#else
      pythonScript += "ax.plot(" + std::to_string(v->getX()) + ", " +
                      std::to_string(v->getY()) +
                      ", 'o', color='bisque', markersize=0.5)\n";
#endif
    }
  }

#ifdef SEMI_ALGEBRAIC_SET
#else
  pythonScript += _drawSetup();
#endif

  if (path && _sol.size() >= 2) {
    pythonScript += "path = np.array([\n";
    for (const auto &vertex : _sol) {
      pythonScript += "\t[" + std::to_string(vertex.getX()) + ", " +
                      std::to_string(vertex.getY()) + "],\n";
    }
    pythonScript += "])\n";
    pythonScript +=
        "ax.plot(path[:, 0], path[:, 1], '-o', color='navy', "
        "markersize=1.0, linewidth=1.0)\n";
    // robot along the path
    if (_sol.size() >= 2) {
      for (size_t nodeNum = 1; nodeNum < _sol.size() - 1; nodeNum++) {
        const auto &vertex = _sol[nodeNum];
        Polygon<T> tmpRobot = _realRobot.moveToCopy(
            vertex.getX(), vertex.getY(), vertex.getTheta());
        pythonScript += tmpRobot.draw("path");
      }
    }
  }
  if (title)
#ifdef COMPLETENESS
    pythonScript +=
        "plt.title(f'Path length={" + std::to_string(getPathLength()) +
        ":.2f} Total Rotation={" + std::to_string(_totalRotation) +
        ":.2f}\\nDistanceWeight={" + std::to_string(_euclideanWeight) +
        ":.2f} RotationWeight={" + std::to_string(_rotationalWeight) +
        ":.2f}')\n";
#else
    pythonScript +=
        "plt.title(f'Resolution=" + std::to_string(_resolution) +
        ", build time={" + std::to_string(_buildTime) +
        ":.3f}s, search time={" + std::to_string(_searchTime) +
        ":.3f}s,\\nPath length={" + std::to_string(getPathLength()) +
        ":.2f} Total rotation={" + std::to_string(_totalRotation) +
        ":.2f}\\nDistanceWeight={" + std::to_string(_euclideanWeight) +
        ":.1f} RotationWeight={" + std::to_string(_rotationalWeight) +
        ":.1f}')\n";
#endif
  pythonScript += "plt.axis('equal')\n";
  pythonScript +=
      "plt.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  if (show) pythonScript += "plt.show()\n";
  std::string pythonSavePath = "path" + std::to_string(_resolution) + ".py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);
}

template <typename T>
void VisibilityGraph<T>::animation(const std::string &figPath, bool title) {
  if (_sol.empty()) {
    if (_verbose) Utils::print("No solution found! Cannot create animation");
    return;
  }

  if (figPath.empty()) return;

  // create temp folder
  std::string tmpFolderPath = Utils::createTempFolder<T>();
  std::string pythonScript;
  // use pillow to create gif
  PYTHON_IMPORTS(pythonScript)
  pythonScript += "image_paths = []\n";
  for (size_t i = 0; i < _sol.size(); i++) {
    pythonScript += "fig, ax = plt.subplots()\n";
    pythonScript += "ax.set_facecolor('gainsboro')\n";
    pythonScript += "plt.axis('equal')\n";
    pythonScript += _drawSetup();
    pythonScript += "path = np.array([\n";
    for (const auto &vertex : _sol) {
      pythonScript += "\t[" + std::to_string(vertex.getX()) + ", " +
                      std::to_string(vertex.getY()) + "],\n";
    }
    pythonScript += "])\n";
    pythonScript +=
        "ax.plot(path[:, 0], path[:, 1], '-o', color='navy', "
        "markersize=1.0, linewidth=1.0)\n";
    const auto &vertex = _sol[i];
    Polygon<T> tmpRobot =
        _realRobot.moveToCopy(vertex.getX(), vertex.getY(), vertex.getTheta());
    pythonScript += tmpRobot.draw("path");
    if (title)
      pythonScript +=
          "plt.title(f'Resolution=" + std::to_string(_resolution) +
          ", build time={" + std::to_string(_buildTime) +
          ":.3f}s, search time={" + std::to_string(_searchTime) +
          ":.3f}s,\\npath length={" + std::to_string(getPathLength()) +
          ":.2f} total rotation={" + std::to_string(_totalRotation) +
          ":.2f}\\nDistanceWeight={" + std::to_string(_euclideanWeight) +
          ":.1f} RotationWeight={" + std::to_string(_rotationalWeight) +
          ":.1f}')\n";
    const std::string && imagePath = tmpFolderPath + "/tmp" + std::to_string(i) + ".png";
    pythonScript += "plt.savefig('" + imagePath +
                    "', dpi=300, bbox_inches='tight')\n";
    pythonScript += "plt.close(fig)\n";
    pythonScript += "image_paths.append('" + imagePath + "')\n";
  }
  pythonScript += "images = [Image.open(img) for img in image_paths]\n";
  pythonScript += "images[0].save('" + figPath +
                  "', save_all=True, append_images=images[1:], optimize=False, "
                  "duration=50, fps=60, loop=0)\n";
  pythonScript += "for img in image_paths:\n";
  pythonScript += "\tos.remove(img)\n";
  std::string pythonSavePath = tmpFolderPath + "/animation" + std::to_string(_resolution) + ".py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);
}

template <typename T>
void VisibilityGraph<T>::drawSetup() const {
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += _drawSetup();
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";

  std::string pythonSavePath = "setup" + std::to_string(_resolution) + ".py";
  std::ofstream pythonFile(pythonSavePath);
  pythonFile << pythonScript;
  pythonFile.close();
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + pythonSavePath;
  int status = system(command.c_str());
  if (status != 0) throw std::runtime_error("Python script failed");
}

template <typename T>
std::string VisibilityGraph<T>::_drawSetup() const {
  std::string pythonScript;
  pythonScript += _border.draw("obs");
  for (const auto &obs : _obstacles) {
    pythonScript += obs.draw("obs");
  }
  // start and goal
  if(_start.hasTheta()){
    Polygon<T> tmpRobot = _realRobot.moveToCopy(_start.getX(), _start.getY(), _start.getTheta());
    pythonScript += tmpRobot.draw("start");
  }
  if(_goal.hasTheta()){
    Polygon<T> tmpRobot = _realRobot.moveToCopy(_goal.getX(), _goal.getY(), _goal.getTheta());
    pythonScript += tmpRobot.draw("goal");
  }
  return pythonScript;
}

template <typename T>
void VisibilityGraph<T>::debugMaxPropagation() {
  int n = _layers.size();
  std::unordered_map<int, std::vector<int>> maxPropagation;
  for (int i = 0; i < n; i++)
    maxPropagation.insert(std::make_pair(i, std::vector<int>{0, 0}));

  for (int i = 0; i < n; i++) {
    const auto &vertices = _layerVertices[i];
    for (const auto &vertex : vertices) {
      int vertexLayer = vertex.first;
      if (vertexLayer == i) continue;
      if (vertexLayer < i) {
        maxPropagation[vertexLayer][0] =
            std::max(maxPropagation[vertexLayer][0], i);
      } else {
        maxPropagation[vertexLayer][1] =
            std::min(maxPropagation[vertexLayer][1], i);
      }
    }
  }
  for (int i = 0; i < n; i++) {
    Utils::print("Layer", i, "#vertices:", _layerVertices[i].size(),
                 "Max forward propagation:", maxPropagation[i][0],
                 "Max backward propagation:", maxPropagation[i][1]);
  }
}

template <typename T>
T VisibilityGraph<T>::getTotalRotation() const {
  return _totalRotation;
}

template <typename T>
void VisibilityGraph<T>::_interpolation(int density) {
  std::vector<Vertex<T>> interpolatedPath;
  for (size_t i = 0; i < _sol.size() - 1; i++) {
    const auto &start = _sol[i];
    const auto &goal = _sol[i + 1];
    T dx = (goal.getX() - start.getX()) / density;
    T dy = (goal.getY() - start.getY()) / density;
    T dTheta = 0;
    // check if the angle is in the range of [0, 2*PI/resolution] and
    // [2*PI-2*PI/resolution, 2*PI] else if(goal.getTheta()-start.getTheta() <
    // -2*PI/_resolution)
    dTheta = (goal.getTheta() - start.getTheta()) / density;

    for (int j = 0; j < density; j++) {
      T x = start.getX() + j * dx;
      T y = start.getY() + j * dy;
      T theta = start.getTheta() + j * dTheta;
      interpolatedPath.emplace_back(x, y, start.getThetaLb(),
                                    start.getThetaUb(), theta, _roundUpTheta);
    }
  }
  interpolatedPath.push_back(_sol.back());
  _sol = interpolatedPath;
}

template <typename T>
void VisibilityGraph<T>::_unwrapPath() {
  for (size_t i = 0; i < _sol.size() - 1; i++) {
    auto &start = _sol[i];
    auto &goal = _sol[i + 1];
    T dTheta = goal.getTheta() - start.getTheta();
    if (dTheta > _roundUpTheta / 2.) {
      dTheta -= _roundUpTheta;
    } else if (dTheta < -_roundUpTheta / 2.) {
      dTheta += _roundUpTheta;
    }
    goal.setTheta(start.getTheta() + dTheta);
  }
}

template <typename T>
void VisibilityGraph<T>::drawLayers3D(const std::string &figPath,
                                      const std::string &type, bool show) {
  std::string pythonScript;
  PYTHON_IMPORTS_3D(pythonScript)
  for (size_t i = 0; i < _layers.size(); i++) {
    const auto &layer = _layers[i];
    pythonScript += layer.drawGrownObs3D(type);
  }
  pythonScript +=
      "plt.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  if (show) pythonScript += "plt.show()\n";
  std::string pythonSavePath = "layers3D" + std::to_string(_resolution) + ".py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);
}

template <typename T>
std::string VisibilityGraph<T>::drawLayers3D(const std::string &type) const {
  std::string pythonScript;
  for (size_t i = 0; i < _layers.size(); i++) {
    const Layer<T> &layer = _layers[i];
    pythonScript += layer.drawGrownObs3D(type);
  }
  return pythonScript;
}

template <typename T>
std::string VisibilityGraph<T>::draw3D(bool graph) const {
#define SEMI_ALGEBRAIC_SET
  std::string pythonScript;
  PYTHON_IMPORTS_3D(pythonScript)
  Polygon<T> tmpRobot =
      _realRobot.moveToCopy(_start.getX(), _start.getY(), _start.getTheta());
  pythonScript += tmpRobot.draw3D("start");
  tmpRobot =
      _realRobot.moveToCopy(_goal.getX(), _goal.getY(), _goal.getTheta());
  pythonScript += tmpRobot.draw3D("goal");
  if (graph) {
    for (const auto &edge : _graph.getEdges()) {
      std::vector<T> edgeX = edge.getX(), edgeY = edge.getY(),
                     edgeTheta = edge.getTheta();
#ifdef SEMI_ALGEBRAIC_SET
      pythonScript +=
          "ax.plot([" + std::to_string(edgeX[0]) + ", " +
          std::to_string(edgeX[1]) + "], [" + std::to_string(edgeY[0]) + ", " +
          std::to_string(edgeY[1]) + "], [" + std::to_string(edgeTheta[0]) +
          ", " + std::to_string(edgeTheta[1]) +
          "], '-', color='darkcyan', linewidth=0.5)\n";
#else
      pythonScript +=
          "ax.plot([" + std::to_string(edgeX[0]) + ", " +
          std::to_string(edgeX[1]) + "], [" + std::to_string(edgeY[0]) + ", " +
          std::to_string(edgeY[1]) + "], [" + std::to_string(edgeTheta[0]) +
          ", " + std::to_string(edgeTheta[1]) +
          "], '-', color='lightcoral', linewidth=0.5)\n";
#endif
    }
    for (const auto &v : _graph.getVertices()) {
#ifdef SEMI_ALGEBRAIC_SET
      pythonScript += "ax.plot(" + std::to_string(v->getX()) + ", " +
                      std::to_string(v->getY()) + ", " +
                      std::to_string(v->getTheta()) +
                      ", 'o', color='#EC008C', markersize=0.5)\n";
#else
      pythonScript += "ax.plot(" + std::to_string(v.getX()) + ", " +
                      std::to_string(v.getY()) + ", " +
                      std::to_string(v.getTheta()) +
                      ", 'o', color='bisque', markersize=0.5)\n";
#endif
    }
  } else
#ifdef SEMI_ALGEBRAIC_SET
    pythonScript += drawLayers3D("obs3d");
#else
    pythonScript += drawLayers3D("graph");
#endif

  if (_sol.size() >= 2) {
    pythonScript += "path = np.array([\n";
    for (const auto &vertex : _sol) {
      pythonScript += "\t[" + std::to_string(vertex.getX()) + ", " +
                      std::to_string(vertex.getY()) + ", " +
                      std::to_string(vertex.getTheta()) + "],\n";
    }
    pythonScript += "])\n";
    pythonScript +=
        "ax.plot(path[:, 0], path[:, 1], path[:, 2], '-o', color='navy', "
        "markersize=1.0, linewidth=1.0)\n";
    // robot along the path
    if (_sol.size() >= 2) {
      for (size_t nodeNum = 1; nodeNum < _sol.size() - 1; nodeNum++) {
        const auto &vertex = _sol[nodeNum];
        Polygon<T> tmpRobot = _realRobot.moveToCopy(
            vertex.getX(), vertex.getY(), vertex.getTheta());
        pythonScript += tmpRobot.draw3D("path");
      }
    }
  }

  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.savefig('3D.png', dpi=500, bbox_inches='tight')\n";
  pythonScript += "plt.show()\n";
  std::string pythonSavePath = "ThreeD" + std::to_string(_resolution) + ".py";
  Utils::writeStringToFile<T>(pythonScript, pythonSavePath);
  Utils::runPythonScriptAndRemove<T>(pythonSavePath);

  return pythonScript;
}

template <typename T>
void VisibilityGraph<T>::drawVisibleAreas() const {
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  for (size_t i = 0; i < _layers.size(); i++) {
    const Layer<T> &layer = _layers[i];
    pythonScript += layer.drawVisibleAreas(6);
    break;
  }
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  Utils::writeStringToFile<T>(pythonScript, "visibleAreas.py");
  Utils::runPythonScriptAndRemove<T>("visibleAreas.py");
}

template <typename T>
const std::vector<Layer<T>> &VisibilityGraph<T>::getLayers() const {
  return _layers;
}

template <typename T>
std::vector<std::vector<
    std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>>
VisibilityGraph<T>::getEdgeLayersForward() const {
  return _edgeLayersForward;
}
template <typename T>
std::vector<std::vector<
    std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>>>
VisibilityGraph<T>::getEdgeLayersBackward() const {
  return _edgeLayersBackward;
}

template <typename T>
std::vector<std::shared_ptr<Vertex<T>>> VisibilityGraph<T>::getAdjacentVertices(
    std::shared_ptr<Vertex<T>> v) const {
  auto neighbors = _graph.getNeighbors(v);
  std::vector<std::shared_ptr<Vertex<T>>> adjacentVertices;
  adjacentVertices.insert(adjacentVertices.end(), neighbors.begin(),
                          neighbors.end());
  return adjacentVertices;
}

template <typename T>
std::vector<Polygon<T>> VisibilityGraph<T>::verticalDecomposition() {
  DECL_CGAL_CONVEX_DECOMPOSITION_TYPES_T
  std::vector<Polygon<T>> verticalCells;
  std::vector<Polygon_2> verticalCellsCGAL;
  std::vector<Polygon_2> holes;
  for (const auto &obs : _obstacles) {
    holes.emplace_back(obs.getPolygon());
  }
  Polygon_with_holes_2 env(_border.getPolygon(), holes.begin(), holes.end());
  CGAL::Polygon_vertical_decomposition_2<K> verticalDecomp;
  verticalDecomp(env, std::back_inserter(verticalCellsCGAL));
  for (const auto &cell : verticalCellsCGAL) {
    verticalCells.emplace_back(cell);
  }

  // std::string pythonScript;
  // PYTHON_IMPORTS(pythonScript)
  // pythonScript += _border.draw("obs");
  // for (const auto &obs : _obstacles) {
  //   pythonScript += obs.draw("obs");
  // }
  // for(const auto & cell: verticalCells){
  //   pythonScript += cell.draw("path_fill");
  // }
  // pythonScript += "plt.axis('equal')\n";
  // pythonScript += "plt.show()\n";
  // Utils::writeStringToFile<T>(pythonScript, "verticalDecomposition.py");
  // Utils::runPythonScriptAndRemove<T>("verticalDecomposition.py");
  return verticalCells;
}

template<typename T>
std::vector<std::shared_ptr<Vertex<T>>> VisibilityGraph<T>::getGraphVertices() const{
  const auto &vertices = _graph.getVertices();
  std::vector<std::shared_ptr<Vertex<T>>> graphVertices;
  graphVertices.insert(graphVertices.end(), vertices.begin(), vertices.end());
  return graphVertices;
}

template class VisibilityGraph<double>;
template class VisibilityGraph<float>;
}  // namespace RotationalVisibilityGraph