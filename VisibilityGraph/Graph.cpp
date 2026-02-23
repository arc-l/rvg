#include "Graph.h"
#include <VisibilityGraph/RotationMinimization.h>

namespace RotationalVisibilityGraph {
template<typename T>
Graph<T>::Graph() = default;

template<typename T>
void Graph<T>::setWeight(T euclideanWeight, T rotationalWeight) {
  _euclideanWeight = euclideanWeight;
  _rotationalWeight = rotationalWeight;
  // _maxDist = _maxRotation = 0.0;
}

template<typename T>
void Graph<T>::addEdge(std::shared_ptr<Vertex<T>> v1, std::shared_ptr<Vertex<T>> v2) {
  ASSERT_MSG(std::isnan(v1->getX()) == false || std::isnan(v1->getY()) == false, "Vertex 1 is NaN");
  ASSERT_MSG(std::isnan(v2->getX()) == false || std::isnan(v2->getY()) == false, "Vertex 2 is NaN");
  {
    _vertices.insert(v1);
    _vertices.insert(v2);
    _adjacencyList[v1].insert(v2);
    _adjacencyList[v2].insert(v1);

  //  if (v1 < v2) {
  //    Edge<T> newEdge(*v1, *v2);
  //    _edges.insert(newEdge);
  //  } else {
  //    Edge<T> newEdge(*v2, *v1);
  //    _edges.insert(newEdge);
  //  }
  }
}

template<typename T>
const std::unordered_set<std::shared_ptr<Vertex<T>>, typename Graph<T>::SharedPtrVertexHash, typename Graph<T>::SharedPtrVertexEqual> &Graph<T>::getNeighbors(std::shared_ptr<Vertex<T>> v) const {
  return _adjacencyList.at(v);
}


template<typename T>
int Graph<T>::size() const {
  return _adjacencyList.size();
}

template<typename T>
const std::unordered_map<std::shared_ptr<Vertex<T>>, std::unordered_set<std::shared_ptr<Vertex<T>>, typename Graph<T>::SharedPtrVertexHash, typename Graph<T>::SharedPtrVertexEqual>, typename Graph<T>::SharedPtrVertexHash, typename Graph<T>::SharedPtrVertexEqual> &Graph<T>::getAdjacencyList() const {
  return _adjacencyList;
}

template<typename T>
const std::unordered_set<Edge<T>, typename Edge<T>::Hash> &Graph<T>::getEdges() const {
  return _edges;
}

template<typename T>
const std::unordered_set<std::shared_ptr<Vertex<T>>, typename Graph<T>::SharedPtrVertexHash, typename Graph<T>::SharedPtrVertexEqual> &Graph<T>::getVertices() const {
  return _vertices;
}

template<typename T>
std::vector<Vertex<T>> Graph<T>::shortestPath(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal, bool verbose) const {
  if(!(_adjacencyList.find(start) != _adjacencyList.end() && _adjacencyList.find(goal) != _adjacencyList.end()))
  {
    if(verbose)
      Utils::print("Start or Goal vertex not in graph");
    return std::vector<Vertex<T>>();
  }
  // Uniform cost search
  std::priority_queue<std::pair<T, std::shared_ptr<Vertex<T>>>, std::vector<std::pair<T, std::shared_ptr<Vertex<T>>>>, std::greater<>> pq;
  std::unordered_map<std::shared_ptr<Vertex<T>>, T, SharedPtrVertexHash, SharedPtrVertexEqual> dist;
  std::unordered_map<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual> parent;
  for (const auto &vertex : _vertices) {
    dist[vertex] = std::numeric_limits<T>::max();
  }
  dist[start] = 0;
  pq.push({0, start});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) {
      continue;
    }
    if (*u == *goal) {
      break;
    }
    for (const auto &v : _adjacencyList.at(u)) {
      T w = _euclideanWeight * u->dist(*v) + _rotationalWeight * u->rotationalDist(*v);
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        pq.push({dist[v], v});
        parent[v] = u;
      }
    }
  }

  std::vector<std::shared_ptr<Vertex<T>>> pathPtrs;
  std::vector<Vertex<T>> path;
  // we need to handle when goal is not reachable
  if (dist[goal] == std::numeric_limits<T>::max()) {
    return path;
  }
  std::shared_ptr<Vertex<T>> curr = goal;
  while (curr != start) {
    pathPtrs.push_back(curr);
    curr = parent[curr];
  }
  pathPtrs.push_back(start);
  std::reverse(pathPtrs.begin(), pathPtrs.end());
  for (const auto &vertex : pathPtrs) {
    path.push_back(*vertex);
  }

#ifdef USE_IPOPT
  std::vector<std::pair<T, T>> theta_ranges;
  for (size_t i = 0; i < path.size(); i++) {
    theta_ranges.emplace_back(path[i].getThetaLb(), path[i].getThetaUb());
  }
  Ipopt::SmartPtr<RotationMinimization<T>> mynlp = new RotationMinimization<T>(theta_ranges);
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-3);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("max_iter", 10); // Maximum number of iterations
  Ipopt::ApplicationReturnStatus status = app->Initialize();
  status = app->OptimizeTNLP(mynlp);
  std::vector<T> thetas = mynlp->getSolution();
  for (size_t i = 0; i < path.size(); i++) {
    path[i].setTheta(thetas[i]);
  }
#endif
  return path;
}

template<typename T>
int Graph<T>::numConnectedComponents() const {
  std::unordered_set<std::shared_ptr<Vertex<T>>> visited;
  int numComponents = 0;
  for (const auto &vertex : _vertices) {
    if (visited.find(vertex) == visited.end()) {
      numComponents++;
      std::queue<std::shared_ptr<Vertex<T>>> q;
      q.push(vertex);
      visited.insert(vertex);
      while (!q.empty()) {
        auto u = q.front();
        q.pop();
        for (const auto &v : _adjacencyList.at(u)) {
          if (visited.find(v) == visited.end()) {
            q.push(v);
            visited.insert(v);
          }
        }
      }
    }
  }
  return numComponents;
}

template<typename T>
void Graph<T>::draw(const std::string &figPath, bool show) {
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  // different colors for different connected components
  std::unordered_map<std::shared_ptr<Vertex<T>>, int> component;
  int numComponents = 0;
  for (const auto &vertex : _vertices) {
    if (component.find(vertex) == component.end()) {
      numComponents++;
      std::queue<std::shared_ptr<Vertex<T>>> q;
      q.push(vertex);
      component[vertex] = numComponents;
      while (!q.empty()) {
        auto u = q.front();
        q.pop();
        for (const auto &v : _adjacencyList.at(u)) {
          if (component.find(v) == component.end()) {
            q.push(v);
            component[v] = numComponents;
          }
        }
      }
    }
  }
  std::vector<std::string> colors = {"tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown",
                                     "tab:pink", "tab:gray", "tab:olive", "tab:cyan"};
  for (const auto &edge : _edges) {
    pythonScript += "ax.plot([" + std::to_string(edge.v1().getX()) + ", " + std::to_string(edge.v2().getX()) + "], ["
        + std::to_string(edge.v1().getY()) + ", " + std::to_string(edge.v2().getY())
        + "], color='black', linewidth=0.5)\n";
  }
  for (const auto &vertex : _vertices) {
    pythonScript += "ax.plot(" + std::to_string(vertex->getX()) + ", " + std::to_string(vertex->getY()) + ", 'o', color='"
        + colors[component[vertex] - 1] + "', markersize=1)\n";
  }
  pythonScript += "plt.axis('off')\n";
  // pythonScript += "plt.savefig('" + figPath + "', bbox_inches='tight', dpi=500)\n";
  if (show) {
    pythonScript += "plt.show()\n";
  }

  Utils::writeStringToFile<T>(pythonScript, "draw.py");
  Utils::runPythonScriptAndRemove<T>("draw.py");
}

template<typename T>
void Graph<T>::draw3D(const std::string &figPath, bool show){
  std::string pythonScript;
  PYTHON_IMPORTS_3D(pythonScript)
  for (const auto &edge : _edges) {
    pythonScript += "ax.plot([" + std::to_string(edge.v1().getX()) + ", " + std::to_string(edge.v2().getX()) + "], ["
        + std::to_string(edge.v1().getY()) + ", " + std::to_string(edge.v2().getY())
        + "], [" + std::to_string(edge.v1().getTheta()) + ", " + std::to_string(edge.v2().getTheta()) + "], color='black', linewidth=0.5)\n";
  }
  pythonScript += "plt.savefig('" + figPath + "', bbox_inches='tight', dpi=500)\n";
  if (show)
  {
    pythonScript += "plt.show()\n";
  }
  
  Utils::writeStringToFile<T>(pythonScript, "draw.py");
  Utils::runPythonScriptAndRemove<T>("draw.py");
}

template<typename T>
std::string Graph<T>::draw() const {
  std::string pythonScript;
  for (const auto &edge : this->getEdges()) {
    std::vector<T> edgeX = edge.getX(), edgeY = edge.getY();
    pythonScript +=
        "ax.plot([" + std::to_string(edgeX[0]) + ", " + std::to_string(edgeX[1]) + "], [" + std::to_string(edgeY[0])
            + ", " + std::to_string(edgeY[1]) + "], '-', color='lightcoral', linewidth=0.5)\n";
  }
  for (const auto &v : this->getVertices()) {
    pythonScript +=
        "ax.plot(" + std::to_string(v->getX()) + ", " + std::to_string(v->getY())
            + ", 'o', color='bisque', markersize=0.5)\n";
  }
  return pythonScript;
}

template<typename T>
void Graph<T>::addEdges(const std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &edges) {
  for (const auto &edge : edges) {
    addEdge(edge.first, edge.second);
  }
}

template<typename T>
void Graph<T>::mergeGraph(const Graph<T> &other) {
  for (const auto &entry : other.getAdjacencyList()) {
    const auto &v = entry.first;
    const auto &neighbors = entry.second;
    for (const auto &neighbor : neighbors) {
      addEdge(v, neighbor);
    }
  }
}

template<typename T>
void Graph<T>::buildEdges(){
  for (const auto &vertex : _vertices) {
    const auto & neighbors = _adjacencyList.at(vertex);
    for (const auto & neighbor : neighbors) {
      Edge<T> newEdge(*vertex, *neighbor);
      _edges.insert(newEdge);
    }
  }
}

template
class Graph<double>;
template
class Graph<float>;

}// namespace RotationalVisibilityGraph
