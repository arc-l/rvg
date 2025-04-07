#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_

#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Edge.h>
#include <VisibilityGraph/Polygon.h>
#include <unordered_map>
#include <unordered_set>
#include <Utils/Utils.h>
#include <Utils/Pragma.h>

namespace RotationalVisibilityGraph {

template<typename T>
class Graph {
 public:
  typedef typename Vertex<T>::Hash VertexHash;
  typedef typename Vertex<T>::SharedPtrVertexHash SharedPtrVertexHash;
  typedef typename Vertex<T>::SharedPtrVertexEqual SharedPtrVertexEqual;
  Graph();
  void addEdge(std::shared_ptr<Vertex<T>> v1, std::shared_ptr<Vertex<T>> v2);
  void addEdges(const std::vector<std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>> &edges);
  const std::unordered_set<std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual> &getNeighbors(std::shared_ptr<Vertex<T>> v) const;
  const std::unordered_map<std::shared_ptr<Vertex<T>>, std::unordered_set<std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual>, SharedPtrVertexHash, SharedPtrVertexEqual> &getAdjacencyList() const;
  const std::unordered_set<Edge<T>, typename Edge<T>::Hash> &getEdges() const;
  const std::unordered_set<std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual> &getVertices() const;
  int size() const;
  void setWeight(T euclideanWeight, T rotationalWeight);
  std::vector<Vertex<T>> shortestPath(std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> goal, bool verbose) const;
//  bool writeGraph(const std::string &filename) const;
  int numConnectedComponents() const;
  void draw(const std::string &figPath, bool show=false);
  void draw3D(const std::string &figPath, bool show=false);
  void buildEdges();
  std::string draw() const;
  ~Graph() = default;

 private:
  T _euclideanWeight, _rotationalWeight;
  std::unordered_map<std::shared_ptr<Vertex<T>>, 
                    std::unordered_set<std::shared_ptr<Vertex<T>>, 
                    SharedPtrVertexHash, 
                    SharedPtrVertexEqual>, 
                    SharedPtrVertexHash, 
                    SharedPtrVertexEqual> _adjacencyList;
  std::unordered_set<Edge<T>, typename Edge<T>::Hash> _edges;
  std::unordered_set<std::shared_ptr<Vertex<T>>, SharedPtrVertexHash, SharedPtrVertexEqual> _vertices;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPH_H_
