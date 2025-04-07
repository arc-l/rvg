#include <VisibilityGraph/Edge.h>
#include <VisibilityGraph/Vertex.h>
#include <Utils/Utils.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
int main() {
  Vertex<float> v1, v2;
  v1 = Vertex<float>(1, 1);
  v2 = Vertex<float>(2, 2);
  Edge<float> edge(v1, v2);
  print(edge);
  print(edge.v1());
  print(edge.v2());
  print("Does edge have this vertex", edge.hasVertex(Vertex<float>(1, 1)));
  print("Does edge have this vertex", edge.hasVertex(Vertex<float>(2, 2)));
  print("Does edge have this vertex", edge.hasVertex(Vertex<float>(3, 3)));
  print("The adjacent vertex:", edge.getAdjacent(Vertex<float>(1, 1)));
  print("The adjacent vertex:", edge.getAdjacent(Vertex<float>(2, 2)));
  print("The length of the edge: ", edge.length());
  v1.setPos(0, 0);
  print("The length of the edge: ", edge.length());
  Vertex<float> v3(1, 2);
  Vertex<float> v4(2, 1);
  Edge<float> edge1(v3, v4);
  print(edge1);
  print("Equal?", edge == edge1);
  print("Equal?", edge != edge1);
  print(edge.intersects(edge1));
  print(edge.intersectionPoint(edge1));
  print("The length of the edge: ", edge.length());
  std::shared_ptr<Vertex<float>> v5 = std::make_shared<Vertex<float>>(1, 1);
  std::shared_ptr<Vertex<float>> v6 = std::make_shared<Vertex<float>>(2, 2);
  Edge<float> edge2(*v5, *v6);
  print(edge2);
  print("The length of the edge: ", edge2.length());
  v5->setPos(0, 0);
  print("The length of the edge: ", edge2.length());
  return 0;
}