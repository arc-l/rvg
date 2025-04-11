#include <unordered_map>
#include <VisibilityGraph/Vertex.h>
#include <Utils/Utils.h>

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;

int main() {
  Vertex<float> vertex(1., 1., 0., 2. * PI, 0);
  print(vertex);
  Vertex<double> vertex1(0, 0);
  print(vertex1);
  vertex1.setBounds(0, 1.0 * PI);
  print(vertex1);
  auto [coordX, coordY] = vertex.getCoord();
  print(coordX, coordY);
  print(vertex);
  vertex.setTheta(PI);
  std::unordered_map<Vertex<float>, int, Vertex<float>::Hash> map;
  map.insert({vertex, 1});
  Vertex<float> vertex2(3, 3);
  map.insert({vertex2, 2});
  for (const auto &pair : map) {
    print(pair.first, pair.second);
  }
  return 0;
}