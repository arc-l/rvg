#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T

int main() {
  Vertex<T> vertex(1., 1., 0., 2. * PI, 0.);
  Vertex<T> vertex1(1., 1.);
  Vertex<T> vertex2(1., 1., PI, 2. * PI, PI);
  Vertex<T> vertex3(1, 1, PI, 2. * PI, PI);
  std::unordered_set<Vertex<T>, Vertex<T>::Hash> set;
  print(set.size());
  set.insert(vertex);
  set.insert(vertex1);
  set.insert(vertex2);
  print(set.size());
  print(*set.begin());
  return 0;
}