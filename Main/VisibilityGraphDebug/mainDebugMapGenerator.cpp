#include <Utils/Utils.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/RectangleObstacleMapGenerator.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main() {
  Polygon<T> robot =
      Polygon<T>(
          std::vector<Vertex<T>>{
              Vertex<T>(-3, 0.5),
              Vertex<T>(-2, 0.5),
              Vertex<T>(-2, 2),
              Vertex<T>(-3, 2)
          },
          false);
  std::shared_ptr<MapGenerator<T>> mapGenerator = std::make_shared<RectangleObstacleMapGenerator<T>>(50, 10, 6, 10, robot);
  mapGenerator->generateMap();
  mapGenerator->drawMap("test");
  mapGenerator->writeMap("../Configs/mapTest.xml");
  return 0;
}