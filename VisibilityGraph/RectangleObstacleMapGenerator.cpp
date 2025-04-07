#include "RectangleObstacleMapGenerator.h"
#include "VisibilityCheck.h"
#include <Utils/Utils.h>

namespace RotationalVisibilityGraph {
template<typename T>
RectangleObstacleMapGenerator<T>::RectangleObstacleMapGenerator(int mapSize,
                                                                int numObstacles,
                                                                T minObstacleSize,
                                                                T maxObstacleSize,
                                                                const Polygon<T> &robot)
    : MapGenerator<T>(mapSize, numObstacles, maxObstacleSize, robot), _minObstacleSize(minObstacleSize) {}

template<typename T>
Polygon<T> RectangleObstacleMapGenerator<T>::_generateObstacle() {
  T x = this->_randomReal(0, this->_mapSize);
  T y = this->_randomReal(0, this->_mapSize);
  T width = this->_randomReal(this->_minObstacleSize, this->_maxObstacleSize);
  T height = this->_randomReal(this->_minObstacleSize, this->_maxObstacleSize);
  return Polygon<T>(
      std::vector<Vertex<T>>{
          Vertex<T>(x, y),
          Vertex<T>(x + width, y),
          Vertex<T>(x + width, y + height),
          Vertex<T>(x, y + height)
      },
      false);
}

template
class RectangleObstacleMapGenerator<double>;
template
class RectangleObstacleMapGenerator<float>;
}
