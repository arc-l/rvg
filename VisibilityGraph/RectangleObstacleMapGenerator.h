#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_RECTANGLEOBSTACLEMAPGENERATOR_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_RECTANGLEOBSTACLEMAPGENERATOR_H_

#include <VisibilityGraph/MapGenerator.h>

namespace RotationalVisibilityGraph {
template<typename T>
class RectangleObstacleMapGenerator : public MapGenerator<T> {
 public:
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T
  using MapGenerator<T>::_mapSize;
  using MapGenerator<T>::_numberObstacles;
  using MapGenerator<T>::_maxObstacleSize;
  using MapGenerator<T>::_robot;
  using MapGenerator<T>::_obstacles;
  using MapGenerator<T>::_start;
  using MapGenerator<T>::_goal;
  using MapGenerator<T>::_border;
  using MapGenerator<T>::_robotRadius;
  using MapGenerator<T>::_marginSize;

  explicit RectangleObstacleMapGenerator(int mapSize,
                                         int numObstacles,
                                         T minObstacleSize,
                                         T maxObstacleSize,
                                         const Polygon<T> &robot);
 protected:
  Polygon<T> _generateObstacle() override;
 private:
  T _minObstacleSize;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_RECTANGLEOBSTACLEMAPGENERATOR_H_
