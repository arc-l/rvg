#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_

#include <random>
#include <VisibilityGraph/Polygon.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

namespace RotationalVisibilityGraph {
template<typename T>
class MapGenerator {
 public:
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T
  MapGenerator(int mapSize, int numObstacles, T maxObstacleSize, const Polygon<T> &robot);
  virtual Vertex<T> getStart() const;
  virtual Vertex<T> getGoal() const;
  virtual std::vector<Polygon<T>> getObstacles() const;
  virtual Polygon<T> getBorder() const;
  virtual void drawMap(const std::string & identifier) const;
  virtual void generateMap();
  virtual void reset();
  virtual void writeMap(const std::string &filename) const;
  virtual void generateFixedStartAndGoalMap(const Vertex<T> &start, const Vertex<T> &goal);
  virtual void generateFixedStartAndGoalMap();
  void generateStartAndGoal();
 protected:
  int _mapSize, _numberObstacles;
  T _maxObstacleSize, _robotRadius, _marginSize;
  std::vector<Polygon<T>> _obstacles;
  Vertex<T> _start, _goal;
  Polygon<T> _border, _robot, _margin;
  [[nodiscard]] int _randomInt(int min, int max);
  [[nodiscard]] T _randomReal(T min, T max);
  std::vector<T> _randomVector(int size, T min, T max);
  void _generateBorder();
  virtual Polygon<T> _generateObstacle();
  virtual void _generateObstacles(bool incremental);
  bool _outsideBorder(const Polygon<T> &obs) const;
  Vertex<T> _getLegalConfiguration();
  bool _circleIntersectsObstacle(const Polygon<T> &obs, const Vertex<T> &center, T radius) const;
  bool _isLegalConfiguration(const Vertex<T> &v) const;
  bool _isStartAndGoalVisible() const;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_
