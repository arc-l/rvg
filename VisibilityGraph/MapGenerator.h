#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_

#include <cstdint>
#include <random>
#include <string>
#include <Utils/Utils.h>
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

enum class MapDifficulty {
  VeryEasy,
  Easy,
  Medium,
  Hard,
  VeryHard
};

struct MapDifficultyProfile {
  MapDifficulty difficulty;
  double normalizedDifficulty;
  int obstacleCount;
  double maxObstacleSize;
};

template<typename T>
class MapGenerator {
 public:
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
  DECL_CGAL_VISIBILITY_GRAPH_TYPES_T
  MapGenerator(int mapSize, int numObstacles, T maxObstacleSize, const Polygon<T> &robot);
  MapGenerator(int mapSize,
               int numObstacles,
               T maxObstacleSize,
               const Polygon<T> &robot,
               std::uint32_t seed);
  MapGenerator(int mapSize,
               MapDifficulty difficulty,
               std::uint32_t seed,
               const Polygon<T> &robot);
  static MapDifficultyProfile getDifficultyProfile(MapDifficulty difficulty);
  static std::string difficultyName(MapDifficulty difficulty);
  virtual Vertex<T> getStart() const;
  virtual Vertex<T> getGoal() const;
  virtual std::vector<Polygon<T>> getObstacles() const;
  virtual Polygon<T> getBorder() const;
  std::uint32_t getSeed() const;
  int getRequestedObstacleCount() const;
  T getMaxObstacleSize() const;
  int getSampledObstacleCount() const;
  int getBorderCrossingCount() const;
  int getSuccessfulAttempt() const;
  int getStaticRvgResolution() const;
  virtual void drawMap(const std::string & identifier) const;
  virtual void generateMap();
  virtual void reset();
  virtual void writeMap(const std::string &filename) const;
  virtual void generateFixedStartAndGoalMap(const Vertex<T> &start, const Vertex<T> &goal);
  virtual void generateFixedStartAndGoalMap();
  virtual void generateSolvableFixedStartAndGoalMap(
      const Vertex<T> &start,
      const Vertex<T> &goal,
      int resolution = 8,
      int numThreads = Utils::availableProcessorCount(),
      bool protectEndpointBorderClearance = true
  );
  virtual void generateSolvableFixedStartAndGoalMap(
      int resolution = 8,
      int numThreads = Utils::availableProcessorCount(),
      bool protectEndpointBorderClearance = true
  );
  bool hasStaticRvgSolution(
      int resolution = 8,
      int numThreads = Utils::availableProcessorCount()
  ) const;
  void generateStartAndGoal();
 protected:
  enum class ObstaclePlacement {
    Rejected,
    Interior,
    Border
  };

  int _mapSize, _numberObstacles;
  T _maxObstacleSize, _robotRadius, _marginSize;
  std::vector<Polygon<T>> _obstacles;
  Vertex<T> _start, _goal;
  Polygon<T> _border, _robot, _margin;
  std::uint32_t _seed;
  std::mt19937 _rng;
  bool _benchmarkMode;
  bool _protectStartAndGoal;
  MapDifficulty _difficulty;
  int _sampledObstacleCount;
  int _borderCrossingCount;
  int _successfulAttempt;
  int _staticRvgResolution;
  [[nodiscard]] int _randomInt(int min, int max);
  [[nodiscard]] T _randomReal(T min, T max);
  std::vector<T> _randomVector(int size, T min, T max);
  void _generateBorder();
  virtual Polygon<T> _generateObstacle();
  virtual void _generateObstacles(bool incremental);
  ObstaclePlacement _integrateObstacle(const Polygon<T> &obstacle);
  bool _polygonIsInsideBorder(const Polygon<T> &polygon, const Polygon<T> &border) const;
  bool _hasBorderClearance(const Vertex<T> &configuration, const Polygon<T> &border) const;
  bool _outsideBorder(const Polygon<T> &obs) const;
  Vertex<T> _getLegalConfiguration();
  bool _circleIntersectsObstacle(const Polygon<T> &obs, const Vertex<T> &center, T radius) const;
  bool _isLegalConfiguration(const Vertex<T> &v) const;
  bool _isStartAndGoalVisible() const;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_MAPGENERATOR_H_
