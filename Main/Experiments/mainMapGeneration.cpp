#include <Utils/Utils.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/RectangleObstacleMapGenerator.h>
#include <VisibilityGraph/Vertex.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main(int argc, char *argv[]) {
  bool randomStartAndGoal = false;
  bool hard = true;
  bool small = true;
  Polygon<T> robot = Polygon<T>(std::vector<Vertex<T>>{Vertex<T>(-3, 0.5), Vertex<T>(-2, 0.5), Vertex<T>(-2, 2), Vertex<T>(-3, 2)}, false);

  int mapSize = small ? 30: 50;
  int numberOfObstacles = hard ? 100 : 10;
  double minObstacleSize = hard ? 0.5 : 3;
  double maxObstacleSize = hard ? 1.5 : 6;
  std::shared_ptr<MapGenerator<T>> mapGeneratorRec = std::make_shared<RectangleObstacleMapGenerator<T>>(mapSize, numberOfObstacles, minObstacleSize, maxObstacleSize, robot);
  std::shared_ptr<MapGenerator<T>> mapGeneratorRandom = std::make_shared<MapGenerator<T>>(mapSize, numberOfObstacles, maxObstacleSize, robot);

  mapGeneratorRandom->generateFixedStartAndGoalMap();
  std::string path = hard?"../Configs/HardDemo.xml":"../Configs/EasyDemo.xml";
  mapGeneratorRandom->writeMap(path);

  // for (int i = 1; i < 2; i++) {
  //   if (randomStartAndGoal)
  //     mapGeneratorRandom->generateMap();
  //   else
  //     mapGeneratorRandom->generateFixedStartAndGoalMap();
  //   mapGeneratorRandom->drawMap(std::to_string(i));
  //   std::string postfix = randomStartAndGoal ? "randomStartAndGoal" : "fixedStartAndGoal";
  //   std::string size = small ? "Small" : "";
  //   std::string difficulty = hard ? "Hard" : "";
  //   mapGeneratorRandom->writeMap("../Configs/map" + postfix + difficulty + size + std::to_string(i) + ".xml");
  // }

  // for (int i = 5; i < 10; i++) {
    // if (randomStartAndGoal)
      // mapGeneratorRec->generateMap();
    // else
      // mapGeneratorRec->generateFixedStartAndGoalMap();
    // mapGeneratorRec->drawMap(std::to_string(i));
    // std::string postfix = randomStartAndGoal ? "randomStartAndGoal" : "fixedStartAndGoal";
    // std::string size = small ? "Small" : "";
    // std::string difficulty = hard ? "Hard" : "";
    // mapGeneratorRec->writeMap("../Configs/map" + postfix + difficulty + size + std::to_string(i) + ".xml");
  // }

  return 0;
}
