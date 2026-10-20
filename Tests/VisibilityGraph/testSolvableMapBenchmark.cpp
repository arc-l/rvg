#include <Utils/Utils.h>
#include <VisibilityGraph/Utils.h>
#include <VisibilityGraph/VisibilityGraph.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace RotationalVisibilityGraph;

namespace {

using T = double;

bool require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

struct DifficultyFiles {
  const char *name;
  int sampledObstacles;
};

const std::array<DifficultyFiles, 5> DIFFICULTIES = {{
    {"VeryEasy", 10},
    {"Easy", 25},
    {"Medium", 50},
    {"Hard", 75},
    {"VeryHard", 100}
}};

bool validateMap(const std::filesystem::path &path,
                 const DifficultyFiles &difficulty,
                 size_t level,
                 int sample,
                 int staticRvgThreads) {
  tinyxml2::XMLDocument document;
  if (!require(
          document.LoadFile(path.string().c_str()) == tinyxml2::XML_SUCCESS,
          "Unable to load " + path.string()
      )) {
    return false;
  }

  const tinyxml2::XMLElement *environment =
      document.RootElement()->FirstChildElement("environment");
  const tinyxml2::XMLElement *metadata =
      environment == nullptr
          ? nullptr
          : environment->FirstChildElement("benchmarkMetadata");
  const std::uint32_t expectedSeed =
      20260720 + static_cast<std::uint32_t>(
          level * 100 + sample
      );
  if (!require(environment != nullptr, "Missing environment in " + path.string()) ||
      !require(
          Utils::get<bool>(*environment, "useBoundary", false),
          "Custom boundary disabled in " + path.string()
      ) ||
      !require(metadata != nullptr, "Missing metadata in " + path.string()) ||
      !require(
          std::string(metadata->Attribute("difficulty")) == difficulty.name,
          "Difficulty metadata mismatch in " + path.string()
      ) ||
      !require(
          metadata->IntAttribute("sampledObstacleCount") ==
              difficulty.sampledObstacles,
          "Sampled obstacle count mismatch in " + path.string()
      ) ||
      !require(
          metadata->UnsignedAttribute("seed") == expectedSeed,
          "Seed metadata mismatch in " + path.string()
      ) ||
      !require(
          metadata->IntAttribute("staticRvgResolution") == 8,
          "Static RVG resolution metadata mismatch in " + path.string()
      )) {
    return false;
  }

  const Polygon<T> robot = Utils::getRobot<T>(document);
  const Polygon<T> border = Utils::getBoundary<T>(document);
  const std::vector<Polygon<T>> obstacles = Utils::getObstacles<T>(document);
  if (!require(border.getPolygon().is_simple(),
               "Non-simple boundary in " + path.string()) ||
      !require(border.getPolygon().is_counterclockwise_oriented(),
               "Clockwise boundary in " + path.string()) ||
      !require(
          metadata->IntAttribute("interiorObstacleCount") ==
              static_cast<int>(obstacles.size()),
          "Interior obstacle metadata mismatch in " + path.string()
      )) {
    return false;
  }
  for (size_t index = 0; index < obstacles.size(); ++index) {
    typename Polygon<T>::Polygon_set_2 outside(
        obstacles[index].getCounterClockWise()
    );
    outside.difference(
        typename Polygon<T>::Polygon_set_2(border.getCounterClockWise())
    );
    if (!require(outside.is_empty(),
                 "Obstacle outside boundary in " + path.string())) {
      return false;
    }
    for (size_t other = index + 1; other < obstacles.size(); ++other) {
      if (!require(
              !obstacles[index].intersects(obstacles[other]),
              "Unmerged obstacle intersection in " + path.string()
          )) {
        return false;
      }
    }
  }

  const tinyxml2::XMLElement *plannerSettings =
      document.RootElement()->FirstChildElement("plannerSettings");
  std::shared_ptr<Vertex<T>> start = Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("start")->FirstChildElement("Vertex")
  );
  std::shared_ptr<Vertex<T>> goal = Utils::getVertex<T>(
      *plannerSettings->FirstChildElement("goal")->FirstChildElement("Vertex")
  );
  VisibilityGraph<T> planner(
      robot,
      border,
      obstacles,
      8,
      staticRvgThreads
  );
  planner.getGraph().setWeight(1.0, 0.1);
  if (!require(planner.addVertex(start),
               "Static RVG rejected start in " + path.string()) ||
      !require(planner.addVertex(goal),
               "Static RVG rejected goal in " + path.string())) {
    return false;
  }
  return require(
      !planner.getGraph().shortestPath(start, goal, false).empty(),
      "Static RVG found no path in " + path.string()
  );
}

}

int main() {
  const std::filesystem::path directory = RVG_BASELINE_CONFIG_DIR;
  const int staticRvgThreads = Utils::availableProcessorCount();
  size_t validated = 0;
  for (size_t level = 0; level < DIFFICULTIES.size(); ++level) {
    const DifficultyFiles &difficulty = DIFFICULTIES[level];
    for (int sample = 0; sample < 4; ++sample) {
      const std::filesystem::path path =
          directory /
          ("mapfixedStartAndGoal" + std::string(difficulty.name) +
           "Small" + std::to_string(sample) + ".xml");
      if (!validateMap(
              path,
              difficulty,
              level,
              sample,
              staticRvgThreads
          )) {
        return 1;
      }
      ++validated;
    }
  }
  return require(validated == 20, "Benchmark must contain 20 validated maps")
      ? 0
      : 1;
}
