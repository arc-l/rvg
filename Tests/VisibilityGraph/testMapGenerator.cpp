#include <Utils/Utils.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/Utils.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace RotationalVisibilityGraph;

namespace {

using T = double;

bool require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

Polygon<T> polygon(std::initializer_list<std::pair<T, T>> coordinates) {
  std::vector<Vertex<T>> vertices;
  vertices.reserve(coordinates.size());
  for (const auto &[x, y] : coordinates) {
    vertices.emplace_back(x, y);
  }
  return Polygon<T>(vertices, false);
}

Polygon<T> robot() {
  return polygon({
      {-0.5, -0.25},
      {0.5, -0.25},
      {0.5, 0.25},
      {-0.5, 0.25}
  });
}

class ExposedMapGenerator : public MapGenerator<T> {
 public:
  using MapGenerator<T>::MapGenerator;

  int integrate(const Polygon<T> &obstacle) {
    const ObstaclePlacement placement = _integrateObstacle(obstacle);
    if (placement == ObstaclePlacement::Interior) {
      return 1;
    }
    if (placement == ObstaclePlacement::Border) {
      return 2;
    }
    return 0;
  }

  void sampleObstacles() {
    reset();
    _generateObstacles(false);
  }

  bool insideBorder(const Polygon<T> &candidate) const {
    return _polygonIsInsideBorder(candidate, _border);
  }
};

std::string geometrySignature(const MapGenerator<T> &generator) {
  std::ostringstream signature;
  signature << std::setprecision(17);
  const auto appendPolygon = [&](const Polygon<T> &candidate) {
    signature << '[';
    for (const Vertex<T> &vertex : candidate.getVertices()) {
      signature << vertex.getX() << ',' << vertex.getY() << ';';
    }
    signature << ']';
  };
  appendPolygon(generator.getBorder());
  for (const Polygon<T> &obstacle : generator.getObstacles()) {
    appendPolygon(obstacle);
  }
  return signature.str();
}

bool testProfiles() {
  const std::array<MapDifficulty, 5> difficulties = {
      MapDifficulty::VeryEasy,
      MapDifficulty::Easy,
      MapDifficulty::Medium,
      MapDifficulty::Hard,
      MapDifficulty::VeryHard
  };
  const std::array<int, 5> counts = {10, 25, 50, 75, 100};
  const std::array<double, 5> sizes = {6.0, 4.875, 3.75, 2.625, 1.5};
  for (size_t index = 0; index < difficulties.size(); ++index) {
    const MapDifficultyProfile profile =
        MapGenerator<T>::getDifficultyProfile(difficulties[index]);
    if (!require(profile.obstacleCount == counts[index],
                 "difficulty obstacle count mismatch") ||
        !require(std::abs(profile.maxObstacleSize - sizes[index]) < 1e-12,
                 "difficulty maximum size mismatch") ||
        !require(
            std::abs(
                profile.normalizedDifficulty -
                static_cast<double>(index) / 4.0
            ) < 1e-12,
            "normalized difficulty mismatch"
        )) {
      return false;
    }
    if (index > 0 &&
        (!require(counts[index] > counts[index - 1],
                  "obstacle counts must increase") ||
         !require(sizes[index] < sizes[index - 1],
                  "maximum obstacle sizes must decrease"))) {
      return false;
    }
  }

  ExposedMapGenerator scaled(
      60,
      MapDifficulty::Medium,
      17,
      robot()
  );
  return require(scaled.getRequestedObstacleCount() == 200,
                 "difficulty count must scale by map area") &&
      require(std::abs(scaled.getMaxObstacleSize() - 7.5) < 1e-12,
              "difficulty size must scale by map width");
}

bool testObstacleClassification() {
  ExposedMapGenerator interiorGenerator(20, 0, 2.0, robot(), 11);
  const Polygon<T> interior = polygon({
      {-1, -1},
      {1, -1},
      {1, 1},
      {-1, 1}
  });
  if (!require(interiorGenerator.integrate(interior) == 1,
               "interior obstacle was not retained") ||
      !require(interiorGenerator.getObstacles().size() == 1,
               "interior obstacle count mismatch") ||
      !require(interiorGenerator.getBorder().size() == 4,
               "interior obstacle changed the border")) {
    return false;
  }

  ExposedMapGenerator borderGenerator(20, 0, 2.0, robot(), 12);
  const Polygon<T> crossing = polygon({
      {9, -2},
      {11, -2},
      {11, 2},
      {9, 2}
  });
  return require(borderGenerator.integrate(crossing) == 2,
                 "crossing obstacle was not absorbed into the border") &&
      require(borderGenerator.getObstacles().empty(),
              "crossing obstacle remained in the obstacle list") &&
      require(borderGenerator.getBorder().size() > 4,
              "crossing obstacle did not indent the border") &&
      require(borderGenerator.getBorderCrossingCount() == 1,
              "border-crossing metadata mismatch");
}

bool testSeededSampling() {
  ExposedMapGenerator first(30, 12, 4.0, robot(), 12345);
  ExposedMapGenerator second(30, 12, 4.0, robot(), 12345);
  ExposedMapGenerator different(30, 12, 4.0, robot(), 12346);
  first.sampleObstacles();
  second.sampleObstacles();
  different.sampleObstacles();
  if (!require(geometrySignature(first) == geometrySignature(second),
               "same seed did not reproduce map geometry") ||
      !require(geometrySignature(first) != geometrySignature(different),
               "different seeds produced identical geometry") ||
      !require(first.getSampledObstacleCount() == 12,
               "sampled obstacle count mismatch")) {
    return false;
  }
  for (const Polygon<T> &obstacle : first.getObstacles()) {
    if (!require(first.insideBorder(obstacle),
                 "interior obstacle lies outside the final border")) {
      return false;
    }
  }
  return require(first.getBorder().getPolygon().is_simple(),
                 "generated border must be simple") &&
      require(first.getBorder().getPolygon().is_counterclockwise_oriented(),
              "generated border must be counter-clockwise");
}

bool testXmlBoundaryAndMetadata() {
  ExposedMapGenerator generator(
      30,
      MapDifficulty::VeryEasy,
      20260720,
      robot()
  );
  generator.sampleObstacles();
  const std::filesystem::path path = "/tmp/rvg_map_generator_test.xml";
  generator.writeMap(path.string());

  tinyxml2::XMLDocument document;
  if (!require(document.LoadFile(path.string().c_str()) == tinyxml2::XML_SUCCESS,
               "generated XML could not be loaded")) {
    return false;
  }
  const tinyxml2::XMLElement *environment =
      document.RootElement()->FirstChildElement("environment");
  const tinyxml2::XMLElement *metadata =
      environment->FirstChildElement("benchmarkMetadata");
  const bool valid =
      require(
          Utils::get<bool>(*environment, "useBoundary", false),
          "generated XML must enable custom boundary"
      ) &&
      require(environment->FirstChildElement("boundary") != nullptr,
              "generated XML is missing boundary") &&
      require(metadata != nullptr, "generated XML is missing benchmark metadata") &&
      require(std::string(metadata->Attribute("difficulty")) == "VeryEasy",
              "difficulty metadata mismatch") &&
      require(metadata->UnsignedAttribute("seed") == 20260720,
              "seed metadata mismatch") &&
      require(Utils::getBoundary<T>(document).getPolygon().is_simple(),
              "round-tripped boundary is not simple");
  std::filesystem::remove(path);
  return valid;
}

}

int main() {
  if (!testProfiles() ||
      !testObstacleClassification() ||
      !testSeededSampling() ||
      !testXmlBoundaryAndMetadata()) {
    return 1;
  }
  return 0;
}
