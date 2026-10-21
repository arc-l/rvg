#include <Utils/Utils.h>
#include <VisibilityGraph/MapGenerator.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>

using namespace RotationalVisibilityGraph;
using T = double;

namespace {

struct DifficultyOutput {
  MapDifficulty difficulty;
  const char *name;
};

constexpr int MAP_SIZE = 30;
constexpr int MAPS_PER_DIFFICULTY = 4;
constexpr int STATIC_RVG_RESOLUTION = 8;
constexpr std::uint32_t BASE_SEED = 20260720;

const std::array<DifficultyOutput, 5> DIFFICULTIES = {{
    {MapDifficulty::VeryEasy, "VeryEasy"},
    {MapDifficulty::Easy, "Easy"},
    {MapDifficulty::Medium, "Medium"},
    {MapDifficulty::Hard, "Hard"},
    {MapDifficulty::VeryHard, "VeryHard"}
}};

Polygon<T> benchmarkRobot() {
  return Polygon<T>(
      std::vector<Vertex<T>>{
          Vertex<T>(-3, 0.5),
          Vertex<T>(-2, 0.5),
          Vertex<T>(-2, 2),
          Vertex<T>(-3, 2)
      },
      false
  );
}

std::string benchmarkFilename(const DifficultyOutput &output, int sample) {
  return "mapfixedStartAndGoal" + std::string(output.name) +
      "Small" + std::to_string(sample) + ".xml";
}

template<typename Value>
bool parseNumber(const char *text, Value &value) {
  const char *end = text + std::char_traits<char>::length(text);
  const auto result = std::from_chars(text, end, value);
  return result.ec == std::errc() && result.ptr == end;
}

int generateOrganicCandidate(int argc, char *argv[]) {
  if (argc != 8 && argc != 9) {
    std::cerr
        << "Usage: mainMapGeneration --organic-candidate <output.xml> <seed> "
        << "<obstacle-count> <maximum-obstacle-size> <static-rvg-resolution> "
        << "<num-threads> [protect-endpoint-border-clearance]\n";
    return 2;
  }

  std::uint32_t seed = 0;
  int obstacleCount = 0;
  T maximumObstacleSize = 0;
  int resolution = 0;
  int numThreads = 0;
  int protectEndpointBorderClearance = 1;
  if (!parseNumber(argv[3], seed) ||
      !parseNumber(argv[4], obstacleCount) ||
      !parseNumber(argv[5], maximumObstacleSize) ||
      !parseNumber(argv[6], resolution) ||
      !parseNumber(argv[7], numThreads) ||
      (argc == 9 && !parseNumber(argv[8], protectEndpointBorderClearance)) ||
      obstacleCount <= 0 ||
      !std::isfinite(maximumObstacleSize) ||
      maximumObstacleSize <= 0 ||
      resolution <= 0 ||
      numThreads <= 0 ||
      (protectEndpointBorderClearance != 0 && protectEndpointBorderClearance != 1)) {
    std::cerr << "Invalid organic-candidate argument.\n";
    return 2;
  }

  MapGenerator<T> generator(
      MAP_SIZE,
      obstacleCount,
      maximumObstacleSize,
      benchmarkRobot(),
      seed
  );
  generator.generateSolvableFixedStartAndGoalMap(
      resolution,
      numThreads,
      protectEndpointBorderClearance != 0
  );
  generator.writeMap(argv[2]);

  tinyxml2::XMLDocument document;
  if (document.LoadFile(argv[2]) != tinyxml2::XML_SUCCESS ||
      document.RootElement() == nullptr ||
      document.RootElement()->FirstChildElement("environment") == nullptr) {
    std::cerr << "Unable to annotate organic candidate: " << argv[2] << '\n';
    return 2;
  }
  tinyxml2::XMLElement *environment =
      document.RootElement()->FirstChildElement("environment");
  tinyxml2::XMLElement *metadata = document.NewElement("organicCandidateMetadata");
  metadata->SetAttribute("seed", static_cast<unsigned>(seed));
  metadata->SetAttribute("requestedObstacleCount", obstacleCount);
  metadata->SetAttribute("maximumObstacleSize", maximumObstacleSize);
  metadata->SetAttribute("generationAttempt", generator.getSuccessfulAttempt());
  metadata->SetAttribute("sampledObstacleCount", generator.getSampledObstacleCount());
  metadata->SetAttribute(
      "interiorObstacleCount",
      static_cast<unsigned>(generator.getObstacles().size())
  );
  metadata->SetAttribute("borderCrossingCount", generator.getBorderCrossingCount());
  metadata->SetAttribute("staticRvgResolution", resolution);
  metadata->SetAttribute(
      "protectEndpointBorderClearance",
      protectEndpointBorderClearance
  );
  environment->InsertEndChild(metadata);
  if (document.SaveFile(argv[2]) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Unable to save annotated organic candidate: " << argv[2] << '\n';
    return 2;
  }
  Utils::print(
      "Wrote",
      argv[2],
      "seed",
      seed,
      "attempt",
      generator.getSuccessfulAttempt(),
      "interior obstacles",
      generator.getObstacles().size(),
      "border crossings",
      generator.getBorderCrossingCount()
  );
  return 0;
}

}

int main(int argc, char *argv[]) {
  if (argc >= 2 && std::string(argv[1]) == "--organic-candidate") {
    return generateOrganicCandidate(argc, argv);
  }
  if (argc != 3 || std::string(argv[1]) != "--official-suite") {
    std::cerr
        << "Usage:\n"
        << "  mainMapGeneration --official-suite <output-directory>\n"
        << "  mainMapGeneration --organic-candidate <output.xml> <seed> "
        << "<obstacle-count> <maximum-obstacle-size> <static-rvg-resolution> "
        << "<num-threads> [protect-endpoint-border-clearance]\n";
    return 2;
  }

  const std::filesystem::path outputDirectory(argv[2]);
  std::filesystem::create_directories(outputDirectory);
  const Polygon<T> robot = benchmarkRobot();
  const int staticRvgThreads = Utils::availableProcessorCount();

  for (size_t level = 0; level < DIFFICULTIES.size(); ++level) {
    const DifficultyOutput &output = DIFFICULTIES[level];
    for (int sample = 0; sample < MAPS_PER_DIFFICULTY; ++sample) {
      const std::uint32_t seed =
          BASE_SEED + static_cast<std::uint32_t>(level * 100 + sample);
      MapGenerator<T> generator(
          MAP_SIZE,
          output.difficulty,
          seed,
          robot
      );
      Utils::print(
          "Generating",
          output.name,
          "sample",
          sample,
          "seed",
          seed
      );
      generator.generateSolvableFixedStartAndGoalMap(
          STATIC_RVG_RESOLUTION,
          staticRvgThreads
      );
      const std::filesystem::path outputPath =
          outputDirectory / benchmarkFilename(output, sample);
      generator.writeMap(outputPath.string());
      Utils::print(
          "Wrote",
          outputPath.string(),
          "attempt",
          generator.getSuccessfulAttempt(),
          "interior obstacles",
          generator.getObstacles().size(),
          "border crossings",
          generator.getBorderCrossingCount()
      );
    }
  }
  return 0;
}
