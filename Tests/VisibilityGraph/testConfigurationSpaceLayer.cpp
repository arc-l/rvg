#include <VisibilityGraph/ConfigurationSpaceLayer.h>
#include <VisibilityGraph/Layer.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace RotationalVisibilityGraph;

namespace {

Polygon<double> polygon(std::initializer_list<std::pair<double, double>> coordinates)
{
    std::vector<Vertex<double>> vertices;
    for (const auto &coordinate : coordinates) {
        vertices.emplace_back(coordinate.first, coordinate.second);
    }
    return Polygon<double>(vertices, false);
}

bool require(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

}

int main()
{
    const Polygon<double> robot = polygon({
        {-0.4, -0.2}, {0.4, -0.2}, {0.4, 0.2}, {-0.4, 0.2}
    });
    const Polygon<double> border = polygon({
        {-5, -5}, {5, -5}, {5, 5}, {-5, 5}
    });
    const std::vector<Polygon<double>> obstacles = {
        polygon({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
    };
    ConfigurationSpaceLayer<double> configurationLayer(
        robot, border, obstacles, 0.0, 0.05, 0.0
    );
    if (!require(configurationLayer.isFeasible(), "configuration layer must be feasible") ||
        !require(configurationLayer.legalConfig(Vertex<double>(3, 3)), "open point must be legal") ||
        !require(!configurationLayer.legalConfig(Vertex<double>(0, 0)), "obstacle point must be occupied") ||
        !require(!configurationLayer.legalConfig(Vertex<double>(4.9, 0)), "border collision must be occupied")) {
        return 1;
    }

    Layer<double> referenceLayer(0.0, 0.05, false);
    referenceLayer.buildVisibilityGraph(robot, border, obstacles);
    const std::vector<Vertex<double>> representativePoints = {
        Vertex<double>(3, 3),
        Vertex<double>(0, 0),
        Vertex<double>(-3, 2)
    };
    for (const Vertex<double> &point : representativePoints) {
        if (!require(
                configurationLayer.legalConfig(point) == referenceLayer.legalConfig(point),
                "standalone configuration layer disagrees with Layer away from boundaries"
            )) {
            return 1;
        }
    }

    const Polygon<double> nonConvexRobot = polygon({
        {-0.5, -0.5}, {0.5, -0.5}, {0.1, 0}, {0.5, 0.5}, {-0.5, 0.5}
    });
    ConfigurationSpaceLayer<double> padded(
        nonConvexRobot, border, {}, 0.0, std::acos(-1.0) / 2.0, 0.25
    );
    if (!require(padded.isFeasible(), "non-convex swept layer must be feasible") ||
        !require(padded.getSweptFootprint().size() >= 4, "swept footprint must be a polygon")) {
        return 1;
    }

    using Segment = ConfigurationSpaceLayer<double>::Segment_2;
    using Point = ConfigurationSpaceLayer<double>::Point_2;
    const std::vector<ConfigurationSpaceLayer<double>::Polygon_2> grownSegment =
        padded.growObstacleSegment(Segment(Point(-1, 0), Point(1, 0)));
    const auto coveredByGrownSegment = [&](const Point &point) {
        return std::any_of(
            grownSegment.begin(),
            grownSegment.end(),
            [&](const ConfigurationSpaceLayer<double>::Polygon_2 &grown) {
                return IN_POLYGON(point, grown) || ON_EDGE(point, grown);
            }
        );
    };
    if (!require(!grownSegment.empty(),
                 "observed segment growth must produce configuration obstacles") ||
        !require(coveredByGrownSegment(Point(0, 0)),
                 "segment growth must cover configurations touching the segment") ||
        !require(!coveredByGrownSegment(Point(0, 3)),
                 "segment growth must not fill unobserved distant space")) {
        return 1;
    }
    return 0;
}
