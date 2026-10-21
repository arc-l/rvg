#include "ConfigurationSpaceLayer.h"

#include <CGAL/convex_hull_2.h>
#include <CGAL/minkowski_sum_2.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace RotationalVisibilityGraph {

template <typename T>
void ConfigurationSpaceLayer<T>::appendObstacleComponents(
    const Polygon<T> &obstacle,
    std::vector<Polygon_2> &components
) const {
    for (const Polygon<T> &invertedComponent : _invertedSweptFootprintComponents) {
        const Polygon_with_holes_2 grown = CGAL::minkowski_sum_2(
            obstacle.getCounterClockWise(),
            invertedComponent.getCounterClockWise()
        );
        if (grown.outer_boundary().size() >= 3) {
            components.push_back(grown.outer_boundary());
        }
    }
}

template <typename T>
std::vector<typename ConfigurationSpaceLayer<T>::Polygon_with_holes_2>
ConfigurationSpaceLayer<T>::mergeGrownObstacleRegions(
    const std::vector<Polygon_2> &components
) {
    std::vector<Polygon_with_holes_2> merged;
    CGAL::join(
        components.begin(),
        components.end(),
        std::back_inserter(merged)
    );
    return merged;
}

template <typename T>
std::vector<typename ConfigurationSpaceLayer<T>::Polygon_2>
ConfigurationSpaceLayer<T>::mergeGrownObstacles(
    const std::vector<Polygon_2> &components
) {
    const std::vector<Polygon_with_holes_2> merged =
        mergeGrownObstacleRegions(components);
    std::vector<Polygon_2> result;
    result.reserve(merged.size());
    for (const Polygon_with_holes_2 &grown : merged) {
        Polygon_2 outer = grown.outer_boundary();
        if (outer.is_clockwise_oriented()) {
            outer.reverse_orientation();
        }
        result.push_back(std::move(outer));
    }
    return result;
}

template <typename T>
Polygon<T> ConfigurationSpaceLayer<T>::buildConservativeSweptFootprint(
    const Polygon<T> &robot,
    const Vertex<T> &rotationCenter,
    T thetaLower,
    T thetaUpper,
    T spatialPadding
) {
    const T pi = std::acos(static_cast<T>(-1));
    const T interval = std::max<T>(static_cast<T>(0), thetaUpper - thetaLower);
    // At most one degree between samples.  The additional radial sagitta
    // padding encloses the circular arcs between adjacent samples.
    const size_t intervals = std::max<size_t>(
        1,
        static_cast<size_t>(std::ceil(interval / (pi / static_cast<T>(180))))
    );
    T maximumRadius = static_cast<T>(0);
    for (const Vertex<T> &vertex : robot.getVertices()) {
        maximumRadius = std::max(
            maximumRadius,
            std::hypot(
                vertex.getX() - rotationCenter.getX(),
                vertex.getY() - rotationCenter.getY()
            )
        );
    }
    const T angularStep = interval / static_cast<T>(intervals);
    const T arcPadding = maximumRadius *
        (static_cast<T>(1) - std::cos(angularStep / static_cast<T>(2)));
    const T totalPadding = std::max<T>(static_cast<T>(0), spatialPadding) + arcPadding;

    std::vector<Point_2> points;
    points.reserve((intervals + 1) * robot.getVertices().size() * 4);
    for (size_t sample = 0; sample <= intervals; ++sample) {
        const T theta = thetaLower + interval *
            static_cast<T>(sample) / static_cast<T>(intervals);
        const T cosine = std::cos(theta);
        const T sine = std::sin(theta);
        for (const Vertex<T> &vertex : robot.getVertices()) {
            const T localX = vertex.getX() - rotationCenter.getX();
            const T localY = vertex.getY() - rotationCenter.getY();
            const T rotatedX = localX * cosine - localY * sine;
            const T rotatedY = localX * sine + localY * cosine;
            points.emplace_back(rotatedX - totalPadding, rotatedY - totalPadding);
            points.emplace_back(rotatedX - totalPadding, rotatedY + totalPadding);
            points.emplace_back(rotatedX + totalPadding, rotatedY - totalPadding);
            points.emplace_back(rotatedX + totalPadding, rotatedY + totalPadding);
        }
    }

    std::vector<Point_2> hull;
    CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
    Polygon_2 polygon(hull.begin(), hull.end());
    if (polygon.is_clockwise_oriented()) {
        polygon.reverse_orientation();
    }
    return Polygon<T>(polygon, Vertex<T>(0, 0));
}

template <typename T>
ConfigurationSpaceLayer<T>::ConfigurationSpaceLayer(
    const Polygon<T> &robot,
    const Polygon<T> &border,
    const std::vector<Polygon<T>> &obstacles,
    T thetaLower,
    T thetaUpper,
    T spatialPadding
) : _thetaLower(thetaLower),
    _thetaUpper(thetaUpper),
    _spatialPadding(spatialPadding),
    _sweptFootprint(buildConservativeSweptFootprint(
        robot,
        robot.getCentroid(),
        thetaLower,
        thetaUpper,
        spatialPadding
    )),
    _invertedSweptFootprint(_sweptFootprint)
{
    _invertedSweptFootprint.invert();

    // Preserve non-convex robot structure for obstacle growth.  Each convex
    // component is swept about the original robot rotation center, then the
    // component C-obstacles are unioned below.  The public aggregate footprint
    // remains a conservative hull and is used for border shrinking.
    if (robot.isConvex()) {
        _sweptFootprintComponents.push_back(_sweptFootprint);
    } else {
        for (const Polygon<T> &component : robot.convexDecomposition()) {
            _sweptFootprintComponents.push_back(buildConservativeSweptFootprint(
                component,
                robot.getCentroid(),
                thetaLower,
                thetaUpper,
                spatialPadding
            ));
        }
    }
    for (const Polygon<T> &component : _sweptFootprintComponents) {
        Polygon<T> inverted(component);
        inverted.invert();
        _invertedSweptFootprintComponents.push_back(inverted);
    }

    Polygon_with_holes_2 complementBorder;
    CGAL::complement(border.getCounterClockWise(), complementBorder);
    complementBorder = CGAL::minkowski_sum_2(
        complementBorder,
        _invertedSweptFootprint.getCounterClockWise()
    );
    for (const Polygon_2 &hole : complementBorder.holes()) {
        if (hole.size() < 3) {
            continue;
        }
        Polygon_2 shrunkBorder = hole;
        if (shrunkBorder.is_clockwise_oriented()) {
            shrunkBorder.reverse_orientation();
        }
        _shrunkBorders.push_back(shrunkBorder);
    }

    std::vector<Polygon_2> componentObstacles;
    for (const Polygon<T> &obstacle : obstacles) {
        appendObstacleComponents(obstacle, componentObstacles);
    }
    _grownObstacleRegions = mergeGrownObstacleRegions(componentObstacles);
    _grownObstacles.reserve(_grownObstacleRegions.size());
    for (const Polygon_with_holes_2 &region : _grownObstacleRegions) {
        Polygon_2 outer = region.outer_boundary();
        if (outer.is_clockwise_oriented()) {
            outer.reverse_orientation();
        }
        _grownObstacles.push_back(std::move(outer));
    }
    _feasible = !_shrunkBorders.empty();
}

template <typename T>
std::vector<typename ConfigurationSpaceLayer<T>::Polygon_2>
ConfigurationSpaceLayer<T>::growObstacle(const Polygon<T> &obstacle) const
{
    std::vector<Polygon_2> components;
    components.reserve(_invertedSweptFootprintComponents.size());
    appendObstacleComponents(obstacle, components);
    return mergeGrownObstacles(components);
}

template <typename T>
std::vector<typename ConfigurationSpaceLayer<T>::Polygon_2>
ConfigurationSpaceLayer<T>::growObstacleSegment(const Segment_2 &segment) const
{
    std::vector<Polygon_2> components;
    components.reserve(_invertedSweptFootprintComponents.size());
    for (const Polygon<T> &invertedComponent :
         _invertedSweptFootprintComponents) {
        std::vector<Point_2> translatedPoints;
        translatedPoints.reserve(
            invertedComponent.getPolygon().size() * static_cast<size_t>(2)
        );
        for (auto vertex = invertedComponent.getPolygon().vertices_begin();
             vertex != invertedComponent.getPolygon().vertices_end();
             ++vertex) {
            translatedPoints.emplace_back(
                vertex->x() + segment.source().x(),
                vertex->y() + segment.source().y()
            );
            translatedPoints.emplace_back(
                vertex->x() + segment.target().x(),
                vertex->y() + segment.target().y()
            );
        }
        std::vector<Point_2> hull;
        CGAL::convex_hull_2(
            translatedPoints.begin(),
            translatedPoints.end(),
            std::back_inserter(hull)
        );
        if (hull.size() < 3) {
            continue;
        }
        Polygon_2 grown(hull.begin(), hull.end());
        if (grown.is_clockwise_oriented()) {
            grown.reverse_orientation();
        }
        components.push_back(std::move(grown));
    }
    return mergeGrownObstacles(components);
}

template <typename T>
bool ConfigurationSpaceLayer<T>::legalConfig(const Vertex<T> &configuration) const
{
    if (!_feasible) {
        return false;
    }
    const Point_2 &point = configuration.getPoint();
    const bool insideBorder = std::any_of(
        _shrunkBorders.begin(),
        _shrunkBorders.end(),
        [&](const Polygon_2 &polygon) {
            return IN_POLYGON(point, polygon);
        }
    );
    if (!insideBorder) {
        return false;
    }
    return std::none_of(
        _grownObstacleRegions.begin(),
        _grownObstacleRegions.end(),
        [&](const Polygon_with_holes_2 &region) {
            const Polygon_2 &outer = region.outer_boundary();
            if (!(IN_POLYGON(point, outer) || ON_EDGE(point, outer))) {
                return false;
            }
            for (auto hole = region.holes_begin();
                 hole != region.holes_end();
                 ++hole) {
                const Polygon_2 &holePolygon = *hole;
                if (IN_POLYGON(point, holePolygon)) {
                    return false;
                }
                if (ON_EDGE(point, holePolygon)) {
                    return true;
                }
            }
            return true;
        }
    );
}

template <typename T>
bool ConfigurationSpaceLayer<T>::isFeasible() const
{
    return _feasible;
}

template <typename T>
T ConfigurationSpaceLayer<T>::getThetaLb() const
{
    return _thetaLower;
}

template <typename T>
T ConfigurationSpaceLayer<T>::getThetaUb() const
{
    return _thetaUpper;
}

template <typename T>
const Polygon<T> &ConfigurationSpaceLayer<T>::getSweptFootprint() const
{
    return _sweptFootprint;
}

template <typename T>
const std::vector<Polygon<T>> &
ConfigurationSpaceLayer<T>::getSweptFootprintComponents() const
{
    return _sweptFootprintComponents;
}

template class ConfigurationSpaceLayer<double>;
template class ConfigurationSpaceLayer<float>;

}
