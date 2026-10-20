#ifndef ROTATIONAL_VISIBILITY_GRAPH_CONFIGURATION_SPACE_LAYER_H
#define ROTATIONAL_VISIBILITY_GRAPH_CONFIGURATION_SPACE_LAYER_H

#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>

#include <vector>

namespace RotationalVisibilityGraph {

// A visibility-graph-free configuration-space layer for one complete angular
// interval.  The swept footprint is deliberately conservative: all sampled
// rotations and the requested spatial half-cell padding are enclosed by one
// convex polygon.
template <typename T>
class ConfigurationSpaceLayer {
public:
    DECL_CGAL_CARTESIAN_TYPES_T
    DECL_CGAL_POLYGON_TYPES_T

    ConfigurationSpaceLayer(
        const Polygon<T> &robot,
        const Polygon<T> &border,
        const std::vector<Polygon<T>> &obstacles,
        T thetaLower,
        T thetaUpper,
        T spatialPadding = static_cast<T>(0)
    );

    [[nodiscard]] bool legalConfig(const Vertex<T> &configuration) const;
    [[nodiscard]] bool isFeasible() const;
    [[nodiscard]] T getThetaLb() const;
    [[nodiscard]] T getThetaUb() const;
    [[nodiscard]] const Polygon<T> &getSweptFootprint() const;
    [[nodiscard]] const std::vector<Polygon<T>> &getSweptFootprintComponents() const;
    [[nodiscard]] std::vector<Polygon_2> growObstacle(
        const Polygon<T> &obstacle
    ) const;
    [[nodiscard]] std::vector<Polygon_2> growObstacleSegment(
        const Segment_2 &segment
    ) const;
    [[nodiscard]] static std::vector<Polygon_2> mergeGrownObstacles(
        const std::vector<Polygon_2> &components
    );

private:
    static std::vector<Polygon_with_holes_2> mergeGrownObstacleRegions(
        const std::vector<Polygon_2> &components
    );
    static Polygon<T> buildConservativeSweptFootprint(
        const Polygon<T> &robot,
        const Vertex<T> &rotationCenter,
        T thetaLower,
        T thetaUpper,
        T spatialPadding
    );
    void appendObstacleComponents(
        const Polygon<T> &obstacle,
        std::vector<Polygon_2> &components
    ) const;

    T _thetaLower;
    T _thetaUpper;
    T _spatialPadding;
    bool _feasible = false;
    Polygon<T> _sweptFootprint;
    Polygon<T> _invertedSweptFootprint;
    std::vector<Polygon<T>> _sweptFootprintComponents;
    std::vector<Polygon<T>> _invertedSweptFootprintComponents;
    std::vector<Polygon_with_holes_2> _grownObstacleRegions;
    std::vector<Polygon_2> _grownObstacles;
    std::vector<Polygon_2> _shrunkBorders;
};

}

#endif
