#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Layer.h>
#include <VisibilityGraph/Polygon.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <cmath>

using namespace RotationalVisibilityGraph;
using namespace RotationalVisibilityGraph::Utils;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main() {
  {
    Polygon<T> robot =
        Polygon<T>(
            std::vector<Vertex<T>>{
                Vertex<T>(-3, 0),
                Vertex<T>(-2, 0),
                Vertex<T>(-2, 2),
                Vertex<T>(-3, 2)
            },
            false);
    std::vector<Vertex<T>> vertices = {
        Vertex<T>(2, -1),
        Vertex<T>(4, -1),
        Vertex<T>(4, -2),
        Vertex<T>(2, -2)
    };
    Polygon<T> polygon(vertices, true);
    std::vector<Vertex<T>> vertices1 = {
        Vertex<T>(2, 1),
        Vertex<T>(4, 1),
        Vertex<T>(4, 2),
        Vertex<T>(2, 2)
    };
    Polygon<T> polygon1 = Polygon(vertices1, true);
    T map_size = 15.;
    std::vector<Vertex<T>> vertices2 = {
        Vertex<T>(-map_size / 2., -map_size / 2.),
        Vertex<T>(map_size / 2., -map_size / 2.),
        Vertex<T>(map_size / 2., map_size / 2.),
        Vertex<T>(-map_size / 2., map_size / 2.)
    };
    Polygon<T> map = Polygon<T>(vertices2, true);
    std::vector<Polygon<T>> obstacles = {polygon, polygon1};
    T theta_lb = 2*M_PI * 8/9;
    T theta_ub = 2 * M_PI;

    Polygon<T> robotLB = robot.rotateCopy(theta_lb);
    Polygon<T> robotUB = robot.rotateCopy(theta_ub);
    std::vector<Point_2> points;
    for (int i = 0; i < robotLB.size(); i++)
      points.push_back(robotLB[i].getPoint());
    for (int i = 0; i < robotUB.size(); i++)
      points.push_back(robotUB[i].getPoint());
    std::vector<std::size_t> indices(points.size()), out;
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points)));
    std::vector<Vertex<T>> bbVertices;
    for (std::size_t i : out) {
      bbVertices.push_back(Vertex<T>(CGAL::to_double(points[i].x()), CGAL::to_double(points[i].y())));
    }
    Polygon<T> robotBBox(bbVertices, false);
    CGAL::draw(robotBBox.getPolygon());
    Utils::print("std::vector<Point_2> vertices0={");
    for (const auto & v : bbVertices){
      Utils::print("Point_2(", v.getX(), ", ", v.getY(), "),");
    }
    Utils::print("};");

    Utils::print("std::vector<Point_2> vertices1={");
    for (const auto & v: vertices){
        Utils::print("Point_2(", v.getX(), ", ", v.getY(), "),");
    }
    Utils::print("};");

    Utils::print("std::vector<Point_2> vertices2={");
    for (const auto & v: vertices1){
        Utils::print("Point_2(", v.getX(), ", ", v.getY(), "),");
    }
    Utils::print("};");
    std::vector<Polygon_2> grownObsBeforeMerge, grownObstacles;
    std::vector<Polygon_with_holes_2> mergedObs;
    for (const Polygon<T> &obstacle : obstacles) {
      Polygon_with_holes_2 grownObs = CGAL::minkowski_sum_2(obstacle.getPolygon(), robotBBox.getPolygon());
      Utils::print("New Obs");
      for (const auto & v: grownObs.outer_boundary().vertices()){
        Utils::print("Vertex<T>(", v.x(), ", ", v.y(), "),");
      }
      grownObsBeforeMerge.push_back(grownObs.outer_boundary());
    }

    CGAL::join(grownObsBeforeMerge.begin(), grownObsBeforeMerge.end(), std::back_inserter(mergedObs));
    for (const Polygon_with_holes_2 &mergedObstacle : mergedObs) {
      Polygon_2 tmpPolygon = mergedObstacle.outer_boundary();
      Utils::print("Is tmpPolygon Simple? ", tmpPolygon.is_simple());
      grownObstacles.push_back(tmpPolygon);
    }
    CGAL::draw(grownObstacles[0]);
    Utils::print("# of vertices: ", grownObstacles[0].vertices().size());
  }
  {
    std::vector<Vertex<T>> vertices = {
        Vertex<T>(-1.52581, -1.44465),
        Vertex<T>(-1, -2),
        Vertex<T>(-0.759765, -2.08744),
        Vertex<T>(1.24023, -2.08744),
        Vertex<T>(2, -2),
        Vertex<T>(2.52581, -0.555349),
        Vertex<T>(2.52581, 0.444651),
        Vertex<T>(2, 1),
        Vertex<T>(1.75977, 1.08744),
        Vertex<T>(-0.240235, 1.08744),
        Vertex<T>(-1, 1),
        Vertex<T>(-1.52581, -0.444651)
    };
    Polygon<T> polygon(vertices, true);
    std::vector<Vertex<T>> vertices1 = {
        Vertex<T>( -1.52581 ,  1.55535 ),
        Vertex<T>( -1 ,  1 ),
        Vertex<T>( -0.759765 ,  0.912562 ),
        Vertex<T>( 1.24023 ,  0.912562 ),
        Vertex<T>( 2 ,  1 ),
        Vertex<T>( 2.52581 ,  2.44465 ),
        Vertex<T>( 2.52581 ,  3.44465 ),
        Vertex<T>( 2 ,  4 ),
        Vertex<T>( 1.75977 ,  4.08744 ),
        Vertex<T>( -0.240235 ,  4.08744 ),
        Vertex<T>( -1 ,  4 ),
        Vertex<T>( -1.52581 ,  2.55535 )
    };
    Polygon<T> polygon1 = Polygon(vertices1, true);
    std::vector<Polygon_with_holes_2> res;
    std::vector<Polygon_2> polygons{polygon.getPolygon(), polygon1.getPolygon()};
    CGAL::join(polygons.begin(), polygons.end(), std::back_inserter(res));
    for (const auto &v : res[0].outer_boundary().vertices()) {
      print(v);
    }
    print(res[0].outer_boundary().vertices().size());
    CGAL::draw(res[0]);
  }
}