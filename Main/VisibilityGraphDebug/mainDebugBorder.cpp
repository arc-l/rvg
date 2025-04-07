#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Vertex.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type> Convex_hull_traits_2;
using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main()
{
    std::vector<Point_2> verticesUpper = {
        Point_2(-20, 6),
        Point_2(-27, 6),
        Point_2(-27, -50),
        Point_2(50, -50),
        Point_2(50, 6),
        Point_2(42.5, 6),
        Point_2(42.5, 0.0),
        Point_2(40, 0.0),
        Point_2(39, 1.0),
        Point_2(39, 6.0),
        Point_2(31.5, 6.0),
        Point_2(31.5, 1.0),
        Point_2(30.5, 0.0),
        Point_2(28, 0.0),
        Point_2(27, 1.0),
        Point_2(27, 6.0),
        Point_2(19.5, 6.0),
        Point_2(19.5, 1.0),
        Point_2(18.5, 0.0),
        Point_2(16, 0.0),
        Point_2(15, 1.0),
        Point_2(15, 6.0),
        Point_2(7.5, 6.0),
        Point_2(7.5, 1.0),
        Point_2(6.5, 0.0),
        Point_2(4, 0.0),
        Point_2(3, 1.0),
        Point_2(3, 6.0),
        Point_2(-4.5, 6.0),
        Point_2(-4.5, 1.0),
        Point_2(-5.5, 0.0),
        Point_2(-8, 0.0),
        Point_2(-9, 1.0),
        Point_2(-9, 6.0),
        Point_2(-16.5, 6.0),
        Point_2(-16.5, 1.0),
        Point_2(-17.5, 0.0),
        Point_2(-20, 0.0)
        };
    Polygon_2 polygon(verticesUpper.begin(), verticesUpper.end());
    print("Polygon is simple: ", polygon.is_simple());

    return 0;
}
