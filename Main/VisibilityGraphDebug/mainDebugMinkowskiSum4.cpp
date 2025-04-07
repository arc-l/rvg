#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type> Convex_hull_traits_2;

int main()
{
    std::vector<Point_2> vertices0 = {
        Point_2(-10, 10),
        Point_2(-5, 10),
        Point_2(-5, 6),
        Point_2(5, 6),
        Point_2(5, 10),
        Point_2(10, 10),
        Point_2(10, -10),
        Point_2(-10, -10)};

    std::vector<Point_2> vertices1 = {
        Point_2(-0.500000, -1.000000),
        Point_2(0.066987, -1.116025),
        Point_2(0.500000, -1.000000),
        Point_2(0.933013, -0.616025),
        Point_2(0.500000, 1.000000),
        Point_2(-0.066987, 1.116025),
        Point_2(-0.500000, 1.000000),
        Point_2(-0.933013, 0.616025),
    };
    Polygon_2 polygon0(vertices0.begin(), vertices0.end());
    Polygon_2 polygon1(vertices1.begin(), vertices1.end());
    std::string robotDrawScript;

    if (polygon0.is_clockwise_oriented())
    {
        polygon0.reverse_orientation();
    }
    if (polygon1.is_clockwise_oriented())
    {
        polygon1.reverse_orientation();
    }

    polygon0.reverse_orientation();
    Polygon_with_holes_2 sum = CGAL::minkowski_sum_by_reduced_convolution_2(polygon0, polygon1);
    Polygon_2 polygon2 = sum.outer_boundary();

    return 0;
}