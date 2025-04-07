#include <boost/version.hpp>
#include <iostream>
#include<CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/Polygon_set_2.h>
#include<CGAL/Polygon_2_algorithms.h>
#include<CGAL/draw_polygon_2.h>
#include<CGAL/draw_polygon_set_2.h>

typedef double T;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;

typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;

int main(){
  std::cout << "Boost version: " << BOOST_LIB_VERSION << std::endl;
  Polygon_set_2 S;
  std::vector<Point_2> vertices2 = {
      Point_2(-1.99618, -1.2929),
      Point_2(-2.7071, -1.99618),
      Point_2(-2.00382, -2.7071),
      Point_2(-1.2929, -2.00382)
  };
    Polygon_2 polygon(vertices2.begin(), vertices2.end());
    CGAL::draw(polygon);
    S.insert(polygon);
    CGAL::draw(S);

  return 0;
}


