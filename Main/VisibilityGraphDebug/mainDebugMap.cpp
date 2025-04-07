#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#ifdef VISIBILITY_QUERY_TEV
#include <CGAL/Triangular_expansion_visibility_2.h>
#elif defined(VISIBILITY_QUERY_RSV)
#include <CGAL/Rotational_sweep_visibility_2.h>
#endif
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <Utils/Pragma.h>
#include <Utils/Utils.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

using namespace RotationalVisibilityGraph;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

int main() {
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
  T map_size = 10.;
  std::vector<Vertex<T>> vertices2 = {
      Vertex<T>(-map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., -map_size / 2.),
      Vertex<T>(map_size / 2., map_size / 2.),
      Vertex<T>(-map_size / 2., map_size / 2.)
  };
  Polygon<T> map = Polygon<T>(vertices2, true);
  std::vector<Polygon_2> polygons;
  polygons.push_back(polygon.getPolygon());
  polygons.push_back(polygon1.getPolygon());
  polygons.push_back(map.getPolygon());
  Polygon_with_holes_2 pwh(map.getPolygon(), polygons.begin(), polygons.end() - 1);
  std::vector<Segment_2> segments;
  for (auto &p: polygons) {
    for (auto edge = p.edges_begin(); edge != p.edges_end(); ++edge) {
      segments.emplace_back(edge->source(), edge->target());
    }
  }

  Arrangement_2 env;
  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());
  Point_2 q(0, 0);
  Arrangement_2::Face_const_handle *face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);
  Arrangement_2 output_arr;
  VQ vq(env);
  Face_handle fh = vq.compute_visibility(q, (*face), output_arr);
  std::cout << "Regularized visibility region of q has "
            << output_arr.number_of_edges()
            << " edges:" << std::endl;
  for (Edge_const_iterator eit = output_arr.edges_begin(); eit != output_arr.edges_end(); ++eit)
    std::cout << "[" << eit->source()->point() << " -> " << eit->target()->point() << "]" << std::endl;
  CGAL::draw(env);
//  Arrangement_2 draw_arr;
//  std::vector<Segment_2> segments1;
//  for (auto v = output_arr.vertices_begin(); v != output_arr.vertices_end(); ++v) {
//    segments1.push_back(Segment_2(q, v->point()));
//  }
//  CGAL::insert_non_intersecting_curves(draw_arr, segments1.begin(), segments1.end());
//  CGAL::draw(draw_arr);
  CGAL::draw(pwh);

  return 0;
}