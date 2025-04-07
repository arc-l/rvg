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
#include <VisibilityGraph/Drawer.h>
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
  polygons.push_back(polygon.getCounterClockWise());
  polygons.push_back(polygon1.getCounterClockWise());
  polygons.push_back(map.getCounterClockWise());
  Polygon_with_holes_2 pwh(map.getPolygon(), polygons.begin(), polygons.end() - 1);
  std::vector<Segment_2> segments;
  std::vector<Point_2> points;
  for (auto &p : polygons) {
    for (auto edge = p.edges_begin(); edge != p.edges_end(); ++edge) {
      segments.push_back(Segment_2(edge->source(), edge->target()));
    }
    for (auto vertex = p.vertices_begin(); vertex != p.vertices_end(); ++vertex) {
      points.push_back(*vertex);
    }
  }

  Arrangement_2 env;
  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());
  {
    Point_2 q(0, 0);
//  Point_2 q(2, 1);
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
    std::vector<Segment_2> segments1;
    for (auto v = output_arr.vertices_begin(); v != output_arr.vertices_end(); ++v) {
      segments1.push_back(Segment_2(q, v->point()));
    }
    RotationalVisibilityGraph::draw<T, Segment_2>(segments1);
  }

  {
    Point_2 q(2, 1);
    Vertex_const_handle v = env.vertices_begin();
    while (v->point() != q)
      v++;

    typename Arrangement_2::Halfedge_around_vertex_const_circulator first, current;
    first = current = v->incident_halfedges();
    while (true) {
      Utils::print("source: ", current->source()->point(), " target: ", current->target()->point());
      Utils::print("Is on inner ccb:", current->is_on_inner_ccb(), "is on outer ccb: ", current->is_on_outer_ccb());
      current++;
      if (current == first)
        break;
    }

    return 0;

    for (auto &he : env.halfedge_handles()) {
      Utils::print("source: ", he->source()->point(), " target: ", he->target()->point());
      if (he->source()->point() == q) {
        Utils::print("Found q in source: source: ", he->source()->point(), " target: ", he->target()->point());
      }
      if (he->target()->point() == q) {
        Utils::print("Found q in target: source: ", he->source()->point(), " target: ", he->target()->point());
        Utils::print("Is on inner boundary: ", he->face()->is_unbounded(), " Is on outer boundary: ",
                     he->twin()->face()->is_unbounded());
        Utils::print("Is on inner ccb:", he->is_on_inner_ccb(), "is on outer ccb: ", he->is_on_outer_ccb());
      }
    }
    Halfedge_const_handle he = env.halfedges_begin();
    while (true) {
      if (he->target()->point() == q && he->is_on_inner_ccb())
        break;
      else
        he++;
      if (he == env.halfedges_end())
        break;
    }
    Utils::print("source: ", he->source()->point(), " target: ", he->target()->point());
    //visibility query
    Arrangement_2 output_arr;
    VQ vq(env);
    Face_handle fh = vq.compute_visibility(q, he, output_arr);

    std::cout << "Regularized visibility region of q has "
              << output_arr.number_of_edges()
              << " edges." << std::endl;
    std::cout << "Boundary edges of the visibility region:" << std::endl;
    Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
    std::cout << "[" << curr->source()->point() << " -> " << curr->target()->point() << "]" << std::endl;
    while (++curr != fh->outer_ccb())
      std::cout << "[" << curr->source()->point() << " -> " << curr->target()->point() << "]" << std::endl;
    std::vector<Segment_2> segments1;
    CGAL::Arr_naive_point_location<Arrangement_2> pl(output_arr);
    for (const auto &polygon : polygons) {
      auto v = polygon.vertices_begin();
      while (true) {
        if (*v == q)
        {
          v++;
          if (v == polygon.vertices_end())
            break;
          continue;
        }
        CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);
        if (boost::get<Arrangement_2::Vertex_const_handle>(&obj))
          segments1.push_back(Segment_2(q, *v));
        v++;
        if (v == polygon.vertices_end())
          break;
      }
    }
    CGAL::draw(output_arr, "output_arr");
    RotationalVisibilityGraph::draw<T, Segment_2>(segments1);
  }
  return 0;
}
