#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYCHECK_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYCHECK_H_

#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/draw_arrangement_2.h>
#ifdef VISIBILITY_QUERY_TEV
#include <CGAL/Triangular_expansion_visibility_2.h>
#elif defined(VISIBILITY_QUERY_RSV)
#include <CGAL/Rotational_sweep_visibility_2.h>
#endif
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>

namespace RotationalVisibilityGraph {

DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

template<typename T>
Arrangement_2 visibleAreaOnObs(const Point_2 &v,
                               Arrangement_2 &env,
                               const std::shared_ptr<VQ> &vq
                               ) {
  std::vector<Point_2> visibleVertices;
  Arrangement_2 output_arr;
  Halfedge_const_handle he = env.halfedges_begin();
  while (true) {
    if (he->target()->point() == v && he->is_on_inner_ccb())
      break;
    else
      he++;
    if (he == env.halfedges_end()) {
      break;
    }
  }
  vq->compute_visibility(v, he, output_arr);
  return output_arr;
}

template<typename T>
std::vector<Point_2> visibleVerticesOnObs(const Point_2 &v,
                                          Arrangement_2 &env) {
  std::vector<Point_2> visibleVertices;
  Arrangement_2 output_arr;
  Halfedge_const_handle he = env.halfedges_begin();
  while (true) {
    if (he->target()->point() == v && he->is_on_inner_ccb())
      break;
    else
      he++;
    if (he == env.halfedges_end()) {
      break;
    }
  }
  VQ vq(env);
  vq.compute_visibility(v, he, output_arr);
  CGAL::Arr_naive_point_location<Arrangement_2> visPl(output_arr);
  for (const auto &validV : env.vertex_handles()) {
    CGAL::Arr_point_location_result<Arrangement_2>::Type obj = visPl.locate(validV->point());
    if (boost::get<Arrangement_2::Vertex_const_handle>(&obj)) {
      visibleVertices.push_back(validV->point());
    }
  }
  return visibleVertices;
}

template<typename T>
Arrangement_2 visibleAreaInMap(
    const Point_2 &v,
    const Arrangement_2 &env,
    const std::shared_ptr<VQ> &vq
    ) {
  std::vector<Vertex<T>> visibleVertices;
  Arrangement_2 output_arr;
  Arrangement_2::Face_const_handle *face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(v);
  // The query point locates in the interior of a face
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);
  ASSERT_MSG(face, "Face not found in the arrangement");
  vq->compute_visibility(v, *face, output_arr);
  return output_arr;
}

template<typename T>
std::vector<Vertex<T>> visibleVerticesInMap(
    const Point_2 &v,
    Arrangement_2 &env,
    const Polygon<T> &border) {
  std::vector<Vertex<T>> visibleVertices;
  Arrangement_2 output_arr;
  Arrangement_2::Face_const_handle *face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(v);
  // The query point locates in the interior of a face
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);
  VQ vq(env);
  vq.compute_visibility(v, *face, output_arr);
  CGAL::Arr_naive_point_location<Arrangement_2> visPl(output_arr);
  for (const auto &validV : env.vertex_handles()) {
    if (isOnBorder(validV->point(), border)) continue;
    CGAL::Arr_point_location_result<Arrangement_2>::Type visObj = visPl.locate(validV->point());
    if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) && validV->point() != v) {
      Vertex<T> v1 = Vertex<T>(
          CGAL::to_double(validV->point().x()),
          CGAL::to_double(validV->point().y())
      );
      visibleVertices.push_back(v1);
    }
  }
  return visibleVertices;
}

template<typename T>
std::vector<Point_2> visibleVertices(
    const Point_2 &v,
    const Arrangement_2 &env,
    const Polygon<T> &border,
    const std::shared_ptr<VQ> &vq
) {
  std::vector<Point_2> visibleVertices, tmpOnEdge;
  Arrangement_2 output_arr;
  Arrangement_2 envCopy = env;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(envCopy);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(v);
//  VQ vq(envCopy);

  bool onObs = false;
  bool onEdge = false;
  Halfedge_const_handle he = envCopy.halfedges_begin();
  int edgeCnt = (int)envCopy.number_of_halfedges();
  int cnt = 0;
  while (true) {
    if (he->target()->point() == v && he->is_on_inner_ccb()) {
      onObs = true;
      break;
    } else if (CGAL::squared_distance(Segment_2(he->source()->point(), he->target()->point()), v) < RVG_EPS
        && he->is_on_outer_ccb() && !onEdge) {
      onEdge = true;
      tmpOnEdge.push_back(he->source()->point());
      tmpOnEdge.push_back(he->target()->point());
    } else {
      he++;
      cnt++;
    }
    if (cnt == edgeCnt) {
      break;
    }
  }
  if (onObs) {
    vq->compute_visibility(v, he, output_arr);
  } else {
    Arrangement_2::Face_const_handle *face;
    face = boost::get<Arrangement_2::Face_const_handle>(&obj);
    if (!face) {
      if (onEdge) return tmpOnEdge;
      else {
        Utils::print("Face not found in the arrangement and also not on edge, returning empty list.");
        return visibleVertices;
      }
    }
    ASSERT_MSG(face, "Face not found in the arrangement")
    vq->compute_visibility(v, *face, output_arr);
  }
  CGAL::Arr_naive_point_location<Arrangement_2> visPl(output_arr);
  for (const auto validV : envCopy.vertex_handles()) {
    if (isOnBorder(validV->point(), border)) continue;
    CGAL::Arr_point_location_result<Arrangement_2>::Type visObj = visPl.locate(validV->point());
    if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) && validV->point() != v) {
      visibleVertices.push_back(validV->point());
    }
  }
  return visibleVertices;
}

template<typename T>
std::vector<Point_2> visibleVerticesOnHoles(
    const Point_2 &v,
    const Arrangement_2 &env,
    const std::shared_ptr<VQ> &vq
) {
  std::vector<Point_2> visibleVertices, tmpOnEdge;
  Arrangement_2 output_arr;
  Arrangement_2 envCopy = env;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(envCopy);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(v);
//  VQ vq(envCopy);

  bool onObs = false;
  bool onEdge = false;
  Halfedge_const_handle he = envCopy.halfedges_begin();
  int edgeCnt = (int)envCopy.number_of_halfedges();
  int cnt = 0;
  while (true) {
    if (he->target()->point() == v && he->is_on_outer_ccb()) {
      onObs = true;
      break;
    } else if (CGAL::squared_distance(Segment_2(he->source()->point(), he->target()->point()), v) < RVG_EPS
        && he->is_on_outer_ccb() && !onEdge) {
      onEdge = true;
      tmpOnEdge.push_back(he->source()->point());
      tmpOnEdge.push_back(he->target()->point());
    } else {
      he++;
      cnt++;
    }
    if (cnt == edgeCnt) {
      break;
    }
  }
  if (onObs) {
    vq->compute_visibility(v, he, output_arr);
  } else {
    Arrangement_2::Face_const_handle *face;
    face = boost::get<Arrangement_2::Face_const_handle>(&obj);
    if (!face) {
      if (onEdge) return tmpOnEdge;
      else {
        Utils::print("Face not found in the arrangement and also not on edge, returning empty list.");
        return visibleVertices;
      }
    }
    ASSERT_MSG(face, "Face not found in the arrangement")
    vq->compute_visibility(v, *face, output_arr);
  }
  CGAL::Arr_naive_point_location<Arrangement_2> visPl(output_arr);
  for (const auto validV : envCopy.vertex_handles()) {
    CGAL::Arr_point_location_result<Arrangement_2>::Type visObj = visPl.locate(validV->point());
    if (boost::get<Arrangement_2::Vertex_const_handle>(&visObj) && validV->point() != v) {
      visibleVertices.push_back(validV->point());
    }
  }
  return visibleVertices;
}

template<typename T>
Arrangement_2 visibleAreaOnHoles(
    const Point_2 &v,
    const Arrangement_2 &env,
    const std::shared_ptr<VQ> &vq
) {
  Arrangement_2 output_arr;
  Arrangement_2 envCopy = env;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(envCopy);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(v);
//  VQ vq(envCopy);

  bool onObs = false;
  bool onEdge = false;
  Halfedge_const_handle he = envCopy.halfedges_begin();
  int edgeCnt = (int)envCopy.number_of_halfedges();
  int cnt = 0;
  while (true) {
    if (he->target()->point() == v && he->is_on_outer_ccb()) {
      onObs = true;
      break;
    } else if (CGAL::squared_distance(Segment_2(he->source()->point(), he->target()->point()), v) < RVG_EPS
        && he->is_on_outer_ccb() && !onEdge) {
      onEdge = true;
    } else {
      he++;
      cnt++;
    }
    if (cnt == edgeCnt) {
      break;
    }
  }
  if (onObs) {
    vq->compute_visibility(v, he, output_arr);
  } else {
    Arrangement_2::Face_const_handle *face;
    face = boost::get<Arrangement_2::Face_const_handle>(&obj);
    if (!face) {
      if (onEdge) return output_arr;
      else {
        Utils::print("Face not found in the arrangement and also not on edge, returning empty list.");
        return output_arr;
      }
    }
    ASSERT_MSG(face, "Face not found in the arrangement")
    vq->compute_visibility(v, *face, output_arr);
  }
  return output_arr;
}

template<typename T>
bool isOnBorder(const Point_2 &v, const Polygon<T> &border) {
  const auto &mapBorder = border.getCounterClockWise();
  return ON_EDGE(v, mapBorder);
}

template<typename T>
Arrangement_2 obsToArrangement(const std::vector<Polygon_2> &obs) {
  Arrangement_2 env;
  for (const auto &obstacle : obs) {
    CGAL::insert_non_intersecting_curves(env, obstacle.edges_begin(), obstacle.edges_end());
  }
  return env;
}

}
#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_VISIBILITYCHECK_H_
