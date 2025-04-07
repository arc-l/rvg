#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPHUTILS_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPHUTILS_H_

#include<Utils/Pragma.h>
#include<Utils/Utils.h>
#include<VisibilityGraph/Vertex.h>
#include<VisibilityGraph/Edge.h>
#include<VisibilityGraph/Polygon.h>
#include<CGAL/Exact_predicates_exact_constructions_kernel.h>
#include<CGAL/Polygon_2_algorithms.h>
#include<CGAL/Polygon_set_2.h>
#include<CGAL/draw_polygon_2.h>
#include<CGAL/draw_polygon_set_2.h>
#include<CGAL/Polygon_2.h>

namespace RotationalVisibilityGraph {
template<typename T>
class PolygonDrawer{
 public:
    DECL_CGAL_CARTESIAN_TYPES_T
    DECL_CGAL_POLYGON_TYPES_T
    explicit PolygonDrawer(const std::vector<std::shared_ptr<Polygon<T>>>& polygons);
    void draw();
 private:
  const std::vector<std::shared_ptr<Polygon<T>>> &_polygons;
};

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_GRAPHUTILS_H_
