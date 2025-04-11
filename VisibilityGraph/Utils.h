#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_UTILS_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_UTILS_H_

#include <Utils/Utils.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#ifdef VISIBILITY_QUERY_TEV
#include <CGAL/Triangular_expansion_visibility_2.h>
#elif defined(VISIBILITY_QUERY_RSV)
#include <CGAL/Rotational_sweep_visibility_2.h>
#endif
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

namespace RotationalVisibilityGraph::Utils {
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
DECL_CGAL_VISIBILITY_GRAPH_TYPES_T

template<typename T>
std::shared_ptr<Vertex<T>> getVertex(const tinyxml2::XMLElement &element) {
  T x, y;
  auto res = element.QueryAttribute("x", &x);
  if (res != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Vertex does not have x attribute");
  res = element.QueryAttribute("y", &y);
  if (res != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Vertex does not have y attribute");
  T theta;
  res = element.QueryAttribute("theta", &theta);
  if (res != tinyxml2::XML_SUCCESS)
    return std::make_shared<Vertex<T>>(x, y);
  else {
    T thetaLb, thetaUb;
    element.QueryAttribute("thetaLb", &thetaLb);
    element.QueryAttribute("thetaUb", &thetaUb);
    return std::make_shared<Vertex<T>>(x, y, thetaLb, thetaUb, theta);
  }
}

template<typename T>
Polygon<T> getPolygon(const tinyxml2::XMLElement &element) {
  std::vector<Vertex<T>> vertices;
  const tinyxml2::XMLElement *vertexRoot = element.FirstChildElement("Vertex");
  if (!vertexRoot)
    throw std::runtime_error("Polygon does not have any vertices");
  while (vertexRoot) {
    std::shared_ptr<Vertex<T>> vertex = getVertex<T>(*vertexRoot);
    vertices.push_back(*vertex);
    vertexRoot = vertexRoot->NextSiblingElement("Vertex");
  }
  bool sortVertices = element.BoolAttribute("sortVertices", false);

  const tinyxml2::XMLElement *rotationalCenter = element.FirstChildElement("RotationCenter");
  if(rotationalCenter){
    std::shared_ptr<Vertex<T>> center = getVertex<T>(*rotationalCenter);
    return Polygon<T>(vertices, *center, sortVertices);
  }
  return Polygon<T>(vertices, sortVertices);
}

template<typename T>
void putVertex(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement &element, const Vertex<T> &vertex) {
  tinyxml2::XMLElement *vertexElement = doc.NewElement("Vertex");
  vertexElement->SetAttribute("x", vertex.getX());
  vertexElement->SetAttribute("y", vertex.getY());
  if (vertex.hasTheta()) {
    vertexElement->SetAttribute("theta", vertex.getTheta());
    vertexElement->SetAttribute("thetaLb", vertex.getThetaLb());
    vertexElement->SetAttribute("thetaUb", vertex.getThetaUb());
  }
  element.InsertEndChild(vertexElement);
}

template<typename T>
void putPolygon(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement &element, const Polygon<T> &polygon) {
  tinyxml2::XMLElement *polygonElement = doc.NewElement("Polygon");
  for (const Vertex<T> &vertex : polygon.getVertices()) {
    putVertex(doc, *polygonElement, vertex);
  }
  polygonElement->SetAttribute("sortVertices", false);
  element.InsertEndChild(polygonElement);
}

template<typename T>
Polygon<T> getRobot(const tinyxml2::XMLDocument &pt) {
  const tinyxml2::XMLElement *env = pt.RootElement()->FirstChildElement("environment");
  if (!env) throw std::runtime_error("Environment not found");
  const tinyxml2::XMLElement *robotRoot = env->FirstChildElement("robot");
  if (!robotRoot) throw std::runtime_error("Robot not found");
  Polygon<T> robot = getPolygon<T>(*(robotRoot->FirstChildElement("Polygon")));
  return robot;
}

template<typename T>
Polygon<T> getBoundary(const tinyxml2::XMLDocument &pt){
  const tinyxml2::XMLElement *env = pt.RootElement()->FirstChildElement("environment");
  if (!env) throw std::runtime_error("Environment not found");
  const tinyxml2::XMLElement *robotRoot = env->FirstChildElement("boundary");
  if (!robotRoot) throw std::runtime_error("Boundary not found");
  Polygon<T> robot = getPolygon<T>(*(robotRoot->FirstChildElement("Polygon")));
  return robot;
}

template<typename T>
std::vector<Polygon<T>> getObstacles(const tinyxml2::XMLDocument &pt) {
  const tinyxml2::XMLElement *env = pt.RootElement()->FirstChildElement("environment");
  if (!env) throw std::runtime_error("Environment not found");
  const tinyxml2::XMLElement *obstacleRoot = env->FirstChildElement("obstacles");
  if (!obstacleRoot) throw std::runtime_error("Obstacles not found");
  const tinyxml2::XMLElement *obstaclePolygon = obstacleRoot->FirstChildElement("Polygon");
  std::vector<Polygon<T>> obstacles;
  while (obstaclePolygon) {
    Polygon<T> polygon = getPolygon<T>(*obstaclePolygon);
    obstacles.push_back(polygon);
    obstaclePolygon = obstaclePolygon->NextSiblingElement("Polygon");
  }
  return obstacles;
}

template<typename T>
Polygon<T> getMap(int mapSize) {
  std::vector<Vertex<T>>
      vertices = {
      Vertex<T>(-mapSize / 2., -mapSize / 2.),
      Vertex<T>(mapSize / 2., -mapSize / 2.),
      Vertex<T>(mapSize / 2., mapSize / 2.),
      Vertex<T>(-mapSize / 2., mapSize / 2.)
  };
  Polygon<T> map = Polygon<T>(vertices, true);
  return map;
}

template<typename T>
bool pointInsideOrOnArrangement(const Point_2 &point, const Arrangement_2 & visibleArea) {
  PL_2 pl(visibleArea);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(point);
  const Vertex_const_handle*   v;
  const Halfedge_const_handle* e;
  const Face_const_handle*     f;
  f = boost::get<Face_const_handle>(&obj);
  if (f) // located inside a face
  {
    return (!(*f)->is_unbounded());
  }
  else {
    e = boost::get<Halfedge_const_handle>(&obj);
    if (e) // located on an edge
    {
      return true;
    }
    else {
      v = boost::get<Vertex_const_handle>(&obj);
      if (v) // located on a vertex
      {
        return true;
      }
    }
  }
  return false;
}

template<typename T>
bool isVertexOfArrangement(const Point_2& point, const PL_2 & pl) {
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(point);
  const Vertex_const_handle*   v;
  v = boost::get<Vertex_const_handle>(&obj);
  return v != nullptr;
}

template<typename T>
Polygon_2 arrangementToPolygon(const Arrangement_2 &arrangement) {
  std::vector<Point_2> points;
  for (auto v = arrangement.vertices_begin(); v != arrangement.vertices_end(); ++v) {
    points.push_back(v->point());
  }
  return {points.begin(), points.end()};
}

template<typename T>
bool isBitangent(const Point_2 & v1, const Point_2 & v1l, const Point_2 & v1r, const Point_2 & v2, const Point_2 & v2l, const Point_2 & v2r) {
  auto orientation1l = CGAL::orientation(v2, v1, v1l);
  auto orientation1r = CGAL::orientation(v2, v1, v1r);
  if ((orientation1l * orientation1r)<0) return false;
  return (CGAL::orientation(v1, v2, v2l) * CGAL::orientation(v1, v2, v2r)) >= 0;
}

}

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_UTILS_H_

