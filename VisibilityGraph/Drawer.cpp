#include "Drawer.h"

namespace RotationalVisibilityGraph {
template<typename T>
SegmentsViewer<T>::SegmentsViewer(QWidget *parent, const std::vector<Segment_2> &segments, const char *title):
    Base(parent, title, true, true, true, false, false), m_segments(segments) {
  // mimic the computation of Camera::pixelGLRatio()
  auto bbox = bounding_box();
  CGAL::qglviewer::Vec minv(bbox.xmin(), bbox.ymin(), 0);
  CGAL::qglviewer::Vec maxv(bbox.xmax(), bbox.ymax(), 0);
  auto diameter = (maxv - minv).norm();
  m_pixel_ratio = diameter / m_height;
}

//template<typename T>
//void SegmentsViewer<T>::resizeGL(int width, int height) {
//  CGAL::QGLViewer::resizeGL(width, height);
//  m_width = width;
//  m_height = height;
//  CGAL::qglviewer::Vec p;
//  auto ratio = camera()->pixelGLRatio(p);
//  m_pixel_ratio = ratio;
//  add_elements();
//} // namespace RotationalVisibilityGraph

template<typename T>
T SegmentsViewer<T>::pixel_ratio() const {
  return m_pixel_ratio;
}

//template<typename T>
//CGAL::Bbox_2 SegmentsViewer<T>::bounding_box() {
//  CGAL::Bbox_2 bbox;
//  for (const auto &segment : m_segments) {
//    bbox = bbox + segment.bbox();
//  }
//  return bbox;
//}

template<typename T>
void SegmentsViewer<T>::add_elements() {
  clear();
  CGAL::IO::Color c(75, 160, 255);
  CGAL::IO::Color p(255, 0, 0);
  for (const auto &segment : m_segments) {
    add_segment(segment.source(), segment.target(), c);
    add_point(segment.source(), p);
    add_point(segment.target(), p);
  }
}

template<typename T>
Drawer<T>::Drawer(QWidget *parent, const char *title):
    Base(parent, title, true, true, true, false, false) {}

//template<typename T>
//void Drawer<T>::resizeGL(int width, int height) {
//  CGAL::QGLViewer::resizeGL(width, height);
//  m_width = width;
//  m_height = height;
//  CGAL::qglviewer::Vec p;
//  auto ratio = camera()->pixelGLRatio(p);
//  m_pixel_ratio = ratio;
////  add_elements();
//}
//
template<typename T>
T Drawer<T>::pixel_ratio() const {
  return m_pixel_ratio;
}

template<typename T>
void Drawer<T>::addPolygon(const Polygon_2 &polygon,
                           const CGAL::IO::Color &edgeColor,
                           const CGAL::IO::Color &vertexColor) {
  for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); ++edge) {
    add_segment(edge->source(), edge->target(), edgeColor);
    add_point(edge->source(), vertexColor);
    add_point(edge->target(), vertexColor);
  }
}

template<typename T>
void Drawer<T>::addPolygons(const std::vector<Polygon_2> &polygons,
                            const CGAL::IO::Color &edgeColor,
                            const CGAL::IO::Color &vertexColor) {
  for (const auto &polygon : polygons)
    addPolygon(polygon, edgeColor, vertexColor);
}

template<typename T>
void Drawer<T>::addEdge(const Segment_2 &segment,
                        const CGAL::IO::Color &edgeColor,
                        const CGAL::IO::Color &vertexColor) {
  add_segment(segment.source(), segment.target(), edgeColor);
  add_point(segment.source(), vertexColor);
  add_point(segment.target(), vertexColor);
}

template<typename T>
void Drawer<T>::addPath(const std::vector<Vertex<T>> &path,
                        const CGAL::IO::Color &edgeColor,
                        const CGAL::IO::Color &vertexColor) {
  for (size_t i = 0; i < path.size() - 1; i++) {
    add_segment(path[i].getPoint(), path[i + 1].getPoint(), edgeColor);
    add_point(path[i].getPoint(), vertexColor);
  }
  add_point(path.back().getPoint(), vertexColor);
}

template<typename T>
void Drawer<T>::addPoint(const Point_2 &point, const CGAL::IO::Color &color) {
  add_point(point, color);
}

}
