#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_DRAWER_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_DRAWER_H_

#include <CGAL/Qt/Basic_viewer_qt.h>
#include <CGAL/Qt/init_ogl_context.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Segment_2.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <Utils/Utils.h>
#include <Utils/Pragma.h>

namespace RotationalVisibilityGraph {
#define RED CGAL::IO::Color(255, 0, 0)
#define GREEN CGAL::IO::Color(0, 255, 0)
#define BLUE CGAL::IO::Color(0, 0, 255)
#define BLACK CGAL::IO::Color(0, 0, 0)
#define WHITE CGAL::IO::Color(255, 255, 255)
#define ORANGE CGAL::IO::Color(255, 165, 0)
#define YELLOW CGAL::IO::Color(255, 255, 0)
#define PURPLE CGAL::IO::Color(128, 0, 128)
#define CYAN CGAL::IO::Color(0, 255, 255)
#define MAGENTA CGAL::IO::Color(255, 0, 255)
#define GRAY CGAL::IO::Color(128, 128, 128)
#define LIGHT_GRAY CGAL::IO::Color(192, 192, 192)
#define DARK_GRAY CGAL::IO::Color(64, 64, 64)
#define BROWN CGAL::IO::Color(165, 42, 42)
#define LIGHT_BLUE CGAL::IO::Color(173, 216, 230)
#define LIGHT_GREEN CGAL::IO::Color(144, 238, 144)
#define LIGHT_YELLOW CGAL::IO::Color(255, 255, 224)
#define LIGHT_ORANGE CGAL::IO::Color(255, 160, 122)
#define LIGHT_PURPLE CGAL::IO::Color(221, 160, 221)

template<typename T>
class SegmentsViewer : public CGAL::Basic_viewer_qt {
  using Base = Basic_viewer_qt;
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T

 public:
  /// Construct the viewer.
  /// @param parent the active window to draw
  /// @param pwh the polygon to view
  /// @param title the title of the window
  SegmentsViewer(QWidget *parent, const std::vector<Segment_2> &segments, const char *title = "Segments Viewer");
  /*! Intercept the resizing of the window.
   */
//  virtual void resizeGL(int width, int height);

  /*! Obtain the pixel ratio.
   */
  T pixel_ratio() const;

  /*! Compute the bounding box.
   */
//  CGAL::Bbox_2 bounding_box();

  /*! Compute the elements of a polygon with holes.
   */
  void add_elements();

 protected:
  virtual void keyPressEvent(QKeyEvent *e) {
    Base::keyPressEvent(e);
  }

 private:
  //! The window width in pixels.
  int m_width = CGAL_BASIC_VIEWER_INIT_SIZE_X;

  //! The window height in pixels.
  int m_height = CGAL_BASIC_VIEWER_INIT_SIZE_Y;

  //! The ratio between pixel and opengl units (in world coordinate system).
  double m_pixel_ratio = 1;

  const std::vector<Segment_2> &m_segments;
  std::shared_ptr<QApplication> m_app;
};

template<typename T>
class Drawer : public CGAL::Basic_viewer_qt {
  using Base = Basic_viewer_qt;
  DECL_CGAL_CARTESIAN_TYPES_T
  DECL_CGAL_POLYGON_TYPES_T
 public:
  Drawer(QWidget *parent, const char *title = "Polygon with Holes Viewer");
//  void resizeGL(int width, int height) override;
  T pixel_ratio() const;
  void addPolygon(const Polygon_2 &polygon, const CGAL::IO::Color &edgeColor, const CGAL::IO::Color &vertexColor);
  void addPolygons(const std::vector<Polygon_2> &polygons,
                   const CGAL::IO::Color &edgeColor,
                   const CGAL::IO::Color &vertexColor);
  void addEdge(const Segment_2 &segment, const CGAL::IO::Color &edgeColor, const CGAL::IO::Color &vertexColor);
  void addPath(const std::vector<Vertex<T>> &path,
               const CGAL::IO::Color &edgeColor,
               const CGAL::IO::Color &vertexColor);
  void addPoint(const Point_2 &point, const CGAL::IO::Color &color);

 protected:
  void keyPressEvent(QKeyEvent *e) override {
    Base::keyPressEvent(e);
  }

 private:
  int m_width = CGAL_BASIC_VIEWER_INIT_SIZE_X;
  int m_height = CGAL_BASIC_VIEWER_INIT_SIZE_Y;
  double m_pixel_ratio = 1;
};

template<typename T, typename Segment_2>
void draw(const std::vector<Segment_2> &segments, const char *title = "Segments Viewer") {
  using Viewer = SegmentsViewer<T>;
  CGAL::Qt::init_ogl_context(4, 3);
  int argc = 1;
  const char *argv[2] = {"t2_viewer", nullptr};
  QApplication app(argc, const_cast<char **>(argv));
  Viewer mainwindow(app.activeWindow(), segments, title);
  mainwindow.add_elements();
  mainwindow.show();
  app.exec();
}

template
class SegmentsViewer<double>;
template
class SegmentsViewer<float>;
template
class Drawer<double>;
template
class Drawer<float>;

} // end of rotational visibility graph namespace

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_DRAWER_H_
