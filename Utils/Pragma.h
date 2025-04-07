#ifndef ROTATIONALVISIBILITYGRAPH_UTILS_PRAGMA_H_
#define ROTATIONALVISIBILITYGRAPH_UTILS_PRAGMA_H_

#define PI M_PI
#define RVG_EPS (1e-9)
#define ASSERT(var) {do{if(!(var)){exit(EXIT_FAILURE);}}while(0);}
#define ASSERT_MSG(var, msg) {do{if(!(var)){printf(msg);exit(EXIT_FAILURE);}}while(0);}
#define ASSERT_MSGV(var, msg, ...) {do{if(!(var)){printf(msg,__VA_ARGS__);exit(EXIT_FAILURE);}}while(0);}
#define FUNCTION_NOT_IMPLEMENTED throw new std::runtime_error(std::string("Function ")+__FUNCTION__+std::string(" not implemented!"));

//typedef typename CGAL::Simple_cartesian<T> Kernel;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//#define EPICK
#define EPECK
//#define SIMPLE_CARTESIAN
#ifdef EPICK
#define DECL_CGAL_CARTESIAN_TYPES_T \
typedef CGAL::Exact_predicates_inexact_constructions_kernel K; \
typedef K::Point_2 Point_2;               \
typedef K::Segment_2 Segment_2;     \

#define DECL_CGAL_POLYGON_TYPES_T \
typedef CGAL::Polygon_2<K> Polygon_2;\
typedef CGAL::Polygon_set_2<K> Polygon_set_2;
#elif defined(EPECK)
#define DECL_CGAL_CARTESIAN_TYPES_T \
typedef CGAL::Exact_predicates_exact_constructions_kernel K; \
typedef K::Point_2 Point_2;               \
typedef K::Segment_2 Segment_2;     \

#define DECL_CGAL_POLYGON_TYPES_T \
typedef CGAL::Polygon_2<K> Polygon_2;\
typedef CGAL::Polygon_set_2<K> Polygon_set_2;\
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

#define DECL_CGAL_CONVEX_DECOMPOSITION_TYPES_T \
typedef CGAL::Partition_traits_2<K>                         Traits; \
typedef Traits::Polygon_2                                   CPolygon_2; \
typedef Traits::Point_2                                     CPoint_2;

#ifdef VISIBILITY_QUERY_TEV
#define DECL_CGAL_VISIBILITY_GRAPH_TYPES_T \
  typedef CGAL::Arr_segment_traits_2<K>                                                         Traits_2;\
  typedef CGAL::Arrangement_2<Traits_2>                                                         Arrangement_2;\
  typedef Arrangement_2::Halfedge_const_handle                                                  Halfedge_const_handle; \
  typedef Arrangement_2::Vertex_const_handle                                                    Vertex_const_handle;\
  typedef Arrangement_2::Face_handle                                                            Face_handle;  \
  typedef Arrangement_2::Face_const_handle                                                      Face_const_handle;  \
  typedef Arrangement_2::Edge_const_iterator                                                    Edge_const_iterator;\
  typedef Arrangement_2::Ccb_halfedge_circulator                                                Ccb_halfedge_circulator; \
  typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>                                VQ;\
  typedef CGAL::Convex_hull_traits_adapter_2<K,CGAL::Pointer_property_map<Point_2>::type >      Convex_hull_traits_2;\
  typedef CGAL::Arr_naive_point_location<Arrangement_2>                                         PL_2;

#elif defined(VISIBILITY_QUERY_RSV)
#define DECL_CGAL_VISIBILITY_GRAPH_TYPES_T \
  typedef CGAL::Arr_segment_traits_2<K>                                                         Traits_2;\
  typedef CGAL::Arrangement_2<Traits_2>                                                         Arrangement_2;\
  typedef Arrangement_2::Halfedge_const_handle                                                  Halfedge_const_handle; \
  typedef Arrangement_2::Vertex_const_handle                                                    Vertex_const_handle;\
  typedef Arrangement_2::Face_handle                                                            Face_handle;  \
  typedef Arrangement_2::Face_const_handle                                                      Face_const_handle;  \
  typedef Arrangement_2::Edge_const_iterator                                                    Edge_const_iterator;\
  typedef Arrangement_2::Ccb_halfedge_circulator                                                Ccb_halfedge_circulator; \
  typedef CGAL::Rotational_sweep_visibility_2<Arrangement_2>                                    VQ;\
  typedef CGAL::Convex_hull_traits_adapter_2<K,CGAL::Pointer_property_map<Point_2>::type >      Convex_hull_traits_2;\
  typedef CGAL::Arr_naive_point_location<Arrangement_2>                                         PL_2;

#endif

#elif defined(SIMPLE_CARTESIAN)
#define DECL_CGAL_CARTESIAN_TYPES_T \
typedef typename CGAL::Simple_cartesian<T> K;\
typedef typename K::Point_2 Point_2;               \
typedef typename K::Segment_2 Segment_2;     \

#define DECL_CGAL_POLYGON_TYPES_T \
typedef typename CGAL::Polygon_2<K> Polygon_2;\
typedef typename CGAL::Polygon_set_2<K> Polygon_set_2;

#endif

#define IN_POLYGON(p, poly) CGAL::bounded_side_2(poly.vertices_begin(), poly.vertices_end(), p, K()) == CGAL::ON_BOUNDED_SIDE
#define ON_EDGE(p, poly) poly.bounded_side(p) == CGAL::ON_BOUNDARY
#define IN_ANY_POLYGON(p, polys) std::any_of(polys.begin(), polys.end(), [p](const Polygon_2 &poly){return IN_POLYGON(p, poly);})
#define ON_ANY_EDGE(p, polys) std::any_of(polys.begin(), polys.end(), [p](const Polygon_2 &poly){return ON_EDGE(p, poly);})


// Drawer


#define INIT_DRAWER(title) \
CGAL::Qt::init_ogl_context(4, 3);\
int argc = 1;\
const char *argv[2] = {"t2_viewer", nullptr};\
QApplication app(argc, const_cast<char **>(argv));\
Drawer<T> drawer(app.activeWindow(), title);

#define SHOW_DRAWER \
drawer.show(); \
app.exec();


//openmp macro prepare
#ifdef _MSC_VER
#define STRINGIFY_OMP(X) #X
#define PRAGMA __pragma
#else
#define STRINGIFY_OMP(X) #X
#define PRAGMA _Pragma
#endif
#ifndef NO_OPENMP
//openmp convenient functions
#define OMP_PARALLEL_FOR_ PRAGMA(STRINGIFY_OMP(omp parallel for schedule(static)))
#define OMP_PARALLEL_FOR_I(...) PRAGMA(STRINGIFY_OMP(omp parallel for schedule(static) __VA_ARGS__))
#define OMP_PARALLEL_FOR_X(X) PRAGMA(STRINGIFY_OMP(omp parallel for num_threads(X) schedule(static)))
#define OMP_PARALLEL_FOR_XI(X, ...) PRAGMA(STRINGIFY_OMP(omp parallel for num_threads(X) schedule(static) __VA_ARGS__))
#define OMP_ADD(...) reduction(+: __VA_ARGS__)
#define OMP_PRI(...) private(__VA_ARGS__)
#define OMP_FPRI(...) firstprivate(__VA_ARGS__)
#define OMP_ATOMIC_    PRAGMA(STRINGIFY_OMP(omp atomic))
#define OMP_ATOMIC_CAPTURE_    PRAGMA(STRINGIFY_OMP(omp atomic capture))
#define OMP_CRITICAL_ PRAGMA(STRINGIFY_OMP(omp critical))
#define OMP_FLUSH_(X) PRAGMA(STRINGIFY_OMP(omp flush(X)))
#define OMP_BARRIER_ PRAGMA(STRINGIFY_OMP(omp barrier))
#else
//openmp convenient functions
#define OMP_PARALLEL_FOR_
#define OMP_PARALLEL_FOR_I(...)
#define OMP_PARALLEL_FOR_X(X)
#define OMP_PARALLEL_FOR_XI(X,...)
#define OMP_ADD(...)
#define OMP_PRI(...)
#define OMP_FPRI(...)
#define OMP_ATOMIC_
#define OMP_ATOMIC_CAPTURE_
#define OMP_CRITICAL_
#define OMP_FLUSH_(X)
#define OMP_BARRIER_
#endif

#define PYTHON_IMPORTS(script) \
script += "import numpy as np\n";\
script += "import matplotlib.pyplot as plt\n";\
script += "from PIL import Image\n";\
script += "import os\n";         \
script += "plt.rcParams['font.family'] = 'serif'\n";\
script += "plt.rcParams['font.serif'] = ['Times New Roman']\n"; \
script += "fig, ax = plt.subplots(dpi=100)\n";\
script += "ax.set_facecolor('gainsboro')\n";\
script += "plt.axis('equal')\n";



#define PYTHON_IMPORTS_3D(script) \
script += "import numpy as np\n";\
script += "import matplotlib.pyplot as plt\n";\
script += "import os\n";         \
script += "plt.rcParams['font.family'] = 'serif'\n";\
script += "plt.rcParams['font.serif'] = ['Times New Roman']\n"; \
script += "fig, ax = plt.subplots(dpi=100)\n";\
script += "ax = fig.add_subplot(111, projection='3d')\n";\
script += "plt.axis('equal')\n";\
script += "plt.axis('off')\n";


#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#define TRACY_DO(x) x
#else
#define TRACY_DO(x)
#endif

#endif //ROTATIONALVISIBILITYGRAPH_UTILS_PRAGMA_H_

