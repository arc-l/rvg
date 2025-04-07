#include "Polygon.h"
#include <Utils/Utils.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/extremal_polygon_2.h>

namespace RotationalVisibilityGraph {

template<typename T>
Polygon<T>::Polygon() : _vertices(), _centroid() {}

template<typename T>
Polygon<T>::Polygon(const std::vector<Vertex<T>> &vertices, bool sortVertices) :
    _vertices(vertices) {
  if (vertices.size() < 3) {
    throw std::invalid_argument("Polygon must have at least 3 vertices.");
  }
  _centroid = _calculateCentroid();
  if (sortVertices) {
    _sortVertices();
  }
  _updatePolygon();
}

template<typename T>
Polygon<T>::Polygon(const std::vector<Vertex<T>> &vertices, bool sortVertices, const Vertex<T> &center):Polygon(vertices, sortVertices) {
  _centroid = center;
}

template<typename T>
Polygon<T>::Polygon(const Polygon<T> &other) :
    _vertices(other._vertices), _centroid(other._centroid), _polygon(other._polygon) {}

template<typename T>
Polygon<T>::Polygon(const Polygon_2 &polygon) {
  _polygon = polygon;
  if (_polygon.is_clockwise_oriented())
    _polygon.reverse_orientation();
  for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); it++) {
    _vertices.emplace_back(CGAL::to_double(it->x()), CGAL::to_double(it->y()));
  }
  _centroid = _calculateCentroid();
}

template<typename T>
Polygon<T>::Polygon(const Polygon_2 &polygon, const Vertex<T> &center)
    : Polygon(polygon) {
  _centroid = center;
}

template<typename T>
Vertex<T> Polygon<T>::_calculateCentroid() const {
  T x = 0;
  T y = 0;
  for (const Vertex<T> &vertex : _vertices) {
    auto [vx, vy] = vertex.getCoord();
    x += vx;
    y += vy;
  }
  x /= _vertices.size();
  y /= _vertices.size();
  return Vertex<T>(x, y);
}

template<typename T>
void Polygon<T>::_sortVertices() {
  std::sort(_vertices.begin(), _vertices.end(), [this](const Vertex<T> &v1, const Vertex<T> &v2) {
    return _centroidAngle(v1) < _centroidAngle(v2);
  });
}

template<typename T>
T Polygon<T>::_centroidAngle(const Vertex<T> &v) const {
  auto [cx, cy] = _centroid.getCoord();
  auto [vx, vy] = v.getCoord();
  return atan2(vy - cy, vx - cx);
}

template<typename T>
int Polygon<T>::size() const {
  return _vertices.size();
}

template<typename T>
const Vertex<T> &Polygon<T>::operator[](int index) const {
  return _vertices[index];
}

template<typename T>
Vertex<T> &Polygon<T>::operator[](int index) {
  return _vertices[index];
}

template<typename T>
void Polygon<T>::translate(T x, T y) {
  for (Vertex<T> &vertex : _vertices) {
    auto [vx, vy] = vertex.getCoord();
    vertex.setPos(x + vx, y + vy);
  }
  _centroid.setPos(_centroid.getX() + x, _centroid.getY() + y);
  _updatePolygon();
}

template<typename T>
void Polygon<T>::rotate(T theta) {
  for (Vertex<T> &vertex : _vertices) {
    auto [vx, vy] = vertex.getCoord();
    auto [cx, cy] = _centroid.getCoord();
    T x = (vx - cx) * std::cos(theta) - (vy - cy) * std::sin(theta) + cx;
    T y = (vx - cx) * std::sin(theta) + (vy - cy) * std::cos(theta) + cy;
    vertex.setPos(x, y);
  }
//  _centroid = _calculateCentroid();
  _updatePolygon();
}

template<typename T>
void Polygon<T>::rotate(T theta, const Vertex<T> &center) {
  // rotate the polygon around the center
  for (Vertex<T> &vertex : _vertices) {
    auto [vx, vy] = vertex.getCoord();
    auto [cx, cy] = center.getCoord();
    T x = (vx - cx) * std::cos(theta) - (vy - cy) * std::sin(theta) + cx;
    T y = (vx - cx) * std::sin(theta) + (vy - cy) * std::cos(theta) + cy;
    vertex.setPos(x, y);
  }
//  _centroid = _calculateCentroid();
  _centroid = center;
  _updatePolygon();
}

/*
 * Returns a copy of the polygon that has been rotated by theta.
 */
template<typename T>
Polygon<T> Polygon<T>::rotateCopy(T theta) const {
  Polygon<T> copy(*this);
  copy.rotate(theta);
  return copy;
}

template<typename T>
Polygon<T> Polygon<T>::rotateCopy(T theta, const Vertex<T> &center) const {
  Polygon<T> copy(*this);
  copy.rotate(theta, center);
  return copy;
}

template<typename T>
typename Polygon<T>::Polygon_2 Polygon<T>::rotateCGALCopy(T theta) const {
  // this method does not change the _vertices, only return a transformed copy of Polygon_2 _polygon
  CGAL::Aff_transformation_2<K> rotate(CGAL::ROTATION, std::sin(theta), std::cos(theta));
  CGAL::Aff_transformationC2<K>
      translateToOrigin(CGAL::TRANSLATION, CGAL::Vector_2<K>(-_centroid.getX(), -_centroid.getY()));
  CGAL::Aff_transformationC2<K>
      translateBack(CGAL::TRANSLATION, CGAL::Vector_2<K>(_centroid.getX(), _centroid.getY()));
  Polygon_2 rotatedPolygon = _polygon;
  CGAL::transform(translateToOrigin, rotatedPolygon);
  CGAL::transform(rotate, rotatedPolygon);
  CGAL::transform(translateBack, rotatedPolygon);
  return rotatedPolygon;
}

template<typename T>
void Polygon<T>::moveTo(T x, T y, T theta) {
  translate(-_centroid.getX(), -_centroid.getY());
  rotate(theta);
  translate(x, y);
  _updatePolygon();
}

template<typename T>
Polygon<T> Polygon<T>::moveToCopy(T x, T y, T theta) const {
  Polygon<T> copy(*this);
  copy.moveTo(x, y, theta);
  for(auto& v : copy._vertices){
    v.setTheta(theta);
  }
  return copy;
}

template<typename T>
void Polygon<T>::_updatePolygon() {
  _polygon.clear();
  for (size_t i = 0; i < _vertices.size(); i++) {
    _polygon.push_back(_vertices[i].getPoint());
  }
  try{
    if (_polygon.is_clockwise_oriented())
      _polygon.reverse_orientation();
  }
  catch(...){
    draw("", "", true);
  }
}

template<typename T>
bool Polygon<T>::isConvex() const {
  return CGAL::is_convex_2(_polygon.vertices_begin(), _polygon.vertices_end());
}

template<typename T>
void Polygon<T>::invert() {
  for (Vertex<T> &vertex : _vertices) {
    auto [x, y] = vertex.getCoord();
    vertex.setPos(-1.0 * x, -1.0 * y);
  }
  _centroid.setPos(-1.0 * _centroid.getX(), -1.0 * _centroid.getY());
//  _centroid = _calculateCentroid();
  _updatePolygon();
}

template<typename T>
const typename Polygon<T>::Polygon_2 &Polygon<T>::getPolygon() const {
  return _polygon;
}

template<typename T>
Polygon<T> Polygon<T>::scale(T scale) const {
  std::vector<Vertex<T>> enlargedVertices;
  for (const Vertex<T> &vertex : _vertices) {
    auto [x, y] = vertex.getCoord();
    auto [cx, cy] = _centroid.getCoord();
    T dx = x - cx;
    T dy = y - cy;
    T nx = cx + dx * scale;
    T ny = cy + dy * scale;
    enlargedVertices.emplace_back(nx, ny);
  }
  return Polygon<T>(enlargedVertices, false);
}

template<typename T>
typename Polygon<T>::Polygon_2 Polygon<T>::getClockWise() const {
  Polygon_2 hole = _polygon;
  if (hole.is_counterclockwise_oriented())
    hole.reverse_orientation();
  return hole;
}
template<typename T>
typename Polygon<T>::Polygon_2 Polygon<T>::getCounterClockWise() const {
  Polygon_2 hole = _polygon;
  if (hole.is_clockwise_oriented())
    hole.reverse_orientation();
  return hole;
}

template<typename T>
const Vertex<T> &Polygon<T>::getCentroid() const {
  return _centroid;
}

template<typename T>
std::vector<T> Polygon<T>::getX() const {
  std::vector<T> x;
  for (const Vertex<T> &vertex : _vertices) {
    x.push_back(vertex.getX());
  }
  x.push_back(_vertices[0].getX());
  return x;
}

template<typename T>
std::vector<T> Polygon<T>::getY() const {
  std::vector<T> y;
  for (const Vertex<T> &vertex : _vertices) {
    y.push_back(vertex.getY());
  }
  y.push_back(_vertices[0].getY());
  return y;
}

template<typename T>
std::vector<T> Polygon<T>::getTheta() const {
  std::vector<T> theta;
  for (const Vertex<T> &vertex : _vertices) {
    theta.push_back(vertex.getTheta());
  }
  theta.push_back(_vertices[0].getTheta());
  return theta;
}
template<typename T>
bool Polygon<T>::intersects(const Polygon<T> &other) const {
  return CGAL::do_intersect(_polygon, other.getPolygon());
}

template<typename T>
const std::vector<Vertex<T>> &Polygon<T>::getVertices() const {
  return _vertices;
}

template<typename T>
bool Polygon<T>::operator==(const Polygon<T> &other) const {
  size_t firstSame;
  bool foundSame = false;
  for (size_t i = 0; i < _vertices.size(); i++) {
    if (_vertices[i].dist(other[0]) < 1e-6) {
      firstSame = i;
      foundSame = true;
      break;
    }
  }
  if (!foundSame)
    return false;

  for (size_t i = 0; i < _vertices.size(); i++) {
    if (_vertices[(i + firstSame) % _vertices.size()].dist(other[i]) > 1e-6) {
      Utils::print("i: ", i);
      Utils::print("dist: ", _vertices[(i + firstSame) % _vertices.size()].dist(other[i]));
      return false;
    }
  }
  return true;
}

template<typename T>
Polygon<T> Polygon<T>::bbox() const {
  T minX = std::numeric_limits<T>::max();
  T minY = std::numeric_limits<T>::max();
  T maxX = std::numeric_limits<T>::min();
  T maxY = std::numeric_limits<T>::min();
  for (const Vertex<T> &vertex : _vertices) {
    auto [x, y] = vertex.getCoord();
    minX = std::min(minX, x);
    minY = std::min(minY, y);
    maxX = std::max(maxX, x);
    maxY = std::max(maxY, y);
  }
  std::vector<Vertex<T>> vertices = {
      Vertex<T>(minX, minY),
      Vertex<T>(maxX, minY),
      Vertex<T>(maxX, maxY),
      Vertex<T>(minX, maxY)
  };
  return Polygon<T>(vertices, false);
}

template<typename T>
Polygon<T> Polygon<T>::merge(const Polygon<T> &other) const {
  const Polygon_2 &polygon1 = _polygon;
  const Polygon_2 &polygon2 = other.getPolygon();
  Polygon_with_holes_2 mergedPolygon;
  CGAL::join(polygon1, polygon2, mergedPolygon);
  Polygon_2 resultPolygon = mergedPolygon.outer_boundary();
  std::vector<Vertex<T>> vertices;
  Polygon_2::Vertex_circulator first, curr, prev;
  first = curr = prev = resultPolygon.vertices_circulator();
  do {
    if (curr == prev) {
      vertices.push_back(Vertex<T>(CGAL::to_double(curr->x()), CGAL::to_double(curr->y())));
      curr++;
      continue;
    }
    if (CGAL::squared_distance(*curr, *prev) < RVG_EPS) {
      Utils::print("Found duplicate vertices in Polygon merge: ", *curr, *prev);
      curr = resultPolygon.erase(curr);
    } else {
      vertices.push_back(Vertex<T>(CGAL::to_double(curr->x()), CGAL::to_double(curr->y())));
      curr++;
      prev++;
    }
  } while (curr != first);
  return Polygon<T>(vertices, false);
}

template<typename T>
bool Polygon<T>::contains(const Vertex<T> &v) const {
  return IN_POLYGON(v.getPoint(), _polygon);
}

template<typename T>
std::vector<Polygon<T>> Polygon<T>::convexDecomposition() const {
  DECL_CGAL_CONVEX_DECOMPOSITION_TYPES_T
  std::vector<Polygon<T>> convexPolygons;
  std::vector<CPolygon_2> convexPolygonsCGAL;
  CPolygon_2 polygon;
  for (const Vertex<T> &vertex : _vertices) {
    polygon.push_back(CPoint_2(vertex.getX(), vertex.getY()));
  }
  if(polygon.is_clockwise_oriented())
    polygon.reverse_orientation();
  CGAL::optimal_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(convexPolygonsCGAL));
  if(!CGAL::partition_is_valid_2(_polygon.vertices_begin(), _polygon.vertices_end(), convexPolygonsCGAL.begin(), convexPolygonsCGAL.end())){
    throw std::runtime_error("Invalid convex decomposition");
  }
  for (const CPolygon_2 &convexPolygon : convexPolygonsCGAL) {
    Polygon_2 tmpPolygon;
    for (auto it = convexPolygon.vertices_begin(); it != convexPolygon.vertices_end(); it++) {
      tmpPolygon.push_back(Point_2(CGAL::to_double(it->x()), CGAL::to_double(it->y())));
    }
    convexPolygons.emplace_back(tmpPolygon);
  }
  return convexPolygons;
}

template<typename T>
void Polygon<T>::draw(const std::string &figPath, const std::string &title, bool show) const {
  if(figPath.empty() && !show) {
    Utils::print("No figPath provided, drawing will not be saved or shown");
    return;
  }
  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  std::vector<T> polyX = this->getX(), polyY = this->getY();
  pythonScript += "obs = np.array([\n";
  for (size_t i = 0; i < polyX.size(); i++) {
    pythonScript += "\t[" + std::to_string(polyX[i]) + ", " + std::to_string(polyY[i]) + "],\n";
  }
  pythonScript += "])\n";
  pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-o', color='darkcyan', markersize=2.0, linewidth=1.0)\n";
  if (!title.empty())
    pythonScript += "plt.title('" + title + "')\n";
  pythonScript += "plt.axis('equal')\n";
  if(!figPath.empty()) {
    pythonScript += "plt.savefig('" + figPath + "', dpi=500, bbox_inches='tight')\n";
  }
  if (show) pythonScript += "plt.show()\n";
  std::string pythonScriptPath = "draw.py";
  Utils::writeStringToFile<T>(pythonScript, pythonScriptPath);
  Utils::runPythonScriptAndRemove<T>(pythonScriptPath);
}

template<typename T>
bool Polygon<T>::intersectsEdge(const Edge<T> &edge) const {
  for (size_t i = 0; i < _vertices.size(); i++) {
    if (edge.intersects(Edge<T>(_vertices[i], _vertices[(i + 1) % _vertices.size()])))
      return true;
  }
  return false;
}

template<typename T>
std::string Polygon<T>::draw3D(const std::string &type, bool dashed) const {
  std::string pythonScript;
  std::vector<T> polyX = this->getX(), polyY = this->getY(), polyTheta = this->getTheta();
  pythonScript += "obs = np.array([\n";
  for (size_t i = 0; i < polyX.size(); i++) {
    pythonScript += "\t[" + std::to_string(polyX[i]) + ", " + std::to_string(polyY[i]) +", " + std::to_string(polyTheta[i]) + "],\n";
  }
  pythonScript += "])\n";
  std::string dash = dashed ? "-" : "";
  if (type == "obs")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "o', color='darkcyan', markersize=2.0, linewidth=1.0)\n";
  else if (type == "start")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "o', color='deeppink', markersize=2.0, linewidth=1.0)\n";
  else if (type == "goal")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "o', color='darkviolet', markersize=2.0, linewidth=1.0)\n";
  else if (type == "path")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "o', color='navy', markersize=2.0, linewidth=1.0, alpha=0.5)\n";
  else if (type == "hole")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "o', color='tab:red', markersize=2.0, linewidth=1.0)\n";
  else if (type == "graph"){
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "', color='lightcoral', linewidth=0.5)\n";
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], 'o', color='bisque', markersize=2.0)\n";
  }
  else if (type == "obs3d"){
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], '-" + dash + "', color='darkcyan', markersize=2.0, linewidth=1.0)\n";
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], obs[:, 2], 'o', color='#EC008C', markersize=2.0)\n";
  }
  else
    throw std::runtime_error("Unknown type: " + type);
  return pythonScript;
}

template<typename T>
std::string Polygon<T>::draw(const std::string &type, bool dashed) const {
  std::string pythonScript;
  std::vector<T> polyX = this->getX(), polyY = this->getY();
  pythonScript += "obs = np.array([\n";
  for (size_t i = 0; i < polyX.size(); i++) {
    pythonScript += "\t[" + std::to_string(polyX[i]) + ", " + std::to_string(polyY[i]) + "],\n";
  }
  pythonScript += "])\n";
  std::string dash = dashed ? "-" : "";
  if (type == "obs")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', color='darkcyan', markersize=2.0, linewidth=1.0)\n";
  else if (type == "start")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', color='deeppink', markersize=2.0, linewidth=1.0)\n";
  else if (type == "start_fill")
    pythonScript += "ax.fill(obs[:, 0], obs[:, 1], '-" + dash + "o', color='deeppink')\n";
  else if (type == "goal")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', color='darkviolet', markersize=2.0, linewidth=1.0)\n";
  else if (type == "goal_fill")
    pythonScript += "ax.fill(obs[:, 0], obs[:, 1], '-" + dash + "o', color='darkviolet')\n";
  else if (type == "path")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', color='navy', markersize=2.0, linewidth=1.0, alpha=0.5)\n";
  else if (type == "path_fill")
    pythonScript += "ax.fill(obs[:, 0], obs[:, 1], '-" + dash + "o', color='navy', alpha=0.5)\n";
  else if (type == "hole")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', color='tab:red', markersize=2.0, linewidth=1.0)\n";
  else if (type=="default")
    pythonScript += "ax.plot(obs[:, 0], obs[:, 1], '-" + dash + "o', markersize=2.0, linewidth=2.0, alpha=0.8)\n";
  else
    throw std::runtime_error("Unknown type: " + type);
  return pythonScript;
}

template<typename T>
typename Polygon<T>::Polygon_with_holes_2 Polygon<T>::getComplement() const {
  Polygon_with_holes_2 complement;
  CGAL::complement(_polygon, complement);
  return complement;
}

template <typename T>
Polygon<T> Polygon<T>::findMaximumInscribedCircle(int numVertices) const {
    const T tolerance = static_cast<T>(1e-6); // Convergence threshold
    const int maxIterations = 1000;
    T stepSize = static_cast<T>(0.01); // Initial step size

    T maxRadius = 0;
    Vertex<T> center = _centroid;

    for (int iter = 0; iter < maxIterations; ++iter) {
        T minDist = std::numeric_limits<T>::max();

        // Calculate the minimum distance to all edges
        for (size_t i = 0; i < _vertices.size(); ++i) {
            const auto& a = _vertices[i];
            const auto& b = _vertices[(i + 1) % _vertices.size()];
            T dist = center.distanceToEdge(a, b);
            minDist = std::min(minDist, dist);
        }

        // Update the radius
        if (std::abs(minDist - maxRadius) < tolerance) {
            Utils::print("Converged after ", iter, " iterations");
            break; // Converged
        }
        maxRadius = minDist;

        // Gradient descent to maximize the radius
        T gradX = 0, gradY = 0;
        for (size_t i = 0; i < _vertices.size(); ++i) {
            const auto& a = _vertices[i];
            const auto& b = _vertices[(i + 1) % _vertices.size()];
            T dist = center.distanceToEdge(a, b);
            if (std::abs(dist - minDist) < tolerance) {
                // Compute gradient for the closest edge
                T dx = b.getY() - a.getY();
                T dy = a.getX() - b.getX();
                T length = std::sqrt(dx * dx + dy * dy);
                gradX += dx / length;
                gradY += dy / length;
            }
        }

        // Normalize the gradient
        T gradNorm = std::sqrt(gradX * gradX + gradY * gradY);
        if (gradNorm > tolerance) {
            gradX /= gradNorm;
            gradY /= gradNorm;
        }

        // Update the center
        center = Vertex<T>(center.getX() + stepSize * gradX, center.getY() + stepSize * gradY);

        // Adjust step size (optional)
        stepSize *= static_cast<T>(0.99); // Slow down as we approach the solution
    }

  std::vector<Vertex<T>> vertices;
  for (int i = 0; i < numVertices; ++i) {
    T angle = static_cast<T>(2 * M_PI * i / numVertices);
    T x = center.getX() + maxRadius * std::cos(angle);
    T y = center.getY() + maxRadius * std::sin(angle);
    vertices.emplace_back(x, y);
  }

  return Polygon<T>(vertices, false);
}


template
class Polygon<double>;
template
class Polygon<float>;

}