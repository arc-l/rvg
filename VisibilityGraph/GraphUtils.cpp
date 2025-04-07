#include "GraphUtils.h"

namespace RotationalVisibilityGraph {
template<typename T>
PolygonDrawer<T>::PolygonDrawer(const std::vector<std::shared_ptr<Polygon<T>>> &polygons) : _polygons(polygons) {}

template <typename T>
void PolygonDrawer<T>::draw() {
    Polygon_set_2 S;
    Polygon_with_holes_2 pwh(_polygons[0]->getPolygon());
    for (size_t i = 1; i < _polygons.size(); i++) {
        Polygon_with_holes_2 tmpPwh;
        if (CGAL::join(pwh, _polygons[i]->getPolygon(), tmpPwh)) {
            pwh = tmpPwh;
        }
        else {
            _polygons[i]->draw(S);
        }
    }
    S.insert(pwh);
    CGAL::draw(S);
}

template class PolygonDrawer<double>;
template class PolygonDrawer<float>;
}
