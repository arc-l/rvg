#include <immintrin.h>
#include <vector>
#include <iostream>
#include <VisibilityGraph/Polygon.h>
#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>


using namespace RotationalVisibilityGraph;
using T=float;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

struct Point {
    T x, y;
};

// Scalar (non-AVX) check for remaining vertices
bool pointInPolygonScalar(const Point& p, const std::vector<Point>& polygon, int startIdx) {
    int inside = 0;
    int n = polygon.size();
    for (int i = startIdx; i < n; ++i) {
        const Point& p1 = polygon[i];
        const Point& p2 = polygon[(i + 1) % n];
        if ((p1.y < p.y && p2.y >= p.y) || (p2.y < p.y && p1.y >= p.y)) {
            T intersectX = p1.x + (p.y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
            if (intersectX > p.x) {
                inside ^= 1;
            }
        }
    }
    return inside & 1;
}

// AVX2 version for checking the point in a polygon
bool pointInPolygonAVX2(const Point& p, const std::vector<Point>& polygon) {
    int n = polygon.size();
    if (n < 3) return false;

    __m256 px = _mm256_set1_ps(p.x);
    __m256 py = _mm256_set1_ps(p.y);

    int inside = 0;
    int i = 0;

    // Process polygon vertices in chunks of 8
    for (; i + 8 <= n; i += 8) {
        // Load 8 consecutive points (x, y) for each polygon edge
        __m256 x1 = _mm256_setr_ps(
            polygon[i % n].x, polygon[(i+1) % n].x, polygon[(i+2) % n].x,
            polygon[(i+3) % n].x, polygon[(i+4) % n].x, polygon[(i+5) % n].x,
            polygon[(i+6) % n].x, polygon[(i+7) % n].x
        );
        __m256 y1 = _mm256_setr_ps(
            polygon[i % n].y, polygon[(i+1) % n].y, polygon[(i+2) % n].y,
            polygon[(i+3) % n].y, polygon[(i+4) % n].y, polygon[(i+5) % n].y,
            polygon[(i+6) % n].y, polygon[(i+7) % n].y
        );
        __m256 x2 = _mm256_setr_ps(
            polygon[(i+1) % n].x, polygon[(i+2) % n].x, polygon[(i+3) % n].x,
            polygon[(i+4) % n].x, polygon[(i+5) % n].x, polygon[(i+6) % n].x,
            polygon[(i+7) % n].x, polygon[(i+8) % n].x
        );
        __m256 y2 = _mm256_setr_ps(
            polygon[(i+1) % n].y, polygon[(i+2) % n].y, polygon[(i+3) % n].y,
            polygon[(i+4) % n].y, polygon[(i+5) % n].y, polygon[(i+6) % n].y,
            polygon[(i+7) % n].y, polygon[(i+8) % n].y
        );

        // Check if point (p.x, p.y) is within the Y bounds of the current edge
        __m256 cmp1 = _mm256_cmp_ps(y1, py, _CMP_LT_OQ); // y1 < py
        __m256 cmp2 = _mm256_cmp_ps(y2, py, _CMP_GE_OQ); // y2 >= py
        __m256 cmp3 = _mm256_cmp_ps(y2, py, _CMP_LT_OQ); // y2 < py
        __m256 cmp4 = _mm256_cmp_ps(y1, py, _CMP_GE_OQ); // y1 >= py
        __m256 withinYBounds = _mm256_or_ps(
            _mm256_and_ps(cmp1, cmp2),
            _mm256_and_ps(cmp3, cmp4)
        );

        // Calculate intersection of ray with polygon edge
        __m256 dx = _mm256_sub_ps(x2, x1);
        __m256 dy = _mm256_sub_ps(y2, y1);
        __m256 t = _mm256_div_ps(_mm256_sub_ps(py, y1), dy);
        __m256 intersectionX = _mm256_fmadd_ps(t, dx, x1);

        // Check if intersection is to the right of the point
        __m256 cmp5 = _mm256_cmp_ps(px, intersectionX, _CMP_LT_OQ); // px < intersectionX
        __m256 validIntersection = _mm256_and_ps(withinYBounds, cmp5);

        // Convert mask to integer
        int mask = _mm256_movemask_ps(validIntersection);
        inside ^= __builtin_popcount(mask);
    }

    // Handle remaining vertices (less than 8)
    if (i < n) {
        inside ^= pointInPolygonScalar(p, polygon, i);
    }

    // If the number of intersections is odd, the point is inside
    return inside & 1;
}

int main() {
    // Define a polygon with more than 8 vertices
    std::vector<Point> polygon = {
        {0.0f, 0.0f}, {5.0f, 0.0f}, {5.0f, 5.0f}, {3.0f, 6.0f},
        {1.0f, 7.0f}, {-2.0f, 4.0f}, {-3.0f, 2.0f}, {-4.0f, 1.0f},
        {-2.0f, -1.0f}
    };

    std::vector<Vertex<T>> vertices;
    for (const auto& p : polygon) {
        vertices.emplace_back(p.x, p.y);
    }
    Polygon<T> polygonRVG(vertices, false);
    // polygonRVG.draw("", "", true);
     

    // Point p = {1.5f, 3.0f}; // Point to test
    Point p = {-4.f, -1.f}; // Point to test
    Polygon_2 polygonCGAL = polygonRVG.getPolygon();
    Point_2 point(p.x, p.y);

    int iteration = 100000000;

    Utils::Timer timer(false);
    for (int i = 0; i< iteration; i++) {
        bool inside = pointInPolygonAVX2(p, polygon);
    }
    std::cout << "AVX2: " << timer.elapsed() << "s\n";

    timer.reset();
    for (int i = 0; i< iteration; i++) {
        bool inside = pointInPolygonScalar(p, polygon, 0);
    }
    Utils::print("Scalar: ", timer.elapsed(), "s");

    timer.reset();
    for (int i = 0; i< iteration; i++) {
        bool inside = IN_POLYGON(point, polygonCGAL);
    }
    Utils::print("CGAL: ", timer.elapsed(), "s");

    if (pointInPolygonAVX2(p, polygon)) {
        std::cout << "The point is inside the polygon.\n";
    } else {
        std::cout << "The point is outside the polygon.\n";
    }

    return 0;
}
