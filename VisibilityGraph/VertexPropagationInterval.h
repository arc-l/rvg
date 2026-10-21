#ifndef ROTATIONALVISIBILITYGRAPH_VERTEXPROPAGATIONINTERVAL_H_
#define ROTATIONALVISIBILITYGRAPH_VERTEXPROPAGATIONINTERVAL_H_

#include <VisibilityGraph/VertexPropagationCuda.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace RotationalVisibilityGraph::detail {

enum class ScalarCandidateClassification : std::uint8_t {
  Reject,
  Accept,
  Uncertain
};

inline double outwardLower(double value) {
  return std::nextafter(value, -std::numeric_limits<double>::infinity());
}

inline double outwardUpper(double value) {
  return std::nextafter(value, std::numeric_limits<double>::infinity());
}

inline PropagationInterval intervalSubtract(
    PropagationInterval left,
    PropagationInterval right
) {
  return {
      outwardLower(left.lower - right.upper),
      outwardUpper(left.upper - right.lower)
  };
}

inline PropagationInterval intervalMultiply(
    PropagationInterval left,
    PropagationInterval right
) {
  const double products[4] = {
      left.lower * right.lower,
      left.lower * right.upper,
      left.upper * right.lower,
      left.upper * right.upper
  };
  return {
      outwardLower(*std::min_element(products, products + 4)),
      outwardUpper(*std::max_element(products, products + 4))
  };
}

inline PropagationInterval orientationInterval(
    const PropagationIntervalPoint &first,
    const PropagationIntervalPoint &second,
    const PropagationIntervalPoint &third
) {
  const auto firstX = intervalSubtract(second.x, first.x);
  const auto firstY = intervalSubtract(second.y, first.y);
  const auto secondX = intervalSubtract(third.x, first.x);
  const auto secondY = intervalSubtract(third.y, first.y);
  return intervalSubtract(
      intervalMultiply(firstX, secondY),
      intervalMultiply(firstY, secondX)
  );
}

inline int certifiedSign(PropagationInterval interval) {
  if (interval.lower > 0) return 1;
  if (interval.upper < 0) return -1;
  return 0;
}

enum class CertifiedComparison : std::uint8_t {
  False,
  True,
  Uncertain
};

inline CertifiedComparison certifiedLessEqual(
    PropagationInterval left,
    PropagationInterval right
) {
  if (left.upper <= right.lower) return CertifiedComparison::True;
  if (left.lower > right.upper) return CertifiedComparison::False;
  return CertifiedComparison::Uncertain;
}

inline CertifiedComparison certifiedGreater(
    PropagationInterval left,
    PropagationInterval right
) {
  if (left.lower > right.upper) return CertifiedComparison::True;
  if (left.upper <= right.lower) return CertifiedComparison::False;
  return CertifiedComparison::Uncertain;
}

inline ScalarCandidateClassification classifyScalarCandidateAabb(
    const PropagationSourceGeometry &source,
    const PropagationTargetGeometry &target
) {
  if (source.point.x.upper < target.boundingX.lower ||
      source.point.x.lower > target.boundingX.upper ||
      source.point.y.upper < target.boundingY.lower ||
      source.point.y.lower > target.boundingY.upper) {
    return ScalarCandidateClassification::Reject;
  }
  // CGAL's adaptive exact predicates are faster than scalar interval
  // orientations for the survivors. Treat them conservatively as uncertain.
  return ScalarCandidateClassification::Uncertain;
}

inline ScalarCandidateClassification classifyScalarCandidate(
    const PropagationSourceGeometry &source,
    const PropagationTargetGeometry &target,
    const std::vector<PropagationIntervalPoint> &polygonPoints
) {
  if (source.point.x.upper < target.boundingX.lower ||
      source.point.x.lower > target.boundingX.upper ||
      source.point.y.upper < target.boundingY.lower ||
      source.point.y.lower > target.boundingY.upper) {
    return ScalarCandidateClassification::Reject;
  }
  if (!source.onHole && !target.onHole) {
    const int sourceLeft = certifiedSign(orientationInterval(
        target.point, source.point, source.neighborLeft
    ));
    const int sourceRight = certifiedSign(orientationInterval(
        target.point, source.point, source.neighborRight
    ));
    const int targetLeft = certifiedSign(orientationInterval(
        source.point, target.point, target.neighborLeft
    ));
    const int targetRight = certifiedSign(orientationInterval(
        source.point, target.point, target.neighborRight
    ));
    if (sourceLeft == 0 || sourceRight == 0 ||
        targetLeft == 0 || targetRight == 0) {
      return ScalarCandidateClassification::Uncertain;
    }
    if (sourceLeft != sourceRight || targetLeft != targetRight) {
      return ScalarCandidateClassification::Reject;
    }
  }
  if (target.polygonCount < 3) {
    return ScalarCandidateClassification::Reject;
  }
  int winding = 0;
  for (std::uint32_t edgeIndex = 0;
       edgeIndex < target.polygonCount; ++edgeIndex) {
    const auto &first = polygonPoints[target.polygonOffset + edgeIndex];
    const auto &second = polygonPoints[
        target.polygonOffset + (edgeIndex + 1) % target.polygonCount
    ];
    const bool overlapsEdgeBounds =
        source.point.x.upper >= std::min(first.x.lower, second.x.lower) &&
        source.point.x.lower <= std::max(first.x.upper, second.x.upper) &&
        source.point.y.upper >= std::min(first.y.lower, second.y.lower) &&
        source.point.y.lower <= std::max(first.y.upper, second.y.upper);
    if (overlapsEdgeBounds &&
        certifiedSign(orientationInterval(first, second, source.point)) == 0) {
      return ScalarCandidateClassification::Uncertain;
    }
    const CertifiedComparison firstBelow =
        certifiedLessEqual(first.y, source.point.y);
    if (firstBelow == CertifiedComparison::Uncertain) {
      return ScalarCandidateClassification::Uncertain;
    }
    if (firstBelow == CertifiedComparison::True) {
      const CertifiedComparison secondAbove =
          certifiedGreater(second.y, source.point.y);
      if (secondAbove == CertifiedComparison::Uncertain) {
        return ScalarCandidateClassification::Uncertain;
      }
      if (secondAbove == CertifiedComparison::True) {
        const int sign = certifiedSign(
            orientationInterval(first, second, source.point)
        );
        if (sign == 0) return ScalarCandidateClassification::Uncertain;
        if (sign > 0) ++winding;
      }
    } else {
      const CertifiedComparison secondBelow =
          certifiedLessEqual(second.y, source.point.y);
      if (secondBelow == CertifiedComparison::Uncertain) {
        return ScalarCandidateClassification::Uncertain;
      }
      if (secondBelow == CertifiedComparison::True) {
        const int sign = certifiedSign(
            orientationInterval(first, second, source.point)
        );
        if (sign == 0) return ScalarCandidateClassification::Uncertain;
        if (sign < 0) --winding;
      }
    }
  }
  return winding == 0
      ? ScalarCandidateClassification::Reject
      : ScalarCandidateClassification::Accept;
}

}  // namespace RotationalVisibilityGraph::detail

#endif  // ROTATIONALVISIBILITYGRAPH_VERTEXPROPAGATIONINTERVAL_H_
