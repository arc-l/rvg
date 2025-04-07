#include <iostream>
#include <cmath>  // Include cmath for fmod

bool circular_overlap_and_merge(double lb1, double ub1, double lb2, double ub2, double cycle, double &newLb, double &newUb) {
  if (fabs(ub1 - cycle) < 1e-5 || fabs(ub2 - cycle) < 1e-5) {
    //deal with full span or right end included
    bool is_full_span1 = fabs(fabs(ub1 - lb1) - cycle) < 1e-5;
    bool is_full_span2 = fabs(fabs(ub2 - lb2) - cycle) < 1e-5;
    if (is_full_span1 || is_full_span2) {
      // Full coverage of the circle
      newLb = 0;
      newUb = cycle;
      return true; // Full coverage of the circle
    }

    if (fabs(ub1 - cycle) < 1e-5) {
      if (lb2 < ub2) {
        if (lb1 <= ub2) {
          newLb = std::min(lb1, lb2);
          newUb = ub1;
          return true;
        } else {
          if (fabs(lb2) < 1e-5) {
            newLb = lb1;
            newUb = ub2;
            return true;
          }
          return false;
        }
      } else {
        newLb = std::min(lb2, lb1);
        newUb = ub2;
        if (newLb <= newUb) {
          newLb = 0;
          newUb = cycle;
        }
        return true;
      }
    }
    if (fabs(ub2 - cycle) < 1e-5) {
      if (lb1 < ub1) {
        if (lb2 <= ub1) {
          newLb = std::min(lb1, lb2);
          newUb = ub2;
          return true;
        } else {
          if (fabs(lb1) < 1e-5) {
            newLb = lb2;
            newUb = ub1;
            return true;
          }
          return false;
        }
      } else {
        newLb = std::min(lb1, lb2);
        newUb = ub1;
        if (newLb <= newUb) {
          newLb = 0;
          newUb = cycle;
        }
        return true;
      }
    }
    return false;
  } else {
    // deal with normal case
    if (lb1 <= ub1 && lb2 <= ub2) {
      if (lb1 <= ub2 && lb2 <= ub1) {
        newLb = std::min(lb1, lb2);
        newUb = std::max(ub1, ub2);
        return true;
      }
      return false;
    } else if (lb1 < ub1 && lb2 > ub2) {
      // second interval wraps around
      double tmpNewLb1, tmpNewUb1, tmpNewLb2, tmpNewUb2;
      bool overlap1 = circular_overlap_and_merge(lb1, ub1, 0, ub2, cycle, tmpNewLb1, tmpNewUb1);
      bool overlap2 = circular_overlap_and_merge(lb1, ub1, lb2, cycle, cycle, tmpNewLb2, tmpNewUb2);
      if (overlap1 && !overlap2) {
        newLb = lb2;
        newUb = tmpNewUb1;
        return true;
      }
      if (overlap2 && !overlap1) {
        newLb = tmpNewLb2;
        newUb = ub2;
        return true;
      }
      if (overlap1 && overlap2) {
        newLb = 0;
        newUb = cycle;
        return true;
      }
      return false;
    } else if (lb2 < ub2 && lb1 > ub1) {
      // first interval wraps around
      double tmpNewLb1, tmpNewUb1, tmpNewLb2, tmpNewUb2;
      bool overlap1 = circular_overlap_and_merge(0, ub1, lb2, ub2, cycle, tmpNewLb1, tmpNewUb1);
      bool overlap2 = circular_overlap_and_merge(lb1, cycle, lb2, ub2, cycle, tmpNewLb2, tmpNewUb2);
      if (overlap1 && !overlap2) {
        newLb = lb1;
        newUb = tmpNewUb1;
        return true;
      }
      if (overlap2 && !overlap1) {
        newLb = tmpNewLb2;
        newUb = ub1;
        return true;
      }
      if (overlap1 && overlap2) {
        newLb = 0;
        newUb = cycle;
        return true;
      }
      return false;
    } else if (lb1 > ub1 && lb2 > ub2) {
      // both intervals wraps around
      newLb = std::min(lb1, lb2);
      newUb = std::max(ub1, ub2);
      if (newLb <= newUb) {
        newLb = 0;
        newUb = cycle;
      }
      return true;
    }

    return false;
  }
}

int main() {
  double T = 10.0;
  double newLb, newUb;
  std::cout << std::boolalpha;
  if (circular_overlap_and_merge(9, 10, 5.1, 9.1, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [5.1, 9.1] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [5.1, 9.1] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(7, 10, 1, 9.5, T, newLb, newUb)) {
    std::cout << "Intervals are [7, 10] and [1, 9.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [7, 10] and [1, 9.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0, 10, 1, 2, T, newLb, newUb)) {
    std::cout << "Intervals are [0, 10] and [1, 2] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0, 10] and [1, 2] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(1, 3, 0, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [1, 3] and [0, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [1, 3] and [0, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 10, 0, 1, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [0, 1] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [0, 1] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0, 1, 9, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [0, 1] and [9, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0, 1] and [9, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 10, 0, 9, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [0, 9] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [0, 9] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0, 9, 9, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [0, 9] and [9, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0, 9] and [9, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 10, 9.5, 9, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [9.5, 9] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [9.5, 9] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9.5, 9, 9, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [9.5, 9] and [9, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9.5, 9] and [9, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(8, 10, 0, 1, T, newLb, newUb)) {
    std::cout << "Intervals are [8, 10] and [0, 1] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [8, 10] and [0, 1] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0, 1, 8, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [0, 1] and [8, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0, 1] and [8, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(5.5, 10, 0, 6.5, T, newLb, newUb)) {
    std::cout << "Intervals are [5.5, 10] and [0, 6.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [5.5, 10] and [0, 6.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 10, 9.5, 1.5, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [9.5, 1.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [9.5, 1.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9.5, 1.5, 9, 10, T, newLb, newUb)) {
    std::cout << "Intervals are [9.5, 1.5] and [9, 10] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9.5, 1.5] and [9, 10] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 10, 9.5, 9, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 10] and [9.5, 9] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 10] and [9.5, 9] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(1, 5, 4, 8, T, newLb, newUb)) {
    std::cout << "Intervals are [1, 5] and [4, 8] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [1, 5] and [4, 8] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(4, 8, 1, 5, T, newLb, newUb)) {
    std::cout << "Intervals are [4, 8] and [1, 5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [4, 8] and [1, 5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0.1, 2, 9, 0.5, T, newLb, newUb)) {
    std::cout << "Intervals are [0.1, 2] and [9, 0.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0.1, 2] and [9, 0.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(7, 9.5, 9, 0.5, T, newLb, newUb)) {
    std::cout << "Intervals are [7, 9.5] and [9, 0.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [7, 9.5] and [9, 0.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0.1, 9.5, 9, 0.5, T, newLb, newUb)) {
    std::cout << "Intervals are [0.1, 9.5] and [9, 0.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0.1, 9.5] and [9, 0.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9, 0.5, 0.1, 2, T, newLb, newUb)) {
    std::cout << "Intervals are [9, 0.5] and [0.1, 2] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9, 0.5] and [0.1, 2] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(7, 0.5, 1, 9.5, T, newLb, newUb)) {
    std::cout << "Intervals are [7, 0.5] and [1, 9.5] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [7, 0.5] and [1, 9.5] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(8, 1, 9.8, 1.1, T, newLb, newUb)) {
    std::cout << "Intervals are [8, 1] and [9.8, 1.1] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [8, 1] and [9.8, 1.1] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(9.8, 1.1, 8, 1, T, newLb, newUb)) {
    std::cout << "Intervals are [9.8, 1.1] and [8, 1] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [9.8, 1.1] and [8, 1] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(8, 2, 0.1, 1.9, T, newLb, newUb)) {
    std::cout << "Intervals are [8, 2] and [0.1, 1.9] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [8, 2] and [0.1, 1.9] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(0.1, 1.9, 8, 2, T, newLb, newUb)) {
    std::cout << "Intervals are [0.1, 1.9] and [8, 2] with T=10"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [0.1, 1.9] and [8, 2] with T=10" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(1.22, 0.17, 2.79, 2.96, 3.14, newLb, newUb)) {
    std::cout << "Intervals are [1.22, 0.17] and [2.79, 2.96] with T=3.14"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [1.22, 0.17] and [2.79, 2.96] with T=3.14" << " and they do not overlap" << std::endl;
  }
  if (circular_overlap_and_merge(2.79, 2.96, 1.22, 0.17, 3.14, newLb, newUb)) {
    std::cout << "Intervals are [2.79, 2.96] and [1.22, 0.17] with T=3.14"
              << " and they overlap. Merged Interval: [" << newLb << ", " << newUb << "]" << std::endl;
  } else {
    std::cout << "Intervals are [2.79, 2.96] and [1.22, 0.17] with T=3.14" << " and they do not overlap" << std::endl;
  }
  return 0;
}
