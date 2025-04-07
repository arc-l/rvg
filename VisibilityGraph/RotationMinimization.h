#ifndef ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_ROTATIONMINIMIZATION_H_
#define ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_ROTATIONMINIMIZATION_H_
#include <cassert>
#include <iostream>
#include <cmath>
#ifdef USE_IPOPT
#include <Ipopt-build/include/coin/IpTNLP.hpp>
#include <Ipopt-build/include/coin/IpIpoptApplication.hpp>

namespace RotationalVisibilityGraph {
// Define the class to handle the optimization problem
template<typename T>
class RotationMinimization : public Ipopt::TNLP {
 private:
  std::vector<std::pair<T, T>> theta_ranges; // Rotation constraints for each vertex
  std::vector<T> _solution;

 public:
  // Constructor with initialization of ranges
  RotationMinimization(const std::vector<std::pair<T, T>> &ranges)
      : theta_ranges(ranges) {}

  const std::vector<T> &getSolution() const {
    return _solution;
  }

  // Method to return the size of the problem
  bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                    Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style) override {
    n = theta_ranges.size();    // Number of variables
    m = 0;                      // Number of constraints
    nnz_jac_g = 0;              // Number of nonzeros in the Jacobian of the constraints
    nnz_h_lag = n;              // Number of nonzeros in the Hessian of the Lagrangian
    index_style = TNLP::C_STYLE; // use the C style indexing (0-based)
    return true;
  }

  // Method to set the initial point
  bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                          bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                          Ipopt::Index m, bool init_lambda,
                          Ipopt::Number *lambda) override {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    // Initialize the variables to the middle of their ranges
    for (Ipopt::Index i = 0; i < n; i++) {
      x[i] = (theta_ranges[i].first + theta_ranges[i].second) / 2.0;
    }
    return true;
  }

  // Method to return the bounds for my problem
  bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                       Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u) override {
    for (Ipopt::Index i = 0; i < n; i++) {
      x_l[i] = theta_ranges[i].first;
      x_u[i] = theta_ranges[i].second;
    }
    return true;
  }

  // Return the objective value
  bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value) override {
    obj_value = 0;
    for (Ipopt::Index i = 1; i < n; i++) {
      obj_value += std::abs(x[i] - x[i - 1]);
    }
    return true;
  }

  // Return the gradient of the objective
  bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f) override {
    for (Ipopt::Index i = 0; i < n; i++) {
      grad_f[i] = 0; // Initialize with zero
    }
    for (Ipopt::Index i = 1; i < n; i++) {
      grad_f[i - 1] += (x[i - 1] > x[i]) ? 1 : -1;
      grad_f[i] += (x[i] > x[i - 1]) ? 1 : -1;
    }
    return true;
  }

  // Method to return the Hessian (only the lower left corner since it is symmetric)
  bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor,
              Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda, Ipopt::Index nele_hess,
              Ipopt::Index iRow[], Ipopt::Index jCol[], Ipopt::Number *values) override {
    // Since the objective is not smooth, we do not provide exact Hessian (Ipopt will approximate it)
    return false;
  }

  void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
                         const Ipopt::Number *z_L, const Ipopt::Number *z_U, Ipopt::Index m,
                         const Ipopt::Number *g, const Ipopt::Number *lambda,
                         Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                         Ipopt::IpoptCalculatedQuantities *ip_cq) override {
    std::cout << "Solution status: " << status << std::endl;
    std::cout << "Objective value: " << obj_value << std::endl;
    _solution.resize(n);
    for (Ipopt::Index i = 0; i < n; i++) {
      std::cout << "Theta_" << i << ": " << x[i] << std::endl;
      _solution[i] = x[i];
    }
  }
  // In RotationMinimization class:

// Method to return the evaluation of the constraints g(x)
  bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g) override {
    // Since there are no constraints other than bounds, do nothing
    return true;
  }

// Method to return the Jacobian structure or values of the constraints g(x)
  bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                  Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol,
                  Ipopt::Number *values) override {
    // Since there are no constraints other than bounds, do nothing
    if (values == nullptr) {
      // If asked for the structure, do nothing
    } else {
      // If asked for the values, do nothing
    }
    return true;
  }

};
} // namespace RotationalVisibilityGraph
#endif //USE_IPOPT

#endif //ROTATIONALVISIBILITYGRAPH_VISIBILITYGRAPH_ROTATIONMINIMIZATION_H_
