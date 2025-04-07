#ifdef USE_IPOPT
#include <VisibilityGraph/RotationMinimization.h>

using namespace RotationalVisibilityGraph;
int main() {
  std::vector<std::pair<double, double>> theta_ranges = {{0, M_PI/2}, {-M_PI/4, M_PI/4}, {0, M_PI/3}, {-M_PI/3, M_PI/2}, {M_PI/4, 3*M_PI/4}};
  Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new RotationMinimization(theta_ranges);
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-9);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << "Error during initialization!" << std::endl;
    return (int)status;
  }
  status = app->OptimizeTNLP(mynlp);
  return (int)status;
}
#else
int main() {
  return 0;
}
#endif //USE_IPOPT