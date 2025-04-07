#ifndef ROTATIONALVISIBILITYGRAPH_UTILS_TIMER_H_
#define ROTATIONALVISIBILITYGRAPH_UTILS_TIMER_H_

#include <chrono>
#include <Utils/Utils.h>

namespace RotationalVisibilityGraph::Utils {
class Timer {
 public:
  Timer(bool verbose) : m_start(std::chrono::high_resolution_clock::now()) {
    m_identifier = "";
    m_verbose = verbose;
  }
  Timer(std::string identifier, bool verbose)
      : m_start(std::chrono::high_resolution_clock::now()), m_identifier(std::move(identifier)), m_verbose(verbose) {
  }

  void reset() {
    m_start = std::chrono::high_resolution_clock::now();
  }

  [[nodiscard]] double elapsed() const {
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - m_start).count();
  }

  void stop() {
    if (m_verbose)
      print(m_identifier, "Time elapsed: ", elapsed(), "s");
  }

  ~Timer() {
    if (m_verbose)
      print(m_identifier, "Time elapsed: ", elapsed(), "s");
  }

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
  std::string m_identifier;
  bool m_verbose;
};
} // namespace RotationalVisibilityGraph::Utils

#endif
