#pragma once

#include <chrono>
#include <memory>

namespace VIO {
namespace utils {

class Timer {
 public:
  static std::chrono::high_resolution_clock::time_point tic() {
    return std::chrono::high_resolution_clock::now();
  }

  // Stop timer and report duration in given time.
  // Returns duration in milliseconds by default.
  // call .count() on returned duration to have number of ticks.
  template <typename T = std::chrono::milliseconds>
  static T toc(const std::chrono::high_resolution_clock::time_point& start) {
    return std::chrono::duration_cast<T>(
        std::chrono::high_resolution_clock::now() - start);
  }
};

// Usage: measure<>::execution(function, arguments)
template <typename T = std::chrono::milliseconds>
struct Measure {
  template <typename F, typename... Args>
  static typename T::rep execution(F&& func, Args&&... args) {
    auto start = std::chrono::steady_clock::now();
    std::forward<decltype(func)>(func)(std::forward<Args>(args)...);
    auto duration =
        std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - start);
    return duration.count();
  }
};

}  // namespace utils
}  // namespace VIO
