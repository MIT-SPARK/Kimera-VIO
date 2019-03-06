#pragma once

#include <cstdint>
#include <memory>

namespace VIO {
  using Timestamp = std::int64_t;

  // Add compatibility for c++11's lack of make_unique.
  template<typename T, typename ...Args>
  std::unique_ptr<T> make_unique(Args&& ...args ) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
}
