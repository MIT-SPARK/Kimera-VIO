#pragma once

#include <atomic>

namespace VIO {
class ProcessControl {
 public:
  ProcessControl() = default;

  bool shutdownAll() {
    shutdown_ = true;
    return true;
  }

 private:
  std::atomic_bool shutdown_ = {false};
};
}  // namespace VIO
