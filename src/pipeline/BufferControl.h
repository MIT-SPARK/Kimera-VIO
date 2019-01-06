#pragma once

#include <atomic>

namespace VIO {
  class BufferControl {
    public:
    BufferControl() = default;

    bool shutdownAll() {
      shutdown_ = true;
      return true;
    }

    private:
      std::atomic_bool shutdown_ = {false};
  };
}
