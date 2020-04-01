#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {
PipelineParams::PipelineParams(const std::string& name) : name_(name) {
  CHECK_EQ(kTotalWidth, kNameWidth + kValueWidth)
      << "Make sure these are consistent for pretty printing.";
}

}  // namespace VIO
