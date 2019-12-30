/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayModule.cpp
 * @brief  Pipeline module to render/display 2D/3D data.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/DisplayModule.h"

#include <glog/logging.h>

namespace VIO {

DisplayModule::DisplayModule(InputQueue* input_queue,
                             OutputQueue* output_queue,
                             bool parallel_run,
                             DisplayBase::UniquePtr&& display)
    : SISO(input_queue, output_queue, "Display", parallel_run),
      display_(std::move(display)) {}

DisplayModule::OutputUniquePtr DisplayModule::spinOnce(
    VisualizerOutput::UniquePtr input) {
  CHECK(input);
  display_->spinOnce(std::move(input));
  return VIO::make_unique<NullPipelinePayload>();
}

}  // namespace VIO
