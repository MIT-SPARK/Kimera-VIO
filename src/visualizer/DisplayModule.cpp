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

DisplayModule::DisplayModule(DisplayQueue* input_queue,
                             OutputQueue* output_queue,
                             bool parallel_run,
                             DisplayBase::UniquePtr&& display)
    : SISO(input_queue, output_queue, "Display", parallel_run),
      display_(std::move(display)) {}

DisplayModule::OutputUniquePtr DisplayModule::spinOnce(InputUniquePtr input) {
  CHECK(input);
  if (display_) display_->spinOnce(std::move(input));
  return std::make_unique<NullPipelinePayload>();
}

typename DisplayModule::MISO::InputUniquePtr DisplayModule::getInputPacket() {
  if (display_ && display_->display_type_ == DisplayType::kPangolin) {
    // If we are using pangolin just fake a constant input of messages
    // to not block the visualizer.
    return std::make_unique<DisplayInputBase>();
  }

  typename MISO::InputUniquePtr input = nullptr;
  bool queue_state = false;

  if (MISO::parallel_run_) {
    queue_state = input_queue_->popBlockingWithTimeout(input, 5);
  } else {
    queue_state = input_queue_->pop(input);
  }

  if (queue_state) {
    return input;
  } else {
    VLOG(10) << "Module: " << MISO::name_id_ << " - "
             << "Input queue: " << input_queue_->queue_id_
             << " didn't return an output.";
    return nullptr;
  }
}

}  // namespace VIO
