/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayModule.cpp
 * @brief  Pipeline module to render/display 2D/3D visualization.
 * @author Antoni Rosinol
 */

#pragma once

#include <utility>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

/**
 * @brief The DisplayModule class receives a VisualizerOutput payload which
 * contains images to be displayed and uses a display to render it.
 * Since displaying must be done in the main thread, the
 * DisplayModule should spin in the main thread to avoid errors.
 */
class DisplayModule
    : public SISOPipelineModule<DisplayInputBase, NullPipelinePayload> {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayModule);

  using SISO = SISOPipelineModule<DisplayInputBase, NullPipelinePayload>;

  DisplayModule(DisplayQueue* input_queue,
                OutputQueue* output_queue,
                bool parallel_run,
                DisplayBase::UniquePtr&& display);

  virtual ~DisplayModule() = default;

  OutputUniquePtr spinOnce(InputUniquePtr input) override;

  typename MISO::InputUniquePtr getInputPacket() override;

 private:
  // The renderer used to display the visualizer output.
  DisplayBase::UniquePtr display_;
};

}  // namespace VIO
