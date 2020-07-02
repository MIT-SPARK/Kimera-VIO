/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Display.h
 * @brief  Class to display visualizer output
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/DisplayParams.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayBase);

  DisplayBase(const DisplayType& display_type);
  virtual ~DisplayBase() = default;

 public:
  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_output Visualizer output, which is the display input.
   */
  virtual void spinOnce(DisplayInputBase::UniquePtr&& viz_output) = 0;

 public:
  DisplayType display_type_;
};

}  // namespace VIO
