/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Display.cpp
 * @brief  Class to display visualizer output
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Display.h"

#include "kimera-vio/visualizer/DisplayParams.h"

namespace VIO {

DisplayBase::DisplayBase(const DisplayType& display_type)
    : display_type_(display_type) {}

}  // namespace VIO
