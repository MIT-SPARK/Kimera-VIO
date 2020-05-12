/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer.cpp
 * @brief  Build and visualize 3D data: 2D mesh from Frame for example.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Visualizer3D.h"

namespace VIO {

Visualizer3D::Visualizer3D(const VisualizationType& viz_type)
    : visualization_type_(viz_type) {}

}  // namespace VIO
