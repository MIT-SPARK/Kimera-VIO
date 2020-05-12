/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3D.h
 * @brief  Build and visualize 3D data: 2D mesh from frame for example.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class Visualizer3D {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(Visualizer3D);
  KIMERA_POINTER_TYPEDEFS(Visualizer3D);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Visualizer3D base constructor
   * @param viz_type: type of 3D visualization
   */
  Visualizer3D(const VisualizationType& viz_type);
  virtual ~Visualizer3D() = default;

 public:
  virtual VisualizerOutput::UniquePtr spinOnce(
      const VisualizerInput& input) = 0;

 public:
  VisualizationType visualization_type_;
};

}  // namespace VIO
