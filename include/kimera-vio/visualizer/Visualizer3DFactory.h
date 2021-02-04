/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3DFactory.h
 * @brief  Factory class for 3D Visualizers
 * @author Antoni Rosinol
 */

#pragma once

#include <glog/logging.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

namespace VIO {

class VisualizerFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerFactory);
  VisualizerFactory() = delete;
  virtual ~VisualizerFactory() = default;

  static Visualizer3D::UniquePtr createVisualizer(
      const VisualizerType visualizer_type,
      const VisualizationType& viz_type,
      const BackendType& backend_type);
};

}  // namespace VIO
