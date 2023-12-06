/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3DFactory.cpp
 * @brief  Factory for 3D Visualizers.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Visualizer3DFactory.h"
#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"

namespace VIO {

Visualizer3D::UniquePtr VisualizerFactory::createVisualizer(
    const VisualizerType visualizer_type,
    const VisualizationType& viz_type,
    const BackendType& backend_type) {
  switch (visualizer_type) {
    case VisualizerType::OpenCV: {
      return std::make_unique<OpenCvVisualizer3D>(viz_type, backend_type);
    }
    default: {
      LOG(FATAL) << "Requested visualizer type is not supported.\n"
                 << "Currently supported visualizer types:\n"
                 << "0: OpenCV 3D viz\n 1: Pangolin (not supported yet)\n"
                 << " but requested visualizer: "
                 << static_cast<int>(visualizer_type);
    }
  }
}

}  // namespace VIO
