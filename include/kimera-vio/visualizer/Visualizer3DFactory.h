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

#include "kimera-vio/backend/VioBackEnd-definitions.h"
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

 public:
  template <typename... Ts>
  static Visualizer3D::UniquePtr createVisualizer(
      const VisualizerType& visualizer_type,
      Ts&&... args) {
    switch (visualizer_type) {
      case VisualizerType::OpenCV: {
        return VIO::make_unique<Visualizer3D>(std::forward<Ts>(args)...);
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
};

}  // namespace VIO
