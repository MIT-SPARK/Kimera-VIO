/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayFactory.h
 * @brief  Display factory
 * @author Antoni Rosinol
 */

#pragma once

#include <glog/logging.h>

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayParams.h"
#include "kimera-vio/visualizer/OpenCvDisplay.h"
#ifdef Pangolin_FOUND
#include "kimera-vio/visualizer/PangolinDisplay.h"
#endif

namespace VIO {

class DisplayFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayFactory);

  DisplayFactory() = default;
  virtual ~DisplayFactory() = default;

  template <class... Types>
  static DisplayBase::UniquePtr makeDisplay(
      const DisplayType& display_type,
      Types... args) {
    switch (display_type) {
      case DisplayType::kPangolin: {
#ifdef Pangolin_FOUND
        return VIO::make_unique<PangolinDisplay>(args...);
#else
        // No pangolin library found...
        return nullptr;
#endif
      }
      case DisplayType::kOpenCV: {
        return VIO::make_unique<OpenCv3dDisplay>(args...);
      }
      default: {
        LOG(FATAL) << "Requested display type is not supported.\n"
                   << "Currently supported display types:\n"
                   << "0: OpenCV 3D viz\n 1: Pangolin (not supported yet)\n"
                   << " but requested display: "
                   << VIO::to_underlying(display_type);
      }
    }
  }
};

}  // namespace VIO
