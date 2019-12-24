/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MesherFactory.h
 * @brief  Build a Mesher
 * @author Antoni Rosinol
 */

#include "kimera-vio/mesh/MesherFactory.h"

namespace VIO {

Mesher::UniquePtr MesherFactory::createMesher(
    const MesherType& mesher_type,
    const MesherParams& mesher_params) {
  switch (mesher_type) {
    case MesherType::PROJECTIVE: {
      return VIO::make_unique<Mesher>(mesher_params);
    }
    default: {
      LOG(FATAL) << "Requested mesher type is not supported.\n"
                 << static_cast<int>(mesher_type);
    }
  }
}

}  // namespace VIO
