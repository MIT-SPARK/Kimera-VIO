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

// If you want to serialize to a file to be able to later load that mesh.
DEFINE_bool(serialize_mesh_2d3d, false, "Serialize 2D/3D Mesh to file.");

namespace VIO {

Mesher::UniquePtr MesherFactory::createMesher(
    const MesherType& mesher_type,
    const MesherParams& mesher_params) {
  switch (mesher_type) {
    case MesherType::PROJECTIVE: {
      return VIO::make_unique<Mesher>(mesher_params, FLAGS_serialize_mesh_2d3d);
    }
    default: {
      LOG(FATAL) << "Requested mesher type is not supported.\n"
                 << static_cast<int>(mesher_type);
    }
  }
}

}  // namespace VIO
