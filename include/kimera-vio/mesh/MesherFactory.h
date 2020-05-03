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

#pragma once

#include <glog/logging.h>

#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/mesh/Mesher.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MesherFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MesherFactory);
  MesherFactory() = delete;
  virtual ~MesherFactory() = default;

 public:
  template <typename... Ts>
  static Mesher::UniquePtr createMesher(
      const MesherType& mesher_type,
      Ts&&... args) {
    switch (mesher_type) {
      case MesherType::PROJECTIVE: {
        static constexpr bool kSerializeMesh2d3d = false;
        return VIO::make_unique<Mesher>(std::forward<Ts>(args)...,
                                        kSerializeMesh2d3d);
      }
      default: {
        LOG(FATAL) << "Requested mesher type is not supported.\n"
                   << static_cast<int>(mesher_type);
      }
    }
  }
};

}  // namespace VIO
