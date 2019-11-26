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
  static Mesher::UniquePtr createMesher(const MesherType& mesher_type,
                                        const MesherParams& mesher_params);
};

}  // namespace VIO
