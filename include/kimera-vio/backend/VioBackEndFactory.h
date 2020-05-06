/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEndModule.h
 * @brief  Pipeline module for the backend.
 *
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/RegularVioBackEnd.h"
#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEnd.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class BackEndFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(BackEndFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(BackEndFactory);
  BackEndFactory() = delete;
  virtual ~BackEndFactory() = default;

 public:
  template <typename... Ts>
  static VioBackEnd::UniquePtr createBackend(const BackendType& backend_type,
                                             Ts&&... args) {
    switch (backend_type) {
      case BackendType::kStereoImu: {
        return VIO::make_unique<VioBackEnd>(std::forward<Ts>(args)...);
      } break;
      case BackendType::kStructuralRegularities: {
        return VIO::make_unique<RegularVioBackEnd>(std::forward<Ts>(args)...);
      } break;
      default: {
        LOG(FATAL) << "Requested backend type is not supported.\n"
                   << "Currently supported backend types:\n"
                   << "0: normal VIO\n 1: regular VIO\n"
                   << " but requested backend: "
                   << static_cast<int>(backend_type);
      }
    }
  }
};

}  // namespace VIO
