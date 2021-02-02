/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrontendInputPacketBase.h
 * @brief  Class describing the minimum input for VIO to run in abstract form.
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

class FrontendInputPacketBase : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendInputPacketBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FrontendInputPacketBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontendInputPacketBase() = delete;

  FrontendInputPacketBase(const Timestamp& timestamp,
                          const ImuStampS& imu_stamps,
                          const ImuAccGyrS& imu_accgyrs)
      : PipelinePayload(timestamp),
        imu_stamps_(imu_stamps),
        imu_accgyrs_(imu_accgyrs) {
    CHECK_GT(imu_stamps_.cols(), 0u);
    CHECK_GT(imu_accgyrs_.cols(), 0u);
    CHECK_EQ(imu_stamps_.cols(), imu_accgyrs_.cols());
  }

  virtual ~FrontendInputPacketBase() = default;

  const ImuStampS imu_stamps_;
  const ImuAccGyrS imu_accgyrs_;
};

}  // namespace VIO
