/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrontendOutputPacketBase.h
 * @brief  Class describing the minimum output of the frontend.
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/common/vio_types.h"
// #include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

class FrontendOutputPacketBase : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendOutputPacketBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FrontendOutputPacketBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontendOutputPacketBase(const Timestamp& timestamp,
                           const bool is_keyframe,
                           const ImuFrontEnd::PimPtr& pim,
                           const ImuAccGyrS& imu_acc_gyrs)
      : PipelinePayload(timestamp),
        is_keyframe_(is_keyframe),
        pim_(pim),
        imu_acc_gyrs_(imu_acc_gyrs) {}

  virtual ~FrontendOutputPacketBase() = default;

 public:
  const bool is_keyframe_;
  const ImuFrontEnd::PimPtr pim_;
  const ImuAccGyrS imu_acc_gyrs_;
};

}  // namespace VIO
