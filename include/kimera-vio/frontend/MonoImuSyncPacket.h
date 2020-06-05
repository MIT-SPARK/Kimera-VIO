/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Monocular Frame with Imu data synchronized from last
 * Keyframe timestamp to the current frame timestamp.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MonoImuSyncPacket : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MonoImuSyncPacket(Frame::ConstUniquePtr frame,
                    const ImuStampS& imu_stamps,
                    const ImuAccGyrS& imu_accgyr);
  virtual ~MonoImuSyncPacket() = default;

 public:
  void print() const;

 public:
  Frame::UniquePtr frame_;
  const ImuStampS imu_stamps_;
  const ImuAccGyrS imu_accgyr_;
};

}  // namespace VIO
