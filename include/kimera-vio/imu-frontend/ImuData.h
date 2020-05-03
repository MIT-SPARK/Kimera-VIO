/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuData.h
 * @brief  Container for IMU data
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"

namespace VIO {

/**
 * @brief The ImuData class Contains inertial measurements in a temporal buffer
 * that is threadsafe.
 */
class ImuData {
 public:
  KIMERA_POINTER_TYPEDEFS(ImuData);
  KIMERA_DELETE_COPY_CONSTRUCTORS(ImuData);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Imu buffer with virtually infinite memory.
  ImuData() : imu_buffer_(-1) {}
  ~ImuData() = default;

 public:
  void print() const;

 public:
  //! Imu data.
  utils::ThreadsafeImuBuffer imu_buffer_;
};

}  // namespace VIO
