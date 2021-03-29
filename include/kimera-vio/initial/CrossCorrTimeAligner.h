/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   crossCorrTimeAligner.h
 * @brief  Class to estimate IMU to camera time offset by cross-correlation
 * @author Nathan Hughes
 */

#pragma once
#include <Eigen/Dense>

#include "kimera-vio/initial/TimeAlignerBase.h"

namespace VIO {
class CrossCorrTimeAligner : public TimeAlignerBase {
 public:
  // TODO(nathan) add other parameters here
  CrossCorrTimeAligner(double imu_time_shift_est = 0.0,
                       bool should_estimate = false,
                       size_t window_size = 100);


  void addNewImuData(const ImuStampS& imu_stamps_, const ImuAccGyrS& imu_acc_gyrs) override;

 protected:
  TimeAlignerBase::Result attemptEstimation(const FrontendOutputPacketBase& input) override;

 private:
  size_t window_size_;
  size_t num_measurements_;
  size_t curr_index_;
  Eigen::VectorXd vision_rotation_angles_;
  Eigen::VectorXd pim_rotation_angles_;
};
}  // namespace VIO
