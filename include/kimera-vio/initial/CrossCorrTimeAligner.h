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
#include <cstddef>
#include <iterator>
#include <vector>

#include <gtsam/navigation/PreintegratedRotation.h>
#include "kimera-vio/initial/RingBuffer.h"
#include "kimera-vio/initial/TimeAlignerBase.h"

namespace VIO {

typedef boost::shared_ptr<gtsam::PreintegratedRotationParams>
    RotOnlyPIMParamPtr;

class CrossCorrTimeAligner : public TimeAlignerBase {
 public:
  struct Measurement {
    Measurement() {}

    Measurement(Timestamp timestamp, double value)
        : timestamp(timestamp), value(value) {}

    friend std::ostream& operator<<(std::ostream& out, const Measurement& m) {
      out << "measurement<(t=" << m.timestamp << ", v=" << m.value << ")>";
      return out;
    }

    Timestamp timestamp{0};
    double value{0.0};
  };
  typedef RingBuffer<Measurement> Buffer;

  // TODO(nathan) add other parameters here
  CrossCorrTimeAligner(bool do_imu_rate_estimation,
                       double imu_period_s,
                       size_t window_size = 100);

 protected:
  TimeAlignerBase::Result attemptEstimation(
      const std::pair<Timestamp, Timestamp>& timestamps_ref_cur,
      const gtsam::Pose3& T_ref_cur,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_acc_gyrs) override;

 private:
  bool add_new_imu_data_(Timestamp frame_timestamp,
                         const ImuStampS& imu_stamps,
                         const ImuAccGyrS& imu_acc_gyrs);

  RotOnlyPIMParamPtr pim_params_;
  bool do_imu_rate_estimation_;
  double imu_period_s_;
  Buffer imu_buffer_;
  Buffer vision_buffer_;
};

}  // namespace VIO
