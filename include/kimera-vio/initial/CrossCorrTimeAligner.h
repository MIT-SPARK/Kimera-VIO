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
#include <gtsam/navigation/PreintegratedRotation.h>

#include <Eigen/Dense>
#include <cstddef>
#include <iterator>
#include <vector>

#include "kimera-vio/initial/RingBuffer.h"
#include "kimera-vio/initial/TimeAlignerBase.h"

namespace VIO {

/**
 * @brief Class to estimate the time delay between the IMU and the camera via
 * cross-correlation between relative rotation angles from the camera and IMU.
 *
 * We follow a similar approach to:
 *   Mair, Elmar, et al. "Spatio-temporal initialization for IMU to camera
 *   registration." 2011 IEEE International Conference on Robotics and
 *   Biomimetics. IEEE, 2011.
 *   https://doi.org/10.1109/ROBIO.2011.6181345
 */
class CrossCorrTimeAligner : public TimeAlignerBase {
 public:
  struct Measurement {
    Measurement() {}

    Measurement(const Timestamp& timestamp, const double& value)
        : timestamp(timestamp), value(value) {}

    friend std::ostream& operator<<(std::ostream& out, const Measurement& m) {
      out << "measurement<(t=" << m.timestamp << ", v=" << m.value << ")>";
      return out;
    }

    Timestamp timestamp{0};
    double value = 0.0;
  };
  typedef RingBuffer<Measurement> Buffer;

  CrossCorrTimeAligner(const ImuParams& params);

  /**
   * @brief get max of a vector starting at index N and iterating outwards
   */
  static size_t getMaxFromN(const std::vector<double>& values, size_t N);

 protected:
  /**
   * @brief Attempt estimation of time delay with all cached data
   *
   * This method either interpolates the estimated relative rotation angle
   * from the camera to IMU rate or does rotation-only preintegration to
   * estimate the relative rotation angle from the IMU at camera rate, and adds
   * all measurements (at either camera or IMU rate) to a circular buffer. Once
   * the buffer is full and the IMU buffer displays enough variance, this
   * computes the time delay using the peak cross-correlation between the two
   * signals.
   */
  TimeAlignerBase::Result attemptEstimation(
      const std::vector<Timestamp>& image_stamps,
      const gtsam::Pose3& T_ref_cur,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_acc_gyrs,
      FrontendLogger* logger = nullptr) override;

  /**
   * @brief Allocate buffers once we have access to both the camera and IMU
   * rates
   */
  void doFirstFrameSetup(const Frame& frame) override;

 private:
  size_t addNewImuDataImuRate(const std::vector<Timestamp>& image_stamps,
                              const ImuStampS& imu_stamps,
                              const ImuAccGyrS& imu_acc_gyrs);

  size_t addNewImuDataFrameRate(const std::vector<Timestamp>& image_stamps,
                                const ImuStampS& imu_stamps,
                                const ImuAccGyrS& imu_acc_gyrs);

  size_t addNewImuData(const std::vector<Timestamp>& image_stamps,
                       const ImuStampS& imu_stamps,
                       const ImuAccGyrS& imu_acc_gyrs);

  void interpNewImageMeasurements(
      const std::vector<Timestamp>& image_timestamps,
      const gtsam::Pose3& T_ref_cur,
      const size_t& num_new_imu_measurements);

  double getTimeShift() const;

  void logData(FrontendLogger* logger,
               const size_t& num_imu_added,
               const bool& not_enough_data,
               const bool& not_enough_variance,
               const double& result);

  bool do_imu_rate_estimation_;
  double imu_period_s_;
  double imu_variance_threshold_;
  double window_size_s_;
  std::unique_ptr<Buffer> imu_buffer_;
  std::unique_ptr<Buffer> vision_buffer_;
};

}  // namespace VIO
