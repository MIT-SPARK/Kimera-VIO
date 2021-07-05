/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   timeAlignerBase.h
 * @brief  Class to estimate IMU to camera time offset
 * @author Nathan Hughes
 */

#pragma once
#include <utility>
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/logging/Logger.h"

namespace VIO {

class TimeAlignerBase {
 public:
  typedef std::unique_ptr<TimeAlignerBase> UniquePtr;

  struct Result {
    bool valid;             //<! Whether or not the estimate is valid
    double imu_time_shift;  //<! Defined as t_imu = t_cam + imu_shift
  };

  TimeAlignerBase() = default;

  virtual ~TimeAlignerBase() = default;

  /**
   * @brief Attempt to estimate the temporal delay between the IMU and the
   * camera
   *
   * This function takes in a single frame (the frontend output) as well as the
   * un-integrated IMU measurements taken from the last frame to this one, and
   * computes the relative rotation from the the previous (chached) frame using
   * 5-pt RANSAC. If RANSAC succeeds, then the resulting IMU packet and relative
   * rotation estimate are sent on to the estimator. If RANSAC fails, this
   * returns a "valid" estimate of 0.0. In general, if this or the underlying
   * derived class encounter a state that make it impossible to estimate the
   * time alignment, they return a "valid" estimate of 0.0.
   *
   * @param tracker used exclusively to compute 5pt RANSAC
   * @param output latest frame from the frontend
   * @param imu_stamps timestamps for all IMU measurements between the last
   * frame and the newest
   * @param imuaccgyrs IMU measurements between the last frame and the newest.
   * @returns the estimate and whether it is valid
   */
  virtual Result estimateTimeAlignment(Tracker& tracker,
                                       const FrontendOutputPacketBase& output,
                                       const ImuStampS& imu_stamps,
                                       const ImuAccGyrS& imu_accgyrs,
                                       FrontendLogger* logger = nullptr);

  /**
   * @brief Perform setup in the derived class when we get the first frame
   */
  virtual void doFirstFrameSetup(const Frame& frame) = 0;

 protected:
  void mergeImuData(const ImuStampS& latest_stamps,
                    const ImuAccGyrS& latest_accgyrs,
                    ImuStampS* new_imu_stamps,
                    ImuAccGyrS* new_imu_values);

  virtual Result attemptEstimation(
      const std::vector<Timestamp>& image_timestamps,
      const gtsam::Pose3& T_ref_cur,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyrs,
      FrontendLogger* logger = nullptr) = 0;

  Frame::UniquePtr last_frame_;
  std::vector<Timestamp> image_stamp_cache_;
  std::vector<ImuStampS> imu_stamp_cache_;
  std::vector<ImuAccGyrS> imu_value_cache_;
};

}  // namespace VIO
