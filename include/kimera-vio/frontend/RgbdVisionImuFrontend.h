/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend.h
 * @brief  Class describing a rgbd tracker
 * @author Nathan Hughes
 */

#pragma once
#include <atomic>
#include <memory>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/RgbdImuSyncPacket.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"

namespace VIO {

class RgbdVisionImuFrontend : public VisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdVisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdVisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  RgbdVisionImuFrontend(
      const FrontendParams& frontend_params,
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const RgbdCamera::ConstPtr& camera,
      DisplayQueue* display_queue = nullptr,
      bool log_output = false,
      boost::optional<OdometryParams> odom_params = boost::none);

  virtual ~RgbdVisionImuFrontend();

 public:
  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const;

  // Convert frame to backend measurements
  void fillSmartStereoMeasurements(const StereoFrame& frame,
                                   StereoMeasurements* measurements) const;

 private:
  /**
   * @brief bootstrapSpin SpinOnce used when initializing the Frontend.
   * @param input
   * @return
   */
  FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override;

  /**
   * @brief nominalSpin SpinOnce used when in nominal mode after initialization
   * @param input
   * @return
   */
  FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override;

  // Frontend initialization.
  void processFirstFrame(const RgbdFrame& frame);

  // high-level consistency checks and logging
  void checkAndLogKeyframe();

  // Frontend nominal processing
  StatusStereoMeasurementsPtr processFrame(
      const RgbdFrame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  // handle feature detection and outlier rejection for new keyframe
  void handleKeyframe(const RgbdFrame& rgbd_frame,
                      StereoFrame& frame,
                      const gtsam::Rot3& keyframe_R_frame);

  // display output of stereo tracker
  static void printStatusStereoMeasurements(
      const StatusStereoMeasurements& measurements);

  void logTrackingImages(const StereoFrame& frame, const RgbdFrame& rgbd_frame);

  void sendMonoTrackingToLogger(const StereoFrame& frame) const;

 private:
  Timestamp last_frame_timestamp_;
  //! Last frame
  StereoFrame::Ptr frame_km1_;
  //! Last keyframe
  StereoFrame::Ptr frame_lkf_;

  // Rotation from last keyframe to reference frame
  // We use this to calculate the rotation btw reference frame and current frame
  // Whenever a keyframe is created, we reset it to identity.
  gtsam::Rot3 keyframe_R_ref_frame_;

  //! feature detector
  FeatureDetector::UniquePtr feature_detector_;

  //! the camera for the frontend
  RgbdCamera::ConstPtr camera_;
};

}  // namespace VIO
