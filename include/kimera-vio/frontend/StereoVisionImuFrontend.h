/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend.h
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <opencv2/highgui/highgui_c.h>

#include <atomic>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

using StereoFrontendInputPayload = StereoImuSyncPacket;

class StereoVisionImuFrontend : public VisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoVisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  StereoVisionImuFrontend(
      const FrontendParams& params,
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const StereoCamera::ConstPtr& stereo_camera,
      DisplayQueue* display_queue = nullptr,
      bool log_output = false,
      std::optional<OdometryParams> odom_params = std::nullopt);

  virtual ~StereoVisionImuFrontend();

 public:
  /* ------------------------------------------------------------------------ */
  // Returns extracted left and right rectified features in a suitable format
  // for VIO.
  void getSmartStereoMeasurements(
      const StereoFrame::Ptr& stereoFrame_kf,
      StereoMeasurements* smart_stereo_measurements) const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const;

 private:
  /* ------------------------------------------------------------------------ */
  // Frontend initialization.
  void processFirstStereoFrame(const StereoFrame& firstFrame);

  /**
   * @brief bootstrapSpin SpinOnce used when initializing the Frontend.
   * @param input
   * @return
   */
  inline FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Bootstrap);
    CHECK(input);
    return bootstrapSpinStereo(
        castUnique<StereoFrontendInputPayload>(std::move(input)));
  }

  /**
   * @brief nominalSpin SpinOnce used when in nominal mode after initialization
   * (bootstrap)
   * @param input
   * @return
   */
  inline FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Nominal ||
          frontend_state_ == FrontendState::InitialTimeAlignment);
    CHECK(input);
    return nominalSpinStereo(
        castUnique<StereoFrontendInputPayload>(std::move(input)));
  }

  /* ------------------------------------------------------------------------ */
  // Used when initializing the Frontend, operates on Stereo Frontend-specific
  // structures.
  StereoFrontendOutput::UniquePtr bootstrapSpinStereo(
      StereoFrontendInputPayload::UniquePtr&& input);

  /* ------------------------------------------------------------------------ */
  // Used when in nominal mode after init, operates on Stereo Frontend-specific
  // structures.
  StereoFrontendOutput::UniquePtr nominalSpinStereo(
      StereoFrontendInputPayload::UniquePtr&& input);

  /* ------------------------------------------------------------------------ */
  // Frontend main function.
  StatusStereoMeasurementsPtr processStereoFrame(
      const StereoFrame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  /* ------------------------------------------------------------------------ */
  // Static function to display output of stereo tracker
  static void printStatusStereoMeasurements(
      const StatusStereoMeasurements& statusStereoMeasurements);

  /* ------------------------------------------------------------------------ */
  // Log, visualize and/or save the feature tracks on the current left frame
  void sendFeatureTracksToLogger() const;

  cv::Mat displayFeatureTracks() const;

  // Log, visualize and/or save quality of temporal and stereo matching
  void sendStereoMatchesToLogger() const;

  /* ------------------------------------------------------------------------ */
  // Log, visualize and/or save quality of temporal and stereo matching
  void sendMonoTrackingToLogger() const;

 private:
  // TODO MAKE THESE GUYS std::unique_ptr, we do not want to have multiple
  // owners, instead they should be passed around.
  // Stereo Frames
  // Current frame
  StereoFrame::Ptr stereoFrame_k_;
  // Last frame
  StereoFrame::Ptr stereoFrame_km1_;
  // Last keyframe
  StereoFrame::Ptr stereoFrame_lkf_;

  // Rotation from last keyframe to reference frame
  // We use this to calculate the rotation btw reference frame and current frame
  // Whenever a keyframe is created, we reset it to identity.
  gtsam::Rot3 keyframe_R_ref_frame_;

  // Create the feature detector
  FeatureDetector::UniquePtr feature_detector_;

  // A stereo camera
  StereoCamera::ConstPtr stereo_camera_;

  // Set of functionalities for stereo matching
  StereoMatcher stereo_matcher_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like
  std::string output_images_path_;
};

}  // namespace VIO
