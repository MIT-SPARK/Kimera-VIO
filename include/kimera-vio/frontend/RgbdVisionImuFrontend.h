/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend.h
 * @brief  Class describing a Rgbd tracking Frontend
 * @author Marcus Abate
 */

#pragma once

#include <memory>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/frontend/RgbdImuSyncPacket.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class RgbdVisionImuFrontend : public VisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdVisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdVisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  RgbdVisionImuFrontend(const ImuParams& imu_params,
                     const ImuBias& imu_initial_bias,
                     const FrontendParams& frontend_params,
                     const StereoCamera::ConstPtr& stereo_camera,
                     DisplayQueue* display_queue = nullptr,
                     bool log_output = false);
  virtual ~RgbdVisionImuFrontend();

  /* ------------------------------------------------------------------------ */
  // Converts the RGBD information into Mimicing stereo format and sends the mimiced
  // feature points on left and right camera n a suitable format for VIO.
  void getSmartRgbdMeasurements(const RgbdFrame::Ptr& frame,
                                RgbdMeasurements* smart_rgbd_measurements);

/* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  // Mimicing stereo setup
  gtsam::Pose3 getRelativePoseBodyRgbd() const;

 private:
  void processFirstFrame(const RgbdFrame& firstFrame);

  inline FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Bootstrap);
    CHECK(input);
    return bootstrapSpinRgbd(
        VIO::safeCast<FrontendInputPacketBase, RgbdFrontendInputPayload>(
            std::move(input)));
  }

  inline FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Nominal);
    CHECK(input);
    return nominalSpinRgbd(
        VIO::safeCast<FrontendInputPacketBase, RgbdFrontendInputPayload>(
            std::move(input)));
  }

  RgbdFrontendOutput::UniquePtr nominalSpinRgbd(
      RgbdFrontendInputPayload::UniquePtr&& input);

  RgbdFrontendOutput::UniquePtr bootstrapSpinRgbd(
      RgbdFrontendInputPayload::UniquePtr&& input);

  StatusRgbdMeasurementsPtr processFrame(
      const RgbdFrame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  // void sendFeatureTracksToLogger() const;

  // void sendRgbdTrackingToLogger() const;

  static void printStatusRgbdMeasurements(
      const StatusRgbdMeasurements& status_rgbd_measurements);

 private:
  // Current frame
  RgbdFrame::Ptr rgbd_frame_k_;
  // Last frame
  RgbdFrame::Ptr rgbd_frame_km1_;
  // Last keyframe
  RgbdFrame::Ptr rgbd_frame_lkf_;

  gtsam::Rot3 keyframe_R_ref_frame_;

  FeatureDetector::UniquePtr feature_detector_;

//   Camera::ConstPtr camera_;

  FrontendParams frontend_params_;
  // A stereo camera (Dummy to mimic stereo when needed)
  StereoCamera::ConstPtr stereo_camera_;
};

}  // namespace VIO
