/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionImuFrontend.h
 * @brief  Class describing a monocular tracking Frontend
 * @author Marcus Abate
 */

#pragma once

#include <memory>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class MonoVisionImuFrontend : public VisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoVisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoVisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  MonoVisionImuFrontend(const ImuParams& imu_params,
                     const ImuBias& imu_initial_bias,
                     const MonoFrontendParams& frontend_params,
                     const Camera::ConstPtr& camera,
                     DisplayQueue* display_queue = nullptr,
                     bool log_output = false);
  virtual ~MonoVisionImuFrontend();

 private:
  void processFirstFrame(const Frame& firstFrame);

  inline FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Bootstrap);
    CHECK(input);
    return bootstrapSpinMono(
        VIO::safeCast<FrontendInputPacketBase, MonoFrontendInputPayload>(
            std::move(input)));
  }

  inline FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    CHECK(frontend_state_ == FrontendState::Nominal);
    CHECK(input);
    return nominalSpinMono(
        VIO::safeCast<FrontendInputPacketBase, MonoFrontendInputPayload>(
            std::move(input)));
  }

  MonoFrontendOutput::UniquePtr nominalSpinMono(
      MonoFrontendInputPayload::UniquePtr&& input);

  MonoFrontendOutput::UniquePtr bootstrapSpinMono(
      MonoFrontendInputPayload::UniquePtr&& input);

  StatusMonoMeasurementsPtr processFrame(
      const Frame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  void getSmartMonoMeasurements(const Frame::Ptr& frame,
                                MonoMeasurements* smart_mono_measurements);

  // void sendFeatureTracksToLogger() const;

  // void sendMonoTrackingToLogger() const;

  static void printStatusMonoMeasurements(
      const StatusMonoMeasurements& status_mono_measurements);

 private:
  // Current frame
  Frame::Ptr mono_frame_k_;
  // Last frame
  Frame::Ptr mono_frame_km1_;
  // Last keyframe
  Frame::Ptr mono_frame_lkf_;

  gtsam::Rot3 keyframe_R_ref_frame_;

  FeatureDetector::UniquePtr feature_detector_;

  Camera::ConstPtr mono_camera_;

  MonoFrontendParams frontend_params_;
};

}  // namespace VIO
