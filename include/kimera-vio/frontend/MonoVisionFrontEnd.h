/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionFrontEnd.h
 * @brief  Class describing a monocular tracking frontend
 * @author Marcus Abate
 */

#pragma once

#include <memory>

#include "kimera-vio/backend/VioBackEnd-definitions.h"

#include "kimera-vio/frontend/MonoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VisionFrontEnd.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/Tracker.h"

#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class MonoVisionFrontEnd : public VisionFrontEnd<MonoFrontEndInputPayload,
                                                 MonoFrontendOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoVisionFrontEnd);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoVisionFrontEnd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  MonoVisionFrontEnd(const ImuParams& imu_params,
                     const ImuBias& imu_initial_bias,
                     const MonoFrontendParams& frontend_params,
                     const Camera::Ptr& camera,
                     DisplayQueue* display_queue = nullptr,
                     bool log_output = false);
  virtual ~MonoVisionFrontEnd();

  gtsam::Pose3 getRelativePoseBody() const;

 private:
  void processFirstFrame(const Frame& firstFrame);

  MonoFrontendOutput::UniquePtr bootstrapSpin(
      const MonoFrontEndInputPayload& input);

  MonoFrontendOutput::UniquePtr nominalSpin(
      const MonoFrontEndInputPayload& input);

  StatusMonoMeasurementsPtr processFrame(
      const Frame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  void outlierRejectionMono(const gtsam::Rot3& calLrectlkf_R_camLrectKf_imu,
                            Frame* left_frame_lkf,
                            Frame* left_frame_k,
                            TrackingStatusPose* status_pose_mono);

  static void printTrackingStatus(const TrackingStatus& status) {
    LOG(INFO) << "Status mono: "
              << TrackerStatusSummary::asString(status);
  }

  void getSmartMonoMeasurements(const Frame::Ptr& frame,
                                MonoMeasurements* smart_mono_measurements);

  void sendFeatureTracksToLogger() const;

  void sendMonoTrackingToLogger() const;

  inline DebugTrackerInfo getTrackerInfo() const {
    // TODO(marcus): add back in
    // return tracker_.debug_info_;
    return DebugTrackerInfo();
  }

 private:
  // Current frame
  Frame::Ptr mono_frame_k_;
  // Last frame
  Frame::Ptr mono_frame_km1_;
  // Last keyframe
  Frame::Ptr mono_frame_lkf_;

  gtsam::Rot3 keyframe_R_ref_frame_;

  FeatureDetector::Ptr feature_detector_;
  
  Tracker::UniquePtr tracker_;

  Camera::Ptr mono_camera_;

  TrackerStatusSummary tracker_status_summary_;

  MonoFrontendParams frontend_params_;
};

}  // namespace VIO
