/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionFrontEnd.h
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <memory>
#include <atomic>

#include <boost/shared_ptr.hpp>  // used for opengv

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class StereoVisionFrontEnd {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoVisionFrontEnd);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVisionFrontEnd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using StereoFrontEndInputPayload = StereoImuSyncPacket;

 public:
  StereoVisionFrontEnd(const ImuParams& imu_params,
                       const ImuBias& imu_initial_bias,
                       const FrontendParams& tracker_params,
                       const CameraParams& camera_params,
                       DisplayQueue* display_queue = nullptr,
                       bool log_output = false);
  virtual ~StereoVisionFrontEnd() {
    LOG(INFO) << "StereoVisionFrontEnd destructor called.";
  }

 public:
  /* ------------------------------------------------------------------------ */
  // Update Imu Bias. This is thread-safe as imu_frontend_->updateBias is
  // thread-safe.
  inline void updateImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
  }

  /**
   * @brief isInitialized Returns whether the frontend is initializing.
   * Needs to be Thread-Safe! Therefore, frontend_state_ is atomic.
   */
  inline bool isInitialized() const {
    return frontend_state_ != FrontendState::Bootstrap;
  }

  /* ------------------------------------------------------------------------ */
  // Get Imu Bias. This is thread-safe as imu_frontend_->getCurrentImuBias is
  // thread-safe.
  inline ImuBias getCurrentImuBias() const {
    return imu_frontend_->getCurrentImuBias();
  }

  /* ------------------------------------------------------------------------ */
  // Update Imu Bias and reset pre-integration during initialization.
  // This is not thread-safe! (no multi-thread during initialization)
  inline void updateAndResetImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
    imu_frontend_->resetIntegrationWithCachedBias();
  }

  /* ------------------------------------------------------------------------ */
  // Returns extracted left and right rectified features in a suitable format
  // for VIO.
  SmartStereoMeasurementsUniquePtr getSmartStereoMeasurements(
      const StereoFrame& stereoFrame_kf) const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const;

  // private: // TODO: Fix access to this function. Is this thread safe???
  /* ------------------------------------------------------------------------ */
  FrontendOutput::UniquePtr spinOnce(const StereoFrontEndInputPayload& input);

  /* ------------------------------------------------------------------------ */
  // Get IMU Params for IMU Frontend.
  gtsam::PreintegratedImuMeasurements::Params getImuFrontEndParams() {
    return imu_frontend_->getGtsamImuParams();
  }

 private:
  enum class FrontendState {
    Bootstrap = 0u,  //! Initialize frontend
    Nominal = 1u     //! Run frontend
  };
  std::atomic<FrontendState> frontend_state_;

  /* ------------------------------------------------------------------------ */
  // Frontend initialization.
  void processFirstStereoFrame(const StereoFrame& firstFrame);

  /**
   * @brief bootstrapSpin SpinOnce used when initializing the frontend.
   * @param input
   * @return
   */
  FrontendOutput::UniquePtr bootstrapSpin(
      const StereoFrontEndInputPayload& input);

  /**
   * @brief nominalSpin SpinOnce used when in nominal mode after initialization
   * (bootstrap)
   * @param input
   * @return
   */
  FrontendOutput::UniquePtr nominalSpin(
      const StereoFrontEndInputPayload& input);

  /* ------------------------------------------------------------------------ */
  // Frontend main function.
  StatusStereoMeasurementsPtr processStereoFrame(
      const StereoFrame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame,
      cv::Mat* feature_tracks = nullptr);

  /* ------------------------------------------------------------------------ */
  void outlierRejectionMono(const gtsam::Rot3& calLrectLkf_R_camLrectKf_imu,
                            Frame* left_frame_lkf,
                            Frame* left_frame_k,
                            TrackingStatusPose* status_pose_mono);

  /* ------------------------------------------------------------------------ */
  void outlierRejectionStereo(const gtsam::Rot3& calLrectLkf_R_camLrectKf_imu,
                              const StereoFrame::Ptr& left_frame_lkf,
                              const StereoFrame::Ptr& left_frame_k,
                              TrackingStatusPose* status_pose_stereo);

  /* ------------------------------------------------------------------------ */
  inline static void printTrackingStatus(const TrackingStatus& status,
                                         const std::string& type = "mono") {
    LOG(INFO) << "Status " << type << ": "
              << TrackerStatusSummary::asString(status);
  }

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

  /* ------------------------------------------------------------------------ */
  // Reset ImuFrontEnd gravity. Trivial gravity is needed for initial alignment.
  // This is thread-safe as imu_frontend_->resetPreintegrationGravity is
  // thread-safe.
  void resetGravity(const gtsam::Vector3& reset_value) const {
    imu_frontend_->resetPreintegrationGravity(reset_value);
  }

  /* ------------------------------------------------------------------------ */
  // Get ImuFrontEnd gravity.
  // This is thread-safe as imu_frontend_->getPreintegrationGravity is
  // thread-safe.
  inline gtsam::Vector3 getGravity() const {
    return imu_frontend_->getPreintegrationGravity();
  }

  /* ------------------------------------------------------------------------ */
  // Force use of 3/5 point methods in initialization phase.
  // This despite the parameter specified in the tracker
  void forceFiveThreePointMethod(const bool force_flag) {
    force_53point_ransac_ = force_flag;
    LOG(WARNING) << "Forcing of 5/3 point method has been turned "
                 << (force_53point_ransac_ ? "ON!!" : "OFF");
  }

  /* ------------------------------------------------------------------------ */
  // Get tracker info.
  inline DebugTrackerInfo getTrackerInfo() {
    return tracker_.getTrackerDebugInfo();
  }

 private:
  // TODO MAKE THESE GUYS std::unique_ptr, we do not want to have multiple
  // owners, instead they should be passed around.
  // Stereo Frames
  // Current frame
  std::shared_ptr<StereoFrame> stereoFrame_k_;
  // Last frame
  std::shared_ptr<StereoFrame> stereoFrame_km1_;
  // Last keyframe
  std::shared_ptr<StereoFrame> stereoFrame_lkf_;

  // Rotation from last keyframe to reference frame
  // We use this to calculate the rotation btw reference frame and current frame
  // Whenever a keyframe is created, we reset it to identity.
  gtsam::Rot3 keyframe_R_ref_frame_;

  // Counters.
  int frame_count_;
  int keyframe_count_;

  // Timestamp of last keyframe.
  Timestamp last_keyframe_timestamp_;

  // Create the feature detector
  FeatureDetector::UniquePtr feature_detector_;

  // Set of functionalities for tracking.
  Tracker tracker_;

  // IMU frontend.
  std::unique_ptr<ImuFrontEnd> imu_frontend_;

  // Used to force the use of 5/3 point ransac, despite parameters
  std::atomic_bool force_53point_ransac_ = {false};

  // Summary of information from the tracker, e.g., relative pose estimates and
  // status of mono and stereo ransac
  TrackerStatusSummary trackerStatusSummary_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like
  std::string output_images_path_;

  // Display queue
  DisplayQueue* display_queue_;

  // Frontend logger.
  std::unique_ptr<FrontendLogger> logger_;
};

}  // namespace VIO
