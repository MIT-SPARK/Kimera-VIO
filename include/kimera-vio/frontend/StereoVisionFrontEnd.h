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
                       const std::string& log_output_path = "");
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
  // Prepare frontend for initial bundle adjustment for online alignment
  void prepareFrontendForOnlineAlignment() {
    LOG(WARNING) << "Preparing frontend for online alignment!\n";
    updateAndResetImuBias(
        gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
    resetGravity(Vector3::Zero());
    forceFiveThreePointMethod(true);
    CHECK(force_53point_ransac_);
    CHECK_DOUBLE_EQ(getGravity().norm(), 0.0);
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(), 0.0);
  }

  /* ------------------------------------------------------------------------ */
  // Check values in frontend for initial bundle adjustment for online alignment
  void checkFrontendForOnlineAlignment() {
    CHECK(force_53point_ransac_);
    CHECK_DOUBLE_EQ(getGravity().norm(), 0.0);
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(), 0.0);
  }

  /* ------------------------------------------------------------------------ */
  // Reset frontend after initial bundle adjustment for online alignment
  void resetFrontendAfterOnlineAlignment(const gtsam::Vector3& gravity,
                                         gtsam::Vector3& gyro_bias) {
    LOG(WARNING) << "Resetting frontend after online alignment!\n";
    forceFiveThreePointMethod(false);
    resetGravity(gravity);
    gtsam::imuBias::ConstantBias final_bias(gtsam::Vector3::Zero(), gyro_bias);
    updateAndResetImuBias(final_bias);
    CHECK_DOUBLE_EQ(getGravity().norm(), gravity.norm());
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(), gyro_bias.norm());
  }

  /* ------------------------------------------------------------------------ */
  // Frontend initialization.
  StereoFrame processFirstStereoFrame(const StereoFrame& firstFrame);

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
  /* ------------------------------------------------------------------------ */
  // Frontend main function.
  StatusStereoMeasurementsPtr processStereoFrame(
      const StereoFrame& cur_frame,
      const gtsam::Rot3& keyframe_R_ref_frame_,
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
