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

#include <boost/shared_ptr.hpp> // used for opengv

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class StereoVisionFrontEnd {
public:
 KIMERA_POINTER_TYPEDEFS(StereoVisionFrontEnd);
 KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVisionFrontEnd);
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 using StereoFrontEndInputPayload = StereoImuSyncPacket;
 // using StereoFrontEndOutputPayload = VioBackEndInputPayload;

public:
 StereoVisionFrontEnd(
     const ImuParams& imu_params,
     const ImuBias& imu_initial_bias,
     const VioFrontEndParams& tracker_params = VioFrontEndParams(),
     bool log_output = false);

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
  inline void updateAndResetImuBias(const ImuBias &imu_bias) const {
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
    CHECK_DOUBLE_EQ(getGravity().norm(),0.0);
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(),0.0);
  }

  /* ------------------------------------------------------------------------ */
  // Check values in frontend for initial bundle adjustment for online alignment
  void checkFrontendForOnlineAlignment() {
    CHECK(force_53point_ransac_);
    CHECK_DOUBLE_EQ(getGravity().norm(),0.0);
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(),0.0);
  }

  /* ------------------------------------------------------------------------ */
  // Reset frontend after initial bundle adjustment for online alignment
  void resetFrontendAfterOnlineAlignment(const gtsam::Vector3 &gravity,
                                      gtsam::Vector3 &gyro_bias) {
    LOG(WARNING) << "Resetting frontend after online alignment!\n";
    forceFiveThreePointMethod(false);
    resetGravity(gravity);
    gtsam::imuBias::ConstantBias final_bias(gtsam::Vector3::Zero(), gyro_bias);
    updateAndResetImuBias(final_bias);
    CHECK_DOUBLE_EQ(getGravity().norm(), gravity.norm());
    CHECK_DOUBLE_EQ(getCurrentImuBias().gyroscope().norm(), gyro_bias.norm());
  }

  /* ************************************************************************ */
  // NOT THREAD-SAFE METHODS
  /* ************************************************************************ */

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
      boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu = boost::none);

  /* ------------------------------------------------------------------------ */
  inline static void logTrackingStatus(const TrackingStatus& status,
                                       const std::string& type = "mono") {
    LOG(INFO) << "Status " << type << ": "
              << TrackerStatusSummary::asString(status);
  }

  /* ------------------------------------------------------------------------ */
  // Static function to display output of stereo tracker
  static void printStatusStereoMeasurements(
      const StatusStereoMeasurements& statusStereoMeasurements);

  // verbosity_ explanation (TODO: include this)
  /*
   * 0: no display
   * 1: show images
   * 2: write images (at each keyframe)
   * 3: write video
   * 4: write an image for each feature matched between left and right
   */
  /* ------------------------------------------------------------------------ */
  // Visualize quality of temporal and stereo matching
  void displayStereoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // Visualize quality of temporal and stereo matching
  void displayMonoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  void displaySaveImage(
      const cv::Mat& img_left,
      const std::string& text_on_img = "",
      const std::string& imshow_name = "mono tracking visualization (1 frame)",
      const std::string& folder_name_append = "-monoMatching1frame",
      const std::string& img_name_prepend = "monoTrackerDisplay1Keyframe_",
      const int verbosity = 0) const;

  /* ------------------------------------------------------------------------ */
  // Reset ImuFrontEnd gravity. Trivial gravity is needed for initial alignment.
  // This is thread-safe as imu_frontend_->resetPreintegrationGravity is thread-safe.
  void resetGravity(const gtsam::Vector3 &reset_value) const {
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

  // Counters.
  // Frame counte.
  int frame_count_;
  // keyframe counte.
  int keyframe_count_;
  // Previous number of landmarks (used for what is new landmark.
  int last_landmark_count_;
  // Timestamp of last keyframe.
  Timestamp last_keyframe_timestamp_;

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

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};

  // Frontend logger.
  std::unique_ptr<FrontendLogger> logger_;
};

} // namespace VIO

