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

#include "kimera-vio/frontend/VisionFrontEnd.h"

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

using StereoFrontEndInputPayload = StereoImuSyncPacket;

class StereoVisionFrontEnd : public VisionFrontEnd<StereoFrame,
                                                   StereoFrontEndInputPayload,
                                                   StereoFrontendOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoVisionFrontEnd);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVisionFrontEnd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  StereoVisionFrontEnd(const ImuParams& imu_params,
                       const ImuBias& imu_initial_bias,
                       const FrontendParams& tracker_params,
                       const StereoCamera::Ptr& stereo_camera,
                       DisplayQueue* display_queue = nullptr,
                       bool log_output = false);
  virtual ~StereoVisionFrontEnd();

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
   * @brief bootstrapSpin SpinOnce used when initializing the frontend.
   * @param input
   * @return
   */
  StereoFrontendOutput::UniquePtr bootstrapSpin(
      const StereoFrontEndInputPayload& input);

  /**
   * @brief nominalSpin SpinOnce used when in nominal mode after initialization
   * (bootstrap)
   * @param input
   * @return
   */
  StereoFrontendOutput::UniquePtr nominalSpin(
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
  static void printTrackingStatus(const TrackingStatus& status,
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
  // Force use of 3/5 point methods in initialization phase.
  // This despite the parameter specified in the tracker
  void forceFiveThreePointMethod(const bool force_flag) {
    force_53point_ransac_ = force_flag;
    LOG(WARNING) << "Forcing of 5/3 point method has been turned "
                 << (force_53point_ransac_ ? "ON!!" : "OFF");
  }

  /* ------------------------------------------------------------------------ */
  // Get tracker info.
  inline DebugTrackerInfo getTrackerInfo() const {
    return tracker_.debug_info_;
  }

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

  // Set of functionalities for tracking.
  Tracker tracker_;

  // A stereo camera
  StereoCamera::Ptr stereo_camera_;

  // Set of functionalities for stereo matching
  StereoMatcher stereo_matcher_;

  // Used to force the use of 5/3 point ransac, despite parameters
  std::atomic_bool force_53point_ransac_ = {false};

  // Summary of information from the tracker, e.g., relative pose estimates and
  // status of mono and stereo ransac
  TrackerStatusSummary trackerStatusSummary_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like
  std::string output_images_path_;

  // Frontend logger.
  std::unique_ptr<FrontendLogger> logger_;

  // Parameters
  FrontendParams frontend_params_;
};

}  // namespace VIO
