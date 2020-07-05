/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionFrontEnd.cpp
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/frontend/StereoVisionFrontEnd.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

DEFINE_bool(visualize_feature_tracks, true, "Display feature tracks.");
DEFINE_bool(visualize_frontend_images,
            false,
            "Display images in frontend logger for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(save_frontend_images,
            false,
            "Save images in frontend logger to disk for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(log_feature_tracks, false, "Display/Save feature tracks images.");
DEFINE_bool(log_mono_tracking_images,
            false,
            "Display/Save mono tracking rectified and unrectified images.");
DEFINE_bool(log_stereo_matching_images,
            false,
            "Display/Save mono tracking rectified and unrectified images.");

namespace VIO {

StereoVisionFrontEnd::StereoVisionFrontEnd(
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const CameraParams& camera_params,
    DisplayQueue* display_queue,
    bool log_output)
    : frontend_state_(FrontendState::Bootstrap),
      stereoFrame_k_(nullptr),
      stereoFrame_km1_(nullptr),
      stereoFrame_lkf_(nullptr),
      keyframe_R_ref_frame_(gtsam::Rot3::identity()),
      frame_count_(0),
      keyframe_count_(0),
      feature_detector_(nullptr),
      tracker_(frontend_params, camera_params, display_queue),
      trackerStatusSummary_(),
      output_images_path_("./outputImages/"),
      display_queue_(display_queue),
      logger_(nullptr) {  // Only for debugging and visualization.
  if (log_output) {
    logger_ = VIO::make_unique<FrontendLogger>();
  }

  // Instantiate FeatureDetector
  feature_detector_ = VIO::make_unique<FeatureDetector>(
      frontend_params.feature_detector_params_);

  // Instantiate IMU frontend.
  imu_frontend_ = VIO::make_unique<ImuFrontEnd>(imu_params, imu_initial_bias);

  if (VLOG_IS_ON(1)) tracker_.tracker_params_.print();
}

/* -------------------------------------------------------------------------- */
FrontendOutput::UniquePtr StereoVisionFrontEnd::spinOnce(
    const StereoFrontEndInputPayload& input) {
  switch (frontend_state_) {
    case FrontendState::Bootstrap: {
      return bootstrapSpin(input);
    } break;
    case FrontendState::Nominal: {
      return nominalSpin(input);
    } break;
    default: { LOG(FATAL) << "Unrecognized frontend state."; } break;
  }
}

FrontendOutput::UniquePtr StereoVisionFrontEnd::bootstrapSpin(
    const StereoFrontEndInputPayload& input) {
  CHECK(frontend_state_ == FrontendState::Bootstrap);

  // Initialize members of the frontend
  processFirstStereoFrame(input.getStereoFrame());

  // Initialization done, set state to nominal
  frontend_state_ = FrontendState::Nominal;

  // Create mostly unvalid output, to send the imu_acc_gyrs to the backend.
  CHECK(stereoFrame_lkf_);
  return VIO::make_unique<FrontendOutput>(stereoFrame_lkf_->isKeyframe(),
                                          nullptr,
                                          TrackingStatus::DISABLED,
                                          getRelativePoseBodyStereo(),
                                          *stereoFrame_lkf_,
                                          nullptr,
                                          input.getImuAccGyrs(),
                                          cv::Mat(),
                                          getTrackerInfo());
}

FrontendOutput::UniquePtr StereoVisionFrontEnd::nominalSpin(
    const StereoFrontEndInputPayload& input) {
  CHECK(frontend_state_ == FrontendState::Nominal);
  // For timing
  utils::StatsCollector timing_stats_frame_rate("VioFrontEnd Frame Rate [ms]");
  utils::StatsCollector timing_stats_keyframe_rate(
      "VioFrontEnd Keyframe Rate [ms]");
  auto start_time = utils::Timer::tic();

  // Get stereo info
  const StereoFrame& stereoFrame_k = input.getStereoFrame();
  const auto& k = stereoFrame_k.getFrameId();
  VLOG(1) << "------------------- Processing frame k = " << k
          << "--------------------";

  ////////////////////////////// PROCESS IMU DATA //////////////////////////////

  // Print IMU data.
  if (VLOG_IS_ON(10)) input.print();

  // For k > 1
  // The preintegration btw frames is needed for RANSAC.
  // But note that we are using interpolated "fake" values when doing the preint
  // egration!! Should we remove them??
  // Actually, currently does not integrate fake interpolated meas as it does
  // not take the last measurement into account (although it takes its stamp
  // into account!!!).
  auto tic_full_preint = utils::Timer::tic();
  const ImuFrontEnd::PimPtr& pim = imu_frontend_->preintegrateImuMeasurements(
      input.getImuStamps(), input.getImuAccGyrs());
  CHECK(pim);

  auto full_preint_duration =
      utils::Timer::toc<std::chrono::microseconds>(tic_full_preint).count();
  // Don't add them because they are confusing
  // utils::StatsCollector stats_full_preint("IMU Preintegration Timing [us]");
  // stats_full_preint.AddSample(full_preint_duration);
  VLOG_IF(1, full_preint_duration != 0.0)
      << "Current IMU Preintegration frequency: " << 10e6 / full_preint_duration
      << " Hz. (" << full_preint_duration << " us).";

  // On the left camera rectified!!
  const gtsam::Rot3 body_Rot_cam =
      stereoFrame_km1_->getBPoseCamLRect().rotation();
  const gtsam::Rot3 cam_Rot_body = body_Rot_cam.inverse();

  // Relative rotation of the left cam rectified from the last keyframe to the
  // curr frame. pim.deltaRij() corresponds to bodyLkf_R_bodyK_imu
  gtsam::Rot3 camLrectLkf_R_camLrectK_imu =
      cam_Rot_body * pim->deltaRij() * body_Rot_cam;

  if (VLOG_IS_ON(10)) {
    body_Rot_cam.print("Body_Rot_cam");
    camLrectLkf_R_camLrectK_imu.print("calLrectLkf_R_camLrectK_imu");
  }
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////////// TRACKING /////////////////////////////////////
  // Main function for tracking.
  // Rotation used in 1 and 2 point ransac.
  VLOG(10) << "Starting processStereoFrame...";
  cv::Mat feature_tracks;
  StatusStereoMeasurementsPtr status_stereo_measurements = processStereoFrame(
      stereoFrame_k, camLrectLkf_R_camLrectK_imu, &feature_tracks);

  CHECK(!stereoFrame_k_);  // processStereoFrame is setting this to nullptr!!!
  VLOG(10) << "Finished processStereoFrame.";
  //////////////////////////////////////////////////////////////////////////////

  if (stereoFrame_km1_->isKeyframe()) {
    // We got a keyframe!
    CHECK_EQ(stereoFrame_lkf_->getTimestamp(),
             stereoFrame_km1_->getTimestamp());
    CHECK_EQ(stereoFrame_lkf_->getFrameId(), stereoFrame_km1_->getFrameId());
    CHECK(!stereoFrame_k_);
    CHECK(stereoFrame_lkf_->isKeyframe());
    VLOG(1) << "Keyframe " << k
            << " with: " << status_stereo_measurements->second.size()
            << " smart measurements";

    ////////////////// DEBUG INFO FOR FRONT-END ////////////////////////////////
    if (logger_) {
      logger_->logFrontendStats(
          stereoFrame_lkf_->getTimestamp(),
          getTrackerInfo(),
          trackerStatusSummary_,
          stereoFrame_km1_->getLeftFrame().getNrValidKeypoints());
      logger_->logFrontendRansac(stereoFrame_lkf_->getTimestamp(),
                                 getRelativePoseBodyMono(),
                                 getRelativePoseBodyStereo());
    }
    ////////////////////////////////////////////////////////////////////////////

    // Reset integration the later the better so that we give to the backend
    // the most time possible to update the IMU bias.
    VLOG(10) << "Reset IMU preintegration with latest IMU bias.";
    imu_frontend_->resetIntegrationWithCachedBias();

    // Record keyframe rate timing
    timing_stats_keyframe_rate.AddSample(utils::Timer::toc(start_time).count());

    // Return the output of the frontend for the others.
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
    return VIO::make_unique<FrontendOutput>(
        true,
        status_stereo_measurements,
        trackerStatusSummary_.kfTrackingStatus_stereo_,
        getRelativePoseBodyStereo(),
        *stereoFrame_lkf_, //! This is really the current keyframe in this if
        pim,
        input.getImuAccGyrs(),
        feature_tracks,
        getTrackerInfo());
  } else {
    // Record frame rate timing
    timing_stats_frame_rate.AddSample(utils::Timer::toc(start_time).count());

    // We don't have a keyframe.
    VLOG(2) << "Frontend output is not a keyframe. Skipping output queue push.";
    return VIO::make_unique<FrontendOutput>(false,
                                            status_stereo_measurements,
                                            TrackingStatus::INVALID,
                                            getRelativePoseBodyStereo(),
                                            *stereoFrame_lkf_,
                                            pim,
                                            input.getImuAccGyrs(),
                                            feature_tracks,
                                            getTrackerInfo());
  }
}

/* -------------------------------------------------------------------------- */
// TODO this can be greatly improved, but we need to get rid of global variables
// stereoFrame_km1_, stereoFrame_lkf_, stereoFrame_k_, etc...
void StereoVisionFrontEnd::processFirstStereoFrame(
    const StereoFrame& firstFrame) {
  VLOG(2) << "Processing first stereo frame \n";
  stereoFrame_k_ =
      std::make_shared<StereoFrame>(firstFrame);  // TODO this can be optimized!
  stereoFrame_k_->setIsKeyframe(true);
  last_keyframe_timestamp_ = stereoFrame_k_->getTimestamp();

  // Tracking is based on left frame.
  Frame* left_frame = stereoFrame_k_->getLeftFrameMutable();
  CHECK_EQ(left_frame->keypoints_.size(), 0)
      << "Keypoints already present in first frame: please do not extract"
         " keypoints manually";

  // Perform feature detection.
  CHECK(feature_detector_);
  feature_detector_->featureDetection(left_frame);

  // Get 3D points via stereo.
  stereoFrame_k_->sparseStereoMatching();

  // Prepare for next iteration.
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_lkf_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;

  ///////////////////////////// IMU FRONTEND ///////////////////////////////////
  // Initialize IMU frontend.
  imu_frontend_->resetIntegrationWithCachedBias();
}

/* -------------------------------------------------------------------------- */
// FRONTEND WORKHORSE
// THIS FUNCTION CAN BE GREATLY OPTIMIZED
StatusStereoMeasurementsPtr StereoVisionFrontEnd::processStereoFrame(
    const StereoFrame& cur_frame,
    const gtsam::Rot3& keyframe_R_cur_frame,
    cv::Mat* feature_tracks) {
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << cur_frame.getTimestamp() << " empirical framerate (sec): "
          << UtilsNumerical::NsecToSec(cur_frame.getTimestamp() -
                                       stereoFrame_km1_->getTimestamp())
          << " (timestamp diff: "
          << cur_frame.getTimestamp() - stereoFrame_km1_->getTimestamp() << ")";

  // TODO this copies the stereo frame!!
  stereoFrame_k_ = std::make_shared<StereoFrame>(cur_frame);

  // Copy rectification from previous frame to avoid recomputing it.
  // TODO avoid copying altogether...
  auto start_time = utils::Timer::tic();
  stereoFrame_k_->cloneRectificationParameters(*stereoFrame_km1_);
  double clone_rect_params_time = utils::Timer::toc(start_time).count();

  /////////////////////// TRACKING /////////////////////////////////////////////
  VLOG(2) << "Starting feature tracking...";
  // Track features from the previous frame
  Frame* left_frame_km1 = stereoFrame_km1_->getLeftFrameMutable();
  Frame* left_frame_k = stereoFrame_k_->getLeftFrameMutable();
  // We need to use the frame to frame rotation.
  gtsam::Rot3 ref_frame_R_cur_frame =
      keyframe_R_ref_frame_.inverse().compose(keyframe_R_cur_frame);
  tracker_.featureTracking(left_frame_km1, left_frame_k, ref_frame_R_cur_frame);

  if (feature_tracks) {
    // TODO(Toni): these feature tracks are not outlier rejected...
    // TODO(Toni): this image should already be computed and inside the
    // display_queue
    // if it is sent to the tracker.
    *feature_tracks = tracker_.getTrackerImage(stereoFrame_lkf_->getLeftFrame(),
                                               stereoFrame_k_->getLeftFrame());
  }
  VLOG(2) << "Finished feature tracking.";
  //////////////////////////////////////////////////////////////////////////////

  // Not tracking at all in this phase.
  trackerStatusSummary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
  trackerStatusSummary_.kfTrackingStatus_stereo_ = TrackingStatus::INVALID;

  // This will be the info we actually care about
  SmartStereoMeasurementsUniquePtr smart_stereo_measurements = nullptr;

  const bool max_time_elapsed =
      stereoFrame_k_->getTimestamp() - last_keyframe_timestamp_ >=
      tracker_.tracker_params_.intra_keyframe_time_ns_;
  const size_t& nr_valid_features = left_frame_k->getNrValidKeypoints();
  const bool nr_features_low =
      nr_valid_features <= tracker_.tracker_params_.min_number_features_;

  // Also if the user requires the keyframe to be enforced
  LOG_IF(WARNING, stereoFrame_k_->isKeyframe()) << "User enforced keyframe!";
  // If max time elaspsed and not able to track feature -> create new keyframe
  if (max_time_elapsed || nr_features_low || stereoFrame_k_->isKeyframe()) {
    ++keyframe_count_;  // mainly for debugging

    VLOG(2) << "+++++++++++++++++++++++++++++++++++++++++++++++++++"
            << "Keyframe after: "
            << UtilsNumerical::NsecToSec(stereoFrame_k_->getTimestamp() -
                                         last_keyframe_timestamp_)
            << " sec.";

    VLOG_IF(2, max_time_elapsed) << "Keyframe reason: max time elapsed.";
    VLOG_IF(2, nr_features_low)
        << "Keyframe reason: low nr of features (" << nr_valid_features << " < "
        << tracker_.tracker_params_.min_number_features_ << ").";

    double sparse_stereo_time = 0;
    if (tracker_.tracker_params_.useRANSAC_) {
      // MONO geometric outlier rejection
      TrackingStatusPose status_pose_mono;
      Frame* left_frame_lkf = stereoFrame_lkf_->getLeftFrameMutable();
      outlierRejectionMono(keyframe_R_cur_frame,
                           left_frame_lkf,
                           left_frame_k,
                           &status_pose_mono);

      // STEREO geometric outlier rejection
      // get 3D points via stereo
      start_time = utils::Timer::tic();
      stereoFrame_k_->sparseStereoMatching();
      sparse_stereo_time = utils::Timer::toc(start_time).count();

      TrackingStatusPose status_pose_stereo;
      outlierRejectionStereo(keyframe_R_cur_frame,
                             stereoFrame_lkf_,
                             stereoFrame_k_,
                             &status_pose_stereo);
      if (status_pose_stereo.first == TrackingStatus::VALID) {
        trackerStatusSummary_.lkf_T_k_stereo_ = status_pose_stereo.second;
      }
    } else {
      trackerStatusSummary_.kfTrackingStatus_mono_ = TrackingStatus::DISABLED;
      if (VLOG_IS_ON(2)) {
        printTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_,
                            "mono");
      }
      trackerStatusSummary_.kfTrackingStatus_stereo_ = TrackingStatus::DISABLED;
      if (VLOG_IS_ON(2)) {
        printTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_,
                            "stereo");
      }
    }
    // If its been long enough, make it a keyframe
    last_keyframe_timestamp_ = stereoFrame_k_->getTimestamp();
    stereoFrame_k_->setIsKeyframe(true);

    // Perform feature detection (note: this must be after RANSAC,
    // since if we discard more features, we need to extract more)
    CHECK(feature_detector_);
    feature_detector_->featureDetection(left_frame_k);

    // Get 3D points via stereo, including newly extracted
    // (this might be only for the visualization).
    start_time = utils::Timer::tic();
    stereoFrame_k_->sparseStereoMatching();
    sparse_stereo_time += utils::Timer::toc(start_time).count();

    // Log images if needed.
    if (logger_ &&
        (FLAGS_visualize_frontend_images || FLAGS_save_frontend_images)) {
      if (FLAGS_log_feature_tracks) sendFeatureTracksToLogger();
      if (FLAGS_log_mono_tracking_images) sendStereoMatchesToLogger();
      if (FLAGS_log_stereo_matching_images) sendMonoTrackingToLogger();
    }
    if (display_queue_ && FLAGS_visualize_feature_tracks) {
      displayImage(stereoFrame_k_->getTimestamp(),
                   "feature_tracks",
                   tracker_.getTrackerImage(stereoFrame_lkf_->getLeftFrame(),
                                            stereoFrame_k_->getLeftFrame()),
                   display_queue_);
    }

    // Populate statistics.
    tracker_.checkStatusRightKeypoints(stereoFrame_k_->right_keypoints_status_);

    // Move on.
    stereoFrame_lkf_ = stereoFrame_k_;

    // Get relevant info for keyframe.
    start_time = utils::Timer::tic();
    smart_stereo_measurements = getSmartStereoMeasurements(*stereoFrame_k_);
    double get_smart_stereo_meas_time = utils::Timer::toc(start_time).count();

    VLOG(2) << "timeClone: " << clone_rect_params_time << '\n'
            << "timeSparseStereo: " << sparse_stereo_time << '\n'
            << "timeGetMeasurements: " << get_smart_stereo_meas_time;
  } else {
    stereoFrame_k_->setIsKeyframe(false);
  }

  // Update keyframe to reference frame for next iteration.
  if (stereoFrame_k_->isKeyframe()) {
    // Reset relative rotation if we have a keyframe.
    keyframe_R_ref_frame_ = gtsam::Rot3::identity();
  } else {
    // Update rotation from keyframe to next iteration reference frame (aka
    // cur_frame in current iteration).
    keyframe_R_ref_frame_ = keyframe_R_cur_frame;
  }

  // Reset frames.
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;
  return VIO::make_unique<StatusStereoMeasurements>(
      std::make_pair(trackerStatusSummary_,
                     // TODO(Toni): please, fix this, don't use std::pair...
                     // copies, manyyyy copies: actually thousands of copies...
                     smart_stereo_measurements ? *smart_stereo_measurements
                                               : SmartStereoMeasurements()));
}

void StereoVisionFrontEnd::outlierRejectionMono(
    const gtsam::Rot3& calLrectLkf_R_camLrectKf_imu,
    Frame* left_frame_lkf,
    Frame* left_frame_k,
    TrackingStatusPose* status_pose_mono) {
  CHECK_NOTNULL(status_pose_mono);
  if (tracker_.tracker_params_.ransac_use_2point_mono_ &&
      !calLrectLkf_R_camLrectKf_imu.equals(gtsam::Rot3::identity()) &&
      !force_53point_ransac_) {
    // 2-point RANSAC.
    *status_pose_mono = tracker_.geometricOutlierRejectionMonoGivenRotation(
        left_frame_lkf, left_frame_k, calLrectLkf_R_camLrectKf_imu);
  } else {
    // 5-point RANSAC.
    *status_pose_mono =
        tracker_.geometricOutlierRejectionMono(left_frame_lkf, left_frame_k);
    LOG_IF(WARNING, force_53point_ransac_) << "5-point RANSAC was forced!";
  }

  // TODO(TONI): check the status of tracking here: aka look at median disp
  // and assess whether we are in LOW_DISP or ROT_ONLY

  // Set relative pose.
  trackerStatusSummary_.kfTrackingStatus_mono_ = status_pose_mono->first;

  if (VLOG_IS_ON(2)) {
    printTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");
  }

  if (status_pose_mono->first == TrackingStatus::VALID) {
    trackerStatusSummary_.lkf_T_k_mono_ = status_pose_mono->second;
  }
}

void StereoVisionFrontEnd::outlierRejectionStereo(
    const gtsam::Rot3& calLrectLkf_R_camLrectKf_imu,
    const StereoFrame::Ptr& left_frame_lkf,
    const StereoFrame::Ptr& left_frame_k,
    TrackingStatusPose* status_pose_stereo) {
  CHECK(left_frame_lkf);
  CHECK(left_frame_k);
  CHECK_NOTNULL(status_pose_stereo);

  gtsam::Matrix infoMatStereoTranslation = gtsam::Matrix3::Zero();
  if (tracker_.tracker_params_.ransac_use_1point_stereo_ &&
      !calLrectLkf_R_camLrectKf_imu.equals(gtsam::Rot3::identity()) &&
      !force_53point_ransac_) {
    // 1-point RANSAC.
    std::tie(*status_pose_stereo, infoMatStereoTranslation) =
        tracker_.geometricOutlierRejectionStereoGivenRotation(
            *stereoFrame_lkf_, *stereoFrame_k_, calLrectLkf_R_camLrectKf_imu);
  } else {
    // 3-point RANSAC.
    *status_pose_stereo = tracker_.geometricOutlierRejectionStereo(
        *stereoFrame_lkf_, *stereoFrame_k_);
    LOG_IF(WARNING, force_53point_ransac_) << "3-point RANSAC was enforced!";
  }

  // Set relative pose.
  trackerStatusSummary_.kfTrackingStatus_stereo_ = status_pose_stereo->first;
  trackerStatusSummary_.infoMatStereoTranslation_ = infoMatStereoTranslation;

  if (VLOG_IS_ON(2)) {
    printTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_,
                        "stereo");
  }
}

/* -------------------------------------------------------------------------- */
// TODO(Toni): THIS FUNCTION CAN BE GREATLY OPTIMIZED...
SmartStereoMeasurementsUniquePtr
StereoVisionFrontEnd::getSmartStereoMeasurements(
    const StereoFrame& stereoFrame_kf) const {
  // Sanity check first.
  CHECK(stereoFrame_kf.isRectified())
      << "getSmartStereoMeasurements: stereo pair is not rectified";
  // Checks dimensionality of the feature vectors.
  stereoFrame_kf.checkStereoFrame();

  // Extract relevant info from the stereo frame:
  // essentially the landmark if and the left/right pixel measurements.
  const LandmarkIds& landmarkId_kf = stereoFrame_kf.getLeftFrame().landmarks_;
  const KeypointsCV& leftKeypoints = stereoFrame_kf.left_keypoints_rectified_;
  const KeypointsCV& rightKeypoints = stereoFrame_kf.right_keypoints_rectified_;
  const std::vector<KeypointStatus>& rightKeypoints_status =
      stereoFrame_kf.right_keypoints_status_;

  // Pack information in landmark structure.
  SmartStereoMeasurementsUniquePtr smart_stereo_measurements =
      VIO::make_unique<SmartStereoMeasurements>();
  smart_stereo_measurements->reserve(landmarkId_kf.size());
  for (size_t i = 0; i < landmarkId_kf.size(); ++i) {
    if (landmarkId_kf.at(i) == -1) {
      continue;  // skip invalid points
    }

    // TODO implicit conversion float to double increases floating-point
    // precision!
    const double& uL = leftKeypoints.at(i).x;
    const double& v = leftKeypoints.at(i).y;
    // Initialize to missing pixel information.
    double uR = std::numeric_limits<double>::quiet_NaN();
    if (!tracker_.tracker_params_.useStereoTracking_) {
      LOG(WARNING) << "getSmartStereoMeasurements: dropping stereo information!"
                      " (set useStereoTracking_ = true to use it)";
    }
    if (tracker_.tracker_params_.useStereoTracking_ &&
        rightKeypoints_status.at(i) == KeypointStatus::VALID) {
      // TODO implicit conversion float to double increases floating-point
      // precision!
      uR = rightKeypoints.at(i).x;
    }
    smart_stereo_measurements->push_back(
        std::make_pair(landmarkId_kf[i], gtsam::StereoPoint2(uL, uR, v)));
  }
  return smart_stereo_measurements;
}

void StereoVisionFrontEnd::sendFeatureTracksToLogger() const {
  const Frame& left_frame_k(stereoFrame_k_->getLeftFrame());
  cv::Mat img_left =
      tracker_.getTrackerImage(stereoFrame_lkf_->getLeftFrame(), left_frame_k);

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left,
                          "monoFeatureTracksLeft",
                          "/monoFeatureTracksLeftImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::sendStereoMatchesToLogger() const {
  // Draw the matchings: assumes that keypoints in the left and right keyframe
  // are ordered in the same way
  const Frame& left_frame_k(stereoFrame_k_->getLeftFrame());
  const Frame& right_frame_k(stereoFrame_k_->getRightFrame());

  cv::Mat img_left =
      tracker_.getTrackerImage(stereoFrame_lkf_->getLeftFrame(), left_frame_k);

  if ((left_frame_k.img_.cols != right_frame_k.img_.cols) ||
      (left_frame_k.img_.rows != right_frame_k.img_.rows)) {
    LOG(FATAL) << "sendStereoMatchesToLogger: image dimension mismatch!";
  }

  cv::Mat img_right;
  right_frame_k.img_.copyTo(img_right);
  // stereoFrame_k_->keypoints_depth_

  std::vector<cv::DMatch> matches;
  if (left_frame_k.keypoints_.size() == right_frame_k.keypoints_.size()) {
    for (size_t i = 0; i < left_frame_k.keypoints_.size(); i++) {
      if (left_frame_k.landmarks_[i] != -1 &&
          stereoFrame_k_->right_keypoints_status_[i] == KeypointStatus::VALID) {
        matches.push_back(cv::DMatch(i, i, 0));
      }
    }
  } else {
    VLOG(10) << "No matches to show, since we did not compute keypoints "
                "in right frame.";
  }

  //############################################################################
  // Plot matches.
  cv::Mat img_left_right =
      UtilsOpenCV::DrawCornersMatches(img_left,
                                      left_frame_k.keypoints_,
                                      img_right,
                                      right_frame_k.keypoints_,
                                      matches,
                                      false);  // true: random color
  cv::putText(img_left_right,
              "S:" + std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left_right,
                          "stereoMatchingUnrectified",
                          "/stereoMatchingUnrectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  //############################################################################

  // Display rectified, plot matches.
  cv::Mat img_left_right_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_k_->left_img_rectified_,
      stereoFrame_k_->left_keypoints_rectified_,
      stereoFrame_k_->right_img_rectified_,
      stereoFrame_k_->right_keypoints_rectified_,
      matches,
      false);  // true: random color
  cv::putText(img_left_right_rectified,
              "S(Rect):" + std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left_right_rectified,
                          "stereoMatchingRectified",
                          "/stereoMatchingRectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::sendMonoTrackingToLogger() const {
  const Frame& cur_left_frame = stereoFrame_k_->getLeftFrame();
  const Frame& ref_left_frame = stereoFrame_lkf_->getLeftFrame();

  // Find keypoint matches.
  std::vector<cv::DMatch> matches;
  for (size_t i = 0; i < cur_left_frame.keypoints_.size(); ++i) {
    if (cur_left_frame.landmarks_.at(i) != -1) {  // if landmark is valid
      auto it = find(ref_left_frame.landmarks_.begin(),
                     ref_left_frame.landmarks_.end(),
                     cur_left_frame.landmarks_.at(i));
      if (it != ref_left_frame.landmarks_.end()) {  // if landmark was found
        int nPos = std::distance(ref_left_frame.landmarks_.begin(), it);
        matches.push_back(cv::DMatch(nPos, i, 0));
      }
    }
  }
  //############################################################################

  // Plot matches.
  cv::Mat img_left_lkf_kf =
      UtilsOpenCV::DrawCornersMatches(ref_left_frame.img_,
                                      ref_left_frame.keypoints_,
                                      cur_left_frame.img_,
                                      cur_left_frame.keypoints_,
                                      matches,
                                      false);  // true: random color
  cv::putText(img_left_lkf_kf,
              "M:" + std::to_string(keyframe_count_ - 1) + "-" +
                  std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(cur_left_frame.id_,
                          img_left_lkf_kf,
                          "monoTrackingUnrectified",
                          "/monoTrackingUnrectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  //############################################################################

  // Display rectified, plot matches.
  cv::Mat img_left_lkf_kf_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_lkf_->left_img_rectified_,
      stereoFrame_lkf_->left_keypoints_rectified_,
      stereoFrame_k_->left_img_rectified_,
      stereoFrame_k_->left_keypoints_rectified_,
      matches,
      false);  // true: random color
  cv::putText(img_left_lkf_kf_rectified,
              "M(Rect):" + std::to_string(keyframe_count_ - 1) + "-" +
                  std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(cur_left_frame.id_,
                          img_left_lkf_kf_rectified,
                          "monoTrackingRectified",
                          "/monoTrackingRectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  // TODO Visualization must be done in the main thread.
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - MONO
// RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyMono() const {
  // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ *
  // kCamera_T_kBody = body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
  gtsam::Pose3 body_Pose_cam_ =
      stereoFrame_lkf_->getBPoseCamLRect();  // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_mono_ *
         body_Pose_cam_.inverse();
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - STEREO
// RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyStereo() const {
  gtsam::Pose3 body_Pose_cam_ =
      stereoFrame_lkf_->getBPoseCamLRect();  // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_stereo_ *
         body_Pose_cam_.inverse();
}

/* ------------------------------------------------------------------------ */
// Static function to display output of stereo tracker
void StereoVisionFrontEnd::printStatusStereoMeasurements(
    const StatusStereoMeasurements& statusStereoMeasurements) {
  LOG(INFO) << " SmartStereoMeasurements with status:";
  printTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_mono_,
                      "mono");
  printTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_stereo_,
                      "stereo");
  LOG(INFO) << " observing landmarks:";
  const SmartStereoMeasurements& smartStereoMeas =
      statusStereoMeasurements.second;
  for (const auto& smart_stereo_meas : smartStereoMeas) {
    std::cout << " " << smart_stereo_meas.first << " ";
  }
  std::cout << std::endl;
}

}  // namespace VIO
