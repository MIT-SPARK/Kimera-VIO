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

#include "kimera-vio/common/vio_types.h"

DEFINE_int32(save_frontend_images_option,
             0,
             "Display/Save images in frontend for debugging (only use if "
             "in sequential mode, otherwise expect segfaults). "
             "Values:\n"
             " - 0: don't display or save images.\n"
             " - 1: display images.\n"
             " - 2: display and save images.");

namespace VIO {

StereoVisionFrontEnd::StereoVisionFrontEnd(
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const VioFrontEndParams& tracker_params,
    bool log_output)
    : frame_count_(0),
      keyframe_count_(0),
      last_landmark_count_(0),
      tracker_(tracker_params),
      trackerStatusSummary_(),
      output_images_path_("./outputImages/"),
      logger_(nullptr) {  // Only for debugging and visualization.

  if (log_output) {
    logger_ = VIO::make_unique<FrontendLogger>();
  }

  // Instantiate IMU frontend.
  imu_frontend_ = VIO::make_unique<ImuFrontEnd>(imu_params, imu_initial_bias);

  if (VLOG_IS_ON(1)) tracker_.trackerParams_.print();
}

/* -------------------------------------------------------------------------- */
FrontendOutput::UniquePtr StereoVisionFrontEnd::spinOnce(
    const StereoFrontEndInputPayload& input) {
  const StereoFrame& stereoFrame_k = input.getStereoFrame();
  const auto& k = stereoFrame_k.getFrameId();
  LOG(INFO) << "------------------- Processing frame k = " << k
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
      input.getImuStamps(), input.getImuAccGyr());
  CHECK(pim);

  auto full_preint_duration =
      utils::Timer::toc<std::chrono::microseconds>(tic_full_preint).count();
  utils::StatsCollector stats_full_preint("IMU Preintegration Timing [us]");
  stats_full_preint.AddSample(full_preint_duration);
  VLOG_IF(1, full_preint_duration != 0.0)
      << "Current IMU Preintegration frequency: " << 10e6 / full_preint_duration
      << " Hz. (" << full_preint_duration << " us).";

  // On the left camera rectified!!
  static const gtsam::Rot3 body_Rot_cam =
      stereoFrame_km1_->getBPoseCamLRect().rotation();
  static const gtsam::Rot3 cam_Rot_body = body_Rot_cam.inverse();

  // Relative rotation of the left cam rectified from the last keyframe to the
  // curr frame. pim.deltaRij() corresponds to bodyLkf_R_bodyK_imu
  gtsam::Rot3 calLrectLkf_R_camLrectK_imu =
      cam_Rot_body * pim->deltaRij() * body_Rot_cam;

  if (VLOG_IS_ON(10)) {
    body_Rot_cam.print("Body_Rot_cam");
    calLrectLkf_R_camLrectK_imu.print("calLrectLkf_R_camLrectK_imu");
  }
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////////// TRACKING /////////////////////////////////////
  // Main function for tracking.
  // Rotation used in 1 and 2 point ransac.
  VLOG(10) << "Starting processStereoFrame...";
  StatusStereoMeasurementsPtr status_stereo_measurements =
      processStereoFrame(stereoFrame_k, calLrectLkf_R_camLrectK_imu);

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
    LOG(INFO) << "Keyframe " << k
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

    // Return the output of the frontend for the others.
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
    return VIO::make_unique<FrontendOutput>(
        true,
        status_stereo_measurements,
        trackerStatusSummary_.kfTrackingStatus_stereo_,
        getRelativePoseBodyStereo(),
        *stereoFrame_lkf_,
        pim,
        getTrackerInfo());
  } else {
    // We don't have a keyframe.
    VLOG(2) << "Frontend output is not a keyframe. Skipping output queue push.";
    return VIO::make_unique<FrontendOutput>(false,
                                            status_stereo_measurements,
                                            TrackingStatus::INVALID,
                                            getRelativePoseBodyStereo(),
                                            *stereoFrame_lkf_,
                                            pim,
                                            getTrackerInfo());
  }
}

/* -------------------------------------------------------------------------- */
// TODO this can be greatly improved, but we need to get rid of global variables
// stereoFrame_km1_, stereoFrame_lkf_, stereoFrame_k_, etc...
StereoFrame StereoVisionFrontEnd::processFirstStereoFrame(
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

  // Initialize mask: this is allocated but it does not play a role
  // in this function.
  cv::Size imgSize = left_frame->img_.size();
  tracker_.camMask_ = cv::Mat(imgSize, CV_8UC1, cv::Scalar(255));

  // Perform feature detection.
  tracker_.featureDetection(left_frame);

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

  return *stereoFrame_lkf_;
}

/* -------------------------------------------------------------------------- */
// FRONTEND WORKHORSE
// THIS FUNCTION CAN BE GREATLY OPTIMIZED
StatusStereoMeasurementsPtr StereoVisionFrontEnd::processStereoFrame(
    const StereoFrame& cur_frame,
    boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu) {
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << cur_frame.getTimestamp() << " empirical framerate (sec): "
          << UtilsOpenCV::NsecToSec(cur_frame.getTimestamp() -
                                    stereoFrame_km1_->getTimestamp())
          << " (timestamp diff: "
          << cur_frame.getTimestamp() - stereoFrame_km1_->getTimestamp() << ")";

  double start_time = UtilsOpenCV::GetTimeInSeconds();
  double time_to_clone_rect_params = 0;

  // TODO this copies the stereo frame!!
  stereoFrame_k_ = std::make_shared<StereoFrame>(cur_frame);

  // Copy rectification from previous frame to avoid recomputing it.
  // TODO avoid copying altogether...
  stereoFrame_k_->cloneRectificationParameters(*stereoFrame_km1_);
  time_to_clone_rect_params = UtilsOpenCV::GetTimeInSeconds() - start_time;

  // Only for visualization.
  int verbosityFrames = FLAGS_save_frontend_images_option;
  int verbosityKeyframes = FLAGS_save_frontend_images_option;

  double timeSparseStereo = 0;
  double timeGetMeasurements = 0;

  /////////////////////// TRACKING /////////////////////////////////////////////
  VLOG(2) << "Starting feature tracking...";
  // Track features from the previous frame
  Frame* left_frame_km1 = stereoFrame_km1_->getLeftFrameMutable();
  Frame* left_frame_k = stereoFrame_k_->getLeftFrameMutable();
  tracker_.featureTracking(left_frame_km1, left_frame_k);
  if (verbosityFrames > 0) {
    // TODO this won't work in parallel mode...
    tracker_.displayFrame(*left_frame_km1, *left_frame_k, false);
  }
  //////////////////////////////////////////////////////////////////////////////

  // Not tracking at all in this phase.
  trackerStatusSummary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
  trackerStatusSummary_.kfTrackingStatus_stereo_ = TrackingStatus::INVALID;

  // This will be the info we actually care about
  SmartStereoMeasurementsUniquePtr smartStereoMeasurements = nullptr;

  const bool max_time_elapsed =
      UtilsOpenCV::NsecToSec(stereoFrame_k_->getTimestamp() -
                             last_keyframe_timestamp_) >=
      tracker_.trackerParams_.intra_keyframe_time_;
  const size_t nr_valid_features = left_frame_k->getNrValidKeypoints();
  const bool nr_features_low =
      nr_valid_features <= tracker_.trackerParams_.min_number_features_;

  // Also if the user requires the keyframe to be enforced
  if (stereoFrame_k_->isKeyframe()) LOG(WARNING) << "User inforced keyframe!";
  // If max time elaspsed and not able to track feature -> create new keyframe
  if (max_time_elapsed || nr_features_low || stereoFrame_k_->isKeyframe()) {
    ++keyframe_count_;  // mainly for debugging

    VLOG(2) << "+++++++++++++++++++++++++++++++++++++++++++++++++++"
            << "Keyframe after: "
            << UtilsOpenCV::NsecToSec(stereoFrame_k_->getTimestamp() -
                                      last_keyframe_timestamp_)
            << " sec.";

    VLOG_IF(2, max_time_elapsed) << "Keyframe reason: max time elapsed.";
    VLOG_IF(2, nr_features_low)
        << "Keyframe reason: low nr of features (" << nr_valid_features << " < "
        << tracker_.trackerParams_.min_number_features_ << ").";

    if (!tracker_.trackerParams_.useRANSAC_) {
      trackerStatusSummary_.kfTrackingStatus_mono_ = TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) {
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");
      }

      trackerStatusSummary_.kfTrackingStatus_stereo_ = TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) {
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_,
                          "stereo");
      }
    } else {
      ////////////////// MONO geometric outlier rejection ////////////////
      std::pair<TrackingStatus, gtsam::Pose3> statusPoseMono;
      Frame* left_frame_lkf = stereoFrame_lkf_->getLeftFrameMutable();
      if (tracker_.trackerParams_.ransac_use_2point_mono_ &&
          calLrectLkf_R_camLrectKf_imu && !force_53point_ransac_) {
        // 2-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMonoGivenRotation(
            left_frame_lkf, left_frame_k, *calLrectLkf_R_camLrectKf_imu);
      } else {
        // 5-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMono(left_frame_lkf,
                                                                left_frame_k);
        if (force_53point_ransac_)
          LOG(WARNING) << "5-point RANSAC was enforced!";
      }

      // Set relative pose.
      trackerStatusSummary_.kfTrackingStatus_mono_ = statusPoseMono.first;

      if (VLOG_IS_ON(2))
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");

      if (statusPoseMono.first == TrackingStatus::VALID) {
        trackerStatusSummary_.lkf_T_k_mono_ = statusPoseMono.second;
      }

      if (verbosityFrames > 0) {
        tracker_.displayFrame(*left_frame_km1, *left_frame_k, false);
      }

      ////////////////// STEREO geometric outlier rejection ////////////////
      // get 3D points via stereo
      start_time = UtilsOpenCV::GetTimeInSeconds();
      stereoFrame_k_->sparseStereoMatching();
      timeSparseStereo = UtilsOpenCV::GetTimeInSeconds() - start_time;

      std::pair<TrackingStatus, gtsam::Pose3> statusPoseStereo;
      gtsam::Matrix infoMatStereoTranslation = gtsam::Matrix3::Zero();
      if (tracker_.trackerParams_.ransac_use_1point_stereo_ &&
          calLrectLkf_R_camLrectKf_imu && !force_53point_ransac_) {
        // 1-point RANSAC.
        std::tie(statusPoseStereo, infoMatStereoTranslation) =
            tracker_.geometricOutlierRejectionStereoGivenRotation(
                *stereoFrame_lkf_,
                *stereoFrame_k_,
                *calLrectLkf_R_camLrectKf_imu);
      } else {
        // 3-point RANSAC.
        statusPoseStereo = tracker_.geometricOutlierRejectionStereo(
            *stereoFrame_lkf_, *stereoFrame_k_);
        if (force_53point_ransac_)
          LOG(WARNING) << "3-point RANSAC was enforced!";
      }

      // Set relative pose.
      trackerStatusSummary_.kfTrackingStatus_stereo_ = statusPoseStereo.first;
      trackerStatusSummary_.infoMatStereoTranslation_ =
          infoMatStereoTranslation;

      if (VLOG_IS_ON(2))
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_,
                          "stereo");

      if (statusPoseStereo.first == TrackingStatus::VALID) {
        trackerStatusSummary_.lkf_T_k_stereo_ = statusPoseStereo.second;
      }
    }
    // If its been long enough, make it a keyframe
    last_keyframe_timestamp_ = stereoFrame_k_->getTimestamp();
    stereoFrame_k_->setIsKeyframe(true);

    // Perform feature detection (note: this must be after RANSAC,
    // since if we discard more features, we need to extract more)
    tracker_.featureDetection(left_frame_k);

    // Get 3D points via stereo, including newly extracted
    // (this might be only for the visualization).
    start_time = UtilsOpenCV::GetTimeInSeconds();
    stereoFrame_k_->sparseStereoMatching();
    timeSparseStereo += UtilsOpenCV::GetTimeInSeconds() - start_time;

    // Show results.
    // verbosityKeyframes = 1;
    if (verbosityKeyframes > 0) {
      displayStereoTrack(verbosityKeyframes);
      displayMonoTrack(verbosityKeyframes);
    }

    // Populate statistics.
    tracker_.checkStatusRightKeypoints(stereoFrame_k_->right_keypoints_status_);

    // Move on.
    last_landmark_count_ = tracker_.landmark_count_;
    stereoFrame_lkf_ = stereoFrame_k_;

    // Get relevant info for keyframe.
    start_time = UtilsOpenCV::GetTimeInSeconds();
    smartStereoMeasurements = getSmartStereoMeasurements(*stereoFrame_k_);
    timeGetMeasurements = UtilsOpenCV::GetTimeInSeconds() - start_time;

    VLOG(2) << "timeClone: " << time_to_clone_rect_params << '\n'
            << "timeSparseStereo: " << timeSparseStereo << '\n'
            << "timeGetMeasurements: " << timeGetMeasurements;
  } else {
    stereoFrame_k_->setIsKeyframe(false);
  }

  // Reset frames.
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;
  VLOG(2) << "Finished feature tracking.";
  return VIO::make_unique<StatusStereoMeasurements>(
      std::make_pair(trackerStatusSummary_,
                     // TODO(Toni): please, fix this, don't use std::pair...
                     // copies, manyyyy copies: actually thousands of copies...
                     smartStereoMeasurements ? *smartStereoMeasurements
                                             : SmartStereoMeasurements()));
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
    if (!tracker_.trackerParams_.useStereoTracking_) {
      LOG(WARNING) << "getSmartStereoMeasurements: dropping stereo information!"
                      " (set useStereoTracking_ = true to use it)";
    }
    if (tracker_.trackerParams_.useStereoTracking_ &&
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

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayStereoTrack(const int& verbosity) const {
  const Frame& left_frame_k = stereoFrame_k_->getLeftFrame();

  // Show current frame with tracking results
  // The output of the following is already a color image
  cv::Mat img_left = tracker_.displayFrame(
      stereoFrame_lkf_->getLeftFrame(), left_frame_k, false);
  // TODO(Toni) put this inside tracker_.displayFrames?
  displaySaveImage(img_left,
                   "",
                   "mono tracking visualization (1 frame)",
                   "-monoMatching1frame",
                   "monoTrackerDisplay1Keyframe_",
                   verbosity == 2 ? 2 : 0);  // Don't display twice...

  // Draw the matchings: assumes that keypoints in the left and right keyframe
  // are ordered in the same way
  const Frame& right_frame_k(stereoFrame_k_->getRightFrame());

  if ((left_frame_k.img_.cols != right_frame_k.img_.cols) ||
      (left_frame_k.img_.rows != right_frame_k.img_.rows)) {
    LOG(FATAL) << "displayStereoTrack: image dimension mismatch!";
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
                                      true);  // true: random color
  // TODO(Toni) put this inside tracker_.displayFrames?
  displaySaveImage(img_left_right,
                   "S:" + std::to_string(keyframe_count_),
                   "stereo tracking visualization",
                   "-stereoMatchingUnrectified",
                   "StereoTrackerDisplayKeyframe_",
                   verbosity);
  //############################################################################
  //////////////////////////////////////////////////////////////////////////////
  //############################################################################
  // Display rectified, plot matches.
  cv::Mat img_left_right_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_k_->left_img_rectified_,
      stereoFrame_k_->left_keypoints_rectified_,
      stereoFrame_k_->right_img_rectified_,
      stereoFrame_k_->right_keypoints_rectified_,
      matches,
      true);  // true: random color
  displaySaveImage(img_left_right_rectified,
                   "S(Rect):" + std::to_string(keyframe_count_),
                   "stereo tracking visualization (rectified)",
                   "-stereoMatchingRectified",
                   "stereoTrackerDisplayKeyframe_",
                   verbosity);
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayMonoTrack(const int& verbosity) const {
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
                                      true);  // true: random color
  // TODO Visualization must be done in the main thread.
  displaySaveImage(img_left_lkf_kf,
                   "M:" + std::to_string(keyframe_count_ - 1) + "-" +
                       std::to_string(keyframe_count_),
                   "mono tracking visualization",
                   "-monoMatchingUnrectified",
                   "monoTrackerDispalyKeyframe_",
                   verbosity);
  //############################################################################

  //############################################################################
  //////////////////////////////////////////////////////////////////////////////
  //############################################################################
  // Display rectified, plot matches.
  cv::Mat img_left_lkf_kf_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_lkf_->left_img_rectified_,
      stereoFrame_lkf_->left_keypoints_rectified_,
      stereoFrame_k_->left_img_rectified_,
      stereoFrame_k_->left_keypoints_rectified_,
      matches,
      true);  // true: random color
  // TODO Visualization must be done in the main thread.
  displaySaveImage(img_left_lkf_kf_rectified,
                   "M(Rect):" + std::to_string(keyframe_count_ - 1) + "-" +
                       std::to_string(keyframe_count_),
                   "mono tracking visualization (rectified)",
                   "-monoMatchingRectified",
                   "monoTrackerDispalyKeyframe_",
                   verbosity);
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

/* -------------------------------------------------------------------------- */
// TODO imshow/waitKey can only be called in main thread.
// This function is just to wrap a lot of duplicated code that was
// floating around.
void StereoVisionFrontEnd::displaySaveImage(
    const cv::Mat& img_left,
    const std::string& text_on_img,
    const std::string& imshow_name,
    const std::string& folder_name_append,
    const std::string& img_name_prepend,
    const int verbosity) const {
  // Plot text with keyframe id.
  if (!text_on_img.empty()) {
    cv::putText(img_left,
                text_on_img,
                KeypointCV(10, 15),
                CV_FONT_HERSHEY_COMPLEX,
                0.6,
                cv::Scalar(0, 255, 0));
  }
  if (verbosity == 1) {  // otherwise just return the image
    cv::imshow(imshow_name, img_left);
    cv::waitKey(1);
  } else if (verbosity == 2) {
    // Create output folders:
    std::string folderName =
        output_images_path_ + "-" + folder_name_append + "/";
    boost::filesystem::path tracker_dir(folderName.c_str());
    boost::filesystem::create_directory(tracker_dir);
    // Write image.
    std::string img_name =
        folderName + img_name_prepend +
        std::to_string(stereoFrame_lkf_->getLeftFrame().id_) + ".png";
    LOG(INFO) << "Writing image: " << img_name;
    cv::imwrite(img_name, img_left);
  }
}

/* ------------------------------------------------------------------------ */
// Static function to display output of stereo tracker
void StereoVisionFrontEnd::printStatusStereoMeasurements(
    const StatusStereoMeasurements& statusStereoMeasurements) {
  LOG(INFO) << " SmartStereoMeasurements with status:";
  logTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_mono_,
                    "mono");
  logTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_stereo_,
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
