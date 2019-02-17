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
 * @author Antoni Rosinol, Luca Carlone
 */

#include "StereoVisionFrontEnd.h"
#include <glog/logging.h>

namespace VIO {

StereoVisionFrontEnd::StereoVisionFrontEnd(
    const VioFrontEndParams& trackerParams,
    int saveImages,
    const std::string& dataset_name) :
  frame_count_(0),
  keyframe_count_(0),
  last_landmark_count_(0),
  tracker_(trackerParams, saveImages),
  saveImages_(saveImages),
  trackerStatusSummary_(),
  outputImagesPath_("./outputImages/") { // Only for debugging and visualization.
  if (saveImages > 0) {
    outputImagesPath_ = "./outputStereoTrackerImages-" + dataset_name;
    tracker_.outputImagesPath_ = "./outputTrackerImages-" + dataset_name;
  }
  tracker_.trackerParams_.print();
}

/* -------------------------------------------------------------------------- */
// TODO this can be greatly improved, but we need to get rid of global variables
// stereoFrame_km1_, stereoFrame_lkf_, stereoFrame_k_, etc...
void StereoVisionFrontEnd::processFirstStereoFrame(
    const StereoFrame& firstFrame) {
  VLOG(2) << "Processing first stereo frame \n";
  stereoFrame_k_ = std::make_shared<StereoFrame>(firstFrame); // TODO this can be optimized!
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
}

/* -------------------------------------------------------------------------- */
// FRONTEND WORKHORSE
// THIS FUNCTION CAN BE GREATLY OPTIMIZED
StatusSmartStereoMeasurements StereoVisionFrontEnd::processStereoFrame(
    StereoFrame cur_frame, // Pass by value and use move semantics!
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

  // ! Using move semantics for efficiency.
  VLOG(10) << "Using move semantics to copy cur_frame.";
  stereoFrame_k_ = std::make_shared<StereoFrame>(std::move(cur_frame));

  // Copy rectification from previous frame to avoid recomputing it.
  // TODO avoid copying altogether...
  stereoFrame_k_->cloneRectificationParameters(*stereoFrame_km1_);
  time_to_clone_rect_params = UtilsOpenCV::GetTimeInSeconds() - start_time;

  // Only for visualization.
  // TODO remove these. Use gflags.
  int verbosityFrames = saveImages_; // default: 0
  int verbosityKeyframes = saveImages_; // default: 1

  double timeSparseStereo = 0;
  double timeGetMeasurements = 0;

  /////////////////////// TRACKING /////////////////////////////////////////////
  VLOG(2) << "Starting feature tracking...";
  // Track features from the previous frame
  Frame* left_frame_km1 = stereoFrame_km1_->getLeftFrameMutable();
  Frame* left_frame_k = stereoFrame_k_->getLeftFrameMutable();
  tracker_.featureTracking(left_frame_km1,
                           left_frame_k);
  if (verbosityFrames > 0) {
    // TODO this won't work in parallel mode...
    tracker_.displayFrame(*left_frame_km1, *left_frame_k, verbosityFrames);
  }
  //////////////////////////////////////////////////////////////////////////////

  // Not tracking at all in this phase.
  trackerStatusSummary_.kfTrackingStatus_mono_ =
      Tracker::TrackingStatus::INVALID;
  trackerStatusSummary_.kfTrackingStatus_stereo_ =
      Tracker::TrackingStatus::INVALID;

  // This will be the info we actually care about
  SmartStereoMeasurements smartStereoMeasurements;

  const bool max_time_elapsed = UtilsOpenCV::NsecToSec(
        stereoFrame_k_->getTimestamp() - last_keyframe_timestamp_) >=
                                  tracker_.trackerParams_.intra_keyframe_time_;
  const size_t nr_valid_features =
      left_frame_k->getNrValidKeypoints();
  const bool nr_features_low =
      nr_valid_features <= tracker_.trackerParams_.min_number_features_;

  // If max time elaspsed and not able to track feature -> create new keyframe
  if (max_time_elapsed || nr_features_low) {
    ++keyframe_count_; // mainly for debugging

    VLOG(2) << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << "Keyframe after: "
            << UtilsOpenCV::NsecToSec(stereoFrame_k_->getTimestamp() - last_keyframe_timestamp_) << " sec.";

    VLOG_IF(2, max_time_elapsed) << "Keyframe reason: max time elapsed.";
    VLOG_IF(2, nr_features_low) << "Keyframe reason: low nr of features ("
                                << nr_valid_features << " < "
                                << tracker_.trackerParams_.min_number_features_ <<").";

    if (!tracker_.trackerParams_.useRANSAC_) {
      trackerStatusSummary_.kfTrackingStatus_mono_ = Tracker::TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) {
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");
      }

      trackerStatusSummary_.kfTrackingStatus_stereo_ = Tracker::TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) {
        logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_, "stereo");
      }
    } else {
      ////////////////// MONO geometric outlier rejection ////////////////
      std::pair<Tracker::TrackingStatus, gtsam::Pose3> statusPoseMono;
      Frame* left_frame_lkf = stereoFrame_lkf_->getLeftFrameMutable();
      if (tracker_.trackerParams_.ransac_use_2point_mono_ &&
          calLrectLkf_R_camLrectKf_imu) {
        // 2-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMonoGivenRotation(
            left_frame_lkf, left_frame_k, *calLrectLkf_R_camLrectKf_imu);
      } else {
        // 5-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMono(
              left_frame_lkf, left_frame_k);
      }

      // Set relative pose.
      trackerStatusSummary_.kfTrackingStatus_mono_ = statusPoseMono.first;

      if (VLOG_IS_ON(2)) logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");

      if (statusPoseMono.first == Tracker::TrackingStatus::VALID) {
          trackerStatusSummary_.lkf_T_k_mono_ = statusPoseMono.second;
      }

      if (verbosityFrames > 0) {
          tracker_.displayFrame(*left_frame_km1, *left_frame_k, verbosityFrames);
      }

      ////////////////// STEREO geometric outlier rejection ////////////////
      // get 3D points via stereo
      start_time = UtilsOpenCV::GetTimeInSeconds();
      stereoFrame_k_->sparseStereoMatching();
      timeSparseStereo = UtilsOpenCV::GetTimeInSeconds() - start_time;

      std::pair<Tracker::TrackingStatus,gtsam::Pose3> statusPoseStereo;
      gtsam::Matrix infoMatStereoTranslation = gtsam::Matrix3::Zero();
      if (tracker_.trackerParams_.ransac_use_1point_stereo_ &&
          calLrectLkf_R_camLrectKf_imu) {
        // 1-point RANSAC.
        std::tie(statusPoseStereo, infoMatStereoTranslation) =
                tracker_.geometricOutlierRejectionStereoGivenRotation(
                    *stereoFrame_lkf_, *stereoFrame_k_,
                    *calLrectLkf_R_camLrectKf_imu);
      } else {
        // 3-point RANSAC.
        statusPoseStereo = tracker_.geometricOutlierRejectionStereo(
                    *stereoFrame_lkf_, *stereoFrame_k_);
      }

      // Set relative pose.
      trackerStatusSummary_.kfTrackingStatus_stereo_ = statusPoseStereo.first;
      trackerStatusSummary_.infoMatStereoTranslation_ = infoMatStereoTranslation;

      if (VLOG_IS_ON(2)) logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_, "stereo");

      if (statusPoseStereo.first == Tracker::TrackingStatus::VALID) {
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
      // cv::waitKey(0);
    }

    // Populate statistics.
    tracker_.checkStatusRightKeypoints(stereoFrame_k_->right_keypoints_status_);

    // Move on.
    last_landmark_count_ = tracker_.landmark_count_;
    stereoFrame_lkf_ = stereoFrame_k_;

    // Get relevant info for keyframe.
    start_time = UtilsOpenCV::GetTimeInSeconds();
    smartStereoMeasurements = getSmartStereoMeasurements(*stereoFrame_k_.get());
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
  return std::make_pair(trackerStatusSummary_, smartStereoMeasurements);
}

/* -------------------------------------------------------------------------- */
SmartStereoMeasurements StereoVisionFrontEnd::getSmartStereoMeasurements(
    const StereoFrame& stereoFrame_kf) const {
  // Sanity check first.
  CHECK(stereoFrame_kf.isRectified())
      << "getSmartStereoMeasurements: stereo pair is not rectified";
  // Checks dimensionality of the feature vectors.
  stereoFrame_kf.checkStereoFrame();

  // Extract relevant info from the stereo frame:
  // essentially the landmark if and the left/right pixel measurements
  LandmarkIds landmarkId_kf = stereoFrame_kf.getLeftFrame().landmarks_;
  KeypointsCV leftKeypoints = stereoFrame_kf.left_keypoints_rectified_;
  KeypointsCV rightKeypoints = stereoFrame_kf.right_keypoints_rectified_;
  std::vector<Kstatus> rightKeypoints_status = stereoFrame_kf.right_keypoints_status_;

  // Pack information in landmark structure structure
  SmartStereoMeasurements smartStereoMeasurements;
  for (size_t i = 0; i < landmarkId_kf.size(); ++i) {
    if (landmarkId_kf.at(i) == -1) {
      continue; // skip invalid points
    }

    double uL = leftKeypoints.at(i).x; // TODO implicit conversion float to double increases floating-point precision!
    double v = leftKeypoints.at(i).y; // TODO implicit conversion float to double increases floating-point precision!
    double uR; // TODO Initialize...
    if (!tracker_.trackerParams_.useStereoTracking_) {
      LOG(WARNING) << "getSmartStereoMeasurements: dropping stereo information!"
                      " (set useStereoTracking_ = true to use it)";
    }
    if (tracker_.trackerParams_.useStereoTracking_ &&
        rightKeypoints_status.at(i) == Kstatus::VALID) {
      uR = rightKeypoints.at(i).x; // TODO implicit conversion float to double increases floating-point precision!
    } else {
      uR = std::numeric_limits<double>::quiet_NaN(); // missing pixel information
    }
    gtsam::StereoPoint2 stereo_px(uL,uR,v);

    smartStereoMeasurements.push_back(std::make_pair(landmarkId_kf[i],stereo_px));
  }
  return smartStereoMeasurements;
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayStereoTrack(const int& verbosity) const {
  const Frame& left_frame_k = stereoFrame_k_->getLeftFrame();

  // Show current frame with tracking results
  // The output of the following is already a color image
  cv::Mat img_left = tracker_.displayFrame(stereoFrame_lkf_->getLeftFrame(),
                                           left_frame_k, 0); // 0 verbosity

  //############################################################################
  displaySaveImage(img_left,
                   "",
                   "mono tracking visualization (1 frame)",
                   "-monoMatching1frame",
                   "monoTrackerDisplay1Keyframe_",
                   verbosity);
  //############################################################################

  // Draw the matchings: assumes that keypoints in the left and right keyframe are ordered in the same way
  const Frame& right_frame_k (stereoFrame_k_->getRightFrame());

  if ((left_frame_k.img_.cols != right_frame_k.img_.cols) ||
      (left_frame_k.img_.rows != right_frame_k.img_.rows))
    throw std::runtime_error("displayStereoTrack: image dimension mismatch!");

  cv::Mat img_right;
  (right_frame_k.img_).copyTo(img_right);
  // stereoFrame_k_->keypoints_depth_

  std::vector<cv::DMatch> matches;
  if (left_frame_k.keypoints_.size() == right_frame_k.keypoints_.size()) {
    for (size_t i = 0; i < left_frame_k.keypoints_.size(); i++) {
      if (left_frame_k.landmarks_[i] != -1 &&
          stereoFrame_k_->right_keypoints_status_[i] == Kstatus::VALID) {
        matches.push_back(cv::DMatch(i, i, 0));
      }
    }
  } else {
    VLOG(10) << "No matches to show, since we did not compute keypoints "
                "in right frame.";
  }

  //############################################################################
  // Plot matches.
  cv::Mat img_left_right = UtilsOpenCV::DrawCornersMatches(
    img_left, left_frame_k.keypoints_,
    img_right, right_frame_k.keypoints_, matches, true); // true: random color

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
      stereoFrame_k_->left_img_rectified_, stereoFrame_k_->left_keypoints_rectified_,
      stereoFrame_k_->right_img_rectified_, stereoFrame_k_->right_keypoints_rectified_, matches, true); // true: random color

  displaySaveImage(img_left_right_rectified,
                   "S(Rect):" + std::to_string(keyframe_count_),
                   "stereo tracking visualization (rectified)",
                   "-stereoMatchingRectified",
                   "stereoTrackerDisplayKeyframe_",
                   verbosity);
  //############################################################################
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayMonoTrack(const int& verbosity) const {
  const Frame& cur_left_frame = stereoFrame_k_->getLeftFrame();
  const Frame& ref_left_frame = stereoFrame_lkf_->getLeftFrame();

  // Find keypoint matches.
  std::vector<cv::DMatch> matches;
  for (size_t i = 0; i < cur_left_frame.keypoints_.size(); ++i) {
    if (cur_left_frame.landmarks_.at(i) != -1) {// if landmark is valid
      auto it = find(ref_left_frame.landmarks_.begin(),
                     ref_left_frame.landmarks_.end(),
                     cur_left_frame.landmarks_.at(i));
      if (it != ref_left_frame.landmarks_.end()) {// if landmark was found
        int nPos = distance(ref_left_frame.landmarks_.begin(), it);
        matches.push_back(cv::DMatch(nPos, i, 0));
      }
    }
  }

  //############################################################################
  // Plot matches.
  cv::Mat img_left_lkf_kf = UtilsOpenCV::DrawCornersMatches(
        ref_left_frame.img_,
        ref_left_frame.keypoints_,
        cur_left_frame.img_,
        cur_left_frame.keypoints_,
        matches, true); // true: random color

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
        matches, true); // true: random color

  // TODO Visualization must be done in the main thread.
  displaySaveImage(img_left_lkf_kf_rectified,
                   "M(Rect):" + std::to_string(keyframe_count_ - 1) + "-" +
                   std::to_string(keyframe_count_),
                   "mono tracking visualization (rectified)",
                   "-monoMatchingRectified",
                   "monoTrackerDispalyKeyframe_",
                   verbosity);
  //############################################################################
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - MONO RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyMono() const {
  // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ * kCamera_T_kBody =
  // body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
  gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->getBPoseCamLRect(); // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_mono_ * body_Pose_cam_.inverse();
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - STEREO RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyStereo() const {
  gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->getBPoseCamLRect(); // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_stereo_ * body_Pose_cam_.inverse();
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
                KeypointCV(10, 15), CV_FONT_HERSHEY_COMPLEX, 0.6,
                cv::Scalar(0, 255, 0));
  }
  if (verbosity == 1) { // otherwise just return the image
    cv::imshow(imshow_name, img_left);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  } else if (verbosity == 2) {
    // Create output folders:
    std::string folderName = outputImagesPath_ + "-" +
        VioFrontEndParams::FeatureSelectionCriterionStr(
          tracker_.trackerParams_.featureSelectionCriterion_)
        + folder_name_append + "/";
    boost::filesystem::path tracker_dir(folderName.c_str());
    boost::filesystem::create_directory(tracker_dir);
    // Write image.
    std::string img_name = folderName + img_name_prepend  +
        std::to_string(stereoFrame_lkf_->getLeftFrame().id_) + ".png";
    LOG(INFO) << "Writing image: " << img_name;
    cv::imwrite(img_name, img_left);
  }
}

} // End of VIO namespace.

