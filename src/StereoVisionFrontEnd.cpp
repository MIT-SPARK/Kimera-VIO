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
void StereoVisionFrontEnd::processFirstStereoFrame(StereoFrame& firstFrame) {
  VLOG(2) << "Processing first stereo frame \n";
  stereoFrame_k_ = std::make_shared<StereoFrame>(firstFrame);
  stereoFrame_k_->setIsKeyframe(true);
  last_keyframe_timestamp_ = stereoFrame_k_->timestamp_;

  // tracking is based on left frame
  Frame& left_frame = stereoFrame_k_->left_frame_;
  CHECK_EQ(left_frame.keypoints_.size(), 0)
    << "Keypoints already present in first frame: please do not extract"
       " keypoints manually";

  // initialize mask: this is allocated but it does not play a role in this function
  cv::Size imgSize = left_frame.img_.size();
  tracker_.camMask_ = cv::Mat(imgSize, CV_8UC1, cv::Scalar(255));

  // Perform feature detection
  tracker_.featureDetection(left_frame);

  // get 3D points via stereo
  stereoFrame_k_->sparseStereoMatching();

  // Prepare for next iteration
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_lkf_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;
}

/* -------------------------------------------------------------------------- */
StatusSmartStereoMeasurements StereoVisionFrontEnd::processStereoFrame(
    StereoFrame& cur_frame,
    boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu) {
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << cur_frame.timestamp_ << " empirical framerate (sec): "
          <<  UtilsOpenCV::NsecToSec(cur_frame.timestamp_ - stereoFrame_km1_->timestamp_)
           << " (timestamp diff: " <<  cur_frame.timestamp_ - stereoFrame_km1_->timestamp_ << ")";

  // only for visualization
  int verbosityFrames = saveImages_; // default: 0
  int verbosityKeyframes = saveImages_; // default: 1

  double timeBefore = 0, timeClone = 0, timeSparseStereo = 0, timeGetMeasurements = 0;

  timeBefore = UtilsOpenCV::GetTimeInSeconds();
  stereoFrame_k_ = std::make_shared<StereoFrame>(cur_frame);
  // copy rectification from previous frame to avoid recomputing it:
  stereoFrame_k_->cloneRectificationParameters(*stereoFrame_km1_);
  timeClone = UtilsOpenCV::GetTimeInSeconds() - timeBefore;

  // Track features from the previous frame
  Frame& left_frame_km1 = stereoFrame_km1_->left_frame_;
  Frame& left_frame_k = stereoFrame_k_->left_frame_;
  tracker_.featureTracking(left_frame_km1, left_frame_k);
  if (verbosityFrames > 0) {
    tracker_.displayFrame(left_frame_km1, left_frame_k, verbosityFrames);
  }

  // Not tracking at all in this phase.
  trackerStatusSummary_.kfTrackingStatus_mono_ = Tracker::TrackingStatus::INVALID;
  trackerStatusSummary_.kfTrackingStatus_stereo_ = Tracker::TrackingStatus::INVALID;

  // This will be the info we actually care about
  SmartStereoMeasurements smartStereoMeasurements;
  // smartStereoMeasurements.clear();

  const bool maxTimeElapsed = UtilsOpenCV::NsecToSec(
              stereoFrame_k_->timestamp_ - last_keyframe_timestamp_) >=
                        tracker_.trackerParams_.intra_keyframe_time_;
  const int nrValidFeatures = left_frame_k.getNrValidKeypoints();
  const bool nrFeaturesIsLow = nrValidFeatures <=
                         tracker_.trackerParams_.minNumberFeatures_;
  if (maxTimeElapsed || nrFeaturesIsLow) {
    ++keyframe_count_; // mainly for debugging

    VLOG(2) << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << "Keyframe after: "
            << UtilsOpenCV::NsecToSec(stereoFrame_k_->timestamp_ - last_keyframe_timestamp_) << " sec.";

    VLOG_IF(2, maxTimeElapsed) << "Keyframe reason: max time elapsed.";
    VLOG_IF(2, nrFeaturesIsLow) << "Keyframe reason: low nr of features ("
                                << nrValidFeatures << " < "
                                << tracker_.trackerParams_.minNumberFeatures_ <<").";

    if (!tracker_.trackerParams_.useRANSAC_) {
      trackerStatusSummary_.kfTrackingStatus_mono_ = Tracker::TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_mono_, "mono");

      trackerStatusSummary_.kfTrackingStatus_stereo_ = Tracker::TrackingStatus::DISABLED;

      if (VLOG_IS_ON(2)) logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_, "stereo");
    } else {
      ////////////////// MONO geometric outlier rejection ////////////////
      std::pair<Tracker::TrackingStatus,gtsam::Pose3> statusPoseMono;
      Frame& left_frame_lkf = stereoFrame_lkf_->left_frame_;
      if (tracker_.trackerParams_.ransac_use_2point_mono_ &&
          calLrectLkf_R_camLrectKf_imu) {
        // 2-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMonoGivenRotation(
            left_frame_lkf, left_frame_k, *calLrectLkf_R_camLrectKf_imu);
      } else {
        // 5-point RANSAC.
        statusPoseMono = tracker_.geometricOutlierRejectionMono(left_frame_lkf, left_frame_k);
      }

      // Set relative pose.
      trackerStatusSummary_.kfTrackingStatus_mono_ = statusPoseMono.first;

      if (VLOG_IS_ON(2)) logTrackingStatus(trackerStatusSummary_.kfTrackingStatus_stereo_, "mono");

      if (statusPoseMono.first == Tracker::TrackingStatus::VALID) {
          trackerStatusSummary_.lkf_T_k_mono_ = statusPoseMono.second;
      }

      if (verbosityFrames > 0) {
          tracker_.displayFrame(left_frame_km1, left_frame_k, verbosityFrames);
      }

      ////////////////// STEREO geometric outlier rejection ////////////////
      // get 3D points via stereo
      timeBefore = UtilsOpenCV::GetTimeInSeconds();
      stereoFrame_k_->sparseStereoMatching();
      timeSparseStereo = UtilsOpenCV::GetTimeInSeconds() - timeBefore;

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
    last_keyframe_timestamp_ = stereoFrame_k_->timestamp_;
    stereoFrame_k_->isKeyframe_ = true;

    // Perform feature detection (note: this must be after RANSAC,
    // since if we discard more features, we need to extract more)
    tracker_.featureDetection(left_frame_k);

    // get 3D points via stereo, including newly extracted
    // (this might be only for the visualization)
    timeBefore = UtilsOpenCV::GetTimeInSeconds();
    stereoFrame_k_->sparseStereoMatching();
    timeSparseStereo += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

    // show results
    if (verbosityKeyframes > 0) {
      displayStereoTrack(verbosityKeyframes);
      displayMonoTrack(verbosityKeyframes);
    }

    // populate statistics
    tracker_.checkStatusRightKeypoints(stereoFrame_k_->right_keypoints_status_);

    // move on
    last_landmark_count_ = tracker_.landmark_count_;
    stereoFrame_lkf_ = stereoFrame_k_;

    // get relevant info for keyframe
    timeBefore = UtilsOpenCV::GetTimeInSeconds();
    smartStereoMeasurements = getSmartStereoMeasurements(*stereoFrame_k_.get());
    timeGetMeasurements = UtilsOpenCV::GetTimeInSeconds() - timeBefore;

    VLOG(2) << "timeClone: " << timeClone << '\n'
            << "timeSparseStereo: " << timeSparseStereo << '\n'
            << "timeGetMeasurements: " << timeGetMeasurements;
  } else {
    stereoFrame_k_->isKeyframe_ = false;
  }

  // Reset frames
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;
  return std::make_pair(trackerStatusSummary_,smartStereoMeasurements);
}

/* -------------------------------------------------------------------------- */
SmartStereoMeasurements StereoVisionFrontEnd::getSmartStereoMeasurements(
    const StereoFrame& stereoFrame_kf) const {
  // sanity check first
  if(!stereoFrame_kf.isRectified_)
    throw std::runtime_error("getSmartStereoMeasurements: stereo pair is not rectified");
  stereoFrame_kf.checkStereoFrame(); // checks dimensionality of the feature vectors

  // extract relevant info from the stereo frame: essentially the landmark if and the left/right pixel measurements
  LandmarkIds landmarkId_kf = stereoFrame_kf.left_frame_.landmarks_;
  KeypointsCV leftKeypoints = stereoFrame_kf.left_keypoints_rectified_;
  KeypointsCV rightKeypoints = stereoFrame_kf.right_keypoints_rectified_;
  std::vector<Kstatus> rightKeypoints_status = stereoFrame_kf.right_keypoints_status_;

  // Pack information in // Pack information in landmark structure structure
  SmartStereoMeasurements smartStereoMeasurements;
  for (size_t i = 0; i < landmarkId_kf.size(); ++i)
  {
    if(landmarkId_kf.at(i)==-1)
      continue; // skip invalid points

    double uL = leftKeypoints.at(i).x;
    double v = leftKeypoints.at(i).y;
    double uR;
    if(!tracker_.trackerParams_.useStereoTracking_) { std::cout << "getSmartStereoMeasurements: dropping stereo information! (set useStereoTracking_ = true to use it)" << std::endl; }
    if(tracker_.trackerParams_.useStereoTracking_ && rightKeypoints_status.at(i) == Kstatus::VALID){
      uR = rightKeypoints.at(i).x;
    }else{
      uR = std::numeric_limits<double>::quiet_NaN(); // missing pixel information
    }
    gtsam::StereoPoint2 stereo_px(uL,uR,v);

    smartStereoMeasurements.push_back(std::make_pair(landmarkId_kf[i],stereo_px));
  }
  return smartStereoMeasurements;
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayStereoTrack(const int& verbosity) const {
  Frame& left_frame_k = stereoFrame_k_->left_frame_;

  // Show current frame with tracking results
  // The output of the following is already a color image
  cv::Mat img_left = tracker_.displayFrame(stereoFrame_lkf_->left_frame_, left_frame_k, 0); // 0 verbosity

  if(verbosity==1){ // otherwise just return the image
    cv::imshow("mono tracking visualization (1 frame)", img_left);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  }
  if(verbosity==2){
    // create output folders:
    std::string folderName = outputImagesPath_ + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(tracker_.trackerParams_.featureSelectionCriterion_) + + "-monoMatching1frame/";
    boost::filesystem::path monoTrackerDir(folderName.c_str());
    boost::filesystem::create_directory(monoTrackerDir);
    // write image
    std::string img_name = folderName + "monoTrackerDisplay1Keyframe_"  + std::to_string(stereoFrame_lkf_->left_frame_.id_) + ".png";
    std::cout << "writing image: " << img_name << std::endl;
    cv::imwrite(img_name, img_left);
  }

  // Draw the matchings: assumes that keypoints in the left and right keyframe are ordered in the same way
  Frame& right_frame_k = stereoFrame_k_->right_frame_;

  if( (left_frame_k.img_.cols != right_frame_k.img_.cols) || (left_frame_k.img_.rows != right_frame_k.img_.rows) )
    throw std::runtime_error("displayStereoTrack: image dimension mismatch!");

  cv::Mat img_right;
  (right_frame_k.img_).copyTo(img_right);
  // stereoFrame_k_->keypoints_depth_

  std::vector<cv::DMatch> matches;
  if(left_frame_k.keypoints_.size() == right_frame_k.keypoints_.size()){
    for (int i = 0; i < left_frame_k.keypoints_.size(); i++) {
      if(left_frame_k.landmarks_[i] != -1 && stereoFrame_k_->right_keypoints_status_[i] == Kstatus::VALID)
        matches.push_back(cv::DMatch(i, i, 0));
    }
  }else{
#ifdef STEREO_TRACKER_DEBUG_COUT
    std::cout << "no matches to show, since we did not compute keypoints in right frame" << std::endl;
#endif
  }

  // plot matches
  cv::Mat img_left_right = UtilsOpenCV::DrawCornersMatches(
      img_left, left_frame_k.keypoints_, img_right, right_frame_k.keypoints_, matches, true); // true: random color

  // plot text with keyframe id
  cv::putText(img_left_right, "S:" + std::to_string(keyframe_count_),
          KeypointCV(10,15), CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0));

  if(verbosity==1){ // otherwise just return the image
    cv::imshow("stereo tracking visualization", img_left_right);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  }
  if(verbosity==2){
    // create output folders:
    std::string folderName = outputImagesPath_ + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(tracker_.trackerParams_.featureSelectionCriterion_) + + "-stereoMatchingUnrectified/";
    boost::filesystem::path stereoTrackerDir(folderName.c_str());
    boost::filesystem::create_directory(stereoTrackerDir);
    // write image
    std::string img_name = folderName + "StereoTrackerDisplayKeyframe_"  + std::to_string(stereoFrame_lkf_->left_frame_.id_) + ".png";
    std::cout << "writing image: " << img_name << std::endl;
    cv::imwrite(img_name, img_left_right);
  }

  //////////////////////////////////////////////////////////////////////////////////
  // display rectified
  // plot matches

  cv::Mat img_left_right_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_k_->left_img_rectified_, stereoFrame_k_->left_keypoints_rectified_,
      stereoFrame_k_->right_img_rectified_, stereoFrame_k_->right_keypoints_rectified_, matches, true); // true: random color

  // plot text with keyframe id
  cv::putText(img_left_right_rectified, "S(Rect):" + std::to_string(keyframe_count_),
      KeypointCV(10,15), CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0));

  if(verbosity==1){ // otherwise just return the image
    cv::imshow("stereo tracking visualization (rectified)", img_left_right_rectified);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  }
  if(verbosity==2){
    // create output folders:
    std::string folderName = outputImagesPath_ + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(tracker_.trackerParams_.featureSelectionCriterion_) + "-stereoMatchingRectified/";
    boost::filesystem::path monoTrackerDir(folderName.c_str());
    boost::filesystem::create_directory(monoTrackerDir);
    // write image
    std::string img_name = folderName + "stereoTrackerDisplayKeyframe_"  + std::to_string(stereoFrame_lkf_->left_frame_.id_) + ".png";
    std::cout << "writing image: " << img_name << std::endl;
    cv::imwrite(img_name, img_left_right_rectified);
  }
}

/* -------------------------------------------------------------------------- */
void StereoVisionFrontEnd::displayMonoTrack(const int& verbosity) const {
  Frame& cur_left_frame = stereoFrame_k_->left_frame_;
  Frame& ref_left_frame = stereoFrame_lkf_->left_frame_;

  // find keypoint matches
  std::vector<cv::DMatch> matches;
  for (size_t i = 0; i < cur_left_frame.keypoints_.size(); ++i)
  {
    if (cur_left_frame.landmarks_.at(i) != -1) // if landmark is valid
    {
      auto it = find(ref_left_frame.landmarks_.begin(), ref_left_frame.landmarks_.end(), cur_left_frame.landmarks_.at(i));
      if (it != ref_left_frame.landmarks_.end()) // if landmark was found
      {
        int nPos = distance(ref_left_frame.landmarks_.begin(), it);
        matches.push_back(cv::DMatch(nPos, i, 0));
      }
    }
  }

  // plot matches
  cv::Mat img_left_lfk_kf = UtilsOpenCV::DrawCornersMatches(
      ref_left_frame.img_, ref_left_frame.keypoints_, cur_left_frame.img_, cur_left_frame.keypoints_, matches, true); // true: random color

  // plot text with keyframe id
  cv::putText(img_left_lfk_kf, "M:" + std::to_string(keyframe_count_-1) + "-" + std::to_string(keyframe_count_),
          KeypointCV(10,15), CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0));

  if(verbosity==1){ // otherwise just return the image
    cv::imshow("mono tracking visualization", img_left_lfk_kf);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  }
  if(verbosity==2){
    // create output folders:
    std::string folderName = outputImagesPath_ + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(tracker_.trackerParams_.featureSelectionCriterion_) + "-monoMatchingUnrectified/";
    boost::filesystem::path monoTrackerDir(folderName.c_str());
    boost::filesystem::create_directory(monoTrackerDir);
    // write image
    std::string img_name = folderName + "monoTrackerDisplayKeyframe_"  + std::to_string(stereoFrame_lkf_->left_frame_.id_) + ".png";
    std::cout << "writing image: " << img_name << std::endl;
    cv::imwrite(img_name, img_left_lfk_kf);
  }

  //////////////////////////////////////////////////////////////////////////////////
  // display rectified
  // plot matches

  cv::Mat img_left_lfk_kf_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_lkf_->left_img_rectified_, stereoFrame_lkf_->left_keypoints_rectified_,
      stereoFrame_k_->left_img_rectified_, stereoFrame_k_->left_keypoints_rectified_, matches, true); // true: random color

  // plot text with keyframe id
  cv::putText(img_left_lfk_kf_rectified, "M(Rect):" + std::to_string(keyframe_count_-1) + "-" + std::to_string(keyframe_count_),
      KeypointCV(10,15), CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0));

  if(verbosity==1){ // otherwise just return the image
    cv::imshow("mono tracking visualization (rectified)", img_left_lfk_kf_rectified);
    cv::waitKey(tracker_.trackerParams_.display_time_);
  }
  if(verbosity==2){
    // create output folders:
    std::string folderName = outputImagesPath_ + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(tracker_.trackerParams_.featureSelectionCriterion_) + "-monoMatchingRectified/";
    boost::filesystem::path monoTrackerDir(folderName.c_str());
    boost::filesystem::create_directory(monoTrackerDir);
    // write image
    std::string img_name = folderName + "monoTrackerDisplayKeyframe_"  + std::to_string(stereoFrame_lkf_->left_frame_.id_) + ".png";
    std::cout << "writing image: " << img_name << std::endl;
    cv::imwrite(img_name, img_left_lfk_kf_rectified);
  }
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - MONO RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyMono() const {
  // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ * kCamera_T_kBody =
  // body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
  gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->B_Pose_camLrect_; // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_mono_ * body_Pose_cam_.inverse();
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - STEREO RANSAC
gtsam::Pose3 StereoVisionFrontEnd::getRelativePoseBodyStereo() const {
  gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->B_Pose_camLrect_; // of the left camera!!
  return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_stereo_ * body_Pose_cam_.inverse();
}

} // End of VIO namespace.

