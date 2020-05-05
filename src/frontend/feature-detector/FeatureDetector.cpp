/**
 * @file   FeatureDetector.cpp
 * @brief  Base class for feature detector interface
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"

#include <algorithm>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"  // Just for ExtractCorners...

#include <numeric>

namespace VIO {

FeatureDetector::FeatureDetector(
    const FeatureDetectorParams& feature_detector_params)
    : feature_detector_params_(feature_detector_params),
      non_max_suppression_(nullptr),
      feature_detector_() {
  // TODO(Toni): parametrize as well whether we use bucketing or anms...
  // Right now we asume we want anms not bucketing...
  if (feature_detector_params.enable_non_max_suppression_) {
    non_max_suppression_ = VIO::make_unique<AdaptiveNonMaximumSuppression>(
                             feature_detector_params.non_max_suppression_type_);
  }

  // TODO(Toni): find a way to pass params here using args lists
  switch (feature_detector_params.feature_detector_type_) {
    case FeatureDetectorType::FAST: {
      // Fast threshold, usually in range [10, 35]
      feature_detector_ = cv::FastFeatureDetector::create(
          feature_detector_params.fast_thresh_, true);
      break;
    }
    case FeatureDetectorType::ORB: {
      static constexpr float scale_factor = 1.2f;
      static constexpr int n_levels = 8;
      static constexpr int edge_threshold =
          10;  // Very small bcs we don't use descriptors (yet).
      static constexpr int first_level = 0;
      static constexpr int WTA_K = 0;  // We don't use descriptors (yet).
      static constexpr cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE;
      static constexpr int patch_size = 0;  // We don't use descriptors (yet).
      feature_detector_ =
          cv::ORB::create(feature_detector_params_.max_features_per_frame_,
                          scale_factor,
                          n_levels,
                          edge_threshold,
                          first_level,
                          WTA_K,
                          score_type,
                          patch_size,
                          feature_detector_params.fast_thresh_);
      break;
    }
    case FeatureDetectorType::AGAST: {
      LOG(FATAL) << "AGAST feature detector not implemented.";
      break;
    }
    case FeatureDetectorType::GFTT: {
      // goodFeaturesToTrack detector.
      feature_detector_ = cv::GFTTDetector::create(
          feature_detector_params_.max_features_per_frame_,
          feature_detector_params_.quality_level_,
          feature_detector_params_
              .min_distance_btw_tracked_and_detected_features_,
          feature_detector_params_.block_size_,
          feature_detector_params_.use_harris_corner_detector_,
          feature_detector_params_.k_);
      break;
    }
    default: {
      LOG(FATAL) << "Unknown feature detector type: "
                 << VIO::to_underlying(
                        feature_detector_params.feature_detector_type_);
    }
  }
}

// TODO(Toni) Optimize this function.
void FeatureDetector::featureDetection(Frame* cur_frame) {
  CHECK_NOTNULL(cur_frame);

  // Check how many new features we need: maxFeaturesPerFrame_ - n_existing
  // features If ref_frame has zero features this simply detects
  // maxFeaturesPerFrame_ new features for cur_frame
  int n_existing = 0;  // count existing (tracked) features
  for (size_t i = 0u; i < cur_frame->landmarks_.size(); ++i) {
    // count nr of valid keypoints
    if (cur_frame->landmarks_[i] != -1) ++n_existing;
    // features that have been tracked so far have Age+1
    cur_frame->landmarks_age_.at(i)++;
  }

  // Detect new features in image.
  // detect this much new corners if possible
  int nr_corners_needed = std::max(
      feature_detector_params_.max_features_per_frame_ - n_existing, 0);
  // debug_info_.need_n_corners_ = nr_corners_needed;

  ///////////////// FEATURE DETECTION //////////////////////
  // Actual feature detection: detects new keypoints where there are no
  // currently tracked ones
  auto start_time_tic = utils::Timer::tic();
  const KeypointsCV& corners = featureDetection(*cur_frame, nr_corners_needed);
  const size_t& n_corners = corners.size();

  // debug_info_.featureDetectionTime_ =
  // utils::Timer::toc(start_time_tic).count(); debug_info_.extracted_corners_ =
  // n_corners;

  if (n_corners > 0u) {
    ///////////////// STORE NEW KEYPOINTS  //////////////////////
    // Store features in our Frame
    const size_t& prev_nr_keypoints = cur_frame->keypoints_.size();
    const size_t& new_nr_keypoints = prev_nr_keypoints + n_corners;
    cur_frame->landmarks_.reserve(new_nr_keypoints);
    cur_frame->landmarks_age_.reserve(new_nr_keypoints);
    cur_frame->keypoints_.reserve(new_nr_keypoints);
    cur_frame->scores_.reserve(new_nr_keypoints);
    cur_frame->versors_.reserve(new_nr_keypoints);

    // Incremental id assigned to new landmarks
    static LandmarkId lmk_id = 0;
    const CameraParams& cam_param = cur_frame->cam_param_;
    for (const KeypointCV& corner : corners) {
      cur_frame->landmarks_.push_back(lmk_id);
      // New keypoint, so seen in a single (key)frame so far.
      cur_frame->landmarks_age_.push_back(1u);
      cur_frame->keypoints_.push_back(corner);
      cur_frame->scores_.push_back(0.0);  // NOT IMPLEMENTED
      cur_frame->versors_.push_back(Frame::calibratePixel(corner, cam_param));
      ++lmk_id;
    }
    VLOG(10) << "featureExtraction: frame " << cur_frame->id_
             << ",  Nr tracked keypoints: " << prev_nr_keypoints
             << ",  Nr extracted keypoints: " << n_corners
             << ",  total: " << cur_frame->keypoints_.size()
             << "  (max: " << feature_detector_params_.max_features_per_frame_
             << ")";
  } else {
    LOG(WARNING) << "No corners extracted for frame with id: "
                 << cur_frame->id_;
  }
}

KeypointsCV FeatureDetector::featureDetection(const Frame& cur_frame,
                                              const int& need_n_corners) {
  // TODO(TONI) need to do grid based approach!

  // cv::namedWindow("Input Image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Input Image", cur_frame.img_);

  // TODO(TONI): instead of using this mask, find all features,
  // do max-suppression, and then remove those detections that are close to
  // the already found ones, even you could cut feature tracks that are no
  // longer good quality or visible early on if they don't have detected
  // keypoints nearby by! The mask is interpreted as: 255 -> consider, 0 ->
  // don't consider.
  cv::Mat mask(cur_frame.img_.size(), CV_8U, cv::Scalar(255));
  for (size_t i = 0u; i < cur_frame.keypoints_.size(); ++i) {
    if (cur_frame.landmarks_.at(i) != -1) {
      // Only mask keypoints that are being triangulated (I guess
      // feature tracks? should be made more explicit)
      cv::circle(mask,
                 cur_frame.keypoints_.at(i),
                 feature_detector_params_
                     .min_distance_btw_tracked_and_detected_features_,
                 cv::Scalar(0),
                 CV_FILLED);
    }
  }

  std::vector<cv::KeyPoint> keypoints;  // vector to keep detected KeyPoints
  CHECK(feature_detector_);
  feature_detector_->detect(cur_frame.img_, keypoints, mask);
  VLOG(1) << "Number of points detected : " << keypoints.size();

  // cv::Mat fastDetectionResults;  // draw FAST detections
  // cv::drawKeypoints(cur_frame.img_,
  //                  keyPoints,
  //                  fastDetectionResults,
  //                  cv::Scalar(94.0, 206.0, 165.0, 0.0));
  // cv::namedWindow("FAST Detections", cv::WINDOW_AUTOSIZE);
  // cv::imshow("FAST Detections", fastDetectionResults);
  // cv::imshow("MASK Detections", mask);
  // cv::waitKey(0);

  VLOG(1) << "Need n corners: " << need_n_corners;
  // Tolerance of the number of returned points in percentage.
  std::vector<cv::KeyPoint>& max_keypoints = keypoints;
  if (non_max_suppression_) {
    static constexpr float tolerance = 0.1;
    max_keypoints =
      non_max_suppression_->suppressNonMax(keypoints,
                                           need_n_corners,
                                           tolerance,
                                           cur_frame.img_.cols,
                                           cur_frame.img_.rows);
  }
  // NOTE: if we don't use max_suppression we may end with more corners than
  // requested...

  // Find new features.
  // TODO(Toni): we should be using cv::KeyPoint... not cv::Point2f...
  KeypointsCV new_corners;
  new_corners.reserve(max_keypoints.size());
  for (const cv::KeyPoint& kp : max_keypoints) {
    new_corners.push_back(kp.pt);
  }
  // if (need_n_corners > 0) {
  //  UtilsOpenCV::ExtractCorners(cur_frame.img_,
  //                              &new_corners,
  //                              need_n_corners, // THIS IS MAX CORNERS not
  //                              tracker_params_.quality_level_,
  //                              tracker_params_.min_distance_,
  //                              tracker_params_.block_size_,
  //                              tracker_params_.k_,
  //                              tracker_params_.use_harris_detector_);

  // TODO this takes a ton of time 27ms each time...
  // Change window_size, and term_criteria to improve timing
  if (new_corners.size() > 0) {
    if (feature_detector_params_.enable_subpixel_corner_refinement_) {
      const auto& subpixel_params =
          feature_detector_params_.subpixel_corner_finder_params_;
      auto tic = utils::Timer::tic();
      cv::cornerSubPix(cur_frame.img_,
                       new_corners,
                       subpixel_params.window_size_,
                       subpixel_params.zero_zone_,
                       subpixel_params.term_criteria_);
      VLOG(1) << "Corner Sub Pixel Refinement Timing [ms]: "
              << utils::Timer::toc(tic).count();
    }
  }
  //}

  return new_corners;
}

}  // namespace VIO
