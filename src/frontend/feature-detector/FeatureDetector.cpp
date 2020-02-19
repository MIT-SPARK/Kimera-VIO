/**
 * @file   FeatureDetector.cpp
 * @brief  Base class for feature detector interface
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"

#include <algorithm>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"  // Just for ExtractCorners...

#include "kimera-vio/frontend/feature-detector/anms/anms.h"  // REMOVE

#include <numeric>

namespace VIO {

FeatureDetector::FeatureDetector(const VisionFrontEndParams& tracker_params)
    : tracker_params_(tracker_params),
      non_max_suppression_(nullptr),
      feature_detector_() {
  // TODO(Toni): parametrize actual non max suppression used (and whether we use
  // bucketing or anms)
  non_max_suppression_ =
      VIO::make_unique<AdaptiveNonMaximumSuppression>(AnmsAlgorithmType::Ssc);

  // Fast threshold. Usually this value is set to be in range [10,35]
  int fastThresh = 1;
  feature_detector_ = cv::FastFeatureDetector::create(fastThresh, true);
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
  int nr_corners_needed =
      std::max(tracker_params_.maxFeaturesPerFrame_ - n_existing, 0);
  debug_info_.need_n_corners_ = nr_corners_needed;

  ///////////////// FEATURE DETECTION //////////////////////
  // Actual feature detection: detects new keypoints where there are no
  // currently tracked ones
  auto start_time_tic = utils::Timer::tic();
  const KeypointsCV& corners = featureDetection(*cur_frame, nr_corners_needed);
  const size_t& n_corners = corners.size();

  debug_info_.featureDetectionTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.extracted_corners_ = n_corners;

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
             << "  (max: " << tracker_params_.maxFeaturesPerFrame_ << ")";
  } else {
    LOG(WARNING) << "No corners extracted for frame with id: "
                 << cur_frame->id_;
  }
}

KeypointsCV FeatureDetector::featureDetection(const Frame& cur_frame,
                                              const int& need_n_corners) {
  // TODO(TONI) need to do grid based approach!

  //cv::namedWindow("Input Image", cv::WINDOW_AUTOSIZE);
  //cv::imshow("Input Image", cur_frame.img_);

  // The mask is interpreted as: 255 -> consider, 0 -> don't consider.
  cv::Mat mask (cur_frame.img_.size(), CV_8U, cv::Scalar(255));
  for (size_t i = 0u; i < cur_frame.keypoints_.size(); ++i) {
    if (cur_frame.landmarks_.at(i) != -1) {
      // Only mask keypoints that are being triangulated (I guess
      // feature tracks? should be made more explicit)
      cv::circle(mask,
                 cur_frame.keypoints_.at(i),
                 tracker_params_.min_distance_,
                 cv::Scalar(0),
                 CV_FILLED);
    }
  }

  std::vector<cv::KeyPoint> keyPoints;  // vector to keep detected KeyPoints
  feature_detector_->detect(cur_frame.img_, keyPoints, mask);
  VLOG(1) << "Number of points detected : " << keyPoints.size();

  //cv::Mat fastDetectionResults;  // draw FAST detections
  //cv::drawKeypoints(cur_frame.img_,
  //                  keyPoints,
  //                  fastDetectionResults,
  //                  cv::Scalar(94.0, 206.0, 165.0, 0.0));
  //cv::namedWindow("FAST Detections", cv::WINDOW_AUTOSIZE);
  //cv::imshow("FAST Detections", fastDetectionResults);
  //cv::imshow("MASK Detections", mask);
  //cv::waitKey(0);

  VLOG(1) << "Need n corners: " << need_n_corners;
  int numRetPoints = need_n_corners;
  // int numRetPoints = 750;  // choose exact number of return points
  // float percentage = 0.1; //or choose percentage of points to be return
  // int numRetPoints = (int)keyPoints.size()*percentage;
  float tolerance = 0.1;  // tolerance of the number of return points
  std::vector<cv::KeyPoint>
      sscKP = non_max_suppression_->suppressNonMax(keyPoints,
                                                   numRetPoints,
                                                   tolerance,
                                                   cur_frame.img_.cols,
                                                   cur_frame.img_.rows);

  // Find new features.
  KeypointsCV new_corners;
  new_corners.reserve(sscKP.size());
  for (const cv::KeyPoint& kp : sscKP) {
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
    if (tracker_params_.enable_subpixel_corner_finder_) {
      const auto& subpixel_params =
          tracker_params_.subpixel_corner_finder_params_;
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
