/**
 * @file   Tracker.cpp
 * @brief  Class describing temporal tracking
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/frontend/Tracker.h"

#include <time.h>
#include <algorithm>   // for sort
#include <functional>  // for less<>
#include <map>         // for map<>
#include <memory>      // for shared_ptr<>
#include <string>
#include <utility>  // for pair<>
#include <vector>   // for vector<>

#include <boost/shared_ptr.hpp>  // used for opengv

#include "kimera-vio/frontend/OpticalFlowPredictorFactory.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

Tracker::Tracker(const FrontendParams& tracker_params,
                 const CameraParams& camera_params)
    : landmark_count_(0),
      tracker_params_(tracker_params),
      camera_params_(camera_params),
      // Only for debugging and visualization:
      optical_flow_predictor_(nullptr),
      output_images_path_("./outputImages/") {
  optical_flow_predictor_ =
      OpticalFlowPredictorFactory::makeOpticalFlowPredictor(
          tracker_params_.optical_flow_predictor_type_,
          camera_params_.K_,
          camera_params_.image_size_);
}

// TODO(Toni) Optimize this function.
void Tracker::featureDetection(Frame* cur_frame) {
  CHECK_NOTNULL(cur_frame);
  // Check how many new features we need: maxFeaturesPerFrame_ - n_existing
  // features If ref_frame has zero features this simply detects
  // maxFeaturesPerFrame_ new features for cur_frame
  int n_existing = 0;  // count existing (tracked) features
  for (size_t i = 0; i < cur_frame->landmarks_.size(); ++i) {
    // count nr of valid keypoints
    if (cur_frame->landmarks_.at(i) != -1) ++n_existing;
    // features that have been tracked so far have Age+1
    cur_frame->landmarks_age_.at(i)++;
  }

  // Detect new features in image.
  // detect this much new corners if possible
  int nr_corners_needed =
      std::max(tracker_params_.maxFeaturesPerFrame_ - n_existing, 0);
  debug_info_.need_n_corners_ = nr_corners_needed;

  ///////////////// FEATURE DETECTION //////////////////////
  // If feature FeatureSelectionCriterion is quality, just extract what you
  // need:
  auto start_time_tic = utils::Timer::tic();
  const KeypointsWithScores& corners_with_scores =
      Tracker::featureDetection(*cur_frame, cam_mask_, nr_corners_needed);

  debug_info_.featureDetectionTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.extracted_corners_ = corners_with_scores.first.size();

  ///////////////// STORE NEW KEYPOINTS  //////////////////////
  // Store features in our Frame
  size_t nrExistingKeypoints =
      cur_frame->keypoints_.size();  // for debug, these are the ones tracked
                                     // from the previous frame
  cur_frame->landmarks_.reserve(cur_frame->keypoints_.size() +
                                corners_with_scores.first.size());
  cur_frame->landmarks_age_.reserve(cur_frame->keypoints_.size() +
                                    corners_with_scores.first.size());
  cur_frame->keypoints_.reserve(cur_frame->keypoints_.size() +
                                corners_with_scores.first.size());
  cur_frame->scores_.reserve(cur_frame->scores_.size() +
                             corners_with_scores.second.size());
  cur_frame->versors_.reserve(cur_frame->keypoints_.size() +
                              corners_with_scores.first.size());

  // TODO(Toni) Fix this loop, very unefficient. Use std::move over keypoints
  // with scores.
  // Counters.
  for (size_t i = 0; i < corners_with_scores.first.size(); i++) {
    cur_frame->landmarks_.push_back(landmark_count_);
    cur_frame->landmarks_age_.push_back(1);  // seen in a single (key)frame
    cur_frame->keypoints_.push_back(corners_with_scores.first.at(i));
    cur_frame->scores_.push_back(corners_with_scores.second.at(i));
    cur_frame->versors_.push_back(Frame::calibratePixel(
        corners_with_scores.first.at(i), cur_frame->cam_param_));
    ++landmark_count_;
  }
  VLOG(10) << "featureExtraction: frame " << cur_frame->id_
           << ",  Nr tracked keypoints: " << nrExistingKeypoints
           << ",  Nr extracted keypoints: " << corners_with_scores.first.size()
           << ",  total: " << cur_frame->keypoints_.size()
           << "  (max: " << tracker_params_.maxFeaturesPerFrame_ << ")";
}

KeypointsWithScores Tracker::featureDetection(const Frame& cur_frame,
                                              const cv::Mat& cam_mask,
                                              const int need_n_corners) {
  // Create mask such that new keypoints are not close to old ones.
  cv::Mat mask;
  cam_mask.copyTo(mask);
  for (size_t i = 0; i < cur_frame.keypoints_.size(); ++i) {
    if (cur_frame.landmarks_.at(i) != -1) {
      cv::circle(mask,
                 cur_frame.keypoints_.at(i),
                 tracker_params_.min_distance_,
                 cv::Scalar(0),
                 CV_FILLED);
    }
  }

  // Find new features and corresponding scores.
  KeypointsWithScores corners_with_scores;
  if (need_n_corners > 0) {
    MyGoodFeaturesToTrackSubPix(cur_frame.img_,
                                need_n_corners,
                                tracker_params_.quality_level_,
                                tracker_params_.min_distance_,
                                mask,
                                tracker_params_.block_size_,
                                tracker_params_.use_harris_detector_,
                                tracker_params_.k_,
                                &corners_with_scores);
  }

  return corners_with_scores;
}

// TODO(Toni): VIT this function should be optimized and cleaned.
// Use std::vector<std::pair<cv::Point2f, double>>
// Or  std::vector<std::pair<KeypointCV, Score>> but not a pair of vectors.
// Remove hardcoded parameters (there are a ton).
void Tracker::MyGoodFeaturesToTrackSubPix(
    const cv::Mat& image,
    const int& max_corners,
    const double& quality_level,
    const double& min_distance,
    const cv::Mat& mask,
    const int& block_size,
    const bool& use_harris_corners,
    const double& harrisK,
    KeypointsWithScores* corners_with_scores) {
  CHECK_NOTNULL(corners_with_scores);
  try {
    // Get image of cornerness response, and get peaks for good features.
    cv::Mat cornerness_response;
    if (use_harris_corners) {
      cv::cornerHarris(image, cornerness_response, block_size, 3, harrisK);
    } else {
      // Cornerness response corresponds to eigen values.
      cv::cornerMinEigenVal(image, cornerness_response, block_size, 3);
    }

    // Cut off corners below quality level.
    double max_val = 0;
    double min_val;
    cv::Point min_loc;
    cv::Point max_loc;
    cv::minMaxLoc(
        cornerness_response, &min_val, &max_val, &min_loc, &max_loc, mask);

    // Cut stuff below quality.
    cv::threshold(cornerness_response,
                  cornerness_response,
                  max_val * quality_level,
                  0,
                  CV_THRESH_TOZERO);
    cv::Mat tmp;
    cv::dilate(cornerness_response, tmp, cv::Mat());

    // Create corners.
    std::vector<std::pair<const float*, float>> tmp_corners_scores;

    // collect list of pointers to features - put them into temporary image
    const cv::Size& img_size = image.size();
    for (int y = 1; y < img_size.height - 1; y++) {
      const float* thresholded_cornerness =
          (const float*)cornerness_response.ptr(y);
      const float* dilated_cornerness = (const float*)tmp.ptr(y);
      const uchar* mask_data = mask.data ? mask.ptr(y) : nullptr;

      for (int x = 1; x < img_size.width - 1; x++) {
        const float& val = thresholded_cornerness[x];
        // TODO this takes a ton of time 12ms each time...
        if (val != 0 && val == dilated_cornerness[x] &&
            (!mask_data || mask_data[x])) {
          tmp_corners_scores.push_back(
              std::make_pair(thresholded_cornerness + x, val));
        }
      }
    }

    std::sort(tmp_corners_scores.begin(),
              tmp_corners_scores.end(),
              myGreaterThanPtr<float>());

    // Put sorted corner in other struct.
    size_t j;
    size_t total = tmp_corners_scores.size();
    size_t ncorners = 0;

    double min_distance_tmp = min_distance;
    if (min_distance_tmp >= 1) {
      // Partition the image into larger grids
      int w = image.cols;
      int h = image.rows;

      const int cell_size = cvRound(min_distance_tmp);
      const int grid_width = (w + cell_size - 1) / cell_size;
      const int grid_height = (h + cell_size - 1) / cell_size;

      std::vector<KeypointsCV> grid(grid_width * grid_height);

      min_distance_tmp *= min_distance_tmp;

      for (size_t i = 0; i < total; i++) {
        int ofs = (int)((const uchar*)tmp_corners_scores[i].first -
                        cornerness_response.data);
        int y = (int)(ofs / cornerness_response.step);
        int x = (int)((ofs - y * cornerness_response.step) / sizeof(float));
        double eigVal = double(tmp_corners_scores[i].second);

        bool good = true;

        int x_cell = x / cell_size;
        int y_cell = y / cell_size;

        int x1 = x_cell - 1;
        int y1 = y_cell - 1;
        int x2 = x_cell + 1;
        int y2 = y_cell + 1;

        // boundary check
        x1 = std::max(0, x1);
        y1 = std::max(0, y1);
        x2 = std::min(grid_width - 1, x2);
        y2 = std::min(grid_height - 1, y2);

        for (int yy = y1; yy <= y2; yy++) {
          for (int xx = x1; xx <= x2; xx++) {
            const KeypointsCV& m = grid[yy * grid_width + xx];

            if (m.size()) {
              for (j = 0; j < m.size(); j++) {
                const float& dx = x - m[j].x;
                const float& dy = y - m[j].y;

                if (dx * dx + dy * dy < min_distance_tmp) {
                  good = false;
                  goto break_out;
                }
              }
            }
          }
        }

      break_out:

        if (good) {
          // printf("%d: %d %d -> %d %d, %d, %d -- %d %d %d %d, %d %d, c=%d\n",
          //    i,x, y, x_cell, y_cell, (int)minDistance, cell_size,x1,y1,x2,y2,
          //    grid_width,grid_height,c);
          grid[y_cell * grid_width + x_cell].push_back(
              KeypointCV((float)x, (float)y));
          corners_with_scores->first.push_back(KeypointCV((float)x, (float)y));
          corners_with_scores->second.push_back(eigVal);
          ++ncorners;
          if (max_corners > 0 && (int)ncorners == max_corners) {
            break;
          }
        }
      }
    } else {
      for (size_t i = 0; i < total; i++) {
        int ofs = (int)((const uchar*)tmp_corners_scores[i].first -
                        cornerness_response.data);
        int y = (int)(ofs / cornerness_response.step);
        int x = (int)((ofs - y * cornerness_response.step) / sizeof(float));
        double eigVal = double(tmp_corners_scores[i].second);
        corners_with_scores->first.push_back(KeypointCV((float)x, (float)y));
        corners_with_scores->second.push_back(eigVal);
        ++ncorners;
        if (max_corners > 0 && (int)ncorners == max_corners) {
          break;
        }
      }
    }

    // subpixel accuracy: TODO: create function for the next 4 lines
    // TODO(Toni): REMOVE all these hardcoded stuff...
    static const cv::TermCriteria criteria(
        CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    static const cv::Size winSize(10, 10);
    static const cv::Size zeroZone(-1, -1);

    // TODO this takes a ton of time 27ms each time...
    cv::cornerSubPix(
        image, corners_with_scores->first, winSize, zeroZone, criteria);
  } catch (...) {
    // Corners remains empty.
    LOG(WARNING) << "ExtractCorners: no corner found in image.";
  }
}

// TODO(Toni) a pity that this function is not const just because
// it modifies debuginfo_...
void Tracker::featureTracking(Frame* ref_frame,
                              Frame* cur_frame,
                              const gtsam::Rot3& inter_frame_rotation) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);
  auto tic = utils::Timer::tic();

  // Fill up structure for reference pixels and their labels.
  const size_t& n_ref_kpts = ref_frame->keypoints_.size();
  KeypointsCV px_ref;
  std::vector<size_t> indices_of_valid_landmarks;
  px_ref.reserve(n_ref_kpts);
  indices_of_valid_landmarks.reserve(n_ref_kpts);
  for (size_t i = 0; i < ref_frame->keypoints_.size(); ++i) {
    if (ref_frame->landmarks_[i] != -1) {
      // Current reference frame keypoint has a valid landmark.
      px_ref.push_back(ref_frame->keypoints_[i]);
      indices_of_valid_landmarks.push_back(i);
    }
  }

  // Setup termination criteria for optical flow.
  const cv::TermCriteria kTerminationCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
      tracker_params_.klt_max_iter_,
      tracker_params_.klt_eps_);
  const cv::Size2i kKltWindowSize(tracker_params_.klt_win_size_,
                                  tracker_params_.klt_win_size_);

  // Initialize to old locations
  LOG_IF(ERROR, px_ref.size() == 0u) << "No keypoints in reference frame!";

  KeypointsCV px_cur;
  CHECK(optical_flow_predictor_->predictFlow(
      px_ref, inter_frame_rotation, &px_cur));

  // Do the actual tracking, so px_cur becomes the new pixel locations.
  VLOG(2) << "Sarting Optical Flow Pyr LK tracking...";

  std::vector<uchar> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(ref_frame->img_,
                           cur_frame->img_,
                           px_ref,
                           px_cur,
                           status,
                           error,
                           kKltWindowSize,
                           tracker_params_.klt_max_level_,
                           kTerminationCriteria,
                           cv::OPTFLOW_USE_INITIAL_FLOW);
  VLOG(2) << "Finished Optical Flow Pyr LK tracking.";

  // TODO(Toni): use the error to further take only the best tracks?

  // TODO(TONI): WTF is this doing? Are we always having empty keypoints??
  if (cur_frame->keypoints_.empty()) {
    // TODO(TOni): this is basically copying the whole px_ref into the
    // current frame as well as the ref_frame information! Absolute nonsense.
    cur_frame->landmarks_.reserve(px_ref.size());
    cur_frame->landmarks_age_.reserve(px_ref.size());
    cur_frame->keypoints_.reserve(px_ref.size());
    cur_frame->scores_.reserve(px_ref.size());
    cur_frame->versors_.reserve(px_ref.size());
    for (size_t i = 0; i < indices_of_valid_landmarks.size(); ++i) {
      // If we failed to track mark off that landmark
      const size_t idx_valid_lmk = indices_of_valid_landmarks[i];
      if (!status[i] ||
          // if we tracked keypoint and feature
          ref_frame->landmarks_age_[idx_valid_lmk] >
              tracker_params_.maxFeatureAge_) {
        // track is not too long
        // we are marking this bad in the ref_frame since features
        // in the ref frame guide feature detection later on
        ref_frame->landmarks_[idx_valid_lmk] = -1;
        continue;
      }
      cur_frame->landmarks_.push_back(ref_frame->landmarks_[idx_valid_lmk]);
      cur_frame->landmarks_age_.push_back(
          ref_frame->landmarks_age_[idx_valid_lmk]);
      cur_frame->scores_.push_back(ref_frame->scores_[idx_valid_lmk]);
      cur_frame->keypoints_.push_back(px_cur[i]);
      cur_frame->versors_.push_back(
          Frame::calibratePixel(px_cur[i], ref_frame->cam_param_));
    }

    // max number of frames in which a feature is seen
    VLOG(10) << "featureTracking: frame " << cur_frame->id_
             << ",  Nr tracked keypoints: " << cur_frame->keypoints_.size()
             << " (max: " << tracker_params_.maxFeaturesPerFrame_ << ")"
             << " (max observed age of tracked features: "
             << *std::max_element(cur_frame->landmarks_age_.begin(),
                                  cur_frame->landmarks_age_.end())
             << " vs. maxFeatureAge_: " << tracker_params_.maxFeatureAge_
             << ")";
  }

  // Fill debug information
  debug_info_.nrTrackerFeatures_ = cur_frame->keypoints_.size();
  debug_info_.featureTrackingTime_ = utils::Timer::toc(tic).count();
}

std::pair<TrackingStatus, gtsam::Pose3> Tracker::geometricOutlierRejectionMono(
    Frame* ref_frame,
    Frame* cur_frame) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  // Vector of bearing vectors.
  BearingVectors f_cur;
  f_cur.reserve(matches_ref_cur.size());
  BearingVectors f_ref;
  f_ref.reserve(matches_ref_cur.size());
  for (const KeypointMatch& it : matches_ref_cur) {
    // TODO(Toni) (luca): if versors are only needed at keyframe,
    // do not compute every frame
    f_ref.push_back(ref_frame->versors_.at(it.first));
    f_cur.push_back(cur_frame->versors_.at(it.second));
  }

  // Setup problem.
  AdapterMono adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMono> problem =
      std::make_shared<ProblemMono>(adapter,
                                    ProblemMono::NISTER,
                                    // last argument kills randomization
                                    tracker_params_.ransac_randomize_);
  opengv::sac::Ransac<ProblemMono> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = tracker_params_.ransac_threshold_mono_;
  ransac.max_iterations_ = tracker_params_.ransac_max_iterations_;
  ransac.probability_ = tracker_params_.ransac_probability_;

  VLOG(10) << "geometricOutlierRejectionMono: starting 5-point RANSAC.";

  // Solve.
  if (!ransac.computeModel(0)) {
    VLOG(10) << "failure: 5pt RANSAC could not find a solution.";
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  }

  VLOG(10) << "geometricOutlierRejectionMono: RANSAC complete.";

  VLOG(10) << "RANSAC (MONO): #iter = " << ransac.iterations_ << '\n'
           << " #inliers = " << ransac.inliers_.size() << " #outliers = "
           << ransac.inliers_.size() - matches_ref_cur.size();
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();

  // Remove outliers. This modifies the frames, that is why this function does
  // not simply accept const Frames. And removes outliers from matches.
  removeOutliersMono(ransac.inliers_, ref_frame, cur_frame, &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (ransac.inliers_.size() < tracker_params_.minNrMonoInliers_) {
    VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }

  double disparity;
  bool median_disparity_success = computeMedianDisparity(ref_frame->keypoints_,
                                                         cur_frame->keypoints_,
                                                         matches_ref_cur,
                                                         &disparity);
  LOG_IF(ERROR, !median_disparity_success)
      << "Median disparity calculation failed...";
  VLOG(10) << "Median disparity: " << disparity;
  if (disparity < tracker_params_.disparityThreshold_) {
    LOG(WARNING) << "LOW_DISPARITY: " << disparity;
    status = TrackingStatus::LOW_DISPARITY;
  } else {
    // Check for rotation only case
    // optical_flow_predictor_->predictFlow(ref_frame->keypoints_,
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  gtsam::Pose3 camLrectlkf_P_camLrectkf =
      UtilsOpenCV::openGvTfToGtsamPose3(best_transformation);

  // TODO(Toni) @Luca?
  // check if we have to compensate for rectification (if we have a valid
  // R_rectify_ )
  // if(ref_frame.cam_param_.R_rectify_.rows == 3 &&
  // cur_frame.cam_param_.R_rectify_.rows == 3){
  //  gtsam::Rot3 camLrect_R_camL_ref =
  //  UtilsOpenCV::Cvmat2rot(ref_frame.cam_param_.R_rectify_); gtsam::Rot3
  //  camLrect_R_camL_cut =
  //  UtilsOpenCV::Cvmat2rot(cur_frame.cam_param_.R_rectify_);
  //  camLrectlkf_P_camLrectkf =
  //      gtsam::Pose3(camLrect_R_camL_ref,Point3()) * camLlkf_P_camLkf *
  //      gtsam::Pose3(camLrect_R_camL_cut.inverse(),Point3());
  //}

  debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrMonoInliers_ = ransac.inliers_.size();
  debug_info_.monoRansacIters_ = ransac.iterations_;

  return std::make_pair(status, camLrectlkf_P_camLrectkf);
}

std::pair<TrackingStatus, gtsam::Pose3>
Tracker::geometricOutlierRejectionMonoGivenRotation(Frame* ref_frame,
                                                    Frame* cur_frame,
                                                    const gtsam::Rot3& R) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);

  // To log the time taken to perform this function.
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  // Vector of bearing vectors.
  BearingVectors f_cur;
  f_cur.reserve(matches_ref_cur.size());
  BearingVectors f_ref;
  f_ref.reserve(matches_ref_cur.size());
  for (const KeypointMatch& it : matches_ref_cur) {
    f_ref.push_back(ref_frame->versors_.at(it.first));
    f_cur.push_back(R.rotate(cur_frame->versors_.at(it.second)));
  }

  // Setup problem.
  AdapterMonoGivenRot adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMonoGivenRot> problem =
      std::make_shared<ProblemMonoGivenRot>(adapter,
                                            tracker_params_.ransac_randomize_);
  opengv::sac::Ransac<ProblemMonoGivenRot> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = tracker_params_.ransac_threshold_mono_;
  ransac.max_iterations_ = tracker_params_.ransac_max_iterations_;
  ransac.probability_ = tracker_params_.ransac_probability_;

  VLOG(10) << "geometricOutlierRejectionMonoGivenRot: starting 2-point RANSAC";

// Solve.
#ifdef sw_frontend
  // TODO(Toni) this function has rotten because of the ifdef :(
  // @Luca can we remove this ifdef and use a flag instead?
  ////////////////////////////////////
  // AMR: 2-point RANSAC
  int actual_iterations;
  std::vector<double> translation;
  translation.resize(3);
  std::vector<int> inliers =
      cv::ransac_2_point(f_ref,
                         f_cur,
                         trackerParams_.ransac_max_iterations_,
                         trackerParams_.ransac_threshold_mono_,
                         trackerParams_.ransac_probability_,
                         translation,
                         actual_iterations);
  gtsam::Matrix3 rot_mat = R.matrix();
  Eigen::Matrix<double, 3, 4> myModel;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) myModel(i, j) = rot_mat(i, j);
    myModel(i, 3) = translation[i];
  }
  ransac.model_coefficients_ = myModel;
  ransac.inliers_ = inliers;
  ransac.iterations_ = actual_iterations;

  VLOG(10)
      << "geometricOutlierRejectionMonoGivenRot: RANSAC complete sw version";

////////////////////////////////////
#else
  if (!ransac.computeModel(0)) {
    VLOG(10) << "failure: 2pt RANSAC could not find a solution";
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  }

  VLOG(10) << "geometricOutlierRejectionMonoGivenRot: RANSAC complete";
#endif

  VLOG(10) << "RANSAC (MONO): #iter = " << ransac.iterations_ << '\n'
           << " #inliers = " << ransac.inliers_.size() << "\n #outliers = "
           << ransac.inliers_.size() - matches_ref_cur.size();
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();  // before cleaning.
  removeOutliersMono(ransac.inliers_, ref_frame, cur_frame, &matches_ref_cur);

  // TODO(Toni):
  // CHECK QUALITY OF TRACKING
  TrackingStatus status = TrackingStatus::VALID;
  if (ransac.inliers_.size() < tracker_params_.minNrMonoInliers_) {
    VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }
  double disparity;
  bool median_disparity_success = computeMedianDisparity(ref_frame->keypoints_,
                                                         cur_frame->keypoints_,
                                                         matches_ref_cur,
                                                         &disparity);
  LOG_IF(ERROR, !median_disparity_success)
      << "Median disparity calculation failed...";

  VLOG(10) << "median disparity " << disparity;
  if (disparity < tracker_params_.disparityThreshold_) {
    VLOG(10) << "LOW_DISPARITY: " << disparity;
    status = TrackingStatus::LOW_DISPARITY;
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  gtsam::Pose3 camLlkf_P_camLkf =
      UtilsOpenCV::openGvTfToGtsamPose3(best_transformation);
  // note: this always returns the identity rotation, hence we have to
  // substitute it:
  camLlkf_P_camLkf = gtsam::Pose3(R, camLlkf_P_camLkf.translation());

  gtsam::Pose3 camLrectlkf_P_camLrectkf = camLlkf_P_camLkf;
  // check if we have to compensate for rectification (if we have a valid
  // R_rectify_ )
  if (ref_frame->cam_param_.R_rectify_.rows == 3 &&
      cur_frame->cam_param_.R_rectify_.rows == 3) {
    gtsam::Rot3 camLrect_R_camL_ref =
        UtilsOpenCV::cvMatToGtsamRot3(ref_frame->cam_param_.R_rectify_);
    gtsam::Rot3 camLrect_R_camL_cut =
        UtilsOpenCV::cvMatToGtsamRot3(cur_frame->cam_param_.R_rectify_);
    camLrectlkf_P_camLrectkf =
        gtsam::Pose3(camLrect_R_camL_ref, Point3()) * camLlkf_P_camLkf *
        gtsam::Pose3(camLrect_R_camL_cut.inverse(), Point3());
  }

  debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrMonoInliers_ = ransac.inliers_.size();
  debug_info_.monoRansacIters_ = ransac.iterations_;

  return std::make_pair(status, camLrectlkf_P_camLrectkf);
}

std::pair<Vector3, Matrix3> Tracker::getPoint3AndCovariance(
    const StereoFrame& stereoFrame,
    const gtsam::StereoCamera& stereoCam,
    const size_t pointId,
    const Matrix3& stereoPtCov,
    boost::optional<gtsam::Matrix3> Rmat) {
  gtsam::StereoPoint2 stereoPoint = gtsam::StereoPoint2(
      static_cast<double>(stereoFrame.left_keypoints_rectified_[pointId].x),
      static_cast<double>(stereoFrame.right_keypoints_rectified_[pointId].x),
      static_cast<double>(
          stereoFrame.left_keypoints_rectified_[pointId].y));  // uL_, uR_, v_;

  Matrix3 Jac_point3_sp2;  // jacobian of the back projection
  Vector3 point3_i_gtsam =
      stereoCam.backproject2(stereoPoint, boost::none, Jac_point3_sp2).vector();
  Vector3 point3_i = stereoFrame.keypoints_3d_.at(pointId);
  // TODO(Toni): Adapt value of this threshold for different calibration models!
  // (1e-1)
  if ((point3_i_gtsam - point3_i).norm() > 1e-1) {
    VLOG(10) << "\n point3_i_gtsam \n " << point3_i_gtsam << "\n point3_i \n"
             << point3_i;
    LOG(FATAL) << "GetPoint3AndCovariance: inconsistent "
               << "backprojection results (ref): "
               << (point3_i_gtsam - point3_i).norm();
  }
  if (Rmat) {
    point3_i = (*Rmat) * point3_i;  // optionally rotated to another ref frame
    Jac_point3_sp2 = (*Rmat) * Jac_point3_sp2;
  }

  gtsam::Matrix3 cov_i =
      Jac_point3_sp2 * stereoPtCov * Jac_point3_sp2.transpose();
  return std::make_pair(point3_i, cov_i);
}

// TODO(Toni) break down this gargantuan function...
std::pair<std::pair<TrackingStatus, gtsam::Pose3>, gtsam::Matrix3>
Tracker::geometricOutlierRejectionStereoGivenRotation(
    StereoFrame& ref_stereoFrame,
    StereoFrame& cur_stereoFrame,
    const gtsam::Rot3& R) {
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      ref_stereoFrame, cur_stereoFrame, &matches_ref_cur);

  VLOG(10) << "geometricOutlierRejectionStereoGivenRot:"
              " starting 1-point RANSAC (voting)";

  // Stereo point covariance: for covariance propagation.
  Matrix3 stereoPtCov = Matrix3::Identity();  // 3 px std in each direction

  // Create stereo camera.
  const gtsam::Cal3_S2& left_undist_rect_cam_mat =
      ref_stereoFrame.getLeftUndistRectCamMat();
  gtsam::Cal3_S2Stereo::shared_ptr K =
      boost::make_shared<gtsam::Cal3_S2Stereo>(left_undist_rect_cam_mat.fx(),
                                               left_undist_rect_cam_mat.fy(),
                                               left_undist_rect_cam_mat.skew(),
                                               left_undist_rect_cam_mat.px(),
                                               left_undist_rect_cam_mat.py(),
                                               ref_stereoFrame.getBaseline());
  // In the ref frame of the left camera.
  gtsam::StereoCamera stereoCam(gtsam::Pose3(), K);

  double timeMatchingAndAllocation_p = 0;
  timeMatchingAndAllocation_p = UtilsOpenCV::GetTimeInSeconds();

  //============================================================================
  // CREATE DATA STRUCTURES
  //============================================================================
  size_t nrMatches = matches_ref_cur.size();
  Vector3 f_ref_i, R_f_cur_i;
  Matrix3 cov_ref_i, cov_R_cur_i;

  // Relative translation suggested by each match and covariances
  // (DOUBLE FOR PRECISE COV COMPUTATION).
  Vectors3 relTran;
  relTran.reserve(
      nrMatches);  // translation such that Rot * f_cur = f_ref + relTran
  Matrices3 cov_relTran;
  cov_relTran.reserve(nrMatches);

  // relative translation suggested by each match and covariances
  // (FLOAT FOR FAST VOTING).
  Vectors3f relTranf;
  relTranf.reserve(
      nrMatches);  // translation such that Rot * f_cur = f_ref + relTran
  Matrices3f cov_relTranf;
  cov_relTranf.reserve(nrMatches);

  for (const KeypointMatch& it : matches_ref_cur) {
    // Get reference vector and covariance:
    std::tie(f_ref_i, cov_ref_i) = Tracker::getPoint3AndCovariance(
        ref_stereoFrame, stereoCam, it.first, stereoPtCov);
    // Get current vectors and covariance:
    std::tie(R_f_cur_i, cov_R_cur_i) = Tracker::getPoint3AndCovariance(
        cur_stereoFrame, stereoCam, it.second, stereoPtCov, R.matrix());

    // Populate relative translation estimates and their covariances.
    Vector3 v = f_ref_i - R_f_cur_i;
    Matrix3 M = cov_R_cur_i + cov_ref_i;

    relTran.push_back(v);
    cov_relTran.push_back(M);

    relTranf.push_back(v.cast<float>());
    cov_relTranf.push_back(M.cast<float>());
  }

  double timeCreatePointsAndCov_p = 0;
  timeCreatePointsAndCov_p = UtilsOpenCV::GetTimeInSeconds();

  //============================================================================
  // VOTING
  //============================================================================
  std::vector<std::vector<int>> coherentSet;
  // (THIS MUST BE RESIZE - fixed size) number of other translations
  // consistent with current one.
  coherentSet.resize(nrMatches);
  for (size_t i = 0; i < nrMatches; i++) coherentSet.at(i).reserve(nrMatches);

  size_t maxCoherentSetSize = 0;
  size_t maxCoherentSetId = 0;
  // double timeMahalanobis = 0, timeAllocate = 0, timePushBack = 0,
  // timeMaxSet = 0;
  float threshold = static_cast<float>(
      tracker_params_
          .ransac_threshold_stereo_);  // residual should be distributed
  // according to chi-square
  // distribution with 3 dofs,
  // considering a tail probability of 0.1, we get this value (x =
  // chi2inv(0.9,3) = 6.2514

  Vector3f v;
  Matrix3f O;  // allocate just once
  Vector3f relTran_i;
  Matrix3f cov_relTran_i;
  float dinv, innovationMahalanobisNorm;  // define just once
  for (size_t i = 0; i < nrMatches; i++) {
    relTran_i = relTranf.at(i);
    cov_relTran_i = cov_relTranf.at(i);
    coherentSet.at(i).push_back(i);  // vector is coherent with itself for sure
    for (size_t j = i + 1; j < nrMatches;
         j++) {  // look at the other vectors (quadratic complexity)
      // timeBefore = UtilsOpenCV::GetTimeInSeconds();
      v = relTran_i - relTranf.at(j);          // relTranMismatch_ij
      O = cov_relTran_i + cov_relTranf.at(j);  // cov_relTran_j
      // timeAllocate += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

      // see testTracker for different implementations and timing for the
      // mahalanobis distance
      // timeBefore = UtilsOpenCV::GetTimeInSeconds();
      dinv = 1 / (O(0, 0) * (O(1, 1) * O(2, 2) - O(1, 2) * O(2, 1)) -
                  O(1, 0) * (O(0, 1) * O(2, 2) - O(0, 2) * O(2, 1)) +
                  O(2, 0) * (O(0, 1) * O(1, 2) - O(1, 1) * O(0, 2)));
      innovationMahalanobisNorm =
          dinv * v(0) * (v(0) * (O(1, 1) * O(2, 2) - O(1, 2) * O(2, 1)) -
                         v(1) * (O(0, 1) * O(2, 2) - O(0, 2) * O(2, 1)) +
                         v(2) * (O(0, 1) * O(1, 2) - O(1, 1) * O(0, 2))) +
          dinv * v(1) * (O(0, 0) * (v(1) * O(2, 2) - O(1, 2) * v(2)) -
                         O(1, 0) * (v(0) * O(2, 2) - O(0, 2) * v(2)) +
                         O(2, 0) * (v(0) * O(1, 2) - v(1) * O(0, 2))) +
          dinv * v(2) * (O(0, 0) * (O(1, 1) * v(2) - v(1) * O(2, 1)) -
                         O(1, 0) * (O(0, 1) * v(2) - v(0) * O(2, 1)) +
                         O(2, 0) * (O(0, 1) * v(1) - O(1, 1) * v(0)));
      // timeMahalanobis += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

      // timeBefore = UtilsOpenCV::GetTimeInSeconds();
      if (innovationMahalanobisNorm < threshold) {
        coherentSet.at(i).push_back(j);
        coherentSet.at(j).push_back(i);  // norm is symmetric
      }
      // timePushBack += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
    }
    // timeBefore = UtilsOpenCV::GetTimeInSeconds();
    if (coherentSet.at(i).size() > maxCoherentSetSize) {
      maxCoherentSetSize = coherentSet.at(i).size();
      maxCoherentSetId = i;
    }
    // timeMaxSet += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
  }
  // std::cout << "timeMahalanobis: " << timeMahalanobis << std::endl
  //<< "timeAllocate: " << timeAllocate << std::endl
  //<< "timePushBack: " << timePushBack << std::endl
  //<< "timeMaxSet: " << timeMaxSet << std::endl
  //<< " relTran.size(): " << relTran.size() << std::endl;

  double timeVoting_p = 0;
  timeVoting_p = UtilsOpenCV::GetTimeInSeconds();

  VLOG(10) << "geometricOutlierRejectionStereoGivenRot: voting complete.";

  //============================================================================
  // OUTLIER REJECTION AND TRANSLATION COMPUTATION
  //============================================================================
  if (maxCoherentSetSize < 2) {
    VLOG(10) << "failure: 1point RANSAC (voting) could not find a solution.";
    return std::make_pair(
        std::make_pair(TrackingStatus::INVALID, gtsam::Pose3()),
        gtsam::Matrix3::Zero());
  }

  // Inliers are max coherent set.
  std::vector<int> inliers = coherentSet.at(maxCoherentSetId);

  // Sort inliers.
  std::sort(inliers.begin(), inliers.end());
  // UtilsOpenCV::PrintVector<int>(inliers,"inliers");
  // std::cout << "maxCoherentSetId: " << maxCoherentSetId << std::endl;

  VLOG(10) << "RANSAC (STEREO): #iter = " << 1 << '\n'
           << " #inliers = " << inliers.size()
           << "\n #outliers = " << inliers.size() - matches_ref_cur.size();
  debug_info_.nrStereoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  removeOutliersStereo(
      inliers, &ref_stereoFrame, &cur_stereoFrame, &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (inliers.size() < tracker_params_.minNrStereoInliers_) {
    VLOG(10) << "FEW_MATCHES: " << inliers.size();
    status = TrackingStatus::FEW_MATCHES;
  }

  // Get the resulting translation.
  Vector3 t = Vector3::Zero();
  Matrix3 totalInfo = Matrix3::Zero();
  for (size_t i = 0; i < inliers.size(); i++) {
    size_t matchId = inliers.at(i);
    Matrix3 infoMat = cov_relTran.at(matchId).inverse();
    t = t + infoMat * relTran.at(matchId);
    totalInfo = totalInfo + infoMat;
  }
  t = totalInfo.inverse() * t;

  // Fill debug info.
  if (VLOG_IS_ON(10)) {
    double timeTranslationComputation_p =
        utils::Timer::toc(start_time_tic).count();
    VLOG(10) << " timeMatchingAndAllocation: " << timeMatchingAndAllocation_p
             << " timeCreatePointsAndCov: "
             << timeCreatePointsAndCov_p - timeMatchingAndAllocation_p
             << " timeVoting: " << timeVoting_p - timeCreatePointsAndCov_p
             << " timeTranslationComputation: "
             << timeTranslationComputation_p - timeVoting_p;
  }
  debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrStereoInliers_ = inliers.size();
  debug_info_.stereoRansacIters_ = 1; // this is bcs we use coherent sets here.

  return std::make_pair(
      std::make_pair(status, gtsam::Pose3(R, gtsam::Point3(t))),
      totalInfo.cast<double>());
}

std::pair<TrackingStatus, gtsam::Pose3>
Tracker::geometricOutlierRejectionStereo(StereoFrame& ref_stereoFrame,
                                         StereoFrame& cur_stereoFrame) {
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      ref_stereoFrame, cur_stereoFrame, &matches_ref_cur);

  VLOG(10) << "geometricOutlierRejectionStereo:"
              " starting 3-point RANSAC (voting)";

  // Vector of 3D vectors
  Points3d f_cur;
  f_cur.reserve(matches_ref_cur.size());
  Points3d f_ref;
  f_ref.reserve(matches_ref_cur.size());
  for (const KeypointMatch& it : matches_ref_cur) {
    f_ref.push_back(ref_stereoFrame.keypoints_3d_.at(it.first));
    f_cur.push_back(cur_stereoFrame.keypoints_3d_.at(it.second));
  }

  // Setup problem (3D-3D adapter) -
  // http://laurentkneip.github.io/opengv/page_how_to_use.html
  AdapterStereo adapter(f_ref, f_cur);
  std::shared_ptr<ProblemStereo> problem = std::make_shared<ProblemStereo>(
      adapter, tracker_params_.ransac_randomize_);
  opengv::sac::Ransac<ProblemStereo> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = tracker_params_.ransac_threshold_stereo_;
  ransac.max_iterations_ = tracker_params_.ransac_max_iterations_;
  ransac.probability_ = tracker_params_.ransac_probability_;

  // Solve.
  if (!ransac.computeModel(0)) {
    VLOG(10) << "failure: (Arun) RANSAC could not find a solution.";
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  }

  VLOG(10) << "geometricOutlierRejectionStereo: voting complete.";

  VLOG(10) << "RANSAC (STEREO): #iter = " << ransac.iterations_ << '\n'
           << " #inliers = " << ransac.inliers_.size() << "\n #outliers = "
           << ransac.inliers_.size() - matches_ref_cur.size();
  debug_info_.nrStereoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  removeOutliersStereo(
      ransac.inliers_, &ref_stereoFrame, &cur_stereoFrame, &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (ransac.inliers_.size() < tracker_params_.minNrStereoInliers_) {
    VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  opengv::transformation_t best_transformation = ransac.model_coefficients_;

  // Fill debug info.
  debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrStereoInliers_ = ransac.inliers_.size();
  debug_info_.stereoRansacIters_ = ransac.iterations_;

  return std::make_pair(status,
                        UtilsOpenCV::openGvTfToGtsamPose3(best_transformation));
}

void Tracker::findOutliers(const KeypointMatches& matches_ref_cur,
                           std::vector<int> inliers,
                           std::vector<int>* outliers) {
  CHECK_NOTNULL(outliers)->clear();
  // Get outlier indices from inlier indices.
  std::sort(inliers.begin(), inliers.end(), std::less<size_t>());
  outliers->reserve(matches_ref_cur.size() - inliers.size());
  // The following is a complicated way of computing a set difference
  size_t k = 0;
  for (size_t i = 0u; i < matches_ref_cur.size(); ++i) {
    if (k < inliers.size()  // If we haven't exhaused inliers
        &&
        static_cast<int>(i) > inliers[k])  // If we are after the inlier[k]
      ++k;                                 // Check the next inlier
    if (k >= inliers.size() ||
        static_cast<int>(i) != inliers[k])  // If i is not an inlier
      outliers->push_back(i);
  }
}

void Tracker::checkStatusRightKeypoints(
    const std::vector<KeypointStatus>& right_keypoints_status) {
  debug_info_.nrValidRKP_ = 0;
  debug_info_.nrNoLeftRectRKP_ = 0;
  debug_info_.nrNoRightRectRKP_ = 0;
  debug_info_.nrNoDepthRKP_ = 0;
  debug_info_.nrFailedArunRKP_ = 0;
  for (const KeypointStatus& right_keypoint_status : right_keypoints_status) {
    switch (right_keypoint_status) {
      case KeypointStatus::VALID: {
        debug_info_.nrValidRKP_++;
        break;
      }
      case KeypointStatus::NO_LEFT_RECT: {
        debug_info_.nrNoLeftRectRKP_++;
        break;
      }
      case KeypointStatus::NO_RIGHT_RECT: {
        debug_info_.nrNoRightRectRKP_++;
        break;
      }
      case KeypointStatus::NO_DEPTH: {
        debug_info_.nrNoDepthRKP_++;
        break;
      }
      case KeypointStatus::FAILED_ARUN: {
        debug_info_.nrFailedArunRKP_++;
        break;
      }
    }
  }
}

// TODO(Toni): this and findOutliers can be greatly optimized...
void Tracker::removeOutliersMono(const std::vector<int>& inliers,
                                 Frame* ref_frame,
                                 Frame* cur_frame,
                                 KeypointMatches* matches_ref_cur) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);
  CHECK_NOTNULL(matches_ref_cur);
  // Find indices of outliers in current frame.
  std::vector<int> outliers;
  findOutliers(*matches_ref_cur, inliers, &outliers);
  // Remove outliers.
  // outliers cannot be a vector of size_t because opengv uses a vector of
  // int.
  for (const size_t& out : outliers) {
    const auto& ref_kp_cur_kp = (*matches_ref_cur)[out];
    ref_frame->landmarks_.at(ref_kp_cur_kp.first) = -1;
    cur_frame->landmarks_.at(ref_kp_cur_kp.second) = -1;
  }

  // Store only inliers from now on.
  KeypointMatches outlier_free_matches_ref_cur;
  outlier_free_matches_ref_cur.reserve(inliers.size());
  for (const size_t& in : inliers) {
    outlier_free_matches_ref_cur.push_back((*matches_ref_cur)[in]);
  }
  *matches_ref_cur = outlier_free_matches_ref_cur;
}

void Tracker::removeOutliersStereo(const std::vector<int>& inliers,
                                   StereoFrame* ref_stereoFrame,
                                   StereoFrame* cur_stereoFrame,
                                   KeypointMatches* matches_ref_cur) {
  CHECK_NOTNULL(ref_stereoFrame);
  CHECK_NOTNULL(cur_stereoFrame);
  CHECK_NOTNULL(matches_ref_cur);
  // Find indices of outliers in current stereo frame.
  std::vector<int> outliers;
  findOutliers(*matches_ref_cur, inliers, &outliers);

  // Remove outliers: outliers cannot be a vector of size_t because opengv uses
  // a vector of int.
  for (const size_t& out : outliers) {
    const KeypointMatch& kp_match = (*matches_ref_cur)[out];
    ref_stereoFrame->right_keypoints_status_.at(kp_match.first) =
        KeypointStatus::FAILED_ARUN;
    ref_stereoFrame->keypoints_depth_.at(kp_match.first) = 0.0;
    ref_stereoFrame->keypoints_3d_.at(kp_match.first) = Vector3::Zero();

    cur_stereoFrame->right_keypoints_status_.at(kp_match.second) =
        KeypointStatus::FAILED_ARUN;
    cur_stereoFrame->keypoints_depth_.at(kp_match.second) = 0.0;
    cur_stereoFrame->keypoints_3d_.at(kp_match.second) = Vector3::Zero();
  }

  // Store only inliers from now on.
  KeypointMatches outlier_free_matches_ref_cur;
  outlier_free_matches_ref_cur.reserve(inliers.size());
  for (const size_t& in : inliers) {
    outlier_free_matches_ref_cur.push_back((*matches_ref_cur)[in]);
  }
  *matches_ref_cur = outlier_free_matches_ref_cur;
}

void Tracker::findMatchingKeypoints(const Frame& ref_frame,
                                    const Frame& cur_frame,
                                    KeypointMatches* matches_ref_cur) {
  CHECK_NOTNULL(matches_ref_cur)->clear();

  // Find keypoints that observe the same landmarks in both frames:
  std::map<LandmarkId, size_t> ref_lm_index_map;
  for (size_t i = 0; i < ref_frame.landmarks_.size(); ++i) {
    const LandmarkId& ref_id = ref_frame.landmarks_.at(i);
    if (ref_id != -1) {
      // Map landmark id -> position in ref_frame.landmarks_
      ref_lm_index_map[ref_id] = i;
    }
  }

  // Map of position of landmark j in ref frame to position of landmark j in
  // cur_frame
  matches_ref_cur->reserve(ref_lm_index_map.size());
  for (size_t i = 0; i < cur_frame.landmarks_.size(); ++i) {
    const LandmarkId& cur_id = cur_frame.landmarks_.at(i);
    if (cur_id != -1) {
      auto it = ref_lm_index_map.find(cur_id);
      if (it != ref_lm_index_map.end()) {
        matches_ref_cur->push_back(std::make_pair(it->second, i));
      }
    }
  }
}

void Tracker::findMatchingStereoKeypoints(
    const StereoFrame& ref_stereoFrame,
    const StereoFrame& cur_stereoFrame,
    KeypointMatches* matches_ref_cur_stereo) {
  CHECK_NOTNULL(matches_ref_cur_stereo)->clear();
  KeypointMatches matches_ref_cur_mono;
  findMatchingKeypoints(ref_stereoFrame.getLeftFrame(),
                        cur_stereoFrame.getLeftFrame(),
                        &matches_ref_cur_mono);
  findMatchingStereoKeypoints(ref_stereoFrame,
                              cur_stereoFrame,
                              matches_ref_cur_mono,
                              matches_ref_cur_stereo);
}

void Tracker::findMatchingStereoKeypoints(
    const StereoFrame& ref_stereoFrame,
    const StereoFrame& cur_stereoFrame,
    const KeypointMatches& matches_ref_cur_mono,
    KeypointMatches* matches_ref_cur_stereo) {
  CHECK_NOTNULL(matches_ref_cur_stereo)->clear();
  for (size_t i = 0; i < matches_ref_cur_mono.size(); ++i) {
    const size_t& ind_ref = matches_ref_cur_mono[i].first;
    const size_t& ind_cur = matches_ref_cur_mono[i].second;
    // At this point we already discarded keypoints with landmark = -1
    if (ref_stereoFrame.right_keypoints_status_[ind_ref] ==
            KeypointStatus::VALID &&
        cur_stereoFrame.right_keypoints_status_[ind_cur] ==
            KeypointStatus::VALID) {
      // Pair of points that has 3D in both stereoFrames.
      matches_ref_cur_stereo->push_back(matches_ref_cur_mono[i]);
    }
  }
}

bool Tracker::computeMedianDisparity(const KeypointsCV& ref_frame_kpts,
                                     const KeypointsCV& cur_frame_kpts,
                                     const KeypointMatches& matches_ref_cur,
                                     double* median_disparity) {
  CHECK_NOTNULL(median_disparity);
  // Compute disparity:
  std::vector<double> disparity_sq;
  disparity_sq.reserve(matches_ref_cur.size());
  for (const KeypointMatch& rc : matches_ref_cur) {
    KeypointCV px_diff = cur_frame_kpts[rc.second] - ref_frame_kpts[rc.first];
    double px_dist = px_diff.x * px_diff.x + px_diff.y * px_diff.y;
    disparity_sq.push_back(px_dist);
  }

  if (disparity_sq.empty()) {
    LOG(WARNING) << "Have no matches for disparity computation.";
    *median_disparity = 0.0;
    return false;
  }

  // Compute median:
  const size_t center = disparity_sq.size() / 2;
  // nth element sorts the array partially until it finds the median.
  std::nth_element(
      disparity_sq.begin(), disparity_sq.begin() + center, disparity_sq.end());
  *median_disparity = std::sqrt(disparity_sq[center]);
  return true;
}

/* -------------------------------------------------------------------------- */
cv::Mat Tracker::getTrackerImage(const Frame& ref_frame,
                                 const Frame& cur_frame,
                                 const KeypointsCV& extra_corners_gray,
                                 const KeypointsCV& extra_corners_blue) const {
  cv::Mat img_rgb(cur_frame.img_.size(), CV_8U);
  cv::cvtColor(cur_frame.img_, img_rgb, cv::COLOR_GRAY2RGB);

  static const cv::Scalar gray(255, 255, 255);
  static const cv::Scalar blue(255, 0, 0);
  static const cv::Scalar red(0, 0, 255);
  static const cv::Scalar green(0, 255, 0);

  // Add extra corners if desired.
  for (const auto& px_cur : extra_corners_gray) {
    cv::circle(img_rgb, px_cur, 4, gray, 2);
  }
  for (const auto& px_cur : extra_corners_blue) {
    cv::circle(img_rgb, px_cur, 4, blue, 2);
  }

  // Add all keypoints in cur_frame with the tracks.
  for (size_t i = 0; i < cur_frame.keypoints_.size(); ++i) {
    const cv::Point2f& px_cur = cur_frame.keypoints_.at(i);
    if (cur_frame.landmarks_.at(i) == -1) {  // Untracked landmarks are red.
      cv::circle(img_rgb, px_cur, 4, red, 2);
    } else {
      const auto& it = find(ref_frame.landmarks_.begin(),
                            ref_frame.landmarks_.end(),
                            cur_frame.landmarks_.at(i));
      if (it != ref_frame.landmarks_.end()) {
        // If feature was in previous frame, display tracked feature with
        // green circle/line:
        cv::circle(img_rgb, px_cur, 6, green, 1);
        int nPos = std::distance(ref_frame.landmarks_.begin(), it);
        const cv::Point2f& px_ref = ref_frame.keypoints_.at(nPos);
        cv::line(img_rgb, px_cur, px_ref, green, 1);
      } else {  // New feature tracks are blue.
        cv::circle(img_rgb, px_cur, 6, blue, 1);
      }
    }
  }
  return img_rgb;
}

}  // namespace VIO
