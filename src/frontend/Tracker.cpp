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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <string>
#include <utility>  // for pair<>
#include <vector>   // for vector<>

#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictorFactory.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/visualizer/Display-definitions.h"

DEFINE_bool(visualize_feature_predictions,
            false,
            "Visualizes feature tracks and predicted keypoints given rotation "
            "from IMU.");

namespace VIO {

std::vector<int> remapOpenGvInliersToKimera(
    const std::vector<int>& ogv_inliers,
    const std::map<int, int>& ogv_to_kimera_map) {
  std::vector<int> kimera_inliers;
  kimera_inliers.reserve(ogv_inliers.size());
  for (int i : ogv_inliers) {
    const auto& it = ogv_to_kimera_map.find(i);
    if (it != ogv_to_kimera_map.end()) {
      kimera_inliers.push_back(it->second);
    } else {
      LOG(FATAL) << "Couldn't find a map from ogv inlier: " << i
                 << " to kimera...";
    }
  }
  CHECK_EQ(kimera_inliers.size(), ogv_inliers.size());
  return kimera_inliers;
}

Tracker::Tracker(const TrackerParams& tracker_params,
                 const Camera::ConstPtr& camera,
                 DisplayQueue* display_queue)
    : tracker_params_(tracker_params),
      landmark_count_(0),
      camera_(camera),
      // Only for debugging and visualization:
      optical_flow_predictor_(nullptr),
      display_queue_(display_queue),
      output_images_path_("./outputImages/") {
  // Create the optical flow prediction module
  optical_flow_predictor_ =
      OpticalFlowPredictorFactory::makeOpticalFlowPredictor(
          tracker_params_.optical_flow_predictor_type_,
          camera_->getCamParams().K_,
          camera_->getCamParams().image_size_);

  // Setup Mono Ransac
  mono_ransac_.threshold_ = tracker_params_.ransac_threshold_mono_;
  mono_ransac_.max_iterations_ = tracker_params_.ransac_max_iterations_;
  mono_ransac_.probability_ = tracker_params_.ransac_probability_;

  // Setup Mono Ransac given Rotation
  mono_ransac_given_rot_.threshold_ = tracker_params_.ransac_threshold_mono_;
  mono_ransac_given_rot_.max_iterations_ =
      tracker_params_.ransac_max_iterations_;
  mono_ransac_given_rot_.probability_ = tracker_params_.ransac_probability_;

  // Setup Stereo Ransac
  stereo_ransac_.threshold_ = tracker_params_.ransac_threshold_stereo_;
  stereo_ransac_.max_iterations_ = tracker_params_.ransac_max_iterations_;
  stereo_ransac_.probability_ = tracker_params_.ransac_probability_;
}

// TODO(Toni) a pity that this function is not const just because
// it modifies debuginfo_...
// NOTE: you do not need R in the mono case. For stereo cameras we pass R
// to ensure we rectify the versors and 3D points of the features we detect.
void Tracker::featureTracking(
    Frame* ref_frame,
    Frame* cur_frame,
    const gtsam::Rot3& ref_R_cur,
    const FeatureDetectorParams& feature_detector_params,
    std::optional<cv::Mat> R) {
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
  const cv::Size2i klt_window_size(tracker_params_.klt_win_size_,
                                   tracker_params_.klt_win_size_);

  // Initialize to old locations
  LOG_IF(ERROR, px_ref.size() == 0u) << "No keypoints in reference frame!";

  KeypointsCV px_cur;
  CHECK(optical_flow_predictor_->predictSparseFlow(px_ref, ref_R_cur, &px_cur));
  KeypointsCV px_predicted = px_cur;

  // Do the actual tracking, so px_cur becomes the new pixel locations.
  VLOG(2) << "Starting Optical Flow Pyr LK tracking...";

  std::vector<uchar> status;
  std::vector<float> error;
  auto time_lukas_kanade_tic = utils::Timer::tic();
  cv::calcOpticalFlowPyrLK(ref_frame->img_,
                           cur_frame->img_,
                           px_ref,
                           px_cur,
                           status,
                           error,
                           klt_window_size,
                           tracker_params_.klt_max_level_,
                           kTerminationCriteria,
                           cv::OPTFLOW_USE_INITIAL_FLOW);
  VLOG(1) << "Optical Flow Timing [ms]: "
          << utils::Timer::toc(time_lukas_kanade_tic).count();
  VLOG(2) << "Finished Optical Flow Pyr LK tracking.";

  // TODO(Toni): use the error to further take only the best tracks?

  // At this point cur_frame should have no keypoints...
  CHECK(cur_frame->keypoints_.empty());
  CHECK(cur_frame->landmarks_.empty());
  CHECK(cur_frame->landmarks_age_.empty());
  CHECK(cur_frame->keypoints_.empty());
  CHECK(cur_frame->scores_.empty());
  CHECK(cur_frame->versors_.empty());
  // TODO(TOni): this is basically copying the whole px_ref into the
  // current frame as well as the ref_frame information! Absolute nonsense.
  cur_frame->landmarks_.reserve(px_ref.size());
  cur_frame->landmarks_age_.reserve(px_ref.size());
  cur_frame->keypoints_.reserve(px_ref.size());
  cur_frame->scores_.reserve(px_ref.size());
  cur_frame->versors_.reserve(px_ref.size());
  for (size_t i = 0u; i < indices_of_valid_landmarks.size(); ++i) {
    // If we failed to track mark off that landmark
    const size_t& idx_valid_lmk = indices_of_valid_landmarks[i];
    const size_t& lmk_age = ref_frame->landmarks_age_[idx_valid_lmk];
    const LandmarkId& lmk_id = ref_frame->landmarks_[idx_valid_lmk];

    // if we tracked keypoint and feature track is not too long
    if (!status[i] || lmk_age > tracker_params_.max_feature_track_age_) {
      // we are marking this bad in the ref_frame since features
      // in the ref frame guide feature detection later on
      ref_frame->landmarks_[idx_valid_lmk] = -1;
      continue;
    }
    cur_frame->landmarks_.push_back(lmk_id);
    cur_frame->landmarks_age_.push_back(lmk_age);
    cur_frame->scores_.push_back(ref_frame->scores_[idx_valid_lmk]);
    cur_frame->keypoints_.push_back(px_cur[i]);
    gtsam::Vector3 bearing_vector = UndistorterRectifier::GetBearingVector(
        px_cur[i], ref_frame->cam_param_, R);
    CHECK_LT(std::abs(bearing_vector.norm() - 1.0), 1e-6)
        << "Versor norm: " << bearing_vector.norm();
    cur_frame->versors_.push_back(bearing_vector);
  }

  // max number of frames in which a feature is seen
  VLOG(5) << "featureTracking: frame " << cur_frame->id_
          << ",  Nr tracked keypoints: " << cur_frame->keypoints_.size()
          << " (max: " << feature_detector_params.max_features_per_frame_ << ")"
          << " (max observed age of tracked features: "
          << *std::max_element(cur_frame->landmarks_age_.begin(),
                               cur_frame->landmarks_age_.end())
          << " vs. max_feature_track_age_: "
          << tracker_params_.max_feature_track_age_ << ")";
  // Display feature tracks together with predicted points.
  if (display_queue_ && FLAGS_visualize_feature_predictions) {
    displayImage(cur_frame->timestamp_,
                 "feature_tracks_with_predicted_keypoints",
                 getTrackerImage(*ref_frame, *cur_frame, px_predicted, px_ref),
                 display_queue_);
  }

  // Fill debug information
  debug_info_.nrTrackerFeatures_ = cur_frame->keypoints_.size();
  debug_info_.featureTrackingTime_ = utils::Timer::toc(tic).count();
}

TrackingStatusPose Tracker::geometricOutlierRejection2d2d(
    const BearingVectors& ref_bearings,
    const BearingVectors& cur_bearings,
    const KeypointMatches& matches_ref_cur,
    std::vector<int>* inliers,
    // TODO(TONI): I think this should be using non-rectified left cameras...
    const gtsam::Pose3& cam_lkf_Pose_cam_kf) {
  CHECK_NOTNULL(inliers);

  TrackingStatusPose status_pose;
  // NOTE: versors are already in the rectified left camera frame.
  // No further rectification needed.
  //! Get bearing vectors for opengv.
  BearingVectors f_ref;
  BearingVectors f_cur;
  const size_t& n_matches = matches_ref_cur.size();
  f_ref.reserve(n_matches);
  f_cur.reserve(n_matches);
  for (const KeypointMatch& it : matches_ref_cur) {
    //! Reference bearing vector
    CHECK_LT(it.first, ref_bearings.size());
    const auto& ref_bearing = ref_bearings.at(it.first);
    f_ref.push_back(ref_bearing);

    //! Current bearing vector
    CHECK_LT(it.second, cur_bearings.size());
    const auto& cur_bearing = cur_bearings.at(it.second);
    f_cur.push_back(cur_bearing);
  }

  //! Setup adapter.
  CHECK_GT(f_ref.size(), 0);
  CHECK_EQ(f_ref.size(), f_cur.size());
  CHECK_EQ(f_ref.size(), n_matches);
  Adapter2d2d adapter(f_ref, f_cur);
  if (tracker_params_.ransac_use_2point_mono_) {
    adapter.setR12(cam_lkf_Pose_cam_kf.rotation().matrix());
    adapter.sett12(cam_lkf_Pose_cam_kf.translation().matrix());
  }

  //! Solve problem.
  gtsam::Pose3 best_pose = gtsam::Pose3();
  bool success = false;
  if (tracker_params_.ransac_use_2point_mono_) {
    success = runRansac(std::make_shared<Problem2d2dGivenRot>(
                            adapter, tracker_params_.ransac_randomize_),
                        tracker_params_.ransac_threshold_mono_,
                        tracker_params_.ransac_max_iterations_,
                        tracker_params_.ransac_probability_,
                        tracker_params_.optimize_2d2d_pose_from_inliers_,
                        &best_pose,
                        inliers);
  } else {
    success = runRansac(
        std::make_shared<Problem2d2d>(adapter,
                                      tracker_params_.pose_2d2d_algorithm_,
                                      tracker_params_.ransac_randomize_),
        tracker_params_.ransac_threshold_mono_,
        tracker_params_.ransac_max_iterations_,
        tracker_params_.ransac_probability_,
        tracker_params_.optimize_2d2d_pose_from_inliers_,
        &best_pose,
        inliers);
  }

  if (!success) {
    status_pose = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  } else {
    // TODO(Toni): it seems we are not removing outliers if we send an invalid
    // tracking status (above), but the backend calls addLandmarksToGraph even
    // when we have an invalid status!

    // TODO(Toni): check quality of tracking
    //! Check enough inliers.
    TrackingStatus status = TrackingStatus::VALID;
    if (inliers->size() <
        static_cast<size_t>(tracker_params_.minNrMonoInliers_)) {
      CHECK(!inliers->empty());
      status = TrackingStatus::FEW_MATCHES;
    }

    // NOTE: 2-point always returns the identity rotation, hence we have to
    // substitute it:
    if (tracker_params_.ransac_use_2point_mono_) {
      CHECK(cam_lkf_Pose_cam_kf.rotation().equals(best_pose.rotation()));
    }

    //! Fill debug info.
    debug_info_.nrMonoPutatives_ = adapter.getNumberCorrespondences();
    debug_info_.nrMonoInliers_ = inliers->size();
    debug_info_.monoRansacIters_ = 0;  // no access to ransac from here
    // debug_info_.monoRansacIters_ = ransac->iterations_;

    status_pose = std::make_pair(status, best_pose);
  }

  VLOG(5) << "2D2D tracking " << (success ? " success " : " failure ") << ":\n"
          << "- Tracking Status: "
          << TrackerStatusSummary::asString(status_pose.first) << '\n'
          << "- Total Correspondences: " << f_ref.size() << '\n'
          << "\t- # inliers: " << inliers->size() << '\n'
          << "\t- # outliers: " << f_ref.size() - inliers->size() << '\n'
          << "- Best pose: \n"
          << status_pose.second;

  return status_pose;
}

// TODO(Toni): this function is almost a replica of the Stereo version,
// factorize.
TrackingStatusPose Tracker::geometricOutlierRejection2d2d(
    Frame* ref_frame,
    Frame* cur_frame,
    const gtsam::Pose3& cam_lkf_Pose_cam_kf) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  TrackingStatusPose result;

  if (matches_ref_cur.empty()) {
    LOG(ERROR) << "No matching keypoints from frame " << ref_frame->id_
               << " to frame " << cur_frame->id_ << ".\n"
               << "Mono Tracking Status = INVALID.";
    result = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  } else {
    std::vector<int> inliers;
    result = geometricOutlierRejection2d2d(ref_frame->versors_,
                                           cur_frame->versors_,
                                           matches_ref_cur,
                                           &inliers,
                                           cam_lkf_Pose_cam_kf);

    // TODO(Toni): should we remove outliers if few matches?
    //! Remove correspondences classified as outliers
    if (result.first != TrackingStatus::FEW_MATCHES) {
      removeOutliersMono(inliers, ref_frame, cur_frame, &matches_ref_cur);
    }

    // THIS IS ONLY USEFUL if we are completely still...
    if (result.first == TrackingStatus::VALID) {
      // TODO(TONI): unrotate due to optical flow before calculating
      // disparity...
      // TODO(TONI): this has no place here... should be somewhere else...
      //! Check enough disparity.
      double disparity;
      if (computeMedianDisparity(ref_frame->keypoints_,
                                 cur_frame->keypoints_,
                                 matches_ref_cur,
                                 &disparity)) {
        if (disparity < tracker_params_.disparityThreshold_) {
          LOG(INFO) << "Low mono disparity.";
          result.first = TrackingStatus::LOW_DISPARITY;
        }
      } else {
        LOG(ERROR) << "Median disparity calculation failed...";
      }
    }
  }

  debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  return result;
}

// TODO(Toni): remove
// TODO(Toni) break down this gargantuan function...
std::pair<TrackingStatusPose, gtsam::Matrix3>
Tracker::geometricOutlierRejection3d3dGivenRotation(
    const StatusKeypointsCV& ref_keypoints_status_left,
    const StatusKeypointsCV& ref_keypoints_status_right,
    const StatusKeypointsCV& cur_keypoints_status_left,
    const StatusKeypointsCV& cur_keypoints_status_right,
    const Landmarks& ref_keypoints_3d,
    const Landmarks& cur_keypoints_3d,
    const gtsam::StereoCamera& stereo_cam,
    const KeypointMatches& matches_ref_cur,
    const gtsam::Rot3& camLrectlkf_R_camLrectkf,
    std::vector<int>* inliers) {
  auto start_time_tic = utils::Timer::tic();

  VLOG(5) << "OutlierRejectionStereoGivenRot:"
             " starting 1-point RANSAC (voting)";

  // TODO(Toni): this is 1px std in each dir as of now? Parametrize at the very
  // least...
  // Stereo point covariance: for covariance propagation.
  // 3 px std in each direction
  gtsam::Matrix3 stereo_pt_cov = gtsam::Matrix3::Identity();

  double timeMatchingAndAllocation_p =
      utils::Timer::toc(start_time_tic).count();

  //============================================================================
  // CREATE DATA STRUCTURES
  //============================================================================
  auto timeCreatePointsAndCov_p_tic = utils::Timer::tic();
  size_t n_matches = matches_ref_cur.size();

  // NOTE: 3d points are constructed by versors, which are already in the
  // rectified left camera frame. No further rectification needed.
  gtsam::Vector3 f_ref_i, R_f_cur_i;
  gtsam::Matrix3 cov_ref_i, cov_R_cur_i;

  // Relative translation suggested by each match and covariances
  // (DOUBLE FOR PRECISE COV COMPUTATION).
  Vectors3 rel_tran;
  // translation such that Rot * f_cur = f_ref + relTran
  rel_tran.reserve(n_matches);
  Matrices3 cov_relTran;
  cov_relTran.reserve(n_matches);

  // relative translation suggested by each match and covariances
  // (FLOAT FOR FAST VOTING).
  Vectors3f relTranf;
  // translation such that Rot * f_cur = f_ref + relTran
  relTranf.reserve(n_matches);
  Matrices3f cov_relTranf;
  cov_relTranf.reserve(n_matches);

  for (const KeypointMatch& kpt_match : matches_ref_cur) {
    // Get reference vector and covariance:
    std::tie(f_ref_i, cov_ref_i) =
        Tracker::getPoint3AndCovariance(ref_keypoints_status_left,
                                        ref_keypoints_status_right,
                                        ref_keypoints_3d,
                                        stereo_cam,
                                        kpt_match.first,
                                        stereo_pt_cov,
                                        std::nullopt);
    // Get current vectors and covariance:
    std::tie(R_f_cur_i, cov_R_cur_i) =
        Tracker::getPoint3AndCovariance(cur_keypoints_status_left,
                                        cur_keypoints_status_right,
                                        cur_keypoints_3d,
                                        stereo_cam,
                                        kpt_match.second,
                                        stereo_pt_cov,
                                        camLrectlkf_R_camLrectkf.matrix());

    // Populate relative translation estimates and their covariances.
    gtsam::Vector3 v = f_ref_i - R_f_cur_i;
    gtsam::Matrix3 M = cov_R_cur_i + cov_ref_i;

    rel_tran.push_back(v);
    cov_relTran.push_back(M);

    relTranf.push_back(v.cast<float>());
    cov_relTranf.push_back(M.cast<float>());
  }

  double timeCreatePointsAndCov_p =
      utils::Timer::toc(timeCreatePointsAndCov_p_tic).count();

  //============================================================================
  // VOTING
  //============================================================================
  auto time_voting_tic = utils::Timer::tic();

  std::vector<std::vector<int>> coherent_set;
  // (THIS MUST BE RESIZE - fixed size) number of other translations
  // consistent with current one.
  coherent_set.resize(n_matches);
  for (size_t i = 0; i < n_matches; i++) coherent_set.at(i).reserve(n_matches);

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
  for (size_t i = 0; i < n_matches; i++) {
    relTran_i = relTranf.at(i);
    cov_relTran_i = cov_relTranf.at(i);
    coherent_set.at(i).push_back(i);  // vector is coherent with itself for sure
    for (size_t j = i + 1; j < n_matches;
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
          dinv * v(0) *
              (v(0) * (O(1, 1) * O(2, 2) - O(1, 2) * O(2, 1)) -
               v(1) * (O(0, 1) * O(2, 2) - O(0, 2) * O(2, 1)) +
               v(2) * (O(0, 1) * O(1, 2) - O(1, 1) * O(0, 2))) +
          dinv * v(1) *
              (O(0, 0) * (v(1) * O(2, 2) - O(1, 2) * v(2)) -
               O(1, 0) * (v(0) * O(2, 2) - O(0, 2) * v(2)) +
               O(2, 0) * (v(0) * O(1, 2) - v(1) * O(0, 2))) +
          dinv * v(2) *
              (O(0, 0) * (O(1, 1) * v(2) - v(1) * O(2, 1)) -
               O(1, 0) * (O(0, 1) * v(2) - v(0) * O(2, 1)) +
               O(2, 0) * (O(0, 1) * v(1) - O(1, 1) * v(0)));
      // timeMahalanobis += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

      // timeBefore = UtilsOpenCV::GetTimeInSeconds();
      if (innovationMahalanobisNorm < threshold) {
        coherent_set.at(i).push_back(j);
        coherent_set.at(j).push_back(i);  // norm is symmetric
      }
      // timePushBack += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
    }
    // timeBefore = UtilsOpenCV::GetTimeInSeconds();
    if (coherent_set.at(i).size() > maxCoherentSetSize) {
      maxCoherentSetSize = coherent_set.at(i).size();
      maxCoherentSetId = i;
    }
    // timeMaxSet += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
  }
  // LOG(INFO) << "timeMahalanobis: " << timeMahalanobis << std::endl
  //<< "timeAllocate: " << timeAllocate << std::endl
  //<< "timePushBack: " << timePushBack << std::endl
  //<< "timeMaxSet: " << timeMaxSet << std::endl
  //<< " relTran.size(): " << relTran.size() << std::endl;

  double time_voting_p = utils::Timer::toc(time_voting_tic).count();

  VLOG(5) << "OutlierRejectionStereoGivenRot: voting complete.";

  //============================================================================
  // OUTLIER REJECTION AND TRANSLATION COMPUTATION
  //============================================================================
  if (maxCoherentSetSize < 2) {
    LOG(WARNING) << "1-point RANSAC (voting) could not find a solution.";
    return std::make_pair(
        std::make_pair(TrackingStatus::INVALID, gtsam::Pose3()),
        gtsam::Matrix3::Zero());
  }

  // Inliers are max coherent set.
  *inliers = coherent_set.at(maxCoherentSetId);

  // Sort inliers.
  std::sort(inliers->begin(), inliers->end());
  // UtilsOpenCV::PrintVector<int>(inliers,"inliers");
  // LOG(INFO) << "maxCoherentSetId: " << maxCoherentSetId << std::endl;

  VLOG(5) << "RANSAC (STEREO): \n"
          << "- #iter = " << 1 << '\n'
          << "- #inliers = " << inliers->size() << '\n'
          << "- #outliers = " << inliers->size() - matches_ref_cur.size()
          << '\n'
          << "Total = " << matches_ref_cur.size();
  debug_info_.nrStereoPutatives_ = matches_ref_cur.size();

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (inliers->size() <
      static_cast<size_t>(tracker_params_.minNrStereoInliers_)) {
    CHECK(!inliers->empty());
    status = TrackingStatus::FEW_MATCHES;
  }

  // Get the resulting translation.
  gtsam::Vector3 t = gtsam::Vector3::Zero();
  gtsam::Matrix3 total_info = gtsam::Matrix3::Zero();
  for (size_t i = 0; i < inliers->size(); i++) {
    size_t match_id = inliers->at(i);
    gtsam::Matrix3 infoMat = cov_relTran.at(match_id).inverse();
    t = t + infoMat * rel_tran.at(match_id);
    total_info = total_info + infoMat;
  }
  t = total_info.inverse() * t;

  // Fill debug info.
  if (VLOG_IS_ON(10)) {
    double time_translation_computation_p =
        utils::Timer::toc(start_time_tic).count();
    VLOG(5) << " Time MatchingAndAllocation: " << timeMatchingAndAllocation_p
            << " Time CreatePointsAndCov: " << timeCreatePointsAndCov_p
            << " Time Voting: " << time_voting_p
            << " Time translation computation p: "
            << time_translation_computation_p;
  }
  debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrStereoInliers_ = inliers->size();
  debug_info_.stereoRansacIters_ = 1;  // this is bcs we use coherent sets here.

  LOG_IF(FATAL, tracker_params_.optimize_3d3d_pose_from_inliers_)
      << "Nonlinear optimization on inliers from 3D3D with prior is not "
         "implemented";

  gtsam::Pose3 best_pose =
      gtsam::Pose3(camLrectlkf_R_camLrectkf, gtsam::Point3(t));

  VLOG(5) << "3D3D tracking given rotation: "
          << "always success ...\n"
          << "- Tracking Status: " << TrackerStatusSummary::asString(status)
          << '\n'
          << "- Total Correspondences: " << matches_ref_cur.size() << '\n'
          << "\t- # inliers: " << inliers->size() << '\n'
          << "\t- # outliers: " << matches_ref_cur.size() - inliers->size()
          << '\n'
          << "- Best pose: \n"
          << best_pose;

  return std::make_pair(std::make_pair(status, best_pose),
                        total_info.cast<double>());
}

std::pair<TrackingStatusPose, gtsam::Matrix3>
Tracker::geometricOutlierRejection3d3dGivenRotation(
    StereoFrame& ref_stereo_frame,
    StereoFrame& cur_stereo_frame,
    const gtsam::StereoCamera& stereo_camera,
    const gtsam::Rot3& camLrectlkf_R_camLrectkf) {
  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      ref_stereo_frame, cur_stereo_frame, &matches_ref_cur);

  std::vector<int> inliers;
  std::pair<TrackingStatusPose, gtsam::Matrix3> result =
      geometricOutlierRejection3d3dGivenRotation(
          ref_stereo_frame.left_keypoints_rectified_,
          ref_stereo_frame.right_keypoints_rectified_,
          cur_stereo_frame.left_keypoints_rectified_,
          cur_stereo_frame.right_keypoints_rectified_,
          ref_stereo_frame.keypoints_3d_,
          cur_stereo_frame.keypoints_3d_,
          stereo_camera,
          matches_ref_cur,
          camLrectlkf_R_camLrectkf,
          &inliers);

  // Remove outliers.
  removeOutliersStereo(
      inliers, &ref_stereo_frame, &cur_stereo_frame, &matches_ref_cur);

  return result;
}

// TODO(Toni): this function is almost a replica of the Mono version,
// factorize.
TrackingStatusPose Tracker::geometricOutlierRejection3d3d(
    const Landmarks& ref_keypoints_3d,
    const Landmarks& cur_keypoints_3d,
    const KeypointMatches& matches_ref_cur,
    std::vector<int>* inliers,
    const gtsam::Pose3& cam_lkf_Pose_cam_kf) {
  auto start_time_tic = utils::Timer::tic();

  // NOTE: 3d points are constructed by versors, which are already in the
  // rectified left camera frame.
  // No further rectification needed.
  //! Get 3D landmarks landmarks for opengv.
  const size_t& n_matches = matches_ref_cur.size();
  BearingVectors f_ref, f_cur;
  f_ref.reserve(n_matches);
  f_cur.reserve(n_matches);
  for (const KeypointMatch& it : matches_ref_cur) {
    const Landmark& ref_lmk = ref_keypoints_3d.at(it.first);
    const Landmark& cur_lmk = cur_keypoints_3d.at(it.second);
    f_ref.push_back(ref_lmk);
    f_cur.push_back(cur_lmk);
  }

  //! Setup adapter.
  Adapter3d3d adapter(f_ref, f_cur);
  // This is not really used, only in nonlinear optimization, but not in 3-point
  adapter.setR12(cam_lkf_Pose_cam_kf.rotation().matrix());
  adapter.sett12(cam_lkf_Pose_cam_kf.translation().matrix());

  VLOG(5) << "OutlierRejection3D3D: starting 3-point RANSAC (voting)";
  //! Solve problem.
  gtsam::Pose3 best_pose = gtsam::Pose3();
  bool success = false;
  //! 3-point Arun method
  success = runRansac(
      std::make_shared<Problem3d3d>(adapter, tracker_params_.ransac_randomize_),
      tracker_params_.ransac_threshold_stereo_,
      tracker_params_.ransac_max_iterations_,
      tracker_params_.ransac_probability_,
      tracker_params_.optimize_3d3d_pose_from_inliers_,
      &best_pose,
      inliers);

  TrackingStatusPose status_pose;
  if (!success) {
    status_pose = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
  } else {
    //! Check enough inliers.
    TrackingStatus status = TrackingStatus::VALID;
    if (inliers->size() <
        static_cast<size_t>(tracker_params_.minNrStereoInliers_)) {
      CHECK(!inliers->empty());
      status = TrackingStatus::FEW_MATCHES;
    }

    //! Fill debug info.
    debug_info_.nrStereoPutatives_ = adapter.getNumberCorrespondences();
    debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
    debug_info_.nrStereoInliers_ = inliers->size();
    debug_info_.stereoRansacIters_ = 0;  // no access to ransac from here
    // debug_info_.stereoRansacIters_ = iterations;

    status_pose = std::make_pair(status, best_pose);
  }

  VLOG(5) << "3D3D tracking " << (success ? " success " : " failure ") << ":\n"
          << "- Tracking Status: "
          << TrackerStatusSummary::asString(status_pose.first) << '\n'
          << "- Total Correspondences: " << f_ref.size() << '\n'
          << "\t- # inliers: " << inliers->size() << '\n'
          << "\t- # outliers: " << f_ref.size() - inliers->size() << '\n'
          << "- Best pose: \n"
          << status_pose.second;

  return status_pose;
}

TrackingStatusPose Tracker::geometricOutlierRejection3d3d(
    StereoFrame* ref_stereo_frame,
    StereoFrame* cur_stereo_frame,
    const gtsam::Pose3& cam_lkf_Pose_cam_kf) {
  CHECK_NOTNULL(ref_stereo_frame);
  CHECK_NOTNULL(cur_stereo_frame);

  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      *ref_stereo_frame, *cur_stereo_frame, &matches_ref_cur);

  std::vector<int> inliers;
  TrackingStatusPose result =
      geometricOutlierRejection3d3d(ref_stereo_frame->keypoints_3d_,
                                    cur_stereo_frame->keypoints_3d_,
                                    matches_ref_cur,
                                    &inliers,
                                    cam_lkf_Pose_cam_kf);

  if (result.first != TrackingStatus::INVALID) {
    //! Remove correspondences classified as outliers
    removeOutliersStereo(
        inliers, ref_stereo_frame, cur_stereo_frame, &matches_ref_cur);
  }
  return result;
}

// And don't use pairs to return :/
std::pair<Vector3, Matrix3> Tracker::getPoint3AndCovariance(
    const StatusKeypointsCV& keypoints_undistorted_left,
    const StatusKeypointsCV& keypoints_undistorted_right,
    const BearingVectors& keypoints_3d,
    const gtsam::StereoCamera& stereo_cam,
    const size_t point_id,
    const gtsam::Matrix3& stereo_point_covariance,
    std::optional<gtsam::Matrix3> Rmat) {
  // uL_, uR_, v_;
  gtsam::StereoPoint2 stereo_point = gtsam::StereoPoint2(
      static_cast<double>(keypoints_undistorted_left[point_id].second.x),
      static_cast<double>(keypoints_undistorted_right[point_id].second.x),
      static_cast<double>(keypoints_undistorted_left[point_id].second.y));

  gtsam::Matrix3 Jac_point3_sp2;  // jacobian of the back projection
  // TODO(nathan) think about just getting both jacobians
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
  Landmark point3_i_gtsam =
      stereo_cam.backproject2(stereo_point, boost::none, Jac_point3_sp2);
#else
  Landmark point3_i_gtsam =
      stereo_cam.backproject2(stereo_point, nullptr, Jac_point3_sp2);
#endif
  Landmark point3_i = keypoints_3d.at(point_id);
  // TODO(Toni): Adapt value of this threshold for different calibration
  // models! // (1e-1)
  if ((point3_i_gtsam - point3_i).norm() > 1e-1) {
    if (VLOG_IS_ON(5)) {
      LOG(WARNING) << "GetPoint3AndCovariance: inconsistent "
                   << "backprojection results: \n"
                   << " - point3_i_gtsam: \n"
                   << point3_i_gtsam << '\n'
                   << " - point3_i: \n"
                   << point3_i << '\n'
                   << "Difference in norm: "
                   << (point3_i_gtsam - point3_i).norm();
    }
  }
  if (Rmat) {
    point3_i = (*Rmat) * point3_i;  // optionally rotated to another ref frame
    Jac_point3_sp2 = (*Rmat) * Jac_point3_sp2;
  }

  gtsam::Matrix3 cov_i =
      Jac_point3_sp2 * stereo_point_covariance * Jac_point3_sp2.transpose();
  return std::make_pair(point3_i, cov_i);
}

std::pair<Vector3, Matrix3> Tracker::getPoint3AndCovariance(
    const StereoFrame& stereo_frame,
    const gtsam::StereoCamera& stereo_cam,
    const size_t point_id,
    const gtsam::Matrix3& stereo_point_covariance,
    std::optional<gtsam::Matrix3> Rmat) {
  return Tracker::getPoint3AndCovariance(
      stereo_frame.left_keypoints_rectified_,
      stereo_frame.right_keypoints_rectified_,
      stereo_frame.keypoints_3d_,
      stereo_cam,
      point_id,
      stereo_point_covariance,
      Rmat);
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
    if (k < inliers.size()                    // If we haven't exhaused inliers
        && static_cast<int>(i) > inliers[k])  // If we are after the inlier[k]
      ++k;                                    // Check the next inlier
    if (k >= inliers.size() ||
        static_cast<int>(i) != inliers[k])  // If i is not an inlier
      outliers->push_back(i);
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

  // Remove outliers: outliers cannot be a vector of size_t because opengv
  // uses a vector of int.
  for (const size_t& out : outliers) {
    const KeypointMatch& kp_match = (*matches_ref_cur)[out];
    ref_stereoFrame->right_keypoints_rectified_.at(kp_match.first).first =
        KeypointStatus::FAILED_ARUN;
    ref_stereoFrame->keypoints_depth_.at(kp_match.first) = 0.0;
    ref_stereoFrame->keypoints_3d_.at(kp_match.first) = Vector3::Zero();

    cur_stereoFrame->right_keypoints_rectified_.at(kp_match.second).first =
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
  findMatchingKeypoints(ref_stereoFrame.left_frame_,
                        cur_stereoFrame.left_frame_,
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
    if (ref_stereoFrame.right_keypoints_rectified_[ind_ref].first ==
            KeypointStatus::VALID &&
        cur_stereoFrame.right_keypoints_rectified_[ind_cur].first ==
            KeypointStatus::VALID) {
      // Pair of points that has 3D in both stereoFrames.
      matches_ref_cur_stereo->push_back(matches_ref_cur_mono[i]);
    } else {
      VLOG(5) << "Failed match status: "
              << to_underlying(
                     ref_stereoFrame.right_keypoints_rectified_[ind_ref].first)
              << " "
              << to_underlying(
                     cur_stereoFrame.right_keypoints_rectified_[ind_cur].first);
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

cv::Mat Tracker::getTrackerImage(const Frame& ref_frame,
                                 const Frame& cur_frame,
                                 const KeypointsCV& extra_corners_gray,
                                 const KeypointsCV& extra_corners_blue) const {
  cv::Mat img_rgb(cur_frame.img_.size(), CV_8U);
  cv::cvtColor(cur_frame.img_, img_rgb, cv::COLOR_GRAY2RGB);

  static const cv::Scalar gray(0, 255, 255);
  static const cv::Scalar red(0, 0, 255);
  static const cv::Scalar green(0, 255, 0);
  static const cv::Scalar blue(255, 0, 0);

  // Add extra corners if desired.
  for (const auto& px : extra_corners_gray) {
    cv::circle(img_rgb, px, 4, gray, 2);
  }
  for (const auto& px : extra_corners_blue) {
    cv::circle(img_rgb, px, 4, blue, 2);
  }

  // Add all keypoints in cur_frame with the tracks.
  for (size_t i = 0; i < cur_frame.keypoints_.size(); ++i) {
    const cv::Point2f& px_cur = cur_frame.keypoints_.at(i);
    if (cur_frame.landmarks_.at(i) == -1) {  // Untracked landmarks are red.
      cv::circle(img_rgb, px_cur, 4, red, 2);
    } else {
      const auto& it = std::find(ref_frame.landmarks_.begin(),
                                 ref_frame.landmarks_.end(),
                                 cur_frame.landmarks_.at(i));
      if (it != ref_frame.landmarks_.end()) {
        // If feature was in previous frame, display tracked feature with
        // green circle/line:
        cv::circle(img_rgb, px_cur, 6, green, 1);
        int i = std::distance(ref_frame.landmarks_.begin(), it);
        const cv::Point2f& px_ref = ref_frame.keypoints_.at(i);
        cv::arrowedLine(img_rgb, px_ref, px_cur, green, 1);
      } else {  // New feature tracks are blue.
        cv::circle(img_rgb, px_cur, 6, blue, 1);
      }
    }
  }
  return img_rgb;
}

bool Tracker::pnp(const StereoFrame& cur_stereo_frame,
                  gtsam::Pose3* W_Pose_cam_estimate,
                  std::vector<int>* inliers,
                  gtsam::Pose3* W_Pose_cam_prior) {
  CHECK_NOTNULL(W_Pose_cam_estimate);

  opengv::bearingVectors_t cam_bearing_vectors;
  opengv::points_t W_points;

  //! Copy landmarks map since otw we may block backend thread. This assumes
  //! copying the whole map is quicker than spending time finding the lmks we
  //! need. Call this as late as possible, so the backend has maximum time.
  LandmarksMap copy_W_landmarks_map;
  {  // Safe-guard the landmarks_map_
    std::lock_guard<std::mutex> lock(landmarks_map_mtx_);
    copy_W_landmarks_map = landmarks_map_;
  }

  // This is horrible, because we are weirdly looping over right_keypoints_rect
  // to know what landmarks are valid and tracked... Re-do this after PR #420...
  // How it should be done: loop over feature tracks alone.
  CHECK_EQ(cur_stereo_frame.left_keypoints_rectified_.size(),
           cur_stereo_frame.left_frame_.landmarks_.size());
  for (size_t i = 0; i < cur_stereo_frame.left_keypoints_rectified_.size();
       i++) {
    const auto& lmk_id = cur_stereo_frame.left_frame_.landmarks_.at(i);
    if (cur_stereo_frame.left_keypoints_rectified_[i].first ==
            KeypointStatus::VALID &&
        lmk_id != -1) {  // why is lmk_id -1 if KeypointStatus is VALID?
      //! We have a valid lmk id.
      //! Query our map of optimized landmarks.
      const auto& it = copy_W_landmarks_map.find(lmk_id);
      if (it != copy_W_landmarks_map.end()) {
        //! We found our landmark, store its value.
        W_points.push_back(it->second);
        cam_bearing_vectors.push_back(cur_stereo_frame.keypoints_3d_[i]);
      } else {
        //! The landmark is not in our time-horizon.
        VLOG(5) << "Landmark Id " << lmk_id << " is out of time-horizon.";
      }
    } else {
      // CHECK_EQ(lmk_id, -1); why is this not true :(
      //! Not interested in this 2D-3D because the right keypoint is not valid
      //! and/or the lmk_id is invalid...
      // NOT ideal because we are dropping valuable info for the mono case...
      VLOG(5) << "Dropping 2D-3D correspondence: " << i;
    }
  }

  bool success = pnp(cam_bearing_vectors,
                     W_points,
                     W_Pose_cam_estimate,
                     inliers,
                     W_Pose_cam_prior);

  return success;
}

bool Tracker::pnp(const BearingVectors& cam_bearing_vectors,
                  const Landmarks& F_points,
                  gtsam::Pose3* F_Pose_cam_estimate,
                  std::vector<int>* inliers,
                  gtsam::Pose3* F_Pose_cam_prior) {
  auto start_time_tic = utils::Timer::tic();
  bool success = false;

  if (F_points.size() == 0) {
    LOG(WARNING) << "No 2D-3D correspondences found for 2D-3D RANSAC...";
    *F_Pose_cam_estimate = gtsam::Pose3();
    *inliers = {};
    success = false;
  } else {
    // Create the central adapter
    CHECK_EQ(cam_bearing_vectors.size(), F_points.size());
    AdapterPnp adapter(cam_bearing_vectors, F_points);

    // Should be similar to current klt_eps, but keep it separate.
    const double reprojection_error = tracker_params_.ransac_threshold_pnp_;
    const double avg_focal_length =
        0.5 * static_cast<double>(camera_->getCamParams().intrinsics_[0] +
                                  camera_->getCamParams().intrinsics_[1]);
    const double threshold =
        1.0 - std::cos(std::atan(std::sqrt(2.0) * reprojection_error /
                                 avg_focal_length));
    VLOG_EVERY_N(5, 100) << "PnP params:\n"
                         << "- Reprojection error: " << reprojection_error
                         << '\n'
                         << "- Focal Length: " << avg_focal_length << '\n'
                         << "- Threshold: " << threshold << '\n'
                         << "- Ransac Max Iters: "
                         << tracker_params_.ransac_max_iterations_ << '\n'
                         << "- Ransac Probability : "
                         << tracker_params_.ransac_probability_ << '\n'
                         << "- Optimize inliers : "
                         << tracker_params_.optimize_2d3d_pose_from_inliers_
                         << '\n'
                         << "- PnP algorithm: "
                         << VIO::to_underlying(tracker_params_.pnp_algorithm_);

    switch (tracker_params_.pnp_algorithm_) {
      case Pose3d2dAlgorithm::KneipP2P: {
        // Uses rotation prior from adapter
        CHECK(F_Pose_cam_prior);
        opengv::rotation_t rotation_prior =
            F_Pose_cam_prior->rotation().matrix();
        adapter.setR(rotation_prior);
        success = runRansac(
            std::make_shared<ProblemPnP>(
                adapter, ProblemPnP::TWOPT, tracker_params_.ransac_randomize_),
            threshold,
            tracker_params_.ransac_max_iterations_,
            tracker_params_.ransac_probability_,
            tracker_params_.optimize_2d3d_pose_from_inliers_,
            F_Pose_cam_estimate,
            inliers);
        break;
      }
      case Pose3d2dAlgorithm::KneipP3P: {
        success = runRansac(
            std::make_shared<ProblemPnP>(
                adapter, ProblemPnP::KNEIP, tracker_params_.ransac_randomize_),
            threshold,
            tracker_params_.ransac_max_iterations_,
            tracker_params_.ransac_probability_,
            tracker_params_.optimize_2d3d_pose_from_inliers_,
            F_Pose_cam_estimate,
            inliers);
        break;
      }
      case Pose3d2dAlgorithm::GaoP3P: {
        success = runRansac(
            std::make_shared<ProblemPnP>(
                adapter, ProblemPnP::GAO, tracker_params_.ransac_randomize_),
            threshold,
            tracker_params_.ransac_max_iterations_,
            tracker_params_.ransac_probability_,
            tracker_params_.optimize_2d3d_pose_from_inliers_,
            F_Pose_cam_estimate,
            inliers);
        break;
      }
      case Pose3d2dAlgorithm::EPNP: {
        success = runRansac(
            std::make_shared<ProblemPnP>(
                adapter, ProblemPnP::EPNP, tracker_params_.ransac_randomize_),
            threshold,
            tracker_params_.ransac_max_iterations_,
            tracker_params_.ransac_probability_,
            tracker_params_.optimize_2d3d_pose_from_inliers_,
            F_Pose_cam_estimate,
            inliers);
        break;
      }
      case Pose3d2dAlgorithm::UPNP: {
        LOG_IF(WARNING, inliers)
            << "UPNP expects outlier free correspondences.";
        // Uses all correspondences.
        const opengv::transformations_t& upnp_transformations =
            opengv::absolute_pose::upnp(adapter);
        // TODO(TONI): what about the rest of transformations?
        CHECK_GT(upnp_transformations.size(), 0);
        *F_Pose_cam_estimate = Eigen::MatrixXd(upnp_transformations[0]);
        success = true;
        break;
      }
      case Pose3d2dAlgorithm::UP3P: {
        CHECK(inliers);
        LOG_IF(FATAL, inliers->empty()) << "UP3P needs to know the inliers.";
        // Uses three correspondences.
        const opengv::transformations_t& upnp_transformations =
            opengv::absolute_pose::upnp(adapter, *inliers);
        // TODO(TONI): what about the rest of transformations?
        CHECK_GT(upnp_transformations.size(), 0);
        *F_Pose_cam_estimate = Eigen::MatrixXd(upnp_transformations[0]);
        success = true;
        break;
      }
      case Pose3d2dAlgorithm::NonlinearOptimization: {
        CHECK(inliers);
        LOG_IF(FATAL, inliers->empty())
            << "NonlinearOptimization needs to know the inliers.";
        // Uses all correspondences.
        CHECK(F_Pose_cam_prior);
        opengv::rotation_t rotation_prior =
            F_Pose_cam_prior->rotation().matrix();
        opengv::translation_t translation_prior =
            F_Pose_cam_prior->translation().matrix();
        adapter.setR(rotation_prior);
        adapter.sett(translation_prior);
        *F_Pose_cam_estimate = Eigen::MatrixXd(
            opengv::absolute_pose::optimize_nonlinear(adapter, *inliers));
        success = true;
        break;
      }
      case Pose3d2dAlgorithm::MLPNP: {
        // TODO(TONI): needs fork of opengv, can we make a static check and use
        // this iff we are having MLPNP support?
        LOG(FATAL) << "MLPNP Not implemented...";
        break;
      }
      default: {
        LOG(ERROR) << "Unknown PnP method selected: "
                   << VIO::to_underlying(tracker_params_.pnp_algorithm_);
        success = false;
        break;
      }
    }

    //! Fill debug info.
    debug_info_.nrStereoPutatives_ = adapter.getNumberCorrespondences();
    debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
    debug_info_.nrStereoInliers_ = inliers->size();
    debug_info_.stereoRansacIters_ = 0;  // no access to ransac from here
    // debug_info_.stereoRansacIters_ = iterations;
  }

  VLOG(5) << "PnP tracking " << (success ? " success " : " failure ") << ":\n"
          << "- Total Correspondences: " << F_points.size() << '\n'
          << "\t- # inliers: " << inliers->size() << '\n'
          << "\t- # outliers: " << F_points.size() - inliers->size() << '\n'
          << "- Estimated pose: \n"
          << *F_Pose_cam_estimate;

  return success;
}

}  // namespace VIO
