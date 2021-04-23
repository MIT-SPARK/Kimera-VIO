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

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

Tracker::Tracker(const FrontendParams& tracker_params,
                 const Camera::ConstPtr& camera,
                 DisplayQueue* display_queue)
    : landmark_count_(0),
      tracker_params_(tracker_params),
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
void Tracker::featureTracking(Frame* ref_frame,
                              Frame* cur_frame,
                              const gtsam::Rot3& ref_R_cur,
                              boost::optional<cv::Mat> R) {
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
    if (!status[i] || lmk_age > tracker_params_.maxFeatureAge_) {
      // we are marking this bad in the ref_frame since features
      // in the ref frame guide feature detection later on
      ref_frame->landmarks_[idx_valid_lmk] = -1;
      continue;
    }
    cur_frame->landmarks_.push_back(lmk_id);
    cur_frame->landmarks_age_.push_back(lmk_age);
    cur_frame->scores_.push_back(ref_frame->scores_[idx_valid_lmk]);
    cur_frame->keypoints_.push_back(px_cur[i]);
    cur_frame->versors_.push_back(
        UndistorterRectifier::UndistortKeypointAndGetVersor(px_cur[i], ref_frame->cam_param_, R));
  }

  // max number of frames in which a feature is seen
  VLOG(5) << "featureTracking: frame " << cur_frame->id_
           << ",  Nr tracked keypoints: " << cur_frame->keypoints_.size()
           << " (max: "
           << tracker_params_.feature_detector_params_.max_features_per_frame_
           << ")"
           << " (max observed age of tracked features: "
           << *std::max_element(cur_frame->landmarks_age_.begin(),
                                cur_frame->landmarks_age_.end())
           << " vs. maxFeatureAge_: " << tracker_params_.maxFeatureAge_ << ")";
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

// TODO(Toni): this function is almost a replica of the Stereo version,
// factorize.
std::pair<TrackingStatus, gtsam::Pose3> Tracker::geometricOutlierRejectionMono(
    Frame* ref_frame,
    Frame* cur_frame) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  // Get bearing vectors for open_gv.
  const size_t& n_matches = matches_ref_cur.size();
  BearingVectors f_cur;
  f_cur.reserve(n_matches);
  BearingVectors f_ref;
  f_ref.reserve(n_matches);
  for (const KeypointMatch& kp_ref_kp_cur : matches_ref_cur) {
    // TODO(Toni) (luca): if versors are only needed at keyframe,
    // do not compute every frame
    f_ref.push_back(ref_frame->versors_.at(kp_ref_kp_cur.first));
    f_cur.push_back(cur_frame->versors_.at(kp_ref_kp_cur.second));
  }

  // Setup problem.
  AdapterMono adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMono> problem = std::make_shared<ProblemMono>(
      adapter, ProblemMono::NISTER, tracker_params_.ransac_randomize_);

  // Update new problem for monocular ransac.
  mono_ransac_.sac_model_ = problem;

  // Solve.
  if (!mono_ransac_.computeModel(0)) {
    VLOG(5) << "failure: 5pt RANSAC could not find a solution.";
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3::identity());
  }

  VLOG(5) << "geometricOutlierRejectionMono: RANSAC complete.";

  VLOG(5) << "RANSAC (MONO): #iter = " << mono_ransac_.iterations_ << '\n'
           << " #inliers = " << mono_ransac_.inliers_.size() << " #outliers = "
           << mono_ransac_.inliers_.size() - matches_ref_cur.size();
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();

  // Remove outliers. This modifies the frames, that is why this function does
  // not simply accept const Frames. And removes outliers from matches.
  removeOutliersMono(
      mono_ransac_.inliers_, ref_frame, cur_frame, &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (mono_ransac_.inliers_.size() < tracker_params_.minNrMonoInliers_) {
    VLOG(5) << "FEW_MATCHES: " << mono_ransac_.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }

  double disparity;
  bool median_disparity_success = computeMedianDisparity(ref_frame->keypoints_,
                                                         cur_frame->keypoints_,
                                                         matches_ref_cur,
                                                         &disparity);
  LOG_IF(ERROR, !median_disparity_success)
      << "Median disparity calculation failed...";
  VLOG(5) << "Median disparity: " << disparity;
  if (disparity < tracker_params_.disparityThreshold_) {
    LOG(WARNING) << "LOW_DISPARITY: " << disparity;
    status = TrackingStatus::LOW_DISPARITY;
  } else {
    // Check for rotation only case
    // optical_flow_predictor_->predictFlow(ref_frame->keypoints_,
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  const opengv::transformation_t& best_transformation =
      mono_ransac_.model_coefficients_;

  debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrMonoInliers_ = mono_ransac_.inliers_.size();
  debug_info_.monoRansacIters_ = mono_ransac_.iterations_;

  return std::make_pair(status,
                        UtilsOpenCV::openGvTfToGtsamPose3(best_transformation));
}

std::pair<TrackingStatus, gtsam::Pose3>
Tracker::geometricOutlierRejectionMonoGivenRotation(
    Frame* ref_frame,
    Frame* cur_frame,
    const gtsam::Rot3& camLrectlkf_R_camLrectkf) {
  CHECK_NOTNULL(ref_frame);
  CHECK_NOTNULL(cur_frame);

  // To log the time taken to perform this function.
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  // NOTE: versors are already in the rectified left camera frame.
  // No further rectification needed.

  // Vector of bearing vectors.
  BearingVectors f_cur;
  f_cur.reserve(matches_ref_cur.size());
  BearingVectors f_ref;
  f_ref.reserve(matches_ref_cur.size());
  for (const KeypointMatch& it : matches_ref_cur) {
    f_ref.push_back(ref_frame->versors_.at(it.first));
    f_cur.push_back(
        camLrectlkf_R_camLrectkf.rotate(cur_frame->versors_.at(it.second)));
  }

  // Setup problem.
  AdapterMonoGivenRot adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMonoGivenRot> problem =
      std::make_shared<ProblemMonoGivenRot>(adapter,
                                            tracker_params_.ransac_randomize_);
  mono_ransac_given_rot_.sac_model_ = problem;

  VLOG(5) << "geometricOutlierRejectionMonoGivenRot: starting 2-point RANSAC";

  // Solve.
  if (!mono_ransac_given_rot_.computeModel(0)) {
    LOG(WARNING) << "2-point RANSAC could not find a solution!";
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3::identity());
  }
  VLOG(5) << "geometricOutlierRejectionMonoGivenRot: RANSAC complete";

  VLOG(5) << "RANSAC (MONO): #iter = " << mono_ransac_given_rot_.iterations_
          << '\n'
          << " #inliers = " << mono_ransac_given_rot_.inliers_.size()
          << "\n #outliers = "
          << matches_ref_cur.size() - mono_ransac_given_rot_.inliers_.size()
          << "\n Total = " << matches_ref_cur.size();
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  debug_info_.nrMonoPutatives_ = matches_ref_cur.size();  // before cleaning.
  removeOutliersMono(
      mono_ransac_given_rot_.inliers_, ref_frame, cur_frame, &matches_ref_cur);

  // TODO(Toni):
  // CHECK QUALITY OF TRACKING
  TrackingStatus status = TrackingStatus::VALID;
  if (mono_ransac_given_rot_.inliers_.size() <
      tracker_params_.minNrMonoInliers_) {
    VLOG(5) << "FEW_MATCHES: " << mono_ransac_given_rot_.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }
  double disparity;
  bool median_disparity_success = computeMedianDisparity(ref_frame->keypoints_,
                                                         cur_frame->keypoints_,
                                                         matches_ref_cur,
                                                         &disparity);
  LOG_IF(ERROR, !median_disparity_success)
      << "Median disparity calculation failed...";

  VLOG(5) << "median disparity " << disparity;
  if (disparity < tracker_params_.disparityThreshold_) {
    VLOG(5) << "LOW_DISPARITY: " << disparity;
    status = TrackingStatus::LOW_DISPARITY;
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  opengv::transformation_t best_transformation =
      mono_ransac_given_rot_.model_coefficients_;
  gtsam::Pose3 camLrectlkf_P_camLrectkf =
      UtilsOpenCV::openGvTfToGtsamPose3(best_transformation);
  // note: this always returns the identity rotation, hence we have to
  // substitute it:
  camLrectlkf_P_camLrectkf = gtsam::Pose3(
      camLrectlkf_R_camLrectkf, camLrectlkf_P_camLrectkf.translation());

  debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrMonoInliers_ = mono_ransac_given_rot_.inliers_.size();
  debug_info_.monoRansacIters_ = mono_ransac_given_rot_.iterations_;

  return std::make_pair(status, camLrectlkf_P_camLrectkf);
}

std::pair<Vector3, Matrix3> Tracker::getPoint3AndCovariance(
    const StereoFrame& stereoFrame,
    const gtsam::StereoCamera& stereoCam,
    const size_t pointId,
    const Matrix3& stereoPtCov,
    boost::optional<gtsam::Matrix3> Rmat) {
  gtsam::StereoPoint2 stereoPoint = gtsam::StereoPoint2(
      static_cast<double>(stereoFrame.left_keypoints_rectified_[pointId].second.x),
      static_cast<double>(stereoFrame.right_keypoints_rectified_[pointId].second.x),
      static_cast<double>(
          stereoFrame.left_keypoints_rectified_[pointId].second.y));  // uL_, uR_, v_;

  Matrix3 Jac_point3_sp2;  // jacobian of the back projection
  Vector3 point3_i_gtsam =
      stereoCam.backproject2(stereoPoint, boost::none, Jac_point3_sp2);
  Vector3 point3_i = stereoFrame.keypoints_3d_.at(pointId);
  // TODO(Toni): Adapt value of this threshold for different calibration
  // models!
  // (1e-1)
  if ((point3_i_gtsam - point3_i).norm() > 1e-1) {
    VLOG(5) << "\n point3_i_gtsam \n " << point3_i_gtsam << "\n point3_i \n"
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
    VIO::StereoCamera::ConstPtr stereo_camera,
    const gtsam::Rot3& camLrectlkf_R_camLrectkf) {
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      ref_stereoFrame, cur_stereoFrame, &matches_ref_cur);

  VLOG(5) << "geometricOutlierRejectionStereoGivenRot:"
              " starting 1-point RANSAC (voting)";

  // Stereo point covariance: for covariance propagation.
  Matrix3 stereoPtCov = Matrix3::Identity();  // 3 px std in each direction

  // Create stereo camera in the ref frame of the left camera.
  gtsam::StereoCamera stereoCam(gtsam::Pose3::identity(),
                                stereo_camera->getStereoCalib());

  double timeMatchingAndAllocation_p =
      utils::Timer::toc(start_time_tic).count();

  //============================================================================
  // CREATE DATA STRUCTURES
  //============================================================================
  auto timeCreatePointsAndCov_p_tic = utils::Timer::tic();
  size_t nrMatches = matches_ref_cur.size();

  // NOTE: 3d points are constructed by versors, which are already in the
  // rectified left camera frame. No further rectification needed.
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
    std::tie(R_f_cur_i, cov_R_cur_i) =
        Tracker::getPoint3AndCovariance(cur_stereoFrame,
                                        stereoCam,
                                        it.second,
                                        stereoPtCov,
                                        camLrectlkf_R_camLrectkf.matrix());

    // Populate relative translation estimates and their covariances.
    Vector3 v = f_ref_i - R_f_cur_i;
    Matrix3 M = cov_R_cur_i + cov_ref_i;

    relTran.push_back(v);
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

  double time_voting_p = utils::Timer::toc(time_voting_tic).count();

  VLOG(5) << "geometricOutlierRejectionStereoGivenRot: voting complete.";

  //============================================================================
  // OUTLIER REJECTION AND TRANSLATION COMPUTATION
  //============================================================================
  if (maxCoherentSetSize < 2) {
    LOG(WARNING) << "1-point RANSAC (voting) could not find a solution.";
    return std::make_pair(
        std::make_pair(TrackingStatus::INVALID, gtsam::Pose3::identity()),
        gtsam::Matrix3::Zero());
  }

  // Inliers are max coherent set.
  std::vector<int> inliers = coherentSet.at(maxCoherentSetId);

  // Sort inliers.
  std::sort(inliers.begin(), inliers.end());
  // UtilsOpenCV::PrintVector<int>(inliers,"inliers");
  // std::cout << "maxCoherentSetId: " << maxCoherentSetId << std::endl;

  VLOG(5) << "RANSAC (STEREO): #iter = " << 1 << '\n'
           << " #inliers = " << inliers.size()
           << "\n #outliers = " << inliers.size() - matches_ref_cur.size()
           << "\n Total = " << matches_ref_cur.size();
  debug_info_.nrStereoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  removeOutliersStereo(
      inliers, &ref_stereoFrame, &cur_stereoFrame, &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (inliers.size() < tracker_params_.minNrStereoInliers_) {
    VLOG(5) << "FEW_MATCHES: " << inliers.size();
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
    double time_translation_computation_p =
        utils::Timer::toc(start_time_tic).count();
    VLOG(5) << " Time MatchingAndAllocation: " << timeMatchingAndAllocation_p
             << " Time CreatePointsAndCov: " << timeCreatePointsAndCov_p
             << " Time Voting: " << time_voting_p
             << " Time translation computation p: "
             << time_translation_computation_p;
  }
  debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrStereoInliers_ = inliers.size();
  debug_info_.stereoRansacIters_ = 1;  // this is bcs we use coherent sets here.

  return std::make_pair(
      std::make_pair(status,
                     gtsam::Pose3(camLrectlkf_R_camLrectkf, gtsam::Point3(t))),
      totalInfo.cast<double>());
}

// TODO(Toni): this function is almost a replica of the Mono version,
// factorize.
std::pair<TrackingStatus, gtsam::Pose3>
Tracker::geometricOutlierRejectionStereo(StereoFrame& ref_stereoFrame,
                                         StereoFrame& cur_stereoFrame) {
  auto start_time_tic = utils::Timer::tic();

  KeypointMatches matches_ref_cur;
  findMatchingStereoKeypoints(
      ref_stereoFrame, cur_stereoFrame, &matches_ref_cur);

  VLOG(5) << "geometricOutlierRejectionStereo:"
              " starting 3-point RANSAC (voting)";

  // NOTE: 3d points are constructed by versors, which are already in the
  // rectified left camera frame. No further rectification needed.

  // Vector of 3D vectors
  const size_t& n_matches = matches_ref_cur.size();
  BearingVectors f_cur;
  f_cur.reserve(n_matches);
  BearingVectors f_ref;
  f_ref.reserve(n_matches);
  for (const KeypointMatch& it : matches_ref_cur) {
    f_ref.push_back(ref_stereoFrame.keypoints_3d_.at(it.first));
    f_cur.push_back(cur_stereoFrame.keypoints_3d_.at(it.second));
  }

  // Setup problem (3D-3D adapter) -
  // http://laurentkneip.github.io/opengv/page_how_to_use.html
  AdapterStereo adapter(f_ref, f_cur);
  std::shared_ptr<ProblemStereo> problem = std::make_shared<ProblemStereo>(
      adapter, tracker_params_.ransac_randomize_);

  // Update new problem for stereo ransac.
  stereo_ransac_.sac_model_ = problem;

  // Solve.
  if (!stereo_ransac_.computeModel(0)) {
    LOG(WARNING) << "failure: (Arun) RANSAC could not find a solution."
                 << "\n  size of matches_ref_cur: " << matches_ref_cur.size()
                 << "\n  size of f_ref: " << f_ref.size()
                 << "\n  size of f_cur: " << f_cur.size();
    return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3::identity());
  }

  VLOG(5) << "geometricOutlierRejectionStereo: voting complete.";

  VLOG(5) << "RANSAC (STEREO): #iter = " << stereo_ransac_.iterations_ << '\n'
           << " #inliers = " << stereo_ransac_.inliers_.size()
           << "\n #outliers = "
           << stereo_ransac_.inliers_.size() - matches_ref_cur.size();
  debug_info_.nrStereoPutatives_ = matches_ref_cur.size();

  // Remove outliers.
  removeOutliersStereo(stereo_ransac_.inliers_,
                       &ref_stereoFrame,
                       &cur_stereoFrame,
                       &matches_ref_cur);

  // Check quality of tracking.
  TrackingStatus status = TrackingStatus::VALID;
  if (stereo_ransac_.inliers_.size() < tracker_params_.minNrStereoInliers_) {
    VLOG(5) << "FEW_MATCHES: " << stereo_ransac_.inliers_.size();
    status = TrackingStatus::FEW_MATCHES;
  }

  // Get the resulting transformation: a 3x4 matrix [R t].
  const opengv::transformation_t& best_transformation =
      stereo_ransac_.model_coefficients_;

  // Fill debug info.
  debug_info_.stereoRansacTime_ = utils::Timer::toc(start_time_tic).count();
  debug_info_.nrStereoInliers_ = stereo_ransac_.inliers_.size();
  debug_info_.stereoRansacIters_ = stereo_ransac_.iterations_;

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

}  // namespace VIO
