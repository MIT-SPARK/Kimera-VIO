/**
 * @file   Tracker.cpp
 * @brief  Class describing temporal tracking
 * @author Antoni Rosinol, Luca Carlone
 */

#include "kimera-vio/frontend/Tracker.h"

#include <string>
#include <algorithm>   // for sort
#include <map>         // for map<>
#include <memory>      // for shared_ptr<>
#include <utility>     // for pair<>
#include <vector>      // for vector<>
#include <functional>  // for less<>

#define TRACKER_VERBOSITY 0  // Should be 1

namespace VIO {

Tracker::Tracker(const VioFrontEndParams& trackerParams)
    : trackerParams_(trackerParams),
      // Only for debugging and visualization:
      outputImagesPath_("./outputImages/"),
      landmark_count_(0),
      pixelOffset_(),
      verbosity_(TRACKER_VERBOSITY) {}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
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
    cur_frame->landmarksAge_.at(i)++;
  }

  // Detect new features in image.
  // detect this much new corners if possible
  int nr_corners_needed =
      std::max(trackerParams_.maxFeaturesPerFrame_ - n_existing, 0);
  debugInfo_.need_n_corners_ = nr_corners_needed;

  ///////////////// FEATURE DETECTION //////////////////////
  // If feature FeatureSelectionCriterion is quality, just extract what you
  // need:
  double start_time = UtilsOpenCV::GetTimeInSeconds();
  auto corners_with_scores = Tracker::featureDetection(
      *cur_frame, trackerParams_, camMask_, nr_corners_needed);
  if (verbosity_ >= 5) {
    debugInfo_.featureDetectionTime_ =
        UtilsOpenCV::GetTimeInSeconds() - start_time;
  }
  debugInfo_.extracted_corners_ = corners_with_scores.first.size();

  ///////////////// STORE NEW KEYPOINTS  //////////////////////
  // Store features in our Frame
  size_t nrExistingKeypoints =
      cur_frame->keypoints_.size();  // for debug, these are the ones tracked
                                     // from the previous frame
  cur_frame->landmarks_.reserve(cur_frame->keypoints_.size() +
                                corners_with_scores.first.size());
  cur_frame->landmarksAge_.reserve(cur_frame->keypoints_.size() +
                                   corners_with_scores.first.size());
  cur_frame->keypoints_.reserve(cur_frame->keypoints_.size() +
                                corners_with_scores.first.size());
  cur_frame->scores_.reserve(cur_frame->scores_.size() +
                             corners_with_scores.second.size());
  cur_frame->versors_.reserve(cur_frame->keypoints_.size() +
                              corners_with_scores.first.size());

  // TODO(Toni) Fix this loop, very unefficient. Use std::move over keypoints
  // with scores.
  for (size_t i = 0; i < corners_with_scores.first.size(); i++) {
    cur_frame->landmarks_.push_back(landmark_count_);
    cur_frame->landmarksAge_.push_back(1);  // seen in a single (key)frame
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
           << "  (max: " << trackerParams_.maxFeaturesPerFrame_ << ")";
  }

  /* --------------------------------------------------------------------------
   */
  std::pair<KeypointsCV, std::vector<double>> Tracker::featureDetection(
      const Frame& cur_frame,
      const VioFrontEndParams& tracker_params,
      const cv::Mat& cam_mask,
      const int need_n_corners) {
    // Create mask such that new keypoints are not close to old ones.
    cv::Mat mask;
    cam_mask.copyTo(mask);
    for (size_t i = 0; i < cur_frame.keypoints_.size(); ++i) {
      if (cur_frame.landmarks_.at(i) != -1) {
        cv::circle(mask, cur_frame.keypoints_.at(i),
                   tracker_params.min_distance_, cv::Scalar(0), CV_FILLED);
      }
    }

    // Find new features and corresponding scores.
    std::pair<KeypointsCV, std::vector<double>> corners_with_scores;
    if (need_n_corners > 0) {
      UtilsOpenCV::MyGoodFeaturesToTrackSubPix(
          cur_frame.img_,
          need_n_corners,
          tracker_params.quality_level_,
          tracker_params.min_distance_,
          mask,
          tracker_params.block_size_,
          tracker_params.use_harris_detector_,
          tracker_params.k_,
          &corners_with_scores);
    }

    return corners_with_scores;
  }

  /* --------------------------------------------------------------------------
   */
  // TODO(Toni) a pity that this function is not const just because
  // it modifies debuginfo_...
  void Tracker::featureTracking(Frame * ref_frame, Frame * cur_frame) {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    double start_time = UtilsOpenCV::GetTimeInSeconds();  // Log timing.

    // Fill up structure for reference pixels and their labels.
    KeypointsCV px_ref;
    std::vector<size_t> indices;
    indices.reserve(ref_frame->keypoints_.size());
    px_ref.reserve(ref_frame->keypoints_.size());
    for (size_t i = 0; i < ref_frame->keypoints_.size(); ++i) {
      if (ref_frame->landmarks_[i] != -1) {
        px_ref.push_back(ref_frame->keypoints_[i]);
        indices.push_back(i);
      }
    }

    // Setup termination criteria for optical flow.
    std::vector<uchar> status;
    std::vector<float> error;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                              trackerParams_.klt_max_iter_,
                              trackerParams_.klt_eps_);

    // Initialize to old locations
    KeypointsCV px_cur = px_ref;
    if (px_cur.size() > 0) {
      // Do the actual tracking, so px_cur becomes the new pixel locations.
      VLOG(2) << "Sarting Optical Flow Pyr LK tracking...";
      cv::calcOpticalFlowPyrLK(ref_frame->img_, cur_frame->img_, px_ref, px_cur,
                               status, error,
                               cv::Size2i(trackerParams_.klt_win_size_,
                                          trackerParams_.klt_win_size_),
                               trackerParams_.klt_max_level_, termcrit,
                               cv::OPTFLOW_USE_INITIAL_FLOW);
      VLOG(2) << "Finished Optical Flow Pyr LK tracking.";

      if (cur_frame->keypoints_.empty()) {  // Do we really need this check?
        cur_frame->landmarks_.reserve(px_ref.size());
        cur_frame->landmarksAge_.reserve(px_ref.size());
        cur_frame->keypoints_.reserve(px_ref.size());
        cur_frame->scores_.reserve(px_ref.size());
        cur_frame->versors_.reserve(px_ref.size());
        for (size_t i = 0, n = 0; i < indices.size(); ++i) {
          const size_t& i_ref = indices[i];
          // If we failed to track mark off that landmark
          if (!status[i] ||
              ref_frame->landmarksAge_[i_ref] >
                  trackerParams_
                      .maxFeatureAge_) {  // if we tracked keypoint and feature
                                          // track is not too long
            ref_frame->landmarks_[i_ref] =
                -1;  // we are marking this bad in the ref_frame since features
                     // in the ref frame guide feature detection later on
            continue;
          }
          cur_frame->landmarks_.push_back(ref_frame->landmarks_[i_ref]);
          cur_frame->landmarksAge_.push_back(ref_frame->landmarksAge_[i_ref]);
          cur_frame->scores_.push_back(ref_frame->scores_[i_ref]);
          cur_frame->keypoints_.push_back(px_cur[i]);
          cur_frame->versors_.push_back(
              Frame::calibratePixel(px_cur[i], ref_frame->cam_param_));
          ++n;
        }
        int maxAge = *std::max_element(
            cur_frame->landmarksAge_.begin(),
            cur_frame->landmarksAge_
                .end());  // max number of frames in which a feature is seen
        VLOG(10) << "featureTracking: frame " << cur_frame->id_
                 << ",  Nr tracked keypoints: " << cur_frame->keypoints_.size()
                 << " (max: " << trackerParams_.maxFeaturesPerFrame_ << ")"
                 << " (max observed age of tracked features: " << maxAge
                 << " vs. maxFeatureAge_: " << trackerParams_.maxFeatureAge_
                 << ")";
      }
    }
    debugInfo_.nrTrackerFeatures_ = cur_frame->keypoints_.size();
    if (verbosity_ >= 5)
      debugInfo_.featureTrackingTime_ =
          UtilsOpenCV::GetTimeInSeconds() - start_time;
  }

  /* --------------------------------------------------------------------------
   */
  std::pair<TrackingStatus, gtsam::Pose3>
  Tracker::geometricOutlierRejectionMono(Frame * ref_frame, Frame * cur_frame) {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    double start_time = UtilsOpenCV::GetTimeInSeconds();

    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

    // Vector of bearing vectors.
    BearingVectors f_cur;
    f_cur.reserve(matches_ref_cur.size());
    BearingVectors f_ref;
    f_ref.reserve(matches_ref_cur.size());
    for (const std::pair<size_t, size_t>& it : matches_ref_cur) {
      // TODO(Toni) (luca): if versors are only needed at keyframe,
      // do not compute every frame
      f_ref.push_back(ref_frame->versors_.at(it.first));
      f_cur.push_back(cur_frame->versors_.at(it.second));
    }

    // Setup problem.
    AdapterMono adapter(f_ref, f_cur);
    std::shared_ptr<ProblemMono> problem =
        std::make_shared<ProblemMono>(adapter, ProblemMono::NISTER,
                                      // last argument kills randomization
                                      trackerParams_.ransac_randomize_);
    opengv::sac::Ransac<ProblemMono> ransac;
    ransac.sac_model_ = problem;
    ransac.threshold_ = trackerParams_.ransac_threshold_mono_;
    ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
    ransac.probability_ = trackerParams_.ransac_probability_;

    VLOG(10) << "geometricOutlierRejectionMono: starting 5-point RANSAC.";

    // Solve.
    if (!ransac.computeModel(0)) {
      VLOG(10) << "failure: 5pt RANSAC could not find a solution.";
      return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
    }

    VLOG(10) << "geometricOutlierRejectionMono: RANSAC complete.";

    // Remove outliers. This modifies the frames, that is why this function does
    // not simply accept const Frames.
    removeOutliersMono(ref_frame, cur_frame, matches_ref_cur, ransac.inliers_,
                       ransac.iterations_);

    // Check quality of tracking.
    TrackingStatus status = TrackingStatus::VALID;
    if (ransac.inliers_.size() < trackerParams_.minNrMonoInliers_) {
      VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
      status = TrackingStatus::FEW_MATCHES;
    }

    double disparity = computeMedianDisparity(*ref_frame, *cur_frame);
    VLOG(10) << "Median disparity: " << disparity;
    if (disparity < trackerParams_.disparityThreshold_) {
      VLOG(10) << "LOW_DISPARITY: " << disparity;
      status = TrackingStatus::LOW_DISPARITY;
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

    if (verbosity_ >= 5) {
      debugInfo_.monoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - start_time;
    }
    debugInfo_.nrMonoInliers_ = ransac.inliers_.size();
    debugInfo_.nrMonoPutatives_ = matches_ref_cur.size();
    debugInfo_.monoRansacIters_ = ransac.iterations_;
    return std::make_pair(status, camLrectlkf_P_camLrectkf);
  }

  /* --------------------------------------------------------------------------
   */
  std::pair<TrackingStatus, gtsam::Pose3>
  Tracker::geometricOutlierRejectionMonoGivenRotation(
      Frame * ref_frame, Frame * cur_frame, const gtsam::Rot3& R) {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);

    // To log the time taken to perform this function.
    double start_time = UtilsOpenCV::GetTimeInSeconds();

    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

    // Vector of bearing vectors.
    BearingVectors f_cur;
    f_cur.reserve(matches_ref_cur.size());
    BearingVectors f_ref;
    f_ref.reserve(matches_ref_cur.size());
    for (const std::pair<size_t, size_t>& it : matches_ref_cur) {
      f_ref.push_back(ref_frame->versors_.at(it.first));
      f_cur.push_back(R.rotate(cur_frame->versors_.at(it.second)));
    }

    // Setup problem.
    AdapterMonoGivenRot adapter(f_ref, f_cur);
    std::shared_ptr<ProblemMonoGivenRot> problem =
        std::make_shared<ProblemMonoGivenRot>(adapter,
                                              trackerParams_.ransac_randomize_);
    opengv::sac::Ransac<ProblemMonoGivenRot> ransac;
    ransac.sac_model_ = problem;
    ransac.threshold_ = trackerParams_.ransac_threshold_mono_;
    ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
    ransac.probability_ = trackerParams_.ransac_probability_;

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
    std::vector<int> inliers = cv::ransac_2_point(
        f_ref, f_cur, trackerParams_.ransac_max_iterations_,
        trackerParams_.ransac_threshold_mono_,
        trackerParams_.ransac_probability_, translation, actual_iterations);
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

    // Remove outliers.
    removeOutliersMono(ref_frame, cur_frame, matches_ref_cur, ransac.inliers_,
                       ransac.iterations_);

    // CHECK QUALITY OF TRACKING
    TrackingStatus status = TrackingStatus::VALID;
    if (ransac.inliers_.size() < trackerParams_.minNrMonoInliers_) {
      VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
      status = TrackingStatus::FEW_MATCHES;
    }
    double disparity = computeMedianDisparity(*ref_frame, *cur_frame);

    VLOG(10) << "median disparity " << disparity;
    if (disparity < trackerParams_.disparityThreshold_) {
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

    if (verbosity_ >= 5) {
      // TODO(Toni) remove verbosity parameters, use gflags instead.
      debugInfo_.monoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - start_time;
    }
    debugInfo_.nrMonoInliers_ = ransac.inliers_.size();
    debugInfo_.nrMonoPutatives_ = matches_ref_cur.size();
    debugInfo_.monoRansacIters_ = ransac.iterations_;

    return std::make_pair(status, camLrectlkf_P_camLrectkf);
  }

  /* --------------------------------------------------------------------------
   */
  std::pair<Vector3, Matrix3> Tracker::getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const Matrix3& stereoPtCov,
      boost::optional<gtsam::Matrix3> Rmat) {
    gtsam::StereoPoint2 stereoPoint = gtsam::StereoPoint2(
        static_cast<double>(stereoFrame.left_keypoints_rectified_[pointId].x),
        static_cast<double>(stereoFrame.right_keypoints_rectified_[pointId].x),
        static_cast<double>(stereoFrame.left_keypoints_rectified_[pointId]
                   .y));  // uL_, uR_, v_;

    Matrix3 Jac_point3_sp2;  // jacobian of the back projection
    Vector3 point3_i_gtsam =
        stereoCam.backproject2(stereoPoint, boost::none, Jac_point3_sp2)
            .vector();
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

  /* --------------------------------------------------------------------------
   */
  // TODO(Toni) break down this gargantuan function...
  std::pair<std::pair<TrackingStatus, gtsam::Pose3>, gtsam::Matrix3>
  Tracker::geometricOutlierRejectionStereoGivenRotation(
      StereoFrame & ref_stereoFrame, StereoFrame & cur_stereoFrame,
      const gtsam::Rot3& R) {
    double start_time = UtilsOpenCV::GetTimeInSeconds();

    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    findMatchingStereoKeypoints(ref_stereoFrame, cur_stereoFrame,
                                &matches_ref_cur);

    VLOG(10) << "geometricOutlierRejectionStereoGivenRot:"
                " starting 1-point RANSAC (voting)";

    // Stereo point covariance: for covariance propagation.
    Matrix3 stereoPtCov = Matrix3::Identity();  // 3 px std in each direction

    // Create stereo camera.
    const gtsam::Cal3_S2& left_undist_rect_cam_mat =
        ref_stereoFrame.getLeftUndistRectCamMat();
    gtsam::Cal3_S2Stereo::shared_ptr K =
        boost::make_shared<gtsam::Cal3_S2Stereo>(
            left_undist_rect_cam_mat.fx(), left_undist_rect_cam_mat.fy(),
            left_undist_rect_cam_mat.skew(), left_undist_rect_cam_mat.px(),
            left_undist_rect_cam_mat.py(), ref_stereoFrame.getBaseline());
    // In the ref frame of the left camera.
    gtsam::StereoCamera stereoCam = gtsam::StereoCamera(gtsam::Pose3(), K);

    double timeMatchingAndAllocation_p = 0;
    if (verbosity_ >= 5) {
      timeMatchingAndAllocation_p = UtilsOpenCV::GetTimeInSeconds();
    }

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

    for (const std::pair<size_t, size_t>& it : matches_ref_cur) {
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
    if (verbosity_ >= 5) {
      timeCreatePointsAndCov_p = UtilsOpenCV::GetTimeInSeconds();
    }

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
    float threshold =
      static_cast<float>(trackerParams_
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
      coherentSet.at(i).push_back(
          i);  // vector is coherent with itself for sure
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
    if (verbosity_ >= 5) {
      timeVoting_p = UtilsOpenCV::GetTimeInSeconds();
    }

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

    // TODO(Toni) remove hardcoded value...
    int iterations = 1;  // to preserve RANSAC api

    // Remove outliers.
    removeOutliersStereo(ref_stereoFrame, cur_stereoFrame, matches_ref_cur,
                         inliers, iterations);

    // Check quality of tracking.
    TrackingStatus status = TrackingStatus::VALID;
    if (inliers.size() < trackerParams_.minNrStereoInliers_) {
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

    if (verbosity_ >= 5) {
      double timeTranslationComputation_p = UtilsOpenCV::GetTimeInSeconds();
      VLOG(10) << " timeMatchingAndAllocation: "
               << timeMatchingAndAllocation_p - start_time
               << " timeCreatePointsAndCov: "
               << timeCreatePointsAndCov_p - timeMatchingAndAllocation_p
               << " timeVoting: " << timeVoting_p - timeCreatePointsAndCov_p
               << " timeTranslationComputation: "
               << timeTranslationComputation_p - timeVoting_p;
      debugInfo_.stereoRansacTime_ =
          UtilsOpenCV::GetTimeInSeconds() - start_time;
    }
    debugInfo_.nrStereoInliers_ = inliers.size();
    debugInfo_.nrStereoPutatives_ = matches_ref_cur.size();
    debugInfo_.stereoRansacIters_ = iterations;

    return std::make_pair(
        std::make_pair(status, gtsam::Pose3(R, gtsam::Point3(t))),
        totalInfo.cast<double>());
  }

  /* --------------------------------------------------------------------------
   */
  std::pair<TrackingStatus, gtsam::Pose3>
  Tracker::geometricOutlierRejectionStereo(StereoFrame & ref_stereoFrame,
                                           StereoFrame & cur_stereoFrame) {
    double start_time = UtilsOpenCV::GetTimeInSeconds();

    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    findMatchingStereoKeypoints(ref_stereoFrame, cur_stereoFrame,
                                &matches_ref_cur);

    VLOG(10) << "geometricOutlierRejectionStereo:"
                " starting 3-point RANSAC (voting)";

    // Vector of 3D vectors
    Points3d f_cur;
    f_cur.reserve(matches_ref_cur.size());
    Points3d f_ref;
    f_ref.reserve(matches_ref_cur.size());
    for (const std::pair<size_t, size_t>& it : matches_ref_cur) {
      f_ref.push_back(ref_stereoFrame.keypoints_3d_.at(it.first));
      f_cur.push_back(cur_stereoFrame.keypoints_3d_.at(it.second));
    }

    // Setup problem (3D-3D adapter) -
    // http://laurentkneip.github.io/opengv/page_how_to_use.html
    AdapterStereo adapter(f_ref, f_cur);
    std::shared_ptr<ProblemStereo> problem = std::make_shared<ProblemStereo>(
        adapter, trackerParams_.ransac_randomize_);
    opengv::sac::Ransac<ProblemStereo> ransac;
    ransac.sac_model_ = problem;
    ransac.threshold_ = trackerParams_.ransac_threshold_stereo_;
    ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
    ransac.probability_ = trackerParams_.ransac_probability_;

    // Solve.
    if (!ransac.computeModel(0)) {
      VLOG(10) << "failure: (Arun) RANSAC could not find a solution.";
      return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
    }

    VLOG(10) << "geometricOutlierRejectionStereo: voting complete.";

    // Remove outliers.
    removeOutliersStereo(ref_stereoFrame, cur_stereoFrame, matches_ref_cur,
                         ransac.inliers_, ransac.iterations_);

    // Check quality of tracking.
    TrackingStatus status = TrackingStatus::VALID;
    if (ransac.inliers_.size() < trackerParams_.minNrStereoInliers_) {
      VLOG(10) << "FEW_MATCHES: " << ransac.inliers_.size();
      status = TrackingStatus::FEW_MATCHES;
    }

    // Get the resulting transformation: a 3x4 matrix [R t].
    opengv::transformation_t best_transformation = ransac.model_coefficients_;

    if (verbosity_ >= 5) {
      debugInfo_.stereoRansacTime_ =
          UtilsOpenCV::GetTimeInSeconds() - start_time;
    }
    debugInfo_.nrStereoInliers_ = ransac.inliers_.size();
    debugInfo_.nrStereoPutatives_ = matches_ref_cur.size();
    debugInfo_.stereoRansacIters_ = ransac.iterations_;

    return std::make_pair(
        status, UtilsOpenCV::openGvTfToGtsamPose3(best_transformation));
  }

  /* ------------------------------------------------------------------------ */
  void Tracker::findOutliers(
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      std::vector<int> inliers, std::vector<int>* outliers) {
    CHECK_NOTNULL(outliers)->clear();
    // Get outlier indices from inlier indices.
    std::sort(inliers.begin(), inliers.end(), std::less<size_t>());
    outliers->reserve(matches_ref_cur.size() - inliers.size());
    // The following is a complicated way of computing a set difference
    size_t k = 0;
    for (size_t i = 0u; i < matches_ref_cur.size(); ++i) {
      if (k < inliers.size()  // If we haven't exhaused inliers
          && static_cast<int>(i) > inliers[k])  // If we are after the inlier[k]
        ++k;                                    // Check the next inlier
      if (k >= inliers.size() ||
          static_cast<int>(i) != inliers[k])  // If i is not an inlier
        outliers->push_back(i);
    }
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::checkStatusRightKeypoints(
      const std::vector<KeypointStatus>& right_keypoints_status) {
    debugInfo_.nrValidRKP_ = 0;
    debugInfo_.nrNoLeftRectRKP_ = 0;
    debugInfo_.nrNoRightRectRKP_ = 0;
    debugInfo_.nrNoDepthRKP_ = 0;
    debugInfo_.nrFailedArunRKP_ = 0;
    for (const KeypointStatus& right_keypoint_status : right_keypoints_status) {
      switch (right_keypoint_status) {
        case KeypointStatus::VALID: {
          debugInfo_.nrValidRKP_++;
          break;
        }
        case KeypointStatus::NO_LEFT_RECT: {
          debugInfo_.nrNoLeftRectRKP_++;
          break;
        }
        case KeypointStatus::NO_RIGHT_RECT: {
          debugInfo_.nrNoRightRectRKP_++;
          break;
        }
        case KeypointStatus::NO_DEPTH: {
          debugInfo_.nrNoDepthRKP_++;
          break;
        }
        case KeypointStatus::FAILED_ARUN: {
          debugInfo_.nrFailedArunRKP_++;
          break;
        }
      }
    }
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::removeOutliersMono(
      Frame * ref_frame, Frame * cur_frame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const std::vector<int>& inliers, const int iterations) {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    // Find indices of outliers in current frame.
    std::vector<int> outliers;
    findOutliers(matches_ref_cur, inliers, &outliers);
    // Remove outliers.
    // outliers cannot be a vector of size_t because opengv uses a vector of
    // int.
    for (const size_t& i : outliers) {
      ref_frame->landmarks_.at(matches_ref_cur[i].first) = -1;
      cur_frame->landmarks_.at(matches_ref_cur[i].second) = -1;
    }
    VLOG(10) << "RANSAC (MONO): #iter = " << iterations
             << ", #inliers = " << inliers.size()
             << " #outliers = " << outliers.size();
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::removeOutliersStereo(
      StereoFrame & ref_stereoFrame, StereoFrame & cur_stereoFrame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const std::vector<int>& inliers, const int iterations) {
    // Find indices of outliers in current stereo frame.
    std::vector<int> outliers;
    findOutliers(matches_ref_cur, inliers, &outliers);

    // Remove outliers
    // outliers cannot be a vector of size_t because opengv uses a vector of
    // int.
    for (const size_t& i : outliers) {
      ref_stereoFrame.right_keypoints_status_.at(matches_ref_cur[i].first) =
          KeypointStatus::FAILED_ARUN;
      ref_stereoFrame.keypoints_depth_.at(matches_ref_cur[i].first) = 0.0;
      ref_stereoFrame.keypoints_3d_.at(matches_ref_cur[i].first) =
          Vector3::Zero();

      cur_stereoFrame.right_keypoints_status_.at(matches_ref_cur[i].second) =
          KeypointStatus::FAILED_ARUN;
      cur_stereoFrame.keypoints_depth_.at(matches_ref_cur[i].second) = 0.0;
      cur_stereoFrame.keypoints_3d_.at(matches_ref_cur[i].second) =
          Vector3::Zero();
    }
    VLOG(10) << "RANSAC (STEREO): #iter = " << iterations
             << ", #inliers = " << inliers.size()
             << " #outliers = " << outliers.size();
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::findMatchingKeypoints(
      const Frame& ref_frame, const Frame& cur_frame,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur) {
    CHECK_NOTNULL(matches_ref_cur)->clear();

    // Find keypoints that observe the same landmarks in both frames:
    std::map<LandmarkId, size_t> ref_lm_index_map;
    for (size_t i = 0; i < ref_frame.landmarks_.size(); ++i) {
      LandmarkId ref_id = ref_frame.landmarks_.at(i);
      if (ref_id != -1) {
        // Map landmark id -> position in ref_frame.landmarks_
        ref_lm_index_map[ref_id] = i;
      }
    }

    // Map of position of landmark j in ref frame to position of landmark j in
    // cur_frame
    matches_ref_cur->reserve(ref_lm_index_map.size());
    for (size_t i = 0; i < cur_frame.landmarks_.size(); ++i) {
      LandmarkId cur_id = cur_frame.landmarks_.at(i);
      if (cur_id != -1) {
        auto it = ref_lm_index_map.find(cur_id);
        if (it != ref_lm_index_map.end()) {
          matches_ref_cur->push_back(std::make_pair(it->second, i));
        }
      }
    }
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame, const StereoFrame& cur_stereoFrame,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur_stereo) {
    CHECK_NOTNULL(matches_ref_cur_stereo)->clear();
    std::vector<std::pair<size_t, size_t>> matches_ref_cur_mono;
    findMatchingKeypoints(ref_stereoFrame.getLeftFrame(),
                          cur_stereoFrame.getLeftFrame(),
                          &matches_ref_cur_mono);
    findMatchingStereoKeypoints(ref_stereoFrame, cur_stereoFrame,
                                matches_ref_cur_mono, matches_ref_cur_stereo);
  }

  /* --------------------------------------------------------------------------
   */
  void Tracker::findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame, const StereoFrame& cur_stereoFrame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur_mono,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur_stereo) {
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

  /* -------------------------------------------------------------------------- */
  double Tracker::computeMedianDisparity(const Frame& ref_frame,
                                        const Frame& cur_frame) {
    // Find keypoints that observe the same landmarks in both frames:
    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    findMatchingKeypoints(ref_frame, cur_frame, &matches_ref_cur);

    // Compute disparity:
    std::vector<double> disparity;
    disparity.reserve(matches_ref_cur.size());
    for (const std::pair<size_t, size_t>& rc : matches_ref_cur) {
      KeypointCV pxDiff =
          cur_frame.keypoints_[rc.second] - ref_frame.keypoints_[rc.first];
      double pxDistance = sqrt(pxDiff.x * pxDiff.x + pxDiff.y * pxDiff.y);
      disparity.push_back(pxDistance);
    }

    if (disparity.empty()) {
      VLOG(10) << "WARNING: Have no matches for disparity computation.";
      return 0.0;
    }

    // Compute median:
    const size_t center = disparity.size() / 2;
    std::nth_element(disparity.begin(),
                    disparity.begin() + center,
                    disparity.end());
    return disparity[center];
  }

  /* -------------------------------------------------------------------------- */
  // TODO this won't work in parallel mode, as visualization must be done in
  // main thread.
  cv::Mat Tracker::displayFrame(const Frame& ref_frame,
                                const Frame& cur_frame,
                                bool write_frame,
                                const std::string& img_title,
                                const KeypointsCV& extra_corners_gray,
                                const KeypointsCV& extra_corners_blue) const {
    cv::Mat img_rgb = cv::Mat(cur_frame.img_.size(), CV_8U);
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

    cv::imshow("Tracker img " + img_title, img_rgb);
    cv::waitKey(1);

    if (write_frame) {
      std::string folderName = outputImagesPath_ + img_title + "-" + "/";
      boost::filesystem::path trackerDir(folderName.c_str());
      boost::filesystem::create_directory(trackerDir);
      std::string img_name = folderName + "/trackerDisplay" + img_title + "_" +
                             std::to_string(cur_frame.id_) + ".png";
      cv::imwrite(img_name, img_rgb);
    }

    return img_rgb;
  }

} // End of VIO namespace.
