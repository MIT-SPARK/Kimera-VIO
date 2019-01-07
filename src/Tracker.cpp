/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Tracker.cpp
 * @brief  Class describing temporal tracking
 * @author Luca Carlone
 */

#include "Tracker.h"

using namespace VIO;

/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void Tracker::featureDetection(Frame& cur_frame)
{
  // Check how many new features we need: maxFeaturesPerFrame_ - n_existing features
  // If ref_frame has zero features this simply detects maxFeaturesPerFrame_ new features for cur_frame
  int n_existing = 0; // count existing (tracked) features
  for(size_t i = 0; i < cur_frame.landmarks_.size(); ++i)
  {   // count nr of valid keypoints
    if(cur_frame.landmarks_.at(i) != -1)
      ++n_existing;
    // features that have been tracked so far have Age+1
    cur_frame.landmarksAge_.at(i)++;
  }
  // Detect new features in image
  int nr_corners_needed = std::max(trackerParams_.maxFeaturesPerFrame_ - n_existing, 0); // detect this much new corners if possible
  debugInfo_.need_n_corners_ = nr_corners_needed;
  ///////////////// FEATURE DETECTION //////////////////////
  // If feature FeatureSelectionCriterion is quality, just extract what you need:
  double startTime;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();
  KeypointsCV corners; std::vector<double> cornerScores;
  std::tie(corners,cornerScores) = Tracker::FeatureDetection(cur_frame, trackerParams_, camMask_, nr_corners_needed);
  if (verbosity_ >= 5) debugInfo_.featureDetectionTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  debugInfo_.extracted_corners_ = corners.size();

  ///////////////// STORE NEW KEYPOINTS  //////////////////////
  // Store features in our Frame
  size_t nrExistingKeypoints = cur_frame.keypoints_.size(); // for debug, these are the ones tracked from the previous frame
  cur_frame.landmarks_.reserve(cur_frame.keypoints_.size() + corners.size());
  cur_frame.landmarksAge_.reserve(cur_frame.keypoints_.size() + corners.size());
  cur_frame.keypoints_.reserve(cur_frame.keypoints_.size() + corners.size());
  cur_frame.scores_.reserve(cur_frame.scores_.size() + cornerScores.size());
  cur_frame.versors_.reserve(cur_frame.keypoints_.size() + corners.size());

  for (size_t i=0; i < corners.size(); i++)
  {
    cur_frame.landmarks_.push_back(landmark_count_);
    cur_frame.landmarksAge_.push_back(1); // seen in a single (key)frame
    cur_frame.keypoints_.push_back(corners.at(i));
    cur_frame.scores_.push_back(cornerScores.at(i));
    cur_frame.versors_.push_back(Frame::CalibratePixel(corners.at(i), cur_frame.cam_param_));
    ++landmark_count_;
  }
#ifdef TRACKER_DEBUG_COUT
  std::cout << "featureExtraction: frame " << cur_frame.id_ <<
      ",  Nr tracked keypoints: " << nrExistingKeypoints <<
      ",  Nr extracted keypoints: " << corners.size() <<
      ",  total: " << cur_frame.keypoints_.size() <<
      "  (max: " << trackerParams_.maxFeaturesPerFrame_ << ")" << std::endl;
#endif
}

/* -------------------------------------------------------------------------- */
std::pair<KeypointsCV, std::vector<double> > Tracker::FeatureDetection(
    Frame& cur_frame,
    const VioFrontEndParams& trackerParams, const cv::Mat camMask,
    const int need_n_corners) {

  // Create mask such that new keypoints are not close to old ones.
  cv::Mat mask;
  camMask.copyTo(mask);
  for(size_t i = 0; i < cur_frame.keypoints_.size(); ++i) {
    if(cur_frame.landmarks_.at(i) != -1) {
      cv::circle(mask, cur_frame.keypoints_.at(i), trackerParams.min_distance_, cv::Scalar(0), CV_FILLED);
    }
  }
  // Find new features and corresponding scores
  KeypointsCV corners; std::vector<double> cornerScores;
  if(need_n_corners > 0){
    std::tie(corners,cornerScores) = UtilsOpenCV::MyGoodFeaturesToTrackSubPix(cur_frame.img_, need_n_corners,
      trackerParams.quality_level_, trackerParams.min_distance_, mask, trackerParams.block_size_,
      trackerParams.use_harris_detector_, trackerParams.k_);
  }

  // if we found new corners, try again on equalized image
  // if(corners.size() < need_n_corners){
  //   std::cout << "EQUALIZING IMAGE, SINCE WE GOT ONLY " << corners.size() <<" OUT OF " <<
  //       need_n_corners << " CORNERS" << std::endl;
  //   cv::Mat eqImg; cv::equalizeHist(ref_frame.img_, eqImg);
  //   std::tie(corners,cornerScores) = UtilsOpenCV::MyGoodFeaturesToTrackSubPix(eqImg, need_n_corners,
  //       trackerParams.quality_level_, trackerParams.min_distance_, mask, trackerParams.block_size_,
  //       trackerParams.use_harris_detector_, trackerParams.k_);
  //   std::cout << "GOT " << corners.size() << " AFTER EQUALIZATION" << std::endl;
  // }
  return std::make_pair(corners,cornerScores);
}

/* -------------------------------------------------------------------------- */
void Tracker::featureTracking(Frame& ref_frame, Frame& cur_frame)
{
  double startTime;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  // Setup termination criteria for optical flow
  std::vector<uchar> status;
  std::vector<float> error;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
      trackerParams_.klt_max_iter_, trackerParams_.klt_eps_);
  KeypointsCV px_ref;
  std::vector<size_t> indices;

  // Fill up structure for reference pixels and their labels
  indices.reserve(ref_frame.keypoints_.size());
  px_ref.reserve(ref_frame.keypoints_.size());
  for (size_t i = 0; i < ref_frame.keypoints_.size(); ++i)
  {
    if (ref_frame.landmarks_[i] != -1)
    {
      px_ref.push_back(ref_frame.keypoints_[i]);
      indices.push_back(i);
    }
  }
  // Initialize to old locations
  KeypointsCV px_cur = px_ref;
  if(px_cur.size() > 0)
  {
    // Do the actual tracking, so px_cur becomes the new pixel locations
    cv::calcOpticalFlowPyrLK(ref_frame.img_, cur_frame.img_,
        px_ref, px_cur,
        status, error,
        cv::Size2i(trackerParams_.klt_win_size_, trackerParams_.klt_win_size_),
        trackerParams_.klt_max_level_, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

    if(cur_frame.keypoints_.empty()) // Do we really need this check?
    {
      cur_frame.landmarks_.reserve(px_ref.size());
      cur_frame.landmarksAge_.reserve(px_ref.size());
      cur_frame.keypoints_.reserve(px_ref.size());
      cur_frame.scores_.reserve(px_ref.size());
      cur_frame.versors_.reserve(px_ref.size());
      for(size_t i = 0, n = 0; i < indices.size(); ++i)
      {
        size_t i_ref = indices[i];
        // If we failed to track mark off that landmark
        if(!status[i] || ref_frame.landmarksAge_[i_ref] > trackerParams_.maxFeatureAge_) // if we tracked keypoint and feature track is not too long
        {
          ref_frame.landmarks_[i_ref] = -1; // we are marking this bad in the ref_frame since features in the ref frame guide feature detection later on
          continue;
        }
        cur_frame.landmarks_.push_back(ref_frame.landmarks_[i_ref]);
        cur_frame.landmarksAge_.push_back(ref_frame.landmarksAge_[i_ref]);
        cur_frame.scores_.push_back(ref_frame.scores_[i_ref]);
        cur_frame.keypoints_.push_back(px_cur[i]);
        cur_frame.versors_.push_back(Frame::CalibratePixel(px_cur[i], ref_frame.cam_param_));
        ++n;
      }
      int maxAge = *std::max_element(cur_frame.landmarksAge_.begin(), cur_frame.landmarksAge_.end()); // max number of frames in which a feature is seen
#ifdef TRACKER_DEBUG_COUT
      std::cout << "featureTracking: frame " << cur_frame.id_ <<
          ",  Nr tracked keypoints: " << cur_frame.keypoints_.size() <<
          " (max: " << trackerParams_.maxFeaturesPerFrame_ << ")" <<
          " (max observed age of tracked features: " << maxAge <<
          " vs. maxFeatureAge_: " << trackerParams_.maxFeatureAge_ << ")" << std::endl;
#endif
    }
  }
  debugInfo_.nrTrackerFeatures_ = cur_frame.keypoints_.size();
  if (verbosity_ >= 5) debugInfo_.featureTrackingTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
}

/* -------------------------------------------------------------------------- */
std::pair<Tracker::TrackingStatus,gtsam::Pose3>
Tracker::geometricOutlierRejectionMono(Frame& ref_frame, Frame& cur_frame) {
  double startTime;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  std::vector<std::pair<size_t, size_t>> matches_ref_cur =
      FindMatchingKeypoints(ref_frame, cur_frame);

  // Vector of bearing vectors
  BearingVectors f_cur; f_cur.reserve(matches_ref_cur.size());
  BearingVectors f_ref; f_ref.reserve(matches_ref_cur.size());
  for (const std::pair<size_t, size_t>& it : matches_ref_cur)
  {
    f_ref.push_back(ref_frame.versors_.at(it.first)); // TODO (luca): if versors are only needed at keyframe, do not compute every frame
    f_cur.push_back(cur_frame.versors_.at(it.second));
  }
  // Setup problem
  AdapterMono adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMono> problem(new ProblemMono(adapter, ProblemMono::NISTER, trackerParams_.ransac_randomize_)); // last argument kills randomization
  opengv::sac::Ransac<ProblemMono> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = trackerParams_.ransac_threshold_mono_;
  ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
  ransac.probability_ = trackerParams_.ransac_probability_;

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMono: starting RANSAC" << std::endl;
#endif

  // Solve
  if (!ransac.computeModel(0))
  {

#ifdef TRACKER_DEBUG_COUT
    std::cout << "failure: 5pt RANSAC could not find a solution" << std::endl;
#endif

    return std::make_pair(Tracker::TrackingStatus::INVALID,gtsam::Pose3());
  }

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMono: RANSAC complete" << std::endl;
#endif

  // REMOVE OUTLIERS
  removeOutliersMono(ref_frame, cur_frame, matches_ref_cur, ransac.inliers_, ransac.iterations_);

  // CHECK QUALITY OF TRACKING
  Tracker::TrackingStatus status = Tracker::TrackingStatus::VALID;
  if(ransac.inliers_.size() < trackerParams_.minNrMonoInliers_){

#ifdef TRACKER_DEBUG_COUT
    std::cout << "FEW_MATCHES: " << ransac.inliers_.size() << std::endl;
#endif

    status = Tracker::TrackingStatus::FEW_MATCHES;
  }

  double disparity = ComputeMedianDisparity(ref_frame,cur_frame);

#ifdef TRACKER_DEBUG_COUT
  std::cout << "median disparity " << disparity << std::endl;
#endif

  if(disparity < trackerParams_.disparityThreshold_){

#ifdef TRACKER_DEBUG_COUT
    std::cout << "LOW_DISPARITY: " << disparity << std::endl;
#endif

    status = Tracker::TrackingStatus::LOW_DISPARITY;
  }
  // GET THE RESULTING TRANSFORMATION: a 3x4 matrix [R t]
  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  gtsam::Pose3 camLrectlkf_P_camLrectkf = UtilsOpenCV::Gvtrans2pose(best_transformation);

  // check if we have to compensate for rectification (if we have a valid R_rectify_ )
  //if(ref_frame.cam_param_.R_rectify_.rows == 3 && cur_frame.cam_param_.R_rectify_.rows == 3){
  //  gtsam::Rot3 camLrect_R_camL_ref = UtilsOpenCV::Cvmat2rot(ref_frame.cam_param_.R_rectify_);
  //  gtsam::Rot3 camLrect_R_camL_cut = UtilsOpenCV::Cvmat2rot(cur_frame.cam_param_.R_rectify_);
  //  camLrectlkf_P_camLrectkf =
  //      gtsam::Pose3(camLrect_R_camL_ref,Point3()) * camLlkf_P_camLkf *
  //      gtsam::Pose3(camLrect_R_camL_cut.inverse(),Point3());
  //}

  if (verbosity_ >= 5) debugInfo_.monoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  debugInfo_.nrMonoInliers_ = ransac.inliers_.size();
  debugInfo_.nrMonoPutatives_ = matches_ref_cur.size();
  debugInfo_.monoRansacIters_ = ransac.iterations_;
  return std::make_pair(status , camLrectlkf_P_camLrectkf);
}

/* -------------------------------------------------------------------------- */
std::pair<Tracker::TrackingStatus,gtsam::Pose3>
Tracker::geometricOutlierRejectionMonoGivenRotation(
    Frame& ref_frame, Frame& cur_frame, const gtsam::Rot3& R)
{
  double startTime;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  std::vector<std::pair<size_t, size_t>> matches_ref_cur =
      FindMatchingKeypoints(ref_frame, cur_frame);

  // Vector of bearing vectors
  BearingVectors f_cur; f_cur.reserve(matches_ref_cur.size());
  BearingVectors f_ref; f_ref.reserve(matches_ref_cur.size());
  for (const std::pair<size_t, size_t>& it : matches_ref_cur)
  {
    f_ref.push_back(ref_frame.versors_.at(it.first));
    f_cur.push_back(R.rotate( cur_frame.versors_.at(it.second) ));
  }
  // Setup problem
  AdapterMonoGivenRot adapter(f_ref, f_cur);
  std::shared_ptr<ProblemMonoGivenRot> problem(new ProblemMonoGivenRot(adapter, trackerParams_.ransac_randomize_)); // last argument kills randomization
  opengv::sac::Ransac<ProblemMonoGivenRot> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = trackerParams_.ransac_threshold_mono_;
  ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
  ransac.probability_ = trackerParams_.ransac_probability_;

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMonoGivenRot: starting RANSAC" << std::endl;
#endif

  // Solve
#ifdef sw_frontend
  ////////////////////////////////////
  // AMR: 2-point RANSAC
  int actual_iterations;
  std::vector<double> translation;
  translation.resize(3);
  std::vector<int> inliers = cv::ransac_2_point(f_ref, f_cur, trackerParams_.ransac_max_iterations_, trackerParams_.ransac_threshold_mono_, trackerParams_.ransac_probability_,
                                                translation, actual_iterations);
  gtsam::Matrix3 rot_mat = R.matrix();
  Eigen::Matrix<double,3,4> myModel;
  for (int i=0 ; i<3 ; i++){
    for (int j=0 ; j<3 ; j++)
      myModel(i,j) = rot_mat(i,j);
      myModel(i,3) = translation[i];
  }
  ransac.model_coefficients_ = myModel;
  ransac.inliers_ = inliers;
  ransac.iterations_ = actual_iterations;

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMonoGivenRot: RANSAC complete sw version" << std::endl;
#endif

  ////////////////////////////////////
#else
  if (!ransac.computeModel(0))
  {

#ifdef TRACKER_DEBUG_COUT
    std::cout << "failure: 2pt RANSAC could not find a solution" << std::endl;
#endif

    return std::make_pair(Tracker::TrackingStatus::INVALID,gtsam::Pose3());
  }

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMonoGivenRot: RANSAC complete" << std::endl;
#endif

#endif
  // REMOVE OUTLIERS
  removeOutliersMono(ref_frame, cur_frame, matches_ref_cur, ransac.inliers_, ransac.iterations_);

  // CHECK QUALITY OF TRACKING
  Tracker::TrackingStatus status = Tracker::TrackingStatus::VALID;
  if(ransac.inliers_.size() < trackerParams_.minNrMonoInliers_){

#ifdef TRACKER_DEBUG_COUT
    std::cout << "FEW_MATCHES: " << ransac.inliers_.size() << std::endl;
#endif

    status = Tracker::TrackingStatus::FEW_MATCHES;
  }

  double disparity = ComputeMedianDisparity(ref_frame,cur_frame);

#ifdef TRACKER_DEBUG_COUT
  std::cout << "median disparity " << disparity << std::endl;
#endif

  if(disparity < trackerParams_.disparityThreshold_){

#ifdef TRACKER_DEBUG_COUT
    std::cout << "LOW_DISPARITY: " << disparity << std::endl;
#endif

    status = Tracker::TrackingStatus::LOW_DISPARITY;
  }
  // GET THE RESULTING TRANSFORMATION: a 3x4 matrix [R t]
  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  gtsam::Pose3 camLlkf_P_camLkf = UtilsOpenCV::Gvtrans2pose(best_transformation);
  // note: this always returns the identity rotation, hence we have to substitute it:
  camLlkf_P_camLkf = gtsam::Pose3(R, camLlkf_P_camLkf.translation());

  gtsam::Pose3 camLrectlkf_P_camLrectkf = camLlkf_P_camLkf;
  // check if we have to compensate for rectification (if we have a valid R_rectify_ )
  if(ref_frame.cam_param_.R_rectify_.rows == 3 && cur_frame.cam_param_.R_rectify_.rows == 3){
    gtsam::Rot3 camLrect_R_camL_ref = UtilsOpenCV::Cvmat2rot(ref_frame.cam_param_.R_rectify_);
    gtsam::Rot3 camLrect_R_camL_cut = UtilsOpenCV::Cvmat2rot(cur_frame.cam_param_.R_rectify_);
    camLrectlkf_P_camLrectkf =
        gtsam::Pose3(camLrect_R_camL_ref,Point3()) * camLlkf_P_camLkf *
        gtsam::Pose3(camLrect_R_camL_cut.inverse(),Point3());
  }

  if (verbosity_ >= 5) debugInfo_.monoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  debugInfo_.nrMonoInliers_ = ransac.inliers_.size();
  debugInfo_.nrMonoPutatives_ = matches_ref_cur.size();
  debugInfo_.monoRansacIters_ = ransac.iterations_;

  return std::make_pair(status , camLrectlkf_P_camLrectkf);
}

/* -------------------------------------------------------------------------- */
std::pair< Vector3, Matrix3 > Tracker::GetPoint3AndCovariance(
    const StereoFrame& stereoFrame,
    const gtsam::StereoCamera& stereoCam,
    const int pointId,
    const Matrix3& stereoPtCov, boost::optional<gtsam::Matrix3> Rmat) {

  gtsam::StereoPoint2 stereoPoint = gtsam::StereoPoint2(
      double(stereoFrame.left_keypoints_rectified_[pointId].x),
      double(stereoFrame.right_keypoints_rectified_[pointId].x),
      double(stereoFrame.left_keypoints_rectified_[pointId].y)  ); // uL_, uR_, v_;

  Matrix3 Jac_point3_sp2; // jacobian of the back projection
  Vector3 point3_i_gtsam = stereoCam.backproject2(stereoPoint,boost::none,Jac_point3_sp2).vector();
  Vector3 point3_i = stereoFrame.keypoints_3d_.at(pointId);
  if((point3_i_gtsam - point3_i).norm() > 1e-1){

#ifdef TRACKER_DEBUG_COUT
    std::cout << "\n point3_i_gtsam \n " << point3_i_gtsam << "\n point3_i \n" << point3_i << std::endl;
#endif

    throw std::runtime_error("GetPoint3AndCovariance: inconsistent backprojection results (ref)");
  }
  if(Rmat){
    point3_i = (*Rmat) * point3_i; // optionally rotated to another ref frame
    Jac_point3_sp2 = (*Rmat) * Jac_point3_sp2;
  }

  gtsam::Matrix3 cov_i = Jac_point3_sp2 * stereoPtCov * Jac_point3_sp2.transpose();

  return std::make_pair(point3_i,cov_i);
}

/* -------------------------------------------------------------------------- */
std::pair< std::pair<Tracker::TrackingStatus,gtsam::Pose3>, gtsam::Matrix3 >
Tracker::geometricOutlierRejectionStereoGivenRotation(
    StereoFrame& ref_stereoFrame, StereoFrame& cur_stereoFrame,
    const gtsam::Rot3& R) {
  double startTime, timeMatchingAndAllocation_p, timeCreatePointsAndCov_p, timeVoting_p, timeTranslationComputation_p;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  std::vector<std::pair<size_t, size_t>> matches_ref_cur =
      FindMatchingStereoKeypoints(ref_stereoFrame, cur_stereoFrame);

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionStereoGivenRot: starting 1-point RANSAC (voting)" << std::endl;
#endif

  // stereo point covariance: for covariance propagation
  Matrix3 stereoPtCov = Matrix3::Identity(); // 3 px std in each direction

  // create stereo camera
  gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(ref_stereoFrame.left_undistRectCameraMatrix_.fx(),
      ref_stereoFrame.left_undistRectCameraMatrix_.fy(), ref_stereoFrame.left_undistRectCameraMatrix_.skew(),
      ref_stereoFrame.left_undistRectCameraMatrix_.px(), ref_stereoFrame.left_undistRectCameraMatrix_.py(),
      ref_stereoFrame.baseline_));
  gtsam::StereoCamera stereoCam = gtsam::StereoCamera(gtsam::Pose3(),K); // in the ref frame of the left camera
  if (verbosity_ >= 5) timeMatchingAndAllocation_p = UtilsOpenCV::GetTimeInSeconds();

  //========================================================================
  // CREATE DATA STRUCTURES
  //========================================================================
  size_t nrMatches = matches_ref_cur.size();
  Vector3 f_ref_i, R_f_cur_i;
  Matrix3 cov_ref_i, cov_R_cur_i;

  // relative translation suggested by each match and covariances (DOUBLE FOR PRECISE COV COMPUTATION)
  Vectors3 relTran; relTran.reserve(nrMatches); // translation such that Rot * f_cur = f_ref + relTran
  Matrices3 cov_relTran; cov_relTran.reserve(nrMatches);

  // relative translation suggested by each match and covariances (FLOAT FOR FAST VOTING)
  Vectors3f relTranf; relTranf.reserve(nrMatches); // translation such that Rot * f_cur = f_ref + relTran
  Matrices3f cov_relTranf; cov_relTranf.reserve(nrMatches);

  for (const std::pair<size_t, size_t>& it : matches_ref_cur)
  {
    // get reference vector and covariance:
    std::tie(f_ref_i,cov_ref_i) = Tracker::GetPoint3AndCovariance(ref_stereoFrame, stereoCam, it.first, stereoPtCov);
    // get current vectors and covariance:
    std::tie(R_f_cur_i, cov_R_cur_i) = Tracker::GetPoint3AndCovariance(cur_stereoFrame, stereoCam, it.second, stereoPtCov, R.matrix());
    // populate relative translation estimates and their covariances

    Vector3 v = f_ref_i - R_f_cur_i;
    Matrix3 M = cov_R_cur_i + cov_ref_i;

    relTran.push_back( v );
    cov_relTran.push_back( M );

    relTranf.push_back( v.cast <float> () );
    cov_relTranf.push_back( M.cast <float> () );
  }
  if (verbosity_ >= 5) timeCreatePointsAndCov_p = UtilsOpenCV::GetTimeInSeconds();
  //========================================================================
  // VOTING
  //========================================================================

  std::vector<std::vector<int> > coherentSet; coherentSet.resize(nrMatches); // (THIS MUST BE RESIZE - fixed size) number of other translations consistent with current one
  for (size_t i=0; i<nrMatches;i++)
    coherentSet.at(i).reserve(nrMatches); // preallocate

  size_t maxCoherentSetSize = 0;
  size_t maxCoherentSetId = 0;
  // double timeMahalanobis = 0, timeAllocate = 0, timePushBack = 0, timeMaxSet = 0;
  float threshold = float(trackerParams_.ransac_threshold_stereo_); // residual should be distributed according to chi-square distribution with 3 dofs,
  // considering a tail probability of 0.1, we get this value (x = chi2inv(0.9,3) = 6.2514

  Vector3f v; Matrix3f O; // allocate just once
  Vector3f relTran_i; Matrix3f cov_relTran_i;
  float dinv,innovationMahalanobisNorm; // define just once
  for (size_t i=0; i<nrMatches;i++)
  {
    relTran_i = relTranf.at(i);
    cov_relTran_i = cov_relTranf.at(i);
    coherentSet.at(i).push_back(i); // vector is coherent with itself for sure
    for (size_t j=i+1; j<nrMatches;j++) // look at the other vectors (quadratic complexity)
    {
      //timeBefore = UtilsOpenCV::GetTimeInSeconds();
      v = relTran_i - relTranf.at(j); // relTranMismatch_ij
      O = cov_relTran_i + cov_relTranf.at(j); // cov_relTran_j
      //timeAllocate += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

      // see testTracker for different implementations and timing for the mahalanobis distance
      //timeBefore = UtilsOpenCV::GetTimeInSeconds();
      dinv = 1/( O(0,0) * ( O(1,1)*O(2,2) - O(1,2)*O(2,1) ) - O(1,0) * ( O(0,1)*O(2,2) - O(0,2)*O(2,1) ) + O(2,0) * ( O(0,1)*O(1,2) - O(1,1)*O(0,2) ) );
      innovationMahalanobisNorm =
          dinv * v(0) * ( v(0) * ( O(1,1)*O(2,2) - O(1,2)*O(2,1) ) - v(1) * ( O(0,1)*O(2,2) - O(0,2)*O(2,1) ) + v(2) * ( O(0,1)*O(1,2) - O(1,1)*O(0,2) ) ) +
          dinv * v(1) * ( O(0,0) * ( v(1) *O(2,2) - O(1,2)*v(2) ) - O(1,0) * ( v(0)*O(2,2) - O(0,2)*v(2) ) + O(2,0) * ( v(0)*O(1,2) - v(1)*O(0,2) ) ) +
          dinv * v(2) * ( O(0,0) * ( O(1,1)*v(2) - v(1)*O(2,1) ) - O(1,0) * ( O(0,1)*v(2) - v(0)*O(2,1) ) + O(2,0) * ( O(0,1)*v(1) - O(1,1)*v(0) ) );
      //timeMahalanobis += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

      //timeBefore = UtilsOpenCV::GetTimeInSeconds();
      if(innovationMahalanobisNorm < threshold){
        coherentSet.at(i).push_back(j);
        coherentSet.at(j).push_back(i); // norm is symmetric
      }
      //timePushBack += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
    }
    // timeBefore = UtilsOpenCV::GetTimeInSeconds();
    if(coherentSet.at(i).size() > maxCoherentSetSize){
      maxCoherentSetSize = coherentSet.at(i).size();
      maxCoherentSetId = i;
    }
    // timeMaxSet += UtilsOpenCV::GetTimeInSeconds() - timeBefore;
  }
  //std::cout << "timeMahalanobis: " << timeMahalanobis << std::endl
  //<< "timeAllocate: " << timeAllocate << std::endl
  //<< "timePushBack: " << timePushBack << std::endl
  //<< "timeMaxSet: " << timeMaxSet << std::endl
  //<< " relTran.size(): " << relTran.size() << std::endl;
  if (verbosity_ >= 5) timeVoting_p = UtilsOpenCV::GetTimeInSeconds();

#ifdef TRACKER_DEBUG_COUT
  std::cout << "geometricOutlierRejectionMonoStereoRot: voting complete" << std::endl;
#endif

  //========================================================================
  // OUTLIER REJECTION AND TRANSLATION COMPUTATION
  //========================================================================
  if(maxCoherentSetSize < 2){
#ifdef TRACKER_DEBUG_COUT
    std::cout << "failure: 1point RANSAC (voting) could not find a solution" << std::endl;
#endif
    return std::make_pair(  std::make_pair(Tracker::TrackingStatus::INVALID,gtsam::Pose3()) , gtsam::Matrix3::Zero());
  }
  // inliers are max coherent set
  std::vector<int> inliers = coherentSet.at(maxCoherentSetId);
  // sort inliers
  std::sort (inliers.begin(), inliers.end());
  //UtilsOpenCV::PrintVector<int>(inliers,"inliers");
  //std::cout << "maxCoherentSetId: " << maxCoherentSetId << std::endl;
  int iterations = 1; // to preserve RANSAC api

  // REMOVE OUTLIERS
  removeOutliersStereo(ref_stereoFrame, cur_stereoFrame, matches_ref_cur, inliers, iterations);

  // CHECK QUALITY OF TRACKING
  Tracker::TrackingStatus status = Tracker::TrackingStatus::VALID;
  if(inliers.size() < trackerParams_.minNrStereoInliers_){
#ifdef TRACKER_DEBUG_COUT
    std::cout << "FEW_MATCHES: " << inliers.size() << std::endl;
#endif
    status = Tracker::TrackingStatus::FEW_MATCHES;
  }

  // GET THE RESULTING TRANSLATION
  Vector3 t = Vector3::Zero();
  Matrix3 totalInfo = Matrix3::Zero();
  for (size_t i=0; i<inliers.size();i++)
  {
    size_t matchId = inliers.at(i);
    Matrix3 infoMat = cov_relTran.at(matchId).inverse();
    t = t + infoMat * relTran.at(matchId);
    totalInfo = totalInfo + infoMat;
  }
  t = totalInfo.inverse() * t;

  if (verbosity_ >= 5){
    timeTranslationComputation_p = UtilsOpenCV::GetTimeInSeconds();
#ifdef TRACKER_DEBUG_COUT
    std::cout << "timeMatchingAndAllocation: " <<  timeMatchingAndAllocation_p - startTime <<
          " " << "timeCreatePointsAndCov: " << timeCreatePointsAndCov_p - timeMatchingAndAllocation_p <<
          " " << "timeVoting: " << timeVoting_p - timeCreatePointsAndCov_p <<
          " " << "timeTranslationComputation: " << timeTranslationComputation_p - timeVoting_p << std::endl;
#endif
    debugInfo_.stereoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  }
  debugInfo_.nrStereoInliers_ = inliers.size();
  debugInfo_.nrStereoPutatives_ = matches_ref_cur.size();
  debugInfo_.stereoRansacIters_ = iterations;

  return std::make_pair( std::make_pair(status , gtsam::Pose3(R,gtsam::Point3(t))) , totalInfo.cast <double> () );
}

/* -------------------------------------------------------------------------- */
std::pair<Tracker::TrackingStatus,gtsam::Pose3>
Tracker::geometricOutlierRejectionStereo(StereoFrame& ref_stereoFrame,
                                         StereoFrame& cur_stereoFrame) {
  double startTime;
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  std::vector<std::pair<size_t, size_t>> matches_ref_cur =
      FindMatchingStereoKeypoints(ref_stereoFrame, cur_stereoFrame);

  // Vector of 3D vectors
  Points3d f_cur; f_cur.reserve(matches_ref_cur.size());
  Points3d f_ref; f_ref.reserve(matches_ref_cur.size());
  for (const std::pair<size_t, size_t>& it : matches_ref_cur)
  {
    f_ref.push_back(ref_stereoFrame.keypoints_3d_.at(it.first));
    f_cur.push_back(cur_stereoFrame.keypoints_3d_.at(it.second));
  }

  // Setup problem (3D-3D adapter) - http://laurentkneip.github.io/opengv/page_how_to_use.html
  AdapterStereo adapter(f_ref, f_cur);
  std::shared_ptr<ProblemStereo> problem(new ProblemStereo(adapter, trackerParams_.ransac_randomize_)); // last argument kills randomization
  opengv::sac::Ransac<ProblemStereo> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = trackerParams_.ransac_threshold_stereo_;
  ransac.max_iterations_ = trackerParams_.ransac_max_iterations_;
  ransac.probability_ = trackerParams_.ransac_probability_;

  // Solve
  if (!ransac.computeModel(0))
  {
#ifdef TRACKER_DEBUG_COUT
    std::cout << "failure: (Arun) RANSAC could not find a solution" << std::endl;
#endif
    return std::make_pair(Tracker::TrackingStatus::INVALID,gtsam::Pose3());
  }

  // REMOVE OUTLIERS
  removeOutliersStereo(ref_stereoFrame, cur_stereoFrame, matches_ref_cur, ransac.inliers_, ransac.iterations_);

  // CHECK QUALITY OF TRACKING
  Tracker::TrackingStatus status = Tracker::TrackingStatus::VALID;
  if(ransac.inliers_.size() < trackerParams_.minNrStereoInliers_){
#ifdef TRACKER_DEBUG_COUT
    std::cout << "FEW_MATCHES: " << ransac.inliers_.size() << std::endl;
#endif
    status = Tracker::TrackingStatus::FEW_MATCHES;
  }

  // GET THE RESULTING TRANSFORMATION: a 3x4 matrix [R t]
  opengv::transformation_t best_transformation = ransac.model_coefficients_;

  if (verbosity_ >= 5) debugInfo_.stereoRansacTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  debugInfo_.nrStereoInliers_ = ransac.inliers_.size();
  debugInfo_.nrStereoPutatives_ = matches_ref_cur.size();
  debugInfo_.stereoRansacIters_ = ransac.iterations_;

  return std::make_pair(status , UtilsOpenCV::Gvtrans2pose(best_transformation));
}

/* -------------------------------------------------------------------------- */
// TODO do not use return for vector, pass by pointer.
std::vector<int> Tracker::FindOutliers(
    const std::vector<std::pair<size_t, size_t> >& matches_ref_cur,
    std::vector<int> inliers) {
  // Get outlier indices from inlier indices.
  std::sort(inliers.begin(), inliers.end(), std::less<int>());
  std::vector<int> outliers;
  outliers.reserve(matches_ref_cur.size() - inliers.size());
  // The following is a complicated way of computing a set difference
  size_t k = 0;
  for (size_t i = 0u; i < matches_ref_cur.size(); ++i) {
    if ( k < inliers.size() // If we haven't exhaused inliers
        && static_cast<int>(i) > inliers[k]) // If we are after the inlier[k]
      ++k;					    // Check the next inlier
    if ( k >= inliers.size()
        || static_cast<int>(i) != inliers[k]) // If i is not an inlier
      outliers.push_back(i);
  }
  return outliers;
}

/* -------------------------------------------------------------------------- */
void Tracker::removeOutliersMono(
    Frame& ref_frame, Frame& cur_frame,
    const std::vector<std::pair<size_t, size_t> >& matches_ref_cur,
    std::vector<int> inliers, const int iterations) {
  // find indices of outliers in current frame
  std::vector<int> outliers = FindOutliers(matches_ref_cur, inliers);

  // Remove outliers
  for (const int i : outliers)
  {
    ref_frame.landmarks_.at(matches_ref_cur[i].first) = -1;
    cur_frame.landmarks_.at(matches_ref_cur[i].second) = -1;
  }
#ifdef TRACKER_DEBUG_COUT
  std::cout << "RANSAC (MONO): #iter = " << iterations << ", #inliers = " << inliers.size() <<
      " #outliers = " << outliers.size() << std::endl;
#endif
}


/* -------------------------------------------------------------------------- */
void Tracker::checkStatusRightKeypoints(
    const std::vector<Kstatus>& right_keypoints_status) {
  debugInfo_.nrValidRKP_ = 0; debugInfo_.nrNoLeftRectRKP_ = 0; debugInfo_.nrNoRightRectRKP_ = 0;
  debugInfo_.nrNoDepthRKP_ = 0; debugInfo_.nrFailedArunRKP_ = 0;
  for(size_t i=0; i<right_keypoints_status.size(); i++){
    if(right_keypoints_status.at(i) == Kstatus::VALID)
      debugInfo_.nrValidRKP_++;
    if(right_keypoints_status.at(i) == Kstatus::NO_LEFT_RECT)
      debugInfo_.nrNoLeftRectRKP_++;
    if(right_keypoints_status.at(i) == Kstatus::NO_RIGHT_RECT)
      debugInfo_.nrNoRightRectRKP_++;
    if(right_keypoints_status.at(i) == Kstatus::NO_DEPTH)
      debugInfo_.nrNoDepthRKP_++;
  }
}

/* -------------------------------------------------------------------------- */
void Tracker::removeOutliersStereo(
    StereoFrame& ref_stereoFrame, StereoFrame& cur_stereoFrame,
    const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
    std::vector<int> inliers, const int iterations) {
  // find indices of outliers in current stereo frame
  std::vector<int> outliers = FindOutliers(matches_ref_cur, inliers);

  // Remove outliers
  for (const int i : outliers)
  {
    ref_stereoFrame.right_keypoints_status_.at(matches_ref_cur[i].first) = Kstatus::FAILED_ARUN;
    ref_stereoFrame.keypoints_depth_.at(matches_ref_cur[i].first) = 0.0;
    ref_stereoFrame.keypoints_3d_ .at(matches_ref_cur[i].first) = Vector3::Zero();

    cur_stereoFrame.right_keypoints_status_.at(matches_ref_cur[i].second) = Kstatus::FAILED_ARUN;
    cur_stereoFrame.keypoints_depth_.at(matches_ref_cur[i].second) = 0.0;
    cur_stereoFrame.keypoints_3d_ .at(matches_ref_cur[i].second) = Vector3::Zero();
  }
#ifdef TRACKER_DEBUG_COUT
  std::cout << "RANSAC (STEREO): #iter = " << iterations << ", #inliers = " << inliers.size() <<
      " #outliers = " << outliers.size() << std::endl;
#endif
}

/* -------------------------------------------------------------------------- */
std::vector<std::pair<size_t, size_t>> Tracker::FindMatchingStereoKeypoints(
    const StereoFrame& ref_stereoFrame, const StereoFrame& cur_stereoFrame)
{
  std::vector<std::pair<size_t, size_t>> matches_ref_cur_mono =
      FindMatchingKeypoints(ref_stereoFrame.left_frame_, cur_stereoFrame.left_frame_);

  return FindMatchingStereoKeypoints(ref_stereoFrame,cur_stereoFrame,matches_ref_cur_mono);
}

/* -------------------------------------------------------------------------- */
std::vector<std::pair<size_t, size_t>> Tracker::FindMatchingStereoKeypoints(
    const StereoFrame& ref_stereoFrame, const StereoFrame& cur_stereoFrame,
    const std::vector<std::pair<size_t, size_t>>& matches_ref_cur_mono)
{
  std::vector<std::pair<size_t, size_t>> matches_ref_cur_stereo;
  for(size_t i = 0; i < matches_ref_cur_mono.size(); ++i)
  {
    size_t ind_ref = matches_ref_cur_mono[i].first;
    size_t ind_cur = matches_ref_cur_mono[i].second;
    if(// at this point we already discarded keypoints with landmark = -1
        ref_stereoFrame.right_keypoints_status_[ind_ref] == Kstatus::VALID &&
        cur_stereoFrame.right_keypoints_status_[ind_cur] == Kstatus::VALID	){
      matches_ref_cur_stereo.push_back(matches_ref_cur_mono[i]); // pair of points that has 3D in both stereoFrames
    }
  }
  return matches_ref_cur_stereo;
}

/* -------------------------------------------------------------------------- */
std::vector<std::pair<size_t, size_t>> Tracker::FindMatchingKeypoints(
    const Frame& ref_frame, const Frame& cur_frame)
{
  // Find keypoints that observe the same landmarks in both frames:
  std::map<LandmarkId, size_t> ref_lm_index_map;
  for(size_t i = 0; i < ref_frame.landmarks_.size(); ++i)
  {
    LandmarkId ref_id = ref_frame.landmarks_.at(i);
    if(ref_id != -1)
    {   // Map landmark id -> position in ref_frame.landmarks_
      ref_lm_index_map[ref_id] = i;
    }
  }
  std::vector<std::pair<size_t, size_t>> matches_ref_cur; // Map of position of landmark j in ref frame to position of landmark j in cur_frame
  matches_ref_cur.reserve(ref_lm_index_map.size());
  for(size_t i = 0; i < cur_frame.landmarks_.size(); ++i)
  {
    LandmarkId cur_id = cur_frame.landmarks_.at(i);
    if(cur_id != -1)
    {
      auto it = ref_lm_index_map.find(cur_id);
      if(it != ref_lm_index_map.end())
      {
        matches_ref_cur.push_back(std::make_pair(it->second, i));
      }
    }
  }
  return matches_ref_cur;
}

/* -------------------------------------------------------------------------- */
double Tracker::ComputeMedianDisparity(const Frame& ref_frame,
                                       const Frame& cur_frame)
{
  // Find keypoints that observe the same landmarks in both frames:
  std::vector<std::pair<size_t, size_t>> matches_ref_cur =
      FindMatchingKeypoints(ref_frame, cur_frame);

  // Compute disparity:
  std::vector<double> disparity;
  disparity.reserve(matches_ref_cur.size());
  // std::cout << "pxDistance: " << std::endl;
  for(const std::pair<size_t, size_t>& rc : matches_ref_cur)
  {
    KeypointCV pxDiff = cur_frame.keypoints_[rc.second] - ref_frame.keypoints_[rc.first];
    double pxDistance = sqrt( pxDiff.x*pxDiff.x + pxDiff.y*pxDiff.y);
    // std::cout << " " << pxDistance;
    disparity.push_back( pxDistance );
  }
  // std::cout << std::endl;

  if(disparity.empty())
  {
#ifdef TRACKER_DEBUG_COUT
    std::cout << "WARNING: Have no matches for disparity computation\n" << std::endl;
#endif
    return 0.0;
  }

  // Compute median:
  const size_t center = disparity.size() / 2;
  std::nth_element(disparity.begin(), disparity.begin() + center, disparity.end());
  return disparity[center];
}

/* -------------------------------------------------------------------------- */
cv::Mat Tracker::displayFrame(
    const Frame& ref_frame, const Frame& cur_frame,
    const int verbosity,
    const KeypointsCV& extraCorners1,
    const KeypointsCV& extraCorners2,
    const std::string& extraString) const
{
  cv::Mat img_rgb = cv::Mat(cur_frame.img_.size(), CV_8U);
  cv::cvtColor(cur_frame.img_, img_rgb, cv::COLOR_GRAY2RGB);

  // add extra corners if desired
  for (auto px_cur : extraCorners1) // gray
    cv::circle(img_rgb, px_cur, 4, cv::Scalar(255,255,255), 2);
  for (auto px_cur : extraCorners2) // blue
    cv::circle(img_rgb, px_cur, 4, cv::Scalar(255,0,0), 2);

  // add all keypoints in cur_frame with the tracks
  for (size_t i = 0; i < cur_frame.keypoints_.size(); ++i)
  {
    cv::Point2f px_cur = cur_frame.keypoints_.at(i);
    if (cur_frame.landmarks_.at(i) == -1) // Untracked landmarks are red.
    {
      cv::circle(img_rgb, px_cur, 4, cv::Scalar(0,0,255), 2);
    }
    else
    {
      auto it = find(ref_frame.landmarks_.begin(), ref_frame.landmarks_.end(), cur_frame.landmarks_.at(i));
      if (it != ref_frame.landmarks_.end())
      {
        // If feature was in previous frame, display tracked feature with green circle/line
        cv::circle(img_rgb, px_cur, 6, cv::Scalar(0,255,0), 1);
        int nPos = distance(ref_frame.landmarks_.begin(), it);
        cv::Point2f px_ref = ref_frame.keypoints_.at(nPos);
        cv::line(img_rgb, px_cur, px_ref, cv::Scalar(0,255,0), 1);
      }
      else // New feature tracks are blue.
      {
        cv::circle(img_rgb, px_cur, 6, cv::Scalar(255,0,0), 1);
      }
    }
  }
  if(verbosity==1){ // otherwise just return the image
    cv::imshow("img"+ extraString, img_rgb);
    cv::waitKey(trackerParams_.display_time_);
  }
  if(verbosity==2){
    std::string folderName = outputImagesPath_ + extraString + "-" + VioFrontEndParams::FeatureSelectionCriterionStr(trackerParams_.featureSelectionCriterion_) + "/";
    boost::filesystem::path trackerDir(folderName.c_str());
    boost::filesystem::create_directory(trackerDir);
    std::string img_name = folderName + "/trackerDisplay" + extraString + "_" + std::to_string(cur_frame.id_) + ".png";
    cv::imwrite(img_name, img_rgb);
  }
  return img_rgb;
}
