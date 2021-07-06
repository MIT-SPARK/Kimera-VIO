/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Pipeline for detection and reporting of Loop Closures between frames.
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/loopclosure/LoopClosureDetector.h"

#include <KimeraRPGO/RobustSolver.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <string>
#include <vector>

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DEFINE_string(vocabulary_path,
              "../vocabulary/ORBvoc.yml",
              "Path to BoW vocabulary file for LoopClosureDetector module.");

/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/

namespace VIO {

/* ------------------------------------------------------------------------ */
LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const StereoCamera::ConstPtr& stereo_camera,
    const StereoMatchingParams& stereo_matching_params,
    bool log_output)
    : lcd_state_(LcdState::Bootstrap),
      lcd_params_(lcd_params),
      log_output_(log_output),
      orb_feature_detector_(),
      orb_feature_matcher_(),
      tracker_(
          lcd_params.tracker_params_,
          std::make_shared<VIO::Camera>(stereo_camera->getLeftCamParams())),
      db_BoW_(nullptr),
      db_frames_(),
      timestamp_map_(),
      lcd_tp_wrapper_(nullptr),
      latest_bowvec_(),
      B_Pose_camLrect_(),
      stereo_camera_(stereo_camera),
      stereo_matcher_(nullptr),
      pgo_(nullptr),
      W_Pose_Blkf_estimates_(),
      logger_(nullptr) {
  gtsam::Vector6 precisions;
  precisions.head<3>().setConstant(lcd_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(lcd_params_.betweenTranslationPrecision_);
  shared_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(precisions);
  B_Pose_camLrect_ = stereo_camera_->getBodyPoseLeftCamRect();

  // Sparse stereo reconstruction members
  stereo_matcher_ =
      VIO::make_unique<StereoMatcher>(stereo_camera_, stereo_matching_params);

  // Initialize the ORB feature detector object:
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
                                          lcd_params_.scale_factor_,
                                          lcd_params_.nlevels_,
                                          lcd_params_.edge_threshold_,
                                          lcd_params_.first_level_,
                                          lcd_params_.WTA_K_,
                                          lcd_params_.score_type_,
                                          lcd_params_.patch_sze_,
                                          lcd_params_.fast_threshold_);

  // Initialize our feature matching object:
  orb_feature_matcher_ =
      cv::DescriptorMatcher::create(lcd_params_.matcher_type_);

  // Load ORB vocabulary:
  std::ifstream f_vocab(FLAGS_vocabulary_path.c_str());
  CHECK(f_vocab.good()) << "LoopClosureDetector: Incorrect vocabulary path: "
                        << FLAGS_vocabulary_path;
  f_vocab.close();

  OrbVocabulary vocab;
  LOG(INFO) << "LoopClosureDetector:: Loading vocabulary from "
            << FLAGS_vocabulary_path;
  vocab.load(FLAGS_vocabulary_path);
  LOG(INFO) << "Loaded vocabulary with " << vocab.size() << " visual words.";

  // Initialize the thirdparty wrapper:
  lcd_tp_wrapper_ = VIO::make_unique<LcdThirdPartyWrapper>(lcd_params_);

  // Initialize db_BoW_:
  db_BoW_ = VIO::make_unique<OrbDatabase>(vocab);

  // Initialize pgo_:
  // TODO(marcus): parametrize the verbosity of PGO params
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(lcd_params_.pgo_trans_threshold_,
                                  lcd_params_.pgo_rot_threshold_,
                                  KimeraRPGO::Verbosity::QUIET);
  pgo_ = VIO::make_unique<KimeraRPGO::RobustSolver>(pgo_params);

  if (log_output) logger_ = VIO::make_unique<LoopClosureDetectorLogger>();
}

LoopClosureDetector::~LoopClosureDetector() {
  LOG(INFO) << "LoopClosureDetector desctuctor called.";
}

/* ------------------------------------------------------------------------ */
LcdOutput::UniquePtr LoopClosureDetector::spinOnce(const LcdInput& input) {
  CHECK_GE(input.cur_kf_id_, 0);

  // Update the PGO with the Backend VIO estimate.
  // TODO(marcus): only add factor if it's a set distance away from previous
  // TODO(marcus): OdometryPose vs OdometryFactor
  timestamp_map_[input.cur_kf_id_] = input.timestamp_;
  // TODO: W_Pose_Blkf_estimates_.push_back(odom_factor.W_Pose_Blkf_);
  OdometryFactor odom_factor(
      input.cur_kf_id_, input.W_Pose_Blkf_, shared_noise_model_);

  switch (lcd_state_) {
    case LcdState::Bootstrap: {
      CHECK_EQ(pgo_->calculateEstimate().size(), 0);
      initializePGO(odom_factor);
      break;
    }
    case LcdState::Nominal: {
      // TODO(marcus): need a better check than this:
      CHECK_GT(pgo_->calculateEstimate().size(), 0);
      addOdometryFactorAndOptimize(odom_factor);
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized LCD state.";
    }
  }

  // Process the StereoFrame and check for a loop closure with previous ones.
  LoopResult loop_result;
  FrameId lcd_frame_id;
  if (input.frontend_output_->frontend_type_ == FrontendType::kStereoImu) {
    StereoFrontendOutput::Ptr stereo_frontend_output =
        VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
            input.frontend_output_);
    lcd_frame_id =
        processAndAddStereoFrame(stereo_frontend_output->stereo_frame_lkf_);
  } else {
    MonoFrontendOutput::Ptr mono_frontend_output =
        VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(
            input.frontend_output_);
    lcd_frame_id = processAndAddMonoFrame(mono_frontend_output->frame_lkf_,
                                          input.W_points_with_ids_,
                                          input.W_Pose_Blkf_);
  }
  detectLoop(lcd_frame_id, &loop_result);

  // Build and add LC factor if result is a loop closure.
  if (loop_result.isLoop()) {
    LoopClosureFactor lc_factor(loop_result.match_id_,
                                loop_result.query_id_,
                                loop_result.relative_pose_,
                                shared_noise_model_);

    utils::StatsCollector stat_pgo_timing(
        "PGO Update/Optimization Timing [ms]");
    auto tic = utils::Timer::tic();

    addLoopClosureFactorAndOptimize(lc_factor);

    auto update_duration = utils::Timer::toc(tic).count();
    stat_pgo_timing.AddSample(update_duration);

    VLOG(1) << "LoopClosureDetector: LOOP CLOSURE detected from keyframe "
            << loop_result.match_id_ << " to keyframe "
            << loop_result.query_id_;
  } else {
    VLOG(2) << "LoopClosureDetector: No loop closure detected. Reason: "
            << LoopResult::asString(loop_result.status_);
  }

  // Timestamps for PGO and for LCD should match now.
  CHECK_EQ(db_frames_.back().timestamp_,
           timestamp_map_.at(db_frames_.back().id_));
  CHECK_EQ(timestamp_map_.size(), db_frames_.size());
  CHECK_EQ(timestamp_map_.size(), W_Pose_Blkf_estimates_.size());

  // Construct output payload.
  CHECK(pgo_);
  const gtsam::Pose3& w_Pose_map = getWPoseMap();
  const gtsam::Values& pgo_states = pgo_->calculateEstimate();
  const gtsam::NonlinearFactorGraph& pgo_nfg = pgo_->getFactorsUnsafe();

  LcdOutput::UniquePtr output_payload = nullptr;
  if (loop_result.isLoop()) {
    output_payload =
        VIO::make_unique<LcdOutput>(true,
                                    input.timestamp_,
                                    timestamp_map_.at(loop_result.query_id_),
                                    timestamp_map_.at(loop_result.match_id_),
                                    loop_result.match_id_,
                                    loop_result.query_id_,
                                    loop_result.relative_pose_,
                                    w_Pose_map,
                                    pgo_states,
                                    pgo_nfg);
  } else {
    output_payload = VIO::make_unique<LcdOutput>(input.timestamp_);
    output_payload->W_Pose_Map_ = w_Pose_map;
    output_payload->states_ = pgo_states;
    output_payload->nfg_ = pgo_nfg;
  }
  CHECK(output_payload) << "Missing LCD output payload.";

  if (logger_) {
    debug_info_.timestamp_ = output_payload->timestamp_;
    debug_info_.loop_result_ = loop_result;
    debug_info_.pgo_size_ = pgo_->size();
    debug_info_.pgo_lc_count_ = pgo_->getNumLC();
    debug_info_.pgo_lc_inliers_ = pgo_->getNumLCInliers();

    logger_->logTimestampMap(timestamp_map_);
    logger_->logDebugInfo(debug_info_);
    logger_->logLCDResult(*output_payload);
  }

  return output_payload;
}

FrameId LoopClosureDetector::processAndAddMonoFrame(
    const Frame& frame,
    const PointsWithIdMap& W_points_with_ids,
    const gtsam::Pose3& W_Pose_Blkf) {
  // We use existing features instead of new ORB ones like in the stereo case
  // because we have to use existing 3D points from the backend as in Mono mode
  // we cannot compute 3D points via stereo reconstruction.

  // TODO(marcus): check the backend param for generating the pointswithidmap
  // should be a boost::optional so that if it's none we can throw exception in
  // lcd ctor
  size_t nr_kpts = frame.keypoints_.size();
  CHECK_EQ(nr_kpts, frame.landmarks_.size());
  CHECK_EQ(nr_kpts, frame.versors_.size());
  CHECK_EQ(nr_kpts, frame.keypoints_undistorted_.size());

  KeypointsCV keypoints;
  BearingVectors undistorted_bearing_vectors;
  StatusKeypointsCV undistorted_keypoints_with_status;
  Landmarks keypoints_3d;
  for (size_t i = 0; i < frame.landmarks_.size(); i++) {
    const LandmarkId& id = frame.landmarks_[i];
    // Check that the id is in the points map
    if (W_points_with_ids.find(id) != W_points_with_ids.end()) {
      // Keep track of keypoints and other data associated with observed
      // landmarks that are in the backend's points map.
      keypoints.push_back(frame.keypoints_.at(i));
      undistorted_bearing_vectors.push_back(frame.versors_.at(i));
      undistorted_keypoints_with_status.push_back(
          frame.keypoints_undistorted_.at(i));

      // Convert point from world frame to local camera frame so that the
      // reference frame matches the convention used in the stereo case.
      // TODO(marcus): unit test to make sure ref frame is correct for these
      // points
      Landmark keypoint_3d =
          W_Pose_Blkf * B_Pose_camLrect_ * W_points_with_ids.at(id);
      keypoints_3d.push_back(keypoint_3d);
    } else {
      LOG(ERROR) << "ProcessAndAddMonoFrame: landmark id not in world points!";
    }
  }

  // Generate descriptors for remaining keypoints.
  // format change required for the descriptor computation
  std::vector<cv::KeyPoint> keypoints_for_descriptor_compute;
  cv::KeyPoint::convert(keypoints, keypoints_for_descriptor_compute);
  OrbDescriptor descriptors_mat;
  orb_feature_detector_->compute(
      frame.img_, keypoints_for_descriptor_compute, descriptors_mat);
  OrbDescriptorVec descriptors_vec;
  descriptorMatToVec(descriptors_mat, &descriptors_vec);

  // Build and store LCDFrame object.
  db_frames_.push_back(LCDFrame(frame.timestamp_,
                                FrameId(db_frames_.size()),
                                frame.id_,
                                keypoints,
                                keypoints_3d,
                                descriptors_vec,
                                descriptors_mat,
                                undistorted_bearing_vectors,
                                undistorted_keypoints_with_status,
                                StatusKeypointsCV()));

  CHECK(!db_frames_.empty());
  return db_frames_.back().id_;
}

/* ------------------------------------------------------------------------ */
FrameId LoopClosureDetector::processAndAddStereoFrame(
    const StereoFrame& stereo_frame) {
  KeypointsCV keypoints;
  OrbDescriptor descriptors_mat;
  getNewFeaturesAndDescriptors(
      stereo_frame.left_frame_.img_, &keypoints, &descriptors_mat);

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  StereoFrame cp_stereo_frame(stereo_frame);
  rewriteStereoFrameFeatures(keypoints, &cp_stereo_frame);

  // Cut descriptors down to relevant size
  // First have to recompute descriptors for surviving keypoints not removed
  // by the update function.
  descriptors_mat = OrbDescriptor();
  std::vector<cv::KeyPoint> keypoints_for_descriptor_matching;
  cv::KeyPoint::convert(cp_stereo_frame.left_frame_.keypoints_,
                        keypoints_for_descriptor_matching);
  orb_feature_detector_->compute(cp_stereo_frame.left_frame_.img_,
                                 keypoints_for_descriptor_matching,
                                 descriptors_mat);
  OrbDescriptorVec descriptors_vec;
  descriptorMatToVec(descriptors_mat, &descriptors_vec);

  // Build and store LCDFrame object.
  db_frames_.push_back(LCDFrame(cp_stereo_frame.timestamp_,
                                FrameId(db_frames_.size()),
                                cp_stereo_frame.id_,
                                cp_stereo_frame.left_frame_.keypoints_,
                                cp_stereo_frame.keypoints_3d_,
                                descriptors_vec,
                                descriptors_mat,
                                cp_stereo_frame.left_frame_.versors_,
                                cp_stereo_frame.left_keypoints_rectified_,
                                cp_stereo_frame.right_keypoints_rectified_));

  CHECK(!db_frames_.empty());
  return db_frames_.back().id_;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::getNewFeaturesAndDescriptors(
    const cv::Mat& img,
    KeypointsCV* keypoints,
    OrbDescriptor* descriptors_mat) {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors_mat);
  // TODO(marcus): switch on feature type (orb, etc) when more are supported
  // Extract ORB features and construct descriptors_vec.
  std::vector<cv::KeyPoint> keypoints_for_compute;
  orb_feature_detector_->detectAndCompute(
      img, cv::Mat(), keypoints_for_compute, *descriptors_mat);

  // Convert keypoints to common VIO type.
  cv::KeyPoint::convert(keypoints_for_compute, *keypoints);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::descriptorMatToVec(
    const OrbDescriptor& descriptors_mat,
    OrbDescriptorVec* descriptors_vec) {
  CHECK_NOTNULL(descriptors_vec);

  // TODO(marcus): tied to ORB, need to generalize!
  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec->resize(descriptors_mat.size().height);

  for (size_t i = 0; i < descriptors_vec->size(); i++) {
    (*descriptors_vec)[i] =
        cv::Mat(1, L, descriptors_mat.type());  // one row only
    descriptors_mat.row(i).copyTo((*descriptors_vec)[i].row(0));
  }
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::detectLoop(const FrameId& frame_id,
                                     LoopResult* result) {
  CHECK_NOTNULL(result);

  result->query_id_ = frame_id;

  // Create BOW representation of descriptors.
  DBoW2::BowVector bow_vec;
  DCHECK(db_BoW_);
  db_BoW_->getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_,
                                      bow_vec);

  int max_possible_match_id = frame_id - lcd_params_.recent_frames_window_;
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  // Query for BoW vector matches in database.
  DBoW2::QueryResults query_result;
  db_BoW_->query(bow_vec,
                 query_result,
                 lcd_params_.max_db_results_,
                 max_possible_match_id);

  // Add current BoW vector to database.
  db_BoW_->add(bow_vec);

  if (query_result.empty()) {
    result->status_ = LCDStatus::NO_MATCHES;
  } else {
    double nss_factor = 1.0;
    if (lcd_params_.use_nss_) {
      nss_factor = db_BoW_->getVocabulary()->score(bow_vec, latest_bowvec_);
    } else {
      LOG(ERROR) << "Setting use_nss as false is deprecated. ";
    }

    if (lcd_params_.use_nss_ && nss_factor < lcd_params_.min_nss_factor_) {
      result->status_ = LCDStatus::LOW_NSS_FACTOR;
    } else {
      // Remove low scores from the QueryResults based on nss.
      DBoW2::QueryResults::iterator query_it =
          lower_bound(query_result.begin(),
                      query_result.end(),
                      DBoW2::Result(0, lcd_params_.alpha_ * nss_factor),
                      DBoW2::Result::geq);
      if (query_it != query_result.end()) {
        query_result.resize(query_it - query_result.begin());
      }

      // Begin grouping and checking matches.
      if (query_result.empty()) {
        result->status_ = LCDStatus::LOW_SCORE;
      } else {
        // Set best candidate to highest scorer.
        result->match_id_ = query_result[0].Id;

        // Compute islands in the matches.
        // An island is a group of matches with close frame_ids.
        std::vector<MatchIsland> islands;
        lcd_tp_wrapper_->computeIslands(&query_result, &islands);

        if (islands.empty()) {
          result->status_ = LCDStatus::NO_GROUPS;
        } else {
          // Find the best island grouping using MatchIsland sorting.
          const MatchIsland& best_island =
              *std::max_element(islands.begin(), islands.end());

          // Run temporal constraint check on this best island.
          bool pass_temporal_constraint =
              lcd_tp_wrapper_->checkTemporalConstraint(frame_id, best_island);

          if (!pass_temporal_constraint) {
            result->status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
          } else {
            // Perform geometric verification check.
            gtsam::Pose3 camMatch_T_camQuery_2d;

            // Find correspondences between keypoints.
            KeypointMatches matches_match_query;
            computeDescriptorMatches(
                db_frames_[result->match_id_].descriptors_mat_,
                db_frames_[result->query_id_].descriptors_mat_,
                &matches_match_query,
                true);

            bool pass_geometric_verification =
                geometricVerificationCam2d2d(result->match_id_,
                                             result->query_id_,
                                             matches_match_query,
                                             &camMatch_T_camQuery_2d);

            if (!pass_geometric_verification) {
              result->status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
            } else {
              gtsam::Pose3 bodyMatch_T_bodyQuery_3d;
              bool pass_3d_pose_compute =
                  recoverPoseBody(result->match_id_,
                                  result->query_id_,
                                  camMatch_T_camQuery_2d,
                                  matches_match_query,
                                  &bodyMatch_T_bodyQuery_3d);

              if (!pass_3d_pose_compute) {
                result->status_ = LCDStatus::FAILED_POSE_RECOVERY;
              } else {
                result->relative_pose_ = bodyMatch_T_bodyQuery_3d;
                result->status_ = LCDStatus::LOOP_DETECTED;
              }
            }
          }
        }
      }
    }
  }

  // Update latest bowvec for normalized similarity scoring (NSS).
  if (static_cast<int>(frame_id + 1) > lcd_params_.recent_frames_window_) {
    latest_bowvec_ = bow_vec;
  } else {
    VLOG(3) << "LoopClosureDetector: Not enough frames processed.";
  }
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::geometricVerificationCam2d2d(
    const FrameId& ref_id,
    const FrameId& cur_id,
    const KeypointMatches& matches_match_query,
    gtsam::Pose3* camMatch_T_camQuery_2d) {
  CHECK_NOTNULL(camMatch_T_camQuery_2d);

  // std::vector<int> inliers;
  // TrackingStatusPose result = tracker_.geometricOutlierRejection2d2d(
  //     db_frames_[ref_id].bearing_vectors_,
  //     db_frames_[cur_id].bearing_vectors_,
  //     matches_match_query,
  //     &inliers);

  // *camMatch_T_camQuery_2d = result.second;

  // if (result.first == TrackingStatus::VALID) return true;
  // return false;






  // OLD VERSION:
  // Correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  for (const auto& match : matches_match_query) {
    i_match.push_back(match.first);
    i_query.push_back(match.second);
  }

  BearingVectors query_versors, match_versors;
  CHECK_EQ(i_query.size(), i_match.size());
  query_versors.resize(i_query.size());
  match_versors.resize(i_match.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    query_versors[i] = (db_frames_[cur_id].bearing_vectors_[i_query[i]]);
    match_versors[i] = (db_frames_[ref_id].bearing_vectors_[i_match[i]]);
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if (static_cast<int>(match_versors.size()) >= 0) {
    opengv::relative_pose::CentralRelativeAdapter adapter(match_versors,
                                                          query_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    opengv::sac::Ransac<
        opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
        ransac;

    ransac.sac_model_ = std::make_shared<
        opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>(
        adapter,
        opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::
            Algorithm::NISTER,
        lcd_params_.tracker_params_.ransac_randomize_);
    ransac.max_iterations_ =
    lcd_params_.tracker_params_.ransac_max_iterations_; ransac.probability_ =
    lcd_params_.tracker_params_.ransac_probability_; ransac.threshold_ =
    lcd_params_.tracker_params_.ransac_threshold_mono_;

    // Compute transformation via RANSAC.
    bool ransac_success = ransac.computeModel();
    LOG(INFO) << "ransac 5pt size of input: " << query_versors.size();
    LOG(INFO) << "ransac 5pt inliers: " << ransac.inliers_.size();
    LOG(INFO) << "ransac 5pt iterations: " << ransac.iterations_;
    debug_info_.mono_input_size_ = query_versors.size();
    debug_info_.mono_inliers_ = ransac.inliers_.size();
    debug_info_.mono_iter_ = ransac.iterations_;

    if (!ransac_success) {
      VLOG(3) << "LoopClosureDetector Failure: RANSAC 5pt could not solve.";
    } else {
      double inlier_percentage =
          static_cast<double>(ransac.inliers_.size()) / query_versors.size();

      if (inlier_percentage >=
          lcd_params_.tracker_params_.ransac_threshold_mono_) {
        if (ransac.iterations_ <
            lcd_params_.tracker_params_.ransac_max_iterations_) {
          opengv::transformation_t transformation =
          ransac.model_coefficients_; *camMatch_T_camQuery_2d =
              UtilsOpenCV::openGvTfToGtsamPose3(transformation);

          return true;
        }
      }
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPoseBody(
    const FrameId& ref_id,
    const FrameId& cur_id,
    const gtsam::Pose3& camMatch_T_camQuery_2d,
    const KeypointMatches& matches_match_query,
    gtsam::Pose3* bodyMatch_T_bodyQuery_3d) {
  CHECK_NOTNULL(bodyMatch_T_bodyQuery_3d);

  gtsam::Pose3 camMatch_T_camQuery_3d;
  std::vector<int> inliers;

  bool success = false;
  if (lcd_params_.use_pnp_pose_recovery_) {
    // 3D points from backend stored in world reference frame. The transform
    // between world and cam is given by the VIO estimate for the given kf.
    gtsam::Pose3 W_T_camQuery;
    success = tracker_.pnp(db_frames_[cur_id].bearing_vectors_,
                           db_frames_[cur_id].keypoints_3d_,
                           camMatch_T_camQuery_2d.rotation(),
                           camMatch_T_camQuery_2d.translation(),
                           &W_T_camQuery,
                           &inliers);

    // TODO(marcus): invert this process
    // We have to convert the pose to the local frame and get the relative
    // between the query and match.
    const gtsam::Pose3& W_T_Bmatch = W_Pose_Blkf_estimates_.at(ref_id);
    camMatch_T_camQuery_3d =
        (W_T_Bmatch * B_Pose_camLrect_).inverse() * W_T_camQuery;
  } else {
    TrackingStatusPose result;
    if (tracker_.tracker_params_.ransac_use_1point_stereo_) {
      std::pair<TrackingStatusPose, gtsam::Matrix3> result_full =
          tracker_.geometricOutlierRejection3d3dGivenRotation(
              db_frames_[ref_id].left_keypoints_rectified_,
              db_frames_[ref_id].right_keypoints_rectified_,
              db_frames_[cur_id].left_keypoints_rectified_,
              db_frames_[cur_id].right_keypoints_rectified_,
              db_frames_[ref_id].keypoints_3d_,
              db_frames_[cur_id].keypoints_3d_,
              stereo_camera_,
              matches_match_query,
              camMatch_T_camQuery_2d.rotation(),
              &inliers);
      result = result_full.first;
      camMatch_T_camQuery_3d = result.second;
    } else {
      result = tracker_.geometricOutlierRejection3d3d(
          db_frames_[ref_id].keypoints_3d_,
          db_frames_[cur_id].keypoints_3d_,
          matches_match_query,
          &inliers);
      camMatch_T_camQuery_3d = result.second;
    }
    if (result.first == TrackingStatus::VALID) success = true;
  }
  if (lcd_params_.refine_pose_) {
    camMatch_T_camQuery_3d = refinePoses(
        ref_id, cur_id, camMatch_T_camQuery_3d, matches_match_query);
  }

  transformCameraPoseToBodyPose(camMatch_T_camQuery_3d,
                                bodyMatch_T_bodyQuery_3d);
  return success;
}

gtsam::Pose3 LoopClosureDetector::refinePoses(
    const FrameId ref_id,
    const FrameId cur_id,
    const gtsam::Pose3& camMatch_T_camQuery_3d,
    const KeypointMatches& matches_match_query) {
  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values values;

  // TODO camMatch_T_camQuery rename to camMatch_T_camQuery
  gtsam::Key key_match = gtsam::Symbol('x', ref_id);
  gtsam::Key key_query = gtsam::Symbol('x', cur_id);
  values.insert(key_match, gtsam::Pose3::identity());
  values.insert(key_query, camMatch_T_camQuery_3d);

  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(6);
  nfg.add(gtsam::PriorFactor<gtsam::Pose3>(
      key_match, gtsam::Pose3::identity(), noise));

  gtsam::SharedNoiseModel noise_stereo = gtsam::noiseModel::Unit::Create(3);

  gtsam::SmartStereoProjectionParams smart_factors_params;
  smart_factors_params =
      SmartFactorParams(gtsam::HESSIAN,             // JACOBIAN_SVD
                        gtsam::ZERO_ON_DEGENERACY,  // IGNORE_DEGENERACY
                        false,                      // ThrowCherality = false
                        true);                      // verboseCherality = true
  smart_factors_params.setRankTolerance(1);
  smart_factors_params.setLandmarkDistanceThreshold(10);
  smart_factors_params.setRetriangulationThreshold(0.001);
  smart_factors_params.setDynamicOutlierRejectionThreshold(3);

  for (size_t i = 0; i < matches_match_query.size(); i++) {
    KeypointCV undistorted_rectified_left_query_keypoint =
        (db_frames_[cur_id]
             .left_keypoints_rectified_.at(matches_match_query[i].first)
             .second);
    KeypointCV undistorted_rectified_right_query_keypoint =
        (db_frames_[cur_id]
             .right_keypoints_rectified_.at(matches_match_query[i].first)
             .second);

    gtsam::StereoPoint2 sp_query_i(undistorted_rectified_left_query_keypoint.x,
                                   undistorted_rectified_right_query_keypoint.x,
                                   undistorted_rectified_left_query_keypoint.y);

    SmartStereoFactor stereo_factor_i(noise_stereo, smart_factors_params);

    stereo_factor_i.add(
        sp_query_i, key_query, stereo_camera_->getStereoCalib());

    KeypointCV undistorted_rectified_left_match_keypoint =
        (db_frames_[ref_id]
             .left_keypoints_rectified_.at(matches_match_query[i].second)
             .second);
    KeypointCV undistorted_rectified_right_match_keypoint =
        (db_frames_[ref_id]
             .right_keypoints_rectified_.at(matches_match_query[i].second)
             .second);

    gtsam::StereoPoint2 sp_match_i(undistorted_rectified_left_match_keypoint.x,
                                   undistorted_rectified_right_match_keypoint.x,
                                   undistorted_rectified_left_match_keypoint.y);

    stereo_factor_i.add(
        sp_match_i, key_match, stereo_camera_->getStereoCalib());

    nfg.add(stereo_factor_i);
  }

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("ERROR");
  try {
    values = gtsam::LevenbergMarquardtOptimizer(nfg, values, params).optimize();
  } catch (const gtsam::CheiralityException& e) {
    LOG(ERROR) << e.what();
  } catch (const gtsam::StereoCheiralityException& e) {
    LOG(ERROR) << e.what();
  } catch (const gtsam::OutOfRangeThreadsafe& e) {
    LOG(ERROR) << e.what();
  } catch (const std::out_of_range& e) {
    LOG(ERROR) << e.what();
  } catch (const std::exception& e) {
    // Catch anything thrown within try block that derives from
    // std::exception.
    LOG(ERROR) << e.what();
  } catch (...) {
    // Catch the rest of exceptions.
    LOG(ERROR) << "Unrecognized exception.";
  }

  return values.at<gtsam::Pose3>(key_query);
}

/* ------------------------------------------------------------------------ */
const gtsam::Pose3 LoopClosureDetector::getWPoseMap() const {
  if (W_Pose_Blkf_estimates_.size() > 1) {
    CHECK(pgo_);
    const gtsam::Pose3& w_Pose_Bkf_estim = W_Pose_Blkf_estimates_.back();
    const gtsam::Pose3& w_Pose_Bkf_optimal =
        pgo_->calculateEstimate().at<gtsam::Pose3>(
            W_Pose_Blkf_estimates_.size() - 1);

    return w_Pose_Bkf_optimal.between(w_Pose_Bkf_estim);
  }

  return gtsam::Pose3::identity();
}

/* ------------------------------------------------------------------------ */
const gtsam::Values LoopClosureDetector::getPGOTrajectory() const {
  CHECK(pgo_);
  return pgo_->calculateEstimate();
}

/* ------------------------------------------------------------------------ */
const gtsam::NonlinearFactorGraph LoopClosureDetector::getPGOnfg() const {
  CHECK(pgo_);
  return pgo_->getFactorsUnsafe();
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::setDatabase(const OrbDatabase& db) {
  db_BoW_ = VIO::make_unique<OrbDatabase>(db);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::setVocabulary(const OrbVocabulary& voc) {
  db_BoW_->setVocabulary(voc);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::print() const {
  // TODO(marcus): implement
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::rewriteStereoFrameFeatures(
    const KeypointsCV& keypoints,
    StereoFrame* stereo_frame) const {
  CHECK_NOTNULL(stereo_frame);

  // Populate frame keypoints with ORB features instead of the normal
  // VIO features that came with the StereoFrame.
  Frame* left_frame_mutable = &stereo_frame->left_frame_;
  Frame* right_frame_mutable = &stereo_frame->right_frame_;
  CHECK_NOTNULL(left_frame_mutable);
  CHECK_NOTNULL(right_frame_mutable);

  // Clear all relevant fields.
  left_frame_mutable->keypoints_.clear();
  left_frame_mutable->versors_.clear();
  left_frame_mutable->scores_.clear();
  right_frame_mutable->keypoints_.clear();
  right_frame_mutable->versors_.clear();
  right_frame_mutable->scores_.clear();
  stereo_frame->keypoints_depth_.clear();
  stereo_frame->keypoints_3d_.clear();
  stereo_frame->left_keypoints_rectified_.clear();
  stereo_frame->right_keypoints_rectified_.clear();

  // Reserve space in all relevant fields
  left_frame_mutable->keypoints_.reserve(keypoints.size());
  left_frame_mutable->versors_.reserve(keypoints.size());
  left_frame_mutable->scores_.reserve(keypoints.size());
  right_frame_mutable->keypoints_.reserve(keypoints.size());
  right_frame_mutable->versors_.reserve(keypoints.size());
  right_frame_mutable->scores_.reserve(keypoints.size());
  stereo_frame->keypoints_depth_.reserve(keypoints.size());
  stereo_frame->keypoints_3d_.reserve(keypoints.size());
  stereo_frame->left_keypoints_rectified_.reserve(keypoints.size());
  stereo_frame->right_keypoints_rectified_.reserve(keypoints.size());

  stereo_frame->setIsRectified(false);

  // Add ORB keypoints
  for (const KeypointCV& keypoint : keypoints) {
    left_frame_mutable->keypoints_.push_back(keypoint);
    left_frame_mutable->versors_.push_back(
        UndistorterRectifier::UndistortKeypointAndGetVersor(
            keypoint, left_frame_mutable->cam_param_));
    left_frame_mutable->scores_.push_back(1.0);
  }

  // Automatically match keypoints in right image with those in left.
  stereo_matcher_->sparseStereoReconstruction(stereo_frame);
  stereo_frame->checkStereoFrame();
}

/* ------------------------------------------------------------------------ */
cv::Mat LoopClosureDetector::computeAndDrawMatchesBetweenFrames(
    const cv::Mat& query_img,
    const cv::Mat& match_img,
    const FrameId& query_id,
    const FrameId& match_id,
    bool cut_matches) const {
  std::vector<DMatchVec> matches;
  DMatchVec good_matches;

  // Use the Lowe's Ratio Test only if asked.
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  // TODO(marcus): this can use computeDescriptorMatches() as well...
  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
                                 db_frames_[match_id].descriptors_mat_,
                                 matches,
                                 2u);

  for (const DMatchVec& match : matches) {
    if (match.at(0).distance < lowe_ratio * match.at(1).distance) {
      good_matches.push_back(match[0]);
    }
  }

  // Draw matches.
  float keypoint_diameter = 2;
  std::vector<cv::KeyPoint> keypoints_query, keypoints_match;
  cv::KeyPoint::convert(
      db_frames_.at(query_id).keypoints_, keypoints_query, keypoint_diameter);
  cv::KeyPoint::convert(
      db_frames_.at(match_id).keypoints_, keypoints_match, keypoint_diameter);
  cv::Mat img_matches;
  cv::drawMatches(query_img,
                  keypoints_query,
                  match_img,
                  keypoints_match,
                  good_matches,
                  img_matches,
                  cv::Scalar(255, 0, 0),
                  cv::Scalar(0, 255, 0));

  return img_matches;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::transformCameraPoseToBodyPose(
    const gtsam::Pose3& camMatch_T_camQuery,
    gtsam::Pose3* bodyMatch_T_bodyQuery) const {
  CHECK_NOTNULL(bodyMatch_T_bodyQuery);
  *bodyMatch_T_bodyQuery =
      B_Pose_camLrect_ * camMatch_T_camQuery * B_Pose_camLrect_.inverse();
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::transformBodyPoseToCameraPose(
    const gtsam::Pose3& bodyMatch_T_bodyQuery,
    gtsam::Pose3* camMatch_T_camQuery) const {
  CHECK_NOTNULL(camMatch_T_camQuery);
  *camMatch_T_camQuery =
      B_Pose_camLrect_.inverse() * bodyMatch_T_bodyQuery * B_Pose_camLrect_;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::computeDescriptorMatches(
    const OrbDescriptor& ref_descriptors,
    const OrbDescriptor& cur_descriptors,
    KeypointMatches* matches_match_query,
    bool cut_matches) const {
  CHECK_NOTNULL(matches_match_query);
  matches_match_query->clear();

  // Get two best matches between frame descriptors.
  std::vector<DMatchVec> matches;
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  orb_feature_matcher_->knnMatch(cur_descriptors, ref_descriptors, matches, 2u);

  const size_t& n_matches = matches.size();
  for (size_t i = 0; i < n_matches; i++) {
    const DMatchVec& match = matches[i];
    if (match.size() < 2) continue;
    if (match[0].distance < lowe_ratio * match[1].distance) {
      // Store trainIdx first because this represents the kpt from the
      // ref frame. For LCD, this would be the match and not the query.
      // For tracker outlier-rejection we use (ref, cur) always.
      matches_match_query->push_back(
          std::make_pair(match[0].trainIdx, match[0].queryIdx));
    }
  }
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::initializePGO(const OdometryFactor& factor) {
  CHECK_EQ(factor.cur_key_, 0u);
  CHECK(lcd_state_ == LcdState::Bootstrap);

  // This push_back should be out above (and remove the one inside
  // addOdometryFactorAndOptimize. Not doing it now bcs code assumes that
  // addOdometryFactorAndOptimize does that...
  W_Pose_Blkf_estimates_.push_back(factor.W_Pose_Blkf_);

  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;

  init_val.insert(gtsam::Symbol(factor.cur_key_), factor.W_Pose_Blkf_);

  init_nfg.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol(factor.cur_key_), factor.W_Pose_Blkf_, factor.noise_));

  CHECK(pgo_);
  pgo_->update(init_nfg, init_val);

  lcd_state_ = LcdState::Nominal;
}

/* ------------------------------------------------------------------------ */
// TODO(marcus): only add nodes if they're x dist away from previous node
// TODO(marcus): consider making the keys of OdometryFactor minus one each so
// that the extra check in here isn't needed...
void LoopClosureDetector::addOdometryFactorAndOptimize(
    const OdometryFactor& factor) {
  CHECK(lcd_state_ == LcdState::Nominal);

  W_Pose_Blkf_estimates_.push_back(factor.W_Pose_Blkf_);

  CHECK_LE(factor.cur_key_, W_Pose_Blkf_estimates_.size())
      << "New odometry factor has a key that is too high.";

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values value;

  CHECK_GT(factor.cur_key_, 0u);
  const gtsam::Values& optimized_values = pgo_->calculateEstimate();
  CHECK_EQ(factor.cur_key_, optimized_values.size());
  const gtsam::Pose3& estimated_last_pose =
      optimized_values.at<gtsam::Pose3>(factor.cur_key_ - 1);

  const gtsam::Pose3& B_llkf_Pose_lkf =
      W_Pose_Blkf_estimates_.at(factor.cur_key_ - 1)
          .between(factor.W_Pose_Blkf_);
  value.insert(gtsam::Symbol(factor.cur_key_),
               estimated_last_pose.compose(B_llkf_Pose_lkf));

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.cur_key_ - 1),
                                             gtsam::Symbol(factor.cur_key_),
                                             B_llkf_Pose_lkf,
                                             factor.noise_));

  CHECK(pgo_);
  pgo_->update(nfg, value);
}
/* ------------------------------------------------------------------------ */
void LoopClosureDetector::addLoopClosureFactorAndOptimize(
    const LoopClosureFactor& factor) {
  gtsam::NonlinearFactorGraph nfg;

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.ref_key_),
                                             gtsam::Symbol(factor.cur_key_),
                                             factor.ref_Pose_cur_,
                                             factor.noise_));

  CHECK(pgo_);
  pgo_->update(nfg);
}

}  // namespace VIO
