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
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <string>
#include <vector>

#include "kimera-vio/frontend/UndistorterRectifier.h"
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
  // TODO(marcus): This should come in with every input payload, not be
  // constant.
  Vector6 precisions;
  precisions.head<3>().setConstant(lcd_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(lcd_params_.betweenTranslationPrecision_);
  shared_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  // Set camera intrinsics for LCD
  CameraParams cam_param = stereo_camera->getLeftCamParams();
  lcd_params_.image_width_ = cam_param.image_size_.width;
  lcd_params_.image_height_ = cam_param.image_size_.height;
  lcd_params_.focal_length_ = cam_param.intrinsics_[0];
  lcd_params_.principle_point_ =
      cv::Point2d(cam_param.intrinsics_[2], cam_param.intrinsics_[3]);

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
      initializePGO(odom_factor);
      // This should be out above (and remove the one inside
      // addOdometryFactorAndOptimize. Not doing it now bcs code assumes that
      // addOdometryFactorAndOptimize does that...
      W_Pose_Blkf_estimates_.push_back(odom_factor.W_Pose_Blkf_);
      break;
    }
    case LcdState::Nominal: {
      // TODO(marcus): need a better check than this:
      CHECK_GT(pgo_->calculateEstimate().size(), 0);
      addOdometryFactorAndOptimize(odom_factor);
      break;
    }
    default: { LOG(FATAL) << "Unrecognized LCD state."; }
  }

  // Process the StereoFrame and check for a loop closure with previous ones.
  LoopResult loop_result;
  if (input.frontend_output_->frontend_type_ == FrontendType::kStereoImu) {
    StereoFrontendOutput::Ptr stereo_frontend_output =
        VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
            input.frontend_output_);
    // Try to find a loop and update the PGO with the result if available.
    if (detectLoop(stereo_frontend_output->stereo_frame_lkf_, &loop_result)) {
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
  } else {
    LOG(ERROR) << "LoopClosureDetector: Not using StereoFrontend! Change frontend.";
  }

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

/* ------------------------------------------------------------------------ */
FrameId LoopClosureDetector::processAndAddFrame(
    const StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  OrbDescriptor descriptors_mat;
  OrbDescriptorVec descriptors_vec;

  // Extract ORB features and construct descriptors_vec.
  orb_feature_detector_->detectAndCompute(
      stereo_frame.left_frame_.img_, cv::Mat(), keypoints, descriptors_mat);

  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec.resize(descriptors_mat.size().height);

  for (size_t i = 0; i < descriptors_vec.size(); i++) {
    descriptors_vec[i] = cv::Mat(1, L, descriptors_mat.type());  // one row only
    descriptors_mat.row(i).copyTo(descriptors_vec[i].row(0));
  }

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  StereoFrame cp_stereo_frame(stereo_frame);
  rewriteStereoFrameFeatures(keypoints, &cp_stereo_frame);

  // Build and store LCDFrame object.
  db_frames_.push_back(LCDFrame(cp_stereo_frame.timestamp_,
                                db_frames_.size(),
                                cp_stereo_frame.id_,
                                keypoints,
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
bool LoopClosureDetector::detectLoop(const StereoFrame& stereo_frame,
                                     LoopResult* result) {
  CHECK_NOTNULL(result);

  FrameId frame_id = processAndAddFrame(stereo_frame);
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
            gtsam::Pose3 camMatch_T_camQuery_mono;

            // Find correspondences between keypoints.
            std::vector<FrameId> i_query, i_match;
            computeMatchedIndices(
                result->query_id_, result->match_id_, &i_query, &i_match, true);

            bool pass_geometric_verification =
                geometricVerificationCheck(result->query_id_,
                                           result->match_id_,
                                           &camMatch_T_camQuery_mono,
                                           &i_query,
                                           &i_match);

            if (!pass_geometric_verification) {
              result->status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
            } else {
              gtsam::Pose3 bodyMatch_T_bodyQuery_stereo;
              bool pass_3d_pose_compute =
                  recoverPose(result->query_id_,
                              result->match_id_,
                              camMatch_T_camQuery_mono,
                              &bodyMatch_T_bodyQuery_stereo,
                              &i_query,
                              &i_match);

              if (!pass_3d_pose_compute) {
                result->status_ = LCDStatus::FAILED_POSE_RECOVERY;
              } else {
                result->relative_pose_ = bodyMatch_T_bodyQuery_stereo;
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

  return result->isLoop();
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::geometricVerificationCheck(
    const FrameId& query_id,
    const FrameId& match_id,
    gtsam::Pose3* camMatch_T_camQuery_mono,
    std::vector<FrameId>* inlier_id_in_query_frame,
    std::vector<FrameId>* inlier_id_in_match_frame) {
  CHECK_NOTNULL(camMatch_T_camQuery_mono);
  switch (lcd_params_.geom_check_) {
    case GeomVerifOption::NISTER: {
      return geometricVerificationNister(query_id,
                                         match_id,
                                         camMatch_T_camQuery_mono,
                                         inlier_id_in_query_frame,
                                         inlier_id_in_match_frame);
    }
    case GeomVerifOption::NONE: {
      return true;
    }
    default: {
      LOG(FATAL) << "LoopClosureDetector: Incorrect geom_check_ option: "
                 << std::to_string(static_cast<int>(lcd_params_.geom_check_));
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPose(
    const FrameId& query_id,
    const FrameId& match_id,
    const gtsam::Pose3& camMatch_T_camQuery_mono,
    gtsam::Pose3* bodyMatch_T_bodyQuery_stereo,
    std::vector<FrameId>* inlier_id_in_query_frame,
    std::vector<FrameId>* inlier_id_in_match_frame) {
  CHECK_NOTNULL(bodyMatch_T_bodyQuery_stereo);
  CHECK_NOTNULL(inlier_id_in_query_frame);
  CHECK_NOTNULL(inlier_id_in_match_frame);

  bool passed_pose_recovery = false;
  gtsam::Pose3 camMatch_T_camQuery_stereo;
  switch (lcd_params_.pose_recovery_option_) {
    case PoseRecoveryOption::RANSAC_ARUN: {
      passed_pose_recovery = recoverPoseArun(query_id,
                                             match_id,
                                             &camMatch_T_camQuery_stereo,
                                             inlier_id_in_query_frame,
                                             inlier_id_in_match_frame);
      break;
    }
    case PoseRecoveryOption::GIVEN_ROT: {
      passed_pose_recovery = recoverPoseGivenRot(query_id,
                                                 match_id,
                                                 camMatch_T_camQuery_mono,
                                                 &camMatch_T_camQuery_stereo,
                                                 inlier_id_in_query_frame,
                                                 inlier_id_in_match_frame);
      break;
    }
    default: {
      LOG(FATAL) << "LoopClosureDetector: Incorrect pose recovery option: "
                 << std::to_string(
                        static_cast<int>(lcd_params_.pose_recovery_option_));
    }
  }
  if (lcd_params_.refine_pose_) {
    camMatch_T_camQuery_stereo = refinePoses(query_id,
                                             match_id,
                                             camMatch_T_camQuery_stereo,
                                             *inlier_id_in_query_frame,
                                             *inlier_id_in_match_frame);
  }

  transformCameraPoseToBodyPose(camMatch_T_camQuery_stereo,
                                bodyMatch_T_bodyQuery_stereo);

  // Use the rotation obtained from 5pt method if needed.
  // TODO(marcus): check that the rotations are close to each other!
  if (lcd_params_.use_mono_rot_ &&
      lcd_params_.pose_recovery_option_ != PoseRecoveryOption::GIVEN_ROT) {
    gtsam::Pose3 bodyMatch_T_bodyQuery_mono;
    transformCameraPoseToBodyPose(camMatch_T_camQuery_mono,
                                  &bodyMatch_T_bodyQuery_mono);

    const gtsam::Rot3& bodyMatch_R_bodyQuery_stereo =
        bodyMatch_T_bodyQuery_mono.rotation();
    const gtsam::Point3& bodyMatch_t_bodyQuery_stereo =
        bodyMatch_T_bodyQuery_stereo->translation();

    *bodyMatch_T_bodyQuery_stereo = gtsam::Pose3(bodyMatch_R_bodyQuery_stereo,
                                                 bodyMatch_t_bodyQuery_stereo);
  }

  return passed_pose_recovery;
}

gtsam::Pose3 LoopClosureDetector::refinePoses(
    const FrameId query_id,
    const FrameId match_id,
    const gtsam::Pose3& camMatch_T_camQuery_stereo,
    const std::vector<FrameId>& inlier_id_in_query_frame,
    const std::vector<FrameId>& inlier_id_in_match_frame) {
  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values values;

  // TODO camMatch_T_camQuery rename to camMatch_T_camQuery
  gtsam::Key key_match = gtsam::Symbol('x', match_id);
  gtsam::Key key_query = gtsam::Symbol('x', query_id);
  values.insert(key_match, gtsam::Pose3::identity());
  values.insert(key_query, camMatch_T_camQuery_stereo);

  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(6);
  nfg.add(gtsam::PriorFactor<gtsam::Pose3>(key_match, gtsam::Pose3::identity(), noise));

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

  for (size_t i = 0; i < inlier_id_in_query_frame.size(); i++) {
    KeypointCV undistorted_rectified_left_query_keypoint =
        (db_frames_[query_id]
             .left_keypoints_rectified_.at(inlier_id_in_query_frame[i]).second);
    KeypointCV undistorted_rectified_right_query_keypoint =
        (db_frames_[query_id]
             .right_keypoints_rectified_.at(inlier_id_in_query_frame[i]).second);

    gtsam::StereoPoint2 sp_query_i(undistorted_rectified_left_query_keypoint.x,
                                   undistorted_rectified_right_query_keypoint.x,
                                   undistorted_rectified_left_query_keypoint.y);

    SmartStereoFactor stereo_factor_i(noise_stereo, smart_factors_params);

    stereo_factor_i.add(sp_query_i, key_query, stereo_camera_->getStereoCalib());

    KeypointCV undistorted_rectified_left_match_keypoint =
        (db_frames_[match_id]
             .left_keypoints_rectified_.at(inlier_id_in_match_frame[i]).second);
    KeypointCV undistorted_rectified_right_match_keypoint =
        (db_frames_[match_id]
             .right_keypoints_rectified_.at(inlier_id_in_match_frame[i]).second);

    gtsam::StereoPoint2 sp_match_i(undistorted_rectified_left_match_keypoint.x,
                                   undistorted_rectified_right_match_keypoint.x,
                                   undistorted_rectified_left_match_keypoint.y);

    stereo_factor_i.add(sp_match_i, key_match, stereo_camera_->getStereoCalib());

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
    const std::vector<cv::KeyPoint>& keypoints,
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

  // stereo_frame->setIsRectified(false);

  // Add ORB keypoints.
  for (const cv::KeyPoint& keypoint : keypoints) {
    left_frame_mutable->keypoints_.push_back(keypoint.pt);
    left_frame_mutable->versors_.push_back(
        UndistorterRectifier::UndistortKeypointAndGetVersor(keypoint.pt, left_frame_mutable->cam_param_));
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

  // TODO(marcus): this can use computeMatchedIndices() as well...
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
  cv::Mat img_matches;
  cv::drawMatches(query_img,
                  db_frames_.at(query_id).keypoints_,
                  match_img,
                  db_frames_.at(match_id).keypoints_,
                  good_matches,
                  img_matches,
                  cv::Scalar(255, 0, 0),
                  cv::Scalar(255, 0, 0));

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
void LoopClosureDetector::computeMatchedIndices(const FrameId& query_id,
                                                const FrameId& match_id,
                                                std::vector<FrameId>* i_query,
                                                std::vector<FrameId>* i_match,
                                                bool cut_matches) const {
  CHECK_NOTNULL(i_query);
  CHECK_NOTNULL(i_match);
  i_query->clear();
  i_match->clear();

  // Get two best matches between frame descriptors.
  std::vector<DMatchVec> matches;
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
                                 db_frames_[match_id].descriptors_mat_,
                                 matches,
                                 2u);

  // We reserve instead of resize because some of the matches will be pruned.
  const size_t& n_matches = matches.size();
  for (size_t i = 0; i < n_matches; i++) {
    const DMatchVec& match = matches[i];
    if (match.size() < 2) continue;
    if (match[0].distance < lowe_ratio * match[1].distance) {
      i_query->push_back(match[0].queryIdx);
      i_match->push_back(match[0].trainIdx);
    }
  }
}

/* ------------------------------------------------------------------------ */
// TODO(marcus): both geometrticVerification and recoverPose run the matching
// alg. this is wasteful. Store the matched indices as latest for use in the
// compute step
bool LoopClosureDetector::geometricVerificationNister(
    const FrameId& query_id,
    const FrameId& match_id,
    gtsam::Pose3* camMatch_T_camQuery_mono,
    std::vector<FrameId>* inlier_id_in_query_frame,
    std::vector<FrameId>* inlier_id_in_match_frame) {
  CHECK_NOTNULL(camMatch_T_camQuery_mono);
  CHECK_NOTNULL(inlier_id_in_query_frame);
  CHECK_NOTNULL(inlier_id_in_match_frame);

  // Correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  // Use mono ransac inliers as correspondences instead of re-matching
  i_query = *inlier_id_in_query_frame;
  i_match = *inlier_id_in_match_frame;

  BearingVectors query_versors, match_versors;

  CHECK_EQ(i_query.size(), i_match.size());
  query_versors.resize(i_query.size());
  match_versors.resize(i_match.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    query_versors[i] = (db_frames_[query_id].versors_[i_query[i]]);
    match_versors[i] = (db_frames_[match_id].versors_[i_match[i]]);
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if (static_cast<int>(match_versors.size()) >=
      lcd_params_.min_correspondences_) {
    AdapterMono adapter(match_versors, query_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    opengv::sac::Ransac<SacProblemMono> ransac;

    ransac.sac_model_ =
        std::make_shared<SacProblemMono>(adapter,
                                         SacProblemMono::Algorithm::NISTER,
                                         lcd_params_.ransac_randomize_mono_);
    ransac.max_iterations_ = lcd_params_.max_ransac_iterations_mono_;
    ransac.probability_ = lcd_params_.ransac_probability_mono_;
    ransac.threshold_ = lcd_params_.ransac_threshold_mono_;

    // Compute transformation via RANSAC.
    bool ransac_success = ransac.computeModel();
    VLOG(3) << "ransac 5pt size of input: " << query_versors.size()
            << "\nransac 5pt inliers: " << ransac.inliers_.size()
            << "\nransac 5pt iterations: " << ransac.iterations_;
    debug_info_.mono_input_size_ = query_versors.size();
    debug_info_.mono_inliers_ = ransac.inliers_.size();
    debug_info_.mono_iter_ = ransac.iterations_;

    if (!ransac_success) {
      VLOG(3) << "LoopClosureDetector Failure: RANSAC 5pt could not solve.";
    } else {
      double inlier_percentage =
          static_cast<double>(ransac.inliers_.size()) / query_versors.size();

      if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_mono_) {
        if (ransac.iterations_ < lcd_params_.max_ransac_iterations_mono_) {
          opengv::transformation_t transformation = ransac.model_coefficients_;
          // Remove the outliers based on ransac.inliers (modify i_query and
          // i_match AND pass the result out)
          inlier_id_in_query_frame->clear();
          inlier_id_in_match_frame->clear();
          for (const auto i : ransac.inliers_) {
            inlier_id_in_query_frame->push_back(i_query[i]);
            inlier_id_in_match_frame->push_back(i_match[i]);
          }
          *camMatch_T_camQuery_mono =
              UtilsOpenCV::openGvTfToGtsamPose3(transformation);

          return true;
        }
      }
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPoseArun(
    const FrameId& query_id,
    const FrameId& match_id,
    gtsam::Pose3* camMatch_T_camQuery,
    std::vector<FrameId>* inlier_id_in_query_frame,
    std::vector<FrameId>* inlier_id_in_match_frame) {
  CHECK_NOTNULL(camMatch_T_camQuery);
  CHECK_NOTNULL(inlier_id_in_query_frame);
  CHECK_NOTNULL(inlier_id_in_match_frame);

  // Correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  // Use mono ransac inliers as correspondences instead of re-matching
  i_query = *inlier_id_in_query_frame;
  i_match = *inlier_id_in_match_frame;

  BearingVectors f_match, f_query;

  // Fill point clouds with matched 3D keypoints.
  CHECK_EQ(i_query.size(), i_match.size());
  f_match.resize(i_match.size());
  f_query.resize(i_query.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    f_query[i] = (db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_match[i] = (db_frames_[match_id].keypoints_3d_.at(i_match[i]));
  }

  AdapterStereo adapter(f_match, f_query);
  opengv::transformation_t transformation;

  // Compute transform using RANSAC 3-point method (Arun).
  opengv::sac::Ransac<SacProblemStereo> ransac;
  ransac.sac_model_ = std::make_shared<SacProblemStereo>(
      adapter, lcd_params_.ransac_randomize_stereo_);
  ransac.max_iterations_ = lcd_params_.max_ransac_iterations_stereo_;
  ransac.probability_ = lcd_params_.ransac_probability_stereo_;
  ransac.threshold_ = lcd_params_.ransac_threshold_stereo_;

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();
  VLOG(3) << "ransac 3pt size of input: " << f_match.size()
          << "\nransac 3pt inliers: " << ransac.inliers_.size()
          << "\nransac 3pt iterations: " << ransac.iterations_;
  debug_info_.stereo_input_size_ = f_match.size();
  debug_info_.stereo_inliers_ = ransac.inliers_.size();
  debug_info_.stereo_iter_ = ransac.iterations_;

  if (!ransac_success) {
    VLOG(3) << "LoopClosureDetector Failure: RANSAC 3pt could not solve.";
  } else {
    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / f_match.size();

    if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_stereo_) {
      if (ransac.iterations_ < lcd_params_.max_ransac_iterations_stereo_) {
        transformation = ransac.model_coefficients_;

        // Remove the outliers based on ransac.inliers (modify i_query and
        // i_match AND pass the result out)
        inlier_id_in_query_frame->clear();
        inlier_id_in_match_frame->clear();
        for (const auto i : ransac.inliers_) {
          inlier_id_in_query_frame->push_back(i_query[i]);
          inlier_id_in_match_frame->push_back(i_match[i]);
        }

        // Transform pose from camera frame to body frame.
        *camMatch_T_camQuery =
            UtilsOpenCV::openGvTfToGtsamPose3(transformation);

        return true;
      }
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPoseGivenRot(
    const FrameId& query_id,
    const FrameId& match_id,
    const gtsam::Pose3& camMatch_T_camQuery_mono,
    gtsam::Pose3* camMatch_T_camQuery,
    std::vector<FrameId>* inlier_id_in_query_frame,
    std::vector<FrameId>* inlier_id_in_match_frame) {
  CHECK_NOTNULL(camMatch_T_camQuery);
  CHECK_NOTNULL(inlier_id_in_query_frame);
  CHECK_NOTNULL(inlier_id_in_match_frame);

  const gtsam::Rot3& camMatch_R_camQuery = camMatch_T_camQuery_mono.rotation();

  // Correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  // Use mono ransac inliers as correspondences instead of re-matching
  i_query = *inlier_id_in_query_frame;
  i_match = *inlier_id_in_match_frame;

  // Fill point clouds with matched 3D keypoints.
  const size_t& n_matches = i_match.size();
  CHECK_EQ(i_query.size(), n_matches);

  if (n_matches > 0) {
    std::vector<double> x_coord(n_matches);
    std::vector<double> y_coord(n_matches);
    std::vector<double> z_coord(n_matches);

    for (size_t i = 0; i < n_matches; i++) {
      const gtsam::Vector3& keypoint_3d_query =
          db_frames_[query_id].keypoints_3d_.at(i_query[i]);
      const gtsam::Vector3& keypoint_3d_match =
          db_frames_[match_id].keypoints_3d_.at(i_match[i]);

      gtsam::Vector3 rotated_keypoint_diff =
          keypoint_3d_match - (camMatch_R_camQuery * keypoint_3d_query);
      x_coord[i] = rotated_keypoint_diff[0];
      y_coord[i] = rotated_keypoint_diff[1];
      z_coord[i] = rotated_keypoint_diff[2];
    }

    CHECK_EQ(x_coord.size(), n_matches);
    CHECK_EQ(y_coord.size(), n_matches);
    CHECK_EQ(z_coord.size(), n_matches);

    // TODO(marcus): decide between median check and scaling factor
    std::sort(x_coord.begin(), x_coord.end());
    std::sort(y_coord.begin(), y_coord.end());
    std::sort(z_coord.begin(), z_coord.end());

    gtsam::Point3 scaled_t(
        x_coord.at(std::floor(static_cast<int>(x_coord.size() / 2))),
        y_coord.at(std::floor(static_cast<int>(y_coord.size() / 2))),
        z_coord.at(std::floor(static_cast<int>(z_coord.size() / 2))));

    // Transform pose from camera frame to body frame.
    *camMatch_T_camQuery = gtsam::Pose3(camMatch_R_camQuery, scaled_t);

    inlier_id_in_query_frame->clear();
    inlier_id_in_match_frame->clear();
    for (size_t i = 0; i < n_matches; i++) {
      gtsam::Vector3 residual_error_vector =
          gtsam::Vector3(x_coord[i] - scaled_t.x(),
                         y_coord[i] - scaled_t.y(),
                         z_coord[i] - scaled_t.z());
      double residual_error = residual_error_vector.norm();

      if (residual_error <= lcd_params_.ransac_threshold_stereo_) {
        inlier_id_in_query_frame->push_back(i_query[i]);
        inlier_id_in_match_frame->push_back(i_match[i]);
      }
    }
    return true;
  }

  return false;

  // TODO(marcus): input should alwasy be with unit translation, no need to
  // check
  // gtsam::Point3 unit_t = camMatch_T_camQuery_mono.translation() /
  // camMatch_T_camQuery_mono.translation().norm();
  // // Get sacling factor for translation by averaging across point cloud.
  // double scaling_factor = 0.0;
  // for (size_t i=0; i<f_ref.size(); i++) {
  //   gtsam::Vector3 keypoint_ref = f_ref[i];
  //   gtsam::Vector3 keypoint_cur = f_cur[i];
  //
  //   gtsam::Vector3 rotated_keypoint_diff =
  //       keypoint_ref - (R*keypoint_cur);
  //
  //   double cur_scaling_factor = rotated_keypoint_diff.dot(unit_t);
  //   if (cur_scaling_factor < 0) {
  //     cur_scaling_factor *= -1.0;
  //   }
  //   scaling_factor += cur_scaling_factor;
  // }
  // scaling_factor /= f_ref.size();
  //
  // if (scaling_factor < 0) {
  //   scaling_factor *= -1.0;
  // }
  //
  // gtsam::Point3 scaled_t(unit_t[0] * scaling_factor,
  //     unit_t[1] * scaling_factor, unit_t[2] * scaling_factor);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::initializePGO(const OdometryFactor& factor) {
  CHECK_EQ(factor.cur_key_, 0u);
  CHECK(lcd_state_ == LcdState::Bootstrap);

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
